"""Mock-only L2/L3 replay. Never imports a real device adapter.

Run from d50ws_nav: python -B tools/offline_l2l3_replay.py
The JSON trace is written to the reports directory and each expected terminal
state is checked. This is interface evidence, not field safety acceptance.
"""

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import (ACTION_CLASSES, ApproachAction, CrossTrackAction,
                         DetachAction, RetreatAction, VentAction, create_action)
from nav.context import PerceptionBundle, RunContext
from perception.mocks import (MockLeverDetector, MockLidarProcessor,
                              MockPipeDetector, MockSceneSegmenter)
from sdk_adapter.mock import MockRobot
from sdk_adapter.mock_actuators import MockAcoustic, MockManipulator, MockRod


class FaultManipulator(MockManipulator):
    """Synthetic receipt faults; the underlying actuator remains a Mock."""

    def __init__(self, mode):
        super().__init__(exec_seconds=0.02)
        self.mode = mode
        self.first_id = None

    def start_vent(self, mode, lever_pose):
        ack = super().start_vent(mode, lever_pose)
        if self.first_id is None:
            self.first_id = ack["command_id"]
        return ack

    def read(self):
        reply = super().read()
        if self.mode == "wrong":
            reply["command_id"] = "other-command"
        elif self.mode == "late":
            reply["timestamp"] = self._t0 - 1.0
        elif self.mode == "repeat" and self._sequence > 1:
            reply["command_id"] = self.first_id
            reply["state"] = "DONE"
        return reply


SAFE_CHASSIS = {MockRobot}
SAFE_SENSORS = {MockLeverDetector, MockPipeDetector,
                MockSceneSegmenter, MockLidarProcessor}
SAFE_ACTUATORS = {MockManipulator, MockRod, MockAcoustic, FaultManipulator}
TERMINAL = {"SUCCESS", "FAIL", "ABORTED"}


def require_mock_only(ctx):
    """Fail before any action starts if a caller supplies a real adapter."""
    if type(ctx.chassis) not in SAFE_CHASSIS:
        raise TypeError("offline replay requires an exact MockRobot chassis")
    for name in ("manipulator", "rod", "acoustic"):
        item = getattr(ctx, name)
        if item is not None and type(item) not in SAFE_ACTUATORS:
            raise TypeError("offline replay refuses non-Mock " + name)
    if ctx.perception is not None:
        for name in ("lever", "pipe", "scene", "lidar"):
            item = getattr(ctx.perception, name, None)
            if item is not None and type(item) not in SAFE_SENSORS:
                raise TypeError("offline replay refuses non-Mock " + name)


class Trace:
    def __init__(self):
        self.events = []
        self.counts = Counter()
        self.action = None

    def add(self, kind, source, data, limit=None):
        key = (self.action, kind, source)
        self.counts[key] += 1
        if limit is not None and self.counts[key] > limit:
            return
        self.events.append({"action": self.action, "kind": kind,
                            "source": source, "data": data})


class Tap:
    """Record calls to existing Mock instances without changing their behavior."""

    METHODS = {"get_pose", "move_vector", "set_velocity", "stand_up",
               "set_terrain_mode", "set_body_height", "read", "enable",
               "disable", "check_detach", "start_vent", "start_detach",
               "stow", "stop", "check"}

    def __init__(self, obj, name, trace, read_fault=None):
        self.obj, self.name, self.trace = obj, name, trace
        self.read_fault = read_fault

    def __getattr__(self, name):
        attr = getattr(self.obj, name)
        if name not in self.METHODS or not callable(attr):
            return attr

        def call(*args, **kwargs):
            reply = attr(*args, **kwargs)
            if name == "read" and self.read_fault is not None:
                reply = self.read_fault(reply)
            if name == "read":
                kind = "observation" if self.name in ("P1", "P2", "P3", "P4") else "receipt"
                limit = 4 if kind == "observation" else 6
            elif name in ("get_pose",):
                kind, limit = "pose", 2
            elif name in ("move_vector", "set_velocity"):
                kind, limit = "mock_motion_command", 5
            elif name in ("check_detach", "check"):
                kind, limit = "check_observation", None
            elif name in ("start_vent", "start_detach", "stow"):
                kind, limit = "command_or_check", None
            else:
                kind, limit = "mock_control", 4
            payload = {"args": args, "kwargs": kwargs, "reply": reply}
            if kind == "observation" and isinstance(reply, dict):
                payload = {k: reply.get(k) for k in (
                    "frame_count", "age_s", "error_code", "target_detected",
                    "pipe_detected", "car_detected", "car_no_text",
                    "safety_status", "risk_level", "sector_scan") if k in reply}
            if kind == "pose":
                payload = {"e": reply.e, "n": reply.n, "yaw": reply.yaw}
            self.trace.add(kind, self.name + "." + name, payload, limit)
            return reply
        return call


def mock_context(trace, *, manipulator=None, rod=None, acoustic=None,
                 with_lidar=True, stale_lever=False):
    robot = MockRobot(dt=0.05)
    sensors = PerceptionBundle(
        lever=MockLeverDetector(chassis=robot, lever_world=(1.1, 0.0),
                                detect_range=2.0, period=0.02),
        pipe=MockPipeDetector(chassis=robot, pipe_world=(1.2, 0.0),
                              detect_range=5.0, period=0.02),
        scene=MockSceneSegmenter(chassis=robot, car_world=(0.6, 0.0),
                                 detect_range=2.5, period=0.02),
        lidar=MockLidarProcessor(period=0.02) if with_lidar else None)
    raw = RunContext(chassis=robot, perception=sensors,
                     manipulator=manipulator, rod=rod, acoustic=acoustic)
    require_mock_only(raw)
    tapped = PerceptionBundle(
        lever=Tap(sensors.lever, "P1", trace,
                  read_fault=(lambda st: {**st, "age_s": 8.0}) if stale_lever else None),
        pipe=Tap(sensors.pipe, "P2", trace),
        scene=Tap(sensors.scene, "P3", trace),
        lidar=Tap(sensors.lidar, "P4", trace) if sensors.lidar else None)
    ctx = RunContext(chassis=Tap(robot, "chassis", trace), perception=tapped,
                     manipulator=Tap(manipulator, "manipulator", trace) if manipulator else None,
                     rod=Tap(rod, "rod", trace) if rod else None,
                     acoustic=Tap(acoustic, "acoustic", trace) if acoustic else None)
    return ctx, sensors, robot


def run_action(action, trace, *, label, expected, error=None, cancel=False,
               deadline_s=8.0):
    trace.action = label
    start = len(trace.events)
    if isinstance(action, CrossTrackAction) and action.ctx.perception.lidar is None:
        trace.add("observation", "P4.read", {"missing": True})
    if isinstance(action, DetachAction) and action._backup_guard is None:
        trace.add("guard", "backup_guard", {"missing": True, "allowed": False})
    if isinstance(action, RetreatAction) and action._retreat_guard is None:
        trace.add("guard", "retreat_guard", {"missing": True, "allowed": False})
    action.start()
    until = time.monotonic() + deadline_s
    last = None
    cancel_sent = False
    while time.monotonic() < until:
        st = action.read()
        signature = (st["state"], getattr(action, "_phase", None),
                     st["data"].get("phase"), st["data"].get("reason"), st["error"])
        if signature != last:
            trace.add("state_transition", label, {"state": st["state"],
                      "internal_phase": getattr(action, "_phase", None),
                      "phase": st["data"].get("phase"),
                      "reason": st["data"].get("reason"),
                      "error": st["error"]})
            last = signature
        if cancel and not cancel_sent and st["data"].get("command_id"):
            action.stop()
            cancel_sent = True
            trace.add("cancel_request", label, {"command_id": st["data"]["command_id"]})
        if st["state"] in TERMINAL:
            break
        time.sleep(0.01)
    else:
        action.stop()
        action.join(1.0)
        raise AssertionError(label + " exceeded the offline replay deadline")
    action.join(1.0)
    actual = action.read()
    assert actual["state"] == expected, (label, actual)
    if error is not None:
        assert actual["error"] == error, (label, actual)
    events = trace.events[start:]
    return {"case": label, "action": action.NAME, "terminal": actual["state"],
            "terminal_reason": actual["error"] or "completed",
            "data": actual["data"], "events": events}


NORMAL_CHAIN = ["cruise", "cross_track", "find_start", "approach", "align",
                "vent_high", "vent_low", "detach", "retreat"]


def normal_replay(trace):
    ctx, sensors, robot = mock_context(trace,
        manipulator=MockManipulator(exec_seconds=0.02),
        rod=MockRod(exec_seconds=0.02),
        acoustic=MockAcoustic(results=["SUCCESS", "SUCCESS"]))
    def permit(name):
        def check():
            trace.add("guard", name, {"allowed": True, "source": "synthetic"}, limit=1)
            return True
        return check

    settings = {
        "cruise": {"end": (0.25, 0.0), "pos_tol": 0.08, "period": 0.02},
        "cross_track": {"exit_point": (0.55, 0.0), "arrive_tol": 0.15,
                        "period": 0.02, "cross_timeout": 4.0},
        "find_start": {"expect_car_no": "45001", "period": 0.02,
                       "scan_timeout": 2.0},
        "approach": {"period": 0.02, "drive_timeout": 2.0},
        "align": {"period": 0.02, "max_adjust_s": 2.0, "coarse_heading": 0.2},
        "vent_high": {"period": 0.02, "exec_timeout": 1.0},
        "vent_low": {"period": 0.02, "exec_timeout": 1.0},
        "detach": {"period": 0.02, "exec_timeout": 1.0,
                   "backup_guard": permit("backup_guard")},
        "retreat": {"period": 0.02, "retreat_distance": 0.2,
                    "backup_timeout": 3.0, "retreat_guard": permit("retreat_guard")},
    }
    results = []
    try:
        assert set(NORMAL_CHAIN) == set(ACTION_CLASSES)
        for name in NORMAL_CHAIN:
            action = create_action(name, ctx, **settings[name])
            result = run_action(action, trace, label="normal/" + name,
                                expected="SUCCESS")
            if name == "align":
                yaw_error = action._wrap(settings[name]["coarse_heading"] - robot.get_pose().yaw)
                assert abs(yaw_error) <= 0.05, ("normal/align heading", yaw_error)
            results.append(result)
    finally:
        for sensor in (sensors.lever, sensors.pipe, sensors.scene, sensors.lidar):
            if sensor:
                sensor.close()
    return results


def adverse_replay(trace):
    results = []
    cases = [
        ("missing_P4", CrossTrackAction, {"exit_point": (0.4, 0), "period": 0.02},
         {"with_lidar": False}, "FAIL", "E_LIDAR_UNAVAILABLE", False),
        ("stale_P1", ApproachAction, {"period": 0.02, "drive_timeout": 0.12},
         {"stale_lever": True}, "FAIL", "FAIL_APPROACH_TIMEOUT", False),
        ("wrong_command", VentAction, {"period": 0.02, "exec_timeout": 0.12},
         {"manipulator": FaultManipulator("wrong"), "acoustic": MockAcoustic()},
         "FAIL", "FAIL_MANIP_TIMEOUT", False),
        ("late_receipt", VentAction, {"period": 0.02, "exec_timeout": 0.12},
         {"manipulator": FaultManipulator("late"), "acoustic": MockAcoustic()},
         "FAIL", "FAIL_MANIP_TIMEOUT", False),
        ("replayed_receipt", VentAction, {"period": 0.02, "exec_timeout": 0.12},
         {"manipulator": FaultManipulator("repeat"),
          "acoustic": MockAcoustic(results=["UNCERTAIN", "SUCCESS"])},
         "FAIL", "FAIL_MANIP_TIMEOUT", False),
        ("cancel_vent", VentAction, {"period": 0.02, "exec_timeout": 6.0},
         {"manipulator": MockManipulator(exec_seconds=5.0),
          "acoustic": MockAcoustic()}, "ABORTED", "aborted_by_request", True),
        ("no_detach_backup_guard", DetachAction, {"period": 0.02},
         {"rod": MockRod(exec_seconds=0.02)},
         "FAIL", "E_BACKUP_GUARD_UNAVAILABLE", False),
        ("no_retreat_guard", RetreatAction, {"period": 0.02},
         {"manipulator": MockManipulator(exec_seconds=0.02)},
         "FAIL", "E_RETREAT_GUARD_UNAVAILABLE", False),
    ]
    for name, cls, kwargs, resources, expected, error, cancel in cases:
        ctx, sensors, robot = mock_context(trace, **resources)
        try:
            result = run_action(cls(ctx, **kwargs), trace, label="adverse/" + name,
                                expected=expected, error=error, cancel=cancel)
            assert robot._cmd == (0.0, 0.0), (name, robot._cmd)
            if name.startswith("no_"):
                assert robot.pose.e == 0.0, name
            results.append(result)
        finally:
            for sensor in (sensors.lever, sensors.pipe, sensors.scene, sensors.lidar):
                if sensor:
                    sensor.close()
    return results


def replay():
    trace = Trace()
    cases = normal_replay(trace) + adverse_replay(trace)
    return {"mode": "MOCK_ONLY", "field_acceptance": False,
            "normal_chain": NORMAL_CHAIN, "cases": cases,
            "limitations": ["Mock receipts do not confirm physical stop.",
                            "Injected backup guards are demonstration values only.",
                            "No rear clearance policy or device access is implemented."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path(__file__).resolve().parents[2] / "reports/offline/replay.json"
    parser.add_argument("--output", type=Path, default=default,
                        help="JSON trace path (default: reports/offline)")
    args = parser.parse_args(argv)
    result = replay()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=str) + "\n",
                           encoding="utf-8")
    print("MOCK_ONLY replay: %d cases, %d normal, %d adverse; trace=%s" % (
        len(result["cases"]), len(result["normal_chain"]),
        len(result["cases"]) - len(result["normal_chain"]), args.output))


if __name__ == "__main__":
    main()
