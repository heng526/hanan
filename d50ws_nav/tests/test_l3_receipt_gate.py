"""L3 offline receipt and perception gates. No device or camera is opened."""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import ApproachAction, DetachAction, RetreatAction, VentAction
from nav.actions.vent_high import HighVentAction
from nav.actions.vent_low import LowVentAction
from nav.context import PerceptionBundle, RunContext
from sdk_adapter.mock import MockRobot
from sdk_adapter.mock_actuators import MockManipulator, MockRod, MockAcoustic


class FixedPerception:
    def __init__(self, **data):
        self.data = data
        self.enabled = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def read(self):
        return {"enabled": self.enabled, "frame_count": 1, "age_s": 0.0,
                "error_code": None, **self.data}


class OldReceiptManipulator(MockManipulator):
    def read(self):
        return {"state": "DONE", "command_id": "vent-previous",
                "timestamp": time.time()}


class LateReceiptManipulator(MockManipulator):
    def read(self):
        reply = super().read()
        reply["timestamp"] = self._t0 - 1.0
        return reply


class LegacyManipulator(MockManipulator):
    def start_vent(self, mode, lever_pose):
        return True


class OldReceiptRod(MockRod):
    def read(self):
        return {"state": "DONE", "command_id": "detach-previous",
                "timestamp": time.time()}


def finish(action, timeout=3):
    action.start()
    action.join(timeout)
    assert not action.is_alive(), action.read()
    return action.read()


def test_approach_stale_frame_never_moves_and_times_out():
    robot = MockRobot(dt=0.01)
    detector = FixedPerception(target_detected=False,
                               error_code="E_TARGET_NOT_FOUND", age_s=8.0)
    action = ApproachAction(RunContext(chassis=robot,
                            perception=PerceptionBundle(lever=detector)),
                            period=0.01, drive_timeout=0.12)
    state = finish(action)
    assert state["error"] == "FAIL_APPROACH_TIMEOUT"
    assert robot.pose.e == 0.0
    assert not detector.enabled


def test_approach_repeated_frame_stops_before_more_motion():
    robot = MockRobot(dt=0.01)
    detector = FixedPerception(target_detected=False,
                               error_code="E_TARGET_NOT_FOUND")
    action = ApproachAction(RunContext(chassis=robot,
                            perception=PerceptionBundle(lever=detector)),
                            period=0.01, drive_timeout=0.12)
    state = finish(action)
    assert state["state"] == "FAIL"
    assert 0.0 < robot.pose.e <= 0.003
    assert robot._cmd == (0.0, 0.0)


@pytest.mark.parametrize("action_class", [VentAction, HighVentAction, LowVentAction])
def test_vent_missing_acoustic_does_not_start_manipulator(action_class):
    manip = MockManipulator(exec_seconds=0.01)
    state = finish(action_class(RunContext(chassis=MockRobot(), manipulator=manip),
                                period=0.01))
    assert state["error"] == "E_ACOUSTIC_UNAVAILABLE"
    assert manip._t0 is None


def test_vent_legacy_start_ack_is_rejected():
    state = finish(VentAction(RunContext(chassis=MockRobot(),
                        manipulator=LegacyManipulator(), acoustic=MockAcoustic()),
                        period=0.01))
    assert state["error"] == "E_MANIP_START_UNCONFIRMED"


def test_vent_replayed_done_receipt_times_out_and_cancels():
    manip = OldReceiptManipulator(exec_seconds=0.01)
    state = finish(VentAction(RunContext(chassis=MockRobot(),
                        manipulator=manip, acoustic=MockAcoustic()),
                        period=0.01, exec_timeout=0.12))
    assert state["error"] == "FAIL_MANIP_TIMEOUT"
    assert manip._cancelled


def test_vent_late_timestamp_receipt_times_out():
    manip = LateReceiptManipulator(exec_seconds=0.01)
    state = finish(VentAction(RunContext(chassis=MockRobot(),
                        manipulator=manip, acoustic=MockAcoustic()),
                        period=0.01, exec_timeout=0.12))
    assert state["error"] == "FAIL_MANIP_TIMEOUT"
    assert manip._cancelled


def test_vent_abort_cancels_current_command():
    manip = MockManipulator(exec_seconds=5.0)
    action = VentAction(RunContext(chassis=MockRobot(), manipulator=manip,
                       acoustic=MockAcoustic()), period=0.01)
    action.start()
    deadline = time.time() + 1.0
    while manip._t0 is None and time.time() < deadline:
        time.sleep(0.005)
    assert manip._t0 is not None
    action.stop()
    action.join(2)
    assert action.read()["state"] == "ABORTED"
    assert manip._cancelled


def pipe():
    return FixedPerception(pipe_detected=True, hook_point_dog=[0.5, 0, 0.4],
                           lift_direction_dog=[0, 0, 1])


class AdvancingPipe(FixedPerception):
    def __init__(self):
        super().__init__(pipe_detected=True, hook_point_dog=[0.5, 0, 0.4],
                         lift_direction_dog=[0, 0, 1])
        self.count = 0

    def read(self):
        self.count += 1
        result = super().read()
        result["frame_count"] = self.count
        return result

    def check_detach(self, **_kwargs):
        return {"detach_success": True, "separation_distance": 0.5,
                "frame_count": 1, "timestamp": time.time() - 10}


def test_detach_missing_guard_prevents_rod_start():
    rod = MockRod(exec_seconds=0.01)
    p2 = pipe()
    state = finish(DetachAction(RunContext(chassis=MockRobot(), rod=rod,
                          perception=PerceptionBundle(pipe=p2)), period=0.01))
    assert state["error"] == "E_BACKUP_GUARD_UNAVAILABLE"
    assert rod._t0 is None
    assert not p2.enabled


def test_detach_old_rod_receipt_prevents_backup_and_cancels():
    robot = MockRobot(dt=0.01)
    rod = OldReceiptRod(exec_seconds=0.01)
    state = finish(DetachAction(RunContext(chassis=robot, rod=rod,
                       perception=PerceptionBundle(pipe=pipe())),
                       period=0.01, exec_timeout=0.12,
                       backup_guard=lambda: True))
    assert state["error"] == "FAIL_ROD_TIMEOUT"
    assert robot.pose.e == 0.0
    assert rod._cancelled


def test_detach_repeated_post_action_frame_cannot_claim_success():
    robot = MockRobot(dt=0.01)
    p2 = pipe()
    p2.check_detach = lambda **_kw: {"detach_success": True,
                          "separation_distance": 0.5,
                          "frame_count": 1, "timestamp": time.time()}
    action = DetachAction(RunContext(chassis=robot, rod=MockRod(exec_seconds=0.01),
                          perception=PerceptionBundle(pipe=p2)),
                          period=0.01, lock_timeout=0.15,
                          backup_guard=lambda: True)
    state = finish(action)
    assert state["error"] == "E_DETACH_CHECK_TIMEOUT"
    assert robot._cmd == (0.0, 0.0)
    assert not p2.enabled


def test_detach_stale_check_receipt_cannot_claim_success():
    p2 = AdvancingPipe()
    state = finish(DetachAction(RunContext(chassis=MockRobot(dt=0.01),
                        rod=MockRod(exec_seconds=0.01),
                        perception=PerceptionBundle(pipe=p2)),
                        period=0.01, backup_guard=lambda: True))
    assert state["error"] == "E_DETACH_CHECK_STALE"
    assert not p2.enabled


def safe_scene():
    return FixedPerception(safety_status="OK", risk_level="NONE")


def test_retreat_missing_guard_prevents_stow_and_reverse():
    robot = MockRobot(dt=0.01)
    manip = MockManipulator(exec_seconds=0.01)
    scene = safe_scene()
    state = finish(RetreatAction(RunContext(chassis=robot, manipulator=manip,
                           perception=PerceptionBundle(scene=scene)), period=0.01))
    assert state["error"] == "E_RETREAT_GUARD_UNAVAILABLE"
    assert manip._t0 is None and robot.pose.e == 0.0
    assert not scene.enabled


def test_retreat_repeated_scene_frame_stops_and_times_out():
    robot = MockRobot(dt=0.01)
    scene = safe_scene()
    action = RetreatAction(RunContext(chassis=robot,
                manipulator=MockManipulator(exec_seconds=0.01),
                perception=PerceptionBundle(scene=scene)), period=0.01,
                backup_timeout=0.18, retreat_guard=lambda: True)
    state = finish(action)
    assert state["error"] == "FAIL_BACKUP_TIMEOUT"
    assert robot.pose.e < 0 and robot.pose.e >= -0.004
    assert robot._cmd == (0.0, 0.0)
    assert not scene.enabled


def test_retreat_abort_cancels_stow_and_disables_scene():
    manip = MockManipulator(exec_seconds=5.0)
    scene = safe_scene()
    action = RetreatAction(RunContext(chassis=MockRobot(), manipulator=manip,
                    perception=PerceptionBundle(scene=scene)), period=0.01,
                    retreat_guard=lambda: True)
    action.start()
    deadline = time.time() + 1.0
    while manip._t0 is None and time.time() < deadline:
        time.sleep(0.005)
    assert manip._t0 is not None
    action.stop()
    action.join(2)
    assert action.read()["state"] == "ABORTED"
    assert manip._cancelled and not scene.enabled
