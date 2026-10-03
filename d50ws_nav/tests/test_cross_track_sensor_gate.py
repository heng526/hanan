"""A2 过轨感知门禁的离线回归；只使用 Mock 底盘和伪感知缓存。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import CrossTrackAction
from nav.context import PerceptionBundle, RunContext
from sdk_adapter.mock import MockRobot


class RecordingRobot(MockRobot):
    def __init__(self):
        super().__init__()
        self.velocity_commands = []

    def set_velocity(self, v, w):
        self.velocity_commands.append((v, w))
        return super().set_velocity(v, w)


class FakePerception:
    def __init__(self, result):
        self.result = result
        self.enabled = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def read(self):
        return {**self.result, "enabled": self.enabled}


def lidar_result(**overrides):
    base = {
        "frame_count": 1, "age_s": 0.0, "error_code": None,
        "risk_level": "NONE",
        "sector_scan": {
            "left": {"nearest_distance": 50.0, "dynamic": False},
            "right": {"nearest_distance": 50.0, "dynamic": False},
        },
    }
    base.update(overrides)
    return base


def scene_result(**overrides):
    base = {
        "frame_count": 1, "age_s": 0.0, "error_code": None,
        "walkable_area_ratio": 0.8, "step_height": 0.05,
        "safety_status": "OK", "risk_level": "NONE",
    }
    base.update(overrides)
    return base


def run_gate(*, lidar=None, scene=None, sensor_ready_timeout=0.05):
    robot = RecordingRobot()
    ctx = RunContext(chassis=robot,
                     perception=PerceptionBundle(lidar=lidar, scene=scene))
    action = CrossTrackAction(ctx, exit_point=(1.0, 0.0), period=0.01,
                              sensor_ready_timeout=sensor_ready_timeout)
    action.start()
    action.join(timeout=2.0)
    assert not action.is_alive(), action.read()
    state = action.read()
    assert state["state"] == "FAIL", state
    assert not any(entry == "terrain=wheel_leg_rl" for entry in robot.log)
    assert all(v == 0.0 and w == 0.0 for v, w in robot.velocity_commands)
    if lidar is not None:
        assert not lidar.enabled
    if scene is not None:
        assert not scene.enabled
    return state


def test_missing_lidar_cannot_enter_crossing():
    state = run_gate(scene=FakePerception(scene_result()))
    assert state["error"] == "E_LIDAR_UNAVAILABLE"


def test_missing_scene_cannot_enter_crossing():
    state = run_gate(lidar=FakePerception(lidar_result()))
    assert state["error"] == "E_SCENE_UNAVAILABLE"


def test_stale_lidar_cannot_enter_crossing():
    state = run_gate(lidar=FakePerception(lidar_result(age_s=5.0)))
    assert state["error"] == "E_LIDAR_DATA_INVALID"


def test_lidar_error_cannot_enter_crossing():
    state = run_gate(lidar=FakePerception(lidar_result(error_code="E_INFER_CRASH")))
    assert state["error"] == "E_LIDAR_DATA_INVALID"


def test_missing_sector_fields_cannot_be_treated_as_clear():
    state = run_gate(lidar=FakePerception(lidar_result(
        sector_scan={"left": {"nearest_distance": 50.0}})))
    assert state["error"] == "E_LIDAR_SECTOR_INVALID"


def test_cold_start_wait_is_bounded():
    state = run_gate(lidar=FakePerception(lidar_result(
        frame_count=0, age_s=None, error_code="E_SENSOR_NO_DATA")))
    assert state["error"] == "E_LIDAR_READY_TIMEOUT"


def test_bad_scene_status_cannot_enter_crossing():
    state = run_gate(lidar=FakePerception(lidar_result()),
                     scene=FakePerception(scene_result(safety_status="UNKNOWN")))
    assert state["error"] == "FAIL_NO_CROSSBOARD"


def test_stale_scene_cannot_enter_crossing():
    state = run_gate(lidar=FakePerception(lidar_result()),
                     scene=FakePerception(scene_result(age_s=2.0)))
    assert state["error"] == "E_SCENE_DATA_INVALID"
