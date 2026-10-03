"""L2/L3 框架的离线契约：不用相机、雷达、SDK 或真实机器狗。"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import create_action
from nav.base_action import ActionState, BaseAction
from nav.context import PerceptionBundle, RunContext
from perception import LeverDetector, PipeDetector, SceneSegmenter, LidarProcessor
from sdk_adapter.mock import MockRobot


class FakeSource:
    def read(self):
        return True, {"frame": "mock"}, time.time()


def wait_frame(perception):
    deadline = time.time() + 2.0
    while time.time() < deadline:
        state = perception.read()
        if state["frame_count"] and state["age_s"] is not None:
            return state
        time.sleep(0.01)
    pytest.fail("感知线程未更新缓存")


def test_lever_tracks_dog_frame_and_pauses():
    seen = []

    def detect(frame, params):
        seen.append(params)
        return {"target_detected": True, "target_point_camera": [1, 2, 3],
                "lever_type": "high", "confidence": 0.9, "quality_score": 0.8}

    p = LeverDetector(FakeSource(), detect, to_dog=lambda xyz: [xyz[0] + 2, xyz[1], xyz[2]])
    try:
        assert p.is_alive()
        assert p.read()["error_code"] == "E_SENSOR_NO_DATA"
        p.enable(roi="left", confidence_threshold=0.5, track_rate=50)
        state = wait_frame(p)
        assert state["target_point_dog"] == [3.0, 2.0, 3.0]
        assert state["target_detected"] and state["track_status"] == "TRACKING"
        assert seen and seen[0]["roi"] == "left"
        p.disable()
        n = p.read()["frame_count"]
        time.sleep(0.06)
        assert p.read()["frame_count"] <= n + 1
    finally:
        p.close()
    assert not p.is_alive()


def test_lever_missing_extrinsic_fails_closed():
    p = LeverDetector(FakeSource(), lambda _f, _p: {
        "target_detected": True, "target_point_camera": [1, 0, 0],
        "confidence": 0.9})
    try:
        p.enable()
        state = wait_frame(p)
        assert not state["target_detected"]
        assert state["error_code"] == "E_TF_UNAVAILABLE"
    finally:
        p.close()


def test_stale_camera_frame_is_rejected():
    class StaleSource:
        def read(self):
            return True, {"frame": "old"}, time.time() - 5.0

    p = LeverDetector(StaleSource(), lambda _f, _p: {
        "target_detected": True, "target_point_camera": [1, 0, 0],
        "confidence": 0.9}, to_dog=lambda xyz: xyz)
    try:
        p.enable()
        assert wait_frame(p)["error_code"] == "E_SENSOR_NO_DATA"
    finally:
        p.close()


def test_pipe_point_count_and_strict_distance_check():
    def detect(_frame, _params):
        return {
            "pipe_detected": True, "connector_detected": True,
            "joint_point_camera": [1, 0, 0], "hook_point_camera": [1, 0, -0.1],
            "lift_direction_camera": [0, 0, 1],
            "separate_direction_camera": [0, 1, 0],
            "separation_points_camera": [[0, 0, 0], [0.4, 0, 0]],
        }

    identity = lambda xyz: xyz
    p = PipeDetector(FakeSource(), detect, to_dog=identity, vector_to_dog=identity)
    try:
        p.enable()
        assert wait_frame(p)["hook_point_dog"] == [1.0, 0.0, -0.1]
        assert p.check_detach(threshold=0.3)["detach_success"]
        assert not p.check_detach(threshold=0.4)["detach_success"]
        p.disable()
        assert p.check_detach()["error_code"] == "E_TARGET_NOT_FOUND"
    finally:
        p.close()


def test_pipe_missing_second_endpoint_does_not_claim_success():
    def detect(_frame, _params):
        return {
            "pipe_detected": True, "joint_point_camera": [1, 0, 0],
            "hook_point_camera": [1, 0, 0], "lift_direction_camera": [0, 0, 1],
            "separate_direction_camera": [0, 1, 0],
            "separation_points_camera": [[0, 0, 0]],
        }

    p = PipeDetector(FakeSource(), detect, to_dog=lambda x: x,
                     vector_to_dog=lambda x: x)
    try:
        p.enable()
        wait_frame(p)
        result = p.check_detach()
        assert not result["detach_success"]
        assert result["separation_distance"] is None
    finally:
        p.close()


def test_scene_and_lidar_publish_injected_results():
    scene = SceneSegmenter(FakeSource(), lambda _f, _p: {
        "walkable_area_ratio": 0.8, "safety_status": "OK",
        "car_detected": True, "car_no_text": "45001", "risk_level": "NONE"})
    lidar = LidarProcessor(FakeSource(), lambda _s, _p: {
        "sector_scan": {"left": {"nearest_distance": 8.0, "dynamic": False}},
        "nearest_distance": 8.0, "risk_level": "NONE"})
    try:
        scene.enable(scene_hint="CROSS_TRACK")
        lidar.enable(warning_distance=10.0)
        assert wait_frame(scene)["car_no_text"] == "45001"
        assert wait_frame(lidar)["sector_scan"]["left"]["nearest_distance"] == 8.0
    finally:
        scene.close()
        lidar.close()


def test_missing_l2_sources_never_report_safe():
    instances = [LeverDetector(), PipeDetector(), SceneSegmenter(), LidarProcessor()]
    try:
        for p in instances:
            p.enable()
            assert wait_frame(p)["error_code"] == "E_SENSOR_NO_DATA"
        assert instances[2].read()["walkable_area_ratio"] == 0.0
        assert instances[3].read()["risk_level"] == "UNKNOWN"
    finally:
        for p in instances:
            p.close()


def test_scene_processor_failure_keeps_crossing_blocked():
    def broken(_frame, _params):
        raise RuntimeError("algorithm unavailable")

    p = SceneSegmenter(FakeSource(), broken)
    try:
        p.enable()
        state = wait_frame(p)
        assert state["error_code"] == "E_INFER_CRASH"
        assert state["walkable_area_ratio"] == 0.0
        assert state["step_height"] > 0.4
    finally:
        p.close()


class FakePerception:
    def __init__(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False


class FailingAction(BaseAction):
    NAME = "failing_action"

    def _setup(self):
        self._perception_set("scene", True)

    def _run_once(self):
        raise RuntimeError("offline failure")

    def _on_finish(self):
        self._perception_set("scene", False)


def test_action_failure_cleans_up_perception():
    perception = FakePerception()
    ctx = RunContext(chassis=MockRobot(),
                     perception=PerceptionBundle(scene=perception))
    action = FailingAction(ctx)
    action.start()
    action.join(timeout=2)
    assert action.state == ActionState.FAIL
    assert not perception.enabled
    assert "offline failure" in action.read()["error"]


def test_action_factory_constructs_only():
    ctx = RunContext(chassis=MockRobot())
    action = create_action("cruise", ctx, end=(1.0, 0.0))
    assert action.read()["state"] == "IDLE"
    assert action.ctx is ctx
    high = create_action("vent_high", ctx)
    low = create_action("vent_low", ctx)
    assert high.read()["action"] == "vent_high" and high._mode == "high"
    assert low.read()["action"] == "vent_low" and low._mode == "low"
    with pytest.raises(ValueError):
        create_action("unknown", ctx)


def test_vent_task_without_actuator_fails_closed():
    ctx = RunContext(chassis=MockRobot())
    action = create_action("vent_high", ctx)
    action.start()
    action.join(timeout=2)
    assert action.read()["state"] == "FAIL"
    assert action.read()["error"] == "E_MANIPULATOR_UNAVAILABLE"
