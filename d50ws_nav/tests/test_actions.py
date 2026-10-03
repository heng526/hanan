# -*- coding: utf-8 -*-
"""动作类全集测试（A2~A10）：每个动作的成功路径 + 关键失败路径。

全部基于 Mock（世界系闭环仿真：狗动了感知相对坐标真实变化），
验证动作类的状态流、错误码、开/关感知的纪律。
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nav.actions import (CrossTrackAction, FindWorkStartAction, ApproachAction,
                         AlignAction, VentAction, DetachAction, RetreatAction)
from nav.context import RunContext, PerceptionBundle
from nav.base_action import ActionState
from sdk_adapter.mock import MockRobot
from sdk_adapter.mock_actuators import MockManipulator, MockRod, MockAcoustic
from perception.mocks import (MockLeverDetector, MockPipeDetector,
                              MockSceneSegmenter, MockLidarProcessor)


def run(act, timeout=30.0):
    act.start()
    ok = act.wait(timeout=timeout)
    return ok, act.read()


def mk_ctx(with_safety=False):
    return RunContext(chassis=MockRobot(dt=0.05))


# ---------------- A2 过轨道 ----------------

def test_cross_track_success():
    robot = MockRobot(dt=0.05)
    bundle = PerceptionBundle(
        lidar=MockLidarProcessor(period=0.05),
        scene=MockSceneSegmenter(chassis=robot, period=0.05))
    ctx = RunContext(chassis=robot, perception=bundle)
    act = CrossTrackAction(ctx, exit_point=(1.0, 0.0), period=0.05,
                           wait_timeout=1.0, cross_timeout=15.0)
    ok, st = run(act, timeout=30)
    assert ok, st
    assert st["data"]["crossed"] is True
    assert robot.pose.dist_to(1.0, 0.0) <= 0.3


def test_cross_track_train_blocked():
    lidar = MockLidarProcessor(period=0.05)
    lidar.left_block = True                      # 左侧来车
    robot = MockRobot(dt=0.05)
    bundle = PerceptionBundle(lidar=lidar)
    ctx = RunContext(chassis=robot, perception=bundle)
    act = CrossTrackAction(ctx, exit_point=(1.0, 0.0), period=0.05,
                           wait_timeout=0.8, cross_timeout=15.0)
    ok, st = run(act, timeout=15)
    assert not ok
    assert st["state"] == "FAIL"
    assert st["error"] == "FAIL_TRAIN_APPROACHING"
    assert st["data"]["wait_side"] == "left"


# ---------------- A3 找车头 ----------------

def test_find_start_success():
    robot = MockRobot(dt=0.05)
    scene = MockSceneSegmenter(chassis=robot, car_world=(1.5, 0.0),
                               car_no_text="C62 45001", period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(scene=scene))
    act = FindWorkStartAction(ctx, expect_car_no="45001", period=0.05,
                              scan_timeout=15.0)
    ok, st = run(act, timeout=20)
    assert ok, st
    assert st["data"]["car_type"] == "C62"
    assert st["data"]["air_type"] == "high"
    assert st["data"]["lever_offset"] == 1.2


def test_find_start_mismatch_timeout():
    robot = MockRobot(dt=0.05)
    scene = MockSceneSegmenter(chassis=robot, car_world=(1.5, 0.0),
                               car_no_text="C64 45032", period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(scene=scene))
    act = FindWorkStartAction(ctx, expect_car_no="45099", period=0.05,
                              scan_timeout=1.5)
    ok, st = run(act, timeout=15)
    assert not ok
    assert st["error"] == "FAIL_CARNO_MISMATCH"


def test_find_start_unknown_car_type():
    robot = MockRobot(dt=0.05)
    scene = MockSceneSegmenter(chassis=robot, car_world=(1.5, 0.0),
                               car_no_text="XX9 45001", period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(scene=scene))
    act = FindWorkStartAction(ctx, expect_car_no="45001", period=0.05,
                              scan_timeout=10.0)
    ok, st = run(act, timeout=15)
    assert not ok
    assert st["error"] == "FAIL_UNKNOWN_CAR_TYPE"


# ---------------- A5 作业接近 ----------------

def test_approach_stops_on_detect():
    robot = MockRobot(dt=0.05)
    lever = MockLeverDetector(chassis=robot, lever_world=(2.5, 0.0),
                              detect_range=2.0, period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    act = ApproachAction(ctx, detector_name="lever", drive_speed=0.3,
                         period=0.05, drive_timeout=15.0)
    ok, st = run(act, timeout=25)
    assert ok, st
    assert st["data"]["target_point_dog"] is not None
    # 狗应停在检测圈附近（2.5 - 2.0 = 0.5m 处）
    assert 0.3 <= robot.pose.e <= 0.8


# ---------------- A6 对位 ----------------

def test_align_high_lateral_converges():
    robot = MockRobot(dt=0.05)
    lever = MockLeverDetector(chassis=robot, lever_world=(0.6, 0.3),
                              detect_range=5.0, period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    act = AlignAction(ctx, mode="high", desired=(0.6, 0.0), tol=(0.05, 0.05),
                      period=0.05, max_adjust_s=20.0)
    ok, st = run(act, timeout=40)
    assert ok, st
    assert abs(st["data"]["final_offset"][1]) <= 0.05
    assert abs(st["data"]["final_offset"][0]) <= 0.05
    # 狗应左移约 0.3m
    assert abs(robot.pose.n - 0.3) <= 0.08


def test_align_forward_converges():
    robot = MockRobot(dt=0.05)
    lever = MockLeverDetector(chassis=robot, lever_world=(1.0, 0.0),
                              detect_range=5.0, period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    act = AlignAction(ctx, mode="high", desired=(0.6, 0.0), tol=(0.05, 0.05),
                      period=0.05, max_adjust_s=20.0)
    ok, st = run(act, timeout=40)
    assert ok, st
    assert abs(robot.pose.e - 0.4) <= 0.08       # 前移 0.4m


def test_align_target_never_found():
    robot = MockRobot(dt=0.05)
    lever = MockLeverDetector(chassis=robot, lever_world=(50.0, 50.0),
                              detect_range=2.0, period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    act = AlignAction(ctx, mode="high", period=0.05, max_adjust_s=5.0)
    ok, st = run(act, timeout=15)
    assert not ok
    assert st["error"] == "E_TARGET_NOT_FOUND"


def test_align_low_squats():
    robot = MockRobot(dt=0.05)
    lever = MockLeverDetector(chassis=robot, lever_world=(0.6, 0.0),
                              detect_range=5.0, period=0.05)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    act = AlignAction(ctx, mode="low", desired=(0.6, 0.0), tol=(0.05, 0.05),
                      period=0.05, max_adjust_s=20.0)
    ok, _ = run(act, timeout=40)
    assert ok
    assert any("height" in entry for entry in robot.log), "低位应对蹲指令"


# ---------------- A7/A8 排风 ----------------

def test_vent_success_with_retry():
    robot = MockRobot(dt=0.05)
    ctx = RunContext(chassis=robot,
                     manipulator=MockManipulator(exec_seconds=0.2),
                     acoustic=MockAcoustic(results=["UNCERTAIN", "SUCCESS"]))
    act = VentAction(ctx, mode="high", period=0.05, exec_timeout=5.0)
    ok, st = run(act, timeout=20)
    assert ok, st
    assert st["data"]["vent_result"] == "SUCCESS"
    assert st["data"]["retried"] is True
    assert st["data"]["manip"] == "started"


def test_vent_fail_after_retry():
    robot = MockRobot(dt=0.05)
    ctx = RunContext(chassis=robot,
                     manipulator=MockManipulator(exec_seconds=0.2),
                     acoustic=MockAcoustic(results=["FAIL", "FAIL"]))
    act = VentAction(ctx, mode="low", period=0.05, exec_timeout=5.0)
    ok, st = run(act, timeout=20)
    assert not ok
    assert st["error"] == "FAIL_VENT_FAIL"


def test_vent_missing_actuator_fails_closed():
    robot = MockRobot(dt=0.05)
    ctx = RunContext(chassis=robot, acoustic=MockAcoustic(results=["SUCCESS"]))
    act = VentAction(ctx, mode="high", period=0.05)
    ok, st = run(act, timeout=10)
    assert not ok, st
    assert st["state"] == "FAIL"
    assert st["error"] == "E_MANIPULATOR_UNAVAILABLE"


# ---------------- A9 摘管 ----------------

def _detach_ctx(detach_distance):
    robot = MockRobot(dt=0.05)
    pipe = MockPipeDetector(chassis=robot, pipe_world=(1.0, 0.0),
                            detect_range=5.0, period=0.05,
                            detach_distance=detach_distance)
    ctx = RunContext(chassis=robot, perception=PerceptionBundle(pipe=pipe),
                     rod=MockRod(exec_seconds=0.2))
    return ctx, pipe


def test_detach_success():
    ctx, _ = _detach_ctx(detach_distance=0.5)
    act = DetachAction(ctx, period=0.05, exec_timeout=5.0, lock_timeout=5.0,
                       backup_guard=lambda: True)
    ok, st = run(act, timeout=25)
    assert ok, st
    assert st["data"]["detach_result"] == "SUCCESS"
    assert st["data"]["separation_distance"] == 0.5


def test_detach_fail_reconnect_risk():
    ctx, _ = _detach_ctx(detach_distance=0.12)
    act = DetachAction(ctx, period=0.05, exec_timeout=5.0, lock_timeout=5.0,
                       backup_guard=lambda: True)
    ok, st = run(act, timeout=25)
    assert not ok
    assert st["error"] == "FAIL_DETACH_NOT_SEPARATED"
    assert st["data"]["reconnect_risk"] is True


# ---------------- A10 退出 ----------------

def test_retreat_backup_and_turn():
    robot = MockRobot(dt=0.05)
    scene = MockSceneSegmenter(chassis=robot, period=0.05)
    ctx = RunContext(chassis=robot, manipulator=MockManipulator(exec_seconds=0.05),
                     perception=PerceptionBundle(scene=scene))
    act = RetreatAction(ctx, mode="high", retreat_distance=0.5,
                        retreat_speed=0.3, target_heading=math.pi / 2,
                        period=0.05, backup_timeout=15.0,
                        retreat_guard=lambda: True)
    ok, st = run(act, timeout=30)
    assert ok, st
    fp = st["data"]["final_pose"]
    assert abs(fp[0] + 0.5) <= 0.1               # 后撤 0.5m
    assert abs(fp[1]) <= 0.05
    assert abs(fp[2] - math.pi / 2) <= 0.06      # 转到目标朝向


def test_retreat_no_turn():
    robot = MockRobot(dt=0.05)
    scene = MockSceneSegmenter(chassis=robot, period=0.05)
    ctx = RunContext(chassis=robot, manipulator=MockManipulator(exec_seconds=0.05),
                     perception=PerceptionBundle(scene=scene))
    act = RetreatAction(ctx, mode="detach", retreat_distance=0.3,
                        retreat_speed=0.3, target_heading=None,
                        period=0.05, backup_timeout=15.0,
                        retreat_guard=lambda: True)
    ok, st = run(act, timeout=20)
    assert ok, st


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS %s" % name)
            except AssertionError as e:
                failed += 1
                print("FAIL %s: %s" % (name, e))
    if failed:
        print("%d FAILED" % failed)
        sys.exit(1)
    print("action tests: all passed")
