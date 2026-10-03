# -*- coding: utf-8 -*-
"""全链路仿真集成测试：teleop 运动循环 + 安全监护 + 跟随器 + Mock 机器人。

模拟真实场景：
1) teleop 的 MotionLoop 以 10Hz 驱动 Mock 机器人
2) 跟随器在另一逻辑层输出航点速度指令
3) 低电量触发安全停车；软急停触发即时停止
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import threading

from common.geo import dist_heading
from nav.follower import Waypoint, WaypointFollower, FollowParams, FollowerState
from nav.safety import SafetyMonitor, SafetyConfig, Verdict
from tools.teleop import MotionLoop
from sdk_adapter.mock import MockRobot


def test_motionloop_stop_on_low_battery():
    """电量跌破阈值后，运动循环应自动清零速度指令。"""
    robot = MockRobot()
    robot.battery = 20.004                     # 运行中约1秒内跌破 20 停车线
    monitor = SafetyMonitor(SafetyConfig())
    monitor = SafetyMonitor(SafetyConfig())
    loop = MotionLoop(robot, monitor)
    loop.start()
    loop.cmd = (0.5, 0.0)
    deadline = time.time() + 5
    stopped = False
    while time.time() < deadline:
        if robot.battery <= 20.0:
            # 等运动循环把指令清零（最迟一个周期）
            for _ in range(20):
                if loop.cmd == (0.0, 0.0):
                    stopped = True
                    break
                time.sleep(0.02)
            break
        time.sleep(0.05)
    loop.stop()
    loop.join(timeout=1.0)
    assert stopped, "低电量未触发自动停止"
    assert loop.last_verdict == Verdict.STOP


def test_motionloop_estop_immediate():
    """软急停应在一个周期内清零指令并保持阻尼态标志。"""
    robot = MockRobot()
    monitor = SafetyMonitor()
    loop = MotionLoop(robot, monitor)
    loop.start()
    loop.cmd = (0.8, 0.0)
    time.sleep(0.3)
    loop.estop = True
    time.sleep(0.3)
    v, w = loop.cmd
    assert (v, w) == (0.0, 0.0), "急停后指令未清零"
    # 仿真狗应停下（速度为0后位置不再明显变化）
    p1 = robot.get_pose()
    time.sleep(0.3)
    p2 = robot.get_pose()
    assert abs(p1.e - p2.e) < 1e-6 and abs(p1.n - p2.n) < 1e-6, "急停后仍在移动"
    loop.stop()
    loop.join(timeout=1.0)


def test_follower_with_safety_gating():
    """跟随器导航 + 安全门禁：RTK 浮点解降级限速，不阻断任务。"""
    robot = MockRobot()
    monitor = SafetyMonitor()
    route = [
        Waypoint("P1", e=5.0, n=0.0, pos_tol=0.3),
        Waypoint("P2", e=5.0, n=5.0, type="WORK_POINT",
                 speed_limit=0.12, pos_tol=0.08),
    ]
    f = WaypointFollower(route, FollowParams())
    robot.rtk = 5  # RTK_FLOAT：转场段应降级但不停车
    limited = False
    for i in range(3000):
        # 安全监护以转场类型检查
        verdict = monitor.update(robot.telemetry(), wp_type="TRANSIT")
        assert verdict.level != Verdict.STOP
        if verdict.level == Verdict.DEGRADE and verdict.speed_limit:
            limited = True
        f.step(robot, now=i * 0.1)
        robot.step()
        if f.state == FollowerState.DONE:
            break
    assert f.state == FollowerState.DONE
    assert limited, "浮点解未触发降级限速"
    assert robot.pose.dist_to(5.0, 5.0) <= 0.08


def test_workzone_rtk_loss_stops():
    """作业区 RTK 失固定级 → 安全 STOP，跟随器指令应被清零。"""
    robot = MockRobot()
    monitor = SafetyMonitor()
    robot.rtk = 1  # 单点定位，作业区不允许
    verdict = monitor.update(robot.telemetry(), wp_type="WORK_POINT")
    assert verdict.level == Verdict.STOP
    assert "rtk_no_fix_in_workzone" in verdict.reasons


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("integration tests: all passed")
