# -*- coding: utf-8 -*-
"""航点跟随器仿真测试：Mock 机器人走完整路线（含过轨点交接、作业点精度）。"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from common.geo import dist_heading
from nav.follower import Waypoint, WaypointFollower, FollowParams, FollowerState, WpType
from sdk_adapter.mock import MockRobot


def make_route():
    return [
        Waypoint("P01", e=30.0, n=0.0, type="TRANSIT",
                 speed_limit=1.5, pos_tol=0.5),
        Waypoint("P02", e=30.0, n=20.0, type="TRANSIT",
                 speed_limit=1.0, pos_tol=0.4),
        Waypoint("P03", e=30.0, n=22.0, type="CROSS",
                 speed_limit=0.2, pos_tol=0.25),
        Waypoint("P04", e=30.0, n=32.0, type="WORK_APPROACH",
                 speed_limit=0.4, pos_tol=0.3),
        Waypoint("P05", e=30.0, n=35.0, type="WORK_POINT",
                 speed_limit=0.12, pos_tol=0.08,
                 heading=math.pi / 2, yaw_tol=0.1, hold_time=1.0),
        Waypoint("P06", e=0.0, n=35.0, type="HOME",
                 speed_limit=1.5, pos_tol=0.5),
    ]


def run_sim(route, max_steps=6000, dt=0.1):
    robot = MockRobot()
    robot.pose.e, robot.pose.n, robot.pose.yaw = 0.0, 0.0, 0.0
    robot.stand_up()
    f = WaypointFollower(route, FollowParams(ctrl_period=dt))
    cross_count = 0
    states = []
    for i in range(max_steps):
        st = f.step(robot, now=i * dt)
        robot.step()
        if st == FollowerState.CROSS_REACHED:
            cross_count += 1
            # 过轨执行器占位：直接推进（真实实现见 M2）
            f.advance()
            if f.idx >= len(f.wps):
                f.state = FollowerState.DONE
                break
            f.state = FollowerState.CRUISING
        states.append(st)
        if f.state == FollowerState.DONE:
            break
    return robot, f, cross_count, states


def test_full_route():
    robot, f, cross_count, states = run_sim(make_route())
    assert f.state == FollowerState.DONE, "未完成: idx=%s state=%s pos=(%.2f,%.2f)" % (
        f.idx, f.state, robot.pose.e, robot.pose.n)
    assert cross_count == 1, "过轨点应恰好触发一次交接, got %s" % cross_count
    # 作业点精度核验：P05 (30,35) ±0.08
    d, _ = dist_heading(robot.pose.e, robot.pose.n, 30.0, 35.0)  # 已走到HOME，改核验过程
    # 逐点核验在 test_workpoint_precision 中做；这里核验终点 HOME (0,35)
    d_home = robot.pose.dist_to(0.0, 35.0)
    assert d_home <= 0.5, "返航点误差 %s" % d_home


def test_workpoint_precision():
    # 单独仿真到作业点即停，核验到位精度与航向锁定
    route = make_route()[:5]  # 到 P05 为止
    robot = MockRobot()
    f = WaypointFollower(route, FollowParams())
    for i in range(6000):
        st = f.step(robot, now=i * 0.1)
        robot.step()
        if st == FollowerState.CROSS_REACHED:
            f.advance()
            f.state = FollowerState.CRUISING
        if f.state == FollowerState.DONE:
            break
    assert f.state == FollowerState.DONE
    d = robot.pose.dist_to(30.0, 35.0)
    assert d <= 0.08, "作业点位置误差 %.3f m 超差" % d
    yaw_err = abs((robot.pose.yaw - math.pi / 2 + math.pi) % (2 * math.pi) - math.pi)
    assert yaw_err <= 0.1, "作业点航向误差 %.3f rad 超差" % yaw_err


def test_hold_time():
    # hold_time 到点后需保持：连续两个 tick 内不应提前 DONE
    wp = Waypoint("H", e=1.0, n=0.0, pos_tol=0.1, hold_time=2.0)
    robot = MockRobot()
    robot.pose.e, robot.pose.n = 0.99, 0.0
    robot.pose.yaw = 0.0
    f = WaypointFollower([wp], FollowParams())
    st = f.step(robot, now=0.0)
    assert st == FollowerState.CRUISING
    st = f.step(robot, now=1.0)   # 保持未满
    assert st == FollowerState.CRUISING
    st = f.step(robot, now=2.5)   # 保持期满 → DONE
    assert st == FollowerState.DONE


def test_speed_limit_workpoint():
    # 作业点附近速度不得超过 work_speed
    p = FollowParams()
    wp = Waypoint("W", e=0.5, n=0.0, type="WORK_POINT", speed_limit=0.12, pos_tol=0.08)
    robot = MockRobot()
    robot.pose.e, robot.pose.n, robot.pose.yaw = 0.0, 0.0, 0.0
    f = WaypointFollower([wp], p)
    v, w = f.compute_command(robot.pose)
    assert v <= p.work_speed + 1e-9, "作业点速度 %.2f 超限" % v


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("follower tests: all passed")
