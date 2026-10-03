# -*- coding: utf-8 -*-
"""航点跟随器：RTK 位置反馈闭环 → 速度指令；到点判定（位置+航向双阈值）。

设计为纯逻辑步进函数（compute_command / check_arrival），
便于单元测试；run() 以固定频率驱动真实/mock 机器人。

航点类型（与 route yaml 中 type 字段对应）：
  TRANSIT       转场巡航点
  TRACK_ENTRY   过轨前置点（减速、姿态检查）
  CROSS         过轨点（到达后交由过轨执行器接管，本模块不产生速度指令）
  WORK_APPROACH 作业接近点（低速，进入定位置信 stricter 区）
  WORK_POINT    作业点（最低速，到位后可选锁定目标航向）
  HOME          返航点
"""
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from common.geo import wrap_angle, dist_heading
from sdk_adapter.base import RobotApi, Pose


class WpType(str, Enum):
    TRANSIT = "TRANSIT"
    TRACK_ENTRY = "TRACK_ENTRY"
    CROSS = "CROSS"
    WORK_APPROACH = "WORK_APPROACH"
    WORK_POINT = "WORK_POINT"
    HOME = "HOME"


@dataclass
class Waypoint:
    name: str
    e: float                    # ENU 东坐标 m
    n: float                    # ENU 北坐标 m
    type: str = WpType.TRANSIT.value
    speed_limit: float = 1.0    # 本段最大速度 m/s
    pos_tol: float = 0.3        # 到点位置阈值 m
    yaw_tol: float = 0.35       # 到点航向阈值 rad（heading_required=True 时生效）
    heading: Optional[float] = None   # 目标航向 rad；None 表示不要求终点航向
    hold_time: float = 0.0      # 到点后保持时间 s（RTK 平均/拍照用）


@dataclass
class FollowParams:
    k_v: float = 0.6            # 速度增益：v ≈ k_v * 距离
    k_w: float = 1.8            # 转向增益
    v_min: float = 0.06         # 最小前进速度（低于则给 0，防蠕动）
    v_max: float = 1.5          # 全局限速（转场）
    slow_speed: float = 0.4     # 接近段限速
    work_speed: float = 0.12    # 作业点限速
    heading_slow_ratio: float = 0.5  # 航向误差>30° 时按比例降速
    ctrl_period: float = 0.1    # 控制周期 s（10 Hz）


class FollowerState(str, Enum):
    IDLE = "IDLE"
    CRUISING = "CRUISING"
    HOLDING = "HOLDING"          # 到点保持中
    CROSS_REACHED = "CROSS_REACHED"  # 抵达过轨点，等待过轨执行器
    WP_DONE = "WP_DONE"
    DONE = "DONE"                # 全部航点完成


class WaypointFollower:
    def __init__(self, waypoints: List[Waypoint], params: FollowParams = None):
        if not waypoints:
            raise ValueError("waypoints 为空")
        self.wps = waypoints
        self.p = params or FollowParams()
        self.idx = 0
        self.state = FollowerState.IDLE
        self._hold_until = 0.0

    @property
    def current(self) -> Waypoint:
        return self.wps[self.idx]

    def reset(self):
        self.idx = 0
        self.state = FollowerState.IDLE
        self._hold_until = 0.0

    # ---- 到点判定：位置 (+可选航向) 双阈值 + 保持时间 ----
    def check_arrival(self, pose: Pose, now: float = None) -> bool:
        wp = self.current
        d = pose.dist_to(wp.e, wp.n)
        if d > wp.pos_tol:
            self._hold_until = 0.0
            return False
        if wp.heading is not None:
            yaw_err = abs(wrap_angle(wp.heading - pose.yaw))
            if yaw_err > wp.yaw_tol:
                self._hold_until = 0.0
                return False
        now = now if now is not None else time.time()
        if self._hold_until == 0.0:
            self._hold_until = now + wp.hold_time
        if now >= self._hold_until:
            self._hold_until = 0.0
            return True
        return False

    # ---- 控制律：跟随器核心，返回 (v, w) ----
    def compute_command(self, pose: Pose):
        wp = self.current
        d, bearing = dist_heading(pose.e, pose.n, wp.e, wp.n)

        # 目标航向：行进中朝向目标点；heading_required 且接近时转向目标航向
        if wp.heading is not None and d < max(1.0, 3 * wp.pos_tol):
            target_yaw = wp.heading
        else:
            target_yaw = bearing
        yaw_err = wrap_angle(target_yaw - pose.yaw)

        # 分段限速
        v_lim = min(self.p.v_max, wp.speed_limit)
        if wp.type in (WpType.WORK_APPROACH.value, WpType.WORK_POINT.value):
            v_lim = min(v_lim, self.p.slow_speed)
        if wp.type == WpType.WORK_POINT.value:
            v_lim = min(v_lim, self.p.work_speed)

        v = self.p.k_v * d
        if abs(yaw_err) > math.radians(30):
            v *= self.p.heading_slow_ratio
        v = max(0.0, min(v_lim, v))
        if d < wp.pos_tol and v < self.p.v_min:
            v = 0.0
        w = max(-1.0, min(1.0, self.p.k_w * yaw_err))
        return v, w

    # ---- 单步推进：由外部定时调用 ----
    def step(self, robot: RobotApi, now: float = None):
        """返回 FollowerState。CROSS_REACHED 时由调用方执行过轨动作后
        调用 advance() 继续。"""
        pose = robot.get_pose()
        if self.state == FollowerState.IDLE:
            self.state = FollowerState.CRUISING

        if self.check_arrival(pose, now):
            wp = self.current
            if wp.type == WpType.CROSS.value:
                self.state = FollowerState.CROSS_REACHED
                return self.state
            self.advance()
            if self.idx >= len(self.wps):
                self.state = FollowerState.DONE
                robot.set_velocity(0.0, 0.0)
                return self.state
            self.state = FollowerState.CRUISING

        v, w = self.compute_command(pose)
        robot.set_velocity(v, w)
        return self.state

    def advance(self):
        """跳过当前航点（含过轨完成后由外部调用）。"""
        self.idx += 1
