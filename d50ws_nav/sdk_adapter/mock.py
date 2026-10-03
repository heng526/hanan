# -*- coding: utf-8 -*-
"""Mock 仿真实现：无真机/无 SDK 时驱动跟随器与安全监护的开发测试。

运动学：差速简化模型（v 前进 + w 转向），步长 step() 推进。
真实 D50WS 的全向能力（横移）在跟随器中未使用，故仿真不影响结论。
"""
import math
import time

from .base import RobotApi, Pose, Telemetry, RtkQuality


class MockRobot(RobotApi):
    def __init__(self, pose: Pose = None, dt: float = 0.1):
        self.pose = pose or Pose()
        self.dt = dt
        self.battery = 100.0
        self.rtk = RtkQuality.RTK_FIX
        self.estop = False
        self._cmd = (0.0, 0.0)
        self._cmd_vy = 0.0
        self.alive = True
        self.log = []

    def set_body_height(self, h):
        self.log.append("height=%.2f" % h)
        return True

    # ---- 模式与基础动作 ----
    def stand_up(self):
        self.log.append("stand_up")
        return True

    def stand_down(self):
        self.log.append("stand_down")
        return True

    def set_damping(self, on):
        self.log.append("damping=%s" % on)
        return True

    def zero_torque(self):
        self.log.append("zero_torque")
        return True

    # ---- 运动控制 ----
    def set_velocity(self, v, w):
        return self.move_vector(v, 0.0, w)

    def move_vector(self, v, vy, w):
        v = max(-1.5, min(1.5, v))
        vy = max(-0.5, min(0.5, vy))
        w = max(-1.0, min(1.0, w))
        self._cmd = (v, w)
        self._cmd_vy = vy
        return True

    def set_terrain_mode(self, mode):
        self.log.append("terrain=%s" % mode)
        return True

    def low_level_body_cmd(self, height=None, pitch=None, roll=None):
        self.log.append("body(h=%s,p=%s,r=%s)" % (height, pitch, roll))
        return True

    # ---- 仿真推进 ----
    def step(self):
        from common.geo import wrap_angle
        v, w = self._cmd
        vy = getattr(self, "_cmd_vy", 0.0)
        self.pose.yaw = wrap_angle(self.pose.yaw + w * self.dt)
        cos_y, sin_y = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        self.pose.e += (v * cos_y - vy * sin_y) * self.dt
        self.pose.n += (v * sin_y + vy * cos_y) * self.dt
        self.battery = max(0.0, self.battery - 0.0005)

    # ---- 状态读取 ----
    def get_pose(self):
        return Pose(self.pose.e, self.pose.n, self.pose.yaw)

    def get_battery(self):
        return self.battery

    def is_alive(self):
        return self.alive

    def get_rtk_quality(self):
        return self.rtk

    def telemetry(self):
        return Telemetry(
            ts=time.time(),
            pose=self.get_pose(),
            battery=self.battery,
            rtk_quality=self.rtk,
            speed_cmd=self._cmd,
            estop=self.estop,
        )
