# -*- coding: utf-8 -*-
"""SDK 适配层抽象接口。

设计原则：所有灵锶 SDK 调用收敛到 RobotApi 一个接口后面。
上层（航点跟随器/安全监护/上位机）只依赖本文件。

已确认的 SDK 事实（2026-09-30，依据 SDK 指南 v3.0.1 + high_level_base.h）：
- 高层 API（301 TCP 与 v5.0.4 Fast DDS 有差异；当前桥接针对 v5.0.4，均无 Python 绑定）：
  Standup/Getdown/ZeroTorque/Damp/StopMove/Move(lm,vm,lrm)/SetHeight/
  GetOdometry/GetRobotSpeed/GetJointState/GetImuLinearAcceleration/
  GetGpsLocation/GetRobotBatteryPercentage/GetAllAlerts/充电与地图文件接口
- 高层无航点导航/到点判定 → 跟随器自研（本仓库 nav/follower.py）
- 底层关节控制 = ROS1 话题 rt/lowcmd + rt/lowstate（需 Ubuntu 20.04 + ROS1）
- SDK 接口非线程安全 → 实现类内部必须单线程串行调用
- 网络及图传连接参数仅由现场私有配置提供；公开副本不含设备地址
- 诊断话题 report_all_item（17 项运控诊断）→ SafetyMonitor 扩展数据源

★ 标注的方法是 M0 真机验证项（候选接口/指令粒度）。
"""
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class RtkQuality(IntEnum):
    NO_FIX = 0
    GPS = 1
    DGPS = 2
    RTK_FIX = 4
    RTK_FLOAT = 5


@dataclass
class Pose:
    """ENU 平面位姿。e 东(m), n 北(m), yaw 航向(rad, 东=0 逆时针为正)。"""
    e: float = 0.0
    n: float = 0.0
    yaw: float = 0.0

    def dist_to(self, e: float, n: float) -> float:
        return math.hypot(e - self.e, n - self.n)


@dataclass
class Telemetry:
    """一帧遥测数据（安全监护与上位机的输入）。"""
    ts: float = 0.0                       # 本帧时间戳
    pose: Pose = field(default_factory=Pose)
    battery: float = 100.0                # 百分比
    rtk_quality: RtkQuality = RtkQuality.NO_FIX
    speed_cmd: tuple = (0.0, 0.0)         # 最近一次速度指令 (v, w)
    estop: bool = False                   # 遥控器/硬件急停触发


class RobotApi:
    """机器狗控制抽象接口（M0 后由真实 SDK 实现替换 Mock）。"""

    # ---- 模式与基础动作（对应手柄 A键/趴下/阻尼/卸力） ----
    def stand_up(self) -> bool: raise NotImplementedError
    def stand_down(self) -> bool: raise NotImplementedError
    def set_damping(self, on: bool) -> bool: raise NotImplementedError
    def zero_torque(self) -> bool: raise NotImplementedError

    # ---- 运动控制：巡航主通道 ----
    # 已确认高层无定点导航原语；set_velocity 的真实实现 =
    # 10Hz 周期调用 Move(v, 0, w)（Move 为"按速度移动1秒"语义）。
    # M0 实测衔接平滑性，若抖动明显切换 SetSpeed+StopMove 方案。
    def set_velocity(self, v: float, w: float) -> bool:
        """v 前进速度 m/s；w 转向角速度 rad/s。"""
        raise NotImplementedError

    def move_vector(self, v: float, vy: float, w: float) -> bool:
        """含横移的运动指令（精对位"往左挪一点/往右挪一点"用）。
        v 前进, vy 左移（狗系+y）, w 转向。真机实现 = Move(lm=v, vm=vy, lrm=w)。"""
        raise NotImplementedError

    # ★ M0 验证项：SDK 无"地形模式"直接接口（遥控器屏幕概念：常规/节能模式）。
    # 候选映射：'wheel_leg'→SwitchToRLMode 或 Highknee；'crawl'→SwitchToCrawlMode。
    def set_terrain_mode(self, mode: str) -> bool:
        """mode: 'wheel' | 'wheel_leg'（越障/过轨） | 'crawl'。M0 真机逐一验证。"""
        raise NotImplementedError

    # 底层姿态微调 = ROS1 话题 rt/lowcmd（12关节PD指令）；高层粗调用 SetHeight。
    def low_level_body_cmd(self, height: Optional[float] = None,
                           pitch: Optional[float] = None,
                           roll: Optional[float] = None) -> bool:
        """身位高度/俯仰/横滚微调。仅在安全监护放行时允许调用。"""
        raise NotImplementedError

    # ---- 状态读取 ----
    def get_pose(self) -> Pose: raise NotImplementedError
    def get_battery(self) -> float: raise NotImplementedError
    def is_alive(self) -> bool: raise NotImplementedError

    # ---- 定位输入（RTK 串口接入后由 GpsSource 提供，先留桩） ----
    def get_rtk_quality(self) -> RtkQuality:
        return RtkQuality.NO_FIX

    def telemetry(self) -> Telemetry:
        return Telemetry(
            ts=time.time(),
            pose=self.get_pose(),
            battery=self.get_battery(),
            rtk_quality=self.get_rtk_quality(),
        )
