# -*- coding: utf-8 -*-
"""灵锶高层 SDK（TCP 路线）的 Python 适配层。

依赖：sdk_adapter/cpp 编译出的 `lingsi` pybind 模块（包装厂商 libhigh_level_remote_tcp_client.so）。
SDK 接口非线程安全 → 本类内部全程持锁串行调用。
指令粒度：Move(lm, vm, lrm) 为"按速度移动 1 秒"，跟随器/遥控以 ~10Hz 重复调用。

用法：
    from sdk_adapter.lingsi_tcp import LingSiTcpRobot
    robot = LingSiTcpRobot()          # 须先独立完成 SDK 版本与现场网络配置验证
"""
import math
import threading
import time
from typing import Optional

try:
    import lingsi as _lingsi
except ImportError:
    _lingsi = None

from .base import RobotApi, Pose, Telemetry, RtkQuality


class SDKNotBuilt(RuntimeError):
    """lingsi 桥接模块未编译。见 sdk_adapter/cpp/build_bridge.sh。"""


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 → 偏航角（绕Z）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LingSiTcpRobot(RobotApi):
    """真机适配层。所有 SDK 调用经 _lock 串行化（SDK 非线程安全）。"""

    def __init__(self):
        if _lingsi is None:
            raise SDKNotBuilt(
                "lingsi 模块未编译：cd sdk_adapter/cpp && ./build_bridge.sh")
        with self._class_lock():
            ret = _lingsi.init()
        if ret != 0:
            raise RuntimeError("SDK init 失败，返回码 %s（检查现场私有网络与 SDK 版本）" % ret)
        # telemetry() 会组合调用同样取锁的状态读取方法。
        self._lock = threading.RLock()
        self._cmd = (0.0, 0.0)
        self._last_cmd_ts = 0.0
        self.estop = False

    @staticmethod
    def _class_lock():
        return _CLASS_LOCK

    def _call(self, fn, *args):
        with self._lock:
            return fn(*args)

    # ---- 模式与基础动作（对应手柄：站立A键/趴下/阻尼/卸力） ----
    def stand_up(self):
        return self._call(_lingsi.standup) == 0

    def stand_down(self):
        return self._call(_lingsi.getdown) == 0

    def set_damping(self, on: bool):
        # SDK 只有"进入阻尼"指令（Damp），无退出参数；恢复运动需站立
        return self._call(_lingsi.damp) == 0 if on else self.stand_up()

    def zero_torque(self):
        return self._call(_lingsi.zero_torque) == 0

    # ---- 运动控制 ----
    def set_velocity(self, v: float, w: float):
        """10Hz 语义：每次调用持续约 1 秒，调用方按周期刷新。
        v 前进 m/s（限 ±1.5），w 转向 rad/s（限 ±1.0）。"""
        v = max(-1.5, min(1.5, v))
        w = max(-1.0, min(1.0, w))
        self._cmd = (v, w)
        self._last_cmd_ts = time.time()
        # Move(lm 前后, vm 左右, lrm 转向)；横移暂不用（沿轨道巡航不需要）
        return self._call(_lingsi.move_1s, v, 0.0, w) == 0

    def stop(self):
        self._cmd = (0.0, 0.0)
        return self._call(_lingsi.stop_move) == 0

    # ★ M0 验证项：过轨步态候选接口
    def set_terrain_mode(self, mode: str):
        if mode == "wheel_leg_rl":
            return self._call(_lingsi.switch_rl_mode, True) == 0
        if mode == "high_knee":
            return self._call(_lingsi.high_knee, True) == 0
        if mode == "crawl":
            return self._call(_lingsi.switch_crawl_mode, True) == 0
        return self._call(_lingsi.stop_move) == 0  # 'wheel'：恢复常规（占位）

    # 底层调姿（rt/lowcmd）走独立 ROS1 节点，见 M2/M3；高层粗调高度：
    def set_body_height(self, height_m: float):
        return self._call(_lingsi.set_height, float(height_m)) == 0

    # ---- 状态读取 ----
    def get_pose(self) -> Pose:
        odo = self._call(_lingsi.get_odometry)
        pos = odo.get("position", [0.0, 0.0, 0.0])
        q = odo.get("orientation", [0.0, 0.0, 0.0, 1.0])
        # 注意：里程计坐标系为狗自身上电原点，非 ENU/RTK 真值。
        # 跟随器使用时以 RTK(双天线航向) 为准，此接口供诊断/无RTK降级。
        return Pose(e=pos[0], n=pos[1], yaw=_quat_to_yaw(q[0], q[1], q[2], q[3]))

    def get_battery(self) -> float:
        return float(self._call(_lingsi.get_battery))

    def is_alive(self) -> bool:
        try:
            self._call(_lingsi.get_sdk_version)
            return True
        except Exception:
            return False

    def get_rtk_quality(self) -> RtkQuality:
        # 真实质量来自外置 RTK(串口NMEA)，见 common/rtk.py；此处返回占位
        return RtkQuality.NO_FIX

    def telemetry(self) -> Telemetry:
        with self._lock:
            tel = Telemetry(
                ts=time.time(),
                pose=self.get_pose(),
                battery=self.get_battery(),
                rtk_quality=self.get_rtk_quality(),
                speed_cmd=self._cmd,
                estop=self.estop,
            )
        return tel

    # ---- 诊断 ----
    def alerts(self):
        return self._call(_lingsi.get_all_alerts)

    def gps(self) -> dict:
        return self._call(_lingsi.get_gps)

    def feed_heartbeat(self, age_s: float) -> bool:
        """供失联检测：最近一次速度指令距今是否超时。"""
        return (time.time() - self._last_cmd_ts) <= age_s


_CLASS_LOCK = threading.Lock()
