# -*- coding: utf-8 -*-
"""安全监护：失联超时 / 电量 / RTK 置信 / 地理围栏 / 急停。

判定分级：
  RUN      正常，按当前任务速度行驶
  DEGRADE  降级（限低速），如 RTK 浮点解
  STOP     立即停车（由调用方执行 set_velocity(0,0) 并进入待命/急停流程）
"""
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from sdk_adapter.base import Telemetry, RtkQuality


class Verdict(str, Enum):
    RUN = "RUN"
    DEGRADE = "DEGRADE"
    STOP = "STOP"


@dataclass
class SafetyConfig:
    heartbeat_timeout_s: float = 1.0     # SDK/狗失联阈值
    battery_min: float = 20.0            # 低电量停车阈值 %
    battery_warn: float = 30.0           # 低电量降级阈值 %
    rtk_fix_required_types: tuple = ("WORK_APPROACH", "WORK_POINT", "CROSS")
    geofence_center: tuple = (0.0, 0.0)  # ENU (e, n)
    geofence_radius: float = 2000.0      # m，作业区半径
    max_speed_degrade: float = 0.3       # DEGRADE 时的限速 m/s


@dataclass
class SafetyVerdict:
    level: Verdict
    reasons: List[str] = field(default_factory=list)
    speed_limit: Optional[float] = None  # 非 None 时覆盖当前速度上限


class SafetyMonitor:
    def __init__(self, cfg: SafetyConfig = None):
        self.cfg = cfg or SafetyConfig()

    def update(self, tel: Telemetry, wp_type: str = "TRANSIT",
               now: float = None) -> SafetyVerdict:
        cfg = self.cfg
        now = now if now is not None else time.time()
        reasons: List[str] = []

        # 1) 急停最高优先级
        if tel.estop:
            return SafetyVerdict(Verdict.STOP, ["estop"])

        # 2) 失联
        if tel.ts <= 0 or (now - tel.ts) > cfg.heartbeat_timeout_s:
            reasons.append("heartbeat_timeout")

        # 3) 电量
        if tel.battery <= cfg.battery_min:
            reasons.append("battery_low")
        elif tel.battery <= cfg.battery_warn:
            reasons.append("battery_warn")

        # 4) RTK 置信分级
        q = tel.rtk_quality
        speed_limit = None
        if wp_type in cfg.rtk_fix_required_types:
            if q != RtkQuality.RTK_FIX:
                reasons.append("rtk_no_fix_in_workzone")
        else:
            if q == RtkQuality.RTK_FLOAT:
                speed_limit = cfg.max_speed_degrade
                reasons.append("rtk_float")
            elif q in (RtkQuality.NO_FIX, RtkQuality.GPS, RtkQuality.DGPS) and \
                    q != RtkQuality.RTK_FIX:
                speed_limit = cfg.max_speed_degrade
                reasons.append("rtk_weak")

        # 5) 地理围栏
        e0, n0 = cfg.geofence_center
        if (tel.pose.e - e0) ** 2 + (tel.pose.n - n0) ** 2 > cfg.geofence_radius ** 2:
            reasons.append("geofence_violation")

        if not reasons:
            return SafetyVerdict(Verdict.RUN, [], None)
        hard = {"estop", "heartbeat_timeout", "battery_low",
                "rtk_no_fix_in_workzone", "geofence_violation"}
        if hard & set(reasons):
            return SafetyVerdict(Verdict.STOP, reasons, 0.0)
        return SafetyVerdict(Verdict.DEGRADE, reasons, speed_limit)
