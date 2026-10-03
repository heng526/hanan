# -*- coding: utf-8 -*-
"""安全监护单元测试。"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nav.safety import SafetyMonitor, SafetyConfig, Verdict
from sdk_adapter.base import Telemetry, Pose, RtkQuality


def mk_tel(**kw):
    d = dict(ts=time.time(), pose=Pose(0, 0, 0), battery=80.0,
             rtk_quality=RtkQuality.RTK_FIX, estop=False)
    d.update(kw)
    return Telemetry(**d)


def test_all_normal():
    m = SafetyMonitor()
    v = m.update(mk_tel(), wp_type="TRANSIT")
    assert v.level == Verdict.RUN, v.reasons


def test_estop_first():
    m = SafetyMonitor()
    v = m.update(mk_tel(estop=True, battery=10.0), wp_type="TRANSIT")
    assert v.level == Verdict.STOP
    assert v.reasons == ["estop"]


def test_heartbeat_timeout():
    m = SafetyMonitor()
    v = m.update(mk_tel(ts=time.time() - 2.0), wp_type="TRANSIT", now=time.time())
    assert v.level == Verdict.STOP
    assert "heartbeat_timeout" in v.reasons


def test_battery_levels():
    m = SafetyMonitor()
    v = m.update(mk_tel(battery=25.0))
    assert v.level == Verdict.DEGRADE and "battery_warn" in v.reasons
    v = m.update(mk_tel(battery=15.0))
    assert v.level == Verdict.STOP and "battery_low" in v.reasons


def test_rtk_in_workzone():
    m = SafetyMonitor()
    # 作业区必须 RTK fix
    v = m.update(mk_tel(rtk_quality=RtkQuality.RTK_FLOAT), wp_type="WORK_POINT")
    assert v.level == Verdict.STOP and "rtk_no_fix_in_workzone" in v.reasons
    # 转场段浮点解 → 降级限速
    v = m.update(mk_tel(rtk_quality=RtkQuality.RTK_FLOAT), wp_type="TRANSIT")
    assert v.level == Verdict.DEGRADE
    assert v.speed_limit == m.cfg.max_speed_degrade
    # 转场段单点定位 → 同样降级
    v = m.update(mk_tel(rtk_quality=RtkQuality.GPS), wp_type="TRANSIT")
    assert v.level == Verdict.DEGRADE


def test_geofence():
    cfg = SafetyConfig(geofence_center=(0, 0), geofence_radius=100.0)
    m = SafetyMonitor(cfg)
    v = m.update(mk_tel(pose=Pose(150, 0, 0)))
    assert v.level == Verdict.STOP and "geofence_violation" in v.reasons
    v = m.update(mk_tel(pose=Pose(50, 0, 0)))
    assert v.level == Verdict.RUN


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("safety tests: all passed")
