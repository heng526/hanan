# -*- coding: utf-8 -*-
"""感知层范式测试：常驻线程 + enable/disable 锁开关 + 非阻塞 read。"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from perception.base_perception import BasePerception
from perception.mock_lever import MockLeverDetector


def test_disabled_consumes_no_frames():
    p = MockLeverDetector(period=0.02)
    p.start()
    time.sleep(0.3)
    assert p.read()["frame_count"] == 0, "未 enable 不应消耗帧"
    assert not p.enabled


def test_enable_starts_inference():
    p = MockLeverDetector(period=0.02)
    p.start()
    p.enable()
    time.sleep(0.5)
    st = p.read()
    assert st["frame_count"] > 5, "enable 后应持续推理"
    assert st["track_status"] == "TRACKING"
    assert st["target_detected"]
    assert abs(st["target_point_dog"][2] - 0.9) < 1e-6   # 狗系 z=0.9m


def test_disable_suspends():
    p = MockLeverDetector(period=0.02)
    p.start()
    p.enable()
    time.sleep(0.3)
    n1 = p.read()["frame_count"]
    p.disable()
    time.sleep(0.3)
    n2 = p.read()["frame_count"]
    assert n1 > 0
    assert n2 - n1 <= 1, "disable 后帧数应停止增长"
    p.close()


def test_error_code_before_target():
    p = MockLeverDetector(period=0.02, appear_after_frames=100)
    p.start()
    p.enable()
    time.sleep(0.3)
    st = p.read()
    assert st["target_detected"] is False
    assert st["error_code"] == "E_TARGET_NOT_FOUND"
    p.close()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("perception tests: all passed")
