# -*- coding: utf-8 -*-
"""RTK 模块测试：内存 NMEA 源 → GGA 状态维护 → fix 判定 → ENU 输出。

不落盘：ReplayNmeaSource 的文件读取逻辑极薄（逐行返回），
核心逻辑（GGA 解析/状态维护/fix 判定/ENU 流水线）用内存源覆盖。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from common.geo import ENUProjector
from common.rtk import NmeaSource, RtkTracker

# 样例（合成坐标）：fix → fix(北移约60m) → float
LINES = [
    "$GNGGA,060500.00,3000.0000000,N,11000.0000000,E,4,12,0.6,150.3,M,7.0,M,3.2,0123*6D",
    "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74",  # 非GGA
    "$GNGGA,060501.00,3000.0324000,N,11000.0000000,E,4,12,0.6,150.3,M,7.0,M,3.2,0123*69",
    "broken line without dollar",                                            # 坏行
    "$GNGGA,060502.00,3000.0648000,N,11000.0000000,E,5,12,0.6,150.3,M,7.0,M,3.2,0123*64",
]


class ListNmeaSource(NmeaSource):
    """内存行数据源（测试用，行为等同 ReplayNmeaSource 的非循环模式）。"""

    def __init__(self, lines):
        self._lines = list(lines)
        self._idx = 0

    def read_line(self, timeout: float = 1.0):
        if self._idx >= len(self._lines):
            return None
        line = self._lines[self._idx]
        self._idx += 1
        return line


def test_replay_tracker():
    tracker = RtkTracker(ListNmeaSource(LINES))
    got = []
    for _ in range(10):
        gga = tracker.poll(timeout=0.05)
        if gga:
            got.append(gga)
    assert len(got) == 3, "应解析出3条GGA, got %d" % len(got)
    assert got[0]["quality"] == 4
    assert got[2]["quality"] == 5        # 第三条是浮点解
    assert not tracker.is_fix(), "最后一条为浮点解，is_fix 应为 False"
    assert tracker.gga_count == 3


def test_rtk_to_enu_pipeline():
    """GGA → ENU：模拟接收机沿北向移动，验证位置流。"""
    proj = ENUProjector(30.0, 110.0)
    tracker = RtkTracker(ListNmeaSource(LINES))
    enus = []
    for _ in range(20):
        gga = tracker.poll(timeout=0.05)
        if gga:
            e, n = proj.to_enu(gga["lat"], gga["lon"])
            enus.append((e, n, gga["quality"]))
    assert len(enus) == 3
    # 纬度增加 ≈ 北向移动：0.0005394° ≈ 60m（第二条 vs 第一条）
    dn = enus[1][1] - enus[0][1]
    assert 55 < dn < 65, "北向位移 %.1f m 不合理" % dn
    assert abs(enus[0][0]) < 1.0         # 经度不变 → 东向≈0
    # 第三条变 float 解，作业区应拒用
    assert enus[2][2] == 5


def test_stale_position_rejected():
    """3 秒无新 GGA → position() 应返回 None（防陈旧定位）。"""
    tracker = RtkTracker(ListNmeaSource(LINES))
    for _ in range(10):
        tracker.poll(timeout=0.05)
    tracker.last_ts = 0.0                # 人为置陈旧
    assert tracker.position() is None
    assert not tracker.is_fix()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("rtk tests: all passed")
