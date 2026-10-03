# -*- coding: utf-8 -*-
"""geo 模块单元测试：GGA 解析与 ENU 投影。可直接 python 运行或 pytest。"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from common.geo import parse_gga, ENUProjector, wrap_angle, dist_heading, nmea_to_deg


def test_nmea_to_deg():
    assert abs(nmea_to_deg(3000.1234, "N") - (30 + 0.1234 / 60)) < 1e-9
    assert abs(nmea_to_deg(11000.5, "E") - (110 + 0.5 / 60)) < 1e-9
    assert nmea_to_deg(3000.0, "S") < 0
    assert nmea_to_deg(11000.0, "W") < 0


def test_parse_gga_rtk_fix():
    # 典型 RTK fix 语句（千寻 NTRIP 差分后）
    s = ("$GNGGA,060521.00,3000.0000000,N,11000.0000000,E,4,12,"
         "0.6,150.3,M,7.0,M,3.2,0123*4A")
    r = parse_gga(s)
    assert r is not None
    assert r["quality"] == 4
    assert abs(r["lat"] - 30.0) < 1e-9
    assert abs(r["lon"] - 110.0) < 1e-9
    assert r["num_sats"] == 12
    assert abs(r["altitude_m"] - 150.3) < 1e-6


def test_parse_gga_invalid():
    assert parse_gga("$GNGGA,060521.00,,,,,0,00,99.99,,,,,,*78") is None
    assert parse_gga("garbage") is None
    assert parse_gga("$GPRMC,123456,A,3000.0,N,11000.0,E*XX") is None  # 非GGA


def test_enu_projection():
    # 原点取合成坐标参考点，向东/北各偏移应有对应米数
    origin_lat, origin_lon = 30.0, 110.0
    proj = ENUProjector(origin_lat, origin_lon)
    # 东向偏移：经度增加约 100m / (111320*cos(30.0°)) 度
    d_lon = 100.0 / (111320.0 * math.cos(math.radians(origin_lat)))
    e, n = proj.to_enu(origin_lat, origin_lon + d_lon)
    assert abs(e - 100.0) < 1.0, "east err %s" % e
    assert abs(n) < 1.0, "north err %s" % n
    # 北向偏移：纬度增加约 100m / 110940 度
    d_lat = 100.0 / 110940.0
    e2, n2 = proj.to_enu(origin_lat + d_lat, origin_lon)
    assert abs(e2) < 1.0
    assert abs(n2 - 100.0) < 1.0, "north err %s" % n2


def test_enu_roundtrip():
    proj = ENUProjector(30.0, 110.0)
    e, n = proj.to_enu(30.001, 110.001)
    lat, lon = proj.llh(e, n)
    assert abs(lat - 30.001) < 1e-4
    assert abs(lon - 110.001) < 1e-4


def test_wrap_and_dist():
    assert abs(wrap_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-9
    d, brg = dist_heading(0, 0, 100, 100)
    assert abs(d - math.hypot(100, 100)) < 1e-9
    assert abs(brg - math.pi / 4) < 1e-9


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("geo tests: all passed")
