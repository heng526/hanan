# -*- coding: utf-8 -*-
"""地理坐标工具：NMEA-0183 GGA 解析 + WGS84 → ENU 站心平面投影。

编组站航点巡航在 ENU 平面直角坐标系下计算距离与航向，
原点取作业区参考点（现场示教时确定一次，全站共用）。
"""
import math
from dataclasses import dataclass
from typing import Optional


def nmea_to_deg(value: float, hemi: str) -> float:
    """NMEA 经纬度（ddmm.mmmm）转十进制度数。hemi: N/S/E/W"""
    deg = int(value // 100)
    minutes = value - deg * 100
    result = deg + minutes / 60.0
    if hemi in ("S", "W"):
        result = -result
    return result


def parse_gga(sentence: str) -> Optional[dict]:
    """解析 $..GGA 语句。定位无效（quality=0）时返回字段但 quality=0。

    返回字段: time, lat, lon, quality(0无效/1单点/2差分/4RTK固定/5RTK浮点),
              num_sats, hdop, altitude_m
    """
    s = sentence.strip()
    if not s.startswith("$"):
        return None
    body = s[1:].split("*")[0]
    fields = body.split(",")
    if len(fields) < 12 or not fields[0].endswith("GGA"):
        return None

    def _f(idx):
        try:
            return float(fields[idx])
        except (ValueError, IndexError):
            return 0.0

    lat_raw, ns = _f(2), fields[3] if len(fields) > 3 else ""
    lon_raw, ew = _f(4), fields[5] if len(fields) > 5 else ""
    if lat_raw == 0 or lon_raw == 0:
        return None
    quality = int(_f(6))
    return {
        "time": fields[1],
        "lat": nmea_to_deg(lat_raw, ns or "N"),
        "lon": nmea_to_deg(lon_raw, ew or "E"),
        "quality": quality,
        "num_sats": int(_f(7)),
        "hdop": _f(8),
        "altitude_m": _f(9),
    }


# WGS84 椭球参数
_WGS84_A = 6378137.0            # 长半轴 m
_WGS84_F = 1.0 / 298.257223563  # 扁率
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h: float = 0.0):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + h) * math.cos(lat) * math.cos(lon)
    y = (n + h) * math.cos(lat) * math.sin(lon)
    z = (n * (1 - _WGS84_E2) + h) * math.sin(lat)
    return x, y, z


class ENUProjector:
    """以 (origin_lat, origin_lon) 为原点的 ENU 东-北-天平面投影。"""

    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt
        self._o = geodetic_to_ecef(origin_lat, origin_lon, origin_alt)
        lat = math.radians(origin_lat)
        lon = math.radians(origin_lon)
        self._m = (
            (-math.sin(lon), math.cos(lon), 0.0),
            (-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)),
            (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)),
        )

    def to_enu(self, lat_deg: float, lon_deg: float, h: float = 0.0):
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, h)
        dx, dy, dz = x - self._o[0], y - self._o[1], z - self._o[2]
        m = self._m
        return (
            m[0][0] * dx + m[0][1] * dy + m[0][2] * dz,  # East
            m[1][0] * dx + m[1][1] * dy + m[1][2] * dz,  # North
        )

    def llh(self, e: float, n: float):
        """ENU 平面坐标反投影回经纬度（迭代法，用于航点导出/核对）。"""
        # 由 ENU 小范围线性反解（编组站尺度 <2km，一次迭代足够）
        lat0 = math.radians(self.origin_lat)
        lon0 = math.radians(self.origin_lon)
        m_lat = 111132.92 - 559.82 * math.cos(2 * lat0) + 1.175 * math.cos(4 * lat0)
        p = _WGS84_A * math.cos(lat0)
        m_lon = p * (math.pi / 180) * 180 / math.pi
        m_lon = 111412.84 * math.cos(lat0) - 93.5 * math.cos(3 * lat0)
        lat_deg = self.origin_lat + n / m_lat
        lon_deg = self.origin_lon + e / (m_lon if m_lon > 0 else 1e-6)
        return lat_deg, lon_deg


def wrap_angle(a: float) -> float:
    """角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def dist_heading(cur_e: float, cur_n: float, tgt_e: float, tgt_n: float):
    """当前位置指向目标位置的距离(m)与方位角(rad，东为零逆时针为正即数学系)。"""
    de, dn = tgt_e - cur_e, tgt_n - cur_n
    return math.hypot(de, dn), math.atan2(dn, de)
