# -*- coding: utf-8 -*-
"""RTK 接入：NMEA-0183 串口读取 + NTRIP 差分数据客户端。

组成：
  NmeaSource   —— 数据源抽象（串口实流 / 文件回放 / 手动注入），输出 GGA dict
  SerialNmeaSource —— pyserial 串口源（RTK 接收机，M1 现场用）
  ReplayNmeaSource —— 文件回放源（无硬件开发/测试）
  NtripClient  —— 千寻等 NTRIP caster 客户端：发送 NMEA-GGA 心跳，
                  接收 RTCM 差分数据并回写接收机串口（差分 fix 的前提）

依赖：pyserial（仅 SerialNmeaSource 需要；回放模式零依赖）
"""
import socket
import threading
import time
from typing import Callable, Optional

from .geo import parse_gga


class NmeaSource:
    """NMEA 行数据源接口。read_line() 返回一行（含 $ 开头），无数据返回 None。"""

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        raise NotImplementedError

    def close(self):
        pass


class ReplayNmeaSource(NmeaSource):
    """从文本文件回放 NMEA（每行一条，# 开头为注释）。"""

    def __init__(self, path: str, loop: bool = False):
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            self._lines = [l.strip() for l in f
                           if l.strip() and not l.strip().startswith("#")
                           and l.strip().startswith("$")]
        if not self._lines:
            raise ValueError("回放文件无 NMEA 语句: %s" % path)
        self._idx = 0
        self._loop = loop

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        if self._idx >= len(self._lines):
            if not self._loop:
                return None
            self._idx = 0
        line = self._lines[self._idx]
        self._idx += 1
        return line


class SerialNmeaSource(NmeaSource):
    """pyserial 串口源。"""

    def __init__(self, port: str, baud: int = 115200):
        try:
            import serial  # pyserial
        except ImportError as e:
            raise RuntimeError("需要 pyserial: pip install pyserial") from e
        self._ser = serial.Serial(port, baud, timeout=1.0)

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        self._ser.timeout = timeout
        raw = self._ser.readline()
        if not raw:
            return None
        try:
            return raw.decode("ascii", errors="replace").strip()
        except Exception:
            return None

    def write(self, data: bytes):
        """回写差分数据（RTCM）给接收机。"""
        self._ser.write(data)

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


class RtkTracker:
    """从 NMEA 源持续提取 GGA，维护最新定位状态。

    quality: 0无效/1单点/2差分/4RTK固定/5RTK浮点（对应 RtkQuality 枚举值）
    """

    def __init__(self, source: NmeaSource):
        self.source = source
        self.last_gga: Optional[dict] = None
        self.last_ts: float = 0.0
        self.gga_count = 0

    def poll(self, timeout: float = 1.0) -> Optional[dict]:
        """读一行；是 GGA 就更新状态并返回。"""
        line = self.source.read_line(timeout)
        if not line:
            return None
        gga = parse_gga(line)
        if gga:
            self.last_gga = gga
            self.last_ts = time.time()
            self.gga_count += 1
        return gga

    def age_s(self) -> float:
        return time.time() - self.last_ts if self.last_ts else float("inf")

    def is_fix(self) -> bool:
        return bool(self.last_gga) and self.last_gga["quality"] == 4 and self.age_s() < 3.0

    def position(self):
        """返回 (lat, lon, quality) 或 None。"""
        if self.last_gga and self.age_s() < 3.0:
            return self.last_gga["lat"], self.last_gga["lon"], self.last_gga["quality"]
        return None


class NtripClient(threading.Thread):
    """NTRIP caster 客户端（千寻知采/.findElement 等网络 RTK 服务）。

    差分数据到达后回调 on_rtcm(bytes)，由调用方写入接收机串口。
    GGA 心跳按 caster 要求周期上报（通常 1~60s，千寻要求 ≤60s）。
    """

    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str, passwd: str,
                 on_rtcm: Callable[[bytes], None],
                 gga_provider: Callable[[], Optional[str]],
                 gga_period_s: float = 10.0):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.mount, self.user, self.passwd = mountpoint, user, passwd
        self.on_rtcm = on_rtcm
        self.gga_provider = gga_provider     # 返回完整 $GPGGA 语句字符串
        self.gga_period_s = gga_period_s
        self._stop = threading.Event()
        self.connected = False
        self.bytes_received = 0

    def _connect_request(self) -> bytes:
        auth = "%s:%s" % (self.user, self.passwd)
        import base64
        token = base64.b64encode(auth.encode()).decode()
        req = ("GET /%s HTTP/1.1\r\n" % self.mount
               + "User-Agent: NTRIP d50ws_nav/0.1\r\n"
               + "Authorization: Basic %s\r\n" % token
               + "Ntrip-Version: Ntrip/2.0\r\n\r\n")
        return req.encode()

    def run(self):
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=10)
                sock.sendall(self._connect_request())
                # 读响应头直到空行
                buf = b""
                while b"\r\n\r\n" not in buf:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("NTRIP 无响应")
                    buf += chunk
                head = buf.split(b"\r\n\r\n")[0].decode(errors="replace")
                if "200" not in head.split("\r\n")[0]:
                    raise ConnectionError("NTRIP 拒绝: %s" % head.split("\r\n")[0][:120])
                self.connected = True
                rest = buf.split(b"\r\n\r\n", 1)[1]
                if rest:
                    self._emit(rest)
                last_gga = 0.0
                sock.settimeout(1.0)
                while not self._stop.is_set():
                    # 周期发 GGA 心跳
                    if time.time() - last_gga >= self.gga_period_s:
                        gga = self.gga_provider()
                        if gga:
                            sock.sendall((gga.strip() + "\r\n").encode())
                        last_gga = time.time()
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("NTRIP 连接断开")
                    self._emit(chunk)
            except Exception as e:
                self.connected = False
                print("[ntrip] 连接异常(10s后重试): %s" % e)
                self._stop.wait(10.0)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
        self.connected = False

    def _emit(self, data: bytes):
        self.bytes_received += len(data)
        try:
            self.on_rtcm(data)
        except Exception as e:
            print("[ntrip] on_rtcm 回调异常: %s" % e)

    def stop(self):
        self._stop.set()
