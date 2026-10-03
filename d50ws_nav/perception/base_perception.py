# -*- coding: utf-8 -*-
"""L2 感知层基类——统一规范（纪要范式 + L2L3深化设计 §3.1）。

范式：
  - 实例化即起常驻线程，内部死循环（取帧→推理→更新结果缓存）
  - enable()/disable() 由行为层按需开关：关闭时线程挂起省算力，模型不卸载
  - read() 非阻塞读最新结果（TRACK 类）；单次判定类（CHECK）另提供 check() 方法
  - 输出统一狗坐标系；业务字段（task_id 等）不进感知类
  - 统一错误码族 E_*（v6 §7.1.9）
"""
import threading
import time
import math


class BasePerception(threading.Thread):
    NAME = "base_perception"

    def __init__(self, period: float = 0.1):
        super().__init__(daemon=True, name=self.NAME)
        self.period = period
        self._enabled = threading.Event()
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._result: dict = {"error_code": "E_SENSOR_NO_DATA"}
        self._frame_count = 0
        self._last_ts = 0.0
        self._params: dict = {}
        self._start_lock = threading.Lock()
        self._ever_started = False
        self.start()                      # 线程常驻；未 enable 前仅等待，不取帧

    def start(self):
        """兼容既有调用：运行中的 start() 幂等；关闭后不可复活。"""
        with self._start_lock:
            if self.is_alive():
                return
            if self._ever_started:
                raise RuntimeError("感知线程不可重启，需创建新实例")
            self._ever_started = True
            super().start()

    # ---- 行为层开关（锁机制省算力） ----
    def enable(self, **params):
        """打开开关；可传 ROI、阈值和 track_rate 等算法参数。"""
        if self._stop_evt.is_set():
            raise RuntimeError("感知线程已关闭，需创建新实例")
        if "track_rate" in params:
            rate = float(params["track_rate"])
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("track_rate 必须为正数")
            self.period = 1.0 / rate
        with self._lock:
            self._params.update(params)
            self._result = {"error_code": "E_SENSOR_NO_DATA"}
            self._last_ts = 0.0
        self._enabled.set()

    def disable(self):
        self._enabled.clear()

    @property
    def enabled(self) -> bool:
        return self._enabled.is_set()

    def read(self) -> dict:
        """非阻塞读最新结果。"""
        with self._lock:
            out = dict(self._result)
            out["frame_count"] = self._frame_count
            out["timestamp"] = self._last_ts if self._last_ts else None
            out["age_s"] = round(time.time() - self._last_ts, 3) if self._last_ts else None
            out["enabled"] = self._enabled.is_set() and not self._stop_evt.is_set()
            return out

    def _parameters(self) -> dict:
        with self._lock:
            return dict(self._params)

    @staticmethod
    def _sample(source, max_age_s: float = 1.0):
        """读取 vCam 风格的 (ret, payload, Unix 时间戳)，拒绝断流旧帧。"""
        if source is None:
            return None, "E_SENSOR_NO_DATA"
        try:
            packet = source.read()
        except Exception:
            return None, "E_SENSOR_NO_DATA"
        if not isinstance(packet, (tuple, list)) or len(packet) != 3:
            return None, "E_SENSOR_NO_DATA"
        ret, payload, timestamp = packet
        if not ret or payload is None:
            return None, "E_SENSOR_NO_DATA"
        try:
            age = time.time() - float(timestamp)
        except (TypeError, ValueError, OverflowError):
            return None, "E_SENSOR_NO_DATA"
        if not math.isfinite(age) or abs(age) > max_age_s:
            return None, "E_SENSOR_NO_DATA"
        return payload, None

    @staticmethod
    def _point_dog(point_camera, to_dog):
        """相机系三维点经外参接口变为狗系；外参缺失时不伪造坐标。"""
        if to_dog is None:
            return None, "E_TF_UNAVAILABLE"
        try:
            point = [float(x) for x in point_camera]
            if len(point) != 3 or not all(math.isfinite(x) for x in point):
                return None, "E_DEPTH_INVALID"
            result = [float(x) for x in to_dog(point)]
            if len(result) != 3 or not all(math.isfinite(x) for x in result):
                return None, "E_TF_UNAVAILABLE"
        except (TypeError, ValueError, OverflowError):
            return None, "E_DEPTH_INVALID"
        return result, None

    # ---- 线程主体 ----
    def run(self):
        while not self._stop_evt.is_set():
            if not self._enabled.wait(timeout=0.1):
                continue
            if self._stop_evt.is_set():
                break
            tick = time.time()
            try:
                res = self._infer_once()          # 子类单帧推理
                with self._lock:
                    self._result = res or {"error_code": "E_LOW_QUALITY"}
                    self._frame_count += 1
                    self._last_ts = time.time()
            except Exception as e:
                with self._lock:
                    self._result = {"error_code": "E_INFER_CRASH", "error_msg": str(e)}
            self._stop_evt.wait(max(0.0, self.period - (time.time() - tick)))

    def close(self):
        self._stop_evt.set()
        self._enabled.set()
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=1.0)

    # ---- 子类接口 ----
    def _infer_once(self) -> dict:
        """单帧推理，返回结果字典（子类实现）。"""
        raise NotImplementedError
