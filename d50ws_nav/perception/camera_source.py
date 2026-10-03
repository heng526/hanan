"""与参考提供方 vCam.read() 兼容的相机帧源；不承担目标识别或机器人控制。"""

import threading
import time
import math


def _open_cv2(src):
    import cv2

    return cv2.VideoCapture(src)


def _resize_cv2(frame, size):
    import cv2

    return cv2.resize(frame, size)


class CameraSource:
    """后台采集最新帧，供 L2 的 source.read() 注入使用。

    source_kind 必须显式指定为 device、file 或 rtsp。真实设备地址、帧率、
    标定和算法由外部实现方配置提供；构造后会立即启动采集线程。
    """

    def __init__(self, src, image_height, image_width, *, source_kind,
                 capture_factory=None, resize=None, reconnect_interval=0.5,
                 max_age_s=1.0, loop_file=False):
        if source_kind not in ("device", "file", "rtsp"):
            raise ValueError("source_kind 必须为 device、file 或 rtsp")
        if source_kind == "device":
            valid_src = (isinstance(src, int) and not isinstance(src, bool) and src >= 0)
            valid_src = valid_src or (isinstance(src, str) and bool(src.strip()))
        elif source_kind == "file":
            valid_src = isinstance(src, str) and bool(src.strip())
        else:
            valid_src = isinstance(src, str) and src.lower().startswith(("rtsp://", "rtsps://"))
        if not valid_src:
            raise ValueError("src 与 source_kind 不匹配")
        if (not isinstance(image_height, int) or isinstance(image_height, bool)
                or image_height <= 0):
            raise ValueError("image_height 必须为正整数")
        if (not isinstance(image_width, int) or isinstance(image_width, bool)
                or image_width <= 0):
            raise ValueError("image_width 必须为正整数")
        if (not math.isfinite(reconnect_interval) or reconnect_interval <= 0
                or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError("重连间隔和帧有效期必须合法")
        if loop_file and source_kind != "file":
            raise ValueError("loop_file 仅适用于文件源")

        self.src = src
        self.source_kind = source_kind
        self._size = (image_width, image_height)
        self._capture_factory = capture_factory or _open_cv2
        self._resize = resize or _resize_cv2
        self._reconnect_interval = reconnect_interval
        self._max_age_s = max_age_s
        self._loop_file = loop_file
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._frame = None
        self._ret = False
        self._timestamp = 0.0
        self._frame_count = 0
        self._state = "CONNECTING"
        self._last_error = None
        self._thread = threading.Thread(target=self._update, name="camera_source", daemon=True)
        self._thread.start()

    def _publish(self, state, *, error=None, frame=None):
        with self._lock:
            if self._stop_evt.is_set() and state != "STOPPED":
                self._state = "STOPPING"
                self._ret = False
                return
            self._state = state
            self._last_error = error
            if frame is None:
                self._ret = False
            else:
                self._frame = frame
                self._timestamp = time.time()
                self._frame_count += 1
                self._ret = True

    @staticmethod
    def _release(capture):
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass

    def _update(self):
        try:
            while not self._stop_evt.is_set():
                self._publish("CONNECTING" if self._frame_count == 0 else "RECONNECTING")
                capture = None
                opened = False
                error = "E_CAMERA_OPEN"
                try:
                    capture = self._capture_factory(self.src)
                    if capture is not None:
                        probe = getattr(capture, "isOpened", None)
                        opened = bool(probe()) if callable(probe) else True
                    if opened and not self._stop_evt.is_set():
                        self._publish("RUNNING")
                        while not self._stop_evt.is_set():
                            try:
                                ret, image = capture.read()
                            except Exception:
                                error = "E_CAMERA_READ"
                                break
                            if self._stop_evt.is_set():
                                break
                            if not ret or image is None:
                                error = "E_CAMERA_READ"
                                break
                            try:
                                frame = self._resize(image, self._size).copy()
                            except Exception:
                                error = "E_CAMERA_FRAME"
                                break
                            self._publish("RUNNING", frame=frame)
                except Exception:
                    error = "E_CAMERA_OPEN"
                finally:
                    self._release(capture)

                if self._stop_evt.is_set():
                    break
                if self.source_kind == "file" and not self._loop_file:
                    self._publish("EOF" if opened and error == "E_CAMERA_READ" else "ERROR",
                                  error=None if opened and error == "E_CAMERA_READ" else error)
                    break
                self._publish("RECONNECTING", error=error)
                if self._stop_evt.wait(self._reconnect_interval):
                    break
        finally:
            if self._stop_evt.is_set():
                self._publish("STOPPED")

    def read(self):
        """返回 (ret, 独立帧副本, Unix 时间戳)；无效/过期帧返回 False、None。"""
        with self._lock:
            fresh = (self._ret and self._timestamp > 0
                     and 0 <= time.time() - self._timestamp <= self._max_age_s)
            return (True, self._frame.copy(), self._timestamp) if fresh else (
                False, None, self._timestamp)

    def status(self):
        """返回采集状态；不回显可能含凭据的 RTSP 地址。"""
        with self._lock:
            age = time.time() - self._timestamp if self._timestamp else None
            fresh = bool(self._ret and age is not None and 0 <= age <= self._max_age_s)
            state = "STALE" if self._state == "RUNNING" and self._ret and not fresh else self._state
            return {
                "state": state,
                "source_kind": self.source_kind,
                "frame_count": self._frame_count,
                "last_frame_ts": self._timestamp,
                "age_s": age,
                "fresh": fresh,
                "last_error": self._last_error,
                "thread_alive": self._thread.is_alive(),
            }

    def stop(self, timeout=2.0):
        """停止并等待采集线程释放资源；阻塞中的后端超时则返回 False。"""
        if timeout is not None and timeout < 0:
            raise ValueError("timeout 不得为负数")
        self._stop_evt.set()
        self._publish("STOPPING")
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=timeout)
        done = not self._thread.is_alive()
        if done:
            self._publish("STOPPED")
        return done

    close = stop
