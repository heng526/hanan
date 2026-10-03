"""相机源仅使用假采集器；不访问真实摄像头、视频文件或网络流。"""

import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception import CameraSource, LeverDetector


def wait_until(predicate, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail("相机工作线程未达到预期状态")


class LiveCapture:
    def __init__(self, value=7):
        self.frame = np.full((2, 3, 3), value, dtype=np.uint8)
        self.release_count = 0

    def isOpened(self):
        return True

    def read(self):
        time.sleep(0.01)
        return True, self.frame

    def release(self):
        self.release_count += 1


class EofCapture(LiveCapture):
    def __init__(self):
        super().__init__()
        self.calls = 0

    def read(self):
        self.calls += 1
        return (True, self.frame) if self.calls == 1 else (False, None)


def same_size(frame, size):
    assert size == (3, 2)
    return frame.copy()


def test_thread_safe_snapshot_and_l2_injection():
    capture = LiveCapture()
    calls = []
    source = CameraSource("fake.mp4", 2, 3, source_kind="file",
                          capture_factory=lambda src: calls.append(src) or capture,
                          resize=same_size)
    lever = None
    try:
        wait_until(lambda: source.status()["frame_count"] >= 1)
        ret, frame, timestamp = source.read()
        assert ret and timestamp > 0 and frame.shape == (2, 3, 3)
        frame[:] = 0
        assert source.read()[1][0, 0, 0] == 7
        assert source.status()["fresh"]
        assert source.status()["source_kind"] == "file"
        assert calls == ["fake.mp4"]

        lever = LeverDetector(source, lambda image, _params: {
            "target_detected": bool(image[0, 0, 0]),
            "target_point_camera": [1, 2, 3], "confidence": 0.9,
        }, to_dog=lambda point: point)
        lever.enable()
        wait_until(lambda: lever.read().get("target_detected") is True)
        assert lever.read()["target_point_dog"] == [1.0, 2.0, 3.0]
    finally:
        if lever is not None:
            lever.close()
        assert source.stop()
    assert capture.release_count == 1
    assert source.status()["state"] == "STOPPED"


def test_file_eof_does_not_reopen_or_return_old_frame():
    capture = EofCapture()
    calls = []
    source = CameraSource("sample.mp4", 2, 3, source_kind="file",
                          capture_factory=lambda src: calls.append(src) or capture,
                          resize=same_size)
    try:
        wait_until(lambda: source.status()["state"] == "EOF")
        assert calls == ["sample.mp4"]
        assert capture.release_count == 1
        assert source.status()["last_frame_ts"] > 0
        assert source.read()[0:2] == (False, None)
    finally:
        assert source.stop()
    assert source.status()["state"] == "STOPPED"


def test_rtsp_reconnects_after_open_failure_without_real_network():
    failed = LiveCapture()
    failed.isOpened = lambda: False
    recovered = LiveCapture(value=9)
    captures = [failed, recovered]
    calls = []

    def factory(src):
        calls.append(src)
        return captures.pop(0)

    source = CameraSource("rtsp://fake.invalid/stream", 2, 3, source_kind="rtsp",
                          capture_factory=factory, resize=same_size,
                          reconnect_interval=0.01)
    try:
        wait_until(lambda: source.status()["frame_count"] >= 1)
        assert len(calls) == 2
        assert failed.release_count == 1
        assert source.read()[1][0, 0, 0] == 9
    finally:
        assert source.stop()
    assert recovered.release_count == 1
    assert len(calls) == 2


def test_rtsp_reconnects_after_read_failure():
    first = EofCapture()
    second = LiveCapture(value=11)
    captures = [first, second]
    calls = []

    def factory(src):
        calls.append(src)
        return captures.pop(0)

    source = CameraSource("rtsp://fake.invalid/stream", 2, 3, source_kind="rtsp",
                          capture_factory=factory, resize=same_size,
                          reconnect_interval=0.01)
    try:
        wait_until(lambda: source.status()["frame_count"] >= 2)
        assert first.release_count == 1
        assert len(calls) == 2
        assert source.read()[1][0, 0, 0] == 11
    finally:
        assert source.stop()
    assert second.release_count == 1


def test_old_frame_becomes_stale():
    class PausedCapture(EofCapture):
        def read(self):
            self.calls += 1
            if self.calls == 1:
                return True, self.frame
            time.sleep(0.15)
            return False, None

    capture = PausedCapture()
    source = CameraSource(0, 2, 3, source_kind="device",
                          capture_factory=lambda _src: capture, resize=same_size,
                          max_age_s=0.02, reconnect_interval=0.2)
    try:
        wait_until(lambda: source.status()["frame_count"] == 1)
        wait_until(lambda: source.status()["state"] == "STALE")
        assert source.read()[0:2] == (False, None)
        assert source.status()["last_frame_ts"] > 0
    finally:
        assert source.stop(timeout=1)
    assert capture.release_count == 1


def test_stop_during_blocked_read_does_not_double_release_or_reopen():
    entered = threading.Event()
    unblock = threading.Event()

    class BlockingCapture(LiveCapture):
        def read(self):
            entered.set()
            unblock.wait()
            return False, None

    capture = BlockingCapture()
    calls = []
    source = CameraSource(0, 2, 3, source_kind="device",
                          capture_factory=lambda src: calls.append(src) or capture,
                          resize=same_size, reconnect_interval=0.01)
    try:
        assert entered.wait(1)
        assert not source.stop(timeout=0.01)
        assert capture.release_count == 0
        assert source.status()["state"] == "STOPPING"
        unblock.set()
        assert source.stop(timeout=1)
        assert source.stop(timeout=1)
        assert source.read()[0:2] == (False, None)
        assert calls == [0]
        assert capture.release_count == 1
    finally:
        unblock.set()
        source.stop(timeout=1)


@pytest.mark.parametrize("src,kind", [
    ("plain", "rtsp"), ("rtsp://fake.invalid/x", "file_wrong"),
    (-1, "device"), ("", "file"),
])
def test_invalid_source_configuration_never_starts_worker(src, kind):
    with pytest.raises(ValueError):
        CameraSource(src, 2, 3, source_kind=kind,
                     capture_factory=lambda _: pytest.fail("不可打开采集器"))
