# -*- coding: utf-8 -*-
"""执行器 Mock：机械臂作业模块（家康）/挑杆模块/声纹检测 的接口占位。

真实模块到位后按同接口替换；动作类只依赖这些方法签名。
"""
import time


class MockManipulator:
    """机械臂排风作业（A7/A8 调用）。start_vent 非阻塞，read 轮询状态。"""

    def __init__(self, exec_seconds: float = 0.4):
        self.exec_seconds = exec_seconds
        self._t0 = None
        self._command_id = None
        self._sequence = 0
        self._cancelled = False

    def _begin(self, kind):
        self._sequence += 1
        self._command_id = "%s-%d" % (kind, self._sequence)
        self._t0 = time.time()
        self._cancelled = False
        return {"accepted": True, "command_id": self._command_id,
                "timestamp": self._t0}

    def start_vent(self, mode: str, lever_pose):
        return self._begin("vent")

    def stow(self, mode: str):
        return self._begin("stow")

    def stop(self, command_id):
        if command_id == self._command_id:
            self._cancelled = True
            return True
        return False

    def read(self) -> dict:
        if self._t0 is None:
            return {"state": "IDLE"}
        state = ("ABORTED" if self._cancelled else
                 "DONE" if time.time() - self._t0 >= self.exec_seconds
                 else "RUNNING")
        return {"state": state, "command_id": self._command_id,
                "timestamp": time.time()}


class MockRod:
    """挑杆摘管作业（A9 调用）。"""

    def __init__(self, exec_seconds: float = 0.4):
        self.exec_seconds = exec_seconds
        self._t0 = None
        self._command_id = None
        self._sequence = 0
        self._cancelled = False

    def start_detach(self, hook_point, lift_dir):
        self._sequence += 1
        self._command_id = "detach-%d" % self._sequence
        self._t0 = time.time()
        self._cancelled = False
        return {"accepted": True, "command_id": self._command_id,
                "timestamp": self._t0}

    def stop(self, command_id):
        if command_id == self._command_id:
            self._cancelled = True
            return True
        return False

    def read(self) -> dict:
        if self._t0 is None:
            return {"state": "IDLE"}
        state = ("ABORTED" if self._cancelled else
                 "DONE" if time.time() - self._t0 >= self.exec_seconds
                 else "RUNNING")
        return {"state": state, "command_id": self._command_id,
                "timestamp": time.time()}


class MockAcoustic:
    """排风声纹检测（P5 CHECK 类占位）。按预设结果序列依次返回（支持重试测试）。"""

    def __init__(self, results=("SUCCESS",)):
        self.results = list(results)
        self.calls = 0

    def check(self, command_id, since_ts, window_s: float = 1.0) -> dict:
        self.calls += 1
        return {"result": self.results.pop(0) if self.results else "FAIL",
                "command_id": command_id, "timestamp": time.time()}
