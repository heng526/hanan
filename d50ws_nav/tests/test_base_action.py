# -*- coding: utf-8 -*-
"""行为层统一范式测试：BaseAction 生命周期 + CruiseAction 示范动作。

验证架构纪要确定的范式：
- 实例化 + start() → 独立线程运行
- read() 非阻塞（任意时刻可调）
- wait() 轮询到终态
- stop() 中断 → ABORTED
- 动作类脱离总控可独立测试（Mock ctx 注入）
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nav.base_action import BaseAction, ActionState
from nav.context import RunContext
from nav.actions.cruise import CruiseAction
from sdk_adapter.mock import MockRobot


def test_lifecycle_and_read_nonblocking():
    robot = MockRobot()
    act = CruiseAction(RunContext(chassis=robot), end=(5.0, 0.0))
    # start 前即可读
    assert act.read()["state"] == "IDLE"
    act.start()
    st = act.read()
    assert st["state"] in ("RUNNING", "SUCCESS")
    assert st["action"] == "cruise"
    ok = act.wait(timeout=30)
    assert ok, act.read()
    final = act.read()
    assert final["state"] == "SUCCESS"
    assert final["progress"] == 1.0
    assert "final_pose" in final["data"]


def test_reach_target():
    robot = MockRobot()
    act = CruiseAction(RunContext(chassis=robot), end=(8.0, 6.0))
    act.start()
    assert act.wait(timeout=60)
    d = robot.pose.dist_to(8.0, 6.0)
    assert d <= 0.3, "到位误差 %.2f 超差" % d


def test_stop_aborts():
    robot = MockRobot()
    act = CruiseAction(RunContext(chassis=robot), end=(50.0, 0.0))
    act.start()
    time.sleep(1.0)
    act.stop()
    deadline = time.time() + 3
    while act.read()["state"] not in ("ABORTED",) and time.time() < deadline:
        time.sleep(0.05)
    assert act.read()["state"] == "ABORTED"


class _BoomAction(BaseAction):
    """主流程抛异常 → 兜底 FAIL（不静默死线程）。"""
    NAME = "boom"

    def _run_once(self):
        raise RuntimeError("boom")


def test_exception_failsafe():
    act = _BoomAction(RunContext(chassis=MockRobot()))
    act.start()
    act.wait(timeout=3)
    st = act.read()
    assert st["state"] == "FAIL"
    assert "boom" in (st["error"] or "")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS %s" % name)
    print("base_action tests: all passed")
