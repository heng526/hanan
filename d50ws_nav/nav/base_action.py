# -*- coding: utf-8 -*-
"""行为层动作基类——统一格式（架构讨论纪要 2026-09-30 确定的范式）。

范式（沿用"摄像头类"案例）：
  - 实例化后 start() 即在独立线程运行主流程 _run()（死循环推进，直到终态）
  - 外部【永不】阻塞等待 _run() 返回；用 read() 非阻塞读状态
  - read() 返回统一状态字典（0.2 规范），主流程每周期刷新
  - wait() 只是外部便捷轮询（内部就是循环 read），不是阻塞钩子
  - stop() 请求中断：置 ABORT 标志，_run 每周期检查并尽快安全停机
  - 每个动作类可脱离总控单独实例化测试（ctx 注入 Mock 即可）

子类只需实现 _setup() 与 _run_once()（单周期逻辑），生命周期由基类管理。
"""
import threading
import time
from enum import Enum
from typing import Optional


class ActionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAIL = "FAIL"
    ABORTED = "ABORTED"


TERMINAL_STATES = {ActionState.SUCCESS, ActionState.FAIL, ActionState.ABORTED}


class BaseAction(threading.Thread):
    #: 动作名（子类覆盖，用于状态字典与日志）
    NAME = "base_action"

    def __init__(self, ctx, period: float = 0.1):
        """
        ctx       运行上下文（chassis/rtk/perception/safety/config），动作类不直接碰 SDK
        period    主循环周期秒（默认 10Hz）
        """
        super().__init__(daemon=True, name=self.NAME)
        self.ctx = ctx
        self.period = period

        self._state = ActionState.IDLE
        self._progress = 0.0
        self._error: Optional[str] = None
        self._data: dict = {}
        self._abort_evt = threading.Event()
        self._start_evt = threading.Event()
        self._state_lock = threading.Lock()
        self._t_start = 0.0

    # ================= 对外接口（非阻塞） =================

    def start(self):
        """触发主流程并启动线程。"""
        if self.is_alive():
            self._set(state=ActionState.RUNNING)
            self._start_evt.set()
            return
        self._set(state=ActionState.RUNNING)
        self._start_evt.set()
        super().start()

    def read(self) -> dict:
        """非阻塞读取统一状态（总控/外部随时调用）。"""
        with self._state_lock:
            return {
                "action": self.NAME,
                "state": self._state.value,
                "progress": round(self._progress, 3),
                "error": self._error,
                "elapsed_s": round(time.time() - self._t_start, 2) if self._t_start else 0.0,
                "data": dict(self._data),
            }

    def wait(self, timeout: float = None, poll: float = 0.1) -> bool:
        """便捷轮询：终态返回 True（SUCCESS 亦 True），超时返回 False。
        注意：这是外部便利函数，内部就是循环 read()，语义与摄像头类 read 相同。"""
        deadline = time.time() + timeout if timeout else None
        while True:
            st = self.read()["state"]
            if st in (ActionState.SUCCESS.value, ActionState.FAIL.value,
                      ActionState.ABORTED.value):
                return st == ActionState.SUCCESS.value
            if deadline is not None and time.time() >= deadline:
                return False
            time.sleep(poll)

    def stop(self):
        """请求安全中断（急停/取消）。主循环下一周期退出并置 ABORTED。"""
        self._abort_evt.set()

    # ================= 线程主体（基类管理，子类不碰） =================

    def run(self):
        self._start_evt.wait(timeout=5.0)
        self._t_start = time.time()
        try:
            self._setup()
            while not self._abort_evt.is_set():
                tick_start = time.time()
                done = self._run_once()          # 子类单周期逻辑
                if done:                          # 返回 True 表示到达终态
                    break
                time.sleep(max(0.0, self.period - (time.time() - tick_start)))
            if self._abort_evt.is_set():
                self._on_abort()
                self._set(state=ActionState.ABORTED, error="aborted_by_request")
        except Exception as e:                    # 主流程异常兜底 → FAIL，不静默死线程
            self._set(state=ActionState.FAIL, error="exception: %s" % e)
        finally:
            try:
                self._on_finish()                # 异常/中断时也关闭本动作启用的感知类
            except Exception as e:
                self._set(state=ActionState.FAIL, error="cleanup_exception: %s" % e)
            finally:
                self._safe_stop_chassis()

    # ================= 子类接口 =================

    def _setup(self):
        """主流程开始前的一次性准备（默认空）。"""

    def _step_chassis(self):
        """仿真推进（Mock 需要；真狗无此方法自动跳过）。"""
        step = getattr(self.ctx.chassis, "step", None)
        if callable(step):
            step()

    def _perception_get(self, name: str):
        """按名取感知类（lever/pipe/scene/lidar），无上下文返回 None。"""
        return getattr(self.ctx.perception, name, None) if self.ctx.perception else None

    def _perception_set(self, name: str, on: bool):
        """开关感知类（锁机制省算力）。"""
        p = self._perception_get(name)
        if p is None:
            return
        p.enable() if on else p.disable()

    def _run_once(self) -> bool:
        """单周期逻辑。返回 True 表示动作到达终态（本周期内已 _set SUCCESS/FAIL）。"""
        raise NotImplementedError

    def _on_abort(self):
        """被中断时的安全处理（默认空，子类可覆盖）。"""

    def _on_finish(self):
        """终态后的收尾（默认空）。"""

    def _safe_stop_chassis(self):
        try:
            if self.ctx is not None and getattr(self.ctx, "chassis", None) is not None:
                self.ctx.chassis.set_velocity(0.0, 0.0)
        except Exception:
            pass

    # ================= 状态写入（子类在 _run_once 中调用） =================

    def _set(self, state: ActionState = None, progress: float = None,
             error: str = None, data: dict = None):
        with self._state_lock:
            if state is not None:
                self._state = state
            if progress is not None:
                self._progress = max(0.0, min(1.0, progress))
            if error is not None:
                self._error = error
            if data is not None:
                self._data.update(data)

    # ---- 便捷判断 ----
    @property
    def aborted(self) -> bool:
        return self._abort_evt.is_set()

    @property
    def state(self) -> ActionState:
        return self._state
