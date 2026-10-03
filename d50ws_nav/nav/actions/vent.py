# -*- coding: utf-8 -*-
"""A7/A8 VentAction 高位/低位排风动作（动作类清单 #A7/A8）。

底盘锁定 → 通知机械臂作业模块执行（接口对接家康模块）
→ 声纹判定（FAIL/UNCERTAIN 自动重试一次，重试仍失败 FAIL 上报总控）。
"""
import time

from ..base_action import BaseAction, ActionState
from ..action_receipt import accepted_command, command_state, cancel_command


class VentAction(BaseAction):
    NAME = "vent"

    def __init__(self, ctx, mode: str = "high", lever_pose=None,
                 exec_timeout: float = 5.0, period: float = 0.1):
        super().__init__(ctx, period=period)
        self._mode = mode
        self._lever_pose = lever_pose
        self._exec_timeout = exec_timeout
        self._t0 = None
        self._retry_used = False
        self._command_id = None
        self._done_at = None

    def _run_once(self) -> bool:
        manip = self.ctx.manipulator
        if manip is None:
            self._set(state=ActionState.FAIL, error="E_MANIPULATOR_UNAVAILABLE")
            return True
        if self.ctx.acoustic is None or not callable(getattr(self.ctx.acoustic, "check", None)):
            self._set(state=ActionState.FAIL, error="E_ACOUSTIC_UNAVAILABLE")
            return True

        # ---- 阶段1：本次命令的接受与完成回执 ----
        if self._t0 is None:
            started_at = time.time()
            reply = manip.start_vent(self._mode, self._lever_pose)
            command_id = accepted_command(reply, started_at)
            if command_id is None:
                self._set(state=ActionState.FAIL, error="E_MANIP_START_UNCONFIRMED")
                return True
            self._t0 = started_at
            self._command_id = command_id
            self._done_at = None
            self._set(data={"manip": "started", "mode": self._mode,
                            "command_id": command_id})
            return False
        try:
            receipt = manip.read()
        except Exception:
            receipt = None
        mst = command_state(receipt, self._command_id, self._t0)
        if mst in ("FAIL", "ABORTED"):
            self._set(state=ActionState.FAIL, error="FAIL_MANIP_STATE:%s" % mst)
            return True
        if mst != "DONE":
            if time.time() - self._t0 > self._exec_timeout:
                self._set(state=ActionState.FAIL, error="FAIL_MANIP_TIMEOUT")
                return True
            self._set(progress=0.5, data={"phase": "HOLD" if mst is None else "RUNNING",
                                          "reason": "E_MANIP_RECEIPT_INVALID" if mst is None else None})
            return False
        if self._done_at is None:
            self._done_at = time.time()

        # ---- 阶段2：只接受本次动作完成后的成功判定 ----
        acoustic = self.ctx.acoustic
        if acoustic is None:
            self._set(state=ActionState.FAIL, error="E_ACOUSTIC_UNAVAILABLE")
            return True
        try:
            observed = acoustic.check(command_id=self._command_id,
                                      since_ts=self._done_at)
        except Exception:
            observed = None
        if (not isinstance(observed, dict)
                or observed.get("command_id") != self._command_id
                or type(observed.get("timestamp")) not in (int, float)
                or not self._done_at <= observed["timestamp"] <= time.time()
                or observed.get("result") not in ("SUCCESS", "FAIL", "UNCERTAIN")):
            self._set(state=ActionState.FAIL, error="E_ACOUSTIC_RECEIPT_INVALID")
            return True
        result = observed["result"]
        if result == "SUCCESS":
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"vent_result": "SUCCESS", "mode": self._mode,
                            "retried": self._retry_used, "phase": "DONE",
                            "reason": None})
            return True
        if not self._retry_used:
            self._retry_used = True
            self._t0 = None                          # 重试：重新执行+判定
            self._command_id = None
            self._done_at = None
            self._set(data={"first_result": result, "retrying": True})
            return False
        self._set(state=ActionState.FAIL, error="FAIL_VENT_%s" % result,
                  data={"vent_result": result})
        return True

    def _on_abort(self):
        cancel_command(self.ctx.manipulator, self._command_id)

    def _on_finish(self):
        if self.state == ActionState.FAIL:
            cancel_command(self.ctx.manipulator, self._command_id)
