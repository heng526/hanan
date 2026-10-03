# -*- coding: utf-8 -*-
"""A9 DetachAction 摘管动作（动作类清单 #A9）。

等待 P2 风管检测锁定钩取点 → 通知挑杆模块（爆发上挑+水平旋转）→
底盘协同后撤（防风管下坠重连）→ P2.check_detach 判定（点距>阈值成功）。
"""
import time
import math

from ..base_action import BaseAction, ActionState
from ..action_receipt import (accepted_command, command_state,
                              perception_fresh, cancel_command)


class DetachAction(BaseAction):
    NAME = "detach"

    def __init__(self, ctx, hook_point=None, lift_dir=None,
                 exec_timeout: float = 5.0, lock_timeout: float = 8.0,
                 detach_threshold: float = 0.3, period: float = 0.1,
                 backup_guard=None):
        super().__init__(ctx, period=period)
        self._hook_point = hook_point
        self._lift_dir = lift_dir
        self._exec_timeout = exec_timeout
        self._lock_timeout = lock_timeout
        self._threshold = detach_threshold
        self._pipe = None
        self._t0 = None
        self._t_rod = None
        self._command_id = None
        self._post_frame_count = None
        self._check_since = None
        self._backup_guard = backup_guard

    def _setup(self):
        self._t_setup = time.time()
        bundle = self.ctx.perception
        self._pipe = getattr(bundle, "pipe", None) if bundle else None
        if self._pipe is not None:
            self._pipe.enable()

    def _run_once(self) -> bool:
        ch = self.ctx.chassis
        now = time.time()
        if self._pipe is None:
            self._set(state=ActionState.FAIL, error="E_DETECTOR_UNAVAILABLE")
            return True
        if not callable(self._backup_guard):
            self._set(state=ActionState.FAIL, error="E_BACKUP_GUARD_UNAVAILABLE")
            return True
        try:
            clear = self._backup_guard() is True
        except Exception:
            clear = False
        if not clear:
            self._set(state=ActionState.FAIL, error="E_BACKUP_UNSAFE")
            return True

        # ---- 阶段1：锁定钩取点（未传入则等 P2 检出） ----
        if self._t_rod is None:
            try:
                st = self._pipe.read()
            except Exception:
                st = None
            if not perception_fresh(st) or not st.get("pipe_detected"):
                if now - self._t_setup > self._lock_timeout:
                    self._set(state=ActionState.FAIL, error="E_TARGET_NOT_FOUND")
                    return True
                self._set(progress=0.2, data={"phase": "HOLD",
                                              "reason": "E_PIPE_NOT_READY"})
                return False
            hp = st.get("hook_point_dog") if self._hook_point is None else self._hook_point
            lift = st.get("lift_direction_dog") if self._lift_dir is None else self._lift_dir
            if (not isinstance(hp, (list, tuple)) or len(hp) < 3
                    or not isinstance(lift, (list, tuple)) or len(lift) < 3
                    or any(type(v) not in (int, float) or not math.isfinite(v)
                           for v in list(hp[:3]) + list(lift[:3]))):
                self._set(state=ActionState.FAIL, error="E_HOOK_POSE_INVALID")
                return True
            self._hook_point = tuple(hp[:3])
            self._lift_dir = tuple(lift[:3])

        # ---- 阶段2：挑杆执行 ----
        rod = self.ctx.rod
        if rod is None:
            self._set(state=ActionState.FAIL, error="E_ROD_UNAVAILABLE")
            return True
        if self._t_rod is None:
            started_at = time.time()
            reply = rod.start_detach(self._hook_point, self._lift_dir)
            command_id = accepted_command(reply, started_at)
            if command_id is None:
                self._set(state=ActionState.FAIL, error="E_ROD_START_UNCONFIRMED")
                return True
            self._command_id = command_id
            self._t_rod = started_at
            self._set(progress=0.4, data={"rod": "started",
                                          "command_id": command_id})
            return False
        try:
            receipt = rod.read()
        except Exception:
            receipt = None
        rst = command_state(receipt, self._command_id, self._t_rod)
        if rst in ("FAIL", "ABORTED"):
            self._set(state=ActionState.FAIL, error="FAIL_ROD_STATE:%s" % rst)
            return True
        if rst != "DONE":
            if now - self._t_rod > self._exec_timeout:
                self._set(state=ActionState.FAIL, error="FAIL_ROD_TIMEOUT")
                return True
            self._set(data={"phase": "HOLD" if rst is None else "RUNNING",
                            "reason": "E_ROD_RECEIPT_INVALID" if rst is None else None})
            return False

        # ---- 阶段3：底盘协同后撤（防重连；距离现场标定） ----
        if self._t0 is None:
            self._t0 = now
            self._post_frame_count = self._pipe.read().get("frame_count", 0)
            self._set(progress=0.7, data={"backup": "started"})
            return False
        if now - self._t0 < 1.0:
            ch.move_vector(-0.2, 0.0, 0.0)           # 旧 Mock 参数；实机需现场定案
            step = getattr(ch, "step", None)
            if callable(step):
                step()
            return False
        ch.set_velocity(0.0, 0.0)

        # ---- 阶段4：P2 摘管成功判定 ----
        if self._check_since is None:
            self._check_since = now
        try:
            st = self._pipe.read()
        except Exception:
            st = None
        if (not perception_fresh(st)
                or st["frame_count"] <= self._post_frame_count):
            if now - self._check_since > self._lock_timeout:
                self._set(state=ActionState.FAIL, error="E_DETACH_CHECK_TIMEOUT")
                return True
            self._set(data={"phase": "HOLD", "reason": "E_POST_FRAME_NOT_READY"})
            return False
        try:
            res = self._pipe.check_detach(threshold=self._threshold)
        except Exception:
            res = None
        if not isinstance(res, dict):
            self._set(state=ActionState.FAIL, error="E_DETACH_CHECK_INVALID")
            return True
        if (type(res.get("frame_count")) is not int
                or res["frame_count"] < st["frame_count"]
                or res["frame_count"] <= self._post_frame_count
                or type(res.get("timestamp")) not in (int, float)
                or not math.isfinite(res["timestamp"])
                or not self._t0 <= res["timestamp"] <= time.time()):
            self._set(state=ActionState.FAIL, error="E_DETACH_CHECK_STALE")
            return True
        distance = res.get("separation_distance")
        if (res.get("detach_success") is True
                and type(distance) in (int, float) and math.isfinite(distance)
                and distance > self._threshold):
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"detach_result": "SUCCESS",
                            "separation_distance": distance,
                            "separation_angle": res.get("separation_angle")})
        else:
            self._set(state=ActionState.FAIL, error="FAIL_DETACH_NOT_SEPARATED",
                      data={"detach_result": "FAIL",
                            "separation_distance": res.get("separation_distance"),
                            "reconnect_risk": res.get("reconnect_risk")})
        return True

    def _on_abort(self):
        cancel_command(self.ctx.rod, self._command_id)

    def _on_finish(self):
        try:
            self.ctx.chassis.set_velocity(0.0, 0.0)
        except Exception:
            pass
        if self._pipe is not None:
            self._pipe.disable()
        if self.state == ActionState.FAIL:
            cancel_command(self.ctx.rod, self._command_id)
