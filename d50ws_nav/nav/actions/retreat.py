# -*- coding: utf-8 -*-
"""A10 RetreatAction 退出动作（动作类清单 #A10，high/low/detach 三变体）。

上装归位（通知作业模块）→ 底盘直线后退出限界 → 转身恢复巡航姿态。
三种作业的退出路径参数化区分（距离/目标朝向）。
"""
import math
import time

from ..base_action import BaseAction, ActionState
from ..action_receipt import (accepted_command, command_state,
                              perception_fresh, cancel_command)


class RetreatAction(BaseAction):
    NAME = "retreat"

    def __init__(self, ctx, mode: str = "high", retreat_distance: float = 1.0,
                 retreat_speed: float = 0.3, target_heading: float = None,
                 backup_timeout: float = 30.0, yaw_tol: float = 0.05,
                 period: float = 0.1, retreat_guard=None):
        super().__init__(ctx, period=period)
        self._mode = mode
        self._distance = retreat_distance
        self._speed = retreat_speed
        self._target_heading = target_heading
        self._backup_timeout = backup_timeout
        self._yaw_tol = yaw_tol
        self._phase = "STOW"
        self._start = None
        self._t0 = None
        self._scene = None
        self._retreat_guard = retreat_guard
        self._stow_id = None
        self._stow_started = None
        self._motion_started = False
        self._last_scene_frame = 0

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _setup(self):
        self._start = self.ctx.chassis.get_pose()
        self._t0 = time.time()
        self._scene = self._perception_get("scene")
        if self._scene is not None:
            self._scene.enable()

    def _run_once(self) -> bool:
        ch = self.ctx.chassis
        pose = ch.get_pose()
        if self._scene is None:
            self._set(state=ActionState.FAIL, error="E_SCENE_UNAVAILABLE")
            return True
        if not callable(self._retreat_guard):
            self._set(state=ActionState.FAIL, error="E_RETREAT_GUARD_UNAVAILABLE")
            return True
        try:
            clear = self._retreat_guard() is True
        except Exception:
            clear = False
        if not clear:
            self._set(state=ActionState.FAIL, error="E_RETREAT_UNSAFE")
            return True
        if time.time() - self._t0 > self._backup_timeout:
            self._set(state=ActionState.FAIL, error="FAIL_BACKUP_TIMEOUT")
            return True

        # ---- 阶段1：上装归位，必须取得本次命令的完成回执 ----
        if self._phase == "STOW":
            manip = self.ctx.manipulator
            if manip is None or not callable(getattr(manip, "stow", None)):
                self._set(state=ActionState.FAIL, error="E_STOW_UNAVAILABLE")
                return True
            if self._stow_started is None:
                started_at = time.time()
                reply = manip.stow(self._mode)
                command_id = accepted_command(reply, started_at)
                if command_id is None:
                    self._set(state=ActionState.FAIL, error="E_STOW_START_UNCONFIRMED")
                    return True
                self._stow_started = started_at
                self._stow_id = command_id
                self._set(data={"phase": "HOLD", "reason": "WAIT_STOW"})
                return False
            try:
                receipt = manip.read()
            except Exception:
                receipt = None
            state = command_state(receipt, self._stow_id, self._stow_started)
            if state in ("FAIL", "ABORTED"):
                self._set(state=ActionState.FAIL, error="FAIL_STOW_STATE:%s" % state)
                return True
            if state != "DONE":
                self._set(data={"phase": "HOLD",
                                "reason": "E_STOW_RECEIPT_INVALID" if state is None
                                          else "WAIT_STOW"})
                return False
            self._phase = "BACKUP"
            self._set(progress=0.2, data={"mode": self._mode,
                                          "stow": "confirmed", "reason": None})
            return False

        try:
            scene_state = self._scene.read()
        except Exception:
            scene_state = None
        scene_safe = (perception_fresh(scene_state)
                      and scene_state.get("safety_status") == "OK"
                      and scene_state.get("risk_level") == "NONE")
        if not scene_safe:
            ch.set_velocity(0.0, 0.0)
            if self._motion_started:
                self._set(state=ActionState.FAIL, error="E_SCENE_DATA_INVALID")
                return True
            self._set(data={"phase": "HOLD", "reason": "E_SCENE_NOT_READY"})
            return False
        if scene_state["frame_count"] <= self._last_scene_frame:
            ch.set_velocity(0.0, 0.0)
            self._set(data={"phase": "HOLD", "reason": "E_SCENE_FRAME_REPEATED"})
            return False
        self._last_scene_frame = scene_state["frame_count"]

        # ---- 阶段2：直线后撤 ----
        if self._phase == "BACKUP":
            traveled = math.hypot(pose.e - self._start.e, pose.n - self._start.n)
            if traveled >= self._distance:
                ch.set_velocity(0.0, 0.0)
                self._phase = "TURN"
                self._set(progress=0.7)
                return False
            ch.set_velocity(-self._speed, 0.0)
            self._motion_started = True
            self._step_chassis()
            self._set(progress=0.2 + 0.5 * traveled / self._distance)
            return False

        # ---- 阶段3：转身恢复巡航姿态 ----
        if self._target_heading is None:
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"final_pose": [round(pose.e, 3), round(pose.n, 3),
                                           round(pose.yaw, 3)]})
            return True
        err = self._wrap(self._target_heading - pose.yaw)
        if abs(err) <= self._yaw_tol:
            ch.set_velocity(0.0, 0.0)
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"final_pose": [round(pose.e, 3), round(pose.n, 3),
                                           round(pose.yaw, 3)]})
            return True
        ch.set_velocity(0.0, max(-0.6, min(0.6, 1.5 * err)))
        self._motion_started = True
        self._step_chassis()
        return False

    def _on_abort(self):
        cancel_command(self.ctx.manipulator, self._stow_id)

    def _on_finish(self):
        if self._scene is not None:
            self._scene.disable()
        if self.state == ActionState.FAIL:
            cancel_command(self.ctx.manipulator, self._stow_id)
