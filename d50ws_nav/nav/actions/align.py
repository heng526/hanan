# -*- coding: utf-8 -*-
"""A6 AlignAction 对位（动作类清单 #A6，粗对位+精对位，high/low/detach 三模式）。

粗对位：按作业姿态要求摆位（朝向；低位附加下蹲）。
精对位：读感知类连续输出的目标狗系坐标 → 偏差 → 低速步进（横移/前进）闭环，
单步限幅 + 总时长上限防失控。偏差进容差 → 锁定 → SUCCESS。
"""
import math
import time

from ..base_action import BaseAction, ActionState


class AlignAction(BaseAction):
    NAME = "align"

    def __init__(self, ctx, mode: str = "high", detector_name: str = "lever",
                 desired=(0.6, 0.0), tol=(0.06, 0.06),
                 coarse_heading: float = None, squat_height: float = 0.35,
                 max_adjust_s: float = 30.0, period: float = 0.05):
        super().__init__(ctx, period=period)
        self._mode = mode
        self._det_name = detector_name
        self._desired = desired                      # 目标应在狗系的 (x前, y左)
        self._tol = tol
        self._coarse_heading = coarse_heading
        self._squat_height = squat_height
        self._max_adjust = max_adjust_s
        self._det = None
        self._phase = "COARSE"
        self._t_coarse = None
        self._t_fine = None
        self._steps = 0
        self._miss = 0

    def _setup(self):
        bundle = self.ctx.perception
        self._det = getattr(bundle, self._det_name, None) if bundle else None
        if self._det is not None:
            self._det.enable()
        self.ctx.chassis.stand_up()
        if self._mode == "low":
            # 低位排风：底盘下蹲（真实高度现场标定）
            h = getattr(self.ctx.chassis, "set_body_height", None)
            if callable(h):
                h(self._squat_height)

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _run_once(self) -> bool:
        ch = self.ctx.chassis
        pose = ch.get_pose()

        # ---- 粗对位：摆朝向 ----
        if self._phase == "COARSE":
            if self._t_coarse is None:
                self._t_coarse = time.time()
            if time.time() - self._t_coarse > self._max_adjust:
                self._set(state=ActionState.FAIL, error="FAIL_ALIGN_TIMEOUT",
                          data={"phase": "COARSE"})
                return True
            if self._coarse_heading is not None:
                err = self._wrap(self._coarse_heading - pose.yaw)
                if abs(err) > 0.05:
                    ch.set_velocity(0.0, max(-0.6, min(0.6, 1.5 * err)))
                    self._step_chassis()
                    return False
            ch.set_velocity(0.0, 0.0)
            self._phase = "FINE"
            self._t_fine = time.time()
            self._set(progress=0.3)
            return False

        # ---- 精对位：感知偏差闭环 ----
        if self._det is None:
            self._set(state=ActionState.FAIL, error="E_DETECTOR_UNAVAILABLE")
            return True
        st = self._det.read()
        tgt = st.get("target_point_dog") or st.get("hook_point_dog")
        if not (st.get("target_detected") or st.get("pipe_detected")) or tgt is None:
            self._miss += 1
            if self._miss > 30:                      # ~1.5s 连续丢失
                self._set(state=ActionState.FAIL, error="E_TARGET_NOT_FOUND")
                return True
            return False
        self._miss = 0

        ex = tgt[0] - self._desired[0]               # 前向偏差：目标偏前→狗前进
        ey = tgt[1] - self._desired[1]               # 横向偏差：目标偏左→狗左移
        if abs(ex) <= self._tol[0] and abs(ey) <= self._tol[1]:
            ch.set_velocity(0.0, 0.0)
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"final_offset": [round(ex, 4), round(ey, 4)],
                            "adjust_steps": self._steps,
                            "target_pose_dog": [round(v, 4) for v in tgt[:3]]})
            return True

        if time.time() - self._t_fine > self._max_adjust:
            self._set(state=ActionState.FAIL, error="FAIL_ALIGN_TIMEOUT",
                      data={"final_offset": [round(ex, 4), round(ey, 4)]})
            return True

        # 低速步进（限幅防振荡）
        vx = max(-0.12, min(0.12, 0.5 * ex))
        vy = max(-0.08, min(0.08, 0.5 * ey))
        ch.move_vector(vx, vy, 0.0)
        step = getattr(ch, "step", None)
        if callable(step):
            step()
        self._steps += 1
        remain = (abs(ex) + abs(ey)) / (abs(self._desired[0]) + abs(self._desired[1]) + 0.3)
        self._set(progress=0.3 + 0.7 * max(0.0, 1 - remain))
        return False

    def _on_finish(self):
        try:
            self.ctx.chassis.set_velocity(0.0, 0.0)
        except Exception:
            pass
        if self._det is not None:
            self._det.disable()
