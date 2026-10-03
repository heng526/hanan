# -*- coding: utf-8 -*-
"""A5 ApproachAction 作业接近（动作类清单 #A5）。

沿车厢低速行进，目标感知类（拉杆/风管）一旦检出即停车——
对应"允许 1~1.5m 误差前进、视觉看见就停"的环节，是粗对位的前置。
"""
import time
import math

from ..base_action import BaseAction, ActionState
from ..action_receipt import perception_fresh


class ApproachAction(BaseAction):
    NAME = "approach"

    def __init__(self, ctx, detector_name: str = "lever", drive_speed: float = 0.25,
                 min_confidence: float = 0.5, drive_timeout: float = 60.0,
                 period: float = 0.1, sensor_max_age: float = 1.0):
        super().__init__(ctx, period=period)
        self._det_name = detector_name
        self._speed = drive_speed
        self._min_conf = min_confidence
        self._timeout = drive_timeout
        self._sensor_max_age = sensor_max_age
        self._det = None
        self._t0 = None
        self._standing = False
        self._last_frame_count = 0

    def _setup(self):
        bundle = self.ctx.perception
        self._det = getattr(bundle, self._det_name, None) if bundle else None
        if self._det is not None:
            self._det.enable()
        self._t0 = time.time()

    def _run_once(self) -> bool:
        if time.time() - self._t0 > self._timeout:
            self._set(state=ActionState.FAIL, error="FAIL_APPROACH_TIMEOUT")
            return True

        if self._det is None:
            self._set(state=ActionState.FAIL, error="E_DETECTOR_UNAVAILABLE")
            return True
        try:
            st = self._det.read()
        except Exception:
            st = None
        ch = self.ctx.chassis
        if not perception_fresh(st, max_age_s=self._sensor_max_age,
                                allow_not_found=True):
            ch.set_velocity(0.0, 0.0)
            self._set(data={"phase": "HOLD", "reason": "E_DETECTOR_DATA_INVALID"})
            return False
        if st["frame_count"] <= self._last_frame_count:
            ch.set_velocity(0.0, 0.0)
            self._set(data={"phase": "HOLD", "reason": "E_DETECTOR_FRAME_REPEATED"})
            return False
        self._last_frame_count = st["frame_count"]
        if st.get("target_detected") or st.get("pipe_detected"):
            confidence = st.get("confidence")
            if (type(confidence) in (int, float) and math.isfinite(confidence)
                    and confidence >= self._min_conf):
                ch.set_velocity(0.0, 0.0)
                tgt = st.get("target_point_dog") or st.get("hook_point_dog")
                if (not isinstance(tgt, (list, tuple)) or len(tgt) < 3
                        or any(type(x) not in (int, float) or not math.isfinite(x)
                               for x in tgt[:3])):
                    self._set(state=ActionState.FAIL, error="E_TARGET_INVALID")
                    return True
                self._set(state=ActionState.SUCCESS, progress=1.0,
                          data={"target_point_dog": tgt,
                                "target_distance": st.get("target_distance"),
                                "detector": self._det_name})
                return True
            ch.set_velocity(0.0, 0.0)
            self._set(data={"phase": "HOLD", "reason": "E_LOW_QUALITY"})
            return False
        if not self._standing:
            ch.stand_up()
            self._standing = True
        ch.move_vector(self._speed, 0.0, 0.0)
        step = getattr(ch, "step", None)
        if callable(step):
            step()
        self._set(data={"scan": st.get("track_status")})
        return False

    def _on_finish(self):
        if self._det is not None:
            self._det.disable()
