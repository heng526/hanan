"""P3 场景分割接口：地面、障碍与车厢信息由注入处理器合并发布。"""

import math

from .base_perception import BasePerception


class SceneSegmenter(BasePerception):
    NAME = "scene_segmenter"

    def __init__(self, source=None, processor=None, period=0.1):
        super().__init__(period=period)
        self.source = source
        self.processor = processor

    def _infer_once(self) -> dict:
        frame, error = self._sample(self.source)
        if error or self.processor is None:
            return {"walkable_area_ratio": 0.0, "step_height": 999.0,
                    "safety_status": "UNKNOWN", "risk_level": "UNKNOWN",
                    "car_detected": False,
                    "error_code": error or "E_ALGORITHM_UNAVAILABLE"}
        try:
            raw = self.processor(frame, self._parameters())
        except Exception:
            return {"walkable_area_ratio": 0.0, "step_height": 999.0,
                    "safety_status": "UNKNOWN", "risk_level": "UNKNOWN",
                    "car_detected": False, "error_code": "E_INFER_CRASH"}
        if not isinstance(raw, dict) or raw.get("error_code") is not None:
            return {"walkable_area_ratio": 0.0, "step_height": 999.0,
                    "safety_status": "UNKNOWN", "risk_level": "UNKNOWN",
                    "car_detected": False,
                    "error_code": raw.get("error_code") if isinstance(raw, dict)
                    else "E_LOW_QUALITY"}
        try:
            ratio = float(raw.get("walkable_area_ratio", 0.0))
            if not math.isfinite(ratio) or not 0.0 <= ratio <= 1.0:
                ratio = 0.0
        except (TypeError, ValueError):
            ratio = 0.0
        try:
            step = float(raw.get("step_height", 999.0))
            if not math.isfinite(step) or step < 0:
                step = 999.0
        except (TypeError, ValueError):
            step = 999.0
        return {
            "walkable_area_mask": raw.get("walkable_area_mask"),
            "walkable_area_ratio": ratio,
            "path_centerline": raw.get("path_centerline"),
            "lateral_deviation": raw.get("lateral_deviation"),
            "step_height": step,
            "safety_status": raw.get("safety_status", "UNKNOWN"),
            "obstacle_list": raw.get("obstacle_list", []),
            "nearest_distance": raw.get("nearest_distance"),
            "risk_level": raw.get("risk_level", "UNKNOWN"),
            "free_space_left": raw.get("free_space_left"),
            "free_space_right": raw.get("free_space_right"),
            "car_detected": bool(raw.get("car_detected", False)),
            "distance_to_car": raw.get("distance_to_car"),
            "lateral_error": raw.get("lateral_error"),
            "yaw_error": raw.get("yaw_error"),
            "car_no_text": raw.get("car_no_text"),
            "error_code": raw.get("error_code"),
        }
