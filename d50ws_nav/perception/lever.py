"""P1 拉杆检测接口：帧源与推理器由外部实现方实现注入，本类只发布狗系结果。"""

import math

from .base_perception import BasePerception


class LeverDetector(BasePerception):
    NAME = "lever_detector"

    def __init__(self, source=None, processor=None, to_dog=None, period=0.1):
        super().__init__(period=period)
        self.source = source             # read() -> (ret, frame, timestamp)
        self.processor = processor       # (frame, params) -> 检测字典
        self.to_dog = to_dog             # 相机系点 -> 狗系点，必须使用实测外参

    def _infer_once(self) -> dict:
        frame, error = self._sample(self.source)
        if error or self.processor is None:
            return {"track_status": "FAILED", "target_detected": False,
                    "error_code": error or "E_ALGORITHM_UNAVAILABLE"}

        params = self._parameters()
        raw = self.processor(frame, params)
        if not isinstance(raw, dict) or not raw.get("target_detected"):
            return {"track_status": "LOST", "target_detected": False,
                    "error_code": "E_TARGET_NOT_FOUND"}
        try:
            confidence = float(raw.get("confidence", 0.0))
            minimum = float(params.get("confidence_threshold", 0.0))
        except (TypeError, ValueError):
            confidence, minimum = 0.0, 1.0
        if not math.isfinite(confidence) or confidence < minimum:
            return {"track_status": "FAILED", "target_detected": False,
                    "error_code": "E_LOW_QUALITY"}

        point, error = self._point_dog(raw.get("target_point_camera"), self.to_dog)
        if error:
            return {"track_status": "FAILED", "target_detected": False,
                    "error_code": error}
        return {
            "track_status": "TRACKING",
            "target_detected": True,
            "lever_type": raw.get("lever_type", params.get("lever_type_hint")),
            "bbox": raw.get("bbox"),
            "corner_points_uv": raw.get("corner_points_uv"),
            "target_point_dog": point,
            "target_distance": math.sqrt(sum(x * x for x in point)),
            "confidence": confidence,
            "quality_score": raw.get("quality_score"),
            "error_code": None,
        }
