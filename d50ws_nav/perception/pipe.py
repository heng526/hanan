"""P2 风管检测及摘管后两端点距离判定；真实分割器由外部实现方注入。"""

import math

from .base_perception import BasePerception


class PipeDetector(BasePerception):
    NAME = "pipe_detector"

    def __init__(self, source=None, processor=None, to_dog=None,
                 vector_to_dog=None, period=0.1):
        super().__init__(period=period)
        self.source = source
        self.processor = processor
        self.to_dog = to_dog
        self.vector_to_dog = vector_to_dog

    def _infer_once(self) -> dict:
        frame, error = self._sample(self.source)
        if error or self.processor is None:
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": error or "E_ALGORITHM_UNAVAILABLE"}
        raw = self.processor(frame, self._parameters())
        if not isinstance(raw, dict) or not raw.get("pipe_detected"):
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": "E_TARGET_NOT_FOUND"}

        joint, error = self._point_dog(raw.get("joint_point_camera"), self.to_dog)
        if error:
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": error}
        hook, error = self._point_dog(raw.get("hook_point_camera"), self.to_dog)
        if error:
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": error}
        lift, error = self._point_dog(
            raw.get("lift_direction_camera"), self.vector_to_dog)
        if error:
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": error}
        separate, error = self._point_dog(
            raw.get("separate_direction_camera"), self.vector_to_dog)
        if error:
            return {"pipe_detected": False, "connector_detected": False,
                    "error_code": error}

        result = {
            "pipe_detected": True,
            "connector_detected": bool(raw.get("connector_detected", False)),
            "centerline_uv": raw.get("centerline_uv"),
            "joint_point_dog": joint,
            "hook_point_dog": hook,
            "lift_direction_dog": lift,
            "separate_direction_dog": separate,
            "confidence": raw.get("confidence"),
            "quality_score": raw.get("quality_score"),
            "error_code": None,
        }
        points = raw.get("separation_points_camera")
        if isinstance(points, (list, tuple)) and len(points) == 2:
            converted = [self._point_dog(p, self.to_dog) for p in points]
            if all(error is None for _, error in converted):
                result["separation_points_dog"] = [p for p, _ in converted]
        return result

    def check_detach(self, threshold: float = 0.3, max_age_s: float = 1.0) -> dict:
        """CHECK：最新有效帧恰有两个端点且点距严格大于阈值才成功。"""
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError("distance_threshold 必须为正数")
        state = self.read()
        evidence = {"frame_count": state["frame_count"],
                    "timestamp": state["timestamp"]}
        points = state.get("separation_points_dog")
        if (not state["enabled"] or state.get("error_code") is not None
                or state["age_s"] is None or state["age_s"] > max_age_s
                or not isinstance(points, list) or len(points) != 2):
            return {"detach_success": False, "separation_distance": None,
                    "separation_angle": None, "reconnect_risk": True,
                    "error_code": "E_TARGET_NOT_FOUND", **evidence}
        distance = math.dist(points[0], points[1])
        success = distance > threshold
        return {"detach_success": success, "separation_distance": distance,
                "separation_angle": None, "reconnect_risk": not success,
                "error_code": None, **evidence}
