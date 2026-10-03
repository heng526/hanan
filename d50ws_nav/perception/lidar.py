"""P4 雷达处理接口：不直接接 SDK，注入点云源与处理器。"""

from .base_perception import BasePerception


class LidarProcessor(BasePerception):
    NAME = "lidar_processor"

    def __init__(self, source=None, processor=None, period=0.1):
        super().__init__(period=period)
        self.source = source
        self.processor = processor

    def _infer_once(self) -> dict:
        scan, error = self._sample(self.source)
        if error or self.processor is None:
            return {"sector_scan": {}, "obstacle_list": [],
                    "risk_level": "UNKNOWN",
                    "error_code": error or "E_ALGORITHM_UNAVAILABLE"}
        raw = self.processor(scan, self._parameters())
        if not isinstance(raw, dict):
            return {"sector_scan": {}, "obstacle_list": [],
                    "risk_level": "UNKNOWN", "error_code": "E_LOW_QUALITY"}
        return {
            "obstacle_list": raw.get("obstacle_list", []),
            "nearest_distance": raw.get("nearest_distance"),
            "sector_scan": raw.get("sector_scan", {}),
            "risk_level": raw.get("risk_level", "UNKNOWN"),
            "error_code": raw.get("error_code"),
        }
