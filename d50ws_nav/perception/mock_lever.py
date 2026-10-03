# -*- coding: utf-8 -*-
"""P1 LeverDetector 拉杆检测——Mock 实现（演示/测试接口契约）。

真实实现：接 perception_v1/rod 的 YOLO 分割推理 + 深度解算 + 相机系→狗系 TF。
Mock 逻辑：enable 后模拟"逐渐靠近→目标进入视野→输出固定相对坐标"。
"""
import math
from .base_perception import BasePerception


class MockLeverDetector(BasePerception):
    NAME = "lever_detector_mock"

    def __init__(self, period: float = 0.1, appear_after_frames: int = 15):
        super().__init__(period=period)
        self._appear_after = appear_after_frames
        # 模拟的狗系坐标（x前/y左/z上）：拉杆在前方 1.2m、左偏 0.3m、高 0.9m
        self._target = (1.2, 0.3, 0.9)

    def _infer_once(self) -> dict:
        n = self._frame_count
        if n < self._appear_after:
            return {"track_status": "SEARCHING", "target_detected": False,
                    "error_code": "E_TARGET_NOT_FOUND"}
        # 距离随帧数缓缓接近（模拟靠近过程）
        approach = max(0.4, self._target[0] - 0.02 * (n - self._appear_after))
        return {
            "track_status": "TRACKING",
            "target_detected": True,
            "lever_type": "lever_high",
            "bbox": [280, 200, 360, 300],
            "target_point_dog": [round(approach, 3), self._target[1], self._target[2]],
            "target_distance": round(math.sqrt(approach**2 + 0.3**2 + 0.9**2), 3),
            "confidence": 0.91,
            "quality_score": 0.88,
            "error_code": None,
        }
