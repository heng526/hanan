# -*- coding: utf-8 -*-
"""感知 Mock 集合：世界系闭环仿真——狗动了，相对坐标真实变化。

设计：目标在世界系固定一点，Mock 每帧根据底盘当前位姿（里程计）把
世界点变换到狗系输出。因此 对位/接近 类动作的闭环在仿真里真实收敛。
真实实现（P1~P4）替换时接口不变，仅数据来源换成相机/雷达推理。
"""
import math

from .base_perception import BasePerception


def world_to_dog(pose, wx: float, wy: float):
    """世界点 → 狗系（x前/y左）。pose 需有 e,n,yaw。"""
    ex, ey = wx - pose.e, wy - pose.n
    cos_y, sin_y = math.cos(pose.yaw), math.sin(pose.yaw)
    return (cos_y * ex + sin_y * ey, -sin_y * ex + cos_y * ey)


class MockLeverDetector(BasePerception):
    """P1 拉杆检测 Mock。

    lever_world: 拉杆在世界系位置 (e, n)，高度 h 固定。
    detect_range: 进入该距离(米)后才"看见"。
    """
    NAME = "lever_detector_mock"

    def __init__(self, chassis=None, lever_world=(1.2, 0.0), height=0.9,
                 detect_range=2.0, period=0.1, min_frames=3):
        super().__init__(period=period)
        self.chassis = chassis
        self.lever_world = lever_world
        self.height = height
        self.detect_range = detect_range
        self.min_frames = min_frames

    def _infer_once(self) -> dict:
        pose = self.chassis.get_pose()
        xd, yd = world_to_dog(pose, *self.lever_world)
        dist = math.hypot(xd, yd)
        if self._frame_count < self.min_frames or dist > self.detect_range:
            return {"track_status": "SEARCHING", "target_detected": False,
                    "error_code": "E_TARGET_NOT_FOUND"}
        return {
            "track_status": "TRACKING", "target_detected": True,
            "lever_type": "lever_high",
            "bbox": [280, 200, 360, 300],
            "target_point_dog": [round(xd, 4), round(yd, 4), self.height],
            "target_distance": round(math.hypot(dist, self.height), 3),
            "confidence": 0.91, "quality_score": 0.88, "error_code": None,
        }


class MockPipeDetector(BasePerception):
    """P2 风管检测 Mock（含摘管成功判定 check_detach）。"""
    NAME = "pipe_detector_mock"

    def __init__(self, chassis=None, pipe_world=(1.5, 0.0), detect_range=2.0,
                 period=0.1, min_frames=3, detach_distance=0.5):
        super().__init__(period=period)
        self.chassis = chassis
        self.pipe_world = pipe_world
        self.detect_range = detect_range
        self.min_frames = min_frames
        self.detach_distance = detach_distance     # check_detach 的返回值

    def _infer_once(self) -> dict:
        pose = self.chassis.get_pose()
        xd, yd = world_to_dog(pose, *self.pipe_world)
        dist = math.hypot(xd, yd)
        if self._frame_count < self.min_frames or dist > self.detect_range:
            return {"track_status": "SEARCHING", "pipe_detected": False,
                    "connector_detected": False, "error_code": "E_TARGET_NOT_FOUND"}
        return {
            "track_status": "TRACKING", "pipe_detected": True,
            "connector_detected": True,
            "joint_point_dog": [round(xd, 4), round(yd, 4), 0.45],
            "hook_point_dog": [round(xd, 4), round(yd, 4), 0.42],
            "lift_direction_dog": [0.2, 0.0, 0.98],
            "separate_direction_dog": [0.0, 1.0, 0.0],
            "confidence": 0.9, "quality_score": 0.87, "error_code": None,
        }

    def check_detach(self, threshold: float = 0.3) -> dict:
        """摘管成功判定（纪要决策：点数+点距）。Mock 按预设距离返回。"""
        state = self.read()
        ok = self.detach_distance >= threshold
        return {"detach_success": ok,
                "separation_distance": self.detach_distance,
                "separation_angle": 95.0 if ok else 40.0,
                "reconnect_risk": not ok,
                "frame_count": state["frame_count"],
                "timestamp": state["timestamp"]}


class MockSceneSegmenter(BasePerception):
    """P3 场景分割 Mock（车厢+OCR / 可通行区域）。"""
    NAME = "scene_segmenter_mock"

    def __init__(self, chassis=None, car_world=(3.0, 0.0), detect_range=2.5,
                 car_no_text="C62 45001", period=0.1, min_frames=3,
                 walkable_ratio=0.85, step_height=0.05):
        super().__init__(period=period)
        self.chassis = chassis
        self.car_world = car_world
        self.detect_range = detect_range
        self.car_no_text = car_no_text
        self.min_frames = min_frames
        self.walkable_ratio = walkable_ratio
        self.step_height = step_height

    def _car_part(self):
        if self.chassis is None:
            return self._frame_count >= self.min_frames, 2.0
        pose = self.chassis.get_pose()
        xd, yd = world_to_dog(pose, *self.car_world)
        dist = math.hypot(xd, yd)
        return (self._frame_count >= self.min_frames and dist <= self.detect_range,
                round(dist, 3))

    def _infer_once(self) -> dict:
        car_seen, car_dist = self._car_part()
        return {
            # 地面/可通行
            "walkable_area_ratio": self.walkable_ratio,
            "path_centerline": [0.0, 0.0],
            "lateral_deviation": 0.02,
            "step_height": self.step_height,
            "safety_status": "OK",
            # 障碍（融合 P4 前的本地判断，Mock 置空）
            "obstacle_list": [], "nearest_distance": None, "risk_level": "NONE",
            # 车厢组
            "car_detected": car_seen, "distance_to_car": car_dist,
            "lateral_error": 0.05, "yaw_error": 0.02,
            "car_no_text": self.car_no_text if car_seen else None,
            "error_code": None,
        }


class MockLidarProcessor(BasePerception):
    """P4 激光雷达 Mock。属性 left_block/right_block 可在测试中置 True
    模拟来车（扇区距离骤降+动态标志）。"""
    NAME = "lidar_mock"

    def __init__(self, period=0.1, clear_distance=50.0):
        super().__init__(period=period)
        self.clear_distance = clear_distance
        self.left_block = False
        self.right_block = False
        self.front_block = False

    def _sector(self, blocked: bool):
        if blocked:
            return {"nearest_distance": 5.0, "dynamic": True}
        return {"nearest_distance": self.clear_distance, "dynamic": False}

    def _infer_once(self) -> dict:
        return {
            "sector_scan": {"left": self._sector(self.left_block),
                            "right": self._sector(self.right_block),
                            "front": self._sector(self.front_block)},
            "obstacle_list": [], "nearest_distance": self.clear_distance,
            "risk_level": "NONE", "error_code": None,
        }
