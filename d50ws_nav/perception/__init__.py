"""L2 四个感知任务类；算法与传感器从外部注入。"""

from .lever import LeverDetector
from .pipe import PipeDetector
from .scene import SceneSegmenter
from .lidar import LidarProcessor
from .camera_source import CameraSource

__all__ = ["LeverDetector", "PipeDetector", "SceneSegmenter", "LidarProcessor",
           "CameraSource"]
