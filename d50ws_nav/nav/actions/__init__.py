# -*- coding: utf-8 -*-
"""行为层动作类统一出口（动作类清单 A1~A10）。"""
from .cruise import CruiseAction          # A1 巡航避障（01）
from .cross_track import CrossTrackAction  # A2 过轨道（02）
from .find_start import FindWorkStartAction  # A3 寻找作业起点（03）
from .approach import ApproachAction       # A5 作业接近
from .align import AlignAction             # A6 对位 high/low/detach
from .vent import VentAction               # A7/A8 排风动作 high/low
from .vent_high import HighVentAction       # A7 高位排风
from .vent_low import LowVentAction         # A8 低位排风
from .detach import DetachAction           # A9 摘管动作（05）
from .retreat import RetreatAction         # A10 退出 ×3 变体

__all__ = [
    "CruiseAction", "CrossTrackAction", "FindWorkStartAction", "ApproachAction",
    "AlignAction", "VentAction", "HighVentAction", "LowVentAction",
    "DetachAction", "RetreatAction",
    "create_action",
]

# 只做类构造；action_list 顺序、重试和跳过仍由 L4 总控负责。
ACTION_CLASSES = {
    "cruise": CruiseAction,
    "cross_track": CrossTrackAction,
    "find_start": FindWorkStartAction,
    "approach": ApproachAction,
    "align": AlignAction,
    "vent_high": HighVentAction,
    "vent_low": LowVentAction,
    "detach": DetachAction,
    "retreat": RetreatAction,
}


def create_action(name, ctx, **kwargs):
    """按行为名创建一个独立动作实例，调用方再 start()/read()/stop()。"""
    try:
        cls = ACTION_CLASSES[name]
    except KeyError as exc:
        raise ValueError("未知行为动作: %s" % name) from exc
    return cls(ctx, **kwargs)
