# -*- coding: utf-8 -*-
"""运行上下文：动作类与总控共享的句柄集合（依赖注入）。

动作类通过 ctx 使用底盘/定位/感知/安全，不直接 import SDK——
保证任何动作类都能用 Mock 上下文脱离总控、脱离真机单独测试。
"""
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PerceptionBundle:
    """感知类集合（L2 四类，均为 BasePerception 子类或 Mock）。"""
    lever: Optional[object] = None    # P1 拉杆检测
    pipe: Optional[object] = None     # P2 风管检测（含摘管成功判定）
    scene: Optional[object] = None    # P3 场景分割（地面/障碍/车厢）
    lidar: Optional[object] = None    # P4 激光雷达处理


@dataclass
class RunContext:
    chassis: Optional[object] = None      # RobotApi（真机 LingSiTcpRobot / Mock）
    rtk: Optional[object] = None          # RtkTracker（位置真值）；None=里程计降级
    safety: Optional[object] = None       # SafetyMonitor
    perception: Optional[object] = None   # PerceptionBundle（按需 enable/disable）
    config: Optional[object] = None       # 航点/车型配置表
    manipulator: Optional[object] = None  # 机械臂作业模块（家康，接口对接 A7/A8）
    rod: Optional[object] = None          # 挑杆作业模块（接口对接 A9）
    acoustic: Optional[object] = None     # 排风声纹检测（P5，CHECK 类）

    # 附加服务（总控填充，动作类只读）
    task_id: str = ""
    robot_id: str = "dog01"
