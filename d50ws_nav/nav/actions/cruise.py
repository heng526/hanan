# -*- coding: utf-8 -*-
"""A1 CruiseAction 巡航避障（动作类清单 #A1）——行为层第一个示范实现。

展示统一范式：实例化 → start() → 内部线程跑跟随器 → 外部 read()/wait() 非阻塞。

用法（含单独测试，不依赖总控）：
    from nav.context import RunContext
    from nav.actions.cruise import CruiseAction
    from sdk_adapter.mock import MockRobot

    ctx = RunContext(chassis=MockRobot())        # 真机换 LingSiTcpRobot()
    act = CruiseAction(ctx, end=(30.0, 0.0))
    act.start()
    while not act.wait(timeout=30):              # 或每周期 act.read() 刷 UI
        ...
"""
import math
import time

from ..base_action import BaseAction, ActionState
from ..follower import Waypoint, WaypointFollower, FollowParams
from ..safety import Verdict


class CruiseAction(BaseAction):
    NAME = "cruise"

    def __init__(self, ctx, end, speed_limit: float = 1.0, pos_tol: float = 0.3,
                 heading: float = None, yaw_tol: float = 0.35, hold_time: float = 0.0,
                 period: float = 0.1):
        super().__init__(ctx, period=period)
        self._end = end
        self._wp = Waypoint(
            name="cruise_target", e=end[0], n=end[1],
            type="TRANSIT", speed_limit=speed_limit,
            pos_tol=pos_tol, heading=heading, yaw_tol=yaw_tol, hold_time=hold_time,
        )

    def _setup(self):
        self._follower = WaypointFollower([self._wp], FollowParams())
        self._start_pose = self.ctx.chassis.get_pose()
        self._total = max(0.1, self._start_pose.dist_to(self._wp.e, self._wp.n))
        self.ctx.chassis.stand_up()

    def _run_once(self) -> bool:
        now = time.time()

        # 1) 安全门禁：STOP 立即 FAIL（总控决定重试/跳过）
        if self.ctx.safety is not None:
            verdict = self.ctx.safety.update(self.ctx.chassis.telemetry(),
                                             wp_type="TRANSIT", now=now)
            if verdict.level == Verdict.STOP:
                self._set(state=ActionState.FAIL,
                          error="safety_stop:%s" % ",".join(verdict.reasons))
                return True
            self._set(data={"safety": verdict.level.value})

        # 2) 跟随器单周期推进
        state = self._follower.step(self.ctx.chassis, now=now)

        # 3) 仿真推进（Mock 需要；真狗无此方法自动跳过）
        step = getattr(self.ctx.chassis, "step", None)
        if callable(step):
            step()

        # 3) 进度与状态刷新
        pose = self.ctx.chassis.get_pose()
        remain = pose.dist_to(self._wp.e, self._wp.n)
        progress = 1.0 - min(1.0, remain / self._total)
        self._set(progress=progress,
                  data={"remain_m": round(remain, 3),
                        "pose": [round(pose.e, 3), round(pose.n, 3), round(pose.yaw, 3)]})

        # 4) 终态判定
        if state.name == "DONE":
            self._set(state=ActionState.SUCCESS, progress=1.0,
                      data={"final_pose": [round(pose.e, 3), round(pose.n, 3),
                                           round(pose.yaw, 3)]})
            return True
        if state.name in ("FAIL",):
            self._set(state=ActionState.FAIL, error="follower_fail")
            return True
        return False
