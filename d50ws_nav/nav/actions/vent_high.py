"""A7 高位排风任务类；执行机构仍通过上下文注入。"""

from .vent import VentAction
from ..base_action import ActionState


class HighVentAction(VentAction):
    NAME = "vent_high"

    def __init__(self, ctx, lever_pose=None, exec_timeout=5.0, period=0.1):
        super().__init__(ctx, mode="high", lever_pose=lever_pose,
                         exec_timeout=exec_timeout, period=period)

    def _run_once(self):
        if self.ctx.manipulator is None:
            self._set(state=ActionState.FAIL, error="E_MANIPULATOR_UNAVAILABLE")
            return True
        return super()._run_once()
