"""A8 低位排风任务类；下蹲/执行接口由现有行为与上下文负责。"""

from .vent import VentAction
from ..base_action import ActionState


class LowVentAction(VentAction):
    NAME = "vent_low"

    def __init__(self, ctx, lever_pose=None, exec_timeout=5.0, period=0.1):
        super().__init__(ctx, mode="low", lever_pose=lever_pose,
                         exec_timeout=exec_timeout, period=period)

    def _run_once(self):
        if self.ctx.manipulator is None:
            self._set(state=ActionState.FAIL, error="E_MANIPULATOR_UNAVAILABLE")
            return True
        return super()._run_once()
