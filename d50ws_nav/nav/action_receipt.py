"""L3 执行回执的最小关联契约；不接触厂商 SDK。"""

import math
import time


def _current_timestamp(value, started_at):
    return (type(value) in (int, float) and math.isfinite(value)
            and started_at <= value <= time.time())


def accepted_command(reply, started_at):
    """返回本次命令 ID；bool/无时间戳的旧式返回不能证明已接受。"""
    if (not isinstance(reply, dict) or reply.get("accepted") is not True
            or not isinstance(reply.get("command_id"), str)
            or not reply["command_id"].strip()
            or not _current_timestamp(reply.get("timestamp"), started_at)):
        return None
    return reply["command_id"]


def command_state(reply, command_id, started_at):
    """仅认可本次命令在开始后的回执；不匹配/过期的返回 None。"""
    if (not isinstance(reply, dict) or reply.get("command_id") != command_id
            or not _current_timestamp(reply.get("timestamp"), started_at)):
        return None
    state = reply.get("state")
    return state if state in ("RUNNING", "DONE", "FAIL", "ABORTED") else None


def perception_fresh(state, *, max_age_s=1.0, allow_not_found=False):
    """只检查缓存证据；未检出目标可作为接近动作的有效空帧。"""
    if not isinstance(state, dict):
        return False
    age = state.get("age_s")
    allowed = (None, "E_TARGET_NOT_FOUND") if allow_not_found else (None,)
    return (state.get("enabled") is True and type(state.get("frame_count")) is int
            and state["frame_count"] > 0 and type(age) in (int, float)
            and math.isfinite(age) and 0 <= age <= max_age_s
            and state.get("error_code") in allowed)


def cancel_command(actuator, command_id):
    """仅请求已注入执行器取消；是否实际停稳需由现场回执确认。"""
    cancel = getattr(actuator, "stop", None)
    if callable(cancel) and command_id is not None:
        try:
            cancel(command_id)
        except Exception:
            pass
