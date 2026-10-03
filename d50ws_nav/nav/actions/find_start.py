# -*- coding: utf-8 -*-
"""A3 FindWorkStartAction 寻找作业起点/找车头（动作类清单 #A3）。

扫车 → OCR 车号与计划交叉校验 → 锁定车头 → 查车型配置表（拉杆位置写死）
→ 输出排风类型与作业点推算依据。车型不符继续前找，超时 FAIL。
"""
import time

from ..base_action import BaseAction, ActionState

# 车型配置表（现场最终以数据库 car_info/system_config 为准，此处为内置兜底）
DEFAULT_CAR_TABLE = {
    "C62":  {"air_type": "high", "lever_offset": 1.2, "car_length": 12.0},
    "C62A": {"air_type": "high", "lever_offset": 1.2, "car_length": 12.5},
    "C62B": {"air_type": "low",  "lever_offset": 1.0, "car_length": 12.5},
    "C64":  {"air_type": "low",  "lever_offset": 0.9, "car_length": 13.0},
}


class FindWorkStartAction(BaseAction):
    NAME = "find_start"

    def __init__(self, ctx, expect_car_no: str, drive_speed: float = 0.25,
                 scan_timeout: float = 60.0, car_table: dict = None,
                 period: float = 0.1):
        super().__init__(ctx, period=period)
        self._expect_no = expect_car_no
        self._speed = drive_speed
        self._timeout = scan_timeout
        self._table = car_table or DEFAULT_CAR_TABLE
        self._t0 = None

    def _scene(self):
        return self._perception_get("scene")

    def _setup(self):
        self._perception_set("scene", True)

    def _on_finish(self):
        self._perception_set("scene", False)

    def _run_once(self) -> bool:
        if self._t0 is None:
            self._t0 = time.time()
        if time.time() - self._t0 > self._timeout:
            self._set(state=ActionState.FAIL, error="FAIL_CARNO_MISMATCH")
            return True

        ch = self.ctx.chassis
        ch.move_vector(self._speed, 0.0, 0.0)
        step = getattr(ch, "step", None)
        if callable(step):
            step()

        scene = self._scene()
        if scene is None:
            self._set(state=ActionState.FAIL, error="E_SCENE_UNAVAILABLE")
            return True
        st = scene.read()
        self._set(progress=0.5, data={"scan_text": st.get("car_no_text")})

        if not st.get("car_detected"):
            return False

        text = st.get("car_no_text") or ""
        if self._expect_no and self._expect_no not in text:
            return False                                  # 车号不符，继续前找

        # 命中：解析车型 → 查表
        car_type = text.split()[0] if text.split() else "UNKNOWN"
        info = self._table.get(car_type)
        if info is None:
            self._set(state=ActionState.FAIL, error="FAIL_UNKNOWN_CAR_TYPE")
            return True
        ch.set_velocity(0.0, 0.0)
        pose = ch.get_pose()
        self._set(state=ActionState.SUCCESS, progress=1.0,
                  data={"car_no": self._expect_no, "car_type": car_type,
                        "air_type": info["air_type"],
                        "lever_offset": info["lever_offset"],
                        "car_length": info["car_length"],
                        "head_pose": [round(pose.e, 3), round(pose.n, 3),
                                      round(pose.yaw, 3)]})
        return True
