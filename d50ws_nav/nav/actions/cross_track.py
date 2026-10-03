# -*- coding: utf-8 -*-
"""A2 CrossTrackAction 过轨道（动作类清单 #A2）。

左看右看向前看：左/右看用雷达扇区判断来车（距离+动态），前看用场景分割
确认过线板/落足区；通过后恢复模式。有车逼近 → 等待，超时 FAIL。
"""
import math
import time

from ..base_action import BaseAction, ActionState


class CrossTrackAction(BaseAction):
    NAME = "cross_track"

    def __init__(self, ctx, exit_point, cross_speed: float = 0.3,
                 arrive_tol: float = 0.3, train_safe_distance: float = 10.0,
                 wait_timeout: float = 120.0, cross_timeout: float = 60.0,
                 max_step_height: float = 0.35, period: float = 0.1,
                 sensor_ready_timeout: float = 1.0,
                 sensor_max_age: float = 1.0):
        super().__init__(ctx, period=period)
        if sensor_ready_timeout <= 0 or sensor_max_age <= 0:
            raise ValueError("感知就绪超时和数据有效期必须为正数")
        self._exit = exit_point
        self._cross_speed = cross_speed
        self._arrive_tol = arrive_tol
        self._train_safe = train_safe_distance
        self._wait_timeout = wait_timeout
        self._cross_timeout = cross_timeout
        self._max_step = max_step_height
        self._phase = "LOOK_LEFT"
        self._wait_since = None
        self._cross_start = None
        self._sensor_ready_timeout = sensor_ready_timeout
        self._sensor_max_age = sensor_max_age
        self._sensor_wait_since = {}
        self._terrain_switched = False

    def _lidar(self):
        return getattr(self.ctx.perception, "lidar", None) if self.ctx.perception else None

    def _scene(self):
        return getattr(self.ctx.perception, "scene", None) if self.ctx.perception else None

    def _sector_blocked(self, sector: dict) -> bool:
        return sector["nearest_distance"] < self._train_safe and sector["dynamic"]

    def _sensor_state(self, name):
        """仅接受已启用且近期成功刷新的感知缓存；冷启动允许有限等待。"""
        sensor = self._perception_get(name)
        if sensor is None:
            self._set(state=ActionState.FAIL, error="E_%s_UNAVAILABLE" % name.upper())
            return None, True
        try:
            st = sensor.read()
        except Exception:
            st = None
        if isinstance(st, dict):
            age = st.get("age_s")
            fresh = (type(age) in (int, float) and math.isfinite(age)
                     and 0 <= age <= self._sensor_max_age)
            if (st.get("enabled") is True and st.get("frame_count", 0) > 0
                    and "error_code" in st and st["error_code"] is None and fresh):
                self._sensor_wait_since.pop(name, None)
                return st, False
            warming = (st.get("enabled") is True and age is None
                       and st.get("error_code") == "E_SENSOR_NO_DATA")
            if warming:
                now = time.time()
                since = self._sensor_wait_since.setdefault(name, now)
                if now - since < self._sensor_ready_timeout:
                    self._set(data={"wait_sensor": name})
                    return None, False
                self._set(state=ActionState.FAIL,
                          error="E_%s_READY_TIMEOUT" % name.upper())
                return None, True
        self._set(state=ActionState.FAIL, error="E_%s_DATA_INVALID" % name.upper())
        return None, True

    @staticmethod
    def _valid_sector(sector):
        if not isinstance(sector, dict) or type(sector.get("dynamic")) is not bool:
            return False
        distance = sector.get("nearest_distance")
        return (type(distance) in (int, float) and math.isfinite(distance)
                and distance >= 0)

    def _scene_clear(self, st):
        ratio, step = st.get("walkable_area_ratio"), st.get("step_height")
        if (type(ratio) not in (int, float) or not math.isfinite(ratio)
                or not 0 <= ratio <= 1 or type(step) not in (int, float)
                or not math.isfinite(step) or step < 0):
            self._set(state=ActionState.FAIL, error="E_SCENE_DATA_INVALID")
            return False
        if (st.get("safety_status") != "OK" or st.get("risk_level") != "NONE"
                or ratio < 0.5 or step > self._max_step):
            self._set(state=ActionState.FAIL, error="FAIL_NO_CROSSBOARD")
            return False
        return True

    def _look(self, side: str, progress: float) -> bool:
        st, failed = self._sensor_state("lidar")
        if st is None:
            return failed
        sectors = st.get("sector_scan")
        sector = sectors.get(side) if isinstance(sectors, dict) else None
        if not self._valid_sector(sector):
            self._set(state=ActionState.FAIL, error="E_LIDAR_SECTOR_INVALID")
            return True
        if st.get("risk_level") != "NONE":
            self._set(state=ActionState.FAIL, error="E_LIDAR_RISK")
            return True
        if self._sector_blocked(sector):
            now = time.time()
            if self._wait_since is None:
                self._wait_since = now
            self._set(data={"wait_side": side,
                            "blocked_distance": sector["nearest_distance"]})
            if now - self._wait_since > self._wait_timeout:
                self._set(state=ActionState.FAIL, error="FAIL_TRAIN_APPROACHING")
                return True
            return False                           # 继续等
        self._wait_since = None
        self._phase = "LOOK_RIGHT" if side == "left" else "LOOK_FRONT"
        self._set(progress=progress)
        return False

    def _run_once(self) -> bool:
        ch = self.ctx.chassis
        pose = ch.get_pose()

        if self._phase == "LOOK_LEFT":
            return self._look("left", 0.15)

        if self._phase == "LOOK_RIGHT":
            return self._look("right", 0.3)

        if self._phase == "LOOK_FRONT":
            st, failed = self._sensor_state("scene")
            if st is None:
                return failed
            if not self._scene_clear(st):
                return True
            ch.set_terrain_mode("wheel_leg_rl")     # ★ M0 验证：越障步态候选
            self._terrain_switched = True
            self._cross_start = time.time()
            self._phase = "CROSS"
            self._set(progress=0.45)
            return False

        if self._phase == "CROSS":
            if time.time() - self._cross_start > self._cross_timeout:
                self._set(state=ActionState.FAIL, error="FAIL_CROSS_TIMEOUT")
                return True
            ex, en = self._exit
            de, dn = ex - pose.e, en - pose.n
            dist = math.hypot(de, dn)
            if dist <= self._arrive_tol:
                ch.set_velocity(0.0, 0.0)
                ch.set_terrain_mode("wheel")        # 恢复常规（M0 核对真实枚举）
                self._terrain_switched = False
                self._phase = "CONFIRM"
                return False
            bearing = math.atan2(dn, de)
            yaw_err = bearing - pose.yaw
            while yaw_err > math.pi:
                yaw_err -= 2 * math.pi
            while yaw_err < -math.pi:
                yaw_err += 2 * math.pi
            ch.set_velocity(self._cross_speed, max(-1.0, min(1.0, 1.8 * yaw_err)))
            self._step_chassis()
            self._set(progress=0.45 + 0.45 * (1 - min(1.0, dist / self._total_cross)),
                      data={"remain_m": round(dist, 3)})
            return False

        # CONFIRM
        self._set(state=ActionState.SUCCESS, progress=1.0,
                  data={"crossed": True, "final_pose": [round(pose.e, 3),
                                                        round(pose.n, 3),
                                                        round(pose.yaw, 3)]})
        return True

    def _setup(self):
        self._perception_set("lidar", True)
        self._perception_set("scene", True)
        pose = self.ctx.chassis.get_pose()
        self._total_cross = max(0.1, math.hypot(self._exit[0] - pose.e,
                                                self._exit[1] - pose.n))
        self.ctx.chassis.stand_up()

    def _on_abort(self):
        try:
            self.ctx.chassis.set_terrain_mode("wheel")
        except Exception:
            pass

    def _on_finish(self):
        if self._terrain_switched:
            try:
                self.ctx.chassis.set_terrain_mode("wheel")
            except Exception:
                pass
        self._perception_set("lidar", False)
        self._perception_set("scene", False)
