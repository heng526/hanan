# -*- coding: utf-8 -*-
"""teleop.py — D50WS 命令行遥控器（替换原厂手柄的日常操作）。

用法：
    python3 tools/teleop.py            # 连接真狗（需 lingsi 桥接已编译 + 网线组网）
    python3 tools/teleop.py --mock     # Mock 仿真模式（无狗开发/演示，Windows 可用）

命令集（替代手柄语义）：
    stand              站立            （= 手柄 A 键）
    down               趴下
    damp               阻尼模式        （= 右拨杆右，关节软，急停保护用）
    zero               卸力模式        （= 阻尼后右拨杆左）
    w / s / a / d      前进 / 后退 / 左移 / 右移（持续移动直到 stop）
    q / e              左转 / 右转（持续转向直到 stop）
    stop               停止运动
    speed <1|2|3>      低/中/高速档（限制移动指令上限）
    status             状态一览（电量/速度/位置/RTK/安全判定）
    alerts             最近告警
    estop              软急停：立即停止 + 阻尼（硬急停仍是手柄右拨杆→阻尼）
    reset              解除软急停
    quit               退出

安全逻辑：
    - 运动线程 10Hz 刷新速度指令；每次 tick 先过安全监护（SafetyMonitor），
      判定 STOP 立即归零并进入急停态
    - 退出/中断时自动 stop
"""
import argparse
import sys
import threading
import time

sys.path.insert(0, ".")

from nav.safety import SafetyMonitor, SafetyConfig, Verdict
from sdk_adapter.base import RtkQuality

SPEED_LIMITS = {1: 0.5, 2: 1.0, 3: 1.5}


def build_robot(mock: bool):
    if mock:
        from sdk_adapter.mock import MockRobot
        r = MockRobot()
        r.stand_up()
        print("[mock] Mock 机器人已站立（仿真模式）")
        return r
    from sdk_adapter.lingsi_tcp import LingSiTcpRobot
    r = LingSiTcpRobot()
    print("[sdk] SDK 初始化成功")
    return r


class MotionLoop(threading.Thread):
    """10Hz 运动线程：持续输出当前速度指令并做安全检查。"""

    def __init__(self, robot, monitor, period=0.1):
        super().__init__(daemon=True)
        self.robot = robot
        self.monitor = monitor
        self.period = period
        self.cmd = (0.0, 0.0)
        self.estop = False
        self.last_verdict = Verdict.RUN
        self.reasons = []
        self._stop_evt = threading.Event()

    def run(self):
        tick = 0
        while not self._stop_evt.is_set():
            start = time.time()
            v, w = self.cmd if not self.estop else (0.0, 0.0)
            tel = self.robot.telemetry()
            tel.estop = self.estop
            verdict = self.monitor.update(tel, wp_type="TRANSIT")
            self.last_verdict = verdict.level
            self.reasons = verdict.reasons
            if verdict.level == Verdict.STOP:
                v, w = 0.0, 0.0
                self.cmd = (0.0, 0.0)
            try:
                self.robot.set_velocity(v, w)
            except Exception as e:
                print("\n[!] 运动指令异常: %s" % e)
            if hasattr(self.robot, "step"):   # Mock 推进仿真
                self.robot.step()
            tick += 1
            time.sleep(max(0.0, self.period - (time.time() - start)))

    def stop(self):
        self._stop_evt.set()


HELP = __doc__


def main():
    ap = argparse.ArgumentParser(description="D50WS 命令行遥控器")
    ap.add_argument("--mock", action="store_true", help="Mock 仿真模式")
    args = ap.parse_args()

    robot = build_robot(args.mock)
    cfg = SafetyConfig(heartbeat_timeout_s=1.0)
    monitor = SafetyMonitor(cfg)
    motion = MotionLoop(robot, monitor)
    motion.start()

    speed_gear = 1
    print(HELP)
    print("当前速度档: %d (上限 %.1f m/s)。输入 help 查看命令。\n" % (speed_gear, SPEED_LIMITS[speed_gear]))

    running = True
    while running:
        try:
            line = input("teleop[%s]> " % motion.last_verdict.value).strip().lower()
        except (EOFError, KeyboardInterrupt):
            line = "quit"
        parts = line.split()

        if not parts:
            continue
        cmd = parts[0]

        try:
            if cmd == "help":
                print(HELP)
            elif cmd in ("w", "s", "a", "d", "q", "e"):
                v_lim = SPEED_LIMITS[speed_gear]
                v, w = 0.0, 0.0
                if cmd == "w": v = v_lim
                elif cmd == "s": v = -v_lim * 0.5   # 倒退限半速
                elif cmd == "a": v = v_lim * 0.4    # 横移低速（SDK Move 支持 vm）
                elif cmd == "d": v = -v_lim * 0.4
                elif cmd == "q": w = 0.6
                elif cmd == "e": w = -0.6
                motion.estop = False
                motion.cmd = (v, w)
                print("速度指令 (%.2f m/s, %.2f rad/s)，stop 停止" % (v, w))
            elif cmd == "stop":
                motion.cmd = (0.0, 0.0)
                robot.stop() if hasattr(robot, "stop") else robot.set_velocity(0, 0)
                print("已停止")
            elif cmd == "stand":
                robot.stand_up() and print("站立指令已发")
            elif cmd == "down":
                motion.cmd = (0.0, 0.0)
                robot.stand_down() and print("趴下指令已发")
            elif cmd == "damp":
                motion.cmd = (0.0, 0.0)
                robot.set_damping(True) and print("阻尼模式（关节软，防甩打）")
            elif cmd == "zero":
                robot.zero_torque() and print("卸力模式（可拖动狗体）")
            elif cmd == "speed" and len(parts) == 2 and parts[1] in SPEED_LIMITS:
                speed_gear = int(parts[1])
                print("速度档 %d，上限 %.1f m/s" % (speed_gear, SPEED_LIMITS[speed_gear]))
            elif cmd == "status":
                tel = robot.telemetry()
                v, w = motion.cmd
                print("  电量: %.0f%%   RTK: %s   安全: %s %s"
                      % (tel.battery, tel.rtk_quality.name,
                         motion.last_verdict.value, motion.reasons or ""))
                p = tel.pose
                print("  里程计位姿: e=%.2f n=%.2f yaw=%.1f°  指令: (%.2f, %.2f)"
                      % (p.e, p.n, p.yaw * 57.3, v, w))
            elif cmd == "alerts":
                if hasattr(robot, "alerts"):
                    for a in robot.alerts():
                        print("  ", a)
                else:
                    print("  （mock 无告警）")
            elif cmd == "estop":
                motion.estop = True
                motion.cmd = (0.0, 0.0)
                robot.set_damping(True)
                print(">>> 软急停已触发（停止+阻尼）。reset 解除 <<<")
            elif cmd == "reset":
                motion.estop = False
                print("软急停解除")
            elif cmd == "quit":
                running = False
            else:
                print("未知命令，help 查看")
        except Exception as e:
            print("[!] 命令执行失败: %s" % e)

    # 收尾：停止 + 趴下前的安全序列（先停运动线程）
    motion.stop()
    try:
        robot.set_velocity(0, 0)
        if hasattr(robot, "stop"):
            robot.stop()
    except Exception:
        pass
    motion.join(timeout=1.0)
    print("已退出（狗保持当前姿态；关机请先 down 再长按电池键）")


if __name__ == "__main__":
    main()
