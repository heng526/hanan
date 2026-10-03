"""Offline regressions for the M0 launcher and real SDK adapter."""

import re
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sdk_adapter import lingsi_tcp


PROJECT = Path(__file__).resolve().parents[1]
SDK_HEADER = (
    PROJECT.parent
    / "_sdk"
    / "robot_sdk_client_wheel_v5.0.4_x86_64_20260817"
    / "include"
    / "high_level_base.h"
)


def test_real_launcher_uses_project_directory():
    script = (PROJECT / "run_real.sh").read_text(encoding="utf-8")
    here_line = next(line for line in script.splitlines() if line.startswith("HERE="))
    assert 'dirname "$0")" && pwd' in here_line
    assert '/..' not in here_line
    assert 'cd "$HERE"' in script
    assert "python3 tools/teleop.py" in script
    assert (PROJECT / "tools" / "teleop.py").is_file()


def test_bridge_matches_v504_header_fields():
    bridge = (PROJECT / "sdk_adapter" / "cpp" / "lingsi_pybind.cpp").read_text(
        encoding="utf-8"
    )
    assert "std::shared_ptr<Robot> g_robot" in bridge
    assert "return g_robot.get();" in bridge
    for field in ("imu_ax", "imu_ay", "imu_az"):
        assert f"a.{field}" in bridge
    for field in ("key", "code", "key_str", "timestamp"):
        assert f'd["{field}"] = a.{field};' in bridge
    assert 'd["raw"]' not in bridge

    if SDK_HEADER.is_file():
        header = SDK_HEADER.read_text(encoding="utf-8")
        assert re.search(r"std::shared_ptr<Robot>\s+createQuadruped\s*\(", header)
        for field in ("imu_ax", "imu_ay", "imu_az", "key_str", "timestamp"):
            assert re.search(rf"\b{field}\s*;", header)


def test_real_telemetry_does_not_self_deadlock(monkeypatch):
    class FakeSdk:
        def init(self):
            return 0

        def get_odometry(self):
            return {"position": [1.0, 2.0, 0.0], "orientation": [0, 0, 0, 1]}

        def get_battery(self):
            return 80

    monkeypatch.setattr(lingsi_tcp, "_lingsi", FakeSdk())
    robot = lingsi_tcp.LingSiTcpRobot()
    result = {}
    thread = threading.Thread(
        target=lambda: result.setdefault("telemetry", robot.telemetry()), daemon=True
    )
    thread.start()
    thread.join(timeout=0.5)

    assert not thread.is_alive(), "telemetry() blocked on its own SDK lock"
    assert result["telemetry"].battery == 80
    assert result["telemetry"].pose.e == 1.0
