"""Mock-only coverage for AlignAction's coarse-heading branch."""

import math
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import AlignAction
from nav.context import PerceptionBundle, RunContext
from perception.mocks import MockLeverDetector
from sdk_adapter.mock import MockRobot


class FrozenYawRobot(MockRobot):
    """A Mock that accepts commands but never changes pose."""

    def __init__(self):
        super().__init__(dt=0.1)
        self.step_seen = threading.Event()

    def step(self):
        self.step_seen.set()


def make_action(robot, heading=0.4, max_adjust_s=1.0):
    lever = MockLeverDetector(chassis=robot,
            lever_world=(0.6 * math.cos(heading), 0.6 * math.sin(heading)),
            detect_range=5.0, period=0.01)
    context = RunContext(chassis=robot, perception=PerceptionBundle(lever=lever))
    action = AlignAction(context, coarse_heading=heading, period=0.01,
                         max_adjust_s=max_adjust_s)
    return action, lever


def clean_up(action, lever):
    action.stop()
    action.join(1.0)
    lever.close()


def test_nonzero_coarse_heading_reaches_fine_and_success():
    robot = MockRobot(dt=0.1)
    action, lever = make_action(robot)
    try:
        action.start()
        action.join(2.0)
        assert not action.is_alive(), action.read()
        state = action.read()
        assert state["state"] == "SUCCESS", state
        assert abs(robot.pose.yaw - 0.4) <= 0.05
        assert action._phase == "FINE"
        assert robot._cmd == (0.0, 0.0)
        assert not lever.enabled
    finally:
        clean_up(action, lever)


def test_frozen_coarse_heading_times_out_and_cleans_up():
    robot = FrozenYawRobot()
    action, lever = make_action(robot, max_adjust_s=0.06)
    try:
        action.start()
        action.join(1.0)
        assert not action.is_alive(), action.read()
        state = action.read()
        assert robot.step_seen.is_set()
        assert state["state"] == "FAIL", state
        assert state["error"] == "FAIL_ALIGN_TIMEOUT"
        assert state["data"]["phase"] == "COARSE"
        assert robot.pose.yaw == 0.0
        assert robot._cmd == (0.0, 0.0)
        assert not lever.enabled
    finally:
        clean_up(action, lever)


def test_cancel_during_coarse_heading_cleans_up():
    robot = FrozenYawRobot()
    action, lever = make_action(robot)
    try:
        action.start()
        assert robot.step_seen.wait(1.0), action.read()
        action.stop()
        action.join(1.0)
        assert not action.is_alive()
        assert action._phase == "COARSE"
        assert action.read()["state"] == "ABORTED"
        assert robot._cmd == (0.0, 0.0)
        assert not lever.enabled
    finally:
        clean_up(action, lever)
