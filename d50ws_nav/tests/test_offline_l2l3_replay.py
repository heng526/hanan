"""Acceptance checks for the one-command Mock-only L2/L3 trace."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.actions import ACTION_CLASSES
from nav.context import PerceptionBundle, RunContext
from sdk_adapter.mock import MockRobot
from sdk_adapter.mock_actuators import MockManipulator
from tools.offline_l2l3_replay import NORMAL_CHAIN, replay, require_mock_only


def events(case, kind=None, source=None):
    return [e for e in case["events"]
            if (kind is None or e["kind"] == kind)
            and (source is None or e["source"] == source)]


def test_replay_covers_every_existing_action_and_fault_boundary():
    result = replay()
    assert result["mode"] == "MOCK_ONLY"
    assert result["field_acceptance"] is False
    assert set(NORMAL_CHAIN) == set(ACTION_CLASSES)
    normal = {c["case"].split("/", 1)[1]: c for c in result["cases"]
              if c["case"].startswith("normal/")}
    adverse = {c["case"].split("/", 1)[1]: c for c in result["cases"]
               if c["case"].startswith("adverse/")}
    assert set(normal) == set(ACTION_CLASSES)
    assert list(normal) == NORMAL_CHAIN
    assert all(c["terminal"] == "SUCCESS" for c in normal.values())
    align_states = events(normal["align"], "state_transition")
    phases = [item["data"]["internal_phase"] for item in align_states]
    assert phases.index("COARSE") < phases.index("FINE")
    assert align_states[-1]["data"]["state"] == "SUCCESS"
    assert any(item["data"]["args"][1] != 0 for item in events(
        normal["align"], "mock_motion_command", "chassis.set_velocity"))
    assert set(adverse) == {"missing_P4", "stale_P1", "wrong_command",
                            "late_receipt", "replayed_receipt", "cancel_vent",
                            "no_detach_backup_guard", "no_retreat_guard"}
    assert adverse["cancel_vent"]["terminal"] == "ABORTED"
    assert all(c["terminal"] == "FAIL" for n, c in adverse.items()
               if n != "cancel_vent")
    assert events(normal["cruise"], "pose")
    assert events(normal["cross_track"], "observation", "P4.read")
    assert events(normal["cross_track"], "observation", "P3.read")
    assert events(normal["approach"], "observation", "P1.read")
    assert events(normal["detach"], "observation", "P2.read")
    assert events(adverse["missing_P4"], "observation", "P4.read")[0]["data"]["missing"]
    assert any(e["data"]["age_s"] == 8.0 for e in
               events(adverse["stale_P1"], "observation", "P1.read"))
    assert events(adverse["cancel_vent"], "cancel_request")
    assert not any(e["kind"] == "mock_motion_command" and
                   e["data"]["args"] and e["data"]["args"][0] < 0
                   for name in ("no_detach_backup_guard", "no_retreat_guard")
                   for e in adverse[name]["events"])


def test_replay_correlates_start_done_and_check_receipts():
    result = replay()
    normal = {c["case"].split("/", 1)[1]: c for c in result["cases"]
              if c["case"].startswith("normal/")}
    for name, start_method, read_source in (
            ("vent_high", "manipulator.start_vent", "manipulator.read"),
            ("vent_low", "manipulator.start_vent", "manipulator.read"),
            ("detach", "rod.start_detach", "rod.read"),
            ("retreat", "manipulator.stow", "manipulator.read")):
        case = normal[name]
        start = events(case, "command_or_check", start_method)[0]
        command_id = start["data"]["reply"]["command_id"]
        assert start["data"]["reply"]["accepted"] is True
        assert any(e["data"]["reply"].get("state") == "DONE" and
                   e["data"]["reply"].get("command_id") == command_id
                   for e in events(case, "receipt", read_source))
        if name.startswith("vent"):
            check = events(case, "check_observation", "acoustic.check")[0]
            assert check["data"]["reply"]["command_id"] == command_id
        if name == "detach":
            check = events(case, "check_observation", "P2.check_detach")[0]
            assert check["data"]["reply"]["detach_success"] is True
            assert check["data"]["reply"]["frame_count"] > 0


def test_entry_rejects_non_mock_dependencies_before_action():
    with pytest.raises(TypeError, match="MockRobot"):
        require_mock_only(RunContext(chassis=object()))
    with pytest.raises(TypeError, match="non-Mock manipulator"):
        require_mock_only(RunContext(chassis=MockRobot(), manipulator=object()))
    with pytest.raises(TypeError, match="non-Mock lever"):
        require_mock_only(RunContext(chassis=MockRobot(),
                          perception=PerceptionBundle(lever=object()),
                          manipulator=MockManipulator()))
