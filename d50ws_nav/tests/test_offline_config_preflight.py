"""Synthetic file-only preflight cases; no device or referenced file is opened."""

import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.offline_config_preflight import (AGE_FIELDS, FRAME_FIELDS,
    REFERENCE_FIELDS, TEXT_FIELDS, check_document, check_manifest, main)


TEMPLATE = Path(__file__).resolve().parents[1] / "config/preflight_template.json"


def field(result, name):
    return next(item for item in result["checks"] if item["field"] == name)


def set_field(doc, dotted, value):
    parts = dotted.split(".")
    target = doc
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = value


def filled_document():
    doc = copy.deepcopy(json.loads(TEMPLATE.read_text(encoding="utf-8")))
    for dotted in TEXT_FIELDS + FRAME_FIELDS:
        set_field(doc, dotted, "synthetic-local-value")
    for dotted in AGE_FIELDS:
        set_field(doc, dotted, 0.5)
    doc["device"]["source_code_sha256"] = "a" * 64
    for dotted in REFERENCE_FIELDS:
        set_field(doc, dotted, TEMPLATE.name)
    return doc


def test_placeholder_sample_reports_missing_and_site_unverified(capsys):
    result = check_manifest(TEMPLATE)
    assert result["overall"] == "MISSING"
    assert field(result, "device.robot_id")["status"] == "MISSING"
    assert field(result, "perception.P1.processor_ref")["status"] == "MISSING"
    assert field(result, "safety.rear_clearance_policy_ref")["status"] == "UNVERIFIED"
    assert main([str(TEMPLATE)]) == 2
    output = capsys.readouterr().out
    assert "MISSING" in output and "UNVERIFIED" in output


def test_missing_wrong_type_and_bad_numeric_value():
    doc = filled_document()
    del doc["device"]["robot_id"]
    doc["perception"]["P2"]["input_frame"] = 12
    doc["perception"]["P3"]["max_age_s"] = False
    doc["receipts"]["stow_schema_ref"] = 123
    result = check_document(doc, TEMPLATE)
    assert result["overall"] == "MISSING"
    for name in ("device.robot_id", "perception.P2.input_frame",
                 "perception.P3.max_age_s", "receipts.stow_schema_ref"):
        assert field(result, name)["status"] == "MISSING"
        assert "类型" in field(result, name)["reason"]


def test_missing_reference_and_remote_uri_are_rejected():
    doc = filled_document()
    doc["perception"]["P1"]["calibration_ref"] = "missing.json"
    doc["receipts"]["vent_schema_ref"] = "rtsp://not-to-be-opened/schema"
    result = check_document(doc, TEMPLATE)
    assert result["overall"] == "MISSING"
    assert "不存在" in field(result, "perception.P1.calibration_ref")["reason"]
    assert "禁止 URL" in field(result, "receipts.vent_schema_ref")["reason"]


def test_complete_local_fields_still_unverified_for_hardware():
    doc = filled_document()
    result = check_document(doc, TEMPLATE)
    assert result["overall"] == "UNVERIFIED"
    assert all(item["status"] != "MISSING" for item in result["checks"])
    assert field(result, "receipts.cancel_stop_schema_ref")["status"] == "PASS"
    assert field(result, "safety.rear_clearance_policy_ref")["status"] == "UNVERIFIED"
    assert field(result, "safety.physical_stop_policy_ref")["status"] == "UNVERIFIED"
    assert field(result, "hardware_and_field_safety")["status"] == "UNVERIFIED"
