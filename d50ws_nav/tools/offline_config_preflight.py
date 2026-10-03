"""Pure-file preflight for proposed D50WS integration references.

This module imports only Python's standard library. It never opens referenced
files, creates a camera/device object, contacts a network, or loads a model.
"""

import argparse
import json
import math
import os
import re
from pathlib import Path, PureWindowsPath


TEXT_FIELDS = (
    "device.robot_id", "device.firmware_version", "device.sdk_version",
    "sources.front_rgb", "sources.rear_rgb", "sources.lidar",
    "sources.clock_domain",
)
SHA_FIELDS = ("device.source_code_sha256",)
PERCEPTIONS = ("P1", "P2", "P3", "P4")
REFERENCE_FIELDS = tuple(
    f"perception.{name}.{kind}_ref" for name in PERCEPTIONS
    for kind in ("processor", "calibration")
) + tuple(f"receipts.{name}_schema_ref" for name in (
    "chassis", "vent", "detach", "stow", "acoustic", "cancel_stop"))
FRAME_FIELDS = tuple(
    f"perception.{name}.{kind}_frame" for name in PERCEPTIONS
    for kind in ("input", "output"))
AGE_FIELDS = tuple(f"perception.{name}.max_age_s" for name in PERCEPTIONS)
POLICY_FIELDS = ("safety.rear_clearance_policy_ref",
                 "safety.physical_stop_policy_ref")
PLACEHOLDERS = {"TODO", "TBD", "PLACEHOLDER", "UNKNOWN", "UNVERIFIED"}
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _value(doc, dotted):
    current = doc
    for key in dotted.split("."):
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _filled(value):
    return (isinstance(value, str) and bool(value.strip())
            and not (value.strip().startswith("<") and value.strip().endswith(">"))
            and value.strip().upper() not in PLACEHOLDERS)


def _local_reference(value, manifest):
    if not isinstance(value, str):
        return "未填写或类型错误：要求本地相对文件路径字符串"
    if not _filled(value):
        return "未填写本地文件引用"
    raw = value.strip()
    if ("://" in raw or raw.startswith(("\\\\", "//"))
            or Path(raw).is_absolute() or PureWindowsPath(raw).drive):
        return "只允许相对本地文件路径，禁止 URL、UNC 和绝对路径"
    root = PROJECT_ROOT if manifest.is_relative_to(PROJECT_ROOT) else manifest.parent
    candidate = Path(os.path.abspath(os.path.normpath(manifest.parent / raw)))
    if not candidate.is_relative_to(root):
        return "引用越出允许的本地目录"
    current = candidate
    while current != root:
        if current.is_symlink():
            return "引用包含符号链接，无法保证纯本地检查"
        current = current.parent
    if not candidate.is_file():
        return "本地引用文件不存在或不是普通文件"
    return None


def check_manifest(path):
    """Return field-level PASS/MISSING and explicit site-only UNVERIFIED items."""
    manifest = Path(path).absolute()
    if str(manifest).startswith(("\\\\", "//")):
        return {"overall": "MISSING", "checks": [{"field": "config",
                "status": "MISSING", "reason": "配置文件必须位于本地，禁止 UNC 路径"}]}
    current = manifest
    while True:
        if current.is_symlink():
            return {"overall": "MISSING", "checks": [{"field": "config",
                    "status": "MISSING", "reason": "配置路径包含符号链接，无法保证纯本地读取"}]}
        if current == current.parent:
            break
        current = current.parent
    try:
        doc = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return {"overall": "MISSING", "checks": [{"field": "config",
                "status": "MISSING", "reason": "配置文件不存在或不是有效 UTF-8 JSON：" + type(exc).__name__}]}
    if not isinstance(doc, dict):
        return {"overall": "MISSING", "checks": [{"field": "config",
                "status": "MISSING", "reason": "配置根节点必须是对象"}]}
    return check_document(doc, manifest)


def check_document(doc, manifest):
    """Validate synthetic or parsed config without opening any reference."""
    manifest = Path(manifest).absolute()

    checks = []

    def add(field, reason=None, pass_reason="字段已填；真实性未核验"):
        checks.append({"field": field, "status": "MISSING" if reason else "PASS",
                       "reason": reason or pass_reason})

    add("schema_version", None if type(doc.get("schema_version")) is int
        and doc["schema_version"] == 1 else "类型或版本错误：要求整数 1",
        "预检配置格式版本有效")
    for field in TEXT_FIELDS + FRAME_FIELDS:
        value = _value(doc, field)
        add(field, None if _filled(value) else "未填写或类型错误：要求非占位字符串")
    for field in SHA_FIELDS:
        value = _value(doc, field)
        add(field, None if isinstance(value, str) and
            re.fullmatch(r"[0-9a-fA-F]{64}", value) else
            "未填写或格式错误：要求 64 位 SHA-256 十六进制字符串")
    for field in AGE_FIELDS:
        value = _value(doc, field)
        add(field, None if type(value) in (int, float) and
            math.isfinite(value) and value > 0 else
            "未填写或类型错误：要求有限正数秒")
    for field in REFERENCE_FIELDS:
        add(field, _local_reference(_value(doc, field), manifest),
            "本地引用文件存在；内容与设备接口未核验")
    for field in POLICY_FIELDS:
        value = _value(doc, field)
        reason = ("现场策略未提供；无法从文件推断后方净空或实体停稳"
                  if not _filled(value) else
                  "仅记录了策略引用；现场净空与实体停稳仍须实测和责任人确认")
        checks.append({"field": field, "status": "UNVERIFIED", "reason": reason})
    checks.append({"field": "hardware_and_field_safety", "status": "UNVERIFIED",
                   "reason": "本工具不连设备、不检模型和网络；字段齐全不代表可运行或安全"})
    overall = "MISSING" if any(c["status"] == "MISSING" for c in checks) else "UNVERIFIED"
    return {"overall": overall, "checks": checks}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="本地 UTF-8 JSON 预检配置")
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = parser.parse_args(argv)
    result = check_manifest(args.config)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        for check in result["checks"]:
            print("%-10s %-43s %s" % (
                check["status"], check["field"], check["reason"]))
        print("总体：%s；仅为离线配置检查，未批准实机运行。" % result["overall"])
    return 2 if result["overall"] == "MISSING" else 0


if __name__ == "__main__":
    raise SystemExit(main())
