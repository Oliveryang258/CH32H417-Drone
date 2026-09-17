#!/usr/bin/env python3
"""Check that the V3F and V5F shared-memory declarations stay identical.

Post-competition engineering improvement; not flight-validated.
This is a source-level consistency check; it is not a hardware coherence test.
"""

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "V3F": REPO_ROOT / "EXAM/GPIO/GPIO_Toggle/V3F/User/shared_data.h",
    "V5F": REPO_ROOT / "EXAM/GPIO/GPIO_Toggle/V5F/User/shared_data.h",
}


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//.*?$", "", text, flags=re.MULTILINE)


def extract_contract(path: Path) -> tuple[list[str], dict[str, str]]:
    text = strip_comments(path.read_text(encoding="utf-8"))
    match = re.search(
        r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*SharedSensorData_t\s*;",
        text,
        flags=re.DOTALL,
    )
    if match is None:
        raise ValueError(f"SharedSensorData_t not found in {path}")

    fields = [
        re.sub(r"\s+", " ", declaration.strip()) + ";"
        for declaration in match.group("body").split(";")
        if declaration.strip()
    ]
    macros: dict[str, str] = {}
    for name in (
        "SHARED_DATA_BASE_ADDR",
        "SHARED_ALARM_BATT_LOW",
        "SHARED_ALARM_OVERCURRENT",
    ):
        macro = re.search(rf"^\s*#define\s+{name}\s+([^\s]+)", text, re.MULTILINE)
        if macro is None:
            raise ValueError(f"{name} not found in {path}")
        macros[name] = macro.group(1)
    return fields, macros


def main() -> int:
    try:
        contracts = {core: extract_contract(path) for core, path in HEADERS.items()}
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    v3_fields, v3_macros = contracts["V3F"]
    v5_fields, v5_macros = contracts["V5F"]
    if v3_fields != v5_fields:
        print("ERROR: SharedSensorData_t field declarations differ:", file=sys.stderr)
        limit = max(len(v3_fields), len(v5_fields))
        for index in range(limit):
            left = v3_fields[index] if index < len(v3_fields) else "<missing>"
            right = v5_fields[index] if index < len(v5_fields) else "<missing>"
            if left != right:
                print(f"  field {index}:\n    V3F: {left}\n    V5F: {right}", file=sys.stderr)
        return 1

    if v3_macros != v5_macros:
        print(f"ERROR: shared constants differ: V3F={v3_macros}, V5F={v5_macros}", file=sys.stderr)
        return 1

    digest_input = "\n".join(v3_fields + [f"{k}={v}" for k, v in sorted(v3_macros.items())])
    digest = hashlib.sha256(digest_input.encode("utf-8")).hexdigest()[:16]
    print(f"PASS: shared ABI declarations match ({len(v3_fields)} fields, contract {digest}).")
    print("NOTE: source agreement does not prove atomic cross-core snapshots or flight validation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
