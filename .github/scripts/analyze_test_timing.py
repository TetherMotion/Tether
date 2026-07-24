#!/usr/bin/env python3
"""Analyze GoogleTest JSON output to find the slowest tests and test groups."""

import json
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

BUILD_DIR = Path(os.environ.get("BUILD_DIR", "build"))
TESTS_DIR = BUILD_DIR / "bin" / "tests"
TIMING_DIR = BUILD_DIR / "timing_results"
TOP_N = int(os.environ.get("TOP_N", "50"))


def parse_time(value) -> float:
    """Parse a GoogleTest time value into seconds as a float.

    GoogleTest emits times as strings that may be bare numbers ("0.123")
    or carry a unit suffix ("0s", "123ms"). Fall back to 0.0 for anything
    that cannot be parsed so a single bad entry does not abort the run.
    """
    if value is None:
        return 0.0
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return 0.0
    # Strip a trailing unit suffix if present and convert to seconds.
    unit_scales = {"ms": 1e-3, "us": 1e-6, "ns": 1e-9, "s": 1.0}
    scale = 1.0
    for unit, factor in unit_scales.items():
        if text.endswith(unit):
            text = text[: -len(unit)].strip()
            scale = factor
            break
    try:
        return float(text) * scale
    except ValueError:
        return 0.0


def main() -> int:
    TIMING_DIR.mkdir(parents=True, exist_ok=True)

    if not TESTS_DIR.exists():
        print(f"Test directory not found: {TESTS_DIR}", file=sys.stderr)
        return 1

    executables = sorted(
        p for p in TESTS_DIR.iterdir()
        if p.is_file() and os.access(p, os.X_OK)
    )
    if not executables:
        print(f"No test executables found in {TESTS_DIR}", file=sys.stderr)
        return 1

    per_test = []
    per_suite = defaultdict(lambda: {"time": 0.0, "tests": 0})
    per_module = defaultdict(lambda: {"time": 0.0, "tests": 0})

    for exe in executables:
        out = TIMING_DIR / f"{exe.name}.json"
        print(f"Running {exe.name} ...")
        result = subprocess.run(
            [str(exe), f"--gtest_output=json:{out}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode != 0:
            print(f"  {exe.name} exited with code {result.returncode}")
        if not out.exists():
            continue
        try:
            data = json.loads(out.read_text())
        except json.JSONDecodeError as exc:
            print(f"  Failed to parse {out}: {exc}")
            continue

        for suite in data.get("testsuites", []):
            suite_name = suite.get("name", "Unknown")
            suite_time = parse_time(suite.get("time", "0"))
            per_suite[suite_name]["time"] += suite_time
            per_suite[suite_name]["tests"] += suite.get("tests", 0)
            for case in suite.get("testsuite", []):
                case_time = parse_time(case.get("time", "0"))
                per_test.append({
                    "module": exe.name,
                    "suite": suite_name,
                    "name": case.get("name", "unknown"),
                    "time": case_time,
                    "file": case.get("file", ""),
                })
                per_module[exe.name]["time"] += case_time
                per_module[exe.name]["tests"] += 1

    per_test.sort(key=lambda x: x["time"], reverse=True)
    suite_list = sorted(per_suite.items(), key=lambda kv: kv[1]["time"], reverse=True)
    module_list = sorted(per_module.items(), key=lambda kv: kv[1]["time"], reverse=True)

    print("\n=== Top {} slowest individual tests ===".format(TOP_N))
    for i, t in enumerate(per_test[:TOP_N], 1):
        print(f"{i:3}. {t['time']:8.4f}s  {t['module']}.{t['suite']}.{t['name']}")

    print("\n=== Top {} slowest test suites ===".format(TOP_N))
    for i, (name, info) in enumerate(suite_list[:TOP_N], 1):
        print(f"{i:3}. {info['time']:8.4f}s  ({info['tests']:4d} tests)  {name}")

    print("\n=== Slowest test modules (executables) ===")
    for i, (name, info) in enumerate(module_list, 1):
        print(f"{i:3}. {info['time']:8.4f}s  ({info['tests']:4d} tests)  {name}")

    summary = {
        "slowest_tests": per_test[:TOP_N * 2],
        "slowest_suites": [
            {"suite": k, **v} for k, v in suite_list
        ],
        "slowest_modules": [
            {"module": k, **v} for k, v in module_list
        ],
    }
    (TIMING_DIR / "timing_summary.json").write_text(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
