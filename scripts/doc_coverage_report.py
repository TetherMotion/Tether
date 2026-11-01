#!/usr/bin/env python3
"""Group a doxygen WARN_LOGFILE into a per-file coverage report.

Reads warnings emitted with WARN_FORMAT = "$file:$line: $text" and writes:

- COVERAGE.md            — warnings grouped by file, public headers first
- doxygen-warnings.txt   — verbatim copy of the input log

Only project sources count: include/tether/** and src/** (dependencies,
_deps, tests, and build trees are filtered out).  The report is consumed by
agents that add missing doc comments — see
.devin/skills/add-missing-documentation/SKILL.md.

Usage:
    doc_coverage_report.py --warnings build/doxygen-warnings.txt \
        --out <output-dir>
"""

import argparse
import re
import shutil
from pathlib import Path

# A warning starts at "file:line:"; following lines until the next match are
# continuation text (e.g. the parameter-name list of WARN_NO_PARAMDOC).
WARN_START = re.compile(r"^(.+?):(\d+):\s*(.*)$")

# Repo-relative path roots that count as documentation surface.
SURFACE = ("include/tether/", "src/")


def repo_relative(path: str):
    """Return the path starting at include/tether/ or src/, else None."""
    for root in SURFACE:
        i = path.find(root)
        if i >= 0:
            return path[i:]
    return None


def parse_warnings(text: str):
    """Parse WARN_FORMAT lines into (file, line, message) tuples.

    File is normalized to its repo-relative form; files outside the
    documentation surface are dropped.
    """
    warnings = []          # (file, line, message)
    current = None         # index into warnings of the open warning
    for raw in text.splitlines():
        m = WARN_START.match(raw.rstrip())
        if m:
            # Any file:line line — kept or not — closes the previous warning.
            current = None
            if repo_relative(m.group(1)) is not None:
                # Split off doxygen's literal "warning:"/"error:" prefix word.
                msg = re.sub(r"^(warning|error):\s*", "", m.group(3))
                warnings.append([repo_relative(m.group(1)),
                                 int(m.group(2)), msg])
                current = len(warnings) - 1
        elif current is not None and raw[:1].isspace() and raw.strip():
            # Indented continuation of the previous warning (detail lines
            # like WARN_NO_PARAMDOC's parameter-name list).
            warnings[current][2] += " " + raw.strip()
    return [(f, l, m) for f, l, m in warnings]


def render(warnings) -> str:
    """Render grouped warnings as COVERAGE.md text."""
    by_file = {}
    for f, line, msg in warnings:
        by_file.setdefault(f, []).append((line, msg))

    # Public headers first, then src/; most-warned files first within each.
    def sort_key(item):
        f, ws = item
        return (0 if f.startswith("include/") else 1, -len(ws), f)

    out = ["# Documentation coverage", ""]
    n, m = len(warnings), len(by_file)
    if n == 0:
        out += ["No doxygen warnings — full coverage on all scanned files."]
        return "\n".join(out) + "\n"
    out += [
        f"{n} warnings in {m} files "
        "(WARN_IF_UNDOCUMENTED / WARN_IF_INCOMPLETE_DOC / WARN_NO_PARAMDOC).",
        "",
        "Regenerate: CI job 'Build Documentation' → artifact `doc-coverage`,",
        "or locally `doxygen Doxyfile` (dev config writes the same format).",
        "",
    ]
    for f, ws in sorted(by_file.items(), key=sort_key):
        out.append(f"## `{f}` — {len(ws)}")
        out.append("")
        for line, msg in sorted(ws):
            out.append(f"- :{line} {msg}")
        out.append("")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--warnings", required=True, type=Path,
                    help="doxygen WARN_LOGFILE output")
    ap.add_argument("--out", required=True, type=Path,
                    help="output directory (created if missing)")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    if args.warnings.is_file():
        text = args.warnings.read_text(errors="replace")
        shutil.copy(args.warnings, args.out / "doxygen-warnings.txt")
    else:
        text = ""

    report = args.out / "COVERAGE.md"
    report.write_text(render(parse_warnings(text)))
    print(f"{report}: {len(parse_warnings(text))} warnings")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
