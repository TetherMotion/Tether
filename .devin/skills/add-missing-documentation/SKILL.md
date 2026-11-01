---
name: add-missing-documentation
description: Fetch the doc-coverage CI artifact (doxygen WARN_IF_UNDOCUMENTED / WARN_NO_PARAMDOC warnings, grouped per file) and turn it into a documented-code todo list. Use when the user wants to find and fix undocumented public API.
---

# Add Missing Documentation

The CI job "Build Documentation" collects all Doxygen coverage warnings
(`WARN_IF_UNDOCUMENTED`, `WARN_IF_INCOMPLETE_DOC`, `WARN_NO_PARAMDOC`,
`WARN_IF_DOC_ERROR`) into `build/doxygen-warnings.txt` via `WARN_LOGFILE`
(docs/conf.py), then packages them into the **`doc-coverage`** artifact via
`scripts/doc_coverage_report.py`. Coverage gaps never fail the build — this
artifact is the reporting channel.

## 1. Fetch the doc-coverage artifact

```bash
# Find the most recent docs run on main and its doc-coverage artifact id
gh run list --repo TetherMotion/Tether --branch main --limit 5
gh api repos/TetherMotion/Tether/actions/runs/<RUN_ID>/artifacts \
  --jq '.artifacts[] | select(.name=="doc-coverage") | .id'

# Download and extract (zip root contains a single doc-coverage/ folder)
curl -sL -H "Authorization: Bearer $(gh auth token)" \
  "https://api.github.com/repos/TetherMotion/Tether/actions/artifacts/<ART_ID>/zip" \
  -o /tmp/doc-coverage.zip
unzip -o /tmp/doc-coverage.zip -d /tmp/doc-coverage
```

`doc-coverage/COVERAGE.md` lists warnings grouped by file (public headers
first, most-warned files first); `doxygen-warnings.txt` is the raw log.
If `gh run download -n doc-coverage` fails with a path-traversal error on an
old artifact, use the `curl` form above — it has no such check.

If the user gives a run URL directly, extract the run id from
`.../actions/runs/<RUN_ID>`.

## 2. Build the todo list

Create a `todo_write` list grouped by file, one item per undocumented
entity (class, function, member, enum value, parameter). The report is
already grouped — walk `COVERAGE.md` top-down. Prioritize public API
headers (`include/tether/`) before `src/` internals.

## 3. Document

For each item, add a concise doc comment following the codebase's
existing style (`/** ... */` / `///` brief + `@param`/`@return` where
applicable — see neighboring documented code in the same header).
Document intent and units/invariants, not just the signature restated in
English. Do NOT add comments to third-party or generated code.

## 4. Verify

```bash
doxygen Doxyfile 2>&1 | grep -E "warning" | head
# or regenerate the same report CI emits:
python3 scripts/doc_coverage_report.py \
    --warnings doxygen-warnings.txt --out /tmp/doc-coverage
```

The dev `Doxyfile` uses the same WARN_FORMAT (`$file:$line: $text`) and the
strict warn set, so its stderr output can be piped into the same script.
A clean run means zero new warnings on the files you touched.
