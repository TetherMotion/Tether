---
name: add-missing-documentation
description: Fetch Doxygen WARN_IF_UNDOCUMENTED warnings from the latest CI "Build Documentation" job via gh and turn them into a documented-code todo list. Use when the user wants to find and fix undocumented public API.
---

# Add Missing Documentation

The project `Doxyfile` already enables `WARN_IF_UNDOCUMENTED = YES`
(plus `WARN_IF_DOC_ERROR`, `WARN_IF_INCOMPLETE_DOC`, `WARN_NO_PARAMDOC`)
with `WARN_FORMAT = "$file:$line: $text"`. The CI job "Build
Documentation" runs the doc build and prints these warnings to the job
log.

## 1. Fetch the warnings from the latest CI run

```bash
# Find the docs job id of the most recent run on main
gh run list --repo TetherMotion/Tether --branch main --limit 5
# then list its jobs
gh run view <RUN_ID> --repo TetherMotion/Tether --json jobs \
  --jq '.jobs[] | select(.name | test("[Dd]ocumentation")) | .databaseId'
# pull the job log
gh run view --repo TetherMotion/Tether --job <JOB_ID> --log > /tmp/docs.log
```

If the docs job is still queued or a specific run is wanted, the user may
give a run URL directly — extract the run id from
`.../actions/runs/<RUN_ID>`.

## 2. Extract undocumented-entity warnings

Doxygen warnings appear as `$file:$line: $text`, typically under the
project path `docs/` or `include/tether/`. Filter for undocumented-code
warnings:

```bash
grep -E "warning:.*(not documented|no documentation|undocumented|is not documented|has no documentation)" /tmp/docs.log
# also catch member-of-undocumented and missing-param warnings
grep -E ":[0-9]+: *(warning|error):" /tmp/docs.log | grep -v "_deps\|dependencies/"
```

Dedupe by file:line. Skip anything under `dependencies/`, `build*/`,
`tests/` (only public headers/sources matter — `include/tether/**` and
`src/**` are the documentation surface scanned by Doxygen).

## 3. Build the todo list

Create a `todo_write` list grouped by file, one item per undocumented
entity (class, function, member, enum value, parameter). Prioritize
public API headers (`include/tether/`) before `src/` internals.

## 4. Document

For each item, add a concise doc comment following the codebase's
existing style (`/** ... */` / `///` brief + `@param`/`@return` where
applicable — see neighboring documented code in the same header).
Document intent and units/invariants, not just the signature restated in
English. Do NOT add comments to third-party or generated code.

## 5. Verify

```bash
cmake --build build --target docs 2>/dev/null || \
  doxygen Doxyfile 2>&1 | grep -E "warning" | head
```

The docs CI job builds the Sphinx tree under `docs/` (`conf.py` drives
the doxygen/breathe setup); a clean run means zero new warnings on the
files you touched.
