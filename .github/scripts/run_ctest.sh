#!/usr/bin/env bash
# run_ctest.sh - Run ctest and print only failed test output.
#
# Successful test progress lines are suppressed so that ASAN/TSAN errors
# and other failure output appear at the top of the log and are not
# truncated by GitHub Actions' log size limits.  The ctest summary
# (pass/fail counts, list of failed tests) is printed at the end and
# may be truncated if very long.
#
# Usage: run_ctest.sh <build_dir> [log_file_name] [extra ctest args...]
#   build_dir      - CMake build directory containing CTestTestfile.cmake
#   log_file_name  - Name of log file to also upload as artifact (optional)
#   extra args     - Additional ctest arguments (e.g. -R "integration", --parallel N)

set -uo pipefail

BUILD_DIR="$1"
shift

LOG_FILE_NAME=""
# Check if second argument is a non-empty, non-flag string (log file name).
# An empty string "" means "no log file name" (used when extra ctest args follow).
if [[ $# -ge 1 && "${1}" != -* && "${1}" != "" ]]; then
    LOG_FILE_NAME="$1"
    shift
elif [[ $# -ge 1 && "${1}" == "" ]]; then
    # Skip empty string argument (placeholder for no log file name).
    shift
fi

FULL_LOG="${BUILD_DIR}/ctest_full.log"

# Run ctest, capturing all output.  Don't let set -e catch the exit code.
ctest --test-dir "$BUILD_DIR" --output-on-failure "$@" > "$FULL_LOG" 2>&1 || ctest_exit=$?
ctest_exit=${ctest_exit:-0}

# Copy the full log to the requested artifact name (if provided).
if [[ -n "$LOG_FILE_NAME" ]]; then
    cp "$FULL_LOG" "$LOG_FILE_NAME"
fi

echo "=== Failed Test Output ==="
# Print only failed test output (gtest output, ASAN/TSAN errors, etc.).
# Skip ctest progress lines:
#   - "Internal ctest changing into directory: ..."
#   - "Test project ..."
#   - "          Start    N: TestName"
#   - "    N/M Test    #K: TestName ... Passed/Failed ... X sec"
# Keep everything else (gtest [ RUN ]/[ OK ]/[ FAILED ] blocks, ASAN errors,
# summary lines).
grep -v -E "^(Internal ctest|Test project)" "$FULL_LOG" | \
    grep -v -E "^[[:space:]]+Start[[:space:]]+[0-9]+:" | \
    grep -v -E "^[[:space:]]+[0-9]+/[0-9]+[[:space:]]+Test[[:space:]]" | \
    grep -v "^[[:space:]]*$"

echo ""
echo "=== Test Summary ==="
# Print the summary section (may be truncated if very long).
grep -E "tests passed|Total Test time|The following tests (did not run|FAILED)|Errors while running" "$FULL_LOG" || true
# List failed/timeout tests (limit to 200 lines to avoid huge output).
grep -E "^[[:space:]]+[0-9]+ - .*\((Failed|Timeout)\)" "$FULL_LOG" | head -200 || true

exit "$ctest_exit"
