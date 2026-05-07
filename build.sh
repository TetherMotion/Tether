#!/usr/bin/env bash
set -euo pipefail

# Tether build helper
# Usage: ./build.sh [options]
# Options:
#   -h, --help         Show this help
#   -d, --dir DIR      Build directory (default: build)
#   -t, --type TYPE    CMake build type (Debug/Release) (default: Debug)
#   -c, --coverage     Enable coverage (sets -DTETHER_ENABLE_COVERAGE=ON)
#   -e, --examples     Build examples (sets -DTETHER_BUILD_EXAMPLES=ON)
#   -x, --extract-esi  Build the extract_esi utility (sets -DTETHER_BUILD_EXTRACT_ESI=ON)#   -x, --extract-esi  Build the extract_esi utility (enabled by default)
#   -X, --no-extract-esi
#                      Do NOT build the extract_esi utility (sets -DTETHER_BUILD_EXTRACT_ESI=OFF)#   -j, --jobs N       Number of parallel build jobs (default: nproc)
#   -r, --run-tests    Run tests (ctest) after build
#   --ctest-args ARGS  Extra args to pass to ctest
#   --sdo-diag         Enable low-level SDO mailbox diagnostics (-DTETHER_DIAG_SDO_IO=ON)
#   --pause-on-sdo-fail
#                      Pause examples on SDO failures to allow packet capture
#                      (-DTETHER_DIAG_PAUSE_ON_SDO_FAIL=ON)
#   --clean            Remove the build directory and exit

BUILD_DIR=build
BUILD_TYPE=Debug
COVERAGE=OFF
BUILD_EXAMPLES=OFF
# Build extract_esi by default (can be disabled with -X / --no-extract-esi)
BUILD_EXTRACT_ESI=ON
JOBS="$(nproc 2>/dev/null || echo 1)"
RUN_TESTS=0
CTEST_ARGS=""
SDO_DIAG=OFF
SDO_PAUSE_ON_FAIL=OFF

# Determine the directory this script lives in (works when sourced or executed)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# If BUILD_DIR is a relative path, interpret it relative to the script directory so
# the script can be executed from any working directory and still use a repository
# local build directory by default.
if [[ "$BUILD_DIR" != /* ]]; then
  BUILD_DIR="$SCRIPT_DIR/$BUILD_DIR"
fi

print_usage() {
  sed -n '1,120p' "$0" | sed -n '1,60p'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) print_usage; exit 0 ;;
    -d|--dir) BUILD_DIR="$2"; shift 2 ;;
    -t|--type) BUILD_TYPE="$2"; shift 2 ;;
    -c|--coverage) COVERAGE=ON; shift ;;
    -e|--examples) BUILD_EXAMPLES=ON; shift ;;
    -j|--jobs) JOBS="$2"; shift 2 ;;
    -r|--run-tests) RUN_TESTS=1; shift ;;
    --ctest-args) CTEST_ARGS="$2"; shift 2 ;;
    --sdo-diag) SDO_DIAG=ON; shift ;;
    --pause-on-sdo-fail) SDO_PAUSE_ON_FAIL=ON; shift ;;
    -x|--extract-esi) BUILD_EXTRACT_ESI=ON; shift ;;
    -X|--no-extract-esi) BUILD_EXTRACT_ESI=OFF; shift ;;
    --clean) echo "Removing '$BUILD_DIR'..."; rm -rf "$BUILD_DIR"; exit 0 ;;
    --) shift; break ;;
    *) echo "Unknown option: $1"; print_usage; exit 1 ;;
  esac
done

echo "🔧 Tether build helper"
echo "Source dir:   $SCRIPT_DIR"
echo "Build dir:    $BUILD_DIR"
echo "Build type:   $BUILD_TYPE"
echo "Coverage:     $COVERAGE"
echo "Examples:     $BUILD_EXAMPLES"
echo "SDO diag:     $SDO_DIAG"
echo "SDO pause-on-fail: $SDO_PAUSE_ON_FAIL"
echo "Extract ESI:  $BUILD_EXTRACT_ESI (default: ON)"
echo "Jobs:         $JOBS"

CMAKE_EXTRA_FLAGS=()
if [[ "$COVERAGE" == "ON" ]]; then
  CMAKE_EXTRA_FLAGS+=("-DTETHER_ENABLE_COVERAGE=ON")
fi
if [[ "$BUILD_EXAMPLES" == "ON" ]]; then
  CMAKE_EXTRA_FLAGS+=("-DTETHER_BUILD_EXAMPLES=ON")
fi
if [[ "$SDO_DIAG" == "ON" ]]; then
  CMAKE_EXTRA_FLAGS+=("-DTETHER_DIAG_SDO_IO=ON")
fi
if [[ "$SDO_PAUSE_ON_FAIL" == "ON" ]]; then
  CMAKE_EXTRA_FLAGS+=("-DTETHER_DIAG_PAUSE_ON_SDO_FAIL=ON")
fi

# Explicitly pass chosen setting so --no-extract-esi correctly disables the tool
CMAKE_EXTRA_FLAGS+=("-DTETHER_BUILD_EXTRACT_ESI=${BUILD_EXTRACT_ESI}")

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "${CMAKE_EXTRA_FLAGS[@]}"

# If the user requested extract_esi OFF and a stale binary exists from a previous build,
# remove it so the build output reflects the requested configuration.
if [[ "$BUILD_EXTRACT_ESI" == "OFF" ]]; then
  if [[ -f "$BUILD_DIR/bin/extract_esi" ]]; then
    echo "⚠️  Removing stale extract_esi binary from '$BUILD_DIR/bin' because build is disabled"
    rm -f "$BUILD_DIR/bin/extract_esi"
  fi
fi

cmake --build "$BUILD_DIR" --parallel "$JOBS"

# Ensure the extract_esi tool is actually built by default when enabled.
# Some CMake configurations may not include the utility in the default 'all' target
# (depending on generator or ordering). Explicitly build it if requested.
if [[ "$BUILD_EXTRACT_ESI" == "ON" ]]; then
  echo "🔧 Building extract_esi tool (requested by default)"
  cmake --build "$BUILD_DIR" --parallel "$JOBS" --target extract_esi || true
fi

if [[ "$RUN_TESTS" -eq 1 ]]; then
  echo "✅ Running tests (ctest)..."
  ctest --test-dir "$BUILD_DIR" --output-on-failure $CTEST_ARGS
fi

# Final check: verify binary exists when it should
if [[ "$BUILD_EXTRACT_ESI" == "ON" ]]; then
  if [[ -f "$BUILD_DIR/bin/extract_esi" ]]; then
    echo "✅ extract_esi present: $BUILD_DIR/bin/extract_esi"
  else
    echo "⚠️  extract_esi requested but binary not found in $BUILD_DIR/bin" >&2
  fi
fi

echo "✅ Build complete"
