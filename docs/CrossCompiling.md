# Cross-Compiling Tether for the Raspberry Pi

This guide explains how to build Tether on an Ubuntu x86-64 host for a
Raspberry Pi target (armv7hf / aarch64). The same recipe works for any
Linux/ARM SBC (BeagleBone, Orange Pi, etc.) — just swap the toolchain and
`CMAKE_FIND_ROOT_PATH` for the matching sysroot.

## Why cross-compile?

Building Tether natively on a Pi works, but a modern desktop CPU is one to
two orders of magnitude faster. Cross-compiling also keeps the target
device free of compiler toolchains, headers, and CMake caches, which is
useful for production images.

## Prerequisites on the host (Ubuntu 22.04 / 24.04)

```bash
# Baseline build tools
sudo apt update
sudo apt install -y build-essential cmake ninja-build git pkg-config

# GNU cross toolchains (pick the one matching your Pi)
# 32-bit Pi Zero/1/2/3 (armv6 / armv7 hard-float, arm-linux-gnueabihf)
sudo apt install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

# 64-bit Pi 3/4/5 (aarch64, aarch64-linux-gnu)
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

The Debian/Ubuntu `g++-arm-linux-gnueabihf` packages ship a GCC that
supports C++23 (GCC 12+ on 22.04, GCC 13/14 on 24.04). Tether requires
C++23, so on older Ubuntu releases you may need a newer toolchain — see
[Alternative toolchains](#alternative-toolchains) below.

## 1. Get the target sysroot

A bare compiler is not enough — you also need the target's libc, libstdc++,
and any system libraries Tether links against (pthread, etc.). The easiest
way to obtain a matching sysroot is to mount the Pi's root filesystem over
SSH and mirror it locally:

```bash
# From the repo root
mkdir -p sysroot
rsync -avz --copy-links \
    pi@<pi-host>:/lib     sysroot/
rsync -avz --copy-links \
    pi@<pi-host>:/usr/lib sysroot/usr/
rsync -avz --copy-links \
    pi@<pi-host>:/usr/include sysroot/usr/
```

For a clean, reproducible sysroot prefer one of:

- **`multistrap` / `debootstrap --foreign --arch=armhf`** — build a minimal
  Debian armhf/arm64 rootfs on disk.
- **A Pi OS image** — loop-mount `2024-xx-xx-raspios-*.img` and rsync out of
  it. This guarantees the libraries match what is actually on the device.
- **`pdebuild-cross` / `dpkg-cross`** — Debian's official cross-build
  workflow if you intend to package Tether as a `.deb`.

Fix up absolute symlinks that point into the Pi's `/` so they resolve
inside `sysroot/`:

```bash
sudo scripts/fix-sysroot-symlinks.sh sysroot   # see snippet below
```

<details><summary><code>scripts/fix-sysroot-symlinks.sh</code></summary>

```bash
#!/usr/bin/env bash
# Rewrite absolute symlinks inside a rsync'd sysroot so they resolve
# relative to the sysroot directory rather than the host's /.
set -euo pipefail
SYSROOT="$1"
find "$SYSROOT" -xtype l -lname '/*' -printf '%p -> %l\n' | while IFS= read -r line; do
    link="${line%% -> *}"
    target="${line##* -> }"
    rel="$(python3 -c "import os,sys;print(os.path.relpath(sys.argv[1], os.path.dirname(sys.argv[2])))" "$SYSROOT$target" "$link")"
    ln -sf "$rel" "$link"
done
```

</details>

## 2. Toolchain file

CMake cross-compiles via a *toolchain file* that sets the system name,
compilers, and the search root for libraries/headers. Create
`cmake/arm-pi.toolchain.cmake` in the repo:

```cmake
# cmake/arm-pi.toolchain.cmake
# Cross-compile Tether for a Raspberry Pi (32-bit armhf) from an Ubuntu host.
# Swap the triple to aarch64-linux-gnu for 64-bit Pi 3/4/5.
set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TRIPLE arm-linux-gnueabihf)

set(CMAKE_C_COMPILER   ${TRIPLE}-gcc)
set(CMAKE_CXX_COMPILER ${TRIPLE}-g++)
set(CMAKE_AR           ${TRIPLE}-ar      CACHE FILEPATH "")
set(CMAKE_RANLIB       ${TRIPLE}-ranlib  CACHE FILEPATH "")
set(CMAKE_STRIP        ${TRIPLE}-strip   CACHE FILEPATH "")

# Point CMake at the rsync'd sysroot. Adjust the path to wherever you
# staged /lib, /usr/lib, /usr/include in step 1.
set(SYSROOT "$ENV{PI_SYSROOT}" CACHE PATH "Path to the Pi sysroot")
if(NOT SYSROOT)
    set(SYSROOT "${CMAKE_CURRENT_LIST_DIR}/../sysroot")
endif()

set(CMAKE_SYSROOT           ${SYSROOT})
set(CMAKE_FIND_ROOT_PATH    ${SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # use host tools (cmake, ninja)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)    # target libs only
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)    # target headers only
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Tether uses C++23; the stock Debian armhf g++ supports it.
set(CMAKE_CXX_STANDARD 23)
```

For a 64-bit target, copy the file to `cmake/aarch64-pi.toolchain.cmake`
and change `TRIPLE` to `aarch64-linux-gnu` and `CMAKE_SYSTEM_PROCESSOR` to
`aarch64`.

## 3. Configure and build

Tether's optional features (Klipper HTTP server, Python bindings, tests,
examples) pull in host-only dependencies (Drogon, pybind11, gtest) that are
painful to cross-compile. Disable them for the target build:

```bash
# From the repo root
cmake -B build-pi -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/arm-pi.toolchain.cmake \
    -DPI_SYSROOT=$PWD/sysroot \
    -DTETHER_BUILD_TESTS=OFF \
    -DTETHER_BUILD_EXAMPLES=OFF \
    -DTETHER_BUILD_BENCHMARKS=OFF \
    -DTETHER_BUILD_PYTHON_BINDINGS=OFF \
    -DTETHER_BUILD_EXTRACT_ESI=OFF \
    -DTETHER_ENABLE_KLIPPER_HTTP=OFF \
    -DTETHER_BUILD_SHARED_LIBS=ON \
    -DTETHER_BUILD_STATIC_LIBS=OFF

cmake --build build-pi -j$(nproc)
```

If you only need a subset of Tether (e.g. just the EtherCAT master for a
Pi-based CNC controller), use `TETHER_COMPONENTS` to build only that
subtree and its transitive dependencies:

```bash
cmake -B build-pi -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/arm-pi.toolchain.cmake \
    -DPI_SYSROOT=$PWD/sysroot \
    -DTETHER_COMPONENTS="ethercat_master;motion_control;klipper" \
    -DTETHER_BUILD_SHARED_LIBS=ON
```

`TETHER_COMPONENTS` automatically disables tests, examples, benchmarks,
extract_esi, and python bindings — see the component dependency graph in
<ref_file file="/home/uli/dev/Tether/CMakeLists.txt" />.

## 4. Verify the binaries are for the target

```bash
file build-pi/bin/*  build-pi/lib/*.so*
# expected: ELF 32-bit LSB pie executable, ARM, EABI5,
#           hard-float ABI, version 1 (SYSV), dynamically linked ...
```

You can also sanity-check dynamic linkage against the sysroot:

```bash
arm-linux-gnueabihf-readelf -d build-pi/lib/libtether_hal.so | grep NEEDED
```

## 5. Deploy to the Pi

```bash
# Ship the shared libraries and any binaries you built
rsync -avz build-pi/lib/libtether_*.so* pi@<pi-host>:/usr/local/lib/
rsync -avz build-pi/bin/<your-binary>    pi@<pi-host>:/usr/local/bin/

# Refresh the dynamic linker cache on the Pi
ssh pi@<pi-host> sudo ldconfig
```

For a static build (`-DTETHER_BUILD_STATIC_LIBS=ON
-DTETHER_BUILD_SHARED_LIBS=OFF`) you only need to copy the final
executable; it has no runtime library dependencies beyond glibc/libstdc++.

## Optional dependencies

| Dependency | Cross-compile notes |
|------------|---------------------|
| `pthread` | Part of glibc — already in the sysroot, no extra step. |
| `magic_enum`, `argparse`, `atomic_queue`, `glaze`, `eigen`, `libSLIPspeed` | All git submodules, header-only or self-contained. They build from source with the cross toolchain automatically. Run `git submodule update --init` first. |
| `jsoncpp` | Auto-fetched via `FetchContent` if missing. Builds cleanly with the cross compiler. |
| `tinyxml2` (extract_esi) | Auto-fetched. Disable with `-DTETHER_BUILD_EXTRACT_ESI=OFF` (recommended for target builds). |
| `Drogon` (Klipper HTTP) | Heavy dependency tree (trantor, jsoncpp, optionally pg/sqlite/mysql). Strongly recommended to leave `-DTETHER_ENABLE_KLIPPER_HTTP=OFF` for Pi targets. If you need it, build Drogon separately with the same toolchain and point `CMAKE_PREFIX_PATH` at the install prefix. |
| `pybind11` (Python bindings) | Requires the target Python dev headers in the sysroot. Usually disabled for Pi builds. |
| `gtest` (tests) | Host-only — keep `-DTETHER_BUILD_TESTS=OFF` for target builds. Run tests natively on the Pi if needed. |
| SocketCAN (`TETHER_ENABLE_KLIPPER_CAN`) | Header-only on Linux; just install `linux-libc-dev:armhf` into the sysroot or rsync `/usr/include/linux` from the Pi. |

## Running tests on the target

Cross-compiled tests cannot run on the host. To run them on the Pi:

```bash
# Build tests for the target
cmake -B build-pi-test -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/arm-pi.toolchain.cmake \
    -DPI_SYSROOT=$PWD/sysroot \
    -DTETHER_BUILD_TESTS=ON \
    -DTETHER_BUILD_EXAMPLES=OFF \
    -DTETHER_ENABLE_KLIPPER_HTTP=OFF \
    -DTETHER_BUILD_PYTHON_BINDINGS=OFF

cmake --build build-pi-test -j$(nproc) --target tether_klipper_tests

# Ship and run
rsync -avz build-pi-test/bin/tests/ pi@<pi-host>:/home/pi/tether-tests/
ssh pi@<pi-host> '/home/pi/tether-tests/tether_klipper_tests --gtest_filter="-ThermalIntegrationTest.*"'
```

## Alternative toolchains

If the stock Ubuntu cross-compiler is too old for C++23:

- **Linaro** — prebuilt ARM GCC binaries from
  https://releases.linaro.org/components/toolchain/binaries/ . Download,
  extract, and point `CMAKE_C_COMPILER` / `CMAKE_CXX_COMPILER` at the
  absolute paths.
- **crosstool-NG** — build a custom GCC/PIC/binutils combo with exactly the
  flags you want (`./ct-ng menuconfig`, enable C++23, hard-float, the
  matching `--with-cpu=cortex-a72` for Pi 4, etc.).
- **LLVM/Clang cross** — `clang++ --target=arm-linux-gnueabihf
  --sysroot=$PWD/sysroot` works without a separate clang build per target;
  just install `clang` and the GNU linker for the triple.

## Common pitfalls

- **`cannot find -lstdc++` / `crt1.o: No such file`** — the sysroot is
  missing or `CMAKE_FIND_ROOT_PATH` does not point at it. Re-check step 1
  and that `PI_SYSROOT` resolves.
- **`find_package(Drogon)` fails** — expected; pass
  `-DTETHER_ENABLE_KLIPPER_HTTP=OFF` for target builds.
- **CMake picks up host `/usr/lib/x86_64-linux-gnu` libraries** — make sure
  `CMAKE_FIND_ROOT_PATH_MODE_LIBRARY` is `ONLY` and that you did not also
  add host paths to `CMAKE_PREFIX_PATH`.
- **`/lib/ld-linux-armhf.so.3: No such file or directory`** when running on
  the Pi — you copied a 64-bit binary to a 32-bit Pi (or vice versa). Check
  `file` output and the toolchain triple.
- **Hard-float vs soft-float mismatch** — Pi 2/3/4 use `gnueabihf`
  (hard-float). Pi Zero/W also use `gnueabihf`. Only original Pi 1 with a
  soft-float distro needs `gnueabi`.
- **Submodule headers missing** — `git submodule update --init` must be run
  on the host before configuring; the CMakeLists aborts otherwise.
- **`std::format` not found** — your cross-compiler is older than GCC 13.
  Upgrade via Linaro/crosstool-NG, or fall back to a newer Ubuntu.

## See also

- <ref_file file="/home/uli/dev/Tether/docs/HAL_PORTING_GUIDE.md" /> —
  HAL interfaces and platform implementations (LinuxHAL is what runs on
  the Pi).
- <ref_file file="/home/uli/dev/Tether/AGENTS.md" /> — native build
  commands and feature flags.
- <ref_file file="/home/uli/dev/Tether/CMakeLists.txt" /> — full list of
  `TETHER_BUILD_*` and `TETHER_ENABLE_*` options.
