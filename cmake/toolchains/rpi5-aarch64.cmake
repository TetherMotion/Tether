# CMake toolchain file: Raspberry Pi 5 (BCM2712, Cortex-A76, 64-bit OS)
#
# Usage:
#   cmake --preset rpi5
#   # or manually:
#   cmake -B build-rpi5 \
#       -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/rpi5-aarch64.cmake
#
# A cross toolchain must be installed; put its bin/ on PATH or set
# RPI_TOOLCHAIN_ROOT to its prefix. Suitable toolchains:
#   - Debian/Ubuntu:  apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#   - Bootlin:        aarch64--glibc--stable-* from https://toolchains.bootlin.com
#   - crosstool-NG:   aarch64-linux-gnu
# See rpi-common.cmake for RPI_TOOLCHAIN_ROOT / RPI_TOOLCHAIN_TRIPLE /
# RPI_SYSROOT / RPI_STATIC_RUNTIME / RPI_FULL_STATIC.

set(TETHER_RPI_SYSTEM_PROCESSOR aarch64)
set(TETHER_RPI_TRIPLES
    aarch64-linux-gnu
    aarch64-buildroot-linux-gnu
    aarch64-rpi3-linux-gnu
    aarch64-none-linux-gnu)
set(TETHER_RPI_ARCH_FLAGS "-mcpu=cortex-a76")

include("${CMAKE_CURRENT_LIST_DIR}/rpi-common.cmake")
