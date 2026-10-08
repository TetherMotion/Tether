# CMake toolchain file: Raspberry Pi 1 B+ (BCM2835, ARM1176JZF-S, ARMv6 hard-float)
#
# Usage:
#   cmake --preset rpi1
#   # or manually:
#   cmake -B build-rpi1 \
#       -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/rpi1-armv6.cmake
#
# IMPORTANT: a Pi 1 needs an ARMv6 hard-float toolchain — regular Debian armhf
# (arm-linux-gnueabihf built for ARMv7) will NOT run on ARM1176JZF-S. Suitable
# toolchains:
#   - Bootlin:     armv6-eabihf--glibc--stable-* from https://toolchains.bootlin.com
#                 (recent GCC — recommended; ships as arm-buildroot-linux-gnueabihf)
#   - crosstool-NG: armv6-rpi-linux-gnueabihf or arm-linux-gnueabihf configured
#                 for armv6 (GCC >= 11 required for Tether's C++23 code)
#   - raspberrypi/tools arm-bcm2708 works only for old checkouts — its bundled
#     GCC (4.9/8.3) is too old for this codebase.
# See rpi-common.cmake for RPI_TOOLCHAIN_ROOT / RPI_TOOLCHAIN_TRIPLE /
# RPI_SYSROOT / RPI_STATIC_RUNTIME / RPI_FULL_STATIC.

set(TETHER_RPI_SYSTEM_PROCESSOR arm)
set(TETHER_RPI_TRIPLES
    arm-linux-gnueabihf
    arm-buildroot-linux-gnueabihf
    armv6-rpi-linux-gnueabihf
    arm-none-linux-gnueabihf)
set(TETHER_RPI_ARCH_FLAGS "-mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard")

include("${CMAKE_CURRENT_LIST_DIR}/rpi-common.cmake")
