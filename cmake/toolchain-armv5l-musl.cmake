# Cross toolchain: Linux ARMv5TE soft-float (armel), musl libc, fully static.
# For targets like MikroTik EN7562CT (hEX S 2025 / hEX Refresh / hAP ax S)
# which only execute arm32v5 binaries.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(_TC "$ENV{HOME}/opt/armv5l-linux-musleabi-cross/bin/armv5l-linux-musleabi-")
set(CMAKE_C_COMPILER   "${_TC}gcc")
set(CMAKE_CXX_COMPILER "${_TC}g++")
set(CMAKE_AR           "${_TC}ar")
set(CMAKE_RANLIB       "${_TC}ranlib")

set(CMAKE_C_FLAGS_INIT   "-march=armv5te -mfloat-abi=soft")
set(CMAKE_CXX_FLAGS_INIT "-march=armv5te -mfloat-abi=soft")
# Fully static so the binary runs in any container (no musl loader needed).
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static")
# ARMv5 lacks 8-byte atomic instructions; GCC emits __atomic_*_8 libcalls that
# live in libatomic. STANDARD_LIBRARIES appends it at the end of the link line
# (after the static libs that reference it), which is required for resolution.
set(CMAKE_C_STANDARD_LIBRARIES "-latomic")
set(CMAKE_CXX_STANDARD_LIBRARIES "-latomic")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
