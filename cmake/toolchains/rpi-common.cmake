# Shared discovery logic for the Tether Raspberry Pi cross toolchain files.
#
# Not meant to be used as a CMAKE_TOOLCHAIN_FILE directly — the per-target
# files (rpi5-aarch64.cmake, rpi1-armv6.cmake) set the variables below and
# then include this file:
#
#   TETHER_RPI_SYSTEM_PROCESSOR   -> value for CMAKE_SYSTEM_PROCESSOR
#   TETHER_RPI_TRIPLES            -> candidate GNU triples, first match wins
#   TETHER_RPI_ARCH_FLAGS         -> -mcpu/-march/-mfpu/-mabi flags
#   TETHER_RPI_STATIC_RUNTIME_DEFAULT (optional, default ON)
#
# User-facing knobs (pass via -D<var>=... or as environment variables):
#   RPI_TOOLCHAIN_ROOT   toolchain install prefix containing bin/<triple>-gcc
#                        (a Bootlin toolchain dir, a crosstool-NG x-tools dir,
#                        or raspberrypi/tools/arm-bcm2708/<toolchain-name>)
#   RPI_TOOLCHAIN_TRIPLE force a specific GNU triple instead of probing
#   RPI_SYSROOT          target sysroot; auto-detected from the compiler when
#                        unset (headers/libs from the Pi's OS image — needed for
#                        optional deps like ncurses/Drogon to be found)
#   RPI_STATIC_RUNTIME   link libstdc++/libgcc statically so the binaries run on
#                        a Pi OS whose libstdc++ is older than the toolchain's
#   RPI_FULL_STATIC      link fully static executables (-static, no runtime
#                        glibc/libstdc++ dependency at all)
#
# If RPI_TOOLCHAIN_ROOT is unset the <triple>-gcc binaries are looked up on
# PATH (e.g. Debian's g++-aarch64-linux-gnu / g++-arm-linux-gnueabihf).

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ${TETHER_RPI_SYSTEM_PROCESSOR})

# ---------------------------------------------------------------------------
# Resolve user knobs (env fallback -> cache)
# ---------------------------------------------------------------------------
foreach(_rpi_var RPI_TOOLCHAIN_ROOT RPI_TOOLCHAIN_TRIPLE RPI_SYSROOT)
    if("${${_rpi_var}}" STREQUAL "" AND DEFINED ENV{${_rpi_var}})
        set(${_rpi_var} "$ENV{${_rpi_var}}")
    endif()
endforeach()
set(RPI_TOOLCHAIN_ROOT   "${RPI_TOOLCHAIN_ROOT}"   CACHE PATH
    "Cross toolchain prefix containing bin/<triple>-gcc")
set(RPI_TOOLCHAIN_TRIPLE "${RPI_TOOLCHAIN_TRIPLE}" CACHE STRING
    "GNU triple of the cross toolchain (e.g. aarch64-linux-gnu)")
set(RPI_SYSROOT          "${RPI_SYSROOT}"          CACHE PATH
    "Target sysroot with the Pi's headers/libraries")

# try_compile() (check_include_file_cxx, check_cxx_source_compiles, ...) runs
# this toolchain file again in a throwaway project that does not inherit the
# cache. Forward the knobs so those checks use the same toolchain setup.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
    RPI_TOOLCHAIN_ROOT RPI_TOOLCHAIN_TRIPLE RPI_SYSROOT
    RPI_STATIC_RUNTIME RPI_FULL_STATIC)

# ---------------------------------------------------------------------------
# Locate the cross compiler
# ---------------------------------------------------------------------------
if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
    set(_tether_rpi_triples ${TETHER_RPI_TRIPLES})
    if(RPI_TOOLCHAIN_TRIPLE)
        set(_tether_rpi_triples ${RPI_TOOLCHAIN_TRIPLE})
    endif()

    set(_tether_rpi_triple "")
    # NOTE: do NOT pre-initialize _tether_rpi_cc — find_program() skips its
    # search when the variable is already defined to a non-NOTFOUND value.
    foreach(_triple ${_tether_rpi_triples})
        if(RPI_TOOLCHAIN_ROOT)
            find_program(_tether_rpi_cc
                NAMES ${_triple}-gcc
                PATHS "${RPI_TOOLCHAIN_ROOT}/bin"
                NO_DEFAULT_PATH)
        else()
            find_program(_tether_rpi_cc NAMES ${_triple}-gcc)
        endif()
        if(_tether_rpi_cc)
            set(_tether_rpi_triple ${_triple})
            break()
        endif()
        unset(_tether_rpi_cc CACHE)
    endforeach()

    if(NOT _tether_rpi_cc)
        string(REPLACE ";" "-gcc, " _tether_rpi_tried "${_tether_rpi_triples}-gcc")
        message(FATAL_ERROR
            "Raspberry Pi cross compiler not found (tried: ${_tether_rpi_tried}).\n"
            "Install a toolchain and either put it on PATH or point\n"
            "  RPI_TOOLCHAIN_ROOT=/path/to/toolchain   (env var or -D)\n"
            "at the prefix containing bin/<triple>-gcc. Suitable toolchains:\n"
            "  - Debian:    apt install g++-aarch64-linux-gnu (Pi 5) resp. an\n"
            "               armv6 toolchain for Pi 1 (Debian armhf is armv7 and\n"
            "               will NOT run on a Pi 1)\n"
            "  - Bootlin:   https://toolchains.bootlin.com (aarch64--glibc resp.\n"
            "               armv6-eabihf, recent GCC)\n"
            "  - crosstool-NG or raspberrypi/tools (GCC >= 11 required for C++23)")
    endif()

    get_filename_component(_tether_rpi_bindir "${_tether_rpi_cc}" DIRECTORY)
    set(CMAKE_C_COMPILER   "${_tether_rpi_bindir}/${_tether_rpi_triple}-gcc")
    set(CMAKE_CXX_COMPILER "${_tether_rpi_bindir}/${_tether_rpi_triple}-g++")
    if(NOT EXISTS "${CMAKE_CXX_COMPILER}")
        message(FATAL_ERROR
            "Found ${CMAKE_C_COMPILER} but no matching C++ compiler\n"
            "${CMAKE_CXX_COMPILER}. Install the toolchain's g++ component.")
    endif()
    set(TETHER_RPI_TRIPLE "${_tether_rpi_triple}" CACHE INTERNAL
        "Resolved GNU triple of the Raspberry Pi cross toolchain")
else()
    # Compilers were set externally (or by try_compile) — recover triple/bindir
    # from the C compiler path for sysroot/binutils discovery.
    get_filename_component(_tether_rpi_bindir "${CMAKE_C_COMPILER}" DIRECTORY)
    get_filename_component(_tether_rpi_ccname "${CMAKE_C_COMPILER}" NAME)
    string(REGEX REPLACE "-gcc(\\.exe)?$" "" _tether_rpi_triple "${_tether_rpi_ccname}")
endif()

# ---------------------------------------------------------------------------
# Binutils (optional; CMake works without them, explicit paths are nicer)
# ---------------------------------------------------------------------------
foreach(_tool ar ranlib nm objdump objcopy strip)
    string(TOUPPER ${_tool} _upper)
    if(NOT CMAKE_${_upper})
        find_program(CMAKE_${_upper}
            NAMES ${_tether_rpi_triple}-${_tool} ${_tool}
            HINTS "${_tether_rpi_bindir}")
    endif()
endforeach()
if(NOT CMAKE_LINKER)
    find_program(CMAKE_LINKER
        NAMES ${_tether_rpi_triple}-ld ld
        HINTS "${_tether_rpi_bindir}")
endif()

# ---------------------------------------------------------------------------
# Sysroot
# ---------------------------------------------------------------------------
get_filename_component(_tether_rpi_root "${_tether_rpi_bindir}/.." ABSOLUTE)

if(NOT RPI_SYSROOT)
    # crosstool-NG / Bootlin / raspberrypi-tools compilers report their
    # built-in sysroot here; distro cross-gcc typically prints nothing or "/".
    execute_process(COMMAND "${CMAKE_C_COMPILER}" -print-sysroot
        OUTPUT_VARIABLE _tether_rpi_sysroot
        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(_tether_rpi_sysroot
            AND NOT _tether_rpi_sysroot STREQUAL "/"
            AND IS_DIRECTORY "${_tether_rpi_sysroot}")
        set(RPI_SYSROOT "${_tether_rpi_sysroot}")
    else()
        # Layout guesses: crosstool-NG <root>/<triple>/sysroot,
        # raspberrypi/tools <root>/<triple>/libc.
        foreach(_cand
                "${_tether_rpi_root}/${_tether_rpi_triple}/sysroot"
                "${_tether_rpi_root}/${_tether_rpi_triple}/libc/sysroot"
                "${_tether_rpi_root}/${_tether_rpi_triple}/libc")
            if(IS_DIRECTORY "${_cand}/usr/include" OR IS_DIRECTORY "${_cand}/include")
                set(RPI_SYSROOT "${_cand}")
                break()
            endif()
        endforeach()
    endif()
    if(RPI_SYSROOT)
        set(RPI_SYSROOT "${RPI_SYSROOT}" CACHE PATH
            "Target sysroot with the Pi's headers/libraries" FORCE)
    endif()
endif()

if(RPI_SYSROOT)
    set(CMAKE_SYSROOT "${RPI_SYSROOT}")
endif()

# Restrict find_library/find_path/find_package to the target environment;
# <root>/<triple> additionally covers Debian's /usr/<triple> layout. Host
# programs (python, git, ...) still come from the host PATH.
set(CMAKE_FIND_ROOT_PATH ${RPI_SYSROOT} "${_tether_rpi_root}/${_tether_rpi_triple}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------------------------------------------------------------------------
# Compile/link flags
# ---------------------------------------------------------------------------
set(CMAKE_C_FLAGS_INIT   "${TETHER_RPI_ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${TETHER_RPI_ARCH_FLAGS}")

if(NOT DEFINED TETHER_RPI_STATIC_RUNTIME_DEFAULT)
    set(TETHER_RPI_STATIC_RUNTIME_DEFAULT ON)
endif()
set(RPI_STATIC_RUNTIME ${TETHER_RPI_STATIC_RUNTIME_DEFAULT} CACHE BOOL
    "Link libstdc++/libgcc statically for Raspberry Pi targets")
set(RPI_FULL_STATIC OFF CACHE BOOL
    "Link fully static executables (-static) for Raspberry Pi targets")

set(_tether_rpi_ldflags "")
if(RPI_FULL_STATIC)
    set(_tether_rpi_ldflags "-static")
elseif(RPI_STATIC_RUNTIME)
    # -static-libstdc++ silently falls back to the shared lib if libstdc++.a
    # is missing; check so the user gets a warning instead of surprise.
    execute_process(COMMAND "${CMAKE_C_COMPILER}" -print-file-name=libstdc++.a
        OUTPUT_VARIABLE _tether_rpi_libstdcxx
        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(_tether_rpi_libstdcxx MATCHES "/")
        set(_tether_rpi_ldflags "-static-libstdc++ -static-libgcc")
    endif()
endif()

set(CMAKE_EXE_LINKER_FLAGS_INIT    "${TETHER_RPI_ARCH_FLAGS} ${_tether_rpi_ldflags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${TETHER_RPI_ARCH_FLAGS} ${_tether_rpi_ldflags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${TETHER_RPI_ARCH_FLAGS} ${_tether_rpi_ldflags}")

# ---------------------------------------------------------------------------
# Announce (once; this file is included several times per configure)
# ---------------------------------------------------------------------------
if(NOT TETHER_RPI_TOOLCHAIN_ANNOUNCED)
    set(TETHER_RPI_TOOLCHAIN_ANNOUNCED TRUE CACHE INTERNAL "")
    execute_process(COMMAND "${CMAKE_C_COMPILER}" -dumpversion
        OUTPUT_VARIABLE _tether_rpi_gccver
        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    message(STATUS "Raspberry Pi cross toolchain:")
    message(STATUS "  processor : ${CMAKE_SYSTEM_PROCESSOR}")
    message(STATUS "  triple    : ${_tether_rpi_triple}")
    message(STATUS "  C compiler: ${CMAKE_C_COMPILER} (GCC ${_tether_rpi_gccver})")
    message(STATUS "  C++       : ${CMAKE_CXX_COMPILER}")
    if(RPI_SYSROOT)
        message(STATUS "  sysroot   : ${RPI_SYSROOT}")
    else()
        message(STATUS "  sysroot   : <compiler default>")
    endif()
    message(STATUS "  arch flags: ${TETHER_RPI_ARCH_FLAGS}")
    if(RPI_FULL_STATIC)
        message(STATUS "  link      : fully static (-static)")
    elseif(_tether_rpi_ldflags)
        message(STATUS "  link      : ${_tether_rpi_ldflags}")
    else()
        message(STATUS "  link      : dynamic (RPI_STATIC_RUNTIME unavailable: no libstdc++.a)")
    endif()

    string(REGEX MATCH "^[0-9]+" _tether_rpi_gcc_major "${_tether_rpi_gccver}")
    if(_tether_rpi_gcc_major AND _tether_rpi_gcc_major LESS 11)
        message(WARNING
            "Cross GCC ${_tether_rpi_gccver} is too old for Tether's C++23 "
            "codebase — GCC >= 11 is required (>= 13 recommended for native "
            "<format>). Choose a newer toolchain.")
    endif()
endif()
