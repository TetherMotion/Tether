# Tether Library - Modular CMakeLists.txt
# Supports both standalone CMake build and ESP-IDF component mode
#
# This builds multiple component libraries:
#   - tether_common: Shared utilities, types, configuration
#   - tether_hal: Hardware abstraction layer
#   - tether_controls: Core control algorithms (PID, LQR, LQG, Kalman, robust)
#   - tether_autotuning: Controller autotuning framework (classical + model-based)
#   - tether_gcode: G-code parsing and trajectory generation
#   - tether_export: CSV/SVG export utilities
#   - tether_motion_planner: Motion planning and replanning
#   - tether_motion_control: Real-time motion control and CiA 402 motion logic
#   - tether_identification: System identification (step/ freq/ friction/ subspace)
#   - tether_ethercat_common: Common EtherCAT types
#   - tether_ethercat_master: EtherCAT master core (protocol, mailbox, DC, FMMU, SII)
#   - tether_cia_profiles: CiA device profiles (301/401/402/404/405/406/408/410/417/430) + ETG5000
#   - tether_drives: Vendor-specific drive helpers (AS715N, DynaDrive, Nexcobot ESC211)
#   - tether_fsoe: Fail-Safe over EtherCAT (ETG 5100) safety protocol
#   - tether_ethercat_slave: EtherCAT slave emulation

# Check if we're being built as an ESP-IDF component
if(ESP_PLATFORM)
    include(${CMAKE_CURRENT_LIST_DIR}/CMakeLists_component.txt)
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(Tether VERSION 2.0.0 LANGUAGES C CXX)

# =============================================================================
# Git Submodule Check
# =============================================================================
set(SUBMODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libSLIPspeed")
if(NOT EXISTS "${SUBMODULE_PATH}")
    message(FATAL_ERROR
        "Git submodule 'libSLIPspeed' is not initialized.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
elseif(NOT EXISTS "${SUBMODULE_PATH}/include/SLIPStream/Buffer.hpp")
    message(FATAL_ERROR
        "Git submodule 'libSLIPspeed' appears to be initialized but is empty or incomplete.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
endif()

set(MAGIC_ENUM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/magic_enum")
if(NOT EXISTS "${MAGIC_ENUM_PATH}")
    message(FATAL_ERROR
        "Git submodule 'magic_enum' is not initialized.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
elseif(NOT EXISTS "${MAGIC_ENUM_PATH}/include/magic_enum/magic_enum.hpp")
    message(FATAL_ERROR
        "Git submodule 'magic_enum' appears to be initialized but is empty or incomplete.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
endif()

# Add magic_enum include directory
include_directories("${MAGIC_ENUM_PATH}/include")

set(ARGPARSE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/argparse")
if(NOT EXISTS "${ARGPARSE_PATH}")
    message(FATAL_ERROR
        "Git submodule 'argparse' is not initialized.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
elseif(NOT EXISTS "${ARGPARSE_PATH}/include/argparse/argparse.hpp")
    message(FATAL_ERROR
        "Git submodule 'argparse' appears to be initialized but is empty or incomplete.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
endif()

# Add argparse include directory (used by examples and tools)
include_directories("${ARGPARSE_PATH}/include")

# atomic_queue (lock-free MPMC queue for PDO mode queues)
set(ATOMIC_QUEUE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/atomic_queue")
if(NOT EXISTS "${ATOMIC_QUEUE_PATH}")
    message(FATAL_ERROR
        "Git submodule 'atomic_queue' is not initialized.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
elseif(NOT EXISTS "${ATOMIC_QUEUE_PATH}/include/atomic_queue/atomic_queue.h")
    message(FATAL_ERROR
        "Git submodule 'atomic_queue' appears to be initialized but is empty or incomplete.\n"
        "Please run the following command in the repository root:\n"
        "  git submodule update --init\n"
        "Then re-run CMake."
    )
endif()
include_directories("${ATOMIC_QUEUE_PATH}/include")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 99)

# Build type defaults to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Coverage support
option(TETHER_ENABLE_COVERAGE "Enable generation of coverage instrumentation and reports" OFF)
option(TETHER_BLEND_RUNTIME_CHECKS "Enable runtime blend validity checks (tangent/curvature verification)" ON)

if(TETHER_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "Coverage enabled: adding compiler flags")
        set(COVERAGE_COMPILE_FLAGS "-g -O0 --coverage")
        set(COVERAGE_LINK_FLAGS "--coverage")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
    else()
        message(WARNING "TETHER_ENABLE_COVERAGE set but compiler doesn't appear to be GCC/Clang; coverage flags not added")
    endif()
endif()

# =============================================================================
# TETHER_COMPONENTS — FindBoost-style component selection
# =============================================================================
# When a parent project sets TETHER_COMPONENTS (e.g. "ethercat_master"),
# Tether disables all optional components and enables only the requested
# ones plus their transitive dependencies.
#
# Component dependency graph:
#   fsoe             → cia_profiles, ethercat_master, common
#   cia_profiles     → ethercat_master, motion_control, common
#   drives           → ethercat_master, common
#   ethercat_master  → ethercat_common, controls, hal, common
#   ethercat_common  → hal, common
#   motion_control   → controls, common
#   identification   → common
#   controls         → common
#   autotuning       → common
#   hal              → common
#   common           → (none)
#
# When TETHER_COMPONENTS is not defined, all TETHER_BUILD_* defaults apply
# (standalone build behavior is unchanged).

# Feature Options (defaults — overridden by TETHER_COMPONENTS if set)
option(TETHER_BUILD_COMMON "Build common utilities" ON)
option(TETHER_BUILD_HAL "Build hardware abstraction layer" ON)
option(TETHER_BUILD_PCAP "Build pcap logging utilities" ON)
option(TETHER_BUILD_CONTROLS "Build core control algorithms" ON)
option(TETHER_BUILD_AUTOTUNING "Build controller autotuning framework" ON)
option(TETHER_BUILD_GCODE "Build G-code interpreter" ON)
option(TETHER_BUILD_EXPORT "Build export utilities" ON)
option(TETHER_BUILD_MOTION_PLANNER "Build motion planner" ON)
option(TETHER_BUILD_MOTION_CONTROL "Build motion control" ON)
option(TETHER_BUILD_IDENTIFICATION "Build system identification" ON)
option(TETHER_BUILD_ETHERCAT_COMMON "Build EtherCAT common" ON)
option(TETHER_BUILD_ETHERCAT_MASTER "Build EtherCAT master core" ON)
option(TETHER_BUILD_CIA_PROFILES "Build CiA device profiles and ETG5000" ON)
option(TETHER_BUILD_DRIVES "Build vendor-specific drive helpers" ON)
option(TETHER_BUILD_FSOE "Build Fail-Safe over EtherCAT (FSoE)" ON)
option(TETHER_BUILD_ETHERCAT_SLAVE "Build EtherCAT slave emulation" ON)
option(TETHER_BUILD_SIMULATION "Build simulation systems" ON)
option(TETHER_BUILD_DESTABILIZER "Build destabilizer adversarial testing" ON)
option(TETHER_BUILD_IO_PROTOCOL "Build IO protocol (SLIP-based parameter streaming)" ON)
option(TETHER_BUILD_TESTS "Build tests" ON)
option(TETHER_BUILD_EXAMPLES "Build examples" ON)
option(TETHER_BUILD_BENCHMARKS "Build benchmarks" OFF)

# Optional small tools (non-library utilities)
# Enabled by default: handy standalone utility to extract human-readable data from ESI XML
option(TETHER_BUILD_EXTRACT_ESI "Build extract_esi_xml utility" ON)

option(TETHER_BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
option(TETHER_BUILD_SHARED_LIBS "Build Tether component libraries as shared" ON)
option(TETHER_BUILD_STATIC_LIBS "Build Tether component libraries as static" OFF)

# EtherCAT-over-UDP encapsulation (default: OFF for zero overhead).
# When enabled, the master can wrap EtherCAT frames in Ethernet/IPv4/UDP.
option(TETHER_ENABLE_UDP_ENCAPSULATION "Enable EtherCAT-over-UDP encapsulation support" OFF)

# Conditional debug gating framework (default: ON).
# When disabled, all DebugGate code compiles to no-ops with zero overhead.
option(TETHER_DEBUG_GATE_ENABLED "Enable conditional debug gating framework" ON)

# --- Apply TETHER_COMPONENTS selection ---
if(DEFINED TETHER_COMPONENTS)
    # Transitive dependency map: component → list of dependencies
    set(_tether_component_deps_common          "")
    set(_tether_component_deps_hal             "common")
    set(_tether_component_deps_controls        "common")
    set(_tether_component_deps_autotuning      "common")
    set(_tether_component_deps_motion_control  "controls;common")
    set(_tether_component_deps_identification  "common")
    set(_tether_component_deps_ethercat_common "hal;common")
    set(_tether_component_deps_ethercat_master "ethercat_common;controls;hal;common")
    set(_tether_component_deps_cia_profiles    "ethercat_master;motion_control;common")
    set(_tether_component_deps_drives          "ethercat_master;common")
    set(_tether_component_deps_fsoe            "cia_profiles;ethercat_master;common")
    set(_tether_component_deps_ethercat_slave  "ethercat_common;hal;common")
    set(_tether_component_deps_pcap            "common")
    set(_tether_component_deps_gcode           "common")
    set(_tether_component_deps_export          "common")
    set(_tether_component_deps_motion_planner  "common")
    set(_tether_component_deps_simulation      "common")
    set(_tether_component_deps_io_protocol     "common")
    set(_tether_component_deps_destabilizer    "simulation;common")

    # All valid component names
    set(_tether_all_components
        common hal pcap controls autotuning gcode export motion_planner
        motion_control identification ethercat_common ethercat_master
        cia_profiles drives fsoe ethercat_slave simulation destabilizer
        io_protocol)

    # Resolve transitive closure of requested components
    set(_tether_enabled ${TETHER_COMPONENTS})
    set(_tether_queue ${TETHER_COMPONENTS})
    while(_tether_queue)
        list(POP_FRONT _tether_queue _comp)
        # Validate component name
        if(NOT _comp IN_LIST _tether_all_components)
            message(FATAL_ERROR "TETHER_COMPONENTS: unknown component '${_comp}'. "
                "Valid components: ${_tether_all_components}")
        endif()
        # Add dependencies not already in the enabled set
        foreach(_dep ${_tether_component_deps_${_comp}})
            if(NOT _dep IN_LIST _tether_enabled)
                list(APPEND _tether_enabled ${_dep})
                list(APPEND _tether_queue ${_dep})
            endif()
        endforeach()
    endwhile()

    # Disable everything first
    set(_tether_build_vars
        COMMON HAL PCAP CONTROLS AUTOTUNING GCODE EXPORT MOTION_PLANNER
        MOTION_CONTROL IDENTIFICATION ETHERCAT_COMMON ETHERCAT_MASTER
        CIA_PROFILES DRIVES FSOE ETHERCAT_SLAVE SIMULATION
        DESTABILIZER IO_PROTOCOL)
    foreach(_var ${_tether_build_vars})
        set(TETHER_BUILD_${_var} OFF CACHE BOOL "" FORCE)
    endforeach()

    # Enable only the resolved components
    foreach(_comp ${_tether_enabled})
        string(TOUPPER ${_comp} _comp_upper)
        set(TETHER_BUILD_${_comp_upper} ON CACHE BOOL "" FORCE)
    endforeach()

    # Disable non-library targets when used as a subproject
    set(TETHER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(TETHER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TETHER_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(TETHER_BUILD_EXTRACT_ESI OFF CACHE BOOL "" FORCE)
    set(TETHER_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)

    message(STATUS "Tether: TETHER_COMPONENTS=${TETHER_COMPONENTS}")
    message(STATUS "Tether: resolved components (with deps): ${_tether_enabled}")
endif()

if(NOT TETHER_BUILD_SHARED_LIBS AND NOT TETHER_BUILD_STATIC_LIBS)
    message(FATAL_ERROR "At least one of TETHER_BUILD_SHARED_LIBS or TETHER_BUILD_STATIC_LIBS must be ON")
endif()

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Platform detection
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(TETHER_PLATFORM_LINUX ON)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(TETHER_PLATFORM_WINDOWS ON)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(TETHER_PLATFORM_MACOS ON)
endif()

# Set root directory for component cmake files
set(TETHER_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Place all executables in a single `bin` directory at the top-level build dir (no subdirectories)
# and place libraries/archives in `lib` next to it. This enforces a flat layout for tools and
# examples so their binaries are all found in ${CMAKE_BINARY_DIR}/bin.
set(TETHER_BIN_DIR "${CMAKE_BINARY_DIR}/bin")

# Single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TETHER_BIN_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
else()
    # Multi-config generators (Visual Studio, Xcode) require per-configuration variables
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${cfg} CFG)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFG} ${TETHER_BIN_DIR})
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CFG} "${CMAKE_BINARY_DIR}/lib")
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CFG} "${CMAKE_BINARY_DIR}/lib")
    endforeach()
endif()

# Generate config header
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tether/TetherConfig.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/tether/TetherConfig.hpp
)

# =============================================================================
# Include Component CMake Files
# =============================================================================

# Common (always built - foundation for everything)
if(TETHER_BUILD_COMMON)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/common.cmake)
endif()

# PcapNG logging
if(TETHER_BUILD_PCAP)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/pcap.cmake)
endif()

# HAL
if(TETHER_BUILD_HAL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/hal.cmake)
endif()

# Controls (core controllers)
if(TETHER_BUILD_CONTROLS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/controls.cmake)
endif()

# Autotuning (depends on common; fully self-contained)
if(TETHER_BUILD_AUTOTUNING)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/autotuning.cmake)
endif()

# GCode
if(TETHER_BUILD_GCODE)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/gcode.cmake)
endif()

# Export
if(TETHER_BUILD_EXPORT)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/export.cmake)
endif()

# Motion Planner
if(TETHER_BUILD_MOTION_PLANNER)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/motion_planner.cmake)
endif()

# Identification (depends on common; fully self-contained)
if(TETHER_BUILD_IDENTIFICATION)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/identification.cmake)
endif()

# Motion Control
if(TETHER_BUILD_MOTION_CONTROL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/motion_control.cmake)
endif()

# EtherCAT Common
if(TETHER_BUILD_ETHERCAT_COMMON)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/ethercat_common.cmake)
endif()

# ESI XML parser (separate from ethercat_common to keep tinyxml2 out of the
# runtime shared library; only needed by tools/tests/examples that parse ESI files)
if(TETHER_BUILD_EXTRACT_ESI)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/esi.cmake)
endif()

# If we are building the EtherCAT master, enable the feature macro so
# host-only stubs and conditional tests are excluded automatically.
if(TETHER_BUILD_ETHERCAT_MASTER)
    add_compile_definitions(TETHER_ENABLE_ETHERCAT=1)
endif()

# Propagate blend runtime checks option to all targets
if(TETHER_BLEND_RUNTIME_CHECKS)
    add_compile_definitions(TETHER_BLEND_RUNTIME_CHECKS=1)
else()
    add_compile_definitions(TETHER_BLEND_RUNTIME_CHECKS=0)
endif()

# SDO diagnostic tracing
if(TETHER_DIAG_SDO_IO)
    add_compile_definitions(TETHER_DIAG_SDO_IO=1)
endif()

# EtherCAT-over-UDP encapsulation
if(TETHER_ENABLE_UDP_ENCAPSULATION)
    add_compile_definitions(TETHER_ENABLE_UDP_ENCAPSULATION=1)
else()
    add_compile_definitions(TETHER_ENABLE_UDP_ENCAPSULATION=0)
endif()

# Conditional debug gating framework
if(TETHER_DEBUG_GATE_ENABLED)
    add_compile_definitions(TETHER_DEBUG_GATE_ENABLED=1)
else()
    add_compile_definitions(TETHER_DEBUG_GATE_ENABLED=0)
endif()

# EtherCAT Master (core: protocol, mailbox, raw, FMMU, SII, reset)
if(TETHER_BUILD_ETHERCAT_MASTER)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/ethercat_master.cmake)
endif()

# CiA device profiles + ETG5000 (depends on ethercat_master and motion_control)
if(TETHER_BUILD_CIA_PROFILES AND TETHER_BUILD_ETHERCAT_MASTER AND TETHER_BUILD_MOTION_CONTROL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/cia_profiles.cmake)
endif()

# Vendor-specific drive helpers (depends on ethercat_master)
if(TETHER_BUILD_DRIVES AND TETHER_BUILD_ETHERCAT_MASTER)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/drives.cmake)
endif()

# FSoE — Fail-Safe over EtherCAT (depends on ethercat_master and cia_profiles)
if(TETHER_BUILD_FSOE AND TETHER_BUILD_ETHERCAT_MASTER AND TETHER_BUILD_CIA_PROFILES AND TETHER_BUILD_MOTION_CONTROL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/fsoe.cmake)
endif()

# EtherCAT Slave
if(TETHER_BUILD_ETHERCAT_SLAVE)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/ethercat_slave.cmake)
endif()

# Simulation
if(TETHER_BUILD_SIMULATION)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/simulation.cmake)
endif()

# Destabilizer (requires simulation)
if(TETHER_BUILD_DESTABILIZER AND TETHER_BUILD_SIMULATION)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/destabilizer.cmake)
endif()

# IO Protocol (SLIP-based parameter/signal streaming)
if(TETHER_BUILD_IO_PROTOCOL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components/io_protocol.cmake)
endif()

# =============================================================================
# Create Combined "tether" Library (for backward compatibility)
# =============================================================================
# TETHER_ALL_TARGETS collects the real (non-alias) library targets for installation.
set(TETHER_ALL_TARGETS "")

# Helper to collect real targets from component variables
macro(tether_collect_targets var_prefix)
    if(DEFINED ${var_prefix}_TARGETS)
        foreach(_tgt ${${var_prefix}_TARGETS})
            if(TARGET ${_tgt})
                list(APPEND TETHER_ALL_TARGETS ${_tgt})
            endif()
        endforeach()
    endif()
endmacro()

tether_collect_targets(TETHER_COMMON)
tether_collect_targets(TETHER_PCAP)
tether_collect_targets(TETHER_HAL)
tether_collect_targets(TETHER_CONTROLS)
tether_collect_targets(TETHER_AUTOTUNING)
tether_collect_targets(TETHER_GCODE)
tether_collect_targets(TETHER_EXPORT)
tether_collect_targets(TETHER_MOTION_PLANNER)
tether_collect_targets(TETHER_MOTION_CONTROL)
tether_collect_targets(TETHER_IDENTIFICATION)
tether_collect_targets(TETHER_ETHERCAT_COMMON)
tether_collect_targets(TETHER_ETHERCAT_MASTER)
tether_collect_targets(TETHER_CIA_PROFILES)
tether_collect_targets(TETHER_DRIVES)
tether_collect_targets(TETHER_FSOE)
tether_collect_targets(TETHER_ETHERCAT_SLAVE)
tether_collect_targets(TETHER_SIMULATION)
tether_collect_targets(TETHER_DESTABILIZER)
tether_collect_targets(TETHER_IO_PROTOCOL)
tether_collect_targets(TETHER_SLIPSPEED)

# Aliases for consumer linking (shared preferred, then static)
set(TETHER_ALL_COMPONENTS "")
if(TARGET tether_common)
    list(APPEND TETHER_ALL_COMPONENTS tether_common)
endif()
if(TARGET tether_hal)
    list(APPEND TETHER_ALL_COMPONENTS tether_hal)
endif()
if(TARGET tether_controls)
    list(APPEND TETHER_ALL_COMPONENTS tether_controls)
endif()
if(TARGET tether_autotuning)
    list(APPEND TETHER_ALL_COMPONENTS tether_autotuning)
endif()
if(TARGET tether_gcode)
    list(APPEND TETHER_ALL_COMPONENTS tether_gcode)
endif()
if(TARGET tether_export)
    list(APPEND TETHER_ALL_COMPONENTS tether_export)
endif()
if(TARGET tether_motion_planner)
    list(APPEND TETHER_ALL_COMPONENTS tether_motion_planner)
endif()
if(TARGET tether_motion_control)
    list(APPEND TETHER_ALL_COMPONENTS tether_motion_control)
endif()
if(TARGET tether_identification)
    list(APPEND TETHER_ALL_COMPONENTS tether_identification)
endif()
if(TARGET tether_ethercat_common)
    list(APPEND TETHER_ALL_COMPONENTS tether_ethercat_common)
endif()
if(TARGET tether_ethercat_master)
    list(APPEND TETHER_ALL_COMPONENTS tether_ethercat_master)
endif()
if(TARGET tether_cia_profiles)
    list(APPEND TETHER_ALL_COMPONENTS tether_cia_profiles)
endif()
if(TARGET tether_drives)
    list(APPEND TETHER_ALL_COMPONENTS tether_drives)
endif()
if(TARGET tether_fsoe)
    list(APPEND TETHER_ALL_COMPONENTS tether_fsoe)
endif()
if(TARGET tether_ethercat_slave)
    list(APPEND TETHER_ALL_COMPONENTS tether_ethercat_slave)
endif()
if(TARGET tether_simulation)
    list(APPEND TETHER_ALL_COMPONENTS tether_simulation)
endif()
if(TARGET tether_destabilizer)
    list(APPEND TETHER_ALL_COMPONENTS tether_destabilizer)
endif()
if(TARGET tether_io_protocol)
    list(APPEND TETHER_ALL_COMPONENTS tether_io_protocol)
endif()
if(TARGET slipspeed)
    list(APPEND TETHER_ALL_COMPONENTS slipspeed)
endif()

# Create interface library that links all components
add_library(tether INTERFACE)
target_link_libraries(tether INTERFACE ${TETHER_ALL_COMPONENTS})

# Add include directories to the interface library
target_include_directories(tether INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tether>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/tether>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/tether>
)

# =============================================================================
# Tests
# =============================================================================
if(TETHER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# Examples
# =============================================================================
if(TETHER_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# =============================================================================
# Benchmarks
# =============================================================================
if(TETHER_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# =============================================================================
# Tools
# =============================================================================
if(TETHER_BUILD_EXTRACT_ESI)
    add_subdirectory(tools/extract_esi)
endif()


# =============================================================================
# Python Bindings
# =============================================================================
if(TETHER_BUILD_PYTHON_BINDINGS)
    if(TETHER_ENABLE_COVERAGE)
        message(STATUS "TETHER_ENABLE_COVERAGE=ON; skipping Python bindings to avoid unrelated build failures during coverage runs.")
        set(TETHER_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Python bindings" FORCE)
    else()
        # Ensure Python development files are present before trying to build bindings.
        # If not found, disable the bindings so the rest of the project can configure normally.
        find_package(Python3 COMPONENTS Interpreter Development QUIET)
        if(Python3_Development_FOUND)
            add_subdirectory(python_bindings)
        else()
            message(STATUS "Python development files or headers not found; disabling Python bindings.")
            set(TETHER_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Python bindings" FORCE)
        endif()
    endif()
endif()

# =============================================================================
# Installation
# =============================================================================
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install all component libraries (real targets only, aliases cannot be installed)
foreach(component ${TETHER_ALL_TARGETS})
    install(TARGETS ${component}
        EXPORT TetherTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endforeach()

# Install interface library
install(TARGETS tether
    EXPORT TetherTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/tether
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/tether/TetherConfig.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tether
)

# CMake package config
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/TetherConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TetherConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Tether
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/TetherConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT TetherTargets
    FILE TetherTargets.cmake
    NAMESPACE Tether::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Tether
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/TetherConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/TetherConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Tether
)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Tether Library Configuration Summary:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "  Components:")
message(STATUS "    tether_common:         ${TETHER_BUILD_COMMON}")
message(STATUS "    tether_pcap:           ${TETHER_BUILD_PCAP}")
message(STATUS "    tether_hal:            ${TETHER_BUILD_HAL}")
message(STATUS "    tether_controls:       ${TETHER_BUILD_CONTROLS}")
message(STATUS "    tether_autotuning:     ${TETHER_BUILD_AUTOTUNING}")
message(STATUS "    tether_gcode:          ${TETHER_BUILD_GCODE}")
message(STATUS "    tether_export:         ${TETHER_BUILD_EXPORT}")
message(STATUS "    tether_motion_planner: ${TETHER_BUILD_MOTION_PLANNER}")
message(STATUS "    tether_motion_control: ${TETHER_BUILD_MOTION_CONTROL}")
message(STATUS "    tether_identification: ${TETHER_BUILD_IDENTIFICATION}")
message(STATUS "    tether_ethercat_common: ${TETHER_BUILD_ETHERCAT_COMMON}")
message(STATUS "    tether_ethercat_master: ${TETHER_BUILD_ETHERCAT_MASTER}")
message(STATUS "    tether_cia_profiles:   ${TETHER_BUILD_CIA_PROFILES}")
message(STATUS "    tether_drives:         ${TETHER_BUILD_DRIVES}")
message(STATUS "    tether_fsoe:           ${TETHER_BUILD_FSOE}")
message(STATUS "    tether_ethercat_slave: ${TETHER_BUILD_ETHERCAT_SLAVE}")
message(STATUS "    tether_simulation:    ${TETHER_BUILD_SIMULATION}")
message(STATUS "    tether_io_protocol:   ${TETHER_BUILD_IO_PROTOCOL}")
message(STATUS "")
message(STATUS "  Optional:")
message(STATUS "    Tests:           ${TETHER_BUILD_TESTS}")
message(STATUS "    Examples:        ${TETHER_BUILD_EXAMPLES}")
message(STATUS "    Benchmarks:      ${TETHER_BUILD_BENCHMARKS}")
message(STATUS "    Python bindings: ${TETHER_BUILD_PYTHON_BINDINGS}")
message(STATUS "  Shared libs:      ${TETHER_BUILD_SHARED_LIBS}")
message(STATUS "  Static libs:      ${TETHER_BUILD_STATIC_LIBS}")
message(STATUS "  Binaries:         ${TETHER_BIN_DIR}")
message(STATUS "")
