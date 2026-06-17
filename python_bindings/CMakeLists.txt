cmake_minimum_required(VERSION 3.16)
project(TetherPythonBindings CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Try to find pybind11
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.11.1
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# Parent Tether library location
set(TETHER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(TETHER_INCLUDE_DIR "${TETHER_ROOT}/include")
set(TETHER_SRC_DIR "${TETHER_ROOT}/src")

# Common include directories for all bindings
set(TETHER_COMMON_INCLUDES
    ${TETHER_INCLUDE_DIR}
    ${TETHER_INCLUDE_DIR}/tether
    ${TETHER_SRC_DIR}
)

# =============================================================================
# Module: _gcode
# G-code parsing and trajectory generation
# =============================================================================
pybind11_add_module(_gcode MODULE
    bindings/gcode_bindings.cpp
)
target_include_directories(_gcode PRIVATE ${TETHER_COMMON_INCLUDES})
target_link_libraries(_gcode PRIVATE tether_gcode tether_common)
target_compile_definitions(_gcode PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _export  
# SVG/CSV export utilities
# =============================================================================
pybind11_add_module(_export MODULE
    bindings/export_bindings.cpp
)
target_include_directories(_export PRIVATE ${TETHER_COMMON_INCLUDES})
target_link_libraries(_export PRIVATE tether_export tether_gcode tether_common)
target_compile_definitions(_export PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _motion_planner (placeholder for now)
# =============================================================================
pybind11_add_module(_motion_planner MODULE
    bindings/motion_planner_stub.cpp
)
target_compile_definitions(_motion_planner PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _motion_control (placeholder for now)
# =============================================================================
pybind11_add_module(_motion_control MODULE
    bindings/motion_control_stub.cpp
)
target_compile_definitions(_motion_control PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _controls (placeholder for now)
# =============================================================================
pybind11_add_module(_controls MODULE
    bindings/controls_stub.cpp
)
target_compile_definitions(_controls PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _ethercat_common (placeholder for now)
# =============================================================================
pybind11_add_module(_ethercat_common MODULE
    bindings/ethercat_common_stub.cpp
)
target_compile_definitions(_ethercat_common PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _ethercat_master (placeholder for now)
# =============================================================================
pybind11_add_module(_ethercat_master MODULE
    bindings/ethercat_master_stub.cpp
)
target_compile_definitions(_ethercat_master PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _ethercat_slave (placeholder for now)
# =============================================================================
pybind11_add_module(_ethercat_slave MODULE
    bindings/ethercat_slave_stub.cpp
)
target_compile_definitions(_ethercat_slave PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _cia402 (placeholder for now)
# =============================================================================
pybind11_add_module(_cia402 MODULE
    bindings/cia402_stub.cpp
)
target_compile_definitions(_cia402 PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _cia401 (placeholder for now)
# =============================================================================
pybind11_add_module(_cia401 MODULE
    bindings/cia401_stub.cpp
)
target_compile_definitions(_cia401 PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# =============================================================================
# Module: _fsoe (placeholder for now)
# =============================================================================
pybind11_add_module(_fsoe MODULE
    bindings/fsoe_stub.cpp
)
target_compile_definitions(_fsoe PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# Set RPATH so Python modules find shared libraries at runtime without LD_LIBRARY_PATH
if(UNIX AND NOT APPLE)
    set_target_properties(
        _gcode _export _motion_planner _motion_control _controls
        _ethercat_common _ethercat_master _ethercat_slave
        _cia402 _cia401 _fsoe
        PROPERTIES INSTALL_RPATH "$ORIGIN/../lib"
    )
endif()

# =============================================================================
# Installation
# =============================================================================
install(TARGETS 
    _gcode _export _motion_planner _motion_control _controls
    _ethercat_common _ethercat_master _ethercat_slave 
    _cia402 _cia401 _fsoe
    LIBRARY DESTINATION tether
)

# Install Python source files
install(DIRECTORY src/tether/ 
    DESTINATION tether
    FILES_MATCHING PATTERN "*.py"
)
