# Examples CMakeLists.txt
#
# This file builds examples that demonstrate Tether library functionality.
# Each example links against the appropriate component libraries.

# Helper function to create an example executable
function(add_tether_example name)
    set(options)
    set(oneValueArgs SOURCE)
    set(multiValueArgs LIBRARIES)
    cmake_parse_arguments(EX "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    if(NOT EX_SOURCE)
        set(EX_SOURCE ${name}.cpp)
    endif()
    
    if(NOT EX_LIBRARIES)
        set(EX_LIBRARIES tether_common)
    endif()
    
    add_executable(${name} ${EX_SOURCE})
    target_link_libraries(${name} PRIVATE ${EX_LIBRARIES})
    target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(${name} PRIVATE cxx_std_20)
endfunction()

# --- Simple Example ---
# Demonstrates basic G-code parsing and trajectory generation
add_tether_example(simple_gcode_example
    LIBRARIES tether_gcode tether_export tether_common)

# --- G-Code Analytical Planner Example ---
add_tether_example(gcode_analytical_planner
    LIBRARIES tether_gcode tether_motion_planner tether_export tether_common)

# Minimal G64 visualizer example: parses X/Y moves and exports them to SVG
add_tether_example(g64_demo_export
    LIBRARIES tether_export tether_common)

# ---AS715N Sine Motion Native Example ---
# Demonstrates EtherCAT master with CiA 402 drive control
add_tether_example(as715n_sine_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---AS715N Sine Motion + FSoE Native Example ---
# Adds an optional Synapticon-style FSoE safe-motion main instance to the sine loop.
add_tether_example(as715n_sine_motion_fsoe_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---PBLR81FGF Sine Motion Native Example ---
# Copy of AS715N example for the PBLR81FGF drive (see TODOs in source)
add_tether_example(pblr81fgf_sine_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---PBLR81FGF Step Motion Native Example ---
# CSP position steps with jerk-limited S-curve ramps
add_tether_example(pblr81fgf_step_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---AS715N Error-code inspector example ---
# Reads manufacturer / CiA402 error codes and optionally resets them.
add_tether_example(as715n_check_error_code
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---EtherCAT SII dump example ---
# Dumps the parsed SII for a given slave index (host + embedded)
add_tether_example(ethercat_dump_sii
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

# ---EtherCAT slave detection example ---
# Scans the bus, reports discovered slaves with SII summary
add_tether_example(detect_slaves
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common)

add_executable(tether_ui_backend
    tether_ui_backend.cpp
    WebSocketTransport.cpp)
target_link_libraries(tether_ui_backend PRIVATE
    tether_io_protocol
    tether_gcode
    tether_ethercat_slave
    tether_common)
target_include_directories(tether_ui_backend PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/tether
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(tether_ui_backend PRIVATE cxx_std_20)

add_executable(controlme_ui_backend
    controlme_ui_backend.cpp
    WebSocketTransport.cpp)
target_link_libraries(controlme_ui_backend PRIVATE
    tether_io_protocol
    tether_simulation
    tether_controls
    tether_common)
target_include_directories(controlme_ui_backend PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/tether
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(controlme_ui_backend PRIVATE cxx_std_20)

message(STATUS "Examples configured")
