# Examples CMakeLists.txt
#
# This file builds examples that demonstrate Tether library functionality.
# Each example links against the appropriate component libraries.

# Shared example utilities (argument parsing, debug flags, VLAN, host setup)
add_library(tether_examples_common STATIC
    common/ExampleHelpers.cpp
    common/EtherCATHostSetup.cpp)
target_link_libraries(tether_examples_common PRIVATE
    tether_ethercat_master
    tether_ethercat_common
    tether_hal
    tether_common)
target_include_directories(tether_examples_common PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(tether_examples_common PRIVATE cxx_std_20)

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
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---AS715N Sine Motion + FSoE Native Example ---
# Adds an optional Synapticon-style FSoE safe-motion main instance to the sine loop.
add_tether_example(as715n_sine_motion_fsoe_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---PBLR81FGF Sine Motion Native Example ---
# Copy of AS715N example for the PBLR81FGF drive (see TODOs in source)
add_tether_example(pblr81fgf_sine_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---PBLR81FGF Step Motion Native Example ---
# CSP position steps with jerk-limited S-curve ramps
add_tether_example(pblr81fgf_step_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---AS715N Error-code inspector example ---
# Reads manufacturer / CiA402 error codes and optionally resets them.
add_tether_example(as715n_check_error_code
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---EtherCAT SII dump example ---
# Dumps the parsed SII for a given slave index (host + embedded)
add_tether_example(ethercat_dump_sii
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---DynaDrive Sine Motion Native Example ---
# DynaDrive (ANYdrive / rsl_drive_sdk) position sine wave using Type-E PDOs
add_tether_example(dynadrive_sine_motion_native
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---EtherCAT slave detection example ---
# Scans the bus, reports discovered slaves with SII summary
add_tether_example(detect_slaves
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- EtherCAT AL reset example ---
# Resets selected slaves to a target ESM state via AL Control
add_tether_example(reset_slaves_al
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- EtherCAT slave identity reader example ---
# Configures mailbox from SII, transitions to PRE-OP, and reads identity object 0x1018
add_tether_example(slave_read_identity
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---ATI Axia80 F/T Sensor streaming example ---
# Streams 6-DOF force/torque data with engineering-unit conversion
add_tether_example(axia80_stream
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_esi tether_hal tether_common tether_examples_common)

# --- Nexcobot ESC211 Safety DI monitor ---
# Reads Safety DI A/B via CoE/SDO and renders a live ncurses panel
find_package(Curses)
if(CURSES_FOUND)
    add_tether_example(esc211_di_monitor
        LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(esc211_di_monitor PRIVATE ${CURSES_INCLUDE_DIRS})
else()
    message(STATUS "ncurses not found; skipping esc211_di_monitor")
endif()

# --- pcapng interpreter example ---
add_tether_example(interpret_pcapng
    LIBRARIES tether_pcap tether_ethercat_common tether_common)

# --- WebSocket UI backends ---
# These examples use Boost.Beast/Boost.Asio (header-only). They are only built
# when Boost is available so the rest of the examples build without it.
find_package(Boost QUIET)
if(Boost_FOUND)
    add_executable(tether_ui_backend
        tether_ui_backend.cpp
        WebSocketTransport.cpp)
    target_link_libraries(tether_ui_backend PRIVATE
        tether_io_protocol
        tether_gcode
        tether_ethercat_slave
        tether_common
        Boost::headers)
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
        tether_common
        Boost::headers)
    target_include_directories(controlme_ui_backend PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/tether
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(controlme_ui_backend PRIVATE cxx_std_20)
else()
    message(STATUS "Boost not found; skipping WebSocket UI backend examples "
                   "(tether_ui_backend, controlme_ui_backend)")
endif()

# --- Generic EtherCAT Slave Emulator Example ---
# Emulates a generic EtherCAT slave on a real network interface using SlaveCore
add_tether_example(slave_emulator
    LIBRARIES tether_ethercat_slave tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Kinco RP20 I/O module initialization demo ---
# Auto-discovers RP20 I/O modules across all slaves/slots, initializes via CoE,
# and runs a cyclic I/O loop. Supports --interactive ncurses UI.
find_package(Curses)
if(CURSES_FOUND)
    add_tether_example(kinco_rp20_io
        LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(kinco_rp20_io PRIVATE ${CURSES_INCLUDE_DIRS})
    target_compile_definitions(kinco_rp20_io PRIVATE HAVE_NCURSES)
else()
    add_tether_example(kinco_rp20_io
        LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)
    message(STATUS "ncurses not found; kinco_rp20_io built without --interactive mode")
endif()

# --- Kinco RP20 module discovery demo ---
# Auto-discovers RP20 I/O modules across all slaves/slots and prints the list.
# No PDO/cyclic I/O — just discovery and listing.
add_tether_example(kinco_rp20_list_modules
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

message(STATUS "Examples configured")
