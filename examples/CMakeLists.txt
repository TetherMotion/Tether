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
# Conditionally link tether_esi so examples can use ESIFile("path.xml").
# When tether_esi is built (TETHER_BUILD_EXTRACT_ESI or another consumer
# enables esi.cmake), TETHER_HAVE_ESI=1 is propagated and ESIFile's
# file-parsing constructor is available. Otherwise ESIFile(path) aborts
# with a critical error at runtime.
if(TARGET tether_esi)
    target_link_libraries(tether_examples_common PUBLIC tether_esi)
endif()

# --- Tether IO binary WebSocket example ---
# Drogon delivers arbitrary binary WebSocket chunks to a queue-backed
# ITransport, while Session remains the single protocol implementation.
if(Drogon_FOUND AND TARGET tether_io_protocol AND TARGET Drogon::Drogon)
    add_executable(tether_io_drogon_websocket
        tether_io_drogon_websocket.cpp
        TetherIoWebSocketController.cpp)
    target_link_libraries(tether_io_drogon_websocket PRIVATE
        tether_io_protocol Drogon::Drogon)
    target_include_directories(tether_io_drogon_websocket PRIVATE
        ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(tether_io_drogon_websocket PRIVATE cxx_std_20)
endif()
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
# NOTE: disabled — uses the old PathBuilder/BezierCurve stack which has
# been replaced by tether::motion::PathBlender/NurbsCurve. To be ported.
# add_tether_example(gcode_analytical_planner
#     LIBRARIES tether_gcode tether_motion_planner tether_export tether_common)

# Minimal G64 visualizer example: parses X/Y moves and exports them to SVG
add_tether_example(g64_demo_export
    LIBRARIES tether_export tether_common)

# --- G-Code Path Comparison Example ---
# Parses a G-code file, blends corners with G64 path deviation tolerance
# (or --exact-stop for G61), and exports an SVG showing the original G-code
# path (gray) overlaid with the Tether-planned path (red).
add_tether_example(gcode_path_compare
    LIBRARIES tether_gcode tether_motion_planner tether_motion_geometry tether_export tether_common)

# --- WSS Velocity/Acceleration Plot Example ---
# Parses a G-code file, runs the Pareto time-energy optimal velocity planner,
# and exports velocity and acceleration curves as SVG plots from the WSS arcs.
add_tether_example(wss_velocity_plot
    LIBRARIES tether_gcode tether_motion_planner tether_motion_geometry tether_common)

# ---AS715N Sine Motion Native Example ---
# Demonstrates EtherCAT master with CiA 402 drive control
if(TETHER_DRIVE_AS715N)
add_tether_example(as715n_sine_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---AS715N Sine Motion + FSoE Native Example ---
# Adds an optional Synapticon-style FSoE safe-motion main instance to the sine loop.
add_tether_example(as715n_sine_motion_fsoe_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---Synapticon SOMANET CST + FSoE Example ---
# Puts a Synapticon SOMANET (AS715N) drive into Cyclic Sync Torque mode,
# maps RxPDO 0x1702 / TxPDO 0x1B02, sends 0 torque, and runs the FSoE
# safe-motion protocol with rich diagnostics.
if(TETHER_DRIVE_SYNAPTICON)
add_tether_example(synapticon_cst_fsoe
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---Synapticon SOMANET FSoE-Only Example ---
# Maps ONLY the FSoE safety PDOs (RxPDO 0x1700 / TxPDO 0x1B00) using the
# multi-PDO-per-sync-manager API (Slave::configureMultiPDOs).  No CiA 402
# process data PDOs are mapped — establishes FSoE safety communication only.
add_tether_example(synapticon_fsoe_only
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---PBLR81FGF Sine Motion Native Example ---
# Copy of AS715N example for the PBLR81FGF drive (see TODOs in source)
if(TETHER_DRIVE_PBLR81FGF)
add_tether_example(pblr81fgf_sine_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---PBLR81FGF Step Motion Native Example ---
# CSP position steps with jerk-limited S-curve ramps
add_tether_example(pblr81fgf_step_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---AS715N Error-code inspector example ---
# Reads manufacturer / CiA402 error codes and optionally resets them.
if(TETHER_DRIVE_AS715N)
add_tether_example(as715n_check_error_code
    LIBRARIES tether_ethercat_master tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---EtherCAT SII dump example ---
# Dumps the parsed SII for a given slave index (host + embedded)
add_tether_example(ethercat_dump_sii
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---DynaDrive Sine Motion Native Example ---
# DynaDrive (ANYdrive / rsl_drive_sdk) position sine wave using Type-E PDOs
if(TETHER_DRIVE_DYNADRIVE)
add_tether_example(dynadrive_sine_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

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

# --- DC SYNC0/SYNC1 characterization example ---
# Brings up a single slave, probes SYNC0/SYNC1 support, then sweeps SYNC0
# cycle times until the slave reports an AL error.
add_tether_example(dc_sync_characterize
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Realtime kernel + network diagnostics example ---
# Runs a fake PDO loop while sweeping priorities, frequencies and CPU affinity,
# reporting color-coded timing, transmit latency and CPU-usage statistics.
add_tether_example(realtime_diagnostics
    LIBRARIES tether_examples_common tether_hal tether_common)

# ---ATI Axia80 F/T Sensor streaming example ---
# Streams 6-DOF force/torque data with engineering-unit conversion.
# Links tether_esi for optional --esi-xml register verification, so it is
# only built when the ESI parser library is available
# (i.e. TETHER_BUILD_EXTRACT_ESI or another consumer enables esi.cmake).
if(TARGET tether_esi)
    add_tether_example(axia80_stream
        LIBRARIES tether_ethercat_master tether_ethercat_common tether_esi tether_hal tether_common tether_examples_common)
else()
    message(STATUS "tether_esi not built; skipping axia80_stream (enable TETHER_BUILD_EXTRACT_ESI to build it)")
endif()

# --- Nexcobot ESC211 Safety DI monitor ---
# Reads Safety DI A/B via CoE/SDO and renders a live ncurses panel
if(TETHER_DRIVE_NEXCOBOT_ESC211)
find_package(Curses)
if(CURSES_FOUND)
    add_tether_example(esc211_di_monitor
        LIBRARIES tether_ethercat_master tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(esc211_di_monitor PRIVATE ${CURSES_INCLUDE_DIRS})
else()
    message(STATUS "ncurses not found; skipping esc211_di_monitor")
endif()
endif()

# --- pcapng interpreter example ---
add_tether_example(interpret_pcapng
    LIBRARIES tether_pcap tether_ethercat_common tether_common)

# --- WebSocket UI backends ---
# These examples use Boost.Beast/Boost.Asio (header-only). They are only built
# when Boost is available so the rest of the examples build without it.
find_package(Boost QUIET)
if(Boost_FOUND AND TARGET Boost::headers)
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
if(TETHER_DRIVE_RP20)
find_package(Curses)
if(CURSES_FOUND)
    add_tether_example(kinco_rp20_io
        LIBRARIES tether_ethercat_master tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(kinco_rp20_io PRIVATE ${CURSES_INCLUDE_DIRS})
    target_compile_definitions(kinco_rp20_io PRIVATE HAVE_NCURSES)
else()
    add_tether_example(kinco_rp20_io
        LIBRARIES tether_ethercat_master tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)
    message(STATUS "ncurses not found; kinco_rp20_io built without --interactive mode")
endif()

# --- Kinco RP20 module discovery demo ---
# Auto-discovers RP20 I/O modules across all slaves/slots and prints the list.
# No PDO/cyclic I/O — just discovery and listing.
add_tether_example(kinco_rp20_list_modules
    LIBRARIES tether_ethercat_master tether_drives tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# --- Phase 5.4 PH fast-path benchmark ---
# Informational benchmark: reports planning-time ratio PH-on vs PH-off
# on a multi-corner zigzag path. Not a hard CI gate.
add_tether_example(ph_benchmark
    SOURCE PHBenchmark.cpp
    LIBRARIES tether_motion_planner tether_motion_geometry tether_common tether_gcode)

message(STATUS "Examples configured")

# --- Klipper protocol examples ---
if(TETHER_BUILD_KLIPPER)
    add_tether_example(klipper_loopback_demo
        SOURCE klipper_loopback_demo.cpp
        LIBRARIES Tether::Klipper tether_common tether_motion_planner tether_gcode tether_export)
    add_tether_example(klipper_motion_blocks
        SOURCE klipper_motion_blocks.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_clock_sync
        SOURCE klipper_clock_sync.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_gpio_pwm
        SOURCE klipper_gpio_pwm.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_klippy_instance
        SOURCE klipper_klippy_instance.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_delta_kinematics
        SOURCE klipper_delta_kinematics.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_thermal_control
        SOURCE klipper_thermal_control.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_multi_mcu
        SOURCE klipper_multi_mcu.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_gcode_commands
        SOURCE klipper_gcode_commands.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_bed_leveling
        SOURCE klipper_bed_leveling.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_moonraker_api
        SOURCE klipper_moonraker_api.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_printer_objects
        SOURCE klipper_printer_objects.cpp
        LIBRARIES Tether::Klipper tether_common)
    add_tether_example(klipper_config_macros
        SOURCE klipper_config_macros.cpp
        LIBRARIES Tether::Klipper tether_common)

    # HTTP/Mainsail example requires the optional HTTP component
    if(TARGET Tether::KlipperHttp)
        add_tether_example(klipper_http_mainsail
            SOURCE klipper_http_mainsail.cpp
            LIBRARIES Tether::KlipperHttp Tether::Klipper tether_common)
    endif()
endif()
