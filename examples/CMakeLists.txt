# Examples CMakeLists.txt
#
# This file builds examples that demonstrate Tether library functionality.
# Each example links against the appropriate component libraries.

set(TETHER_EXAMPLE_TARGETS "")

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
# Drogon delivers complete WebSocket frames to the TetherIOWebSocketController,
# which feeds them to Session with Framing::None (no SLIP — WebSocket already
# provides message boundaries).
if(Drogon_FOUND AND TARGET tether_io_protocol AND TARGET Drogon::Drogon)
    add_executable(tether_io_drogon_websocket
        tether_io_drogon_websocket.cpp
        TetherIOWebSocketController.cpp)
    list(APPEND TETHER_EXAMPLE_TARGETS tether_io_drogon_websocket)
    target_link_libraries(tether_io_drogon_websocket PRIVATE
        tether_io_protocol Drogon::Drogon)
    target_include_directories(tether_io_drogon_websocket PRIVATE
        ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(tether_io_drogon_websocket PRIVATE cxx_std_20)

    # --- Web dashboard example ---
    # Serves the pre-built Tether IO dashboard (web/tether-io-dashboard/dist/)
    # as static files alongside the binary WebSocket endpoint, so a single
    # process provides both the UI and the data backend.
    set(WEB_DASHBOARD_DIST_DIR "${PROJECT_SOURCE_DIR}/web/tether-io-dashboard/dist")
    add_executable(web_dashboard_example
        web_dashboard_example.cpp
        TetherIOWebSocketController.cpp)
    list(APPEND TETHER_EXAMPLE_TARGETS web_dashboard_example)
    target_link_libraries(web_dashboard_example PRIVATE
        tether_io_protocol Drogon::Drogon)
    target_include_directories(web_dashboard_example PRIVATE
        ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_definitions(web_dashboard_example PRIVATE
        WEB_DASHBOARD_DIST_DIR="${WEB_DASHBOARD_DIST_DIR}")
    target_compile_features(web_dashboard_example PRIVATE cxx_std_20)
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
    set(TETHER_EXAMPLE_TARGETS ${TETHER_EXAMPLE_TARGETS} ${name} PARENT_SCOPE)
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
add_tether_example(as715n_sine_motion
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

endif()

# ---Synapticon SOMANET CST Example (no FSoE) ---
# Puts a Synapticon SOMANET drive into Cyclic Sync Torque mode,
# maps RxPDO 0x1600 / TxPDO 0x1A00, and sends a sinusoidal torque command.
if(TETHER_DRIVE_SYNAPTICON)
add_tether_example(synapticon_cst
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---Synapticon SOMANET FSoE Example (no motion) ---
# Runs ONLY the FSoE safe-motion protocol over the drive's safety PDOs
# (RxPDO 0x1700 / TxPDO 0x1B00).  The CiA 402 drive is never enabled.
add_tether_example(synapticon_cst_fsoe
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---Synapticon SOMANET FSoE-Only Example ---
# Maps ONLY the FSoE safety PDOs (RxPDO 0x1700 / TxPDO 0x1B00) using the
# multi-PDO-per-sync-manager API (Slave::configureMultiPDOs).  No CiA 402
# process data PDOs are mapped — establishes FSoE safety communication only.
add_tether_example(synapticon_fsoe_only
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---PBLR81FGF Sine Motion Native Example ---
# Copy of AS715N example for the PBLR81FGF drive (see TODOs in source)
if(TETHER_DRIVE_PBLR81FGF)
add_tether_example(pblr81fgf_sine_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---PBLR81FGF Step Motion Native Example ---
# CSP position steps with jerk-limited S-curve ramps
add_tether_example(pblr81fgf_step_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---AS715N Error-code inspector example ---
# Reads manufacturer / CiA402 error codes and optionally resets them.
if(TETHER_DRIVE_AS715N)
add_tether_example(as715n_check_error_code
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---AS715N Sensorless Homing Example ---
# Moves a single AS715N against a mechanical stop using a low, clamped
# CST torque from a closed-loop velocity PI controller. Once the drive
# stalls, it sets the current position as home (method 35) and exits.
add_tether_example(as715n_sensorless_homing
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_controls tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---AS715N Fault Reset Example ---
# Dedicated fault-reset utility using the AS715N-specific mechanism
# (0x2031:01 F31.00 after clearing S-ON, or 0x2031:02 software reset).
add_tether_example(as715n_reset_errors
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---EtherCAT SII dump example ---
# Dumps the parsed SII for a given slave index (host + embedded)
add_tether_example(ethercat_dump_sii
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# ---DynaDrive Sine Motion Native Example ---
# DynaDrive (ANYdrive / rsl_drive_sdk) position sine wave using Type-E PDOs
if(TETHER_DRIVE_DYNADRIVE)
add_tether_example(dynadrive_sine_motion_native
    LIBRARIES tether_ethercat_master tether_cia_profiles tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)
endif()

# ---EtherCAT slave listing example ---
# Scans the bus, reports discovered slaves with SII summary
add_tether_example(list_slaves
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- EtherCAT AL reset example ---
# Resets selected slaves to a target ESM state via AL Control
add_tether_example(reset_slaves_al
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff digital output terminal demo ---
# Auto-detects all EL200x terminals (EL2002/2004/2008 — x = channel count)
# via the MultiOutputTerminal driver, brings them to OP, and steps a single
# active output bit across all modules on each Enter keypress.  Shows a
# navigable coupler/terminal device tree when tether_terminal_ui is built
# (ncurses); otherwise stream mode only.
if(TETHER_DRIVER_BECKHOFF AND TARGET tether_device_drivers)
add_tether_example(beckhoff_output_toggle
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff digital input terminal monitor ---
# Auto-detects all EL1014 terminals via the MultiEL1014 driver, brings them
# to OP, and shows the live input state of every channel — same device-tree
# TUI as the output demo.  Without tether_terminal_ui it still builds
# (stream mode only).
add_tether_example(beckhoff_input_monitor
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Generic PDO mapping probe ---
# Tests how a slave's PDO mapping objects react to arbitrary configurations:
# writes Rx/Tx PDOs with custom entries, requests SAFE-OP with a short
# timeout, and reports the AL status code (e.g. 0x0024 Invalid input
# mapping).  Supports entry-count / bit-size / total-size sweeps and a
# negative battery, with an A-B-A integrity re-check after every failure.
add_tether_example(pdo_mapping_probe
    LIBRARIES tether_ethercat_master tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff analog input terminal monitor ---
# Auto-detects all EL3xxx analog-input terminals via the
# MultiAnalogInputTerminal driver (mailbox + SM3 process inputs, per-channel
# layout resolved from SII), brings them to OP, and shows every channel's
# live value + status word — same device-tree TUI.
add_tether_example(beckhoff_analog_input_monitor
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff analog output terminal sweep demo ---
# Auto-detects all EL4xxx analog-output terminals via the
# MultiAnalogOutputTerminal driver (mailbox + SM2 process outputs) and
# sweeps one channel at a time with a triangle ramp.
add_tether_example(beckhoff_analog_output_sweep
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff position/encoder input terminal monitor ---
# Auto-detects all EL5xxx position/encoder terminals via the
# MultiPositionInputTerminal driver (SSI/SinCos/EnDat/BiSS/incremental),
# brings them to OP, and shows every channel's live counter + latch values.
add_tether_example(beckhoff_position_monitor
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff stepper terminal jog demo ---
# Finds the first POS-interface stepper (EL703x/EL704x), enables the
# output stage and jogs back and forth with a velocity setpoint.
add_tether_example(beckhoff_stepper_jog
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff compact drive jog demo ---
# Finds the first DRV-interface drive (EL7062/EL7411/EL72xx/ELM72xx),
# walks the CiA402 FSA to OperationEnabled and jogs in profile velocity.
add_tether_example(beckhoff_drive_move
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff PWM terminal sweep demo ---
# Finds the first PWM output terminal (EL2502/EL2535/EL2564) and sweeps
# the duty cycle on every channel as a staggered triangle wave.
add_tether_example(beckhoff_pwm_sweep
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff pulse-train jog demo ---
# Finds the first PTO terminal (EL2521/EL2522) and jogs the step/dir
# pulse output with a frequency setpoint + direction bits.
add_tether_example(beckhoff_pulse_jog
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff combined I/O toggle demo ---
# Auto-detects all packed-bit combined I/O devices (EL1852/EL1859,
# EP23xx, EL922x, EL203x w/ diagnostics, EK18xx couplers, ...) via the
# MultiCombinedIoTerminal driver and walks one lit output across the
# flat channel space — same device-tree TUI as the digital demos.
add_tether_example(beckhoff_combined_io_toggle
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff DC-motor jog demo ---
# Finds the first MOT-interface DC-motor terminal (EL7332/EL7342 +
# EJ/EP/ER/EPP variants), enables the power stages and jogs back and
# forth with a velocity setpoint.
add_tether_example(beckhoff_dcmotor_jog
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff power-measurement monitor ---
# Finds the first EL34xx terminal and prints per-phase V/I/P/S/Q/PF
# once per second.
add_tether_example(beckhoff_power_meter
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff serial/comm FIFO demo ---
# Finds the first EL6xxx byte-FIFO terminal (RS232/485, IO-Link, ...)
# and dumps the ctrl/status words + RX image while writing a rolling
# pattern into TX.
add_tether_example(beckhoff_serial_bridge
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff oversampling monitor ---
# Finds the first EL37xx/ELM3xxx oversampling terminal and summarizes
# each channel's per-cycle sample block once per second.
add_tether_example(beckhoff_oversampling_monitor
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# --- Beckhoff universal TUI explorer ---
# Discovers the whole chain, classifies every Beckhoff terminal by its
# product code, and displays a navigable device tree with the family
# name and the example binary to run for the selected device.
add_tether_example(beckhoff_tui
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)

# All Beckhoff examples get the interactive TUI when tether_terminal_ui exists.
if(TARGET tether_terminal_ui)
    foreach(_tui_ex beckhoff_output_toggle beckhoff_input_monitor
                    beckhoff_analog_input_monitor beckhoff_analog_output_sweep
                    beckhoff_position_monitor beckhoff_stepper_jog
                    beckhoff_drive_move beckhoff_pwm_sweep
                    beckhoff_pulse_jog beckhoff_combined_io_toggle
                    beckhoff_tui)
        target_link_libraries(${_tui_ex} PRIVATE tether_terminal_ui ${CURSES_LIBRARIES})
        target_include_directories(${_tui_ex} PRIVATE ${CURSES_INCLUDE_DIRS})
    endforeach()
else()
    message(STATUS "tether_terminal_ui unavailable; beckhoff_* examples are stream-only")
endif()
endif()

# --- Beckhoff TwinSAFE monitor (needs the FSoE stack too) ---
if(TETHER_DRIVER_BECKHOFF AND TETHER_BUILD_FSOE AND TARGET tether_device_drivers)
add_tether_example(beckhoff_safety_monitor
    LIBRARIES tether_ethercat_master tether_device_drivers tether_fsoe tether_ethercat_common tether_hal tether_common tether_examples_common)
if(TARGET tether_terminal_ui)
    target_link_libraries(beckhoff_safety_monitor PRIVATE tether_terminal_ui ${CURSES_LIBRARIES})
    target_include_directories(beckhoff_safety_monitor PRIVATE ${CURSES_INCLUDE_DIRS})
endif()
endif()

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
if(CURSES_FOUND AND TARGET tether_terminal_ui)
    add_tether_example(esc211_di_monitor
        LIBRARIES tether_ethercat_master tether_device_drivers tether_terminal_ui tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(esc211_di_monitor PRIVATE ${CURSES_INCLUDE_DIRS})
else()
    message(STATUS "ncurses/tether_terminal_ui not found; skipping esc211_di_monitor")
endif()
endif()

# --- pcapng interpreter example ---
add_tether_example(interpret_pcapng
    LIBRARIES tether_pcap tether_ethercat_common tether_common)

# --- WebSocket UI backends ---
# These examples use Drogon for the WebSocket transport (same as the
# tether_io_drogon_websocket example). They are only built when Drogon is
# available so the rest of the examples build without it.
if(Drogon_FOUND AND TARGET Drogon::Drogon)
    add_executable(tether_ui_backend
        tether_ui_backend.cpp
        TetherIOWebSocketController.cpp)
    list(APPEND TETHER_EXAMPLE_TARGETS tether_ui_backend)
    target_link_libraries(tether_ui_backend PRIVATE
        tether_io_protocol
        tether_gcode
        tether_ethercat_slave
        tether_common
        Drogon::Drogon)
    target_include_directories(tether_ui_backend PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/tether
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(tether_ui_backend PRIVATE cxx_std_20)

    add_executable(controlme_ui_backend
        controlme_ui_backend.cpp
        TetherIOWebSocketController.cpp)
    list(APPEND TETHER_EXAMPLE_TARGETS controlme_ui_backend)
    target_link_libraries(controlme_ui_backend PRIVATE
        tether_io_protocol
        tether_simulation
        tether_controls
        tether_common
        Drogon::Drogon)
    target_include_directories(controlme_ui_backend PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/tether
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(controlme_ui_backend PRIVATE cxx_std_20)
else()
    message(STATUS "Drogon not found; skipping WebSocket UI backend examples "
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
        LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common ${CURSES_LIBRARIES})
    target_include_directories(kinco_rp20_io PRIVATE ${CURSES_INCLUDE_DIRS})
    target_compile_definitions(kinco_rp20_io PRIVATE HAVE_NCURSES)
else()
    add_tether_example(kinco_rp20_io
        LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)
    message(STATUS "ncurses not found; kinco_rp20_io built without --interactive mode")
endif()

# --- Kinco RP20 module discovery demo ---
# Auto-discovers RP20 I/O modules across all slaves/slots and prints the list.
# No PDO/cyclic I/O — just discovery and listing.
add_tether_example(kinco_rp20_list_modules
    LIBRARIES tether_ethercat_master tether_device_drivers tether_ethercat_common tether_hal tether_common tether_examples_common)
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

    # HTTP example requires the optional HTTP component
    if(TETHER_ENABLE_KLIPPER_HTTP AND TARGET Tether::KlipperHttp)
        add_tether_example(klipper_http
            SOURCE klipper_http.cpp
            LIBRARIES Tether::KlipperHttp Tether::Klipper tether_common tether_simulation)
    else()
        message(STATUS "TETHER_ENABLE_KLIPPER_HTTP is OFF; skipping klipper_http example")
    endif()
endif()

add_custom_target(examples DEPENDS ${TETHER_EXAMPLE_TARGETS})
