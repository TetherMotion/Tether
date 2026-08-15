# Component: tether_klipper
# Klipper protocol implementation for Tether motion kernel.
#
# Provides a clean-slate Klipper protocol implementation with two roles:
#   - klippy (host): connects to a device, downloads the data dictionary,
#     syncs the clock, and dispatches commands. Translates Tether MotionPlans
#     into queue_step sequences.
#   - device: serves the data dictionary, processes commands, and executes
#     motion via Tether's motion subsystem (passthrough or reconstruct+replan).
#
# Subcomponents:
#   - protocol: Crc16, Vlq, MessageBlock, Constants, ParameterFormat,
#     DataDictionary, IdentifyProtocol, CommandTable
#   - transport: IByteStreamTransport, Loopback, Pipe, TCP, CAN
#   - reliability: SequenceCounter, RtoEstimator, SerialQueue, AckMessage
#   - clock: McuClock, ClockSync
#   - objects: OidAllocator, Stepper, DigitalOut, PWMOut, AnalogIn, Endstop,
#     Trsync, Spi, I2c
#   - motion: MotionBlock, MotionTranslator, MotionReconstructor, MotionBlockSink
#   - klippy: KlippyHost
#   - device: KlipperDevice
#   - config: KlipperConfig, StandardCommands
#
# Dependencies: common, hal (for CAN transport), motion_planner (for MotionTranslator)
# Optional: TETHER_ENABLE_KLIPPER_CAN enables the CAN transport + LinuxCan HAL.

file(GLOB_RECURSE TETHER_KLIPPER_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/klipper/protocol/*.cpp"
    "${TETHER_ROOT}/src/klipper/transport/*.cpp"
    "${TETHER_ROOT}/src/klipper/reliability/*.cpp"
    "${TETHER_ROOT}/src/klipper/objects/*.cpp"
    "${TETHER_ROOT}/src/klipper/motion/*.cpp"
    "${TETHER_ROOT}/src/klipper/klippy/*.cpp"
    "${TETHER_ROOT}/src/klipper/device/*.cpp"
)

# CAN transport is gated behind TETHER_ENABLE_KLIPPER_CAN.
# When enabled, the Linux SocketCAN HAL implementation is also compiled.
if(NOT TETHER_ENABLE_KLIPPER_CAN)
    list(FILTER TETHER_KLIPPER_SOURCES EXCLUDE REGEX "CanTransport\\.cpp$")
    list(FILTER TETHER_KLIPPER_SOURCES EXCLUDE REGEX "LinuxCan\\.cpp")
endif()

# Filter to only existing files
set(TETHER_KLIPPER_SOURCES_FILTERED "")
foreach(src ${TETHER_KLIPPER_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_KLIPPER_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_klipper_shared SHARED ${TETHER_KLIPPER_SOURCES_FILTERED})
    list(APPEND _variants tether_klipper_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_klipper_static STATIC ${TETHER_KLIPPER_SOURCES_FILTERED})
    list(APPEND _variants tether_klipper_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${GLAZE_PATH}/include>
        PRIVATE
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/klipper>
    )

    target_compile_features(${_tgt} PRIVATE cxx_std_23)

    # Link against dependencies
    if(TARGET tether_common_static)
        target_link_libraries(${_tgt} PUBLIC tether_common_static)
    elseif(TARGET tether_common_shared)
        target_link_libraries(${_tgt} PUBLIC tether_common_shared)
    endif()
    # Kinematics (header-only INTERFACE library)
    if(TARGET tether_kinematics)
        target_link_libraries(${_tgt} PUBLIC tether_kinematics)
    endif()
    if(TARGET tether_motion_planner_static)
        target_link_libraries(${_tgt} PUBLIC tether_motion_planner_static)
    elseif(TARGET tether_motion_planner_shared)
        target_link_libraries(${_tgt} PUBLIC tether_motion_planner_shared)
    endif()

    # SPI driver (PosixSpiDriver) for ADXL345 auto-wiring
    if(TARGET tether_io_protocol_static)
        target_link_libraries(${_tgt} PUBLIC tether_io_protocol_static)
    elseif(TARGET tether_io_protocol_shared)
        target_link_libraries(${_tgt} PUBLIC tether_io_protocol_shared)
    endif()

    # Autotuning framework (for PID_CALIBRATE / M303 / SHAPER_CALIBRATE)
    # The klipper bridge delegates all autotuning to the Tether framework.
    if(TARGET tether_autotuning_static)
        target_link_libraries(${_tgt} PUBLIC tether_autotuning_static)
    elseif(TARGET tether_autotuning_shared)
        target_link_libraries(${_tgt} PUBLIC tether_autotuning_shared)
    endif()

    # Control algorithms (PIDController used by Heater::control())
    if(TARGET tether_controls_static)
        target_link_libraries(${_tgt} PUBLIC tether_controls_static)
    elseif(TARGET tether_controls_shared)
        target_link_libraries(${_tgt} PUBLIC tether_controls_shared)
    endif()

    # G-code shared types (Units, DistanceMode, Plane enums used by
    # PrinterMotionState for modal state tracking)
    if(TARGET tether_gcode_static)
        target_link_libraries(${_tgt} PUBLIC tether_gcode_static)
    elseif(TARGET tether_gcode_shared)
        target_link_libraries(${_tgt} PUBLIC tether_gcode_shared)
    endif()

    # Identification framework (for resonance testing / frequency analysis)
    if(TARGET tether_identification_static)
        target_link_libraries(${_tgt} PUBLIC tether_identification_static)
    elseif(TARGET tether_identification_shared)
        target_link_libraries(${_tgt} PUBLIC tether_identification_shared)
    endif()

    # CAN transport requires HAL and SocketCAN libs
    if(TETHER_ENABLE_KLIPPER_CAN)
        if(TARGET tether_hal_static)
            target_link_libraries(${_tgt} PUBLIC tether_hal_static)
        elseif(TARGET tether_hal_shared)
            target_link_libraries(${_tgt} PUBLIC tether_hal_shared)
        endif()
        target_link_libraries(${_tgt} PRIVATE pthread)
    endif()

    # TCP transport requires pthreads (for non-blocking connect)
    target_link_libraries(${_tgt} PRIVATE pthread)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
endforeach()

# Export for install/collection (matches other components)
set(TETHER_KLIPPER_TARGETS ${_variants})

# Alias targets for convenience (shared preferred, matches other components)
if(TARGET tether_klipper_shared)
    add_library(tether_klipper ALIAS tether_klipper_shared)
    add_library(tether::klipper ALIAS tether_klipper_shared)
    add_library(Tether::Klipper ALIAS tether_klipper_shared)
elseif(TARGET tether_klipper_static)
    add_library(tether_klipper ALIAS tether_klipper_static)
    add_library(tether::klipper ALIAS tether_klipper_static)
    add_library(Tether::Klipper ALIAS tether_klipper_static)
endif()
