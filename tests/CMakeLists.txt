cmake_minimum_required(VERSION 3.16)

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Include GoogleTest for gtest_discover_tests
include(GoogleTest)

# Tests are split into one executable per library/module (see create_module_test
# below). The core per-module executables (those that made up the former
# monolithic "tether_tests" binary) are always built. This option additionally
# enables optional module tests that are not part of the default suite because
# they rely on machine-specific fixtures (currently the ESI parser tests). It is
# kept so callers such as the ASAN/TSAN CI jobs can build those targets.
option(TETHER_BUILD_MODULE_TESTS "Build optional/extra per-module test executables" OFF)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pcap)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/motion)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gcode)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/fsoe)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hal)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/control)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/destabilizer)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/export)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/replanner)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/motion_planner)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/io)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/identification)

# Helper function to create module test executable
function(create_module_test MODULE_NAME SOURCES LIBS DEFINITIONS)
    list(FILTER SOURCES EXCLUDE REGEX ".*main\\.cpp$")
    list(FILTER SOURCES EXCLUDE REGEX ".*mocks_private/.*")
    
    add_executable(tether_${MODULE_NAME}_tests
        main.cpp
        ${SOURCES}
    )
    
    target_include_directories(tether_${MODULE_NAME}_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether/export
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether/motion_replanner
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether/motion_planner
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    
    target_link_libraries(tether_${MODULE_NAME}_tests PRIVATE
        ${LIBS}
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
    )
    
    target_compile_definitions(tether_${MODULE_NAME}_tests PRIVATE
        GCODE_STANDALONE_CAPI=1
        TETHER_ENABLE_ETHERCAT=1
        TETHER_COMPILE_MASTER=1
        TEST_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\"
        ${DEFINITIONS}
    )
    
    gtest_discover_tests(tether_${MODULE_NAME}_tests)
endfunction()

# Collect test sources by module
file(GLOB_RECURSE CONTROL_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp")
file(GLOB_RECURSE DC_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/dc/*.cpp")
file(GLOB_RECURSE DESTABILIZER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/destabilizer/*.cpp")
file(GLOB_RECURSE GCODE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/gcode/*.cpp")
file(GLOB_RECURSE HAL_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/hal/*.cpp")
file(GLOB_RECURSE PCAP_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/pcap/*.cpp")
file(GLOB_RECURSE IO_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp")
file(GLOB_RECURSE MOTION_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/motion/*.cpp")
file(GLOB_RECURSE MOTION_PLANNER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/*.cpp")
file(GLOB_RECURSE MOTION_REPLANNER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/*.cpp")
file(GLOB_RECURSE PROFILES_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/*.cpp")
file(GLOB_RECURSE SIMULATION_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/simulation/*.cpp")
file(GLOB_RECURSE SLAVE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/slave/*.cpp")
file(GLOB_RECURSE SII_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/sii/*.cpp")
file(GLOB_RECURSE ESI_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/esi/*.cpp")
file(GLOB_RECURSE MAILBOX_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mailbox/*.cpp")
file(GLOB_RECURSE SLAVE_EMULATION_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/*.cpp")
file(GLOB_RECURSE EXPORT_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/export/*.cpp")
file(GLOB_RECURSE REPLANNER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/replanner/*.cpp")
file(GLOB_RECURSE PLATFORM_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/platform/*.cpp")
file(GLOB_RECURSE MOCKS_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mocks/*.cpp")

# FSoE unit tests, excluding the integration test that has its own executable
# (tether_fsoe_integration_tests) below.
file(GLOB_RECURSE FSOE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fsoe/*.cpp")
list(FILTER FSOE_TEST_SOURCES EXCLUDE REGEX ".*test_synapticon_safe_motion\\.cpp$")

# Filter IO tests if IO protocol is disabled
if(NOT TETHER_BUILD_IO_PROTOCOL)
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_server_integration\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_session\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_session_integration\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_tcp_transport\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_threshold_filter\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_threshold_filter_extra\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_serial_transport\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_registry\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_registry_extra\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_datalogging\\.cpp$")
    list(FILTER IO_TEST_SOURCES EXCLUDE REGEX ".*test_io_datalogging_extra\\.cpp$")
endif()

# Create one test executable per library/module (only if sources exist).
# This replaces the former single monolithic "tether_tests" binary: each module
# now compiles and links independently, so a change in one area only rebuilds and
# relinks that module's test executable.
set(TETHER_FULL_TEST_LIBS
    tether_common
    tether_gcode
    tether_motion_control
    tether_motion_planner
    tether_controls
    tether_export
    tether_ethercat_common
    tether_ethercat_master
    tether_ethercat_slave
    tether_simulation
    tether_destabilizer
)

if(CONTROL_TEST_SOURCES)
    create_module_test(control
        "${CONTROL_TEST_SOURCES}"
        "tether_common;tether_hal;tether_gcode;tether_motion_control;tether_motion_planner;tether_controls;tether_export;tether_ethercat_common;tether_ethercat_master;tether_ethercat_slave;tether_simulation;tether_destabilizer"
        ""
    )
endif()

if(DC_TEST_SOURCES)
    create_module_test(dc
        "${DC_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_master"
        ""
    )
endif()

if(DESTABILIZER_TEST_SOURCES)
    create_module_test(destabilizer
        "${DESTABILIZER_TEST_SOURCES}"
        "tether_common;tether_destabilizer"
        ""
    )
endif()

if(GCODE_TEST_SOURCES)
    create_module_test(gcode
        "${GCODE_TEST_SOURCES}"
        "tether_common;tether_gcode;tether_export"
        ""
    )
endif()

if(HAL_TEST_SOURCES)
    create_module_test(hal
        "${HAL_TEST_SOURCES}"
        "tether_common;tether_pcap;tether_hal;tether_ethercat_common;tether_ethercat_master"
        ""
    )
endif()

if(PCAP_TEST_SOURCES)
    create_module_test(pcap
        "${PCAP_TEST_SOURCES}"
        "tether_common;tether_pcap;tether_ethercat_common"
        ""
    )
endif()

if(TETHER_BUILD_IO_PROTOCOL AND IO_TEST_SOURCES)
    create_module_test(io
        "${IO_TEST_SOURCES}"
        "tether_common;tether_io_protocol"
        ""
    )
endif()

if(MOTION_TEST_SOURCES)
    create_module_test(motion
        "${MOTION_TEST_SOURCES}"
        "tether_common;tether_motion_control;tether_ethercat_common"
        ""
    )
endif()

if(MOTION_PLANNER_TEST_SOURCES)
    create_module_test(motion_planner
        "${MOTION_PLANNER_TEST_SOURCES}"
        "tether_common;tether_motion_control;tether_motion_planner;tether_ethercat_common"
        ""
    )
endif()

if(MOTION_REPLANNER_TEST_SOURCES)
    create_module_test(motion_replanner
        "${MOTION_REPLANNER_TEST_SOURCES}"
        "tether_common;tether_motion_control;tether_motion_planner;tether_ethercat_common"
        ""
    )
endif()

if(PROFILES_TEST_SOURCES)
    create_module_test(profiles
        "${PROFILES_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master"
        ""
    )
endif()

if(SIMULATION_TEST_SOURCES)
    create_module_test(simulation
        "${SIMULATION_TEST_SOURCES}"
        "tether_common;tether_simulation"
        ""
    )
endif()

if(SLAVE_TEST_SOURCES)
    create_module_test(slave
        "${SLAVE_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave"
        ""
    )
endif()

if(SII_TEST_SOURCES)
    create_module_test(sii
        "${SII_TEST_SOURCES};${CMAKE_CURRENT_SOURCE_DIR}/sii/mocks_private/raw_transport_test_stubs.cpp"
        "tether_common;tether_ethercat_common;tether_ethercat_master"
        ""
    )
endif()

# The ESI parser tests load XML fixtures from hardcoded absolute paths that only
# exist on the original author's machine, so they are not part of the default
# suite (they were never in the old monolithic binary). They are only built when
# TETHER_BUILD_MODULE_TESTS is enabled (e.g. by the ASAN/TSAN CI jobs).
if(TETHER_BUILD_MODULE_TESTS AND ESI_TEST_SOURCES)
    create_module_test(esi
        "${ESI_TEST_SOURCES}"
        "tether_common;tether_ethercat_common"
        ""
    )
endif()

if(MAILBOX_TEST_SOURCES)
    create_module_test(mailbox
        "${MAILBOX_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_master;tether_ethercat_slave"
        ""
    )
endif()

if(SLAVE_EMULATION_TEST_SOURCES)
    create_module_test(slave_emulation
        "${SLAVE_EMULATION_TEST_SOURCES}"
        "tether_common;tether_pcap;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master"
        ""
    )
endif()

if(EXPORT_TEST_SOURCES)
    create_module_test(export
        "${EXPORT_TEST_SOURCES}"
        "tether_common;tether_export"
        ""
    )
endif()

if(REPLANNER_TEST_SOURCES)
    create_module_test(replanner
        "${REPLANNER_TEST_SOURCES}"
        "tether_common;tether_motion_planner"
        ""
    )
endif()

if(FSOE_TEST_SOURCES)
    create_module_test(fsoe
        "${FSOE_TEST_SOURCES}"
        "tether_common;tether_hal;tether_controls;tether_motion_control;tether_ethercat_common;tether_ethercat_master"
        ""
    )
endif()

if(PLATFORM_TEST_SOURCES)
    create_module_test(platform
        "${PLATFORM_TEST_SOURCES}"
        "tether_common;tether_hal"
        ""
    )
endif()

if(MOCKS_TEST_SOURCES)
    create_module_test(mocks
        "${MOCKS_TEST_SOURCES}"
        "${TETHER_FULL_TEST_LIBS}"
        ""
    )
endif()

add_executable(tether_fsoe_integration_tests
    fsoe/test_synapticon_safe_motion.cpp
)

target_include_directories(tether_fsoe_integration_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_fsoe_integration_tests
    tether_common
    tether_hal
    tether_controls
    tether_motion_control
    tether_ethercat_common
    tether_ethercat_master
    GTest::gtest
    GTest::gtest_main
)

target_compile_definitions(tether_fsoe_integration_tests PRIVATE
    TETHER_ENABLE_ETHERCAT=1
    TETHER_COMPILE_MASTER=1
)

gtest_discover_tests(tether_fsoe_integration_tests)

add_executable(tether_header_tests
    sii/zero_coverage_header_tests.cpp
    sii/zero_coverage_headers_more.cpp
)

target_include_directories(tether_header_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_header_tests
    GTest::gtest_main
)

target_compile_definitions(tether_header_tests PRIVATE
)

gtest_discover_tests(tether_header_tests)

add_executable(tether_ethercat_common_tests
    sii/foe_tests.cpp
    sii/fault_detection_tests.cpp
    sii/slave_emulator_tests.cpp
    ethercat/test_fault_detection.cpp
    ethercat/test_write_verify.cpp
    ethercat/test_sdo_async.cpp
    ethercat/test_foe.cpp
    ethercat/test_pdo_manager.cpp
    ethercat/test_logical_address_manager.cpp
    ethercat/test_dc.cpp
    ethercat/test_realtime_loop.cpp
    ethercat/test_jitter_monitor.cpp
    ethercat/test_fmmu_manager.cpp
    ethercat/test_retry_global.cpp
    mocks/ethercat_platform_stubs.cpp
    sii/mocks_private/eeprom_test_stubs.cpp
    ../src/ethercat/SlaveEmulator.cpp
    ../src/ethercat/raw/FoE.cpp
    ../src/ethercat/FaultDetection.cpp
    ../src/ethercat/WriteVerify.cpp
    ../src/ethercat/raw/sdo_async.cpp
    ../src/ethercat/raw/PDOManager.cpp
    ../src/ethercat/raw/LogicalAddressManager.cpp
    ../src/ethercat/DCClass.cpp
    ../src/ethercat/RealtimeLoop.cpp
    ../src/ethercat/RealtimeJitterMonitor.cpp
    ../src/ethercat/ConditionalPacketRouter.cpp
    ../src/ethercat/TransactionRouter.cpp
    ../src/ethercat/Retry.cpp
    ../src/fmmu/FMMUConfiguration.cpp
)

target_include_directories(tether_ethercat_common_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_ethercat_common_tests
    tether_ethercat_common
    GTest::gtest_main
    GTest::gmock
)

target_compile_definitions(tether_ethercat_common_tests PRIVATE
)

gtest_discover_tests(tether_ethercat_common_tests)

add_executable(tether_model_identification_tests
    identification/test_ModelIdentification.cpp
)

target_include_directories(tether_model_identification_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_model_identification_tests
    tether_common
    tether_controls
    tether_motion_control
    GTest::gtest_main
)

target_compile_definitions(tether_model_identification_tests PRIVATE
)

gtest_discover_tests(tether_model_identification_tests)

add_executable(tether_simulation_identification_tests
    identification/test_SimulationIdentificationIntegration.cpp
)

target_include_directories(tether_simulation_identification_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_simulation_identification_tests
    tether_common
    tether_controls
    tether_motion_control
    tether_simulation
    GTest::gtest_main
)

target_compile_definitions(tether_simulation_identification_tests PRIVATE
)

gtest_discover_tests(tether_simulation_identification_tests)

if(TETHER_ENABLE_COVERAGE)
    find_program(GCOVR_EXECUTABLE NAMES gcovr)
    if(GCOVR_EXECUTABLE)
        add_custom_target(coverage
            COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
            COMMAND ${GCOVR_EXECUTABLE} -r ${CMAKE_SOURCE_DIR} --xml -o coverage.xml --html --html-details -o coverage.html
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Running tests and generating coverage reports (coverage.xml, coverage.html)"
        )
    else()
        message(WARNING "gcovr not found; install 'gcovr' to enable 'coverage' target")
    endif()
endif()
