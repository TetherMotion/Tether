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

# Route all test executables into bin/tests to avoid cluttering the shared bin/
# directory used by examples and tools.
if(NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${TETHER_BIN_DIR}/tests")
else()
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${cfg} CFG)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFG} "${TETHER_BIN_DIR}/tests")
    endforeach()
endif()

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

    gtest_discover_tests(tether_${MODULE_NAME}_tests PROPERTIES TIMEOUT 30)
endfunction()

# Like create_module_test but with a custom per-test timeout (seconds).
function(create_module_test_timeout MODULE_NAME SOURCES LIBS DEFINITIONS TEST_TIMEOUT)
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

    gtest_discover_tests(tether_${MODULE_NAME}_tests PROPERTIES TIMEOUT ${TEST_TIMEOUT})
endfunction()

# Collect test sources by module
# Control test sources split by sub-area
set(CONTROL_TEST_LIBS "tether_common;tether_hal;tether_gcode;tether_motion_control;tether_motion_planner;tether_controls;tether_autotuning;tether_export;tether_ethercat_common;tether_ethercat_master;tether_ethercat_slave;tether_simulation;tether_destabilizer")
set(CONTROL_CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/ControlTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/FractionalPIDTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/MatrixUtilsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/test_controllers.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/test_scurve_ramper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/test_classical_tuning_factory.cpp"
)
set(CONTROL_COMPOSITE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/CompositeControllersTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/StateSpaceControllersTests.cpp"
)
set(CONTROL_ROBUST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/RobustControllersTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/LearningControllersTests.cpp"
)
set(CONTROL_QFT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/test_QFT_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/QFTTests.cpp"
)
set(CONTROL_AUTOTUNING_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/AdaptiveMethodsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/AutotuningFrameworkTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/ClassicalTuningMethodsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/HybridMethodsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/IndustrialAutotunersTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/LQRTuningTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/ModelBasedMethodsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/MuSynthesisTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/OptimizationAlgorithmsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/SlidingModeControlTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/test_IndustrialAutotuners_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/test_LQRTuning_coverage.cpp"
)
file(GLOB CONTROL_AUTOTUNING_METHODS_SOURCES_CLASSICAL "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/classical/*.cpp")
file(GLOB CONTROL_AUTOTUNING_METHODS_SOURCES_MODEL_BASED "${CMAKE_CURRENT_SOURCE_DIR}/control/autotuning/model_based/*.cpp")
set(CONTROL_AUTOTUNING_METHODS_SOURCES ${CONTROL_AUTOTUNING_METHODS_SOURCES_CLASSICAL} ${CONTROL_AUTOTUNING_METHODS_SOURCES_MODEL_BASED})
file(GLOB_RECURSE DC_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/dc/*.cpp")
file(GLOB_RECURSE DESTABILIZER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/destabilizer/*.cpp")
# GCode test sources split by sub-area
set(GCODE_LEXER_PARSER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/GCodeLexerTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/GCodeParserTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/GCodeOCodesTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/test_gcode_parser.cpp"
)
set(GCODE_INTERPOLATION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/InterpolationStrategyTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/MotionPrecomputeTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/PlotSamplingTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/ProjectionRegressionTests.cpp"
)
set(GCODE_TYPES_VARIABLES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/GCodeTypesTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/GCodeVariablesTests.cpp"
)
set(GCODE_EXPORT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/gcode/ExportTests.cpp"
)
file(GLOB_RECURSE HAL_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/hal/*.cpp")
file(GLOB_RECURSE PCAP_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/pcap/*.cpp")
# IO test sources split by sub-area
set(IO_PROTOCOL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_protocol.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_protocol_extra.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_feature_exchange.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_binary_struct.cpp"
)
set(IO_TRANSPORT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_session.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_session_integration.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_tcp_transport.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_serial_transport.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_pipe_transport.cpp"
)
set(IO_SERVICES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_registry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_registry_extra.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_server_integration.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_datalogging.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_datalogging_extra.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_threshold_filter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/test_io_threshold_filter_extra.cpp"
)
set(IO_TEST_SOURCES ${IO_PROTOCOL_SOURCES} ${IO_TRANSPORT_SOURCES} ${IO_SERVICES_SOURCES})
file(GLOB_RECURSE MOTION_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/motion/*.cpp")
# Motion planner test sources split by sub-area
set(MOTION_PLANNER_LIBS "tether_common;tether_motion_control;tether_motion_planner;tether_ethercat_common")
set(MOTION_PLANNER_CURVES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/MathTypesTest.cpp"
)
# NOTE: NURBSCurveTest.cpp and BezierCurveTest.cpp removed — they tested
# the old templated NURBSCurve/BezierCurve which have been replaced by
# tether::motion::NurbsCurve. The new geometry is covered by
# tether_motion_geometry_tests.
set(MOTION_PLANNER_BLEND_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/DebugSCurveTest.cpp"
)
# NOTE: ScoringBlendTests, MotionPlannerArcBlendTests, MixedTransitionTests,
# CornerLimitTests, NegativeToleranceTest, RegressionTests, VelocityProfilerTest,
# BlendVisualRegressionTests removed — they tested the old
# BlendConfig/PathBuilder/CornerBlending stack which has been replaced by
# tether::motion::PathBlender/BlendSolver. The new blend stack is covered by
# tether_motion_blend_new_tests.
set(MOTION_PLANNER_INTEGRATION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/test_GCodeAdapter_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/Phase5IntegrationTest.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/PHFastPathTest.cpp"
)
# NOTE: MotionPlannerIntegrationTest.cpp removed — it tested the old
# PiecewiseBezierPath/PathBuilder/CornerAnalyzer stack.
# Motion replanner test sources split by sub-area
set(MOTION_REPLANNER_LIBS "tether_common;tether_motion_control;tether_motion_planner;tether_ethercat_common;tether_identification")
set(MOTION_REPLANNER_CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_MotionReplanner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_MotionReplanner_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_TrajectorySampleConverter.cpp"
)
set(MOTION_REPLANNER_MACHINE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_MachineTester.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_MachineTesterTrajectories.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_GCodeGenerator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_GCodePatterns_coverage.cpp"
)
set(MOTION_REPLANNER_IDENTIFICATION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_SystemIdentifier.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_SystemIdentifierAnalysis.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_ModelIdentificationAdvanced.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_PerformanceHeatmap.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_replanner/test_TestDataExporter.cpp"
)
# Simulation test sources split by sub-area
set(SIMULATION_SYSTEMS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/simulation/test_simulation_systems.cpp"
)
set(SIMULATION_ENGINE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/simulation/test_simulation_engine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/simulation/test_simulation_metadata_and_initial_response.cpp"
)
file(GLOB_RECURSE SLAVE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/slave/*.cpp")
file(GLOB_RECURSE SII_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/sii/*.cpp")
file(GLOB_RECURSE ESI_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/esi/*.cpp")
file(GLOB_RECURSE MAILBOX_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mailbox/*.cpp")
# Slave emulation test sources split by sub-area
set(SLAVE_EMULATION_LIBS "tether_common;tether_pcap;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles")
set(SLAVE_EMULATION_CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_EtherCATSlaveEmulator_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_SlaveCore_coverage.cpp"
)
set(SLAVE_EMULATION_ETG5000_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_ETG5000_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_ETG5000_coverage2.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_ETG5000ModularDevice.cpp"
)
set(SLAVE_EMULATION_PROFILES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_slave_profiles_full.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/slave_emulation/test_PcapLogger_coverage.cpp"
)
file(GLOB_RECURSE EXPORT_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/export/*.cpp")
file(GLOB_RECURSE REPLANNER_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/replanner/*.cpp")
file(GLOB_RECURSE PLATFORM_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/platform/*.cpp")
file(GLOB_RECURSE MOCKS_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/mocks/*.cpp")

# Per-profile test sources (split from monolithic PROFILES_TEST_SOURCES)
file(GLOB CIA301_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia301/*.cpp")
file(GLOB CIA401_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia401/*.cpp")
# CiA402 test sources split by sub-area
set(CIA402_PROFILE_LIBS "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles;tether_motion_control;tether_drives")
set(CIA402_STATE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_CiA402StateMachine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_CiA402StateMachine_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_cia402_state_machine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_set_preop_retry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_set_preop_fault_diagnose.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_set_preop_fallback.cpp"
)
set(CIA402_HOMING_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_HomingStateMachine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_HomingHandler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_HomingHandler_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_HomingMethods_coverage.cpp"
)
set(CIA402_MOTION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_MotionController_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_MotionProfile.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_MultiAxisPath.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_ElectronicGearing.cpp"
)
set(CIA402_MOTOR_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_AdvancedMotorModel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_motor_model_full.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_PIDController.cpp"
)
set(CIA402_COVERAGE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_CiA402_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_CiA402Axis_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_EtherCATBackend_coverage.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_CiA402DrivePDO.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_60xxParameters.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_cia402_pdo_fallback.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_cia402_protocol.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia402/test_as715n_pdo_utils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_drives_as715n.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_as715n_errc11_integration.cpp"
)
file(GLOB CIA404_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia404/*.cpp")
file(GLOB CIA405_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia405/*.cpp")
list(APPEND CIA405_TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_slave_profiles_extra.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_slave_profile_no_global_state.cpp"
)
file(GLOB CIA406_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia406/*.cpp")
file(GLOB CIA408_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia408/*.cpp")
file(GLOB CIA410_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia410/*.cpp")
file(GLOB CIA417_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia417/*.cpp")
list(APPEND CIA417_TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/test_CiA430_CiA417.cpp"
)
file(GLOB CIA430_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/profiles/cia430/*.cpp")

# FSoE unit tests, excluding the integration test that has its own executable
# (tether_fsoe_integration_tests) below.
file(GLOB_RECURSE FSOE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/fsoe/*.cpp")
list(FILTER FSOE_TEST_SOURCES EXCLUDE REGEX ".*test_synapticon_safe_motion\\.cpp$")

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
    tether_autotuning
    tether_identification
    tether_export
    tether_ethercat_common
    tether_ethercat_master
    tether_cia_profiles
    tether_drives
    tether_fsoe
    tether_ethercat_slave
    tether_simulation
    tether_destabilizer
)

create_module_test(control_core "${CONTROL_CORE_SOURCES}" "${CONTROL_TEST_LIBS}" "")
create_module_test(control_composite "${CONTROL_COMPOSITE_SOURCES}" "${CONTROL_TEST_LIBS}" "")
create_module_test(control_robust "${CONTROL_ROBUST_SOURCES}" "${CONTROL_TEST_LIBS}" "")
create_module_test(control_qft "${CONTROL_QFT_SOURCES}" "${CONTROL_TEST_LIBS}" "")
create_module_test(control_autotuning "${CONTROL_AUTOTUNING_SOURCES}" "${CONTROL_TEST_LIBS}" "")
if(CONTROL_AUTOTUNING_METHODS_SOURCES)
    create_module_test(control_autotuning_methods "${CONTROL_AUTOTUNING_METHODS_SOURCES}" "${CONTROL_TEST_LIBS}" "")
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

create_module_test(gcode_lexer_parser "${GCODE_LEXER_PARSER_SOURCES}" "tether_common;tether_gcode;tether_export" "")
create_module_test(gcode_interpolation "${GCODE_INTERPOLATION_SOURCES}" "tether_common;tether_gcode;tether_export" "")
create_module_test(gcode_types_variables "${GCODE_TYPES_VARIABLES_SOURCES}" "tether_common;tether_gcode;tether_export" "")
create_module_test(gcode_export "${GCODE_EXPORT_SOURCES}" "tether_common;tether_gcode;tether_export" "")

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

if(TETHER_BUILD_IO_PROTOCOL)
    create_module_test(io_protocol "${IO_PROTOCOL_SOURCES}" "tether_common;tether_io_protocol" "")
    create_module_test(io_transport "${IO_TRANSPORT_SOURCES}" "tether_common;tether_io_protocol" "")
    create_module_test(io_services "${IO_SERVICES_SOURCES}" "tether_common;tether_io_protocol" "")
endif()

if(MOTION_TEST_SOURCES)
    create_module_test(motion
        "${MOTION_TEST_SOURCES}"
        "tether_common;tether_motion_control;tether_ethercat_common"
        ""
    )
endif()

create_module_test(motion_planner_curves "${MOTION_PLANNER_CURVES_SOURCES}" "${MOTION_PLANNER_LIBS}" "")
create_module_test(motion_planner_blend "${MOTION_PLANNER_BLEND_SOURCES}" "${MOTION_PLANNER_LIBS}" "")
# PH fast-path integration tests need a longer timeout (PH blend certification is expensive).
create_module_test_timeout(motion_planner_integration "${MOTION_PLANNER_INTEGRATION_SOURCES}" "${MOTION_PLANNER_LIBS}" "" 120)

# New geometry core (namespace tether::motion), Phase 1 of the motion-planner
# rewrite. Same wiring as create_module_test, plus a TEST_PREFIX so that
# `ctest -R motion_geometry` selects exactly this suite.
set(MOTION_GEOMETRY_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/VectorTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/NurbsCurveTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/BernsteinTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/PointCurveDistanceTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/PiecewiseNurbsPathTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/geometry/CertifiedCurvatureSamplerTests.cpp"
)
add_executable(tether_motion_geometry_tests
    main.cpp
    ${MOTION_GEOMETRY_SOURCES}
)
target_include_directories(tether_motion_geometry_tests PRIVATE
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
target_link_libraries(tether_motion_geometry_tests PRIVATE
    tether_common
    tether_motion_planner
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    GTest::gmock_main
)
target_compile_definitions(tether_motion_geometry_tests PRIVATE
    GCODE_STANDALONE_CAPI=1
    TETHER_ENABLE_ETHERCAT=1
    TETHER_COMPILE_MASTER=1
    TEST_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\"
)
# 120 s: the fuzz tests (1e5-sample brute force) and the 1e6-piece laziness
# test exceed the default 30 s under Debug/ASAN.
gtest_discover_tests(tether_motion_geometry_tests
    TEST_PREFIX motion_geometry.
    PROPERTIES TIMEOUT 120
)

# New blend layer (namespace tether::motion), Phase 2 of the motion-planner
# rewrite. `ctest -R motion_blend_new` selects exactly this suite.
set(MOTION_BLEND_NEW_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/CornerAnalysisTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/BoundaryConditionsTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/BlendCurveBuilderTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/DeviationCertifierTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/PHQuinticBlendTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/ContinuityPropertyTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/BlendSolverTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/PathBlenderTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/ToleranceFuzzTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion_planner/blend/SegmentConverterTests.cpp"
)
add_executable(tether_motion_blend_new_tests
    main.cpp
    ${MOTION_BLEND_NEW_SOURCES}
)
target_include_directories(tether_motion_blend_new_tests PRIVATE
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
target_link_libraries(tether_motion_blend_new_tests PRIVATE
    tether_common
    tether_motion_planner
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    GTest::gmock_main
)
target_compile_definitions(tether_motion_blend_new_tests PRIVATE
    GCODE_STANDALONE_CAPI=1
    TETHER_ENABLE_ETHERCAT=1
    TETHER_COMPILE_MASTER=1
    TEST_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\"
)
gtest_discover_tests(tether_motion_blend_new_tests
    TEST_PREFIX motion_blend_new.
    PROPERTIES TIMEOUT 300
)

create_module_test(motion_replanner_core "${MOTION_REPLANNER_CORE_SOURCES}" "${MOTION_REPLANNER_LIBS}" "")
create_module_test(motion_replanner_machine "${MOTION_REPLANNER_MACHINE_SOURCES}" "${MOTION_REPLANNER_LIBS}" "")
create_module_test(motion_replanner_identification "${MOTION_REPLANNER_IDENTIFICATION_SOURCES}" "${MOTION_REPLANNER_LIBS}" "")

if(CIA301_TEST_SOURCES)
    create_module_test(profiles_cia301
        "${CIA301_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA401_TEST_SOURCES)
    create_module_test(profiles_cia401
        "${CIA401_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

create_module_test(profiles_cia402_state "${CIA402_STATE_SOURCES}" "${CIA402_PROFILE_LIBS}" "")
create_module_test(profiles_cia402_homing "${CIA402_HOMING_SOURCES}" "${CIA402_PROFILE_LIBS}" "")
create_module_test(profiles_cia402_motion "${CIA402_MOTION_SOURCES}" "${CIA402_PROFILE_LIBS}" "")
create_module_test(profiles_cia402_motor "${CIA402_MOTOR_SOURCES}" "${CIA402_PROFILE_LIBS}" "")
create_module_test(profiles_cia402_coverage "${CIA402_COVERAGE_SOURCES}" "${CIA402_PROFILE_LIBS}" "")

if(CIA404_TEST_SOURCES)
    create_module_test(profiles_cia404
        "${CIA404_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA405_TEST_SOURCES)
    create_module_test(profiles_cia405
        "${CIA405_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA406_TEST_SOURCES)
    create_module_test(profiles_cia406
        "${CIA406_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA408_TEST_SOURCES)
    create_module_test(profiles_cia408
        "${CIA408_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA410_TEST_SOURCES)
    create_module_test(profiles_cia410
        "${CIA410_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA417_TEST_SOURCES)
    create_module_test(profiles_cia417
        "${CIA417_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

if(CIA430_TEST_SOURCES)
    create_module_test(profiles_cia430
        "${CIA430_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_slave;tether_ethercat_master;tether_cia_profiles"
        ""
    )
endif()

create_module_test(simulation_systems "${SIMULATION_SYSTEMS_SOURCES}" "tether_common;tether_simulation" "")
# The simulation_engine module includes AllSystemsStayNonzeroThroughHalfSecondWithNonzeroInitialCondition
# which runs 70 dynamical systems × 100 RK4 steps each. Under TSAN/ASAN the per-step
# overhead is 5-10×, and with --parallel $(nproc) on a 4-core CI runner the
# wall-clock time can exceed the default 30 s timeout.
create_module_test_timeout(simulation_engine "${SIMULATION_ENGINE_SOURCES}" "tether_common;tether_simulation" "" 120)

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
# They also require the tether_esi library, which is only created when
# TETHER_BUILD_EXTRACT_ESI is ON, so guard on the target being present.
if(TETHER_BUILD_MODULE_TESTS AND ESI_TEST_SOURCES AND TARGET tether_esi)
    create_module_test(esi
        "${ESI_TEST_SOURCES}"
        "tether_common;tether_esi"
        ""
    )
endif()

if(MAILBOX_TEST_SOURCES)
    create_module_test(mailbox
        "${MAILBOX_TEST_SOURCES}"
        "tether_common;tether_ethercat_common;tether_ethercat_master;tether_ethercat_slave;tether_cia_profiles"
        ""
    )
endif()

create_module_test(slave_emulation_core "${SLAVE_EMULATION_CORE_SOURCES}" "${SLAVE_EMULATION_LIBS}" "")
create_module_test(slave_emulation_etg5000 "${SLAVE_EMULATION_ETG5000_SOURCES}" "${SLAVE_EMULATION_LIBS}" "")
create_module_test(slave_emulation_profiles "${SLAVE_EMULATION_PROFILES_SOURCES}" "${SLAVE_EMULATION_LIBS}" "")

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
        "tether_common;tether_hal;tether_controls;tether_motion_control;tether_ethercat_common;tether_ethercat_master;tether_cia_profiles;tether_fsoe"
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
    tether_cia_profiles
    tether_fsoe
    GTest::gtest
    GTest::gtest_main
)

target_compile_definitions(tether_fsoe_integration_tests PRIVATE
    TETHER_ENABLE_ETHERCAT=1
    TETHER_COMPILE_MASTER=1
)

gtest_discover_tests(tether_fsoe_integration_tests PROPERTIES TIMEOUT 30)

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

gtest_discover_tests(tether_header_tests PROPERTIES TIMEOUT 30)

# Helper to create ethercat test executables with common settings
function(create_ethercat_test NAME)
    cmake_parse_arguments(ET "" "" "SOURCES;LIBS;DEFS" ${ARGN})
    add_executable(tether_ethercat_${NAME}_tests
        ${ET_SOURCES}
    )
    target_include_directories(tether_ethercat_${NAME}_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    target_link_libraries(tether_ethercat_${NAME}_tests
        tether_ethercat_common
        ${ET_LIBS}
        GTest::gtest_main
        GTest::gmock
    )
    target_compile_definitions(tether_ethercat_${NAME}_tests PRIVATE
        ${ET_DEFS}
    )
    gtest_discover_tests(tether_ethercat_${NAME}_tests PROPERTIES TIMEOUT 30)
endfunction()

# Common stubs and source files compiled into each ethercat test exe
set(ETHERCAT_COMMON_STUBS
    mocks/ethercat_platform_stubs.cpp
    sii/mocks_private/eeprom_test_stubs.cpp
)
set(ETHERCAT_COMMON_SRC
    ../src/ethercat/SlaveEmulator.cpp
    ../src/ethercat/raw/FoE.cpp
    ../src/ethercat/FaultDetection.cpp
    ../src/ethercat/WriteVerify.cpp
    ../src/ethercat/CoEManager.cpp
    ../src/ethercat/raw/SDOErrorDecoder.cpp
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

# PDO/FMMU tests
create_ethercat_test(pdo
    SOURCES
        ethercat/test_pdo_manager.cpp
        ethercat/test_pdo_modes.cpp
        ethercat/test_logical_address_manager.cpp
        ethercat/test_fmmu_manager.cpp
        ethercat/test_pdo_api.cpp
        ethercat/test_pdo_to_string.cpp
        ethercat/test_sync_manager_validation.cpp
        ethercat/test_custom_pdo_mapping.cpp
        ${ETHERCAT_COMMON_STUBS}
        ../src/ethercat/raw/PDOManager.cpp
        ../src/ethercat/raw/LogicalAddressManager.cpp
        ../src/ethercat/CoEManager.cpp
        ../src/ethercat/raw/SDOErrorDecoder.cpp
        ../src/ethercat/FaultDetection.cpp
        ../src/ethercat/DebugGate.cpp
        ../src/ethercat/utils/PDO.cpp
        ../src/ethercat/SyncManagerValidation.cpp
        ../src/fmmu/FMMUConfiguration.cpp
)

# Protocol tests (FoE, SDO, fault detection, write-verify, retry)
create_ethercat_test(protocol
    SOURCES
        sii/foe_tests.cpp
        sii/fault_detection_tests.cpp
        sii/slave_emulator_tests.cpp
        ethercat/test_fault_detection.cpp
        ethercat/test_fault_detector_coverage.cpp
        ethercat/test_fault_names_coverage.cpp
        ethercat/test_write_verify.cpp
        ethercat/test_sdo_async.cpp
        ethercat/test_foe.cpp
        ethercat/test_retry_global.cpp
        ethercat/test_ethercat_retry.cpp
        ethercat/test_ethercat_retry_smoke.cpp
        ethercat/test_host_stubs.cpp
        ethercat/test_slave_status_poller.cpp
        ${ETHERCAT_COMMON_STUBS}
        ../src/ethercat/SlaveEmulator.cpp
        ../src/ethercat/raw/FoE.cpp
        ../src/ethercat/FaultDetection.cpp
        ../src/ethercat/SlaveStatusPoller.cpp
        ../src/ethercat/WriteVerify.cpp
        ../src/ethercat/CoEManager.cpp
        ../src/ethercat/raw/SDOErrorDecoder.cpp
        ../src/ethercat/raw/PDOManager.cpp
        ../src/ethercat/raw/LogicalAddressManager.cpp
        ../src/ethercat/DebugGate.cpp
        ../src/ethercat/Retry.cpp
        ../src/ethercat/host_stubs.cpp
        ../src/ethercat/ConditionalPacketRouter.cpp
        ../src/ethercat/TransactionRouter.cpp
)

# Timing tests (DC, realtime loop, jitter)
create_ethercat_test(timing
    SOURCES
        ethercat/test_dc.cpp
        ethercat/test_realtime_loop.cpp
        ethercat/test_jitter_monitor.cpp
        ethercat/test_dc_configuration_validation.cpp
        ethercat/test_dc_consistency_coverage.cpp
        ${ETHERCAT_COMMON_STUBS}
        ../src/ethercat/DCClass.cpp
        ../src/ethercat/RealtimeLoop.cpp
        ../src/ethercat/RealtimeJitterMonitor.cpp
    LIBS tether_ethercat_master
    DEFS TETHER_COMPILE_MASTER=1
)

# Master/coverage tests (vlan router, master frame, sync manager)
create_ethercat_test(coverage
    SOURCES
        ethercat/test_vlan_router.cpp
        ethercat/test_master_frame_coverage.cpp
        ethercat/test_master_udp_encapsulation.cpp
        ethercat/test_sync_manager.cpp
        ethercat/test_set_preop_fault_diagnose.cpp
        ethercat/test_set_preop_retry.cpp
        ethercat/test_mailbox_utils.cpp
        ethercat/test_voe_eoe.cpp
        ethercat/test_dc_manager_coverage.cpp
        ethercat/test_dc_utils.cpp
        ${ETHERCAT_COMMON_STUBS}
        ../src/ethercat/SlaveEmulator.cpp
        ../src/ethercat/ConditionalPacketRouter.cpp
        ../src/ethercat/TransactionRouter.cpp
    LIBS tether_ethercat_master;tether_cia_profiles
    DEFS TETHER_COMPILE_MASTER=1
)

# EtherCAT integration tests (master ↔ slave communication via paired
# loopback interfaces).  Separated from unit tests because they exercise
# full end-to-end communication paths and may require more time/resources.
add_executable(tether_ethercat_integration_tests
    ethercat/test_integration_paired.cpp
    ${ETHERCAT_COMMON_STUBS}
    ../src/ethercat/SlaveEmulator.cpp
    ../src/ethercat/ConditionalPacketRouter.cpp
    ../src/ethercat/TransactionRouter.cpp
)

target_include_directories(tether_ethercat_integration_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_ethercat_integration_tests
    tether_ethercat_common
    tether_ethercat_master
    GTest::gtest_main
    GTest::gmock
)

target_compile_definitions(tether_ethercat_integration_tests PRIVATE
    TETHER_COMPILE_MASTER=1
)

gtest_discover_tests(tether_ethercat_integration_tests PROPERTIES TIMEOUT 30)

# EtherCAT master + slave state-machine tests (needs full Master/Slave linkage)
add_executable(tether_ethercat_master_tests
    ethercat/test_ethercat_slave.cpp
)

target_include_directories(tether_ethercat_master_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tether
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(tether_ethercat_master_tests
    tether_ethercat_common
    tether_ethercat_master
    GTest::gtest_main
    GTest::gmock
)

target_compile_definitions(tether_ethercat_master_tests PRIVATE
    TETHER_COMPILE_MASTER=1
)

gtest_discover_tests(tether_ethercat_master_tests PROPERTIES TIMEOUT 30)

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
    tether_identification
    GTest::gtest_main
)

target_compile_definitions(tether_model_identification_tests PRIVATE
)

gtest_discover_tests(tether_model_identification_tests PROPERTIES TIMEOUT 30)

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
    tether_identification
    tether_simulation
    GTest::gtest_main
)

target_compile_definitions(tether_simulation_identification_tests PRIVATE
)

gtest_discover_tests(tether_simulation_identification_tests PROPERTIES TIMEOUT 30)

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
