# CMakeLists.txt for Motion Planner Tests
# Builds unit tests for the advanced motion planner framework

cmake_minimum_required(VERSION 3.14)

# Add test executables
set(MOTION_PLANNER_TEST_SOURCES
    motion_planner/MathTypesTest.cpp
    motion_planner/BezierCurveTest.cpp
    motion_planner/NURBSCurveTest.cpp
    motion_planner/MotionPlannerIntegrationTest.cpp
    motion_planner/RegressionTests.cpp
    motion_planner/CornerLimitTests.cpp
)

# Include the motion planner headers
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create test executable for each test file
foreach(test_source ${MOTION_PLANNER_TEST_SOURCES})
    # Get test name from source file name
    get_filename_component(test_name ${test_source} NAME_WE)
    
    # Create test executable
    add_executable(${test_name} ${test_source})
    
    # Link with GoogleTest
    target_link_libraries(${test_name}
        PRIVATE
        GTest::gtest
        GTest::gtest_main
    )
    
    # Add compile options
    target_compile_features(${test_name} PRIVATE cxx_std_20)
    target_compile_options(${test_name} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    
    # Add include directories
    target_include_directories(${test_name}
        PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )
    
    # Add to CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 30)
    
    # Add to test list for combined target
    list(APPEND MOTION_PLANNER_TESTS ${test_name})
endforeach()

# Create a combined test target
add_custom_target(motion_planner_tests
    DEPENDS ${MOTION_PLANNER_TESTS}
    COMMENT "Building all motion planner tests"
)

# If coverage is enabled, add coverage target
if(TETHER_ENABLE_COVERAGE)
    foreach(test_name ${MOTION_PLANNER_TESTS})
        target_compile_options(${test_name} PRIVATE --coverage)
        target_link_options(${test_name} PRIVATE --coverage)
    endforeach()
endif()
