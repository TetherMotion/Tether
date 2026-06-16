# Component: tether_motion_planner
# Motion planning, replanning, trajectory generation, and machine testing

set(TETHER_MOTION_PLANNER_SOURCES
    ${TETHER_ROOT}/src/replanner/MotionReplanner.cpp
    ${TETHER_ROOT}/src/replanner/GCodeProgram.cpp
    ${TETHER_ROOT}/src/replanner/GCodePatterns.cpp
    ${TETHER_ROOT}/src/replanner/GCodeExporter.cpp
    ${TETHER_ROOT}/src/replanner/MachineTesterCore.cpp
    ${TETHER_ROOT}/src/replanner/MachineTesterTrajectory.cpp
    ${TETHER_ROOT}/src/replanner/MachineTesterAnalysis.cpp
    ${TETHER_ROOT}/src/replanner/PerformanceHeatmap.cpp
    ${TETHER_ROOT}/src/replanner/SystemIdentifierCore.cpp
    ${TETHER_ROOT}/src/replanner/SystemIdentifierAnalysis.cpp
    ${TETHER_ROOT}/src/replanner/SystemIdentifierUtils.cpp
    ${TETHER_ROOT}/src/replanner/TestDataExporterBase.cpp
    ${TETHER_ROOT}/src/replanner/TestDataExporterTrajectory.cpp
    ${TETHER_ROOT}/src/replanner/TestDataExporterHeatmap.cpp
    ${TETHER_ROOT}/src/replanner/TestDataExporterTestResult.cpp
    ${TETHER_ROOT}/src/replanner/TestDataExporterBatch.cpp
)

# Note: motion_replanner/*.cpp are monolithic originals that have been
# split into replanner/*.cpp files above. Do not include them to avoid
# duplicate symbol definitions.

# Filter to only existing files
set(TETHER_MOTION_PLANNER_SOURCES_FILTERED "")
foreach(src ${TETHER_MOTION_PLANNER_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_MOTION_PLANNER_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Fallback if no files found
if(NOT TETHER_MOTION_PLANNER_SOURCES_FILTERED)
    file(GLOB_RECURSE TETHER_MOTION_PLANNER_SOURCES_FILTERED "${TETHER_ROOT}/src/replanner/*.cpp")
    # Exclude disabled/main files
    list(FILTER TETHER_MOTION_PLANNER_SOURCES_FILTERED EXCLUDE REGEX "main\\.cpp$")
endif()

# Create the motion planner library
add_library(tether_motion_planner STATIC ${TETHER_MOTION_PLANNER_SOURCES_FILTERED})
add_library(tether::motion_planner ALIAS tether_motion_planner)

target_include_directories(tether_motion_planner
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/motion_replanner>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
        $<INSTALL_INTERFACE:include/tether/motion_replanner>
    PRIVATE
        ${TETHER_ROOT}/src
        ${TETHER_ROOT}/src/replanner
)

target_link_libraries(tether_motion_planner
    PUBLIC tether_common tether_gcode tether_export
)

set_target_properties(tether_motion_planner PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_MOTION_PLANNER_LIBRARY tether_motion_planner)
