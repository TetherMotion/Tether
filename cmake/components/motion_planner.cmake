# Component: tether_motion_planner
# Motion planning, replanning, trajectory generation, and machine testing

set(TETHER_MOTION_PLANNER_SOURCES
    ${TETHER_ROOT}/src/motion_planner/blend/CornerAnalysis.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/BoundaryConditions.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/BlendCurveBuilder.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/BlendSpec.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/DeviationCertifier.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/PHQuinticBlendBuilder.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/BlendSolver.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/PathBlender.cpp
    ${TETHER_ROOT}/src/motion_planner/blend/SegmentConverter.cpp
    ${TETHER_ROOT}/src/replanner/MotionReplanner.cpp
    ${TETHER_ROOT}/src/replanner/TrajectorySampleConverter.cpp
    ${TETHER_ROOT}/src/replanner/CertifiedContourError.cpp
    ${TETHER_ROOT}/src/replanner/CertifiedCornerDetection.cpp
    ${TETHER_ROOT}/src/replanner/CurvatureAwareLimiter.cpp
    ${TETHER_ROOT}/src/replanner/CertifiedSuggestionSolver.cpp
    ${TETHER_ROOT}/src/replanner/OnlineReblender.cpp
    ${TETHER_ROOT}/src/replanner/ProfileReplanner.cpp
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
    ${TETHER_ROOT}/src/replanner/PathEvaluator.cpp
    ${TETHER_ROOT}/src/replanner/PathQualityGrader.cpp
    ${TETHER_ROOT}/src/replanner/PathRelativeFFT.cpp
    ${TETHER_ROOT}/src/replanner/SvgExporter.cpp
    ${TETHER_ROOT}/src/replanner/SvgCanvas.cpp
    ${TETHER_ROOT}/src/replanner/DependenceAnalyzer.cpp
    ${TETHER_ROOT}/src/replanner/BandwidthSelector.cpp
    ${TETHER_ROOT}/src/replanner/KdeStatistics.cpp
    ${TETHER_ROOT}/src/replanner/KdeGridEvaluator.cpp
    ${TETHER_ROOT}/src/replanner/KdeDerivativeAnalyzer.cpp
)

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

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_planner_shared SHARED ${TETHER_MOTION_PLANNER_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_planner_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_planner_static STATIC ${TETHER_MOTION_PLANNER_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_planner_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/motion_replanner>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/motion_replanner>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt}
        PUBLIC
            tether_common
            tether_gcode
            tether_motion_geometry
        PRIVATE
            tether_export
    )

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_planner ALIAS tether_motion_planner_shared)
    add_library(tether::motion_planner ALIAS tether_motion_planner_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_planner ALIAS tether_motion_planner_static)
    add_library(tether::motion_planner ALIAS tether_motion_planner_static)
endif()

set(TETHER_MOTION_PLANNER_LIBRARY tether_motion_planner)
set(TETHER_MOTION_PLANNER_TARGETS ${_variants})

# CLI tool for motion replanner analysis
if(TARGET tether_motion_planner)
    add_executable(motion_replanner_cli ${TETHER_ROOT}/src/replanner/motion_replanner_main.cpp)
    target_link_libraries(motion_replanner_cli PRIVATE tether_motion_planner)
    target_compile_definitions(motion_replanner_cli PRIVATE
        GCODE_STANDALONE_CAPI=1
    )
endif()
