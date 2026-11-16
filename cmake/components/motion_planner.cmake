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
    ${TETHER_ROOT}/src/motion_planner/blend/OutsideCircleBlender.cpp
    ${TETHER_ROOT}/src/motion_planner/geometry/PlanningSegmentConverter.cpp
    ${TETHER_ROOT}/src/livetune/OnlineTuner.cpp
    ${TETHER_ROOT}/src/livetune/TrajectorySampleConverter.cpp
    ${TETHER_ROOT}/src/livetune/CertifiedContourError.cpp
    ${TETHER_ROOT}/src/livetune/CertifiedCornerDetection.cpp
    ${TETHER_ROOT}/src/livetune/CurvatureAwareLimiter.cpp
    ${TETHER_ROOT}/src/livetune/CertifiedSuggestionSolver.cpp
    ${TETHER_ROOT}/src/livetune/OnlineReblender.cpp
    ${TETHER_ROOT}/src/livetune/ProfileReplanner.cpp
    ${TETHER_ROOT}/src/livetune/GCodeProgram.cpp
    ${TETHER_ROOT}/src/livetune/GCodePatterns.cpp
    ${TETHER_ROOT}/src/livetune/GCodeExporter.cpp
    ${TETHER_ROOT}/src/livetune/MachineTesterCore.cpp
    ${TETHER_ROOT}/src/livetune/MachineTesterTrajectory.cpp
    ${TETHER_ROOT}/src/livetune/MachineTesterAnalysis.cpp
    ${TETHER_ROOT}/src/livetune/PerformanceHeatmap.cpp
    ${TETHER_ROOT}/src/livetune/SystemIdentifierCore.cpp
    ${TETHER_ROOT}/src/livetune/SystemIdentifierAnalysis.cpp
    ${TETHER_ROOT}/src/livetune/SystemIdentifierUtils.cpp
    ${TETHER_ROOT}/src/livetune/TestDataExporterBase.cpp
    ${TETHER_ROOT}/src/livetune/TestDataExporterTrajectory.cpp
    ${TETHER_ROOT}/src/livetune/TestDataExporterHeatmap.cpp
    ${TETHER_ROOT}/src/livetune/TestDataExporterTestResult.cpp
    ${TETHER_ROOT}/src/livetune/TestDataExporterBatch.cpp
    ${TETHER_ROOT}/src/livetune/PathEvaluator.cpp
    ${TETHER_ROOT}/src/livetune/PathQualityGrader.cpp
    ${TETHER_ROOT}/src/livetune/PathRelativeFFT.cpp
    ${TETHER_ROOT}/src/livetune/FFTProcessor.cpp
    ${TETHER_ROOT}/src/livetune/SvgExporter.cpp
    ${TETHER_ROOT}/src/livetune/SvgCanvas.cpp
    ${TETHER_ROOT}/src/livetune/KdeSvgRenderer.cpp
    ${TETHER_ROOT}/src/livetune/DependenceAnalyzer.cpp
    ${TETHER_ROOT}/src/livetune/BandwidthSelector.cpp
    ${TETHER_ROOT}/src/livetune/KdeStatistics.cpp
    ${TETHER_ROOT}/src/livetune/KdeGridEvaluator.cpp
    ${TETHER_ROOT}/src/livetune/KdeDerivativeAnalyzer.cpp
    ${TETHER_ROOT}/src/motion_planner/profile_renurbs/ProfileSplineFitter.cpp
    ${TETHER_ROOT}/src/motion_planner/profile_renurbs/ReNURBSProfileBuilder.cpp
    ${TETHER_ROOT}/src/motion_planner/profile_renurbs/ProfileConstraintCertifier.cpp
    ${TETHER_ROOT}/src/motion_planner/profile_renurbs/GenericReNURBSBuilder.cpp
    ${TETHER_ROOT}/src/motion_planner/profile_renurbs/GenericReNURBSCertifier.cpp
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
    file(GLOB_RECURSE TETHER_MOTION_PLANNER_SOURCES_FILTERED "${TETHER_ROOT}/src/livetune/*.cpp")
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
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/livetune>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/livetune>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt}
        PUBLIC
            tether_common
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

# CLI tool for motion livetune analysis
if(TARGET tether_motion_planner)
    add_executable(livetune_cli ${TETHER_ROOT}/src/livetune/livetune_main.cpp)
    target_link_libraries(livetune_cli PRIVATE tether_motion_planner)
    target_compile_definitions(livetune_cli PRIVATE
        GCODE_STANDALONE_CAPI=1
    )
endif()
