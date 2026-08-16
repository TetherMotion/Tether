# Component: tether_analysis
# G-code / motion path analysis components ported from WebGCodeViewer.
# 10 analysis modules + shared parsing utilities + IO protocol exposer.

set(TETHER_ANALYSIS_SOURCES
    ${TETHER_ROOT}/src/analysis/MachineLimitChecker.cpp
    ${TETHER_ROOT}/src/analysis/CurvatureAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/ArcAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/ModalStateAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/PathTopologyDetector.cpp
    ${TETHER_ROOT}/src/analysis/ToolpathEfficiencyAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/RetractionAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/AccelerationProfileAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/CoordinateSystemAnalyzer.cpp
    ${TETHER_ROOT}/src/analysis/PathContinuityChecker.cpp
)

# Filter to only existing files
set(TETHER_ANALYSIS_SOURCES_FILTERED "")
foreach(src ${TETHER_ANALYSIS_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_ANALYSIS_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Get all source files from analysis directory as fallback
if(NOT TETHER_ANALYSIS_SOURCES_FILTERED)
    file(GLOB_RECURSE TETHER_ANALYSIS_SOURCES_FILTERED "${TETHER_ROOT}/src/analysis/*.cpp")
endif()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_analysis_shared SHARED ${TETHER_ANALYSIS_SOURCES_FILTERED})
    list(APPEND _variants tether_analysis_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_analysis_static STATIC ${TETHER_ANALYSIS_SOURCES_FILTERED})
    list(APPEND _variants tether_analysis_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/analysis>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/analysis>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_analysis ALIAS tether_analysis_shared)
    add_library(tether::analysis ALIAS tether_analysis_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_analysis ALIAS tether_analysis_static)
    add_library(tether::analysis ALIAS tether_analysis_static)
endif()

set(TETHER_ANALYSIS_LIBRARY tether_analysis)
set(TETHER_ANALYSIS_TARGETS ${_variants})
