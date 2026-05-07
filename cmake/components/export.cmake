# Component: tether_export
# Export functionality: CSV, SVG trajectory visualization and analysis

set(TETHER_EXPORT_SOURCES
    ${TETHER_ROOT}/src/export/CSVExporter.cpp
    ${TETHER_ROOT}/src/export/SVGExporter.cpp
    ${TETHER_ROOT}/src/export/TrajectoryAnalyzer.cpp
)

# Filter to only existing files
set(TETHER_EXPORT_SOURCES_FILTERED "")
foreach(src ${TETHER_EXPORT_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_EXPORT_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Fallback to glob
if(NOT TETHER_EXPORT_SOURCES_FILTERED)
    file(GLOB_RECURSE TETHER_EXPORT_SOURCES_FILTERED "${TETHER_ROOT}/src/export/*.cpp")
endif()

# Create the export library
add_library(tether_export STATIC ${TETHER_EXPORT_SOURCES_FILTERED})
add_library(tether::export ALIAS tether_export)

target_include_directories(tether_export
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/export>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
        $<INSTALL_INTERFACE:include/tether/export>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_export
    PUBLIC tether_common tether_gcode
)

set_target_properties(tether_export PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_EXPORT_LIBRARY tether_export PARENT_SCOPE)
