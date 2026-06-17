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

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_export_shared SHARED ${TETHER_EXPORT_SOURCES_FILTERED})
    list(APPEND _variants tether_export_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_export_static STATIC ${TETHER_EXPORT_SOURCES_FILTERED})
    list(APPEND _variants tether_export_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
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

    target_link_libraries(${_tgt} PUBLIC tether_common tether_gcode)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_export ALIAS tether_export_shared)
    add_library(tether::export ALIAS tether_export_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_export ALIAS tether_export_static)
    add_library(tether::export ALIAS tether_export_static)
endif()

set(TETHER_EXPORT_LIBRARY tether_export)
set(TETHER_EXPORT_TARGETS ${_variants})
