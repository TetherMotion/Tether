# Component: tether_gcode
# G-code parsing, interpretation, and trajectory generation

# Collect all gcode sources
# Note: CONFIGURE_DEPENDS makes CMake re-run when new .cpp files are added.
file(GLOB_RECURSE TETHER_GCODE_SOURCES CONFIGURE_DEPENDS "${TETHER_ROOT}/src/gcode/*.cpp")

# Exclude legacy C API files
list(FILTER TETHER_GCODE_SOURCES EXCLUDE REGEX "GCodeCAPI")

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_gcode_shared SHARED ${TETHER_GCODE_SOURCES})
    list(APPEND _variants tether_gcode_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_gcode_static STATIC ${TETHER_GCODE_SOURCES})
    list(APPEND _variants tether_gcode_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/gcode>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/gcode/motion>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/gcode>
        PRIVATE
            ${TETHER_ROOT}/src
            ${TETHER_ROOT}/src/gcode
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_gcode ALIAS tether_gcode_shared)
    add_library(tether::gcode ALIAS tether_gcode_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_gcode ALIAS tether_gcode_static)
    add_library(tether::gcode ALIAS tether_gcode_static)
endif()

set(TETHER_GCODE_LIBRARY tether_gcode)
set(TETHER_GCODE_TARGETS ${_variants})
