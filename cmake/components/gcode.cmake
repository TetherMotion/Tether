# Component: tether_gcode
# G-code parsing, interpretation, and trajectory generation

# Collect all gcode sources
# Note: CONFIGURE_DEPENDS makes CMake re-run when new .cpp files are added.
file(GLOB_RECURSE TETHER_GCODE_SOURCES CONFIGURE_DEPENDS "${TETHER_ROOT}/src/gcode/*.cpp")

# Exclude legacy C API files
list(FILTER TETHER_GCODE_SOURCES EXCLUDE REGEX "GCodeCAPI")

# Create the gcode library
add_library(tether_gcode STATIC ${TETHER_GCODE_SOURCES})
add_library(tether::gcode ALIAS tether_gcode)

target_include_directories(tether_gcode
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

target_link_libraries(tether_gcode
    PUBLIC tether_common
)

set_target_properties(tether_gcode PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_GCODE_LIBRARY tether_gcode)
