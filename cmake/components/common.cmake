# Component: tether_common
# Shared utilities, types, platform abstraction, and configuration

set(TETHER_COMMON_SOURCES
    ${TETHER_ROOT}/src/platform/Platform.cpp
    ${TETHER_ROOT}/src/platform/HostTimer.cpp
    ${TETHER_ROOT}/src/logging/Logger.cpp
    ${TETHER_ROOT}/src/logging/DeduplicatingLogger.cpp
)

# Add ESP32-specific platform sources if building for ESP32
if(ESP_PLATFORM)
    list(APPEND TETHER_COMMON_SOURCES
        ${TETHER_ROOT}/src/platform/ESP32Timer.cpp
    )
endif()

set(TETHER_COMMON_HEADERS
    ${TETHER_ROOT}/include/tether/Tether.hpp
    ${TETHER_ROOT}/include/tether/TetherConfig.hpp
    ${TETHER_ROOT}/include/tether/platform/Platform.hpp
    ${TETHER_ROOT}/include/tether/platform/ESPStubs.hpp
    ${TETHER_ROOT}/include/tether/platform/IPlatformTimer.hpp
)

# Create the common library
add_library(tether_common STATIC ${TETHER_COMMON_SOURCES})
add_library(tether::common ALIAS tether_common)

target_include_directories(tether_common
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/tether>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_compile_definitions(tether_common PUBLIC
    UNIT_TEST_HOST=1
)

set_target_properties(tether_common PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_COMMON_LIBRARY tether_common PARENT_SCOPE)
