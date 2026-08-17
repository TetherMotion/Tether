# Component: tether_common
# Shared utilities, types, platform abstraction, and configuration

set(TETHER_COMMON_SOURCES
    ${TETHER_ROOT}/src/platform/Platform.cpp
    ${TETHER_ROOT}/src/platform/HostTimer.cpp
    ${TETHER_ROOT}/src/logging/Logger.cpp
    ${TETHER_ROOT}/src/logging/DeduplicatingLogger.cpp
    ${TETHER_ROOT}/src/common/MotionProfile.cpp
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
    ${TETHER_ROOT}/include/tether/common/IMotionController.hpp
    ${TETHER_ROOT}/include/tether/common/ISetpointSource.hpp
    ${TETHER_ROOT}/include/tether/common/IAxis.hpp
    ${TETHER_ROOT}/include/tether/common/MotionTypes.hpp
    ${TETHER_ROOT}/include/tether/platform/Platform.hpp
    ${TETHER_ROOT}/include/tether/platform/ESPStubs.hpp
    ${TETHER_ROOT}/include/tether/platform/IPlatformTimer.hpp
)

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_common_shared SHARED ${TETHER_COMMON_SOURCES})
    list(APPEND _variants tether_common_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_common_static STATIC ${TETHER_COMMON_SOURCES})
    list(APPEND _variants tether_common_static)
endif()

# Apply properties to every variant
foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
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

    target_compile_definitions(${_tgt} PUBLIC)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

# Create convenience aliases (shared preferred)
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_common ALIAS tether_common_shared)
    add_library(tether::common ALIAS tether_common_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_common ALIAS tether_common_static)
    add_library(tether::common ALIAS tether_common_static)
endif()

# Export for other components
set(TETHER_COMMON_LIBRARY tether_common)
set(TETHER_COMMON_TARGETS ${_variants})
