# Component: tether_hal
# Hardware Abstraction Layer - clock, threading, networking

set(TETHER_HAL_SOURCES
    ${TETHER_ROOT}/src/hal/HAL_Common.cpp
    ${TETHER_ROOT}/src/hal/EthernetHAL_Common.cpp
    ${TETHER_ROOT}/src/hal/PcapNGLogger.cpp
    ${TETHER_ROOT}/src/hal/StateMachineLogger.cpp
    # Note: FIFOHAL.cpp and LoopbackHAL.cpp excluded - outdated interface implementation
)

# Add platform-specific HAL sources
if(TETHER_PLATFORM_LINUX OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND TETHER_HAL_SOURCES
        ${TETHER_ROOT}/src/hal/LinuxClock.cpp
        ${TETHER_ROOT}/src/hal/LinuxEthernet.cpp
        ${TETHER_ROOT}/src/hal/LinuxThreading.cpp
    )
    set(TETHER_HAL_PLATFORM_LIBS pthread)
elseif(TETHER_PLATFORM_ESP32)
    list(APPEND TETHER_HAL_SOURCES
        ${TETHER_ROOT}/src/hal/ESP32Clock.cpp
        ${TETHER_ROOT}/src/hal/ESP32Ethernet.cpp
        ${TETHER_ROOT}/src/hal/ESP32Timer.cpp
        ${TETHER_ROOT}/src/hal/ESP32Threading.cpp
    )
elseif(TETHER_PLATFORM_STM32)
    list(APPEND TETHER_HAL_SOURCES
        ${TETHER_ROOT}/src/hal/STM32Clock.cpp
        ${TETHER_ROOT}/src/hal/STM32Ethernet.cpp
        ${TETHER_ROOT}/src/hal/STM32Threading.cpp
    )
endif()

# Create the HAL library
add_library(tether_hal STATIC ${TETHER_HAL_SOURCES})
add_library(tether::hal ALIAS tether_hal)

target_include_directories(tether_hal
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/hal>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
        $<INSTALL_INTERFACE:include/tether/hal>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_hal
    PUBLIC tether_common
    ${TETHER_HAL_PLATFORM_LIBS}
)

set_target_properties(tether_hal PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_HAL_LIBRARY tether_hal)
