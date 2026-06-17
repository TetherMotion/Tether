# Component: tether_ethercat_master
# EtherCAT master implementation and CiA device profiles

# Core EtherCAT master sources
file(GLOB ETHERCAT_CORE_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/ethercat/*.cpp"
    "${TETHER_ROOT}/src/ethercat/utils/*.cpp"
    "${TETHER_ROOT}/src/ethercat/mailbox/*.cpp"
)
file(GLOB ETHERCAT_RAW_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/ethercat/*.cpp"
    "${TETHER_ROOT}/src/ethercat/utils/*.cpp"
    "${TETHER_ROOT}/src/ethercat/mailbox/*.cpp"
    "${TETHER_ROOT}/src/ethercat/raw/*.cpp"
)

# Filter out platform-specific and stub files from ethercat sources
list(FILTER ETHERCAT_RAW_SOURCES EXCLUDE REGEX "platform_esp32\\.cpp$")
list(FILTER ETHERCAT_RAW_SOURCES EXCLUDE REGEX "host_stubs\\.cpp$")
list(FILTER ETHERCAT_RAW_SOURCES EXCLUDE REGEX "pdo_stubs\\.cpp$")

# Reset handling
file(GLOB RESET_SOURCES "${TETHER_ROOT}/src/reset/*.cpp")

# FSoE support
file(GLOB FSOE_SOURCES "${TETHER_ROOT}/src/fsoe/*.cpp")

# ETG5000 modular device
file(GLOB ETG5000_SOURCES "${TETHER_ROOT}/src/etg5000/*.cpp")

# CiA profile implementations (master-side device control)
file(GLOB CIA301_SOURCES "${TETHER_ROOT}/src/profiles/cia301/*.cpp")
file(GLOB CIA401_SOURCES "${TETHER_ROOT}/src/profiles/cia401/*.cpp")
file(GLOB CIA404_SOURCES "${TETHER_ROOT}/src/profiles/cia404/*.cpp")
file(GLOB CIA405_SOURCES "${TETHER_ROOT}/src/profiles/cia405/*.cpp")
file(GLOB CIA406_SOURCES "${TETHER_ROOT}/src/profiles/cia406/*.cpp")
file(GLOB CIA408_SOURCES "${TETHER_ROOT}/src/profiles/cia408/*.cpp")
file(GLOB CIA410_SOURCES "${TETHER_ROOT}/src/profiles/cia410/*.cpp")
file(GLOB CIA417_SOURCES "${TETHER_ROOT}/src/profiles/cia417/*.cpp")
file(GLOB CIA430_SOURCES "${TETHER_ROOT}/src/profiles/cia430/*.cpp")

# CiA 402 drive sources (excluding motion control which is in motion_control component)
file(GLOB CIA402_SOURCES "${TETHER_ROOT}/src/profiles/cia402/*.cpp")
# Keep only drive protocol sources, not motion control
list(FILTER CIA402_SOURCES EXCLUDE REGEX "MotionController\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "AxisMotion\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "PathMotion\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "ProfileGenerator\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "MultiAxisPath\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "MotorModel.*\\.cpp$")
list(FILTER CIA402_SOURCES EXCLUDE REGEX "AdvancedMotorModel.*\\.cpp$")

# Driver-specific helpers (device-specific code such as AS715N)
file(GLOB DRIVES_SOURCES "${TETHER_ROOT}/src/drives/*.cpp")

set(TETHER_ETHERCAT_MASTER_SOURCES
    ${ETHERCAT_CORE_SOURCES}
    ${ETHERCAT_RAW_SOURCES}
    ${TETHER_ROOT}/src/fmmu/FMMUConfiguration.cpp
    ${RESET_SOURCES}
    ${FSOE_SOURCES}
    ${ETG5000_SOURCES}
    ${CIA301_SOURCES}
    ${CIA401_SOURCES}
    ${CIA402_SOURCES}
    ${CIA404_SOURCES}
    ${CIA405_SOURCES}
    ${CIA406_SOURCES}
    ${CIA408_SOURCES}
    ${CIA410_SOURCES}
    ${CIA417_SOURCES}
    ${CIA430_SOURCES}
    ${DRIVES_SOURCES}
)

# Remove duplicates
list(REMOVE_DUPLICATES TETHER_ETHERCAT_MASTER_SOURCES)

# Create the ethercat master library
add_library(tether_ethercat_master STATIC ${TETHER_ETHERCAT_MASTER_SOURCES})
add_library(tether::ethercat_master ALIAS tether_ethercat_master)

target_include_directories(tether_ethercat_master
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/ethercat>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia301>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia401>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia402>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia404>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia405>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia406>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia408>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia410>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia417>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia430>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/etg5000>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/fsoe>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/reset>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/drives>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
    PRIVATE
        ${TETHER_ROOT}/src
        ${TETHER_ROOT}/src/ethercat
)

target_link_libraries(tether_ethercat_master
    PUBLIC tether_common tether_hal tether_ethercat_common tether_controls tether_motion_control
)

target_compile_definitions(tether_ethercat_master PUBLIC TETHER_COMPILE_MASTER=1)

set_target_properties(tether_ethercat_master PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_ETHERCAT_MASTER_LIBRARY tether_ethercat_master)
