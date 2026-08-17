# Component: tether_cia_profiles
# CiA (CAN in Automation) device profile implementations and ETG5000 modular
# device support, master-side. Contains the drive-protocol / device-profile
# classes that sit on top of the EtherCAT master core:
#   - CiA 301: common communication profile
#   - CiA 401: generic I/O modules
#   - CiA 402: drive protocol (state machine, PDO mapping, homing, DS402 master,
#              electronic gearing, motion profile, EtherCAT backend). The CiA 402
#              *motion logic* (MotionController, motor models, profile generator,
#              multi-axis path) remains in tether_motion_control.
#   - CiA 404: weighing/torque measurement
#   - CiA 405: IEC 61131-3 programmable
#   - CiA 406: encoder
#   - CiA 408: fluid power / valves
#   - CiA 410: inclinometer
#   - CiA 417: lift control
#   - CiA 430: energy metering
#   - ETG5000: EtherCAT Technology Group modular device profile (not a CiA
#              profile, but grouped here as a device-profile sibling).
#
# CiA 402 register/type-definition headers (60xx-Parameters.hpp,
# 1Cxx-SyncManagerParameters.hpp, CiA402Config.hpp, HomingModes.hpp, etc.)
# remain under include/tether/profiles/cia402/ and are also exported by
# tether_ethercat_master so users can tightly integrate without pulling in
# the full profile implementation.
#
# Split from tether_ethercat_master to keep the master core focused on the
# EtherCAT protocol itself. Depends on the EtherCAT master core (CoEManager,
# Master, DC, FaultDetection, SDOManager, PDOManager).

# CiA 402 drive-protocol sources: all cia402/*.cpp EXCEPT the motion-control
# files that belong to tether_motion_control.
file(GLOB CIA402_PROFILE_SOURCES CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia402/*.cpp")
list(FILTER CIA402_PROFILE_SOURCES EXCLUDE REGEX "MotionController\\.cpp$")
list(FILTER CIA402_PROFILE_SOURCES EXCLUDE REGEX "MotorModel.*\\.cpp$")
list(FILTER CIA402_PROFILE_SOURCES EXCLUDE REGEX "MultiAxisPath\\.cpp$")
list(FILTER CIA402_PROFILE_SOURCES EXCLUDE REGEX "PIDController\\.cpp$")
list(FILTER CIA402_PROFILE_SOURCES EXCLUDE REGEX "ProfileGenerator\\.cpp$")

# Other CiA profile sources
file(GLOB CIA301_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia301/*.cpp")
file(GLOB CIA401_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia401/*.cpp")
file(GLOB CIA404_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia404/*.cpp")
file(GLOB CIA405_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia405/*.cpp")
file(GLOB CIA406_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia406/*.cpp")
file(GLOB CIA408_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia408/*.cpp")
file(GLOB CIA410_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia410/*.cpp")
file(GLOB CIA417_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia417/*.cpp")
file(GLOB CIA430_SOURCES  CONFIGURE_DEPENDS "${TETHER_ROOT}/src/profiles/cia430/*.cpp")

# ETG5000 modular device profile
file(GLOB ETG5000_SOURCES CONFIGURE_DEPENDS "${TETHER_ROOT}/src/etg5000/*.cpp")

set(TETHER_CIA_PROFILES_SOURCES "")
if(TETHER_ENABLE_CIA402)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA402_PROFILE_SOURCES})
endif()
if(TETHER_ENABLE_CIA301)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA301_SOURCES})
endif()
if(TETHER_ENABLE_CIA401)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA401_SOURCES})
endif()
if(TETHER_ENABLE_CIA404)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA404_SOURCES})
endif()
if(TETHER_ENABLE_CIA405)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA405_SOURCES})
endif()
if(TETHER_ENABLE_CIA406)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA406_SOURCES})
endif()
if(TETHER_ENABLE_CIA408)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA408_SOURCES})
endif()
if(TETHER_ENABLE_CIA410)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA410_SOURCES})
endif()
if(TETHER_ENABLE_CIA417)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA417_SOURCES})
endif()
if(TETHER_ENABLE_CIA430)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${CIA430_SOURCES})
endif()
if(TETHER_ENABLE_ETG5000)
    list(APPEND TETHER_CIA_PROFILES_SOURCES ${ETG5000_SOURCES})
endif()

list(REMOVE_DUPLICATES TETHER_CIA_PROFILES_SOURCES)

if(NOT TETHER_CIA_PROFILES_SOURCES)
    message(WARNING "TETHER_BUILD_CIA_PROFILES is ON but all per-profile options are OFF; skipping tether_cia_profiles")
    return()
endif()

# Public include directories only for enabled profiles
set(TETHER_CIA_PROFILES_PUBLIC_INCLUDES
    $<BUILD_INTERFACE:${TETHER_ROOT}/include>
    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
)
if(TETHER_ENABLE_CIA301)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia301>)
endif()
if(TETHER_ENABLE_CIA401)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia401>)
endif()
if(TETHER_ENABLE_CIA402)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia402>)
endif()
if(TETHER_ENABLE_CIA404)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia404>)
endif()
if(TETHER_ENABLE_CIA405)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia405>)
endif()
if(TETHER_ENABLE_CIA406)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia406>)
endif()
if(TETHER_ENABLE_CIA408)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia408>)
endif()
if(TETHER_ENABLE_CIA410)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia410>)
endif()
if(TETHER_ENABLE_CIA417)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia417>)
endif()
if(TETHER_ENABLE_CIA430)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia430>)
endif()
if(TETHER_ENABLE_ETG5000)
    list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/etg5000>)
endif()
list(APPEND TETHER_CIA_PROFILES_PUBLIC_INCLUDES
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/tether>
)

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_cia_profiles_shared SHARED ${TETHER_CIA_PROFILES_SOURCES})
    list(APPEND _variants tether_cia_profiles_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_cia_profiles_static STATIC ${TETHER_CIA_PROFILES_SOURCES})
    list(APPEND _variants tether_cia_profiles_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            ${TETHER_CIA_PROFILES_PUBLIC_INCLUDES}
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common tether_ethercat_master tether_motion_control)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_cia_profiles ALIAS tether_cia_profiles_shared)
    add_library(tether::cia_profiles ALIAS tether_cia_profiles_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_cia_profiles ALIAS tether_cia_profiles_static)
    add_library(tether::cia_profiles ALIAS tether_cia_profiles_static)
endif()

set(TETHER_CIA_PROFILES_LIBRARY tether_cia_profiles)
set(TETHER_CIA_PROFILES_TARGETS ${_variants})
