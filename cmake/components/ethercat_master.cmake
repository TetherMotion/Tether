# Component: tether_ethercat_master
# EtherCAT master core: protocol implementation, mailbox (CoE/SoE/FoE/EoE),
# raw transport, distributed clocks, FMMU configuration, SII reader, reset
# handling, diagnostics, realtime loop, and CiA 402 register/type-definition
# headers.
#
# CiA device profile implementations, ETG5000, vendor drives, and FSoE now live
# in separate opt-in components:
#   - tether_cia_profiles : CiA 301/401/402/404/405/406/408/410/417/430 + ETG5000
#   - tether_drives       : vendor-specific drive helpers (AS715N, DynaDrive, ...)
#   - tether_fsoe         : Fail-Safe over EtherCAT (ETG 5100)
#
# The CiA 402 register/type-definition headers (60xx-Parameters.hpp,
# 1Cxx-SyncManagerParameters.hpp, CiA402Config.hpp, HomingModes.hpp, etc.)
# remain under include/tether/profiles/cia402/ and are exported by this
# target so users can tightly integrate without pulling in the full profile
# implementation.

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
# ESIParser.cpp lives in tether_esi (requires tinyxml2) — keep it out of the
# runtime master library.
list(FILTER ETHERCAT_CORE_SOURCES EXCLUDE REGEX "ESIParser\\.cpp$")
list(FILTER ETHERCAT_RAW_SOURCES EXCLUDE REGEX "ESIParser\\.cpp$")

# Reset handling
file(GLOB RESET_SOURCES "${TETHER_ROOT}/src/reset/*.cpp")

set(TETHER_ETHERCAT_MASTER_SOURCES
    ${ETHERCAT_CORE_SOURCES}
    ${ETHERCAT_RAW_SOURCES}
    ${TETHER_ROOT}/src/fmmu/FMMUConfiguration.cpp
    ${TETHER_ROOT}/src/sii/SIIReader.cpp
    ${TETHER_ROOT}/src/sii/SIILogger.cpp
    ${RESET_SOURCES}
)

# Remove duplicates
list(REMOVE_DUPLICATES TETHER_ETHERCAT_MASTER_SOURCES)

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_ethercat_master_shared SHARED ${TETHER_ETHERCAT_MASTER_SOURCES})
    list(APPEND _variants tether_ethercat_master_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_ethercat_master_static STATIC ${TETHER_ETHERCAT_MASTER_SOURCES})
    list(APPEND _variants tether_ethercat_master_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/ethercat>
            # CiA 402 register/type-definition headers stay accessible from the
            # master core so users can tightly integrate without tether_cia_profiles.
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia402>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/reset>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
        PRIVATE
            ${TETHER_ROOT}/src
            ${TETHER_ROOT}/src/ethercat
    )

    target_link_libraries(${_tgt}
        PUBLIC tether_common tether_hal tether_ethercat_common tether_controls
    )

    target_compile_definitions(${_tgt} PUBLIC TETHER_COMPILE_MASTER=1)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_ethercat_master ALIAS tether_ethercat_master_shared)
    add_library(tether::ethercat_master ALIAS tether_ethercat_master_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_ethercat_master ALIAS tether_ethercat_master_static)
    add_library(tether::ethercat_master ALIAS tether_ethercat_master_static)
endif()

set(TETHER_ETHERCAT_MASTER_LIBRARY tether_ethercat_master)
set(TETHER_ETHERCAT_MASTER_TARGETS ${_variants})
