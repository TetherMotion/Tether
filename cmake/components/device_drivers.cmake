# Component: tether_device_drivers
# Vendor-specific EtherCAT device drivers: CiA-402 drive helpers (register
# maps, error code tables, device-specific CoE helpers — AS715N, DynaDrive,
# Nexcobot ESC211, Synapticon, ...) and Beckhoff terminal drivers (EL200x
# digital outputs, EL1xxx digital inputs, ...).
#
# Split from tether_ethercat_master to keep the master core free of
# vendor-specific code. Depends on the EtherCAT master core (CoEManager).
#
# Per-driver source selection via the TETHER_DRIVE_<NAME> options.

set(TETHER_DEVICE_DRIVERS_SOURCES "")

# --- CiA-402 drive helpers (src/drives/) ------------------------------------

if(TETHER_DRIVE_AS715N)
    file(GLOB AS715N_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/AS715N.cpp"
        "${TETHER_ROOT}/src/drives/AS715NErrors.cpp"
        "${TETHER_ROOT}/src/drives/AS715NRegisters.cpp")
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${AS715N_SOURCES})
endif()

if(TETHER_DRIVE_DYNADRIVE)
    file(GLOB DYNADRIVE_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/DynaDrive.cpp")
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${DYNADRIVE_SOURCES})
endif()

if(TETHER_DRIVE_NEXCOBOT_ESC211)
    file(GLOB NEXCOBOT_ESC211_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/NexcobotESC211Errors.cpp")
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${NEXCOBOT_ESC211_SOURCES})
endif()

if(TETHER_DRIVE_RP20)
    file(GLOB_RECURSE RP20_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/RP20/*.cpp")
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${RP20_SOURCES})
endif()

if(TETHER_DRIVE_SYNAPTICON)
    file(GLOB SYNAPTICON_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/Synapticon*.cpp")
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${SYNAPTICON_SOURCES})
endif()

# --- Beckhoff E-Bus terminals (src/Beckhoff/) --------------------------------
# Infrastructure terminals (couplers/digital I/O) live in namespace
# EtherCAT::Beckhoff — not CiA-402 drives, but device drivers all the same.

if(TETHER_DRIVER_BECKHOFF)
    file(GLOB_RECURSE BECKHOFF_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/Beckhoff/*.cpp")
    # The TwinSAFE (FSoE) driver needs the tether_fsoe component — leave
    # it out when that component isn't part of the build.
    if(NOT TETHER_BUILD_FSOE)
        list(FILTER BECKHOFF_SOURCES EXCLUDE REGEX "SafetyTerminal\\.cpp$")
    endif()
    list(APPEND TETHER_DEVICE_DRIVERS_SOURCES ${BECKHOFF_SOURCES})
endif()

# Filter to only existing files
set(TETHER_DEVICE_DRIVERS_SOURCES_FILTERED "")
foreach(src ${TETHER_DEVICE_DRIVERS_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_DEVICE_DRIVERS_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Build compile definitions for enabled drivers (including header-only ones)
set(TETHER_DEVICE_DRIVERS_COMPILE_DEFS "")
if(TETHER_DRIVE_AS715N)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_AS715N=1)
endif()
if(TETHER_DRIVE_DYNADRIVE)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_DYNADRIVE=1)
endif()
if(TETHER_DRIVE_NEXCOBOT_ESC211)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_NEXCOBOT_ESC211=1)
endif()
if(TETHER_DRIVE_RP20)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_RP20=1)
endif()
if(TETHER_DRIVE_PBLR81FGF)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_PBLR81FGF=1)
endif()
if(TETHER_DRIVE_SYNAPTICON)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_SYNAPTICON=1)
endif()
if(TETHER_DRIVER_BECKHOFF)
    list(APPEND TETHER_DEVICE_DRIVERS_COMPILE_DEFS TETHER_HAS_BECKHOFF=1)
endif()

# Create variant targets
set(_variants "")
if(TETHER_DEVICE_DRIVERS_SOURCES_FILTERED)
    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_device_drivers_shared SHARED ${TETHER_DEVICE_DRIVERS_SOURCES_FILTERED})
        list(APPEND _variants tether_device_drivers_shared)
    endif()
    if(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_device_drivers_static STATIC ${TETHER_DEVICE_DRIVERS_SOURCES_FILTERED})
        list(APPEND _variants tether_device_drivers_static)
    endif()
else()
    # No compiled sources — create a header-only INTERFACE library so that
    # consumers can still link tether_device_drivers for header-only drivers
    # (RP20, PBLR81FGF).
    add_library(tether_device_drivers_header INTERFACE)
    list(APPEND _variants tether_device_drivers_header)
endif()

foreach(_tgt IN LISTS _variants)
    if(TARGET ${_tgt})
        get_target_property(_tgt_type ${_tgt} TYPE)
        if(_tgt_type STREQUAL "INTERFACE_LIBRARY")
            target_include_directories(${_tgt}
                INTERFACE
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include>
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/drives>
                    $<INSTALL_INTERFACE:include>
                    $<INSTALL_INTERFACE:include/tether>
                    $<INSTALL_INTERFACE:include/tether/drives>
            )
            target_link_libraries(${_tgt} INTERFACE tether_common tether_ethercat_master)
            if(TETHER_DEVICE_DRIVERS_COMPILE_DEFS)
                target_compile_definitions(${_tgt} INTERFACE ${TETHER_DEVICE_DRIVERS_COMPILE_DEFS})
            endif()
        else()
            target_include_directories(${_tgt}
                PUBLIC
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include>
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
                    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/drives>
                    $<INSTALL_INTERFACE:include>
                    $<INSTALL_INTERFACE:include/tether>
                    $<INSTALL_INTERFACE:include/tether/drives>
                PRIVATE
                    ${TETHER_ROOT}/src
            )
            target_link_libraries(${_tgt} PUBLIC tether_common tether_ethercat_master)
            # TwinSAFE terminal driver links the FSoE stack when present
            # (resolved at generate time — the target may be created by a
            # later component include).
            if(TETHER_BUILD_FSOE)
                target_link_libraries(${_tgt} PUBLIC tether_fsoe)
            endif()
            if(TETHER_DEVICE_DRIVERS_COMPILE_DEFS)
                target_compile_definitions(${_tgt} PUBLIC ${TETHER_DEVICE_DRIVERS_COMPILE_DEFS})
            endif()
            set_target_properties(${_tgt} PROPERTIES
                POSITION_INDEPENDENT_CODE ON
                CXX_STANDARD 23
                CXX_STANDARD_REQUIRED ON
            )
        endif()
    endif()
endforeach()

if(TETHER_DEVICE_DRIVERS_SOURCES_FILTERED)
    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_device_drivers ALIAS tether_device_drivers_shared)
        add_library(tether::device_drivers ALIAS tether_device_drivers_shared)
    elseif(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_device_drivers ALIAS tether_device_drivers_static)
        add_library(tether::device_drivers ALIAS tether_device_drivers_static)
    endif()
else()
    add_library(tether_device_drivers ALIAS tether_device_drivers_header)
    add_library(tether::device_drivers ALIAS tether_device_drivers_header)
endif()

set(TETHER_DEVICE_DRIVERS_LIBRARY tether_device_drivers)
set(TETHER_DEVICE_DRIVERS_TARGETS ${_variants})
