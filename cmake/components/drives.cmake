# Component: tether_drives
# Vendor-specific drive helpers: register maps, error code tables, and
# device-specific CoE helpers (AS715N, DynaDrive, Nexcobot ESC211, etc.).
#
# Split from tether_ethercat_master to keep the master core free of
# vendor-specific code. Depends on the EtherCAT master core (CoEManager).

# Per-driver source selection (controlled by TETHER_DRIVE_<NAME> options)
set(TETHER_DRIVES_SOURCES "")

if(TETHER_DRIVE_AS715N)
    file(GLOB AS715N_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/AS715N.cpp"
        "${TETHER_ROOT}/src/drives/AS715NErrors.cpp"
        "${TETHER_ROOT}/src/drives/AS715NRegisters.cpp")
    list(APPEND TETHER_DRIVES_SOURCES ${AS715N_SOURCES})
endif()

if(TETHER_DRIVE_DYNADRIVE)
    file(GLOB DYNADRIVE_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/DynaDrive.cpp")
    list(APPEND TETHER_DRIVES_SOURCES ${DYNADRIVE_SOURCES})
endif()

if(TETHER_DRIVE_NEXCOBOT_ESC211)
    file(GLOB NEXCOBOT_ESC211_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/NexcobotESC211Errors.cpp")
    list(APPEND TETHER_DRIVES_SOURCES ${NEXCOBOT_ESC211_SOURCES})
endif()

if(TETHER_DRIVE_RP20)
    file(GLOB_RECURSE RP20_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/RP20/*.cpp")
    list(APPEND TETHER_DRIVES_SOURCES ${RP20_SOURCES})
endif()

if(TETHER_DRIVE_SYNAPTICON)
    file(GLOB SYNAPTICON_SOURCES CONFIGURE_DEPENDS
        "${TETHER_ROOT}/src/drives/Synapticon*.cpp")
    list(APPEND TETHER_DRIVES_SOURCES ${SYNAPTICON_SOURCES})
endif()

# Filter to only existing files
set(TETHER_DRIVES_SOURCES_FILTERED "")
foreach(src ${TETHER_DRIVES_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_DRIVES_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Build compile definitions for enabled drivers (including header-only ones)
set(TETHER_DRIVES_COMPILE_DEFS "")
if(TETHER_DRIVE_AS715N)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_AS715N=1)
endif()
if(TETHER_DRIVE_DYNADRIVE)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_DYNADRIVE=1)
endif()
if(TETHER_DRIVE_NEXCOBOT_ESC211)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_NEXCOBOT_ESC211=1)
endif()
if(TETHER_DRIVE_RP20)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_RP20=1)
endif()
if(TETHER_DRIVE_PBLR81FGF)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_PBLR81FGF=1)
endif()
if(TETHER_DRIVE_SYNAPTICON)
    list(APPEND TETHER_DRIVES_COMPILE_DEFS TETHER_HAS_SYNAPTICON=1)
endif()

# Create variant targets
set(_variants "")
if(TETHER_DRIVES_SOURCES_FILTERED)
    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_drives_shared SHARED ${TETHER_DRIVES_SOURCES_FILTERED})
        list(APPEND _variants tether_drives_shared)
    endif()
    if(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_drives_static STATIC ${TETHER_DRIVES_SOURCES_FILTERED})
        list(APPEND _variants tether_drives_static)
    endif()
else()
    # No compiled sources — create a header-only INTERFACE library so that
    # consumers can still link tether_drives for header-only drivers (RP20, PBLR81FGF).
    add_library(tether_drives_header INTERFACE)
    list(APPEND _variants tether_drives_header)
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
            if(TETHER_DRIVES_COMPILE_DEFS)
                target_compile_definitions(${_tgt} INTERFACE ${TETHER_DRIVES_COMPILE_DEFS})
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
            if(TETHER_DRIVES_COMPILE_DEFS)
                target_compile_definitions(${_tgt} PUBLIC ${TETHER_DRIVES_COMPILE_DEFS})
            endif()
            set_target_properties(${_tgt} PROPERTIES
                POSITION_INDEPENDENT_CODE ON
                CXX_STANDARD 23
                CXX_STANDARD_REQUIRED ON
            )
        endif()
    endif()
endforeach()

if(TETHER_DRIVES_SOURCES_FILTERED)
    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_drives ALIAS tether_drives_shared)
        add_library(tether::drives ALIAS tether_drives_shared)
    elseif(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_drives ALIAS tether_drives_static)
        add_library(tether::drives ALIAS tether_drives_static)
    endif()
else()
    add_library(tether_drives ALIAS tether_drives_header)
    add_library(tether::drives ALIAS tether_drives_header)
endif()

set(TETHER_DRIVES_LIBRARY tether_drives)
set(TETHER_DRIVES_TARGETS ${_variants})
