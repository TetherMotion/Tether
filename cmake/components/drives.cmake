# Component: tether_drives
# Vendor-specific drive helpers: register maps, error code tables, and
# device-specific CoE helpers (AS715N, DynaDrive, Nexcobot ESC211, etc.).
#
# Split from tether_ethercat_master to keep the master core free of
# vendor-specific code. Depends on the EtherCAT master core (CoEManager).

file(GLOB_RECURSE TETHER_DRIVES_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/drives/*.cpp"
)

# Filter to only existing files
set(TETHER_DRIVES_SOURCES_FILTERED "")
foreach(src ${TETHER_DRIVES_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_DRIVES_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_drives_shared SHARED ${TETHER_DRIVES_SOURCES_FILTERED})
    list(APPEND _variants tether_drives_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_drives_static STATIC ${TETHER_DRIVES_SOURCES_FILTERED})
    list(APPEND _variants tether_drives_static)
endif()

foreach(_tgt IN LISTS _variants)
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

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_drives ALIAS tether_drives_shared)
    add_library(tether::drives ALIAS tether_drives_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_drives ALIAS tether_drives_static)
    add_library(tether::drives ALIAS tether_drives_static)
endif()

set(TETHER_DRIVES_LIBRARY tether_drives)
set(TETHER_DRIVES_TARGETS ${_variants})
