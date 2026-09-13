# Component: tether_beckhoff
# Beckhoff EtherCAT terminal drivers (EL2004 digital output, ...).
#
# Unlike drives/, these are infrastructure terminals on the E-Bus — they
# live in namespace EtherCAT::Beckhoff and only depend on the master core.

file(GLOB_RECURSE TETHER_BECKHOFF_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/Beckhoff/*.cpp")

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_beckhoff_shared SHARED ${TETHER_BECKHOFF_SOURCES})
    list(APPEND _variants tether_beckhoff_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_beckhoff_static STATIC ${TETHER_BECKHOFF_SOURCES})
    list(APPEND _variants tether_beckhoff_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
        PRIVATE
            ${TETHER_ROOT}/src
    )
    target_link_libraries(${_tgt} PUBLIC tether_common tether_ethercat_master)
    target_compile_definitions(${_tgt} PUBLIC TETHER_HAS_BECKHOFF=1)
    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_beckhoff ALIAS tether_beckhoff_shared)
    add_library(tether::beckhoff ALIAS tether_beckhoff_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_beckhoff ALIAS tether_beckhoff_static)
    add_library(tether::beckhoff ALIAS tether_beckhoff_static)
endif()

set(TETHER_BECKHOFF_LIBRARY tether_beckhoff)
set(TETHER_BECKHOFF_TARGETS ${_variants})
