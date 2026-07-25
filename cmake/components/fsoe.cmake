# Component: tether_fsoe
# FSoE (Fail-Safe over EtherCAT, ETG 5100) safety protocol implementation:
# master connection, master/slave FSMs, CRC, typed process data, and the
# Synapticon SafeMotion FSoE integration.
#
# Split from tether_ethercat_master to keep the master core free of
# safety-protocol code. FSoE is an opt-in safety layer that depends on the
# EtherCAT master core (CyclicTaskScheduler, PDOManager, PDORegionManager,
# RealtimeLoop) and the CiA 402 DS402 master (for the Synapticon integration).

file(GLOB_RECURSE TETHER_FSOE_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/fsoe/*.cpp"
)

# Filter to only existing files
set(TETHER_FSOE_SOURCES_FILTERED "")
foreach(src ${TETHER_FSOE_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_FSOE_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_fsoe_shared SHARED ${TETHER_FSOE_SOURCES_FILTERED})
    list(APPEND _variants tether_fsoe_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_fsoe_static STATIC ${TETHER_FSOE_SOURCES_FILTERED})
    list(APPEND _variants tether_fsoe_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/fsoe>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/fsoe/Synapticon>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/fsoe>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt}
        PUBLIC tether_common tether_ethercat_master tether_cia_profiles
    )

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_fsoe ALIAS tether_fsoe_shared)
    add_library(tether::fsoe ALIAS tether_fsoe_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_fsoe ALIAS tether_fsoe_static)
    add_library(tether::fsoe ALIAS tether_fsoe_static)
endif()

set(TETHER_FSOE_LIBRARY tether_fsoe)
set(TETHER_FSOE_TARGETS ${_variants})
