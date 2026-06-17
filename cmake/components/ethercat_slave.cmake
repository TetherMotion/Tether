# Component: tether_ethercat_slave
# EtherCAT slave emulation and profile implementations

# Core slave sources
file(GLOB_RECURSE SLAVE_CORE_SOURCES "${TETHER_ROOT}/src/slave/core/*.cpp")
file(GLOB_RECURSE SLAVE_DC_SOURCES "${TETHER_ROOT}/src/slave/dc/*.cpp")
file(GLOB_RECURSE SLAVE_HAL_SOURCES "${TETHER_ROOT}/src/slave/hal/*.cpp")
file(GLOB_RECURSE SLAVE_LOGGING_SOURCES "${TETHER_ROOT}/src/slave/logging/*.cpp")
file(GLOB_RECURSE SLAVE_MAILBOX_SOURCES "${TETHER_ROOT}/src/slave/mailbox/*.cpp")
file(GLOB_RECURSE SLAVE_PROFILES_SOURCES "${TETHER_ROOT}/src/slave/profiles/*.cpp")

set(TETHER_ETHERCAT_SLAVE_SOURCES
    ${SLAVE_CORE_SOURCES}
    ${SLAVE_DC_SOURCES}
    ${SLAVE_HAL_SOURCES}
    ${SLAVE_LOGGING_SOURCES}
    ${SLAVE_MAILBOX_SOURCES}
    ${SLAVE_PROFILES_SOURCES}
)

# Remove duplicates
list(REMOVE_DUPLICATES TETHER_ETHERCAT_SLAVE_SOURCES)

# Create the ethercat slave library (only if sources found)
if(TETHER_ETHERCAT_SLAVE_SOURCES)
    set(_variants "")
    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_ethercat_slave_shared SHARED ${TETHER_ETHERCAT_SLAVE_SOURCES})
        list(APPEND _variants tether_ethercat_slave_shared)
    endif()
    if(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_ethercat_slave_static STATIC ${TETHER_ETHERCAT_SLAVE_SOURCES})
        list(APPEND _variants tether_ethercat_slave_static)
    endif()

    foreach(_tgt IN LISTS _variants)
        target_include_directories(${_tgt}
            PUBLIC
                $<BUILD_INTERFACE:${TETHER_ROOT}/include>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/core>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/dc>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/hal>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/logging>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/mailbox>
                $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/slave/profiles>
                $<INSTALL_INTERFACE:include>
                $<INSTALL_INTERFACE:include/tether>
            PRIVATE
                ${TETHER_ROOT}/src
                ${TETHER_ROOT}/src/slave
        )

        target_link_libraries(${_tgt}
            PUBLIC tether_common tether_hal tether_ethercat_common
        )

        set_target_properties(${_tgt} PROPERTIES
            POSITION_INDEPENDENT_CODE ON
            CXX_STANDARD 20
            CXX_STANDARD_REQUIRED ON
        )
    endforeach()

    if(TETHER_BUILD_SHARED_LIBS)
        add_library(tether_ethercat_slave ALIAS tether_ethercat_slave_shared)
        add_library(tether::ethercat_slave ALIAS tether_ethercat_slave_shared)
    elseif(TETHER_BUILD_STATIC_LIBS)
        add_library(tether_ethercat_slave ALIAS tether_ethercat_slave_static)
        add_library(tether::ethercat_slave ALIAS tether_ethercat_slave_static)
    endif()

    set(TETHER_ETHERCAT_SLAVE_LIBRARY tether_ethercat_slave)
    set(TETHER_ETHERCAT_SLAVE_TARGETS ${_variants})
else()
    message(STATUS "No EtherCAT slave sources found, skipping tether_ethercat_slave")
    set(TETHER_ETHERCAT_SLAVE_LIBRARY "")
    set(TETHER_ETHERCAT_SLAVE_TARGETS "")
endif()
