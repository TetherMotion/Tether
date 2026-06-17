# Component: tether_ethercat_common
# Common EtherCAT types, definitions, and utilities used by both master and slave

set(TETHER_ETHERCAT_COMMON_SOURCES
    # Debug flags (must be in common so all EtherCAT libraries can link to it)
    ${TETHER_ROOT}/src/ethercat/DebugFlags.cpp

    # SII parser
    ${TETHER_ROOT}/src/ethercat/ESIParser.cpp

    # Stubs for host builds (only include when we are NOT building the full master)
)

if(NOT TETHER_BUILD_ETHERCAT_MASTER)
    list(APPEND TETHER_ETHERCAT_COMMON_SOURCES
        ${TETHER_ROOT}/src/ethercat/host_stubs.cpp
        ${TETHER_ROOT}/src/ethercat/pdo_stubs.cpp
    )
endif()
foreach(src ${TETHER_ETHERCAT_COMMON_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_ETHERCAT_COMMON_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_ethercat_common_shared SHARED ${TETHER_ETHERCAT_COMMON_SOURCES_FILTERED})
    list(APPEND _variants tether_ethercat_common_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_ethercat_common_static STATIC ${TETHER_ETHERCAT_COMMON_SOURCES_FILTERED})
    list(APPEND _variants tether_ethercat_common_static)
endif()

# If tinyxml2 is available or needed by ESI parsing, link it
find_package(TinyXML2 QUIET)
if(NOT TinyXML2_FOUND)
    include(FetchContent)
    FetchContent_Declare(tinyxml2 GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git GIT_TAG 9.0.0)
    FetchContent_MakeAvailable(tinyxml2)
endif()

foreach(_tgt IN LISTS _variants)
    if(TARGET tinyxml2)
        target_link_libraries(${_tgt} PUBLIC tinyxml2)
    elseif(TARGET tinyxml2::tinyxml2)
        target_link_libraries(${_tgt} PUBLIC tinyxml2::tinyxml2)
    elseif(TARGET TinyXML2::TinyXML2)
        target_link_libraries(${_tgt} PUBLIC TinyXML2::TinyXML2)
    endif()

    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/ethercat>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/fmmu>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/sii>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common tether_hal)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_ethercat_common ALIAS tether_ethercat_common_shared)
    add_library(tether::ethercat_common ALIAS tether_ethercat_common_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_ethercat_common ALIAS tether_ethercat_common_static)
    add_library(tether::ethercat_common ALIAS tether_ethercat_common_static)
endif()

set(TETHER_ETHERCAT_COMMON_LIBRARY tether_ethercat_common)
set(TETHER_ETHERCAT_COMMON_TARGETS ${_variants})
