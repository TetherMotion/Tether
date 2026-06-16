# Component: tether_ethercat_common
# Common EtherCAT types, definitions, and utilities used by both master and slave

set(TETHER_ETHERCAT_COMMON_SOURCES
    # SII parser
    ${TETHER_ROOT}/src/sii/SIIReader.cpp
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

# Create the ethercat common library
add_library(tether_ethercat_common STATIC ${TETHER_ETHERCAT_COMMON_SOURCES_FILTERED})
add_library(tether::ethercat_common ALIAS tether_ethercat_common)

# If tinyxml2 is available or needed by ESI parsing, link it
find_package(TinyXML2 QUIET)
if(NOT TinyXML2_FOUND)
    include(FetchContent)
    FetchContent_Declare(tinyxml2 GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git GIT_TAG 9.0.0)
    FetchContent_MakeAvailable(tinyxml2)
endif()

if(TARGET tinyxml2)
    target_link_libraries(tether_ethercat_common PUBLIC tinyxml2)
elseif(TARGET tinyxml2::tinyxml2)
    target_link_libraries(tether_ethercat_common PUBLIC tinyxml2::tinyxml2)
elseif(TARGET TinyXML2::TinyXML2)
    target_link_libraries(tether_ethercat_common PUBLIC TinyXML2::TinyXML2)
endif()

target_include_directories(tether_ethercat_common
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

target_link_libraries(tether_ethercat_common
    PUBLIC tether_common tether_hal
)

set_target_properties(tether_ethercat_common PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_ETHERCAT_COMMON_LIBRARY tether_ethercat_common)
