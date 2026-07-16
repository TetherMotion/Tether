# Component: tether_esi
# ESI (EtherCAT Slave Information) XML parser — an offline tooling utility.
# Separated from tether_ethercat_common to keep tinyxml2 out of the
# runtime shared library.

set(TETHER_ESI_SOURCES
    ${TETHER_ROOT}/src/ethercat/ESIParser.cpp
)

# Fetch tinyxml2 if not found on the system
find_package(TinyXML2 QUIET)
if(NOT TinyXML2_FOUND)
    include(FetchContent)
    FetchContent_Declare(tinyxml2
        GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
        GIT_TAG 9.0.0)
    FetchContent_MakeAvailable(tinyxml2)
endif()

set(_esi_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_esi_shared SHARED ${TETHER_ESI_SOURCES})
    list(APPEND _esi_variants tether_esi_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_esi_static STATIC ${TETHER_ESI_SOURCES})
    list(APPEND _esi_variants tether_esi_static)
endif()

foreach(_tgt IN LISTS _esi_variants)
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
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_esi ALIAS tether_esi_shared)
    add_library(tether::esi ALIAS tether_esi_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_esi ALIAS tether_esi_static)
    add_library(tether::esi ALIAS tether_esi_static)
endif()

set(TETHER_ESI_LIBRARY tether_esi)
set(TETHER_ESI_TARGETS ${_esi_variants})
