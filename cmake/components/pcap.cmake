# Component: tether_pcap
# PcapNG packet logging library (header-only dependency on standard C++)

set(TETHER_PCAP_SOURCES
    ${TETHER_ROOT}/src/packetloggers/pcap/PCAPWriter.cpp
    ${TETHER_ROOT}/src/packetloggers/pcap/PCAPLogger.cpp
    ${TETHER_ROOT}/src/packetloggers/pcap/PCAPNGReader.cpp
)

set(TETHER_PCAP_HEADERS
    ${TETHER_ROOT}/include/tether/packetloggers/PacketLogger.hpp
    ${TETHER_ROOT}/include/tether/packetloggers/pcap/PCAPLoggerConfig.hpp
    ${TETHER_ROOT}/include/tether/packetloggers/pcap/PCAPWriter.hpp
    ${TETHER_ROOT}/include/tether/packetloggers/pcap/PCAPLogger.hpp
    ${TETHER_ROOT}/include/tether/packetloggers/pcap/PCAPNGReader.hpp
)

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_pcap_shared SHARED ${TETHER_PCAP_SOURCES})
    list(APPEND _variants tether_pcap_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_pcap_static STATIC ${TETHER_PCAP_SOURCES})
    list(APPEND _variants tether_pcap_static)
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

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_pcap ALIAS tether_pcap_shared)
    add_library(tether::pcap ALIAS tether_pcap_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_pcap ALIAS tether_pcap_static)
    add_library(tether::pcap ALIAS tether_pcap_static)
endif()

set(TETHER_PCAP_LIBRARY tether_pcap)
set(TETHER_PCAP_TARGETS ${_variants})
