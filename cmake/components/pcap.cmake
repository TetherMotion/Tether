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

# Create the pcap library
add_library(tether_pcap STATIC ${TETHER_PCAP_SOURCES})
add_library(tether::pcap ALIAS tether_pcap)

target_include_directories(tether_pcap
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_pcap
    PUBLIC tether_common
)

set_target_properties(tether_pcap PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_PCAP_LIBRARY tether_pcap)
