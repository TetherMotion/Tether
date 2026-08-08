// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file FrameInterpreter.hpp
 * @brief Ethernet/EtherCAT frame interpretation extracted from PCAPNGReader
 *
 * @details
 * Encapsulates the frame-interpretation sub-responsibility of PCAPNGReader:
 *  - Ethernet frame parsing (MAC, EtherType, VLAN decapsulation)
 *  - Linux SLL frame parsing
 *  - Raw IP frame parsing
 *  - BSD loopback (null) frame parsing
 *  - EtherCAT-over-UDP encapsulation detection
 *  - EtherCAT datagram extraction
 *
 * The interpreter holds a pointer to the PCAPNG interface list so it can
 * look up the link type and FCS length for each interface.
 */

#include "packetloggers/pcap/PCAPNGReader.hpp" // InterpretedFrame, PCAPNGInterfaceInfo, EtherCATDatagramInfo

#include <cstdint>
#include <vector>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

class FrameInterpreter {
public:
    /// Construct an interpreter bound to an interface list.
    /// @param interfaces  Reference to the PCAPNGReader's interface vector.
    ///                    The caller must ensure this outlives the interpreter.
    explicit FrameInterpreter(const std::vector<PCAPNGInterfaceInfo>& interfaces)
        : interfaces_(interfaces) {}

    /// Dispatch frame interpretation based on the interface link type.
    void interpretFrameByLinkType(const uint8_t* data, size_t length,
                                  InterpretedFrame& frame) const;

    /// Interpret an Ethernet frame (LINKTYPE_ETHERNET=1).
    /// @return true if the frame was parsed (not necessarily EtherCAT).
    bool interpretEthernetFrame(const uint8_t* data, size_t length,
                                InterpretedFrame& frame) const;

    /// Interpret a Linux cooked-capture (SLL) frame (LINKTYPE_LINUX_SLL=113).
    void interpretLinuxSllFrame(const uint8_t* data, size_t length,
                                InterpretedFrame& frame) const;

    /// Interpret a raw IP frame (LINKTYPE_RAW=101/228, no link-layer header).
    void interpretRawIpFrame(const uint8_t* data, size_t length,
                             InterpretedFrame& frame) const;

    /// Interpret a BSD loopback frame (LINKTYPE_NULL=0): 4-byte family + IP.
    void interpretNullFrame(const uint8_t* data, size_t length,
                            InterpretedFrame& frame) const;

    /// Common post-EtherType payload interpretation (VLAN already stripped).
    void interpretPayload(uint16_t etherType, const uint8_t* data,
                          size_t length, size_t payloadOffset,
                          InterpretedFrame& frame) const;

    /// Parse EtherCAT datagrams starting at @p ecatOffset within @p data.
    /// @p dataEnd is the exclusive end boundary.
    void parseEtherCATDatagrams(const uint8_t* data, size_t dataEnd,
                                size_t ecatOffset,
                                InterpretedFrame& frame) const;

    /// Detect and parse EtherCAT-over-UDP encapsulation (UDP dst port 34980).
    /// @p ipOffset points to the start of the IP header within @p data.
    void parseEtherCATOverUDP(const uint8_t* data, size_t length,
                              size_t ipOffset,
                              InterpretedFrame& frame) const;

    /// @return FCS length (bytes) for the given interface, or 0 if unknown.
    uint8_t interfaceFcsLen(uint32_t interfaceId) const;

    /// @return LINKTYPE_* for the given interface, or LINKTYPE_ETHERNET if unknown.
    uint16_t interfaceLinkType(uint32_t interfaceId) const;

private:
    const std::vector<PCAPNGInterfaceInfo>& interfaces_;
};

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
