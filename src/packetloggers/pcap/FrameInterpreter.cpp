// SPDX-License-Identifier: MIT

/**
 * @file FrameInterpreter.cpp
 * @brief Implementation of Ethernet/EtherCAT frame interpretation
 */

#include "packetloggers/pcap/FrameInterpreter.hpp"

#include <cstring>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

namespace {

inline uint16_t read16_be(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

inline uint32_t read32_be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

inline uint32_t read32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline uint16_t read16_le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
}

constexpr uint16_t kEtherTypeVlan       = 0x8100;
constexpr uint16_t kEtherTypeVlan8021ad = 0x88A8;  // Service-tag (QinQ outer)
constexpr uint16_t kEtherTypeIPv4       = 0x0800;
constexpr uint16_t kEtherTypeIPv6       = 0x86DD;
constexpr uint8_t  kIpProtocolUDP       = 0x11;
constexpr uint16_t kEtherCATOverUdpPort = 0x88A4; // 34980

constexpr uint16_t kLinkTypeNull       = 0;   // BSD loopback
constexpr uint16_t kLinkTypeEthernet   = 1;   // LINKTYPE_ETHERNET
constexpr uint16_t kLinkTypeRaw        = 101; // LINKTYPE_RAW (old)
constexpr uint16_t kLinkTypeLinuxSll   = 113; // LINKTYPE_LINUX_SLL
constexpr uint16_t kLinkTypeRaw228     = 228; // LINKTYPE_RAW (new)

} // anonymous namespace

//=============================================================================
// Interface lookup
//=============================================================================

uint8_t FrameInterpreter::interfaceFcsLen(uint32_t interfaceId) const {
    if (interfaceId < interfaces_.size()) {
        return interfaces_[interfaceId].fcsLen;
    }
    return 0;
}

uint16_t FrameInterpreter::interfaceLinkType(uint32_t interfaceId) const {
    if (interfaceId < interfaces_.size()) {
        return interfaces_[interfaceId].linkType;
    }
    return kLinkTypeEthernet;
}

//=============================================================================
// Frame interpretation dispatch
//=============================================================================

void FrameInterpreter::interpretFrameByLinkType(const uint8_t* data, size_t length,
                                                 InterpretedFrame& frame) const {
    frame.linkType = interfaceLinkType(frame.interfaceId);
    switch (frame.linkType) {
        case kLinkTypeEthernet:
            interpretEthernetFrame(data, length, frame);
            break;
        case kLinkTypeLinuxSll:
            interpretLinuxSllFrame(data, length, frame);
            break;
        case kLinkTypeRaw:
        case kLinkTypeRaw228:
            interpretRawIpFrame(data, length, frame);
            break;
        case kLinkTypeNull:
            interpretNullFrame(data, length, frame);
            break;
        default:
            // Unknown link type: leave frameData as-is, do not interpret.
            break;
    }
}

//=============================================================================
// Ethernet frame interpretation
//=============================================================================

bool FrameInterpreter::interpretEthernetFrame(const uint8_t* data, size_t length,
                                               InterpretedFrame& frame) const {
    if (length < sizeof(EtherCAT::EthernetHeader)) {
        return false;
    }

    EtherCAT::EthernetHeader ethHdr;
    std::memcpy(&ethHdr, data, sizeof(ethHdr));
    std::memcpy(frame.dstMac.data(), ethHdr.dst, 6);
    std::memcpy(frame.srcMac.data(), ethHdr.src, 6);

    uint16_t etherType = read16_be(reinterpret_cast<const uint8_t*>(&ethHdr.etherType_be));
    size_t payloadOffset = sizeof(EtherCAT::EthernetHeader);

    // Handle 802.1Q / 802.1ad VLAN tag(s).  The outer EtherType field already
    // held the TPID, so payloadOffset points at the TCI word.
    while ((etherType == kEtherTypeVlan || etherType == kEtherTypeVlan8021ad) &&
           payloadOffset + 4 <= length) {
        uint16_t tci = read16_be(data + payloadOffset);
        uint16_t inner = read16_be(data + payloadOffset + 2);
        frame.vlanId = tci & 0x0FFF;
        frame.vlanPcp = static_cast<uint8_t>((tci >> 13) & 0x07);
        frame.vlanDei = ((tci >> 12) & 0x01) != 0;
        etherType = inner;
        payloadOffset += 4;
    }

    interpretPayload(etherType, data, length, payloadOffset, frame);
    return true; // parsed but not necessarily EtherCAT
}

//=============================================================================
// Linux SLL frame interpretation
//=============================================================================

void FrameInterpreter::interpretLinuxSllFrame(const uint8_t* data, size_t length,
                                               InterpretedFrame& frame) const {
    // SLL header (16 bytes):
    //   0: packet type (2 bytes, BE)
    //   2: ARPHRD type (2 bytes, BE)
    //   4: address length (2 bytes, BE)
    //   6: address (8 bytes)
    //  14: protocol / EtherType (2 bytes, BE)
    if (length < 16) return;

    uint16_t etherType = read16_be(data + 14);
    // The 8-byte address field holds the source MAC (padded/truncated).
    std::memcpy(frame.srcMac.data(), data + 6, 6);
    interpretPayload(etherType, data, length, 16, frame);
}

//=============================================================================
// Raw IP frame interpretation
//=============================================================================

void FrameInterpreter::interpretRawIpFrame(const uint8_t* data, size_t length,
                                            InterpretedFrame& frame) const {
    if (length < 1) return;
    uint8_t version = (data[0] >> 4) & 0x0F;
    if (version == 4) {
        frame.innerEtherType = kEtherTypeIPv4;
        parseEtherCATOverUDP(data, length, 0, frame);
    } else if (version == 6) {
        frame.innerEtherType = kEtherTypeIPv6;
        parseEtherCATOverUDP(data, length, 0, frame);
    }
}

//=============================================================================
// BSD loopback (null) frame interpretation
//=============================================================================

void FrameInterpreter::interpretNullFrame(const uint8_t* data, size_t length,
                                           InterpretedFrame& frame) const {
    // BSD loopback header: 4-byte address family (host byte order on the
    // capturing machine; on a little-endian host AF_INET=2, AF_INET6=30).
    if (length < 4) return;
    uint32_t family = read32_le(data);
    if (family == 2 || family == 0x02000000u) {
        frame.innerEtherType = kEtherTypeIPv4;
        parseEtherCATOverUDP(data, length, 4, frame);
    } else if (family == 30 || family == 0x1E000000u ||
               family == 24 || family == 0x18000000u) { // AF_INET6 on BSDs varies
        frame.innerEtherType = kEtherTypeIPv6;
        parseEtherCATOverUDP(data, length, 4, frame);
    }
}

//=============================================================================
// Payload interpretation (post-EtherType)
//=============================================================================

void FrameInterpreter::interpretPayload(uint16_t etherType, const uint8_t* data,
                                         size_t length, size_t payloadOffset,
                                         InterpretedFrame& frame) const {
    frame.innerEtherType = etherType;

    // Direct EtherCAT via EtherType 0x88A4.
    if (etherType == EtherCAT::kEtherTypeEtherCAT) {
        frame.isEtherCAT = true;
        parseEtherCATDatagrams(data, length, payloadOffset, frame);
        return;
    }

    // EtherCAT-over-UDP encapsulation: IPv4 or IPv6 -> UDP ->
    // dst port 34980 (0x88A4) -> EtherCAT frame as UDP payload.
    if (etherType == kEtherTypeIPv4 || etherType == kEtherTypeIPv6) {
        parseEtherCATOverUDP(data, length, payloadOffset, frame);
    }
}

//=============================================================================
// EtherCAT-over-UDP parsing
//=============================================================================

void FrameInterpreter::parseEtherCATOverUDP(const uint8_t* data, size_t length,
                                             size_t ipOffset,
                                             InterpretedFrame& frame) const {
    if (ipOffset + 1 > length) return;

    const uint8_t* ip = data + ipOffset;
    uint8_t version = (ip[0] >> 4) & 0x0F;
    size_t udpOffset = 0;

    if (version == 4) {
        // Minimum IPv4 header is 20 bytes.
        if (ipOffset + 20 > length) return;
        uint8_t versionIhl = ip[0];
        uint8_t ihl = (versionIhl & 0x0F) * 4;
        if (ihl < 20 || ipOffset + ihl > length) return;

        uint8_t protocol = ip[9];
        if (protocol != kIpProtocolUDP) return; // not UDP

        frame.ipVersion = 4;
        frame.srcIp = read32_be(ip + 12);
        frame.dstIp = read32_be(ip + 16);
        udpOffset = ipOffset + ihl;
    } else if (version == 6) {
        // Minimum IPv6 header is 40 bytes.
        if (ipOffset + 40 > length) return;
        uint8_t nextHeader = ip[6];
        if (nextHeader != kIpProtocolUDP) return; // not UDP

        frame.ipVersion = 6;
        std::memcpy(frame.srcIpv6.data(), ip + 8, 16);
        std::memcpy(frame.dstIpv6.data(), ip + 24, 16);
        udpOffset = ipOffset + 40;
    } else {
        return; // not IP
    }

    if (udpOffset + 8 > length) return;

    const uint8_t* udp = data + udpOffset;
    frame.srcPort = read16_be(udp + 0);
    frame.dstPort = read16_be(udp + 2);
    uint16_t udpLen = read16_be(udp + 4);
    if (udpLen < 8) return;

    // EtherCAT-over-UDP uses destination port 34980 (0x88A4).
    if (frame.dstPort != kEtherCATOverUdpPort) return;

    size_t ecatOffset = udpOffset + 8;
    size_t ecatAvail = length - ecatOffset;
    // Clamp to UDP length minus header.
    size_t udpPayloadLen = udpLen - 8;
    if (udpPayloadLen > ecatAvail) udpPayloadLen = ecatAvail;
    if (udpPayloadLen < sizeof(EtherCAT::FrameHeader)) return;

    frame.isEtherCAT = true;
    frame.isEtherCATOverUDP = true;
    parseEtherCATDatagrams(data, ecatOffset + udpPayloadLen, ecatOffset, frame);
}

//=============================================================================
// EtherCAT datagram parsing
//=============================================================================

void FrameInterpreter::parseEtherCATDatagrams(const uint8_t* data, size_t dataEnd,
                                               size_t ecatOffset,
                                               InterpretedFrame& frame) const {
    if (ecatOffset + sizeof(EtherCAT::FrameHeader) > dataEnd) {
        return;
    }

    const uint8_t* ecatHdrBytes = data + ecatOffset;
    frame.ecatFrameLength = static_cast<uint16_t>(ecatHdrBytes[0]) |
                            static_cast<uint16_t>((ecatHdrBytes[1] & 0x07) << 8);
    frame.ecatFrameType = static_cast<uint8_t>((ecatHdrBytes[1] >> 4) & 0x0F);

    size_t datagramOffset = ecatOffset + sizeof(EtherCAT::FrameHeader);
    size_t remaining = frame.ecatFrameLength;

    while (remaining >= sizeof(EtherCAT::DatagramHeader) + sizeof(uint16_t)) {
        if (datagramOffset + sizeof(EtherCAT::DatagramHeader) > dataEnd) {
            break;
        }

        const uint8_t* dg = data + datagramOffset;
        EtherCATDatagramInfo info;
        info.cmd = static_cast<EtherCAT::Command>(dg[0]);
        info.idx = dg[1];
        info.adp = read16_le(dg + 2);
        info.ado = read16_le(dg + 4);
        uint16_t lenFlags = read16_le(dg + 6);
        info.dataLength = lenFlags & 0x07FF;
        info.more = (lenFlags & 0x8000) != 0;
        info.circulating = (lenFlags & 0x4000) != 0;
        info.irq = read16_le(dg + 8);

        size_t dgTotalSize = sizeof(EtherCAT::DatagramHeader) + info.dataLength + sizeof(uint16_t);
        if (datagramOffset + dgTotalSize > dataEnd || dgTotalSize > remaining) {
            break;
        }

        const uint8_t* payload = dg + sizeof(EtherCAT::DatagramHeader);
        info.data.assign(payload, payload + info.dataLength);
        info.wkc = read16_le(payload + info.dataLength);

        frame.datagrams.push_back(std::move(info));

        datagramOffset += dgTotalSize;
        remaining -= dgTotalSize;
    }
}

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
