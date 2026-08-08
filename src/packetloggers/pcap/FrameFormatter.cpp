// SPDX-License-Identifier: MIT

/**
 * @file FrameFormatter.cpp
 * @brief Implementation of frame formatting functions extracted from PCAPNGReader.
 */

#include "packetloggers/pcap/FrameFormatter.hpp"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

namespace {

inline std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

const char* directionString(PacketDirection dir) {
    switch (dir) {
        case PacketDirection::Inbound:  return "RX";
        case PacketDirection::Outbound: return "TX";
        case PacketDirection::Loopback: return "LOOPBACK";
        default:                        return "?";
    }
}

} // anonymous namespace

// ============================================================================
// Formatting helpers
// ============================================================================

std::string macToString(const std::array<uint8_t, 6>& mac) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < mac.size(); ++i) {
        if (i > 0) oss << ':';
        oss << std::setw(2) << static_cast<int>(mac[i]);
    }
    return oss.str();
}

std::string ipToString(uint32_t ip) {
    std::ostringstream oss;
    oss << ((ip >> 24) & 0xFF) << "."
        << ((ip >> 16) & 0xFF) << "."
        << ((ip >> 8) & 0xFF) << "."
        << (ip & 0xFF);
    return oss.str();
}

std::string bytesToHex(const uint8_t* data, size_t length, const std::string& separator) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) oss << separator;
        oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::string formatInterpretedFrame(const InterpretedFrame& frame, bool verbose, size_t maxDataBytes) {
    std::ostringstream oss;
    oss << "[" << directionString(frame.direction) << "] "
        << "ts=" << frame.timestampNs << " ns"
        << "  iface=" << frame.interfaceId
        << "  cap=" << frame.capturedLength
        << "  orig=" << frame.originalLength;
    if (frame.dropCount > 0) {
        oss << "  drops=" << frame.dropCount;
    }
    if (!frame.comment.empty()) {
        oss << "  comment=\"" << frame.comment << "\"";
    }
    oss << "\n";

    oss << "  Dst: " << macToString(frame.dstMac)
        << "  Src: " << macToString(frame.srcMac);
    if (frame.vlanId.has_value()) {
        oss << "  VLAN: " << *frame.vlanId
            << " PCP: " << static_cast<int>(frame.vlanPcp)
            << " DEI: " << (frame.vlanDei ? "1" : "0");
    }
    oss << "  EtherType: 0x" << std::hex << std::setfill('0') << std::setw(4) << frame.innerEtherType
        << std::dec << "\n";

    if (frame.isEtherCATOverUDP) {
        oss << "  EtherCAT-over-UDP  "
            << ipToString(frame.srcIp) << ":" << frame.srcPort
            << " -> " << ipToString(frame.dstIp) << ":" << frame.dstPort
            << "\n";
    }

    if (frame.isEtherCAT) {
        oss << "  EtherCAT frame  length=" << frame.ecatFrameLength
            << "  type=" << static_cast<int>(frame.ecatFrameType);
        if (frame.slaveAddress != 0) {
            oss << "  slave=" << frame.slaveAddress;
        }
        if (frame.isProcessData) {
            oss << "  PDO";
        }
        if (frame.workingCounter != 0) {
            oss << "  wc=" << static_cast<int>(frame.workingCounter);
        }
        oss << "\n";

        for (size_t i = 0; i < frame.datagrams.size(); ++i) {
            const auto& dg = frame.datagrams[i];
            oss << "    DG" << i << ": "
                << EtherCAT::commandToString(dg.cmd)
                << " idx=" << static_cast<int>(dg.idx);

            switch (dg.cmd) {
                case EtherCAT::Command::LRD:
                case EtherCAT::Command::LWR:
                case EtherCAT::Command::LRW:
                    oss << " logAddr=0x" << std::hex << std::setfill('0') << std::setw(8)
                        << dg.logicalAddress() << std::dec;
                    break;
                default:
                    oss << " adp=" << dg.adp << " ado=0x" << std::hex << std::setfill('0')
                        << std::setw(4) << dg.ado << std::dec;
                    break;
            }

            oss << " len=" << dg.dataLength
                << " wkc=" << dg.wkc;
            if (dg.irq != 0) {
                oss << " irq=0x" << std::hex << std::setfill('0')
                    << std::setw(4) << dg.irq << std::dec;
            }
            oss << (dg.more ? " [M]" : "")
                << (dg.circulating ? " [C]" : "")
                << "\n";

            if (verbose && !dg.data.empty()) {
                size_t dumpLen = maxDataBytes > 0 ? std::min(dg.data.size(), maxDataBytes) : dg.data.size();
                oss << "      Data: " << bytesToHex(dg.data.data(), dumpLen);
                if (maxDataBytes > 0 && dg.data.size() > maxDataBytes) {
                    oss << " ... (" << (dg.data.size() - maxDataBytes) << " more bytes)";
                }
                oss << "\n";
            }
        }
    }

    return oss.str();
}

std::string frameToJson(const InterpretedFrame& frame) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"timestampNs\": " << frame.timestampNs << ",\n";
    oss << "  \"interfaceId\": " << frame.interfaceId << ",\n";
    oss << "  \"capturedLength\": " << frame.capturedLength << ",\n";
    oss << "  \"originalLength\": " << frame.originalLength << ",\n";
    oss << "  \"direction\": \"" << directionString(frame.direction) << "\",\n";
    oss << "  \"packetFlags\": " << frame.packetFlags << ",\n";
    oss << "  \"dropCount\": " << frame.dropCount << ",\n";
    oss << "  \"comment\": \"" << jsonEscape(frame.comment) << "\",\n";
    oss << "  \"dstMac\": \"" << macToString(frame.dstMac) << "\",\n";
    oss << "  \"srcMac\": \"" << macToString(frame.srcMac) << "\",\n";
    if (frame.vlanId.has_value()) {
        oss << "  \"vlanId\": " << *frame.vlanId << ",\n";
        oss << "  \"vlanPcp\": " << static_cast<int>(frame.vlanPcp) << ",\n";
        oss << "  \"vlanDei\": " << (frame.vlanDei ? "true" : "false") << ",\n";
    }
    oss << "  \"innerEtherType\": " << frame.innerEtherType << ",\n";
    oss << "  \"isEtherCAT\": " << (frame.isEtherCAT ? "true" : "false") << ",\n";
    oss << "  \"isEtherCATOverUDP\": " << (frame.isEtherCATOverUDP ? "true" : "false") << ",\n";
    if (frame.isEtherCATOverUDP) {
        oss << "  \"srcIp\": \"" << ipToString(frame.srcIp) << "\",\n";
        oss << "  \"dstIp\": \"" << ipToString(frame.dstIp) << "\",\n";
        oss << "  \"srcPort\": " << frame.srcPort << ",\n";
        oss << "  \"dstPort\": " << frame.dstPort << ",\n";
    }
    if (frame.isEtherCAT) {
        oss << "  \"ecatFrameLength\": " << frame.ecatFrameLength << ",\n";
        oss << "  \"ecatFrameType\": " << static_cast<int>(frame.ecatFrameType) << ",\n";
        oss << "  \"slaveAddress\": " << frame.slaveAddress << ",\n";
        oss << "  \"isProcessData\": " << (frame.isProcessData ? "true" : "false") << ",\n";
        oss << "  \"workingCounter\": " << static_cast<int>(frame.workingCounter) << ",\n";
        oss << "  \"datagrams\": [\n";
        for (size_t i = 0; i < frame.datagrams.size(); ++i) {
            const auto& dg = frame.datagrams[i];
            oss << "    {\n";
            oss << "      \"cmd\": \"" << EtherCAT::commandToString(dg.cmd) << "\",\n";
            oss << "      \"idx\": " << static_cast<int>(dg.idx) << ",\n";
            oss << "      \"adp\": " << dg.adp << ",\n";
            oss << "      \"ado\": " << dg.ado << ",\n";
            oss << "      \"dataLength\": " << dg.dataLength << ",\n";
            oss << "      \"irq\": " << dg.irq << ",\n";
            oss << "      \"wkc\": " << dg.wkc << ",\n";
            oss << "      \"more\": " << (dg.more ? "true" : "false") << ",\n";
            oss << "      \"circulating\": " << (dg.circulating ? "true" : "false") << ",\n";
            oss << "      \"data\": \"" << bytesToHex(dg.data.data(), dg.data.size(), "") << "\"\n";
            oss << "    }";
            if (i + 1 < frame.datagrams.size()) oss << ",";
            oss << "\n";
        }
        oss << "  ]\n";
    } else {
        oss << "  \"frameData\": \"" << bytesToHex(frame.frameData.data(), frame.frameData.size(), "") << "\"\n";
    }
    oss << "}\n";
    return oss.str();
}

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
