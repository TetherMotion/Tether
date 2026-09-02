// SPDX-License-Identifier: MIT
/**
 * @file PacketDebugger.cpp
 * @brief Stateless EtherCAT packet debug printing utilities
 *
 * @details
 * Extracted from Master_transport.cpp.  These functions parse and
 * pretty-print raw EtherCAT frames for diagnostic logging.  They are
 * pure/stateless and depend only on the frame buffer and the EtherCAT
 * wire-format type definitions.
 *
 * @internal
 */

#include "PacketDebugger.hpp"

#include "tether/ethercat/Types.hpp"        // EthernetHeader, FrameHeader, DatagramHeader, commandToString
#include "tether/ethercat/TetherConfig.hpp" // TETHER_ENABLE_UDP_ENCAPSULATION
#include "tether/packet_interpreters/CoE.hpp"
#include "tether/platform/Platform.hpp"     // TETHER_LOGI

#include <cstdio>
#include <string>
#include <vector>

#include "RawWireFormat.hpp"                // Raw::bswap16, le16_to_host, UDP encap types

namespace EtherCAT {
namespace PacketDebugger {

using namespace Raw;  // for le16_to_host, bswap16, etc.

// ============================================================================
// EtherType lookup
// ============================================================================

const char* etherTypeToString(uint16_t ether_type)
{
    switch (ether_type) {
        case 0x0800: return "IPv4";
        case 0x0806: return "ARP";
        case 0x0842: return "WoL";
        case 0x22F3: return "IETF TRILL";
        case 0x22EA: return "Stream Reservation";
        case 0x6003: return "DECnet Phase IV";
        case 0x8035: return "RARP";
        case 0x809B: return "AppleTalk";
        case 0x80F3: return "AARP";
        case 0x8100: return "VLAN (802.1Q)";
        case 0x8204: return "QNX Qnet";
        case 0x86DD: return "IPv6";
        case 0x8808: return "Ethernet Flow Control";
        case 0x8809: return "Ethernet Slow Protocols (LACP)";
        case 0x8819: return "CobraNet";
        case 0x8847: return "MPLS unicast";
        case 0x8848: return "MPLS multicast";
        case 0x8863: return "PPPoE Discovery";
        case 0x8864: return "PPPoE Session";
        case 0x887B: return "HomePlug 1.0 MME";
        case 0x888E: return "EAPoL (802.1X)";
        case 0x8892: return "PROFINET";
        case 0x889A: return "HyperSCSI";
        case 0x88A2: return "ATAoE";
        case 0x88A4: return "EtherCAT";
        case 0x88A8: return "Provider Bridging (802.1ad)";
        case 0x88AB: return "EtherCAT Automation Protocol";
        case 0x88B8: return "GOOSE (IEC 61850)";
        case 0x88B9: return "GSE Management";
        case 0x88BA: return "SV (IEC 61850)";
        case 0x88BF: return "MikroTik RoMON";
        case 0x88CC: return "LLDP";
        case 0x88CD: return "SERCOS III";
        case 0x88E1: return "HomePlug AV MME";
        case 0x88E3: return "MRP (IEC 62439-2)";
        case 0x88E5: return "MACsec (802.1AE)";
        case 0x88E7: return "PBB (802.1ah)";
        case 0x88F7: return "PTP (IEEE 1588)";
        case 0x88F8: return "NC-SI";
        case 0x88FB: return "PRP (IEC 62439-3)";
        case 0x8902: return "IEEE 802.1ag CFM";
        case 0x8906: return "FCoE";
        case 0x8914: return "FCoE Initialization";
        case 0x8915: return "RoCE";
        case 0x891D: return "TTE";
        case 0x892F: return "HSR (IEC 62439-3)";
        case 0x8932: return "802.1Qbj MVRP";
        case 0x9000: return "Loopback";
        case 0x9100: return "Q-in-Q";
        default: return nullptr;
    }
}

// ============================================================================
// Frame pretty-printer
// ============================================================================

void printEtherCATFrame(const uint8_t* frame, size_t length, bool is_tx, bool print_ethernet)
{
    const char* dir = is_tx ? "TX" : "RX";

    if (print_ethernet && length >= sizeof(EtherCAT::EthernetHeader)) {
        const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
        TETHER_LOGI("ec_pkt", "[{}] Ethernet: dst={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} src={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} etherType=0x{:04X}",
                    dir,
                    eth->dst[0], eth->dst[1], eth->dst[2], eth->dst[3], eth->dst[4], eth->dst[5],
                    eth->src[0], eth->src[1], eth->src[2], eth->src[3], eth->src[4], eth->src[5],
                    bswap16(eth->etherType_be));
    }

    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader)) {
        TETHER_LOGI("ec_pkt", "[{}] Frame too short for EtherCAT header ({} bytes)",
                    dir, static_cast<unsigned>(length));
        return;
    }

    // Determine the EtherCAT frame offset — handle both direct EtherCAT and
    // EtherCAT-over-UDP encapsulation.
    size_t ecat_offset = sizeof(EtherCAT::EthernetHeader);
    {
        const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
        const uint16_t ether_type = bswap16(eth->etherType_be);
#if TETHER_ENABLE_UDP_ENCAPSULATION
        if (ether_type == kEtherTypeIPv4) {
            const size_t ip_off = sizeof(EtherCAT::EthernetHeader);
            if (ip_off + sizeof(IPv4Header) > length) return;
            const auto* ip = reinterpret_cast<const IPv4Header*>(frame + ip_off);
            const uint8_t ihl = (ip->version_ihl & 0x0F) * 4;
            if (ip_off + ihl + sizeof(UDPHeader) > length) return;
            const auto* udp = reinterpret_cast<const UDPHeader*>(frame + ip_off + ihl);
            TETHER_LOGI("ec_pkt", "[{}] EtherCAT-over-UDP: src_port={} dst_port={}",
                        dir, bswap16(udp->src_port_be), bswap16(udp->dst_port_be));
            ecat_offset = ip_off + ihl + sizeof(UDPHeader);
        }
#endif // TETHER_ENABLE_UDP_ENCAPSULATION
    }

    if (length < ecat_offset + sizeof(EtherCAT::FrameHeader)) {
        TETHER_LOGI("ec_pkt", "[{}] Frame too short for EtherCAT header after encap ({} bytes)",
                    dir, static_cast<unsigned>(length));
        return;
    }

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(frame + ecat_offset);
    const uint16_t ec_raw = le16_to_host(ec_hdr->raw_le);
    const uint16_t ec_len = ec_raw & 0x07FFu;
    const uint16_t ec_type = (ec_raw >> 12) & 0x0Fu;

    struct DgInfo {
        const char* cmd;
        uint8_t idx;
        uint16_t adp;
        uint16_t ado;
        uint16_t datalen;
        uint16_t wkc;
        bool more;
        bool circulating;
        bool has_data;
        bool all_zero;
        size_t dump_len;
        std::string hex;
        std::string data;
        std::vector<std::string> coe_lines;
    };
    std::vector<DgInfo> dgs;
    dgs.reserve(8);

    size_t offset = ecat_offset + sizeof(EtherCAT::FrameHeader);
    size_t remaining = ec_len;
    uint8_t dg_idx = 0;
    bool can_collapse = true;

    while (remaining >= sizeof(EtherCAT::DatagramHeader) &&
           offset + sizeof(EtherCAT::DatagramHeader) <= length) {
        const auto* dg = reinterpret_cast<const EtherCAT::DatagramHeader*>(frame + offset);
        const uint16_t dg_len_flags = le16_to_host(dg->lenFlags_le);
        const uint16_t datalen = dg_len_flags & 0x07FFu;
        const bool more = (dg_len_flags & 0x8000u) != 0;
        const bool circulating = (dg_len_flags & 0x4000u) != 0;
        const uint16_t adp = le16_to_host(dg->adp_le);
        const uint16_t ado = le16_to_host(dg->ado_le);

        const size_t data_offset = offset + sizeof(EtherCAT::DatagramHeader);
        const size_t wkc_offset = data_offset + datalen;
        uint16_t wkc = 0;
        if (length >= wkc_offset + sizeof(uint16_t)) {
            wkc = le16_to_host(*reinterpret_cast<const uint16_t*>(frame + wkc_offset));
        }

        DgInfo info;
        info.cmd = commandToString(dg->cmd);
        info.idx = dg->idx;
        info.adp = adp;
        info.ado = ado;
        info.datalen = datalen;
        info.wkc = wkc;
        info.more = more;
        info.circulating = circulating;
        info.has_data = (datalen > 0 && length >= data_offset + datalen);
        info.all_zero = false;
        info.dump_len = 0;

        if (info.has_data) {
            info.all_zero = true;
            for (size_t i = 0; i < datalen; ++i) {
                if (frame[data_offset + i] != 0) {
                    info.all_zero = false;
                    break;
                }
            }
            if (!info.all_zero) {
                constexpr size_t kMaxHexDump = 64;
                const size_t dump_len = (datalen < kMaxHexDump) ? datalen : kMaxHexDump;
                info.dump_len = dump_len;
                info.hex.reserve(dump_len * 3 + 4);
                info.data.reserve(dump_len * 3);
                for (size_t i = 0; i < dump_len; ++i) {
                    char byte_str[4];
                    char byte_no_space[3];
                    std::snprintf(byte_str, sizeof(byte_str), "%02X ", frame[data_offset + i]);
                    std::snprintf(byte_no_space, sizeof(byte_no_space), "%02X", frame[data_offset + i]);
                    info.hex += byte_str;
                    if (i > 0) info.data += ' ';
                    info.data += byte_no_space;
                }
                if (datalen > kMaxHexDump) {
                    info.hex += "...";
                }
                // If payload looks like a CoE mailbox packet, interpret it
                if (datalen >= 8) {
                    const uint8_t* mbx_data = frame + data_offset;
                    const uint16_t mbx_len = mbx_data[0] | (static_cast<uint16_t>(mbx_data[1]) << 8);
                    const uint8_t mbx_type = mbx_data[5] & 0x0F;
                    if (mbx_type == 0x03 && mbx_len >= 2 && mbx_len + 6 <= datalen) {
                        EtherCAT::PacketInterpreters::CoEPacketInterpreter interp(mbx_data, datalen);
                        std::string coe_str = interp.toString();
                        size_t line_start = 0;
                        while (line_start < coe_str.size()) {
                            size_t line_end = coe_str.find('\n', line_start);
                            if (line_end == std::string::npos) line_end = coe_str.size();
                            if (line_end > line_start) {
                                info.coe_lines.emplace_back(coe_str.substr(line_start, line_end - line_start));
                            }
                            line_start = line_end + 1;
                        }
                    }
                }
            }
        }

        if (!info.has_data || (!info.all_zero && info.datalen > 16)) {
            can_collapse = false;
        }

        dgs.push_back(info);

        const size_t dg_total = sizeof(EtherCAT::DatagramHeader) + datalen + sizeof(uint16_t);
        if (remaining < dg_total) {
            can_collapse = false;
            break;
        }
        remaining -= dg_total;
        offset += dg_total;
        dg_idx++;

        if (!more) break;
    }

    if (can_collapse && !dgs.empty()) {
        std::string line;
        line.reserve(256);
        line += "[";
        line += dir;
        line += "] Frame: length=";
        line += std::to_string(static_cast<unsigned>(ec_len));
        line += " type=";
        line += std::to_string(static_cast<unsigned>(ec_type));
        const bool single = (dgs.size() == 1);
        for (size_t i = 0; i < dgs.size(); ++i) {
            const auto& dg = dgs[i];
            if (!single) {
                char prefix[32];
                std::snprintf(prefix, sizeof(prefix), " | DG%u: ", static_cast<unsigned>(i));
                line += prefix;
            } else {
                line += " ";
            }
            line += dg.cmd;
            char fields[128];
            std::snprintf(fields, sizeof(fields), " idx=0x%02X adp=0x%04X ado=0x%04X len=%u wkc=%u more=%s circ=%s",
                          static_cast<unsigned>(dg.idx), dg.adp, dg.ado,
                          static_cast<unsigned>(dg.datalen), dg.wkc,
                          dg.more ? "yes" : "no", dg.circulating ? "yes" : "no");
            line += fields;
            if (dg.has_data) {
                if (dg.all_zero) {
                    char dbuf[32];
                    std::snprintf(dbuf, sizeof(dbuf), " Data=%u zero", static_cast<unsigned>(dg.datalen));
                    line += dbuf;
                } else {
                    line += " Data=";
                    line += dg.data;
                }
            }
        }
        TETHER_LOGI("ec_pkt", "{}", line.c_str());
    } else {
        TETHER_LOGI("ec_pkt", "[{}] EtherCAT Frame: length={} type={}", dir, ec_len, ec_type);
        for (size_t i = 0; i < dgs.size(); ++i) {
            const auto& dg = dgs[i];
            std::string dgram_line;
            dgram_line += "[";
            dgram_line += dir;
            dgram_line += "]   Datagram[";
            dgram_line += std::to_string(static_cast<unsigned>(i));
            dgram_line += "]: ";
            dgram_line += dg.cmd;
            char fields[128];
            std::snprintf(fields, sizeof(fields), " idx=0x%02X adp=0x%04X ado=0x%04X len=%u wkc=%u more=%s circulating=%s",
                          static_cast<unsigned>(dg.idx), dg.adp, dg.ado,
                          static_cast<unsigned>(dg.datalen), dg.wkc,
                          dg.more ? "yes" : "no", dg.circulating ? "yes" : "no");
            dgram_line += fields;
            bool inline_data = (dg.has_data && (dg.all_zero || dg.datalen <= 16));
            if (inline_data) {
                if (dg.all_zero) {
                    char dbuf[32];
                    std::snprintf(dbuf, sizeof(dbuf), " Data=%u zero", static_cast<unsigned>(dg.datalen));
                    dgram_line += dbuf;
                } else {
                    dgram_line += " Data=";
                    dgram_line += dg.data;
                }
            }
            TETHER_LOGI("ec_pkt", "{}", dgram_line.c_str());
            if (dg.has_data && !inline_data) {
                TETHER_LOGI("ec_pkt", "[{}]     Data ({}/{} bytes): {}",
                            dir, static_cast<unsigned>(dg.dump_len), static_cast<unsigned>(dg.datalen), dg.hex.c_str());
                for (const auto& coe_line : dg.coe_lines) {
                    TETHER_LOGI("ec_pkt", "[{}]     CoE: {}", dir, coe_line.c_str());
                }
            }
        }
    }
}

} // namespace PacketDebugger
} // namespace EtherCAT
