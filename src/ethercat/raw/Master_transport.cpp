/**
 * @file Master_transport.cpp
 * @brief Master — Low-level datagram transport, register I/O and packet debug
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/packet_interpreters/CoE.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

// TX retry constants
static constexpr int       kMaxTxRetries   = ECAT_TX_MAX_RETRIES;
static constexpr uint32_t  kTxRetryDelayUs = ECAT_TX_RETRY_DELAY_US;

static const char* TAG = "ethercat";

bool Master::sendDatagram(Command cmd, uint8_t idx,
                                  SlaveAddress slave_address, RegisterAddress register_address,
                                  const void* data, uint16_t datalen,
                                  bool roundtrip)
{
    Command routed_command = cmd;
    if (slave_address.isLogical()) {
        switch (cmd) {
            case Command::APRD: routed_command = Command::FPRD; break;
            case Command::APWR: routed_command = Command::FPWR; break;
            case Command::APRW: routed_command = Command::FPRW; break;
            default: break;
        }
    }

    return sendSingleDatagram(routed_command, idx, slave_address.raw(), register_address.raw(), data, datalen, roundtrip);
}

bool Master::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = 1;
    const uint16_t adp = slave_address.raw();
    const uint16_t ado = register_address.raw();
    if (apwr_cb_) return apwr_cb_(adp, ado, data, len, timeout_ms);
    if (ado == Raw::EC_REG_EEPCTL && !aprd_responses_.empty()) return true;

    const uint8_t idx = allocIdx();
    RxDatagram resp{};
    size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
    if (slot >= TransactionRouter::kNumSlots) return false;

    if (!sendDatagram(Command::APWR, idx, slave_address, register_address, data, len, true)) {
        packet_router_.cancelPreRegistered(slot);
        return false;
    }

    WaitResult result = waitForPreRegistered(slot, timeout_ms);
    last_wkc_.store(result.wkc, std::memory_order_relaxed);
    return result.success && result.wkc > 0;
}

bool Master::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                   uint16_t value)
{
    const uint16_t little_endian_value = Raw::host_to_le16(value);
    return writeRegister(slave_address, register_address, &little_endian_value, sizeof(little_endian_value), 200);
}

bool Master::readRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                  void* out, uint16_t len,
                                  unsigned int timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = 1;
    const uint16_t adp = slave_address.raw();
    const uint16_t ado = register_address.raw();
    // Debug: show whether a per-instance APRD test callback is present
    if (aprd_cb_) {
        TETHER_LOGD(TAG, "readRegister: using instance aprd_cb_");
        return aprd_cb_(adp, ado, out, len, timeout_ms);
    }

    // EEPROM status short-circuit for tests
    if (ado == 0x0502 && apwr_cb_) {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    }

    // Check queued test responses
    for (auto it = aprd_responses_.begin(); it != aprd_responses_.end(); ++it) {
        if (it->adp == adp && it->ado == ado) {
            AprdResponse resp = *it;
            aprd_responses_.erase(it);
            if (!resp.success) return false;
            if (out && !resp.data.empty()) {
                uint16_t copy_len =
                    static_cast<uint16_t>(std::min<size_t>(len, resp.data.size()));
                std::memcpy(out, resp.data.data(), copy_len);
            }
            return true;
        }
    }
    if (!aprd_responses_.empty()) {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    }

    const uint8_t idx = allocIdx();
    RxDatagram resp{};
    size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
    if (slot >= TransactionRouter::kNumSlots) return false;

    if (!sendDatagram(Command::APRD, idx, slave_address, register_address, nullptr, len, true)) {
        packet_router_.cancelPreRegistered(slot);
        return false;
    }

    WaitResult result = waitForPreRegistered(slot, timeout_ms);
    last_wkc_.store(result.wkc, std::memory_order_relaxed);
    if (!result.success) return false;

    resp.datalen = static_cast<uint16_t>(result.data_length);
    if (resp.datalen < len) return false;
    if (out && len > 0) std::memcpy(out, resp.data, len);

    // Debug gate intercept: notify conditions of this register read
    if (debug_gate_ && debug_gate_->hasAnyConditions()) {
        debug_gate_->onRegisterRead(slave_address.slavePosition(),
                                    register_address.raw(),
                                    reinterpret_cast<const uint8_t*>(out), len);
    }

    return result.wkc > 0;
}
// ============================================================================
// Wait helpers
// ============================================================================

bool Master::waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                         RxDatagram& out)
{
    if (cancel_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    PacketFilter filter = PacketFilter::byIndex(idx);
    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (cancel_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

bool Master::waitForResponseAdo(uint16_t ado, Command cmd,
                                         unsigned int timeout_ms,
                                         RxDatagram& out)
{
    if (cancel_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    PacketFilter filter{};
    filter.command   = cmd;
    filter.ado       = ado;
    filter.match_ado = true;

    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (cancel_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

size_t Master::preRegisterResponseWaiter(uint8_t idx,
                                                  uint8_t* buffer,
                                                  size_t buffer_size)
{
    PacketFilter filter = PacketFilter::byIndex(idx);
    return packet_router_.preRegisterWaiter(filter, buffer, buffer_size);
}

WaitResult Master::waitForPreRegistered(size_t slot, uint32_t timeout_ms)
{
    if (cancel_requested_.load(std::memory_order_acquire)) {
        return WaitResult::Timeout();
    }
    return packet_router_.waitForPreRegistered(slot, timeout_ms);
}

// ============================================================================
// Index allocation
// ============================================================================

uint8_t Master::allocIdx()
{
    uint8_t idx;
    do { idx = next_idx_.fetch_add(1, std::memory_order_relaxed); }
    while (idx == kFireAndForgetIdx);
    return idx;
}

// Mailbox override helpers (allow application to enforce XML-derived mailbox values)
struct MailboxOverride {
    bool enabled{false};
    uint16_t wr_addr{0};
    uint16_t wr_len{0};
    uint16_t rd_addr{0};
    uint16_t rd_len{0};
    uint16_t proto{0};
};

void Master::setMailboxOverride(SlaveAddress slave_address, uint16_t wr_addr, uint16_t wr_len,
                                       uint16_t rd_addr, uint16_t rd_len, uint16_t proto)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return;
    }

    std::lock_guard<std::mutex> _lg(m_mailbox_override_mutex_);
    if (slave_index >= PDO::kMaxPDOSlaves) return;
    if (m_mailbox_overrides_.size() < PDO::kMaxPDOSlaves) m_mailbox_overrides_.resize(PDO::kMaxPDOSlaves);
    auto& ov = m_mailbox_overrides_[slave_index];
    ov.enabled = true;
    ov.wr_addr = wr_addr;
    ov.wr_len = wr_len;
    ov.rd_addr = rd_addr;
    ov.rd_len = rd_len;
    ov.proto = proto;
}
// ============================================================================
// Packet debug printer
// ============================================================================

static const char* etherTypeToString(uint16_t ether_type)
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

static void printEtherCATFrame(const uint8_t* frame, size_t length, bool is_tx, bool print_ethernet)
{
    using namespace Raw;
    const char* dir = is_tx ? "TX" : "RX";

    if (print_ethernet && length >= sizeof(EtherCAT::EthernetHeader)) {
        const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
        TETHER_LOGI("ec_pkt", "[%s] Ethernet: dst=%02X:%02X:%02X:%02X:%02X:%02X src=%02X:%02X:%02X:%02X:%02X:%02X etherType=0x%04X",
                    dir,
                    eth->dst[0], eth->dst[1], eth->dst[2], eth->dst[3], eth->dst[4], eth->dst[5],
                    eth->src[0], eth->src[1], eth->src[2], eth->src[3], eth->src[4], eth->src[5],
                    bswap16(eth->etherType_be));
    }

    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader)) {
        TETHER_LOGI("ec_pkt", "[%s] Frame too short for EtherCAT header (%u bytes)",
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
            TETHER_LOGI("ec_pkt", "[%s] EtherCAT-over-UDP: src_port=%u dst_port=%u",
                        dir, bswap16(udp->src_port_be), bswap16(udp->dst_port_be));
            ecat_offset = ip_off + ihl + sizeof(UDPHeader);
        }
#endif // TETHER_ENABLE_UDP_ENCAPSULATION
    }

    if (length < ecat_offset + sizeof(EtherCAT::FrameHeader)) {
        TETHER_LOGI("ec_pkt", "[%s] Frame too short for EtherCAT header after encap (%u bytes)",
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
        TETHER_LOGI("ec_pkt", "%s", line.c_str());
    } else {
        TETHER_LOGI("ec_pkt", "[%s] EtherCAT Frame: length=%u type=%u", dir, ec_len, ec_type);
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
            TETHER_LOGI("ec_pkt", "%s", dgram_line.c_str());
            if (dg.has_data && !inline_data) {
                TETHER_LOGI("ec_pkt", "[%s]     Data (%u/%u bytes): %s",
                            dir, static_cast<unsigned>(dg.dump_len), static_cast<unsigned>(dg.datalen), dg.hex.c_str());
                for (const auto& coe_line : dg.coe_lines) {
                    TETHER_LOGI("ec_pkt", "[%s]     CoE: %s", dir, coe_line.c_str());
                }
            }
        }
    }
}

// ============================================================================
// EtherCAT-over-UDP encapsulation helpers
// ============================================================================

#if TETHER_ENABLE_UDP_ENCAPSULATION
uint16_t Master::computeIpChecksum(const uint8_t* ip_header)
{
    // Standard IPv4 header checksum: ones-complement sum of all 16-bit words.
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) {
        sum += static_cast<uint32_t>(ip_header[i]) << 8;
        sum += static_cast<uint32_t>(ip_header[i + 1]);
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return static_cast<uint16_t>(~sum & 0xFFFF);
}

bool Master::encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                              uint8_t* out_buf, size_t out_cap, size_t* out_len) const
{
    using namespace Raw;

    // Input frame must have at least an Ethernet header (14 bytes).
    if (in_len < sizeof(EtherCAT::EthernetHeader)) return false;

    // The EtherCAT payload is everything after the 14-byte Ethernet header.
    const size_t ecat_payload_len = in_len - sizeof(EtherCAT::EthernetHeader);
    const size_t encap_len = sizeof(EtherCAT::EthernetHeader) +
                             sizeof(IPv4Header) + sizeof(UDPHeader) + ecat_payload_len;

    // Minimum Ethernet frame size (without FCS).
    constexpr size_t kMinEthFrame = 60;
    const size_t padded_len = (encap_len < kMinEthFrame) ? kMinEthFrame : encap_len;

    if (padded_len > out_cap) return false;

    std::memset(out_buf, 0, padded_len);

    // --- Ethernet header (reuse dst/src MAC from input, change EtherType to IPv4) ---
    const auto* in_eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(in_frame);
    auto* out_eth = reinterpret_cast<EtherCAT::EthernetHeader*>(out_buf);
    std::memcpy(out_eth->dst, in_eth->dst, 6);
    std::memcpy(out_eth->src, in_eth->src, 6);
    out_eth->etherType_be = host_to_be16(kEtherTypeIPv4);

    // --- IPv4 header ---
    auto* ip = reinterpret_cast<IPv4Header*>(out_buf + sizeof(EtherCAT::EthernetHeader));
    ip->version_ihl = 0x45;
    ip->tos = 0;
    const uint16_t ip_total_len = static_cast<uint16_t>(sizeof(IPv4Header) + sizeof(UDPHeader) + ecat_payload_len);
    ip->total_length_be = host_to_be16(ip_total_len);
    ip->identification_be = host_to_be16(ip_id_counter_.fetch_add(1, std::memory_order_relaxed));
    ip->flags_fragment_be = host_to_be16(0x4000); // Don't Fragment, no offset
    ip->ttl = 64;
    ip->protocol = 0x11; // UDP
    ip->checksum_be = 0; // Will be computed below
    ip->src_ip_be = host_to_be32(config_.udp_encapsulation.source_ip);
    ip->dst_ip_be = host_to_be32(config_.udp_encapsulation.destination_ip);
    // Compute checksum over the 20-byte header.
    ip->checksum_be = host_to_be16(computeIpChecksum(reinterpret_cast<const uint8_t*>(ip)));

    // --- UDP header ---
    auto* udp = reinterpret_cast<UDPHeader*>(out_buf + sizeof(EtherCAT::EthernetHeader) + sizeof(IPv4Header));
    udp->src_port_be = host_to_be16(config_.udp_encapsulation.source_port);
    udp->dst_port_be = host_to_be16(config_.udp_encapsulation.destination_port);
    const uint16_t udp_len = static_cast<uint16_t>(sizeof(UDPHeader) + ecat_payload_len);
    udp->length_be = host_to_be16(udp_len);
    udp->checksum_be = 0; // Not computed (optional for IPv4 UDP)

    // --- EtherCAT payload (everything from the original frame after the Ethernet header) ---
    uint8_t* ecat_dst = out_buf + sizeof(EtherCAT::EthernetHeader) + sizeof(IPv4Header) + sizeof(UDPHeader);
    std::memcpy(ecat_dst, in_frame + sizeof(EtherCAT::EthernetHeader), ecat_payload_len);

    *out_len = padded_len;
    return true;
}

bool Master::sendWithEncapsulation(const uint8_t* frame, size_t len)
{
    if (!config_.udp_encapsulation.enabled) {
        return iface_.send ? iface_.send(frame, len) : false;
    }

    // Build the encapsulated frame in a stack buffer.
    // Max frame: 1514 (max Ethernet) + 28 (IP+UDP overhead) = 1542, round up to 1600.
    constexpr size_t kEncapBufSize = 1600;
    uint8_t encap_buf[kEncapBufSize] = {0};
    size_t encap_len = 0;
    if (!encapsulateFrame(frame, len, encap_buf, kEncapBufSize, &encap_len)) {
        TETHER_LOGE(TAG, "encapsulateFrame failed (in_len=%u)", static_cast<unsigned>(len));
        return false;
    }
    return iface_.send ? iface_.send(encap_buf, encap_len) : false;
}

size_t Master::maxEtherCATPayloadPerFrame() const
{
    if (config_.udp_encapsulation.enabled) {
        return Raw::kMaxEtherCATPayloadPerFrame - Raw::kUdpEncapOverhead;
    }
    return Raw::kMaxEtherCATPayloadPerFrame;
}
#endif // TETHER_ENABLE_UDP_ENCAPSULATION

// ============================================================================
// Transport primitives
// ============================================================================

bool Master::sendRawFrame(const void* buf, size_t len)
{
    if (debug_flags_.txPackets) {
        printEtherCATFrame(reinterpret_cast<const uint8_t*>(buf), len, true, false);
    }
    return sendWithEncapsulation(reinterpret_cast<const uint8_t*>(buf), len);
}

bool Master::sendSingleDatagram(Command cmd, uint8_t idx,
                                         uint16_t adp, uint16_t ado,
                                         const void* data, uint16_t datalen,
                                         bool roundtrip)
{
    using namespace Raw;

    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    constexpr size_t kMinEthFrameNoFcs = 60;
    constexpr size_t kMaxEthFrameNoFcs = 1514;

    const size_t required_len =
        sizeof(EtherCATSingleDgramFrameHeader) + datalen + sizeof(uint16_t);
    // Account for UDP encapsulation overhead in the max frame size check.
#if TETHER_ENABLE_UDP_ENCAPSULATION
    const size_t encap_overhead = config_.udp_encapsulation.enabled ? kUdpEncapOverhead : 0;
#else
    constexpr size_t encap_overhead = 0;
#endif
    if (required_len + encap_overhead > kMaxEthFrameNoFcs) {
        TETHER_LOGE(TAG,
            "Datagram exceeds max Ethernet frame size (datalen=%u required=%u encap=%u, "
            "max_frame=%zu). This is a physical Ethernet frame size limit, not a Tether buffer.",
            datalen, static_cast<unsigned>(required_len),
            static_cast<unsigned>(encap_overhead), kMaxEthFrameNoFcs);
        return false;
    }

    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;
    uint8_t txbuf[kMaxEthFrameNoFcs] = {0};

    auto* hdr = reinterpret_cast<EtherCATSingleDgramFrameHeader*>(txbuf);
    std::memcpy(hdr->eth.dst, dst_mac, 6);
    std::memcpy(hdr->eth.src, src_mac_, 6);
    hdr->eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);

    const uint16_t payload_len =
        static_cast<uint16_t>(sizeof(EtherCATDatagramHeader) + datalen + sizeof(uint16_t));
    constexpr uint16_t type = 0x1;
    hdr->ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu) | ((type & 0x0Fu) << 12)));

    hdr->dg.cmd    = cmd;
    hdr->dg.idx    = idx;
    hdr->dg.adp_le = host_to_le16(adp);
    hdr->dg.ado_le = host_to_le16(ado);
    const uint16_t flags = roundtrip ? (1u << 14) : 0u;
    hdr->dg.lenFlags.raw_le =
        host_to_le16(static_cast<uint16_t>((datalen & 0x07FFu) | flags));
    hdr->dg.irq_le = host_to_le16(0);

    uint8_t* payload = txbuf + sizeof(EtherCATSingleDgramFrameHeader);
    if (datalen > 0) {
        if (data) std::memcpy(payload, data, datalen);
        else      std::memset(payload, 0, datalen);
    }
    *reinterpret_cast<uint16_t*>(payload + datalen) = host_to_le16(0);

    if (iface_.send) {
        if (debug_flags_.txPackets) {
            printEtherCATFrame(txbuf, frame_len, true, false);
        }
        auto& clock = Tether::Platform::Clock::instance();
        int last_errno = 0;
        for (int retry = 0; retry <= kMaxTxRetries; retry++) {
            if (cancel_requested_.load(std::memory_order_acquire)) {
                TETHER_LOGW(TAG, "sendSingleDatagram cancelled (cmd=%s idx=%u)",
                            commandToString(cmd), static_cast<unsigned>(idx));
                return false;
            }
            errno = 0;
            if (sendWithEncapsulation(txbuf, frame_len)) return true;
            last_errno = errno;
#if TETHER_ENABLE_ETHERCAT_STATS
            tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
#endif
            const int64_t t0 = clock.getMicroseconds();
            while ((clock.getMicroseconds() - t0) <
                   static_cast<int64_t>(kTxRetryDelayUs)) {}
        }

        char msg[256];
        if (last_errno != 0) {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after %d retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u errno=%d:%s). "
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in TetherConfig.hpp).",
                          kMaxTxRetries + 1,
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          last_errno,
                          std::strerror(last_errno),
                          kMaxTxRetries);
        } else {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after %d retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u errno=0). "
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in TetherConfig.hpp).",
                          kMaxTxRetries + 1,
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          kMaxTxRetries);
        }
        send_fail_log_.logLegacy(1, TAG, msg);
#if TETHER_ENABLE_ETHERCAT_STATS
        tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
#endif
        return false;
    }

    TETHER_LOGE(TAG, "No NetworkInterface available for send");
    return false;
}

// ============================================================================
// sendMultiDatagram — pack multiple datagrams into one or more frames
// ============================================================================

size_t Master::sendMultiDatagram(const MultiDatagramSpec* specs, size_t count)
{
    using namespace Raw;

    if (count == 0 || !specs) return 0;

    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    constexpr size_t kMinEthFrameNoFcs = 60;
    constexpr size_t kMaxEthFrameNoFcs = 1514;
    constexpr size_t kHeaderSize = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
#if TETHER_ENABLE_UDP_ENCAPSULATION
    const size_t max_payload = maxEtherCATPayloadPerFrame();
#else
    constexpr size_t max_payload = kMaxEtherCATPayloadPerFrame;
#endif

    size_t frames_sent = 0;
    size_t i = 0;

    while (i < count) {
        uint8_t txbuf[kMaxEthFrameNoFcs] = {0};

        // Ethernet header
        auto* eth = reinterpret_cast<EtherCAT::EthernetHeader*>(txbuf);
        std::memcpy(eth->dst, dst_mac, 6);
        std::memcpy(eth->src, src_mac_, 6);
        eth->etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);

        // Start packing datagrams after Ethernet + EtherCAT frame header
        size_t payload_offset = kHeaderSize;
        size_t payload_bytes = 0;
        size_t dg_count_in_frame = 0;

        while (i + dg_count_in_frame < count) {
            const auto& spec = specs[i + dg_count_in_frame];
            const size_t dg_size = kDatagramOverhead + spec.datalen;

            if (payload_bytes + dg_size > max_payload)
                break;

            size_t dg_offset = payload_offset + payload_bytes;
            auto* dg = reinterpret_cast<EtherCAT::DatagramHeader*>(txbuf + dg_offset);

            dg->cmd    = spec.cmd;
            dg->idx    = spec.idx;
            dg->adp_le = host_to_le16(spec.adp);
            dg->ado_le = host_to_le16(spec.ado);

            // Set more flag if this isn't the last datagram in this frame
            // (will be finalized below)
            uint16_t flags = spec.roundtrip ? (1u << 14) : 0u;
            dg->lenFlags_le =
                host_to_le16(static_cast<uint16_t>((spec.datalen & 0x07FFu) | flags));
            dg->irq_le = host_to_le16(0);

            // Copy payload
            uint8_t* dg_payload = txbuf + dg_offset + sizeof(EtherCAT::DatagramHeader);
            if (spec.datalen > 0) {
                if (spec.data) std::memcpy(dg_payload, spec.data, spec.datalen);
                else            std::memset(dg_payload, 0, spec.datalen);
            }
            // WKC = 0 (filled by slave)
            *reinterpret_cast<uint16_t*>(dg_payload + spec.datalen) = host_to_le16(0);

            payload_bytes += dg_size;
            dg_count_in_frame++;
        }

        if (dg_count_in_frame == 0) {
            // Single datagram too big for one frame
            TETHER_LOGE(TAG,
                "sendMultiDatagram: datagram %zu exceeds max Ethernet frame size (datalen=%u, "
                "max_payload=%zu). This is a physical Ethernet frame size limit, not a Tether buffer.",
                i, specs[i].datalen, max_payload);
            return frames_sent > 0 ? frames_sent : 0;
        }

        // Set the more flag on all datagrams except the last in this frame
        for (size_t d = 0; d < dg_count_in_frame; d++) {
            size_t dg_offset = payload_offset;
            for (size_t prev = 0; prev < d; prev++) {
                dg_offset += kDatagramOverhead + specs[i + prev].datalen;
            }
            auto* dg = reinterpret_cast<EtherCAT::DatagramHeader*>(txbuf + dg_offset);
            uint16_t cur = le16_to_host(dg->lenFlags_le);
            if (d < dg_count_in_frame - 1) {
                cur |= 0x8000u;  // Set more flag
            } else {
                cur &= ~0x8000u; // Clear more flag on last
            }
            dg->lenFlags_le = host_to_le16(cur);
        }

        // Fill EtherCAT frame header
        auto* ec_hdr = reinterpret_cast<EtherCAT::FrameHeader*>(txbuf + sizeof(EtherCAT::EthernetHeader));
        constexpr uint16_t type = 0x1;
        ec_hdr->raw_le = host_to_le16(
            static_cast<uint16_t>((payload_bytes & 0x07FFu) | ((type & 0x0Fu) << 12)));

        // Frame length (minimum 60 bytes)
        size_t frame_len = kHeaderSize + payload_bytes;
        if (frame_len < kMinEthFrameNoFcs) frame_len = kMinEthFrameNoFcs;

        // Send with retry
        if (iface_.send) {
            if (debug_flags_.txPackets) {
                printEtherCATFrame(txbuf, frame_len, true, false);
            }
            auto& clock = Tether::Platform::Clock::instance();
            bool sent = false;
            for (int retry = 0; retry <= kMaxTxRetries; retry++) {
                if (cancel_requested_.load(std::memory_order_acquire)) {
                    TETHER_LOGW(TAG, "sendMultiDatagram cancelled");
                    return frames_sent;
                }
                if (sendWithEncapsulation(txbuf, frame_len)) { sent = true; break; }
#if TETHER_ENABLE_ETHERCAT_STATS
                tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
#endif
                const int64_t t0 = clock.getMicroseconds();
                while ((clock.getMicroseconds() - t0) <
                       static_cast<int64_t>(kTxRetryDelayUs)) {}
            }
            if (!sent) {
                send_fail_log_.logLegacy(1, TAG,
                    "sendMultiDatagram: send failed after retries");
#if TETHER_ENABLE_ETHERCAT_STATS
                tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
#endif
                return frames_sent;
            }
            frames_sent++;
        } else {
            TETHER_LOGE(TAG, "No NetworkInterface available for sendMultiDatagram");
            return frames_sent;
        }

        i += dg_count_in_frame;
    }

    return frames_sent;
}

// ============================================================================
// BatchTransaction implementation
// ============================================================================

Master::BatchTransaction::BatchTransaction(TransactionRouter* router,
                                             std::vector<uint8_t> idxs,
                                             std::vector<size_t> slots,
                                             std::vector<RxDatagram> responses)
    : router_(router)
    , idxs_(std::move(idxs))
    , slots_(std::move(slots))
    , responses_(std::move(responses))
{
}

Master::BatchTransaction::~BatchTransaction()
{
    // Clean up any unclaimed slots
    if (router_) {
        for (size_t i = 0; i < slots_.size(); i++) {
            if (slots_[i] < TransactionRouter::kNumSlots) {
                router_->cancelPreRegistered(slots_[i]);
            }
        }
    }
}

BatchReadResult Master::BatchTransaction::getResult(size_t i, uint32_t timeout_ms)
{
    BatchReadResult result;
    if (i >= idxs_.size() || !router_) return result;

    if (slots_[i] >= TransactionRouter::kNumSlots) return result;

    // If already completed (response was routed before we called), return it
    if (responses_[i].wkc != 0 || responses_[i].datalen > 0) {
        result.success = true;
        result.wkc = responses_[i].wkc;
        result.datalen = responses_[i].datalen;
        result.data = responses_[i].data;
        return result;
    }

    // Wait for the response
    WaitResult wr = router_->waitForPreRegistered(slots_[i], timeout_ms);
    if (wr.success) {
        result.success = true;
        result.wkc = wr.wkc;
        result.datalen = wr.data_length;
        result.data = responses_[i].data;
    }

    return result;
}

bool Master::BatchTransaction::waitAll(uint32_t timeout_ms,
                                        std::vector<BatchReadResult>& out)
{
    out.resize(idxs_.size());
    bool all_ok = true;
    for (size_t i = 0; i < idxs_.size(); i++) {
        out[i] = getResult(i, timeout_ms);
        if (!out[i].success) all_ok = false;
    }
    return all_ok;
}

void Master::BatchTransaction::cancel()
{
    cancelled_ = true;
    if (router_) {
        for (size_t i = 0; i < slots_.size(); i++) {
            if (slots_[i] < TransactionRouter::kNumSlots) {
                router_->cancelPreRegistered(slots_[i]);
            }
        }
    }
}

// ============================================================================
// Batch read/write APIs
// ============================================================================

Master::BatchTransaction Master::readRegistersBatch(
    const SlaveAddress* slave_addresses,
    const uint16_t* register_addresses,
    const uint16_t* lengths,
    size_t count)
{
    if (count == 0 || !slave_addresses || !register_addresses || !lengths)
        return BatchTransaction();

    std::vector<MultiDatagramSpec> specs(count);
    std::vector<uint8_t> idxs(count);
    std::vector<size_t> slots(count);
    std::vector<RxDatagram> responses(count);

    for (size_t i = 0; i < count; i++) {
        idxs[i] = allocIdx();
        slots[i] = idxs[i]; // slot index == idx in TransactionRouter

        Command cmd = slave_addresses[i].isPhysical() ? Command::APRD : Command::FPRD;
        specs[i] = MultiDatagramSpec{
            cmd,
            idxs[i],
            slave_addresses[i].raw(),
            register_addresses[i],
            nullptr,        // no data for reads
            lengths[i],
            true            // roundtrip
        };

        // Pre-register the waiter slot
        PacketFilter filter = PacketFilter::byIndex(idxs[i]);
        slots[i] = packet_router_.preRegisterWaiter(
            filter, responses[i].data, sizeof(responses[i].data));
    }

    // Send all datagrams in one frame (auto-splits if needed)
    size_t sent = sendMultiDatagram(specs.data(), count);
    if (sent == 0) {
        // Send failed — cancel all pre-registered slots
        for (size_t i = 0; i < count; i++) {
            if (slots[i] < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slots[i]);
        }
        return BatchTransaction();
    }

    return BatchTransaction(&packet_router_, std::move(idxs), std::move(slots),
                            std::move(responses));
}

Master::BatchTransaction Master::writeRegistersBatch(
    const SlaveAddress* slave_addresses,
    const uint16_t* register_addresses,
    const void* const* data,
    const uint16_t* lengths,
    size_t count)
{
    if (count == 0 || !slave_addresses || !register_addresses || !data || !lengths)
        return BatchTransaction();

    std::vector<MultiDatagramSpec> specs(count);
    std::vector<uint8_t> idxs(count);
    std::vector<size_t> slots(count);
    std::vector<RxDatagram> responses(count);

    for (size_t i = 0; i < count; i++) {
        idxs[i] = allocIdx();
        slots[i] = idxs[i];

        Command cmd = slave_addresses[i].isPhysical() ? Command::APWR : Command::FPWR;
        specs[i] = MultiDatagramSpec{
            cmd,
            idxs[i],
            slave_addresses[i].raw(),
            register_addresses[i],
            data[i],
            lengths[i],
            true            // roundtrip
        };

        PacketFilter filter = PacketFilter::byIndex(idxs[i]);
        slots[i] = packet_router_.preRegisterWaiter(
            filter, responses[i].data, sizeof(responses[i].data));
    }

    size_t sent = sendMultiDatagram(specs.data(), count);
    if (sent == 0) {
        for (size_t i = 0; i < count; i++) {
            if (slots[i] < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slots[i]);
        }
        return BatchTransaction();
    }

    return BatchTransaction(&packet_router_, std::move(idxs), std::move(slots),
                            std::move(responses));
}


void Master::resetIdx()
{
    next_idx_.store(0, std::memory_order_relaxed);
}

// ============================================================================
// Internal: frame parsing
// ============================================================================

void Master::parseEtherCATFrame(const uint8_t* frame, size_t length)
{
    using namespace Raw;  // for le16_to_host, Command, RxDatagram, etc.

#if TETHER_ENABLE_ETHERCAT_STATS
    rx_frame_count_.fetch_add(1, std::memory_order_relaxed);
#endif

    // Use fully-qualified EtherCAT::EthernetHeader to avoid ambiguity
    // with Raw::EthernetHeader
    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader))
        return;

    const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
    const uint16_t ether_type = bswap16(eth->etherType_be);

    // The EtherCAT frame header and datagrams start right after the Ethernet
    // header for direct EtherCAT (EtherType 0x88A4).  For EtherCAT-over-UDP,
    // we need to skip the IPv4 + UDP headers first.
    size_t ecat_offset = sizeof(EtherCAT::EthernetHeader);

    if (ether_type == EtherCAT::kEtherTypeEtherCAT) {
        // Direct EtherCAT — nothing extra to skip.
    }
#if TETHER_ENABLE_UDP_ENCAPSULATION
    else if (ether_type == kEtherTypeIPv4) {
        // EtherCAT-over-UDP encapsulation: parse IPv4 + UDP headers.
        const size_t ip_offset = sizeof(EtherCAT::EthernetHeader);
        if (ip_offset + sizeof(IPv4Header) > length) return;

        const auto* ip = reinterpret_cast<const IPv4Header*>(frame + ip_offset);
        const uint8_t ihl = (ip->version_ihl & 0x0F) * 4;
        if (ihl < sizeof(IPv4Header) || ip_offset + ihl > length) return;
        if (ip->protocol != 0x11) return; // not UDP

        const size_t udp_offset = ip_offset + ihl;
        if (udp_offset + sizeof(UDPHeader) > length) return;

        const auto* udp = reinterpret_cast<const UDPHeader*>(frame + udp_offset);
        const uint16_t dst_port = bswap16(udp->dst_port_be);
        if (dst_port != kEtherCATOverUdpPort) return; // not EtherCAT-over-UDP

        ecat_offset = udp_offset + sizeof(UDPHeader);
        if (ecat_offset + sizeof(EtherCAT::FrameHeader) > length) return;
    }
#endif // TETHER_ENABLE_UDP_ENCAPSULATION
    else {
        if (debug_flags_.rxPackets) {
            const char* name = etherTypeToString(ether_type);
            if (name) {
                TETHER_LOGI("ec_pkt", "[RX] Non-EtherCAT frame: %s (0x%04X, len=%u)",
                            name, ether_type, static_cast<unsigned>(length));
            } else {
                TETHER_LOGI("ec_pkt", "[RX] Non-EtherCAT frame: unknown (0x%04X, len=%u)",
                            ether_type, static_cast<unsigned>(length));
            }
        }
        return;
    }

    if (debug_flags_.rxPackets) {
        printEtherCATFrame(frame, length, false, false);
    }

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(frame + ecat_offset);
    const uint16_t ec_len = le16_to_host(ec_hdr->raw_le) & 0x07FFu;

    if (length < ecat_offset + sizeof(EtherCAT::FrameHeader) + ec_len)
        return;

    const size_t payload_offset = ecat_offset + sizeof(EtherCAT::FrameHeader);
    size_t offset = payload_offset;
    size_t remaining = ec_len;
    uint8_t dg_idx = 0;

    while (remaining >= sizeof(EtherCAT::DatagramHeader) &&
           offset + sizeof(EtherCAT::DatagramHeader) <= length) {
        const auto* dg = reinterpret_cast<const EtherCAT::DatagramHeader*>(frame + offset);

        const uint16_t ado     = le16_to_host(dg->ado_le);
        const uint16_t adp     = le16_to_host(dg->adp_le);
        const uint16_t len_flags = le16_to_host(dg->lenFlags_le);
        const uint16_t datalen = len_flags & 0x07FFu;
        const bool more_flag   = (len_flags & 0x8000u) != 0;

        const size_t data_offset = offset + sizeof(EtherCAT::DatagramHeader);
        const size_t wkc_offset  = data_offset + datalen;
        if (length < wkc_offset + sizeof(uint16_t)) break;

        const uint16_t wkc =
            le16_to_host(*reinterpret_cast<const uint16_t*>(frame + wkc_offset));

        RxDatagram msg{};
        msg.idx = dg->idx; msg.cmd = dg->cmd; msg.adp = adp; msg.ado = ado;
        msg.datalen = datalen; msg.wkc = wkc;
        if (datalen > 0)
            std::memcpy(msg.data, frame + data_offset,
                        std::min<size_t>(datalen, sizeof(msg.data)));

        if (dg->idx == kFireAndForgetIdx) {
            if (dg->cmd == Command::APRD && txpdo_rx_queue_)
                txpdo_rx_queue_->send(msg, 0);
        } else {
            size_t routed = packet_router_.routePacket(msg);
            if (routed == 0) {
                if (unrouted_log_count_ < 10) {
                    TETHER_LOGW("ec_rx", "Unrouted pkt dg=%u idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u",
                             dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc);
                }
                unrouted_log_count_++;
                if (rx_queue_ && rx_queue_->send(msg, 0)) {
#if TETHER_ENABLE_ETHERCAT_STATS
                    rx_queue_sent_.fetch_add(1, std::memory_order_relaxed);
#endif
                } else {
                    if (rx_drop_log_count_ < 10 || (rx_drop_log_count_ % 500 == 0)) {
                        TETHER_LOGW("ec_rx", "RX queue full! Dropped dg=%u idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u (total dropped: %u)",
                                 dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc, rx_drop_log_count_);
                    }
                    rx_drop_log_count_++;
                }
            }
        }

        const size_t dg_total = sizeof(EtherCAT::DatagramHeader) + datalen + sizeof(uint16_t);
        if (remaining < dg_total) break;
        remaining -= dg_total;
        offset += dg_total;
        dg_idx++;

        if (!more_flag) break;
    }
}

} // namespace EtherCAT
