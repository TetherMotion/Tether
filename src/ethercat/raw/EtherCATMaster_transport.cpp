/**
 * @file EtherCATMaster_transport.cpp
 * @brief EtherCATMaster — Low-level datagram transport, register I/O and packet debug
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/EtherCATRealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
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
static constexpr int       kMaxTxRetries   = 3;
static constexpr uint32_t  kTxRetryDelayUs = 50;

static const char* TAG = "ethercat";

// Global debug flag for ethercat-statemachine (shared with EtherCATSlave)
extern bool g_debug_statemachine;

// Global debug flags for tx/rx packet logging (shared with EtherCATSlave)
extern bool g_debug_tx_packets;
extern bool g_debug_rx_packets;

// Global debug flags for PDO logging (shared with PDOManager)
extern bool g_debug_rx_pdo;
extern bool g_debug_tx_pdo;

bool EtherCATMaster::sendDatagram(Command cmd, uint8_t idx,
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

bool EtherCATMaster::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
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
    return result.success && result.wkc > 0;
}

bool EtherCATMaster::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                   uint16_t value)
{
    const uint16_t little_endian_value = Raw::host_to_le16(value);
    return writeRegister(slave_address, register_address, &little_endian_value, sizeof(little_endian_value), 200);
}

bool EtherCATMaster::readRegister(SlaveAddress slave_address, RegisterAddress register_address,
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
    if (!result.success) return false;

    resp.datalen = static_cast<uint16_t>(result.data_length);
    if (resp.datalen < len) return false;
    if (out && len > 0) std::memcpy(out, resp.data, len);
    return result.wkc > 0;
}
// ============================================================================
// Wait helpers
// ============================================================================

bool EtherCATMaster::waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                         RxDatagram& out)
{
    PacketFilter filter = PacketFilter::byIndex(idx);
    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

bool EtherCATMaster::waitForResponseAdo(uint16_t ado, Command cmd,
                                         unsigned int timeout_ms,
                                         RxDatagram& out)
{
    PacketFilter filter{};
    filter.command   = cmd;
    filter.ado       = ado;
    filter.match_ado = true;

    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

size_t EtherCATMaster::preRegisterResponseWaiter(uint8_t idx,
                                                  uint8_t* buffer,
                                                  size_t buffer_size)
{
    PacketFilter filter = PacketFilter::byIndex(idx);
    return packet_router_.preRegisterWaiter(filter, buffer, buffer_size);
}

WaitResult EtherCATMaster::waitForPreRegistered(size_t slot, uint32_t timeout_ms)
{
    return packet_router_.waitForPreRegistered(slot, timeout_ms);
}

// ============================================================================
// Index allocation
// ============================================================================

uint8_t EtherCATMaster::allocIdx()
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

void EtherCATMaster::setMailboxOverride(SlaveAddress slave_address, uint16_t wr_addr, uint16_t wr_len,
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

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(
        frame + sizeof(EtherCAT::EthernetHeader));
    const uint16_t ec_raw = le16_to_host(ec_hdr->raw_le);
    const uint16_t ec_len = ec_raw & 0x07FFu;
    const uint16_t ec_type = (ec_raw >> 12) & 0x0Fu;

    TETHER_LOGI("ec_pkt", "[%s] EtherCAT Frame: length=%u type=%u", dir, ec_len, ec_type);

    size_t offset = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
    size_t remaining = ec_len;
    uint8_t dg_idx = 0;

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

        TETHER_LOGI("ec_pkt", "[%s]   Datagram[%u]: cmd=%s idx=0x%02X adp=0x%04X ado=0x%04X len=%u wkc=%u more=%s circulating=%s",
                    dir, dg_idx,
                    commandToString(dg->cmd),
                    dg->idx,
                    adp, ado,
                    datalen, wkc,
                    more ? "yes" : "no",
                    circulating ? "yes" : "no");

        if (datalen > 0 && length >= data_offset + datalen) {
            constexpr size_t kMaxHexDump = 64;
            const size_t dump_len = (datalen < kMaxHexDump) ? datalen : kMaxHexDump;
            char hexbuf[256];
            size_t pos = 0;
            for (size_t i = 0; i < dump_len && pos + 3 < sizeof(hexbuf); i++) {
                pos += std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "%02X ", frame[data_offset + i]);
            }
            if (datalen > kMaxHexDump) {
                pos += std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "...");
            }
            TETHER_LOGI("ec_pkt", "[%s]     Data (%u/%u bytes): %s",
                        dir, static_cast<unsigned>(dump_len), static_cast<unsigned>(datalen), hexbuf);
        }

        const size_t dg_total = sizeof(EtherCAT::DatagramHeader) + datalen + sizeof(uint16_t);
        if (remaining < dg_total) break;
        remaining -= dg_total;
        offset += dg_total;
        dg_idx++;

        if (!more) break;
    }
}

// ============================================================================
// Transport primitives
// ============================================================================

bool EtherCATMaster::sendRawFrame(const void* buf, size_t len)
{
    if (g_debug_tx_packets) {
        printEtherCATFrame(reinterpret_cast<const uint8_t*>(buf), len, true, false);
    }
    if (iface_.send)
        return iface_.send(reinterpret_cast<const uint8_t*>(buf), len);
    TETHER_LOGE(TAG, "No NetworkInterface registered!");
    return false;
}

bool EtherCATMaster::sendSingleDatagram(Command cmd, uint8_t idx,
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
    if (required_len > kMaxEthFrameNoFcs) {
        TETHER_LOGE(TAG, "Datagram too big (datalen=%u required=%u)",
                    datalen, static_cast<unsigned>(required_len));
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
        if (g_debug_tx_packets) {
            printEtherCATFrame(txbuf, frame_len, true, false);
        }
        auto& clock = Tether::Platform::Clock::instance();
        int last_errno = 0;
        for (int retry = 0; retry <= kMaxTxRetries; retry++) {
            errno = 0;
            if (iface_.send(txbuf, frame_len)) return true;
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
                          "NetworkInterface::send failed after retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u retries=%d errno=%d:%s)",
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          kMaxTxRetries + 1,
                          last_errno,
                          std::strerror(last_errno));
        } else {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u retries=%d errno=0)",
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          kMaxTxRetries + 1);
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


void EtherCATMaster::resetIdx()
{
    next_idx_.store(0, std::memory_order_relaxed);
}

// ============================================================================
// Internal: frame parsing
// ============================================================================

void EtherCATMaster::parseEtherCATFrame(const uint8_t* frame, size_t length)
{
    using namespace Raw;  // for le16_to_host, Command, RxDatagram, etc.

#if TETHER_ENABLE_ETHERCAT_STATS
    rx_frame_count_++;
#endif

    // Use fully-qualified EtherCAT::EthernetHeader to avoid ambiguity
    // with Raw::EthernetHeader
    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader))
        return;

    const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
    const uint16_t ether_type = bswap16(eth->etherType_be);
    if (ether_type != EtherCAT::kEtherTypeEtherCAT) {
        if (g_debug_rx_packets) {
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

    if (g_debug_rx_packets) {
        printEtherCATFrame(frame, length, false, false);
    }

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(
        frame + sizeof(EtherCAT::EthernetHeader));
    const uint16_t ec_len = le16_to_host(ec_hdr->raw_le) & 0x07FFu;

    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader) + ec_len)
        return;

    const size_t payload_offset = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
    const auto* dg = reinterpret_cast<const EtherCAT::DatagramHeader*>(frame + payload_offset);

    const uint16_t ado     = le16_to_host(dg->ado_le);
    const uint16_t adp     = le16_to_host(dg->adp_le);
    const uint16_t datalen = le16_to_host(dg->lenFlags_le) & 0x07FFu;

    const size_t data_offset = payload_offset + sizeof(EtherCAT::DatagramHeader);
    const size_t wkc_offset  = data_offset + datalen;
    if (length < wkc_offset + sizeof(uint16_t)) return;

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
            static uint32_t unrouted_count = 0;
            if (unrouted_count < 10) {
                TETHER_LOGW("ec_rx", "Unrouted pkt idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u",
                         dg->idx, (unsigned)dg->cmd, ado, adp, wkc);
            }
            unrouted_count++;
            if (rx_queue_ && rx_queue_->send(msg, 0)) {
#if TETHER_ENABLE_ETHERCAT_STATS
                rx_queue_sent_++;
#endif
            } else {
                static uint32_t drop_count = 0;
                if (drop_count < 10 || (drop_count % 500 == 0)) {
                    TETHER_LOGW("ec_rx", "RX queue full! Dropped idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u (total dropped: %u)",
                             dg->idx, (unsigned)dg->cmd, ado, adp, wkc, drop_count);
                }
                drop_count++;
            }
        }
    }
}

} // namespace EtherCAT
