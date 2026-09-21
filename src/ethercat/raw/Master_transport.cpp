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
#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/PacketDebugger.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>
#ifdef __linux__
#include <poll.h>
#include <unistd.h>
#endif

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
        if (debug_flags_.eeprom && (ado == 0x0502 || ado == 0x0508)) {
            TETHER_LOGW(TAG, "EEPROM: APWR sendDatagram FAILED adp=0x{:04X} ado=0x{:04X} len={}", adp, ado, len);
        }
        packet_router_.cancelPreRegistered(slot);
        return false;
    }

    WaitResult result = waitForPreRegistered(slot, timeout_ms);
    last_wkc_.store(result.wkc, std::memory_order_relaxed);
    if (debug_flags_.eeprom && (ado == 0x0502 || ado == 0x0508)) {
        TETHER_LOGI(TAG, "EEPROM: APWR adp=0x{:04X} ado=0x{:04X} len={} success={} wkc={}",
                    adp, ado, len, result.success, result.wkc);
    }
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
    if (debug_flags_.eeprom && (ado == 0x0502 || ado == 0x0508)) {
        TETHER_LOGI(TAG, "EEPROM: APRD adp=0x{:04X} ado=0x{:04X} len={} success={} wkc={} datalen={}",
                    adp, ado, len, result.success, result.wkc, result.data_length);
    }
    if (!result.success) return false;

    resp.datalen = static_cast<uint16_t>(result.data_length);
    if (resp.datalen < len) {
        if (debug_flags_.eeprom && (ado == 0x0502 || ado == 0x0508)) {
            TETHER_LOGW(TAG, "EEPROM: APRD adp=0x{:04X} ado=0x{:04X} datalen={} < len={}", adp, ado, resp.datalen, len);
        }
        return false;
    }
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
    // Skip the entire reserved cyclic-slot range (0xF8..0xFD) plus the
    // fire-and-forget index (0xFE) — i.e. everything >= kCyclicSlotBase.
    // This also skips 0xFF, leaving 0..0xF7 for regular traffic.
    uint8_t idx;
    do { idx = next_idx_.fetch_add(1, std::memory_order_relaxed); }
    while (idx >= IPDOTransport::kCyclicSlotBase);
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
// EtherCAT-over-UDP encapsulation helpers
// ============================================================================

#if TETHER_ENABLE_UDP_ENCAPSULATION
uint16_t Master::computeIpChecksum(const uint8_t* ip_header)
{
    return EtherCATTransport::computeIpChecksum(ip_header);
}

bool Master::encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                              uint8_t* out_buf, size_t out_cap, size_t* out_len) const
{
    if (!transport_) return false;
    return transport_->encapsulateFrame(in_frame, in_len, out_buf, out_cap, out_len);
}

bool Master::sendWithEncapsulation(const uint8_t* frame, size_t len)
{
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!transport_) {
        // Fallback before start() — direct send without encapsulation
        return iface_.send ? iface_.send(frame, len) : false;
    }
    return transport_->send(frame, len);
}

size_t Master::maxEtherCATPayloadPerFrame() const
{
    if (!transport_) return Raw::kMaxEtherCATPayloadPerFrame;
    return transport_->maxEtherCATPayloadPerFrame();
}
#endif // TETHER_ENABLE_UDP_ENCAPSULATION

// ============================================================================
// Transport primitives
// ============================================================================

bool Master::sendRawFrame(const void* buf, size_t len)
{
    if (debug_flags_.txPackets) {
        PacketDebugger::printEtherCATFrame(reinterpret_cast<const uint8_t*>(buf), len, true, false);
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
            "Datagram exceeds max Ethernet frame size (datalen={} required={} encap={}, "
            "max_frame={}). This is a physical Ethernet frame size limit, not a Tether buffer.",
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
            PacketDebugger::printEtherCATFrame(txbuf, frame_len, true, false);
        }
        auto& clock = Tether::Platform::Clock::instance();
        int last_errno = 0;
        for (int retry = 0; retry <= kMaxTxRetries; retry++) {
            if (cancel_requested_.load(std::memory_order_acquire)) {
                if (!cancel_warn_logged_.exchange(true, std::memory_order_acq_rel)) {
                    TETHER_LOGW(TAG, "sendSingleDatagram cancelled (cmd={} idx={}) — "
                                      "further cancellation warnings suppressed",
                                commandToString(cmd), static_cast<unsigned>(idx));
                }
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
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in EtherCATConfig.hpp).",
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
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in EtherCATConfig.hpp).",
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
                "sendMultiDatagram: datagram {} exceeds max Ethernet frame size (datalen={}, "
                "max_payload={}). This is a physical Ethernet frame size limit, not a Tether buffer.",
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
                PacketDebugger::printEtherCATFrame(txbuf, frame_len, true, false);
            }
            auto& clock = Tether::Platform::Clock::instance();
            bool sent = false;
            for (int retry = 0; retry <= kMaxTxRetries; retry++) {
                if (cancel_requested_.load(std::memory_order_acquire)) {
                    if (!cancel_warn_logged_.exchange(true, std::memory_order_acq_rel)) {
                        TETHER_LOGW(TAG, "sendMultiDatagram cancelled — "
                                          "further cancellation warnings suppressed");
                    }
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
    uint16_t ether_type = bswap16(eth->etherType_be);

    // The EtherCAT frame header and datagrams start right after the Ethernet
    // header for direct EtherCAT (EtherType 0x88A4).  For EtherCAT-over-UDP,
    // we need to skip the IPv4 + UDP headers first.
    size_t ecat_offset = sizeof(EtherCAT::EthernetHeader);

    // Q15: 802.1Q tag — the inner EtherType and payload sit 4 bytes deeper.
    // (Only reachable when NIC VLAN offload didn't strip the tag.)
    if (ether_type == 0x8100) {
        if (length < ecat_offset + 4 + sizeof(EtherCAT::FrameHeader)) return;
        ether_type = bswap16(*reinterpret_cast<const uint16_t*>(
            frame + ecat_offset + 2));
        ecat_offset += 4;
    }

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
            const char* name = PacketDebugger::etherTypeToString(ether_type);
            if (name) {
                TETHER_LOGI("ec_pkt", "[RX] Non-EtherCAT frame: {} (0x{:04X}, len={})",
                            name, ether_type, static_cast<unsigned>(length));
            } else {
                TETHER_LOGI("ec_pkt", "[RX] Non-EtherCAT frame: unknown (0x{:04X}, len={})",
                            ether_type, static_cast<unsigned>(length));
            }
        }
        return;
    }

    if (debug_flags_.rxPackets) {
        PacketDebugger::printEtherCATFrame(frame, length, false, false);
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

        if (dg->idx >= IPDOTransport::kCyclicSlotBase &&
            dg->idx <  IPDOTransport::kCyclicSlotBase + IPDOTransport::kNumCyclicSlots) {
            // Cyclic fast path: deposit into the fixed slot without building
            // an RxDatagram or touching the router.  The cyclic thread waits
            // on the slot's sequence counter.
            depositCyclicSlot(dg->idx - IPDOTransport::kCyclicSlotBase,
                              dg->cmd, adp, ado, frame + data_offset,
                              datalen, wkc);
        } else {
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
                if (unrouted_log_count_ < 10 &&
                    !cancel_requested_.load(std::memory_order_acquire)) {
                    TETHER_LOGW("ec_rx", "Unrouted pkt dg={} idx=0x{:02X} cmd=0x{:02X} ado=0x{:04X} adp=0x{:04X} wkc={}",
                             dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc);
                }
                unrouted_log_count_++;
                if (rx_queue_ && rx_queue_->send(msg, 0)) {
#if TETHER_ENABLE_ETHERCAT_STATS
                    rx_queue_sent_.fetch_add(1, std::memory_order_relaxed);
#endif
                } else {
                    rx_drop_log_count_++;
                    if (rx_drop_log_count_ <= 10 || (rx_drop_log_count_ % 500 == 0)) {
                        TETHER_LOGW("ec_rx", "RX queue full! Dropped dg={} idx=0x{:02X} cmd=0x{:02X} ado=0x{:04X} adp=0x{:04X} wkc={} (total dropped: {})",
                                 dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc, rx_drop_log_count_);
                    }
                }
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

// ============================================================================
// Cyclic fast path — reserved-index fixed response slots
// ============================================================================

void Master::depositCyclicSlot(uint8_t slot_idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc)
{
    auto& s = cyclic_slots_[slot_idx];
    // A copy-mode deposit replaces any held channel view — release the old
    // cookie first so ring slots aren't pinned forever.
    if (s.cookie >= 0 && cyclic_channel_) {
        cyclic_channel_->rxRelease(static_cast<uint32_t>(s.cookie));
    }
    s.cookie  = -1;
    s.payload = s.data;
    s.cmd     = static_cast<uint8_t>(cmd);
    s.adp     = adp;
    s.ado     = ado;
    s.datalen = datalen;
    s.wkc     = wkc;
    s.stamp_ns = Tether::Platform::Clock::instance().getMicroseconds() * 1000;
    if (datalen > 0) {
        std::memcpy(s.data, payload,
                    std::min<size_t>(datalen, sizeof(s.data)));
    }
    // seq_cst publish (Q13): the waiter's token compare must see a total
    // order with this bump — acquire/release leaves an S-store-buffer
    // window where a waiter can observe the new seq before the payload
    // fields on weak memory.
    s.seq.fetch_add(1, std::memory_order_seq_cst);

#ifdef __linux__
    // Wake a cyclic thread blocked in ppoll() — needed when the poll thread
    // won the recv race and this deposit happened on the "wrong" fd.  The
    // write is gated on a registered waiter: with the BPF demux working,
    // deposits almost never coincide with a sleep, so the idle path pays
    // no syscall per frame.
    if (cyclic_notify_fd_ >= 0 &&
        cyclic_waiters_.load(std::memory_order_acquire) > 0) {
        const uint64_t one = 1;
        ssize_t r = ::write(cyclic_notify_fd_, &one, sizeof(one));
        (void)r;  // EAGAIN (counter saturated) is fine — waiter already woken
    }
#endif
}

void Master::publishCyclicSlotView(uint8_t slot_idx, Command cmd,
                                   uint16_t adp, uint16_t ado,
                                   const uint8_t* payload, uint16_t datalen,
                                   uint16_t wkc, uint32_t cookie,
                                   uint64_t stamp_ns)
{
    auto& s = cyclic_slots_[slot_idx];
    if (s.cookie >= 0 && cyclic_channel_) {
        cyclic_channel_->rxRelease(static_cast<uint32_t>(s.cookie));
    }
    s.cookie  = static_cast<int64_t>(cookie);
    s.payload = payload;
    s.cmd     = static_cast<uint8_t>(cmd);
    s.adp     = adp;
    s.ado     = ado;
    s.datalen = datalen;
    s.wkc     = wkc;
    // The channel's kernel stamp when present, else stamp at deposit.
    s.stamp_ns = stamp_ns ? stamp_ns
        : Tether::Platform::Clock::instance().getMicroseconds() * 1000;
    s.seq.fetch_add(1, std::memory_order_seq_cst);   // see depositCyclicSlot

#ifdef __linux__
    if (cyclic_notify_fd_ >= 0 &&
        cyclic_waiters_.load(std::memory_order_acquire) > 0) {
        const uint64_t one = 1;
        ssize_t r = ::write(cyclic_notify_fd_, &one, sizeof(one));
        (void)r;
    }
#endif
}

uint64_t Master::cyclicSlotToken(uint8_t slot) const
{
    if (slot >= IPDOTransport::kNumCyclicSlots) return 0;
    return cyclic_slots_[slot].seq.load(std::memory_order_seq_cst);
}

namespace {

/// Build the 26-byte [eth][ecat][datagram-hdr] prefix of a cyclic frame.
void buildCyclicFrameHeader(uint8_t* buf, const uint8_t* src_mac,
                            EtherCAT::Command cmd, uint8_t slot,
                            uint16_t adp, uint16_t ado, uint16_t datalen,
                            bool roundtrip)
{
    using namespace EtherCAT::Raw;
    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    auto* hdr = reinterpret_cast<EtherCATSingleDgramFrameHeader*>(buf);
    std::memcpy(hdr->eth.dst, dst_mac, 6);
    std::memcpy(hdr->eth.src, src_mac, 6);
    hdr->eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);

    const uint16_t payload_len = static_cast<uint16_t>(
        sizeof(EtherCATDatagramHeader) + datalen + sizeof(uint16_t));
    constexpr uint16_t type = 0x1;
    hdr->ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu) | ((type & 0x0Fu) << 12)));

    hdr->dg.cmd    = cmd;
    hdr->dg.idx    = EtherCAT::IPDOTransport::kCyclicSlotBase + slot;
    hdr->dg.adp_le = host_to_le16(adp);
    hdr->dg.ado_le = host_to_le16(ado);
    const uint16_t flags = roundtrip ? (1u << 14) : 0u;
    hdr->dg.lenFlags.raw_le =
        host_to_le16(static_cast<uint16_t>((datalen & 0x07FFu) | flags));
    hdr->dg.irq_le = host_to_le16(0);
}

} // namespace

bool Master::sendCyclicDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    using namespace Raw;
    if (slot >= IPDOTransport::kNumCyclicSlots) return false;
    if (cancel_requested_.load(std::memory_order_acquire)) return false;

    // Channel path (socket backend): header on the stack + payload in place
    // → one sendmsg(), zero payload copies.  Ring backend would copy once
    // into a TX slot — callers in Rotating image mode use sendCyclicFrame
    // to avoid even that.
    if (cyclic_channel_) {
        uint8_t hdr_buf[kCyclicFramePayloadOff];
        buildCyclicFrameHeader(hdr_buf, src_mac_, cmd, slot, adp, ado,
                               datalen, roundtrip);
        CyclicTxParts parts;
        parts.header      = hdr_buf;
        parts.header_len  = kCyclicFramePayloadOff;
        parts.payload     = static_cast<const uint8_t*>(data);
        parts.payload_len = datalen;
        parts.wkc         = 0;
        if (debug_flags_.txPackets) {
            // Compose into cyclic_tx_buf_ purely for the debug dump.
            std::memcpy(cyclic_tx_buf_, hdr_buf, sizeof(hdr_buf));
            uint8_t* p = cyclic_tx_buf_ + sizeof(hdr_buf);
            if (datalen > 0) std::memcpy(p, data, datalen);
            *reinterpret_cast<uint16_t*>(p + datalen) = host_to_le16(0);
            PacketDebugger::printEtherCATFrame(cyclic_tx_buf_,
                kCyclicFramePayloadOff + datalen + sizeof(uint16_t),
                true, false);
        }
        return cyclic_channel_->txSendParts(parts);
    }

    constexpr size_t kMinEthFrameNoFcs = 60;

    const size_t required_len =
        sizeof(EtherCATSingleDgramFrameHeader) + datalen + sizeof(uint16_t);
    if (required_len > sizeof(cyclic_tx_buf_)) {
        return false;
    }
    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;

    buildCyclicFrameHeader(cyclic_tx_buf_, src_mac_, cmd, slot, adp, ado,
                           datalen, roundtrip);

    uint8_t* payload = cyclic_tx_buf_ + sizeof(EtherCATSingleDgramFrameHeader);
    if (datalen > 0) {
        if (data) std::memcpy(payload, data, datalen);
        else      std::memset(payload, 0, datalen);
    }
    *reinterpret_cast<uint16_t*>(payload + datalen) = host_to_le16(0);

    if (debug_flags_.txPackets) {
        PacketDebugger::printEtherCATFrame(cyclic_tx_buf_, frame_len, true, false);
    }
    // Single attempt — no retry spin on the cyclic path; a missed cycle is
    // reported via the error counter rather than retried inside the deadline.
    return sendWithEncapsulation(cyclic_tx_buf_, frame_len);
}

uint8_t* Master::acquireCyclicTxFrame()
{
    return cyclic_channel_ ? cyclic_channel_->txAcquire() : nullptr;
}

bool Master::sendCyclicFrame(uint32_t frame_len)
{
    if (!cyclic_channel_) return false;
    return cyclic_channel_->txCommitFrame(frame_len);
}

void Master::composeCyclicHeader(uint8_t* frame, Command cmd, uint8_t slot,
                                 uint16_t adp, uint16_t ado, uint16_t datalen,
                                 bool roundtrip)
{
    buildCyclicFrameHeader(frame, src_mac_, cmd, slot, adp, ado,
                           datalen, roundtrip);
}

/**
 * @brief Walk a frame received on the cyclic channel.
 *
 * Pure-cyclic frames (all datagrams idx >= kCyclicSlotBase) publish their
 * payload views into the slots without copying.  Anything else (async or
 * mixed frames — only possible when the BPF demux is absent/failed) is
 * routed through the regular parser, which deposits/routes per datagram.
 */
void Master::dispatchChannelFrame(const CyclicFrameView& v)
{
    using namespace Raw;
    const uint8_t* f    = v.frame;
    const size_t   flen = v.frame_len;
    constexpr size_t kEcatHdr = sizeof(EtherCAT::EthernetHeader);       // 14
    constexpr size_t kDgHdr   = sizeof(EtherCAT::DatagramHeader);       // 10
    // Q15: with the VLAN-aware demux a 0x8100-tagged frame can arrive on
    // the cyclic socket — the EtherCAT header sits 4 bytes deeper.
    size_t base = kEcatHdr;
    if (flen >= kEcatHdr + 4 &&
        f[12] == 0x81 && f[13] == 0x00) {
        base += 4;
    }
    if (flen < base + sizeof(EtherCAT::FrameHeader)) return;

    const uint16_t ec_len = le16_to_host(
        *reinterpret_cast<const uint16_t*>(f + base)) & 0x07FFu;
    if (flen < base + sizeof(EtherCAT::FrameHeader) + ec_len) return;

    // Classify pass: any non-cyclic idx → whole frame to the parser.
    size_t off = base + sizeof(EtherCAT::FrameHeader);
    size_t rem = ec_len;
    bool   all_cyclic = true;
    while (rem >= kDgHdr + sizeof(uint16_t) && off + kDgHdr <= flen) {
        const uint8_t  idx       = f[off + 1];
        const uint16_t len_flags = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 6));
        const uint16_t dl   = len_flags & 0x07FFu;
        const bool     more = (len_flags & 0x8000u) != 0;
        if (idx <  IPDOTransport::kCyclicSlotBase ||
            idx >= IPDOTransport::kCyclicSlotBase +
                   IPDOTransport::kNumCyclicSlots) {
            all_cyclic = false;
            break;
        }
        const size_t dgt = kDgHdr + dl + sizeof(uint16_t);
        if (rem < dgt || off + dgt > flen) { all_cyclic = false; break; }
        rem -= dgt; off += dgt;
        if (!more) break;
    }

    if (!all_cyclic) {
        handleRxFrame(f, flen);   // parser deposits/routes per datagram
        return;
    }

    // Publish pass — every datagram is cyclic; hand payload views to slots.
    off = base + sizeof(EtherCAT::FrameHeader);
    rem = ec_len;
    while (rem >= kDgHdr + sizeof(uint16_t) && off + kDgHdr <= flen) {
        const uint8_t  idx       = f[off + 1];
        const uint8_t  cmd       = f[off];
        const uint16_t adp       = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 2));
        const uint16_t ado       = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 4));
        const uint16_t len_flags = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 6));
        const uint16_t dl   = len_flags & 0x07FFu;
        const bool     more = (len_flags & 0x8000u) != 0;
        const size_t   data_off = off + kDgHdr;
        const size_t   dgt      = kDgHdr + dl + sizeof(uint16_t);
        if (rem < dgt || off + dgt > flen) break;
        const uint16_t wkc = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + data_off + dl));

        cyclic_channel_->rxHold(v.cookie);
        publishCyclicSlotView(idx - IPDOTransport::kCyclicSlotBase,
                              static_cast<Command>(cmd), adp, ado,
                              f + data_off, dl, wkc, v.cookie, v.stamp_ns);
        rem -= dgt; off += dgt;
        if (!more) break;
    }
}

bool Master::waitCyclicSlotView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    if (slot >= IPDOTransport::kNumCyclicSlots) return false;
    auto& s = cyclic_slots_[slot];

    auto read_slot = [&]() -> bool {
        // Seqlock read: the publisher writes the fields then bumps seq
        // (release).  Re-check seq after the read — a second publish
        // landing mid-read is detected and retried.
        for (int tries = 0; tries < 8; ++tries) {
            const uint64_t s0 = s.seq.load(std::memory_order_acquire);
            out.cmd     = s.cmd;
            out.adp     = s.adp;
            out.ado     = s.ado;
            out.datalen = s.datalen;
            out.wkc     = s.wkc;
            out.stamp_ns = s.stamp_ns;
            out.payload = s.payload ? s.payload : s.data;
            const int64_t cookie = s.cookie;
            out.cookie  = cookie >= 0 ? static_cast<uint32_t>(cookie) : 0;
            out.channel = cookie >= 0 ? cyclic_channel_.get() : nullptr;
            if (s.seq.load(std::memory_order_acquire) == s0) return true;
        }
        return true;   // accept last read — publisher is single-writer
    };

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    // Fast path: response already deposited.  seq_cst pairs with the
    // deposit's publish bump (Q13).
    if (s.seq.load(std::memory_order_seq_cst) != token) {
        return read_slot();
    }

    // Register as a waiter BEFORE any blocking point so a deposit landing
    // between the fast-path check and the sleep still wakes us (deposit
    // paths write the eventfd only while cyclic_waiters_ > 0).  The seq
    // re-check inside the loop covers deposits that raced ahead of the
    // registration.
    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{cyclic_waiters_};
    cyclic_waiters_.fetch_add(1, std::memory_order_acq_rel);

    // Unified wait: one ppoll over every wake source — the deposit eventfd
    // (covers frames consumed by the poll thread / socket-B deposits when
    // the BPF is absent) and the wire fd (the channel's own socket, or the
    // iface socket on the no-channel path).
#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (cyclic_notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { cyclic_notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (cyclic_channel_) {
        wire_fd = cyclic_channel_->fd();
    } else if (iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        if (s.seq.load(std::memory_order_seq_cst) != token) {
            return read_slot();
        }
        if (cancel_requested_.load(std::memory_order_acquire)) return false;
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        int64_t remain = deadline_ns - now_ns;
        if (remain <= 0) return false;

        if (cyclic_channel_) {
            // Spin phase: the ring backend's rxPending() is pure memory
            // reads — a NIC DMA write becomes visible before the kernel
            // could ever wake a ppoll() sleeper.  Shares the deadline
            // budget; skipped entirely when slot_spin_ns_ == 0.  (This is
            // the *slot-wait* spin — rx_spin_ns_ is the channel's own
            // rxPoll busy-poll, a separate knob, Q11.)
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(remain, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    if (s.seq.load(std::memory_order_seq_cst) != token) {
                        return read_slot();
                    }
                    if (cyclic_channel_->rxPending()) break;
                }
            }
            CyclicFrameView views[8];
            const int n = cyclic_channel_->rxPoll(views, 8, 0);
            for (int i = 0; i < n; ++i) dispatchChannelFrame(views[i]);
            if (n > 0) continue;      // re-check slots before sleeping
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;   // deadline check at loop top
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            if (ret == 0) return false;  // deadline reached

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (cyclic_channel_) {
                    CyclicFrameView views[8];
                    const int n = cyclic_channel_->rxPoll(views, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchChannelFrame(views[i]);
                } else {
                    // Bounded drain: an async burst must not burn the
                    // whole RX budget — POLLIN stays asserted so the
                    // remaining frames are handled next iteration.
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(cyclic_notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        // No fd wake-up path (non-Linux / no eventfd / no wire fd): loop —
        // a bounded spin on the sequence counter plus channel drain, with
        // the seq/cancel/deadline checks at the top bounding it so a dead
        // link cannot wedge the cyclic thread.  Q22: the fallback policy
        // is user-configurable — Yield (default) drops the thread's time
        // slice each pass; Spin keeps the tightest re-check cadence at
        // full CPU burn.
        if (slot_wait_fallback_ ==
            CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
}

uint32_t Master::waitCyclicSlotMask(uint32_t slot_mask,
                                    const uint64_t* tokens,
                                    uint32_t timeout_ns,
                                    CyclicSlotView* views)
{
    // Q3: one wake for the whole mask — a sliced collect pays a single
    // ppoll registration instead of one sleep per slice.  Each wake
    // re-scans all outstanding bits; a deposit on ANY slot is progress.
    slot_mask &= (1u << IPDOTransport::kNumCyclicSlots) - 1u;
    if (!slot_mask || !tokens || !views) return 0;

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    // Outstanding bits → arrived so far; fill views for arrived slots.
    auto scan = [&]() -> uint32_t {
        uint32_t outstanding = 0;
        for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots; ++s) {
            if (!(slot_mask & (1u << s))) continue;
            if (cyclic_slots_[s].seq.load(std::memory_order_seq_cst)
                != tokens[s]) continue;
            outstanding |= 1u << s;
        }
        return outstanding;
    };
    auto fill = [&](uint32_t arrived) {
        for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots; ++s) {
            if (!(arrived & (1u << s))) continue;
            auto& slot = cyclic_slots_[s];
            for (int tries = 0; tries < 8; ++tries) {
                const uint64_t s0 = slot.seq.load(std::memory_order_acquire);
                views[s].cmd     = slot.cmd;
                views[s].adp     = slot.adp;
                views[s].ado     = slot.ado;
                views[s].datalen = slot.datalen;
                views[s].wkc     = slot.wkc;
                views[s].stamp_ns = slot.stamp_ns;
                views[s].payload = slot.payload ? slot.payload : slot.data;
                const int64_t cookie = slot.cookie;
                views[s].cookie  = cookie >= 0
                    ? static_cast<uint32_t>(cookie) : 0;
                views[s].channel = cookie >= 0 ? cyclic_channel_.get()
                                               : nullptr;
                if (slot.seq.load(std::memory_order_acquire) == s0) break;
            }
        }
    };

    uint32_t outstanding = scan();
    if (!outstanding) { fill(slot_mask); return slot_mask; }

    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{cyclic_waiters_};
    cyclic_waiters_.fetch_add(1, std::memory_order_acq_rel);

#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (cyclic_notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { cyclic_notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (cyclic_channel_) {
        wire_fd = cyclic_channel_->fd();
    } else if (iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        outstanding = scan();
        if (!outstanding) { fill(slot_mask); return slot_mask; }
        if (cancel_requested_.load(std::memory_order_acquire)) {
            const uint32_t arrived = slot_mask & ~outstanding;
            fill(arrived);
            return arrived;
        }
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        if (deadline_ns - now_ns <= 0) {
            const uint32_t arrived = slot_mask & ~outstanding;
            fill(arrived);
            return arrived;
        }

        if (cyclic_channel_) {
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(deadline_ns - now_ns, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    bool any = false;
                    for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots;
                         ++s) {
                        if (!(slot_mask & (1u << s))) continue;
                        if (cyclic_slots_[s].seq.load(
                                std::memory_order_seq_cst) != tokens[s]) {
                            any = true; break;
                        }
                    }
                    if (any || cyclic_channel_->rxPending()) break;
                }
            }
            CyclicFrameView fviews[8];
            const int n = cyclic_channel_->rxPoll(fviews, 8, 0);
            for (int i = 0; i < n; ++i) dispatchChannelFrame(fviews[i]);
            if (n > 0) continue;
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                outstanding = scan();
                const uint32_t arrived = slot_mask & ~outstanding;
                fill(arrived);
                return arrived;
            }
            if (ret == 0) break;   // deadline reached — final scan below

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (cyclic_channel_) {
                    CyclicFrameView fviews[8];
                    const int n = cyclic_channel_->rxPoll(fviews, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchChannelFrame(fviews[i]);
                } else {
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(cyclic_notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        if (slot_wait_fallback_ ==
            CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
    // ppoll deadline expired — one last scan, then report what arrived.
    outstanding = scan();
    const uint32_t arrived = slot_mask & ~outstanding;
    fill(arrived);
    return arrived;
}

bool Master::waitCyclicSlot(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out)
{
    CyclicSlotView view{};
    if (!waitCyclicSlotView(slot, token, timeout_ns, view)) return false;
    out.idx     = IPDOTransport::kCyclicSlotBase + slot;
    out.cmd     = static_cast<Command>(view.cmd);
    out.adp     = view.adp;
    out.ado     = view.ado;
    out.datalen = view.datalen;
    out.wkc     = view.wkc;
    if (view.datalen > 0 && view.payload) {
        std::memcpy(out.data, view.payload,
                    std::min<size_t>(view.datalen, sizeof(out.data)));
    }
    return true;
}

} // namespace EtherCAT
