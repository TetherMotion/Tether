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
    // Skip the entire fastpath reservation — PDO-slice slots (0xE0..0xEF),
    // cyclic slots (0xF8..0xFD) — plus the fire-and-forget index (0xFE),
    // i.e. everything >= kSliceSlotBaseIdx.  This also skips 0xFF,
    // leaving 0..0xDF for regular traffic.
    uint8_t idx;
    do { idx = next_idx_.fetch_add(1, std::memory_order_relaxed); }
    while (idx >= kSliceSlotBaseIdx);
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
    // Jumbo-aware ceiling — config_.max_frame_size is clamped in start(),
    // but clamp here too for pre-start/fallback calls.
    const size_t max_eth_frame = std::clamp<size_t>(
        config_.max_frame_size, 1514, kMaxJumboFrameSize);

    if (datalen > kMaxDatagramDataSize) {
        TETHER_LOGE(TAG,
            "Datagram exceeds the 11-bit protocol length limit (datalen={} "
            "> {}); split it into multiple datagrams",
            datalen, kMaxDatagramDataSize);
        return false;
    }

    const size_t required_len =
        sizeof(EtherCATSingleDgramFrameHeader) + datalen + sizeof(uint16_t);
    // Account for UDP encapsulation overhead in the max frame size check.
#if TETHER_ENABLE_UDP_ENCAPSULATION
    const size_t encap_overhead = config_.udp_encapsulation.enabled ? kUdpEncapOverhead : 0;
#else
    constexpr size_t encap_overhead = 0;
#endif
    if (required_len + encap_overhead > max_eth_frame) {
        TETHER_LOGE(TAG,
            "Datagram exceeds configured Ethernet frame size (datalen={} required={} encap={}, "
            "max_frame={}). Raise Config::max_frame_size for jumbo links.",
            datalen, static_cast<unsigned>(required_len),
            static_cast<unsigned>(encap_overhead), max_eth_frame);
        return false;
    }

    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;
    uint8_t txbuf[kMaxJumboFrameSize] = {0};

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
    constexpr size_t kHeaderSize = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
    const size_t max_payload = transport_
        ? transport_->maxEtherCATPayloadPerFrame()
        : static_cast<size_t>(Raw::kMaxEtherCATPayloadPerFrame);

    size_t frames_sent = 0;
    size_t i = 0;

    while (i < count) {
        uint8_t txbuf[kMaxJumboFrameSize] = {0};

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
            if (spec.datalen > kMaxDatagramDataSize) {
                TETHER_LOGE(TAG,
                    "sendMultiDatagram: datagram {} exceeds the 11-bit "
                    "protocol length limit (datalen={} > {}); split it into "
                    "multiple datagrams",
                    i + dg_count_in_frame, spec.datalen,
                    kMaxDatagramDataSize);
                return frames_sent;
            }
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
        // ecat_offset already accounts for an outer 802.1Q tag.
        const size_t ip_offset = ecat_offset;
        if (ip_offset + sizeof(IPv4Header) > length) return;

        const auto* ip = reinterpret_cast<const IPv4Header*>(frame + ip_offset);
        const uint8_t ihl = (ip->version_ihl & 0x0F) * 4;
        if (ihl < sizeof(IPv4Header) || ip_offset + ihl > length) return;
        if (ip->protocol != 0x11) return; // not UDP

        const size_t udp_offset = ip_offset + ihl;
        if (udp_offset + sizeof(UDPHeader) > length) return;

        const auto* udp = reinterpret_cast<const UDPHeader*>(frame + udp_offset);
        const uint16_t dst_port = bswap16(udp->dst_port_be);
        const uint16_t expected_port = config_.udp_encapsulation.enabled
            ? config_.udp_encapsulation.destination_port
            : kEtherCATOverUdpPort;
        if (dst_port != expected_port) return; // not EtherCAT-over-UDP

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

        if (isFastPathIdx(dg->idx)) {
            // Fastpath deposit (cyclic slot or PDO slice): fixed slot
            // keyed on the wire idx — no RxDatagram, no router.  The
            // cyclic thread waits on the slot's sequence counter.
            depositCyclicSlot(dg->idx, dg->cmd, adp, ado,
                              frame + data_offset, datalen, wkc,
                              static_cast<uint8_t>((len_flags >> 13) & 0x1u));
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

} // namespace EtherCAT
