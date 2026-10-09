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
// Index allocation
// ============================================================================

uint8_t Master::allocIdx()
{
    // Skip the entire cyclic band — the 100-index reservation at
    // 0x9C..0xFF (rotating pool + PDO-slice slots + 0xFE fire-and-forget
    // + 0xFF DC timepoint).  Async traffic keeps 0x00..0x9B.
    uint8_t idx;
    do { idx = next_idx_.fetch_add(1, std::memory_order_relaxed); }
    while (idx >= kFastSlotBaseIdx);
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
        int attempts = 0;
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
            ++attempts;
            if (sendWithEncapsulation(txbuf, frame_len)) return true;
            last_errno = errno;
#if TETHER_ENABLE_ETHERCAT_STATS
            tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
#endif
            // ENOBUFS = TX ring full and not draining (e.g. carrier
            // lost).  We only ever SEE this errno because the TX path
            // sets PACKET_QDISC_BYPASS — without it the qdisc would
            // silently absorb-and-drop frames and sendto() would keep
            // reporting success on a dead link.  ENOBUFS cannot recover
            // within microseconds — fail this cycle fast instead of
            // burning the budget spinning sendto.  The next exchange
            // cycle sends normally: no suppression state is kept, so
            // recovery is automatic when the ring drains.
            if (last_errno == ENOBUFS) break;
            const int64_t t0 = clock.getMicroseconds();
            while ((clock.getMicroseconds() - t0) <
                   static_cast<int64_t>(kTxRetryDelayUs)) {}
        }

        last_tx_errno_.store(last_errno, std::memory_order_relaxed);
        char msg[256];
        if (last_errno != 0) {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after %d attempt(s) (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u errno=%d:%s). "
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in EtherCATConfig.hpp; ENOBUFS fails fast).",
                          attempts,
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
                          "NetworkInterface::send failed after %d attempt(s) (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u errno=0). "
                          "Tether TX retry limit is %d (ECAT_TX_MAX_RETRIES in EtherCATConfig.hpp).",
                          attempts,
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

    // Test seam: without a live interface, drive the batch through the
    // register-level test callbacks — each datagram is answered by
    // apwr_cb_/aprd_cb_ and the synthesized response routed to its
    // pre-registered waiter.  A callback returning false aborts the
    // "frame", matching a wire send failure (callers see done < count).
    if (!iface_.send && (apwr_cb_ || aprd_cb_)) {
        size_t done = 0;
        for (size_t k = 0; k < count; ++k) {
            const auto& sp = specs[k];
            RxDatagram resp{};
            resp.idx     = sp.idx;
            resp.cmd     = sp.cmd;
            resp.adp     = sp.adp;
            resp.ado     = sp.ado;
            resp.datalen = sp.datalen;
            bool ok = false;
            switch (sp.cmd) {
                case Command::APWR:
                case Command::FPWR:
                    if (apwr_cb_)
                        ok = apwr_cb_(sp.adp, sp.ado, sp.data, sp.datalen, 0);
                    if (ok && sp.data && sp.datalen <= sizeof(resp.data))
                        std::memcpy(resp.data, sp.data, sp.datalen);  // write echo
                    break;
                case Command::APRD:
                case Command::FPRD:
                    if (aprd_cb_)
                        ok = aprd_cb_(sp.adp, sp.ado, resp.data,
                                      std::min<uint16_t>(sp.datalen,
                                                         sizeof(resp.data)),
                                      0);
                    break;
                default:
                    break;   // no test semantics for other commands
            }
            if (!ok) break;
            resp.wkc = 1;
            packet_router_.routePacket(resp);
            ++done;
        }
        return done;
    }

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
            int last_errno = 0;
            for (int retry = 0; retry <= kMaxTxRetries; retry++) {
                if (cancel_requested_.load(std::memory_order_acquire)) {
                    if (!cancel_warn_logged_.exchange(true, std::memory_order_acq_rel)) {
                        TETHER_LOGW(TAG, "sendMultiDatagram cancelled — "
                                          "further cancellation warnings suppressed");
                    }
                    return frames_sent;
                }
                errno = 0;
                if (sendWithEncapsulation(txbuf, frame_len)) { sent = true; break; }
                last_errno = errno;
#if TETHER_ENABLE_ETHERCAT_STATS
                tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
#endif
                // ENOBUFS cannot drain within the retry window — fail
                // fast this cycle; next cycle transmits normally (no
                // latch).  Note this errno only surfaces because TX uses
                // PACKET_QDISC_BYPASS — the qdisc would otherwise absorb
                // the frames and hide the dead link.
                if (last_errno == ENOBUFS) break;
                const int64_t t0 = clock.getMicroseconds();
                while ((clock.getMicroseconds() - t0) <
                       static_cast<int64_t>(kTxRetryDelayUs)) {}
            }
            if (!sent) {
                last_tx_errno_.store(last_errno, std::memory_order_relaxed);
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

} // namespace EtherCAT
