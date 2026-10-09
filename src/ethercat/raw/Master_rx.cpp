/**
 * @file Master_rx.cpp
 * @brief Master — response wait helpers and EtherCAT frame parsing.
 *
 * TU split out of Master_transport.cpp.
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
                // No registered waiter claimed this datagram — a stray
                // reply (timed-out transaction, unsolicited broadcast,
                // foreign traffic).  Discarding it is correct behaviour,
                // so this is an informational counter, not an error.
                const uint64_t un = unrouted_datagrams_.fetch_add(
                    1, std::memory_order_relaxed) + 1;
                if (un <= 10 &&
                    !cancel_requested_.load(std::memory_order_acquire)) {
                    TETHER_LOGW("ec_rx", "Unrouted pkt dg={} idx=0x{:02X} cmd=0x{:02X} ado=0x{:04X} adp=0x{:04X} wkc={}",
                             dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc);
                }
                if (rx_queue_ && rx_queue_->send(msg, 0)) {
#if TETHER_ENABLE_ETHERCAT_STATS
                    rx_queue_sent_.fetch_add(1, std::memory_order_relaxed);
#endif
                } else {
                    // Catch-all overflow — still only stray traffic (a
                    // routed reply never reaches this queue), so this is
                    // NOT packet loss; cyclic_health() reports it as such.
                    const uint64_t dropped = rx_queue_overflow_.fetch_add(
                        1, std::memory_order_relaxed) + 1;
                    if (dropped <= 10 || (dropped % 500 == 0)) {
                        TETHER_LOGD("ec_rx", "Unrouted queue full — discarded stray dg={} idx=0x{:02X} cmd=0x{:02X} ado=0x{:04X} adp=0x{:04X} wkc={} (total discarded: {})",
                                 dg_idx, dg->idx, (unsigned)dg->cmd, ado, adp, wkc, dropped);
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
