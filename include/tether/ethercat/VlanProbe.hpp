#pragma once

/**
 * @file VlanProbe.hpp
 * @brief Startup probe: how does the kernel deliver 802.1Q-tagged EtherCAT
 *        replies — tag inline in the frame bytes, or stripped into skb
 *        auxdata (PACKET_AUXDATA / SKF_AD_VLAN_*)?
 *
 * When VLAN encapsulation is enabled (--encapsulation vlan:N /
 * Master::Config::wire_encap), the answer decides which leg of the
 * composed cBPF program and which RX parsing path actually carries
 * traffic.  Sending a bogus-logical-address LRW datagram exercises the
 * real path end to end: slaves forward it around the segment unchanged
 * (WKC=0, no FMMU match), so the reply shows exactly how the kernel
 * presents tagged frames to this socket on this NIC/driver.
 *
 * Usage:
 *   - Run BEFORE attaching socket filters and BEFORE starting any RX
 *     consumer thread (the probe eats whatever it reads).
 *   - NoReply after `timeout_ms` means the segment is not a closed loop
 *     or the configured VID is wrong — fail startup loudly.
 *
 * classifyVlanReply() is a pure function so the wire-format logic is
 * testable without CAP_NET_RAW.
 */

#include <cstddef>
#include <cstdint>
#include <optional>

#include "tether/ethercat/Types.hpp"   // VlanDeliveryHint

namespace EtherCAT {
namespace HAL { class IEthernet; }

/// How the reply's 802.1Q tag was presented to the socket.
enum class VlanTagDelivery {
    NoReply,     ///< nothing matching came back within the timeout
    Untagged,    ///< reply arrived without any tag
    InlineTag,   ///< tag present in the frame bytes ([12] = TPID)
    StrippedTag, ///< kernel stripped the tag into PACKET_AUXDATA
};

const char* toString(VlanTagDelivery d);

/// Map a detected delivery mode to the filter-generation hint used by
/// CBPFSpec / WireEncap.  Ambiguous results (NoReply, Untagged) map to
/// Auto — the safe both-legs program.
inline VlanDeliveryHint vlanDeliveryHint(VlanTagDelivery d) {
    switch (d) {
    case VlanTagDelivery::StrippedTag: return VlanDeliveryHint::StrippedOnly;
    case VlanTagDelivery::InlineTag:   return VlanDeliveryHint::InlineOnly;
    default:                         return VlanDeliveryHint::Auto;
    }
}

struct VlanProbeResult {
    VlanTagDelivery delivery = VlanTagDelivery::NoReply;
    uint16_t vid     = 0;    ///< VID observed on the reply (0 if none)
    uint16_t tpid    = 0;    ///< TPID observed (inline [12] or auxdata)
    int64_t  rtt_us  = -1;   ///< probe round-trip time (-1 on NoReply)
};

/**
 * @brief Classify one received frame's VLAN delivery mode.
 *
 * @param frame          raw Ethernet frame starting at the MAC header
 * @param len            frame length
 * @param aux_vlan_valid PACKET_AUXDATA reported TP_STATUS_VLAN_VALID
 * @param aux_tci        auxdata tp_vlan_tci (VID = tci & 0x0FFF)
 * @param aux_tpid       auxdata tp_vlan_tpid when TP_STATUS_VLAN_TPID_VALID
 * @param out_vid        receives the observed VID (0 when none)
 * @param out_tpid       receives the observed TPID (0 when none)
 */
VlanTagDelivery classifyVlanReply(const uint8_t* frame, size_t len,
                                  bool aux_vlan_valid, uint16_t aux_tci,
                                  uint16_t aux_tpid,
                                  uint16_t& out_vid, uint16_t& out_tpid);

/**
 * @brief Send a bogus-logical-address LRW probe and wait for its return.
 *
 * Sends one LRW datagram (idx @ref kVlanProbeIdx, logical address
 * @ref kVlanProbeLogicalAddr — deliberately unmapped so slaves forward it
 * untouched with WKC=0), optionally 802.1Q-tagged with @p vid (vid == 0
 * sends untagged).  Waits up to @p timeout_ms (default 10) for a matching
 * reply, then classifies the tag delivery.
 *
 * @param fd         bound AF_PACKET socket (e.g. an IEthernet's
 *                   nativeHandle()).  PACKET_AUXDATA is enabled on it for
 *                   the duration of the probe if not already.
 * @param ifindex    interface index the socket is bound to
 * @param src_mac    source MAC to put on the probe frame (may be zeros —
 *                   slaves forward regardless)
 * @param vid        VLAN id to tag the probe with; 0 = untagged probe
 * @param timeout_ms reply window (default 10 ms per the spec)
 * @return the result, or std::nullopt when the probe itself could not run
 *         (unsupported platform, unusable fd, send failure).  A probe that
 *         ran but saw no reply yields delivery == NoReply.
 */
inline constexpr uint8_t  kVlanProbeIdx         = 0x2C;   ///< outside fastpath
inline constexpr uint32_t kVlanProbeLogicalAddr = 0xDEADBEEFu;

std::optional<VlanProbeResult> probeVlanTagDelivery(
    int fd, int ifindex, const uint8_t src_mac[6], uint16_t vid,
    uint32_t timeout_ms = 10);

/**
 * @brief Convenience overload: derive fd, ifindex and source MAC from an
 *        initialized HAL device.  Same contract as the fd overload.
 */
std::optional<VlanProbeResult> probeVlanTagDelivery(
    HAL::IEthernet& eth, uint16_t vid, uint32_t timeout_ms = 10);

} // namespace EtherCAT
