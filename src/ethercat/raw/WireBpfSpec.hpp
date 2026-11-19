/**
 * @file WireBpfSpec.hpp
 * @brief Shared WireEncap → CBPFSpec translation for the composed socket
 *        demux programs.
 *
 * CyclicDatapath::setup() composes encap ∧ idx-clause programs for the
 * cyclic/async socket pair; the proxy forwarder and the runtime
 * band re-attach need the same encapsulation clause on their sockets.
 * Keep the mapping in one place.
 *
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/Types.hpp"                 // WireEncap, kFastSlot*

namespace EtherCAT {

/**
 * CBPFSpec covering exactly the EtherCAT legs WireEncap describes:
 * untagged EtherCAT + any-VID tagged EtherCAT, or tagged-only restricted
 * to the configured VID range.  Mirrors attachEncapsulationBpfFilter's
 * spec construction (UDP legs are not part of WireEncap — callers that
 * need them compose them in).
 */
inline CBPFSpec wireEncapBpfSpec(const EtherCAT::WireEncap& we) {
    CBPFSpec spec;
    spec.vlan_delivery = we.delivery_hint;
    if (we.tagged()) {
        spec.untagged_ethercat = false;
        spec.tagged_ethercat   = true;
        if (!we.rx_vlan_any) {
            // A range clause only when a VID set was configured —
            // rx_any must accept every TCI like the main filter.
            spec.vlan_range = CBPFVlanRange{
                we.rx_vlan_lo,
                we.rx_vlan_hi ? we.rx_vlan_hi : we.rx_vlan_lo};
        }
    }
    return spec;
}

/**The reserved cyclic band [0x9C,0xFF] minus the 0xFE fire-and-forget
 * hole — the primary idx clause of the cyclic/async socket pair. */
inline CBPFIdxRange fastPathIdxRange() {
    return CBPFIdxRange{kFastSlotBaseIdx, kFastSlotEndIdx,
                        kFastSlotReservedIdx};   // 0xFE stays async
}

} // namespace EtherCAT
