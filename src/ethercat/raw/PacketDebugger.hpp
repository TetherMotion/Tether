// SPDX-License-Identifier: MIT
/**
 * @file PacketDebugger.hpp
 * @brief Stateless EtherCAT packet debug printing utilities
 *
 * @details
 * Provides human-readable pretty-printing of raw EtherCAT frames for
 * diagnostic logging.  These functions are pure/stateless — they operate
 * only on the frame buffer passed to them and have no dependency on the
 * Master class instance state.
 *
 * Extracted from Master_transport.cpp to improve modularity.
 *
 * @internal
 * This is an internal header used by the EtherCAT raw transport
 * implementation.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace EtherCAT {
namespace PacketDebugger {

/// Map an EtherType value to a human-readable string.
/// Returns nullptr for unknown EtherTypes.
const char* etherTypeToString(uint16_t ether_type);

/// Pretty-print an EtherCAT frame (Ethernet header + EtherCAT header +
/// datagrams) to the log.  Handles both direct EtherCAT (EtherType 0x88A4)
/// and EtherCAT-over-UDP encapsulation.
///
/// @param frame          Raw frame buffer starting at the Ethernet header
/// @param length         Total frame length in bytes
/// @param is_tx          True if this is a transmitted frame (TX), false for RX
/// @param print_ethernet If true, print the Ethernet header fields
void printEtherCATFrame(const uint8_t* frame, size_t length,
                        bool is_tx, bool print_ethernet);

} // namespace PacketDebugger
} // namespace EtherCAT
