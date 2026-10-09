/**
 * @file EtherCATProtocolConstants.hpp
 * @brief EtherCAT types: Protocol Constants
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// Protocol Constants
// ============================================================================

/// EtherType for EtherCAT frames (big-endian on wire)
constexpr uint16_t kEtherTypeEtherCAT = 0x88A4;

/// Maximum EtherCAT datagram data size — the datagram length field is
/// 11 bits (0x07FF), so a single datagram can never exceed 2047 bytes
/// regardless of the link MTU.  Carrying a full-size datagram requires a
/// jumbo frame (see Master::Config::max_frame_size).
constexpr size_t kMaxDatagramDataSize = 2047;

/// Maximum Ethernet frame size
constexpr size_t kMaxFrameSize = 1518;

/// Maximum jumbo Ethernet frame size (incl. 14-byte header, excl. FCS):
/// 9000-byte MTU + Ethernet header.
constexpr size_t kMaxJumboFrameSize = 9014;

/// EtherCAT destination MAC (broadcast)
constexpr std::array<uint8_t, 6> kEtherCATBroadcastMAC = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

} // namespace EtherCAT
