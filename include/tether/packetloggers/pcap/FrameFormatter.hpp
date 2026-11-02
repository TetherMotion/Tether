// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file FrameFormatter.hpp
 * @brief Human-readable and JSON formatting for InterpretedFrame, extracted
 *        from PCAPNGReader.
 *
 * @details
 * Encapsulates the formatting sub-responsibility of PCAPNGReader:
 *  - Human-readable text rendering of an InterpretedFrame
 *  - Compact JSON rendering of an InterpretedFrame
 *  - Helper conversions (MAC address, IP address, byte-span to hex)
 *
 * These functions are pure / stateless and depend only on the InterpretedFrame
 * data structure and the EtherCAT command-to-string helper.
 */

#include "packetloggers/pcap/PCAPNGReader.hpp" // InterpretedFrame, PacketDirection
#include "ethercat/Types.hpp"                  // EtherCAT::commandToString

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

/**
 * @brief Format an interpreted frame as human-readable text.
 * @param frame         Frame to format.
 * @param verbose       If true, include full payload hex dumps.
 * @param maxDataBytes  Maximum payload bytes to dump per datagram (0 = no limit).
 * @return formatted multi-line text representation.
 */
std::string formatInterpretedFrame(const InterpretedFrame& frame,
                                   bool verbose = false,
                                   size_t maxDataBytes = 64);

/**
 * @brief Format an interpreted frame as compact JSON.
 * @param frame  Frame to serialize.
 * @return single-line JSON string.
 */
std::string frameToJson(const InterpretedFrame& frame);

/**
 * @brief Convert a MAC address to a colon-separated hex string.
 * @param mac  6-byte MAC address.
 * @return "xx:xx:xx:xx:xx:xx" string.
 */
std::string macToString(const std::array<uint8_t, 6>& mac);

/**
 * @brief Convert an IPv4 address (host byte order) to dotted-quad string.
 * @param ip  IPv4 address in host byte order.
 * @return "a.b.c.d" string.
 */
std::string ipToString(uint32_t ip);

/**
 * @brief Convert a byte span to a hex string with optional separator.
 * @param data       Bytes to dump.
 * @param length     Byte count.
 * @param separator  Inserted between bytes (default single space).
 * @return lowercase hex string.
 */
std::string bytesToHex(const uint8_t* data, size_t length,
                       const std::string& separator = " ");

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
