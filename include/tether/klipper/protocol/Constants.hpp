/**
 * @file Constants.hpp
 * @brief Klipper wire-protocol framing constants.
 *
 * @details
 * These constants describe the on-the-wire message-block frame used by the
 * Klipper messaging protocol. They are derived purely from the protocol
 * specification and are independent of any particular Klipper source code.
 *
 * A message block has a 2-byte header, a variable-length content region, and
 * a 3-byte trailer:
 *
 *   +-------+-------+------------------+-----+------+------+-------+
 *   | LEN   | SEQ   | CONTENT ...          | CRC_HI | CRC_LO | SYNC |
 *   +-------+-------+------------------+-----+------+------+-------+
 *
 * @see MessageBlock.hpp for frame build/parse helpers.
 */

#pragma once

#include <cstdint>

namespace tether::klipper::protocol {

/// Minimum message-block length in bytes (header + trailer, empty content).
inline constexpr uint8_t kMinBlockLength = 5;

/// Maximum message-block length in bytes.
inline constexpr uint8_t kMaxBlockLength = 64;

/// Header size in bytes (LEN + SEQ).
inline constexpr uint8_t kHeaderSize = 2;

/// Trailer size in bytes (CRC_HI + CRC_LO + SYNC).
inline constexpr uint8_t kTrailerSize = 3;

/// Maximum content size in bytes (kMaxBlockLength - header - trailer).
inline constexpr uint8_t kMaxContentLength = kMaxBlockLength - kHeaderSize - kTrailerSize; // 59

/// Sync byte used to find block boundaries after corruption.
inline constexpr uint8_t kSyncByte = 0x7E;

/// Mask for the 4-bit sequence counter in the SEQ byte.
inline constexpr uint8_t kSequenceMask = 0x0F;

/// Destination marker (high nibble of the SEQ byte, always 0x1).
inline constexpr uint8_t kSequenceDestMarker = 0x10;

/// Hard-coded msgid for the `identify_response` message (MCU -> host).
inline constexpr uint16_t kMsgIdIdentifyResponse = 0;

/// Hard-coded msgid for the `identify` command (host -> MCU).
inline constexpr uint16_t kMsgIdIdentify = 1;

/// First non-hard-coded encoded msgid.
inline constexpr uint16_t kFirstDynamicMsgId = 2;

/// Maximum encoded msgid value (fits in 2 wire bytes).
inline constexpr uint16_t kMaxMsgId = 0x3FFF; // 16383

/// Maximum buffer/string parameter length in bytes (limited by block size).
inline constexpr uint8_t kMaxBufferLength = 64;

/// Default chunk size used by the identify handshake (bytes per request).
inline constexpr uint8_t kDefaultIdentifyChunkSize = 40;

/// Default host-side maximum number of unacknowledged blocks in flight.
inline constexpr uint8_t kDefaultMaxPendingBlocks = 12;

/// Minimum retransmission timeout in seconds.
inline constexpr double kMinRtoSeconds = 0.025;

/// Maximum retransmission timeout in seconds.
inline constexpr double kMaxRtoSeconds = 5.0;

/// Default retry interval when the send window is full (seconds).
inline constexpr double kWindowFullRetrySeconds = 0.250;

} // namespace tether::klipper::protocol
