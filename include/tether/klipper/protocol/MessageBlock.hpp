/**
 * @file MessageBlock.hpp
 * @brief Klipper message-block frame build/parse.
 *
 * @details
 * A message block has a 2-byte header, a variable-length content region, and
 * a 3-byte trailer:
 *
 *   +-------+-------+------------------+------+------+-------+
 *   | LEN   | SEQ   | CONTENT ...          | CRC_HI | CRC_LO | SYNC |
 *   +-------+-------+------------------+------+------+-------+
 *
 * LEN is the total block length (5..64). SEQ low nibble is the 4-bit sequence
 * counter; the high nibble is always 0x1 (destination marker 0x10). CRC is
 * big-endian and covers header + content. SYNC is 0x7E.
 *
 * The content region holds one or more encoded messages (msgid + parameters).
 * This header provides frame-level build/parse only; message (de)serialisation
 * of parameters is handled by CommandTable/ParameterFormat.
 *
 * @see Constants.hpp, Crc16.hpp, Vlq.hpp
 */

#pragma once

#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/protocol/Crc16.hpp"

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>
#include <optional>
#include <algorithm>

namespace tether::klipper::protocol {

/**
 * @brief A decoded message-block frame (header + content; trailer stripped).
 */
struct MessageBlock {
    uint8_t sequence = 0;          ///< 4-bit sequence counter
    std::vector<uint8_t> content;  ///< Content bytes (0..59)

    /// @return Total wire length this block would occupy.
    size_t wireLength() const { return kHeaderSize + content.size() + kTrailerSize; }
};

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

/**
 * @brief Build a complete message block (header + content + trailer) into a
 *        flat byte buffer.
 *
 * @param sequence 4-bit sequence counter (0..15).
 * @param content  Content bytes (size must be <= kMaxContentLength).
 * @param out       Output buffer (must hold at least wireLength() bytes).
 * @return Number of bytes written, or 0 if content too large / sequence invalid.
 */
inline size_t buildBlock(uint8_t sequence, std::span<const uint8_t> content, uint8_t* out) {
    if (content.size() > kMaxContentLength) return 0;
    if (sequence > 0x0F) return 0;
    const size_t total = kHeaderSize + content.size() + kTrailerSize;
    if (total > kMaxBlockLength) return 0;

    out[0] = static_cast<uint8_t>(total);                 // LEN
    out[1] = static_cast<uint8_t>(kSequenceDestMarker | (sequence & kSequenceMask)); // SEQ
    for (size_t i = 0; i < content.size(); ++i) {
        out[kHeaderSize + i] = content[i];
    }
    const size_t crcLen = kHeaderSize + content.size();
    const uint16_t crc = crc16Ccitt(out, crcLen);
    out[kHeaderSize + content.size() + 0] = static_cast<uint8_t>((crc >> 8) & 0xFF); // CRC_HI
    out[kHeaderSize + content.size() + 1] = static_cast<uint8_t>(crc & 0xFF);        // CRC_LO
    out[kHeaderSize + content.size() + 2] = kSyncByte;                               // SYNC
    return total;
}

/**
 * @brief Build a complete message block into a std::vector.
 */
inline std::vector<uint8_t> buildBlockVec(uint8_t sequence, std::span<const uint8_t> content) {
    std::vector<uint8_t> out(kMaxBlockLength);
    size_t n = buildBlock(sequence, content, out.data());
    out.resize(n);
    return out;
}

/**
 * @brief Build an empty ack/nak block (5 bytes).
 * @param sequence 4-bit sequence counter.
 */
inline std::vector<uint8_t> buildAckBlock(uint8_t sequence) {
    return buildBlockVec(sequence, {});
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

/**
 * @brief Result of attempting to parse a block from a byte stream.
 */
enum class BlockParseStatus : uint8_t {
    Ok,            ///< Block parsed successfully
    NeedMoreData,  ///< Not enough bytes yet for a complete block
    BadSync,       ///< Trailer sync byte mismatch
    BadCrc,        ///< CRC mismatch (corrupt block)
    BadLength,     ///< LEN field out of range
};

/**
 * @brief Parsed block result.
 */
struct ParsedBlock {
    BlockParseStatus status = BlockParseStatus::NeedMoreData;
    MessageBlock block;           ///< Valid when status == Ok
    size_t consumedBytes = 0;     ///< Bytes consumed from the input (to advance buffer)
    size_t wireLength = 0;        ///< Total wire length of the parsed block (0 if not Ok)
    size_t skippedBytes = 0;      ///< Garbage bytes skipped before finding this block
};

/**
 * @brief Attempt to parse one message block from the front of a byte buffer.
 *
 * Scans for a valid block: it reads LEN, validates the range, checks that
 * enough bytes are present, verifies CRC and the sync byte. If the leading
 * byte is not a plausible LEN, the parser skips one byte at a time looking
 * for a sync byte to resynchronise (mimicking block-finding after corruption).
 *
 * @param buffer Incoming bytes (may contain partial/garbage data).
 * @return ParsedBlock; on NeedMoreData, consumedBytes indicates how many
 *         bytes were skipped as garbage (caller may discard them or keep).
 */
inline ParsedBlock parseBlock(std::span<const uint8_t> buffer) {
    ParsedBlock result;
    size_t i = 0;
    const size_t size = buffer.size();
    size_t startOffset = 0; // tracks how many garbage bytes were skipped

    while (i < size) {
        uint8_t len = buffer[i];
        // A plausible block must have a valid length and room for the trailer.
        if (len >= kMinBlockLength && len <= kMaxBlockLength) {
            if (i + len > size) {
                // Need more data to complete this candidate block.
                result.consumedBytes = i; // keep scanning from i once more arrives
                result.skippedBytes = startOffset;
                result.status = BlockParseStatus::NeedMoreData;
                return result;
            }
            // Validate sync byte.
            if (buffer[i + len - 1] != kSyncByte) {
                // Not a real block boundary; advance one byte.
                ++i;
                continue;
            }
            // Validate CRC over header + content.
            const size_t crcLen = len - kTrailerSize;
            const uint16_t computed = crc16Ccitt(buffer.data() + i, crcLen);
            const uint16_t stored = (static_cast<uint16_t>(buffer[i + crcLen]) << 8)
                                  | static_cast<uint16_t>(buffer[i + crcLen + 1]);
            if (computed != stored) {
                result.status = BlockParseStatus::BadCrc;
                result.consumedBytes = i + len; // discard the corrupt block
                result.wireLength = len;
                result.skippedBytes = startOffset;
                return result;
            }
            // Valid block.
            result.status = BlockParseStatus::Ok;
            result.block.sequence = buffer[i + 1] & kSequenceMask;
            result.block.content.assign(buffer.data() + i + kHeaderSize,
                                        buffer.data() + i + crcLen);
            result.consumedBytes = i + len;
            result.wireLength = len;
            result.skippedBytes = startOffset;
            return result;
        }
        // Not a plausible length: skip to next sync byte to resync.
        ++i;
        startOffset = i;
    }
    result.consumedBytes = i;
    result.skippedBytes = startOffset;
    result.status = BlockParseStatus::NeedMoreData;
    return result;
}

} // namespace tether::klipper::protocol
