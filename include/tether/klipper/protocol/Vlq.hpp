/**
 * @file Vlq.hpp
 * @brief Variable-length integer encoding for the Klipper protocol.
 *
 * @details
 * Two distinct VLQ encodings are used:
 *
 * 1. **Msgid encoding** — a simplified *unsigned* VLQ, 1–2 bytes, no sign
 *    extension. Values < 128 fit in one byte; otherwise two bytes with the
 *    continuation bit set on the first byte. Max value 16383 (0x3FFF).
 *
 * 2. **Parameter encoding** — a *signed* VLQ, 1–5 bytes, with sign extension.
 *    Small integers near zero use fewer bytes; positive integers generally
 *    encode more compactly than negative ones.
 *
 *    First byte layout:
 *      bit 7: continuation (1 = more bytes follow)
 *      bits 6,5: sign extension bits
 *      bits 4..0: payload (low 5 bits of first chunk)
 *    Subsequent bytes:
 *      bit 7: continuation
 *      bits 6..0: payload (7 bits)
 *
 *    If both sign bits are set (byte & 0x60 == 0x60) the value is negative and
 *    the upper bits are filled with 1s (OR with -0x20 / 0xFFFFFFE0).
 *
 * String/buffer parameters are NOT VLQ integers; they use a length-prefixed
 * encoding (VLQ length followed by raw bytes) — see ParameterFormat.hpp.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>
#include <optional>

namespace tether::klipper::protocol {

// ============================================================================
// Msgid VLQ (unsigned, 1–2 bytes)
// ============================================================================

/**
 * @brief Encode an unsigned msgid (0..16383) into 1 or 2 bytes.
 * @param msgid Value to encode (must be <= kMaxMsgId).
 * @param out   Output buffer (at least 2 bytes).
 * @return Number of bytes written (1 or 2).
 */
inline size_t encodeMsgId(uint16_t msgid, uint8_t* out) {
    if (msgid < 0x80) {
        out[0] = static_cast<uint8_t>(msgid);
        return 1;
    }
    out[0] = static_cast<uint8_t>((msgid >> 7) | 0x80);
    out[1] = static_cast<uint8_t>(msgid & 0x7F);
    return 2;
}

/**
 * @brief Decode an unsigned msgid from a byte stream.
 * @param p   Pointer to bytes (advanced past the decoded msgid).
 * @param end End of buffer.
 * @return Decoded msgid, or std::nullopt on truncation/overflow.
 */
inline std::optional<uint16_t> decodeMsgId(const uint8_t*& p, const uint8_t* end) {
    if (p >= end) return std::nullopt;
    uint8_t b0 = *p++;
    if ((b0 & 0x80) == 0) {
        return static_cast<uint16_t>(b0 & 0x7F);
    }
    if (p >= end) return std::nullopt;
    uint8_t b1 = *p++;
    uint16_t v = static_cast<uint16_t>((b0 & 0x7F) << 7) | static_cast<uint16_t>(b1 & 0x7F);
    if (v > 0x3FFF) return std::nullopt; // exceeds kMaxMsgId
    return v;
}

// ============================================================================
// Parameter VLQ (signed, 1–5 bytes, sign-extended)
// ============================================================================

/**
 * @brief Encode a signed 32-bit integer as a parameter VLQ.
 * @param value Value to encode (interpreted as int32_t; unsigned values up to
 *              4294967295 are supported via the 5-byte form).
 * @param out   Output buffer (at least 5 bytes).
 * @return Number of bytes written (1..5).
 */
inline size_t encodeParam(int32_t value, uint8_t* out) {
    uint32_t v = static_cast<uint32_t>(value);
    size_t n = 0;
    // Determine the minimum number of bytes from the documented ranges.
    // Boundaries (as signed comparisons on the original value):
    //   1 byte: -32 .. 95
    //   2 bytes: -4096 .. 12287
    //   3 bytes: -524288 .. 1572863
    //   4 bytes: -67108864 .. 201326591
    //   5 bytes: everything else (full int32 range)
    if (value < -32) {
        if (value < -4096) {
            if (value < -524288) {
                if (value < -67108864) n = 5; else n = 4;
            } else n = 3;
        } else n = 2;
    } else if (value > 95) {
        if (value > 12287) {
            if (value > 1572863) {
                if (value > 201326591) n = 5; else n = 4;
            } else n = 3;
        } else n = 2;
    } else {
        n = 1;
    }

    // Emit n-1 continuation bytes (each contributing 7 bits, MSB first),
    // then the final byte with the low 7 bits.
    for (size_t i = n - 1; i > 0; --i) {
        out[n - 1 - i] = static_cast<uint8_t>(((v >> (7 * i)) & 0x7F) | 0x80);
    }
    out[n - 1] = static_cast<uint8_t>(v & 0x7F);
    return n;
}

/**
 * @brief Decode a signed parameter VLQ.
 * @param p   Pointer to bytes (advanced past the decoded value).
 * @param end End of buffer.
 * @return Decoded int32_t, or std::nullopt on truncation.
 *
 * The decoded value is sign-extended to 32 bits. Callers that need an
 * unsigned interpretation should mask the result with 0xFFFFFFFF.
 */
inline std::optional<int32_t> decodeParam(const uint8_t*& p, const uint8_t* end) {
    if (p >= end) return std::nullopt;
    uint8_t b0 = *p++;
    uint32_t v = b0 & 0x7F;
    // Sign extension: if both bits 5 and 6 are set, fill upper bits with 1s.
    if ((b0 & 0x60) == 0x60) {
        v |= 0xFFFFFFE0; // -0x20
    }
    uint8_t b = b0;
    while (b & 0x80) {
        if (p >= end) return std::nullopt;
        b = *p++;
        v = (v << 7) | static_cast<uint32_t>(b & 0x7F);
    }
    return static_cast<int32_t>(v);
}

/**
 * @brief Decode a parameter and return it as an unsigned 32-bit value
 *        (masking to 32 bits, for %u/%hu/%c types).
 */
inline std::optional<uint32_t> decodeParamUnsigned(const uint8_t*& p, const uint8_t* end) {
    auto r = decodeParam(p, end);
    if (!r) return std::nullopt;
    return static_cast<uint32_t>(*r);
}

} // namespace tether::klipper::protocol
