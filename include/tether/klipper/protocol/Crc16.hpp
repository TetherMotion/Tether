/**
 * @file Crc16.hpp
 * @brief CRC-16 used by the Klipper message-block trailer.
 *
 * @details
 * The protocol specification describes a "CRC-16/CCITT-FALSE with a twist":
 * a compact byte-oriented table-less computation. The CRC is initialised to
 * 0xFFFF and processed one input byte at a time. For each byte:
 *
 *   1. The low byte of the running CRC is XORed into the data byte.
 *   2. The data byte's low nibble is shifted left by 4 and XORed back into
 *      the data byte.
 *   3. The new CRC is computed by XORing: the shifted-out high byte of the
 *      old CRC (crc >> 8), the shifted data byte (data << 8), the data byte
 *      shifted left by 3 (data << 3), and the data byte shifted right by 4
 *      (data >> 4).
 *
 * The returned value is stored big-endian in the block (CRC_HI first, then
 * CRC_LO). The CRC covers the header and content bytes (everything except
 * the 3-byte trailer).
 *
 * @note The implementation follows the documented byte-oriented algorithm
 *       verbatim. (The protocol docs' worked-example hex value for a sample
 *       block is illustrative; this implementation matches the algorithm
 *       description, which is the authoritative specification.)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <span>

namespace tether::klipper::protocol {

/**
 * @brief Compute the protocol CRC-16 over a byte range.
 *
 * @param data Pointer to the bytes to checksum.
 * @param length Number of bytes.
 * @return 16-bit CRC value (store big-endian on the wire).
 */
inline uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        uint8_t b = data[i] ^ static_cast<uint8_t>(crc & 0xFF);
        b ^= static_cast<uint8_t>(b << 4);
        crc = static_cast<uint16_t>((crc >> 8)
            ^ static_cast<uint16_t>(b << 8)
            ^ static_cast<uint16_t>(b << 3)
            ^ static_cast<uint16_t>(b >> 4));
    }
    return crc;
}

/**
 * @brief Convenience overload for std::span.
 */
inline uint16_t crc16Ccitt(std::span<const uint8_t> data) {
    return crc16Ccitt(data.data(), data.size());
}

} // namespace tether::klipper::protocol
