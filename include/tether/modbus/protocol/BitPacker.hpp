/**
 * @file BitPacker.hpp
 * @brief LSB-first bit pack/unpack for Modbus bit data (plan §6.6).
 */
#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace tether::modbus {

/**@brief Bytes needed to carry `bitCount` bits. */
constexpr uint16_t bitBytes(uint16_t bitCount) {
    return static_cast<uint16_t>((bitCount + 7) / 8);
}

/**
 * @brief Pack bits LSB-first into a byte buffer sized bitBytes(n).
 * Tail bits in the last byte are zeroed.
 */
inline std::vector<uint8_t> packBits(const std::vector<bool>& bits) {
    std::vector<uint8_t> out(bitBytes(static_cast<uint16_t>(bits.size())), 0);
    for (size_t i = 0; i < bits.size(); ++i)
        if (bits[i]) out[i >> 3] |= uint8_t(1u << (i & 7));
    return out;
}

/**
 * @brief Unpack `count` bits LSB-first from `bytes`.
 * Returns fewer than `count` if the buffer is short.
 */
inline std::vector<bool> unpackBits(std::span<const uint8_t> bytes,
                                    uint16_t count) {
    std::vector<bool> out;
    out.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
        const size_t byteIdx = i >> 3;
        if (byteIdx >= bytes.size()) break;
        out.push_back((bytes[byteIdx] >> (i & 7)) & 1);
    }
    return out;
}

} // namespace tether::modbus
