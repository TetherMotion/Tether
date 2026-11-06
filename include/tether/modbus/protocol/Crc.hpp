/**
 * @file Crc.hpp
 * @brief CRC-16/MODBUS (poly 0xA001, init 0xFFFF, low byte first).
 */
#pragma once

#include <cstdint>
#include <span>

#include "tether/modbus/protocol/Constants.hpp"

namespace tether::modbus {

/**
 * @brief Incremental CRC-16/MODBUS.
 * Wire order: low byte first, then high byte.
 */
class Crc16 {
public:
    void update(uint8_t b) {
        crc_ ^= b;
        for (int i = 0; i < 8; ++i)
            crc_ = (crc_ & 1) ? (crc_ >> 1) ^ kCrcPolynomial : crc_ >> 1;
    }
    void update(std::span<const uint8_t> bytes) {
        for (uint8_t b : bytes) update(b);
    }
    uint16_t value() const { return crc_; }
    void     reset() { crc_ = kCrcInitial; }

    static uint16_t of(std::span<const uint8_t> bytes) {
        Crc16 c; c.update(bytes); return c.value();
    }
    /// Check that `frame` ends with a valid CRC (len ≥ 3).
    static bool check(std::span<const uint8_t> frame) {
        if (frame.size() < 3) return false;
        const uint16_t want = of(frame.subspan(0, frame.size() - 2));
        const uint16_t got = uint16_t(frame[frame.size() - 2]) |
                             (uint16_t(frame[frame.size() - 1]) << 8);
        return want == got;
    }
    /// Append CRC (low byte first) to `frame`; returns new size.
    static size_t appendTo(std::span<uint8_t> buf, size_t len) {
        const uint16_t crc = of(buf.subspan(0, len));
        buf[len] = uint8_t(crc);
        buf[len + 1] = uint8_t(crc >> 8);
        return len + 2;
    }

private:
    uint16_t crc_ = kCrcInitial;
};

/**@brief ASCII-mode LRC: (-sum(bytes)) & 0xFF. */
inline uint8_t lrc(std::span<const uint8_t> bytes) {
    uint8_t sum = 0;
    for (uint8_t b : bytes) sum = uint8_t(sum + b);
    return uint8_t(-sum);
}

} // namespace tether::modbus
