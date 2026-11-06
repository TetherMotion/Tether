/**
 * @file RtuTiming.hpp
 * @brief RTU t1.5/t3.5 character-time computation (plan §7.4).
 */
#pragma once

#include <chrono>
#include <cstdint>

#include "tether/modbus/link/SerialConfig.hpp"

namespace tether::modbus {

struct RtuTiming {
    /// Character times, rounded up to whole microseconds.
    std::chrono::microseconds t15;
    std::chrono::microseconds t35;
    std::chrono::microseconds charTime;

    /// >19200 baud → fixed 750us/1750us floors (Modbus spec).
    static RtuTiming forSerial(const SerialConfig& cfg) {
        return forParams(cfg.baud, cfg.bitsPerChar());
    }
    static RtuTiming forParams(uint32_t baud, uint8_t bitsPerChar) {
        const double cUs = 1e6 * bitsPerChar / double(baud ? baud : 1);
        auto us = [](double v) {
            return std::chrono::microseconds(uint64_t(v + 0.999));
        };
        RtuTiming t{us(cUs * 1.5), us(cUs * 3.5), us(cUs)};
        if (baud > 19200) {
            t.t15 = std::max(t.t15, std::chrono::microseconds(750));
            t.t35 = std::max(t.t35, std::chrono::microseconds(1750));
        }
        return t;
    }
    /// Whole-PDU wire time estimate (incl. start/stop bits per char).
    std::chrono::microseconds txTime(size_t bytes) const {
        return charTime * bytes;
    }
};

} // namespace tether::modbus
