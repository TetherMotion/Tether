/**
 * @file McuClock.hpp
 * @brief MCU 32-bit clock with wraparound and 64-bit uptime tracking.
 *
 * @details
 * The device exposes a 32-bit hardware clock that ticks at CLOCK_FREQ Hz
 * (declared in the data dictionary). The clock wraps around every
 * 2^32 / CLOCK_FREQ seconds (~23.8s at 180 MHz). To convert to a monotonic
 * 64-bit clock, the host/device tracks the number of wraparounds based on
 * observed time deltas.
 *
 * The McuClock class models this: it holds the 32-bit tick count and the
 * clock frequency, and can convert to a 64-bit tick count or to seconds.
 */

#pragma once

#include <cstdint>

namespace tether::klipper::clock {

/**
 * @brief MCU 32-bit clock with wraparound tracking.
 */
class McuClock {
public:
    /**
     * @brief Construct with a clock frequency in Hz.
     * @param freqHz Clock frequency (ticks per second), e.g. 180000000.
     */
    explicit McuClock(uint32_t freqHz = 180000000) : freqHz_(freqHz) {}

    /// @return The clock frequency in Hz.
    uint32_t frequency() const { return freqHz_; }

    /// @return The 32-bit tick count (wraps at 2^32).
    uint32_t ticks32() const { return ticks32_; }

    /// @return The 64-bit monotonic tick count (accumulates across wraps).
    uint64_t ticks64() const { return ticks64_; }

    /// @return The elapsed time in seconds (from 64-bit ticks).
    double seconds() const {
        return static_cast<double>(ticks64_) / static_cast<double>(freqHz_);
    }

    /**
     * @brief Advance the clock to a new 32-bit tick reading, detecting
     *        wraparound and updating the 64-bit accumulator.
     * @param newTicks32 The latest 32-bit tick reading from hardware.
     */
    void advanceTo(uint32_t newTicks32) {
        // Unsigned subtraction wraps naturally: (new - old) gives the forward
        // delta even across a 32-bit wrap. E.g. old=500, new=100 (wrapped):
        //   100 - 500 = -400 = 4294966896 (unsigned), which is the correct
        //   forward delta of 4294966896 ticks (just under one full wrap).
        // For a small forward step old=100, new=200: delta = 100.
        uint32_t delta = newTicks32 - ticks32_;
        ticks64_ += delta;
        ticks32_ = newTicks32;
    }

    /**
     * @brief Convert a 32-bit tick value to a 64-bit tick value, assuming it
     *        is near the current clock reading (within half a wrap).
     */
    uint64_t toTicks64(uint32_t ticks32) const {
        // Forward delta (wraps naturally). If the result would be a large
        // forward jump (> half range), the reading is actually in the past
        // (previous wrap), so subtract from 2^32.
        uint32_t diff = ticks32 - ticks32_;
        if (diff > 0x80000000u) {
            // ticks32 is behind current — previous wrap.
            return ticks64_ - (0x100000000ull - diff);
        }
        return ticks64_ + diff;
    }

    /**
     * @brief Convert a 32-bit tick value to seconds (64-bit precision).
     */
    double toSeconds(uint32_t ticks32) const {
        return static_cast<double>(toTicks64(ticks32)) / static_cast<double>(freqHz_);
    }

    /// @brief Reset the clock to zero.
    void reset() { ticks32_ = 0; ticks64_ = 0; }

private:
    uint32_t freqHz_;
    uint32_t ticks32_ = 0;
    uint64_t ticks64_ = 0;
};

} // namespace tether::klipper::clock
