/**
 * @file PositionTracker.hpp
 * @brief Multi-turn/wraparound tracking for position counters
 *
 * EL5xxx counters wrap at their native bit width (16/32/64-bit counters,
 * or narrower SSI singleturn fields inside a wider word).  Feeding the
 * raw cyclic value into a MultiTurnTracker unwraps it into a monotonic
 * int64 position — the helper is pure logic and RT-safe.
 *
 * @code
 *   PositionTracker tr(16);                  // EL5101 16-bit counter
 *   while (running) {
 *       int64_t pos = tr.update(enc.rawPosition(0));
 *   }
 * @endcode
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
namespace Beckhoff {

/**
 * @brief Unwraps a wrapping unsigned counter into a continuous position.
 *
 * update() takes the raw (zero-extended) counter word and returns the
 * accumulated signed position, tracking how often the counter wrapped.
 * Forward/reverse wraps are detected by the half-range rule: a raw jump
 * larger than half the modulus is a wrap in the opposite direction.
 */
class PositionTracker {
public:
    explicit PositionTracker(uint8_t bits)
        : bits_(bits > 64 ? 64 : bits),
          mask_(bits >= 64 ? ~uint64_t{0} : (uint64_t{1} << bits) - 1) {}

    /**
     * @brief Feed one raw counter sample, get the unwrapped position.
     *
     * The first call establishes the origin (returns the raw value as a
     * signed offset from 0).  Subsequent calls accumulate ±modulus per
     * detected wrap.
     */
    int64_t update(uint64_t raw) {
        const uint64_t cur = raw & mask_;
        if (!initialized_) {
            initialized_ = true;
            last_ = cur;
            position_ = static_cast<int64_t>(cur);
            return position_;
        }

        const uint64_t mod = mask_ + 1;   // == 0 when bits_ == 64
        const uint64_t half = mask_ / 2 + 1;

        if (mod == 0) {
            // 64-bit counter — wrapping arithmetic handles it.
            position_ += static_cast<int64_t>(cur - last_);
        } else if (cur >= last_) {
            const uint64_t d = cur - last_;
            position_ += (d < half) ? static_cast<int64_t>(d)
                                    : static_cast<int64_t>(d) -
                                          static_cast<int64_t>(mod);
        } else {
            const uint64_t d = last_ - cur;
            position_ -= (d < half) ? static_cast<int64_t>(d)
                                    : static_cast<int64_t>(d) -
                                          static_cast<int64_t>(mod);
        }
        last_ = cur;
        return position_;
    }

    /// Reset the tracker so the next update() becomes the new origin.
    void reset() { initialized_ = false; }

    /// Shift the accumulated position by `delta` without touching the
    /// raw-sample tracking (e.g. after homing: adjust(-position())).
    void adjust(int64_t delta) { position_ += delta; }

    /// The accumulated (unwrapped) position of the last update().
    int64_t position() const { return position_; }

    /// Number of full modulus turns in the accumulated position
    /// (meaningful only for counters narrower than 64 bits).
    int32_t turns() const {
        if (bits_ >= 64) return 0;
        const int64_t mod = static_cast<int64_t>(mask_) + 1;
        return static_cast<int32_t>(position_ >= 0
            ? position_ / mod
            : -((-position_ - 1) / mod) - 1);
    }

    /// The configured field width in bits.
    uint8_t bits() const { return bits_; }

private:
    uint8_t  bits_;
    uint64_t mask_;
    uint64_t last_       = 0;
    int64_t  position_   = 0;
    bool     initialized_ = false;
};

} // namespace Beckhoff

} // namespace EtherCAT
