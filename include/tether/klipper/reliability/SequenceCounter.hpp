/**
 * @file SequenceCounter.hpp
 * @brief 4-bit mod-16 sequence counter for Klipper message blocks.
 *
 * @details
 * The SEQ nibble in the message-block header is a 4-bit counter that wraps
 * around at 16. The host assigns sequence numbers to outgoing blocks and
 * tracks which have been acknowledged; the device echoes the last-received
 * in-order sequence number in its ack blocks.
 */

#pragma once

#include <cstdint>

namespace tether::klipper::reliability {

/**
 * @brief 4-bit wrapping sequence counter (0..15).
 */
class SequenceCounter {
public:
    SequenceCounter() = default;
    explicit SequenceCounter(uint8_t initial) : value_(initial & 0x0F) {}

    /// @return Current sequence value (0..15).
    uint8_t value() const { return value_; }

    /// @brief Advance to the next sequence number (wraps at 16).
    SequenceCounter next() const {
        return SequenceCounter{static_cast<uint8_t>((value_ + 1) & 0x0F)};
    }

    /// @brief Advance this counter to the next value.
    void advance() { value_ = (value_ + 1) & 0x0F; }

    /// @brief Set the counter value (masked to 4 bits).
    void set(uint8_t v) { value_ = v & 0x0F; }

    /// @brief Number of pending (unacknowledged) sequence numbers between
    ///        @p acked (exclusive) and @p sent (inclusive), modulo 16.
    static uint8_t pending(uint8_t sent, uint8_t acked) {
        return static_cast<uint8_t>((sent - acked) & 0x0F);
    }

private:
    uint8_t value_ = 0;
};

} // namespace tether::klipper::reliability
