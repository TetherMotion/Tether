/**
 * @file Endstop.hpp
 * @brief Endstop peripheral and proxy.
 */

#pragma once

#include <cstdint>
#include <functional>

namespace tether::klipper::objects {

// ============================================================================
// Endstop
// ============================================================================

/**@brief Endstop peripheral. */
class Endstop {
public:
    using PinReadFunc = std::function<bool()>;

    Endstop(uint8_t oid) : oid_(oid) {}
    Endstop(uint8_t oid, PinReadFunc pinRead)
        : oid_(oid), pinRead_(std::move(pinRead)) {}

    uint8_t oid() const { return oid_; }

    /**
     * @brief Check if endstop is currently triggered.
     * @return live pin state (false when no read function is set).
     */
    bool triggered() const { return pinRead_ ? pinRead_() : false; }

    /**
     * @brief Set the pin read function.
     * @param func  Callable returning the live pin state.
     */
    void setPinReadFunc(PinReadFunc func) { pinRead_ = std::move(func); }

    /**
     * @brief Set the sample count for debouncing.
     * @param count  Consecutive samples required to change state.
     */
    void setSampleCount(int count) { sampleCount_ = count; }

    /**
     * @brief Get the sample count.
     * @return configured debounce sample count.
     */
    int sampleCount() const { return sampleCount_; }

    /**
     * @brief Set the endstop state directly (for testing/simulation).
     * @param triggered  Non-zero = triggered.
     */
    void setState(uint32_t triggered) { state_ = triggered != 0; }

    /**
     * @brief Get the endstop state.
     * @return 1 when triggered, 0 otherwise.
     */
    uint32_t state() const { return state_ ? 1u : 0u; }

private:
    uint8_t oid_;
    PinReadFunc pinRead_;
    int sampleCount_ = 8;
    bool state_ = false;
};

// ============================================================================
// Endstop proxy
// ============================================================================

/**@brief Proxy for Endstop that allows setting state. */
class EndstopProxy {
public:
    explicit EndstopProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void setState(bool triggered) { triggered_ = triggered; }
    bool triggered() const { return triggered_; }

private:
    uint8_t oid_;
    bool triggered_ = false;
};

} // namespace tether::klipper::objects
