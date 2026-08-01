/**
 * @file Actuators.hpp
 * @brief Actuators: Fan, Neopixel LED strip, and LedColor struct.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace tether::klipper::objects {

// ============================================================================
// Fan
// ============================================================================

/// @brief PWM-controlled fan with optional tachometer.
class Fan {
public:
    using PwmWriteFunc = std::function<void(double)>;
    using TachReadFunc = std::function<uint32_t()>;

    Fan(uint8_t oid, PwmWriteFunc pwmWrite)
        : oid_(oid)
        , pwmWrite_(std::move(pwmWrite)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Set fan speed (0.0 to 1.0).
    void setSpeed(double speed) {
        speed_ = std::clamp(speed, 0.0, 1.0);
        if (!offTime_.has_value() || speed_ >= offTime_.value()) {
            pwmWrite_(speed_);
        } else {
            pwmWrite_(0.0);
        }
    }

    /// @brief Get current fan speed.
    double speed() const { return speed_; }

    /// @brief Set tachometer reader.
    void setTachometer(TachReadFunc tachRead, uint32_t tachCpr = 2) {
        tachRead_ = std::move(tachRead);
        tachCpr_ = tachCpr;
    }

    /// @brief Compute RPM from tachometer readings.
    /// @param interval Time between readings in seconds.
    double computeRpm(double interval) {
        if (!tachRead_) return 0.0;
        uint32_t current = tachRead_();
        uint32_t delta = current - lastTach_;
        lastTach_ = current;
        if (interval <= 0) return 0.0;
        return static_cast<double>(delta) * 60.0 / (tachCpr_ * interval);
    }

    /// @brief Set off-time (fan turns off below this speed).
    void setOffTime(double offTime) { offTime_ = offTime; }

    /// @brief Set cycle time for PWM.
    void setCycleTime(double cycleTime) { cycleTime_ = cycleTime; }

    /// @brief Get cycle time.
    double cycleTime() const { return cycleTime_; }

private:
    uint8_t oid_;
    PwmWriteFunc pwmWrite_;
    TachReadFunc tachRead_;
    double speed_ = 0.0;
    uint32_t lastTach_ = 0;
    uint32_t tachCpr_ = 2;
    double cycleTime_ = 0.01;
    std::optional<double> offTime_;
};

// ============================================================================
// Neopixel / WS2812
// ============================================================================

/// @brief RGB color for LEDs.
struct LedColor {
    uint8_t r = 0, g = 0, b = 0, w = 0; ///< w = white (for RGBW)

    bool operator==(const LedColor& o) const {
        return r == o.r && g == o.g && b == o.b && w == o.w;
    }
};

/// @brief WS2812/NeoPixel LED strip.
class Neopixel {
public:
    using SpiWriteFunc = std::function<void(std::span<const uint8_t>)>;

    Neopixel(uint8_t oid, int numLeds, SpiWriteFunc spiWrite, bool hasWhite = false)
        : oid_(oid)
        , numLeds_(numLeds)
        , spiWrite_(std::move(spiWrite))
        , hasWhite_(hasWhite) {
        colors_.resize(numLeds);
    }

    uint8_t oid() const { return oid_; }

    /// @brief Set color of a single LED.
    void setColor(int index, const LedColor& color) {
        if (index >= 0 && index < numLeds_) {
            colors_[index] = color;
        }
    }

    /// @brief Set color of all LEDs.
    void setAll(const LedColor& color) {
        for (auto& c : colors_) c = color;
    }

    /// @brief Get color of a LED.
    LedColor color(int index) const {
        if (index >= 0 && index < numLeds_) return colors_[index];
        return {};
    }

    /// @brief Update the LED strip (send colors via SPI).
    void update() {
        // WS2812 encoding: each bit is ~1.25us
        // 0: high ~0.4us, low ~0.85us
        // 1: high ~0.8us, low ~0.45us
        // Using SPI at 6.4 MHz: each bit = 8 SPI bits
        // 0: 0b11000000, 1: 0b11111000
        std::vector<uint8_t> data;
        int bytesPerLed = hasWhite_ ? 4 : 3;
        data.reserve(numLeds_ * bytesPerLed * 8);

        for (const auto& c : colors_) {
            // Order: GRB (or GRBW for RGBW)
            encodeByte(c.g, data);
            encodeByte(c.r, data);
            encodeByte(c.b, data);
            if (hasWhite_) encodeByte(c.w, data);
        }
        spiWrite_(data);
    }

    /// @brief Get number of LEDs.
    int numLeds() const { return numLeds_; }

    /// @brief Clear all LEDs to off.
    void clear() {
        for (auto& c : colors_) c = {};
    }

private:
    void encodeByte(uint8_t byte, std::vector<uint8_t>& out) {
        for (int i = 7; i >= 0; --i) {
            if (byte & (1 << i)) {
                out.push_back(0b11111000);
            } else {
                out.push_back(0b11000000);
            }
        }
    }

    uint8_t oid_;
    int numLeds_;
    SpiWriteFunc spiWrite_;
    bool hasWhite_;
    std::vector<LedColor> colors_;
};

} // namespace tether::klipper::objects
