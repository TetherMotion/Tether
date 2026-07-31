/**
 * @file Peripherals.hpp
 * @brief Additional peripherals: fans, LEDs, filament sensors, pulse counters.
 *
 * This file provides peripheral objects that are commonly used in 3D printers:
 *   - Fan: PWM-controlled fan with tachometer
 *   - Neopixel: WS2812/NeoPixel LED strip
 *   - FilamentSensor: runout and hall effect width sensor
 *   - PulseCounter: quadrature encoder / pulse counter
 *   - Adxl345: accelerometer for resonance measurement
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace tether::klipper::objects {

// ============================================================================
// Digital output
// ============================================================================

/// @brief Digital output pin object.
class DigitalOut {
public:
    using WriteFunc = std::function<void(bool)>;

    DigitalOut(uint8_t oid) : oid_(oid) {}
    DigitalOut(uint8_t oid, WriteFunc writeFunc)
        : oid_(oid), writeFunc_(std::move(writeFunc)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Set the output value.
    void setValue(bool value) {
        value_ = value;
        if (writeFunc_) writeFunc_(value);
    }

    /// @brief Get the current output value.
    uint32_t value() const { return value_ ? 1u : 0u; }

    /// @brief Schedule a value change at a specific clock.
    void scheduleValue(uint32_t value, uint32_t clock) {
        scheduled_.push_back({clock, value != 0});
        pending_++;
    }

    /// @brief Check if a value is scheduled.
    bool hasSchedule() const { return !scheduled_.empty(); }

    /// @brief Get the scheduled clock (next scheduled).
    uint32_t scheduledClock() const {
        return scheduled_.empty() ? 0 : scheduled_.front().clock;
    }

    /// @brief Get the scheduled value (next scheduled).
    bool scheduledValue() const {
        return scheduled_.empty() ? false : scheduled_.front().value;
    }

    /// @brief Clear the schedule.
    void clearSchedule() { scheduled_.clear(); }

    /// @brief Set the write function.
    void setWriteFunc(WriteFunc func) { writeFunc_ = std::move(func); }

    /// @brief Get pending scheduled count.
    uint32_t pending() const { return pending_; }

    /// @brief Process scheduled events at a given clock.
    void tick(uint32_t clock) {
        while (!scheduled_.empty() && clock >= scheduled_.front().clock) {
            auto& next = scheduled_.front();
            setValue(next.value);
            if (pending_ > 0) pending_--;
            scheduled_.erase(scheduled_.begin());
        }
    }

private:
    struct ScheduledEvent {
        uint32_t clock;
        bool value;
    };

    uint8_t oid_;
    bool value_ = false;
    WriteFunc writeFunc_;
    std::vector<ScheduledEvent> scheduled_;
    uint32_t pending_ = 0;
};

// ============================================================================
// PWM output
// ============================================================================

/// @brief PWM output pin object.
class PwmOut {
public:
    using WriteFunc = std::function<void(double)>;

    PwmOut(uint8_t oid) : oid_(oid) {}
    PwmOut(uint8_t oid, WriteFunc writeFunc)
        : oid_(oid), writeFunc_(std::move(writeFunc)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Set the duty cycle (0.0 to 1.0).
    void setDuty(double duty) {
        duty_ = std::clamp(duty, 0.0, 1.0);
        if (writeFunc_) writeFunc_(duty_);
    }

    /// @brief Set the duty cycle with a max cycle value (integer API).
    void setDuty(uint32_t duty, uint32_t maxCycle) {
        dutyRaw_ = duty;
        maxCycle_ = maxCycle;
        if (maxCycle > 0) {
            duty_ = static_cast<double>(duty) / static_cast<double>(maxCycle);
            if (writeFunc_) writeFunc_(duty_);
        }
    }

    /// @brief Get the current duty cycle (raw integer value).
    uint32_t duty() const { return dutyRaw_; }

    /// @brief Get the current duty cycle as double (0.0-1.0).
    double dutyDouble() const { return duty_; }

    /// @brief Set the cycle time in seconds.
    void setCycleTime(double cycleTime) { cycleTime_ = cycleTime; }

    /// @brief Get the cycle time.
    double cycleTime() const { return cycleTime_; }

    /// @brief Set the write function.
    void setWriteFunc(WriteFunc func) { writeFunc_ = std::move(func); }

    /// @brief Schedule a duty cycle change at a specific clock.
    void scheduleDuty(uint32_t duty, uint32_t clock) {
        scheduledClock_ = clock;
        scheduledDutyRaw_ = duty;
        hasSchedule_ = true;
        pending_++;
    }

    /// @brief Get pending scheduled count.
    uint32_t pending() const { return pending_; }

    /// @brief Process scheduled events at a given clock.
    void tick(uint32_t clock) {
        if (hasSchedule_ && clock >= scheduledClock_) {
            dutyRaw_ = scheduledDutyRaw_;
            if (maxCycle_ > 0) {
                duty_ = static_cast<double>(scheduledDutyRaw_) / static_cast<double>(maxCycle_);
                if (writeFunc_) writeFunc_(duty_);
            }
            hasSchedule_ = false;
            if (pending_ > 0) pending_--;
        }
    }

    /// @brief Get the maximum cycle time.
    uint32_t maxCycle() const { return maxCycle_; }

    /// @brief Set the maximum cycle time.
    void setMaxCycle(uint32_t maxCycle) { maxCycle_ = maxCycle; }

private:
    uint8_t oid_;
    double duty_ = 0.0;
    uint32_t dutyRaw_ = 0;
    double cycleTime_ = 0.001;
    WriteFunc writeFunc_;
    uint32_t scheduledClock_ = 0;
    uint32_t scheduledDutyRaw_ = 0;
    bool hasSchedule_ = false;
    uint32_t pending_ = 0;
    uint32_t maxCycle_ = 0;
};

/// @brief Proxy for DigitalOut that tracks pending scheduled values.
class DigitalOutProxy {
public:
    explicit DigitalOutProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void setValue(uint32_t value) { value_ = value; pending_++; }
    uint32_t value() const { return value_; }
    uint32_t pending() const { return pending_; }

    void tick(uint32_t clock) {
        // Process scheduled events
        if (pending_ > 0) pending_--;
    }

private:
    uint8_t oid_;
    uint32_t value_ = 0;
    uint32_t pending_ = 0;
};

/// @brief Proxy for PwmOut that tracks pending scheduled values.
class PwmOutProxy {
public:
    explicit PwmOutProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void setDuty(double duty) { duty_ = duty; pending_++; }
    double duty() const { return duty_; }
    uint32_t pending() const { return pending_; }

    void tick(uint32_t clock) {
        if (pending_ > 0) pending_--;
    }

private:
    uint8_t oid_;
    double duty_ = 0.0;
    uint32_t pending_ = 0;
};

// ============================================================================
// Endstop
// ============================================================================

/// @brief Endstop state.
enum class TrsyncState {
    Idle,
    Armed,
    Triggered,
    Sent,
};

/// @brief Endstop peripheral.
class Endstop {
public:
    using PinReadFunc = std::function<bool()>;

    Endstop(uint8_t oid) : oid_(oid) {}
    Endstop(uint8_t oid, PinReadFunc pinRead)
        : oid_(oid), pinRead_(std::move(pinRead)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Check if endstop is currently triggered.
    bool triggered() const { return pinRead_ ? pinRead_() : false; }

    /// @brief Set the pin read function.
    void setPinReadFunc(PinReadFunc func) { pinRead_ = std::move(func); }

    /// @brief Set the sample count for debouncing.
    void setSampleCount(int count) { sampleCount_ = count; }

    /// @brief Get the sample count.
    int sampleCount() const { return sampleCount_; }

    /// @brief Set the endstop state directly (for testing/simulation).
    void setState(uint32_t triggered) { state_ = triggered != 0; }

    /// @brief Get the endstop state.
    uint32_t state() const { return state_ ? 1u : 0u; }

private:
    uint8_t oid_;
    PinReadFunc pinRead_;
    int sampleCount_ = 8;
    bool state_ = false;
};

// ============================================================================
// Trsync (trigger synchronization)
// ============================================================================

/// @brief Trsync peripheral for homing synchronization.
class Trsync {
public:
    explicit Trsync(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    /// @brief Arm the trsync.
    void arm(uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
        timeoutClock_ = timeoutClock;
        triggerClock_ = 0;
    }

    /// @brief Arm with a report clock and timeout.
    void arm(uint32_t reportClock, uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
        timeoutClock_ = timeoutClock;
        reportClock_ = reportClock;
        triggerClock_ = 0;
    }

    /// @brief Process a tick at a given clock.
    void tick(uint32_t clock) {
        if (state_ == TrsyncState::Armed && clock >= timeoutClock_) {
            expire();
        }
    }

    /// @brief Trigger the trsync.
    void trigger(uint32_t clock) {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
            triggerClock_ = clock;
        }
    }

    /// @brief Expire the trsync (timeout).
    void expire() {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
            triggerClock_ = timeoutClock_;
        }
    }

    /// @brief Mark as sent (notification dispatched).
    void markSent() {
        if (state_ == TrsyncState::Triggered) {
            state_ = TrsyncState::Sent;
        }
    }

    /// @brief Get the current state.
    TrsyncState state() const { return state_; }

    /// @brief Get the trigger clock.
    uint32_t triggerClock() const { return triggerClock_; }

    /// @brief Get the timeout clock.
    uint32_t timeoutClock() const { return timeoutClock_; }

    /// @brief Reset to idle.
    void reset() {
        state_ = TrsyncState::Idle;
        triggerClock_ = 0;
        timeoutClock_ = 0;
    }

private:
    uint8_t oid_;
    TrsyncState state_ = TrsyncState::Idle;
    uint32_t triggerClock_ = 0;
    uint32_t timeoutClock_ = 0;
    uint32_t reportClock_ = 0;
};

/// @brief Proxy for Trsync that tracks state changes.
class TrsyncProxy {
public:
    explicit TrsyncProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void arm(uint32_t reportClock, uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
    }

    void trigger(uint32_t clock) {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
        }
    }

    TrsyncState state() const { return state_; }

private:
    uint8_t oid_;
    TrsyncState state_ = TrsyncState::Idle;
};

// ============================================================================
// Analog input
// ============================================================================

/// @brief Analog input pin object.
class AnalogIn {
public:
    using ReadFunc = std::function<uint16_t()>;

    AnalogIn(uint8_t oid) : oid_(oid) {}
    AnalogIn(uint8_t oid, ReadFunc readFunc)
        : oid_(oid), readFunc_(std::move(readFunc)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Read the analog value (0-65535).
    uint16_t read() const { return readFunc_ ? readFunc_() : 0; }

    /// @brief Set the read function.
    void setReadFunc(ReadFunc func) { readFunc_ = std::move(func); }

    /// @brief Get the last sample.
    uint16_t lastSample() const { return lastSample_; }

    /// @brief Get the last value (alias for lastSample).
    uint16_t lastValue() const { return lastSample_; }

    /// @brief Set a sample value directly (for testing/simulation).
    void setSample(uint16_t value) { lastSample_ = value; }

    /// @brief Update (read and store the sample).
    void update() { lastSample_ = read(); }

private:
    uint8_t oid_;
    ReadFunc readFunc_;
    uint16_t lastSample_ = 0;
};

/// @brief Proxy for AnalogIn that stores a value for testing.
class AnalogInProxy {
public:
    explicit AnalogInProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void setValue(uint16_t value) { value_ = value; }
    uint16_t read() const { return value_; }

private:
    uint8_t oid_;
    uint16_t value_ = 0;
};

// ============================================================================
// SPI
// ============================================================================

/// @brief SPI peripheral.
class Spi {
public:
    using TransferFunc = std::function<std::vector<uint8_t>(std::span<const uint8_t>)>;

    Spi(uint8_t oid) : oid_(oid) {}
    Spi(uint8_t oid, TransferFunc transfer)
        : oid_(oid), transfer_(std::move(transfer)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Transfer data over SPI.
    std::vector<uint8_t> transfer(std::span<const uint8_t> data) {
        if (transfer_) return transfer_(data);
        // Default: return same size vector of zeros
        return std::vector<uint8_t>(data.size(), 0);
    }

    /// @brief Transfer data over SPI (vector overload).
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& data) {
        return transfer(std::span<const uint8_t>(data));
    }

    /// @brief Set the transfer function.
    void setTransferFunc(TransferFunc func) { transfer_ = std::move(func); }

private:
    uint8_t oid_;
    TransferFunc transfer_;
};

/// @brief Proxy for SPI that echoes or stores data.
class SpiProxy {
public:
    explicit SpiProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    std::vector<uint8_t> transfer(std::span<const uint8_t> data) {
        lastData_.assign(data.begin(), data.end());
        return std::vector<uint8_t>(data.size(), 0);
    }

    const std::vector<uint8_t>& lastData() const { return lastData_; }

private:
    uint8_t oid_;
    std::vector<uint8_t> lastData_;
};

// ============================================================================
// I2C
// ============================================================================

/// @brief I2C peripheral.
class I2c {
public:
    using ReadFunc = std::function<std::vector<uint8_t>(uint8_t addr, uint8_t reg, size_t len)>;
    using WriteFunc = std::function<void(uint8_t addr, uint8_t reg, std::span<const uint8_t> data)>;
    using ReadNoRegFunc = std::function<std::vector<uint8_t>(uint8_t addr, size_t len)>;
    using WriteNoRegFunc = std::function<void(uint8_t addr, std::span<const uint8_t> data)>;
    using Read16Func = std::function<std::vector<uint8_t>(uint8_t addr, uint16_t reg, size_t len)>;
    using Write16Func = std::function<void(uint8_t addr, uint16_t reg, std::span<const uint8_t> data)>;

    I2c(uint8_t oid) : oid_(oid) {}
    I2c(uint8_t oid, ReadFunc readFunc, WriteFunc writeFunc)
        : oid_(oid), readFunc_(std::move(readFunc)), writeFunc_(std::move(writeFunc)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Read from an I2C device with an 8-bit register address.
    std::vector<uint8_t> read(uint8_t addr, uint8_t reg, size_t len) {
        if (readFunc_) return readFunc_(addr, reg, len);
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Read from an I2C device without register addressing.
    /// Some devices (e.g. MLX90614) don't use register addresses.
    std::vector<uint8_t> readNoRegister(uint8_t addr, size_t len) {
        if (readNoRegFunc_) return readNoRegFunc_(addr, len);
        if (readFunc_) return readFunc_(addr, 0, len); // Fallback
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Read from an I2C device with a 16-bit register address.
    /// Used by devices with many registers (e.g. EEPROM, large sensors).
    std::vector<uint8_t> read16(uint8_t addr, uint16_t reg, size_t len) {
        if (read16Func_) return read16Func_(addr, reg, len);
        // Fallback: split 16-bit reg into two 8-bit writes
        if (readFunc_) return readFunc_(addr, static_cast<uint8_t>(reg >> 8), len);
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Write to an I2C device with an 8-bit register address.
    bool write(uint8_t addr, uint8_t reg, std::span<const uint8_t> data) {
        if (writeFunc_) { writeFunc_(addr, reg, data); return true; }
        return true; // Return true even without writeFunc (no-op success)
        }

    /// @brief Write to an I2C device without register addressing.
    bool writeNoRegister(uint8_t addr, std::span<const uint8_t> data) {
        if (writeNoRegFunc_) { writeNoRegFunc_(addr, data); return true; }
        if (writeFunc_) { writeFunc_(addr, 0, data); return true; }
        return true;
    }

    /// @brief Write to an I2C device with a 16-bit register address.
    bool write16(uint8_t addr, uint16_t reg, std::span<const uint8_t> data) {
        if (write16Func_) { write16Func_(addr, reg, data); return true; }
        if (writeFunc_) { writeFunc_(addr, static_cast<uint8_t>(reg >> 8), data); return true; }
        return true;
    }

    // Convenience overloads
    std::vector<uint8_t> read(uint8_t addr, size_t len) {
        return readNoRegister(addr, len);
    }
    bool write(uint8_t addr, std::span<const uint8_t> data) {
        return writeNoRegister(addr, data);
    }
    bool write(uint8_t addr, const std::vector<uint8_t>& data) {
        return writeNoRegister(addr, std::span<const uint8_t>(data));
    }

    void setReadFunc(ReadFunc func) { readFunc_ = std::move(func); }
    void setWriteFunc(WriteFunc func) { writeFunc_ = std::move(func); }
    void setReadNoRegFunc(ReadNoRegFunc func) { readNoRegFunc_ = std::move(func); }
    void setWriteNoRegFunc(WriteNoRegFunc func) { writeNoRegFunc_ = std::move(func); }
    void setRead16Func(Read16Func func) { read16Func_ = std::move(func); }
    void setWrite16Func(Write16Func func) { write16Func_ = std::move(func); }

private:
    uint8_t oid_;
    ReadFunc readFunc_;
    WriteFunc writeFunc_;
    ReadNoRegFunc readNoRegFunc_;
    WriteNoRegFunc writeNoRegFunc_;
    Read16Func read16Func_;
    Write16Func write16Func_;
};

/// @brief Proxy for I2C that stores data.
class I2cProxy {
public:
    explicit I2cProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    std::vector<uint8_t> read(uint8_t addr, uint8_t reg, size_t len) {
        return std::vector<uint8_t>(len, 0);
    }

    void write(uint8_t addr, uint8_t reg, std::span<const uint8_t> data) {
        lastAddr_ = addr;
        lastReg_ = reg;
        lastData_.assign(data.begin(), data.end());
    }

    uint8_t lastAddr() const { return lastAddr_; }
    uint8_t lastReg() const { return lastReg_; }
    const std::vector<uint8_t>& lastData() const { return lastData_; }

private:
    uint8_t oid_;
    uint8_t lastAddr_ = 0;
    uint8_t lastReg_ = 0;
    std::vector<uint8_t> lastData_;
};

// ============================================================================
// Endstop proxy
// ============================================================================

/// @brief Proxy for Endstop that allows setting state.
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

// ============================================================================
// Filament sensors
// ============================================================================

/// @brief Filament runout sensor.
class FilamentSensor {
public:
    using PinReadFunc = std::function<bool()>;

    FilamentSensor(uint8_t oid, PinReadFunc pinRead)
        : oid_(oid)
        , pinRead_(std::move(pinRead)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Check if filament is present.
    bool filamentPresent() const { return !pinRead_(); }

    /// @brief Check if filament has run out.
    bool runout() const { return pinRead_(); }

    /// @brief Update sensor state (call periodically).
    void update() {
        bool current = pinRead_();
        if (current && !lastState_) {
            // Just ran out
            runoutEvent_ = true;
        }
        lastState_ = current;
    }

    /// @brief Check and clear runout event.
    bool consumeRunoutEvent() {
        bool event = runoutEvent_;
        runoutEvent_ = false;
        return event;
    }

    /// @brief Set minimum event time (debounce).
    void setMinEventTime(double seconds) { minEventTime_ = seconds; }

private:
    uint8_t oid_;
    PinReadFunc pinRead_;
    bool lastState_ = false;
    bool runoutEvent_ = false;
    double minEventTime_ = 0.0;
};

/// @brief Hall effect filament width sensor.
class HallFilamentSensor {
public:
    using AdcReadFunc = std::function<double()>;

    HallFilamentSensor(uint8_t oid, AdcReadFunc adcRead)
        : oid_(oid)
        , adcRead_(std::move(adcRead)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Read filament diameter in mm.
    double diameter() const {
        double adc = adcRead_();
        // Linear mapping: 1.75mm at 0V, 2.85mm at 5V (example calibration)
        return 1.75 + adc / 4095.0 * 1.1;
    }

    /// @brief Check if diameter is within tolerance.
    bool withinTolerance(double nominal = 1.75, double tolerance = 0.1) const {
        double d = diameter();
        return std::abs(d - nominal) <= tolerance;
    }

private:
    uint8_t oid_;
    AdcReadFunc adcRead_;
};

// ============================================================================
// Pulse counter / encoder
// ============================================================================

/// @brief Pulse counter (quadrature encoder or simple pulse counting).
class PulseCounter {
public:
    using PinReadFunc = std::function<bool()>;

    PulseCounter(uint8_t oid)
        : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    /// @brief Process a pin edge (called on interrupt).
    void onEdge(bool rising) {
        if (rising) {
            count_++;
        } else {
            count_--;
        }
    }

    /// @brief Get current count.
    int32_t count() const { return count_; }

    /// @brief Reset count.
    void reset() { count_ = 0; }

    /// @brief Set sample time for rate calculation.
    void setSampleTime(double seconds) { sampleTime_ = seconds; }

    /// @brief Compute pulse rate (pulses per second).
    double pulseRate() const {
        if (sampleTime_ <= 0) return 0.0;
        return static_cast<double>(count_ - lastCount_) / sampleTime_;
    }

    /// @brief Update rate calculation (call at sample interval).
    void updateRate() {
        lastCount_ = count_;
    }

private:
    uint8_t oid_;
    int32_t count_ = 0;
    int32_t lastCount_ = 0;
    double sampleTime_ = 1.0;
};

// ============================================================================
// ADXL345 accelerometer
// ============================================================================

/// @brief ADXL345 accelerometer for resonance measurement.
class Adxl345 {
public:
    using SpiTransferFunc = std::function<std::vector<uint8_t>(std::span<const uint8_t>)>;

    struct Acceleration {
        double x, y, z; ///< in g (9.81 m/s²)
    };

    Adxl345(uint8_t oid, SpiTransferFunc spiTransfer)
        : oid_(oid)
        , spiTransfer_(std::move(spiTransfer)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Initialize the ADXL345.
    bool init() {
        // Read DEVID register (0x00) - should return 0xE5
        std::vector<uint8_t> cmd = {0x80, 0x00}; // Read register 0x00
        auto resp = spiTransfer_(cmd);
        if (resp.size() < 2) return false;
        return resp[1] == 0xE5;
    }

    /// @brief Read acceleration values.
    Acceleration read() {
        // Read 6 bytes starting from DATAX0 (0x32)
        // Read multiple bytes: 0x80 | 0x40 | 0x32 = 0xF2
        std::vector<uint8_t> cmd = {0xF2, 0, 0, 0, 0, 0, 0};
        auto resp = spiTransfer_(cmd);
        if (resp.size() < 7) return {0, 0, 0};

        int16_t x = static_cast<int16_t>(resp[2] | (resp[3] << 8));
        int16_t y = static_cast<int16_t>(resp[4] | (resp[5] << 8));
        int16_t z = static_cast<int16_t>(resp[6] | (resp[7] << 8));

        // Scale: ±2g range, 10-bit resolution, ~0.0039 g/LSB
        return {x * 0.0039, y * 0.0039, z * 0.0039};
    }

    /// @brief Start data collection for resonance measurement.
    void startMeasurement() { measuring_ = true; samples_.clear(); }

    /// @brief Stop data collection.
    void stopMeasurement() { measuring_ = false; }

    /// @brief Collect a sample (call periodically during measurement).
    void collectSample() {
        if (!measuring_) return;
        samples_.push_back(read());
    }

    /// @brief Get collected samples.
    const std::vector<Acceleration>& samples() const { return samples_; }

    /// @brief Clear collected samples.
    void clearSamples() { samples_.clear(); }

    /// @brief Check if measurement is in progress.
    bool isMeasuring() const { return measuring_; }

private:
    uint8_t oid_;
    SpiTransferFunc spiTransfer_;
    bool measuring_ = false;
    std::vector<Acceleration> samples_;
};

// ============================================================================
// TSL1401CL filament width sensor
// ============================================================================

/// @brief TSL1401CL linear CCD-based filament width sensor.
/// Measures filament width by reading a 128-pixel CCD array and
/// detecting the shadow cast by the filament.
class Tsl1401clFilamentSensor {
public:
    using AdcReadFunc = std::function<double()>;

    struct Params {
        double nominalWidth = 1.75;    ///< Nominal filament width (mm)
        double tolerance = 0.1;        ///< Width tolerance (mm)
        double minWidth = 1.5;         ///< Minimum valid width (mm)
        double maxWidth = 2.0;         ///< Maximum valid width (mm)
        int pixelCount = 128;          ///< CCD pixel count
        double pixelSpacing = 0.1;     ///< mm per pixel (calibrated)
    };

    Tsl1401clFilamentSensor(uint8_t oid, Params params, AdcReadFunc adcRead)
        : oid_(oid)
        , params_(params)
        , adcRead_(std::move(adcRead)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Read the ADC value and compute filament width.
    /// The ADC reading represents the total light intensity, which
    /// correlates with the filament shadow width.
    /// @return Filament width in mm, or NaN on error.
    double readWidth() const {
        double adcValue = adcRead_();
        if (adcValue < 0) return NAN;

        // Simplified model: the ADC value represents the number of
        // shadowed pixels (higher ADC = more shadow = wider filament).
        // Convert ADC to pixel count, then to width.
        double shadowFraction = adcValue / 4095.0; // 0..1
        int shadowPixels = static_cast<int>(shadowFraction * params_.pixelCount);
        double width = shadowPixels * params_.pixelSpacing;

        if (width < params_.minWidth || width > params_.maxWidth) return NAN;
        return width;
    }

    /// @brief Check if filament width is within tolerance.
    bool widthOk() const {
        double w = readWidth();
        if (std::isnan(w)) return false;
        return std::abs(w - params_.nominalWidth) <= params_.tolerance;
    }

    /// @brief Get the error from nominal width.
    double widthError() const {
        double w = readWidth();
        if (std::isnan(w)) return NAN;
        return w - params_.nominalWidth;
    }

    const Params& params() const { return params_; }

private:
    uint8_t oid_;
    Params params_;
    AdcReadFunc adcRead_;
};

} // namespace tether::klipper::objects
