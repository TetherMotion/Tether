/**
 * @file Gpio.hpp
 * @brief Digital and analog GPIO peripherals: DigitalOut, PwmOut, AnalogIn and proxies.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
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

} // namespace tether::klipper::objects
