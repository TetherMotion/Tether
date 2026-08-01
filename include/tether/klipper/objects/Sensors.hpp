/**
 * @file Sensors.hpp
 * @brief Sensors: filament sensors, pulse counter, ADXL345 accelerometer, TSL1401CL sensor.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace tether::klipper::objects {

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
