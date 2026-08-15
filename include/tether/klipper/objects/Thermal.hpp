/**
 * @file Thermal.hpp
 * @brief Temperature control peripherals: sensors, heaters, and PID control.
 *
 * This file provides the thermal control peripherals for the Klipper object
 * model:
 *   - TemperatureSensor: base class for temperature measurement
 *   - Thermistor: NTC thermistor via ADC
 *   - Thermocouple: SPI-based thermocouple (MAX31856, etc.)
 *   - Heater: PWM-controlled heater with PID regulation and safety limits
 *
 * The Heater class wraps a PwmOut peripheral and uses a PID controller
 * (from tether_controls) to maintain a target temperature. It includes
 * min_temp / max_temp safety watchdog that triggers shutdown if the
 * sensor reading goes out of bounds.
 */

#pragma once

#include "tether/control/PIDControllers.hpp"
#if TETHER_ENABLE_PRESSURE_ADVANCE
#include "tether/control/extrusion/FlowAdaptiveHeaterController.hpp"
#include "tether/klipper/motion/ExtrusionFlowTracker.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace tether::klipper::objects {

// ============================================================================
// Temperature sensor interface
// ============================================================================

/// @brief Base class for temperature sensors.
class TemperatureSensor {
public:
    explicit TemperatureSensor(uint8_t oid) : oid_(oid) {}
    virtual ~TemperatureSensor() = default;

    uint8_t oid() const { return oid_; }

    /// @brief Read the current temperature in degrees Celsius.
    /// @return Temperature in °C, or NaN on error.
    virtual double read() = 0;

    /// @brief Sensor type name (e.g. "thermistor", "thermocouple").
    virtual std::string type() const = 0;

    /// @brief Last measured temperature.
    double lastTemperature() const { return lastTemp_; }

    /// @brief Update the sensor (called periodically).
    virtual void update() { lastTemp_ = read(); }

protected:
    uint8_t oid_;
    double lastTemp_ = NAN;
};

// ============================================================================
// Thermistor (NTC via ADC)
// ============================================================================

/// @brief NTC thermistor temperature sensor using beta model or Steinhart-Hart.
class Thermistor : public TemperatureSensor {
public:
    struct CalPoint {
        double temperature; ///< Temperature in °C
        double resistance;  ///< Resistance in ohms
    };

    struct Params {
        double pullupResistor = 4700.0;   ///< Pullup resistor in ohms.
        double referenceVoltage = 3.3;    ///< ADC reference voltage.
        double adcMax = 4095.0;           ///< ADC max value (12-bit).
        double resistanceAt25C = 100000.0; ///< Thermistor resistance at 25°C.
        double beta = 3950.0;             ///< Beta coefficient.
        double minTemp = -50.0;           ///< Minimum valid temperature.
        double maxTemp = 300.0;           ///< Maximum valid temperature.
        std::vector<CalPoint> calibrationTable; ///< Multi-point calibration (optional)
    };

    using AdcReadFunc = std::function<double()>;

    Thermistor(uint8_t oid, Params params, AdcReadFunc adcRead)
        : TemperatureSensor(oid)
        , params_(params)
        , adcRead_(std::move(adcRead)) {}

    double read() override {
        double adcValue = adcRead_();
        if (adcValue < 0 || adcValue > params_.adcMax) return NAN;

        // Calculate thermistor resistance from ADC value
        // Voltage divider: V_adc = V_ref * R_therm / (R_pullup + R_therm)
        double vadc = adcValue * params_.referenceVoltage / params_.adcMax;
        if (vadc <= 0 || vadc >= params_.referenceVoltage) return NAN;
        double r_therm = params_.pullupResistor * vadc /
                         (params_.referenceVoltage - vadc);

        double temp_c;
        if (params_.calibrationTable.size() >= 2) {
            // Use calibration table with linear interpolation
            temp_c = interpolateFromTable(r_therm);
        } else {
            // Beta model: 1/T = 1/T0 + (1/beta) * ln(R/R0)
            double t0 = 298.15; // 25°C in Kelvin
            double inv_t = 1.0 / t0 + (1.0 / params_.beta) * std::log(r_therm / params_.resistanceAt25C);
            double temp_k = 1.0 / inv_t;
            temp_c = temp_k - 273.15;
        }

        if (temp_c < params_.minTemp || temp_c > params_.maxTemp) return NAN;
        return temp_c;
    }

    std::string type() const override { return "thermistor"; }

    const Params& params() const { return params_; }

private:
    Params params_;
    AdcReadFunc adcRead_;

    /// @brief Interpolate temperature from resistance using calibration table.
    /// Table is assumed sorted by resistance (descending for NTC).
    double interpolateFromTable(double resistance) const {
        const auto& table = params_.calibrationTable;
        if (table.empty()) return NAN;
        if (table.size() == 1) return table[0].temperature;

        // Find the two surrounding points
        for (size_t i = 0; i + 1 < table.size(); ++i) {
            double r1 = table[i].resistance;
            double r2 = table[i + 1].resistance;
            double t1 = table[i].temperature;
            double t2 = table[i + 1].temperature;
            // Check if resistance is between r1 and r2 (either order)
            if ((resistance <= r1 && resistance >= r2) ||
                (resistance >= r1 && resistance <= r2)) {
                double frac = (r1 != r2) ? (resistance - r1) / (r2 - r1) : 0.0;
                return t1 + frac * (t2 - t1);
            }
        }
        // Extrapolate from the nearest endpoint
        if (resistance > table[0].resistance) return table[0].temperature;
        return table.back().temperature;
    }
};

// ============================================================================
// Thermocouple (SPI-based)
// ============================================================================

/// @brief SPI-based thermocouple sensor (MAX31856, MAX31855, etc.)
class Thermocouple : public TemperatureSensor {
public:
    using SpiTransferFunc = std::function<std::vector<uint8_t>(std::span<const uint8_t>)>;

    enum class Type { K, J, T, E, N, R, S, B };

    Thermocouple(uint8_t oid, Type type, SpiTransferFunc spiTransfer)
        : TemperatureSensor(oid)
        , type_(type)
        , spiTransfer_(std::move(spiTransfer)) {}

    double read() override {
        // Read 4 bytes from the thermocouple device
        std::vector<uint8_t> cmd(4, 0);
        auto resp = spiTransfer_(cmd);
        if (resp.size() < 4) return NAN;

        // MAX31855 format: 32-bit, bits [31:18] = 14-bit signed temp (0.25°C)
        int32_t raw = (static_cast<int32_t>(resp[0]) << 24) |
                      (static_cast<int32_t>(resp[1]) << 16) |
                      (static_cast<int32_t>(resp[2]) << 8) |
                      static_cast<int32_t>(resp[3]);
        // Check fault bit (bit 16)
        if (raw & 0x00010000) return NAN;
        // Extract 14-bit signed temperature
        int16_t temp_raw = static_cast<int16_t>(raw >> 18);
        double temp = temp_raw * 0.25;
        return temp;
    }

    std::string type() const override { return "thermocouple"; }

private:
    Type type_;
    SpiTransferFunc spiTransfer_;
};

// ============================================================================
// RTD (PT100/PT1000 via ADC or SPI)
// ============================================================================

/// @brief RTD (Resistance Temperature Detector) sensor.
/// Supports PT100 and PT1000 using the Callendar-Van Dusen equation.
/// Reads resistance via an ADC (with a reference resistor circuit) or
/// via an SPI-based RTD amplifier (e.g. MAX31865).
class RtdSensor : public TemperatureSensor {
public:
    struct Params {
        double nominalResistance = 100.0;   ///< Resistance at 0°C (100 for PT100, 1000 for PT1000)
        double alpha = 0.003851;            ///< Temperature coefficient of resistance (PT385)
        double referenceResistor = 430.0;   ///< Reference resistor for ADC circuit (ohms)
        double adcMax = 4095.0;             ///< ADC max value (12-bit)
        double referenceVoltage = 3.3;      ///< ADC reference voltage
        double minTemp = -200.0;            ///< Minimum valid temperature
        double maxTemp = 850.0;             ///< Maximum valid temperature
    };

    using AdcReadFunc = std::function<double()>;

    RtdSensor(uint8_t oid, Params params, AdcReadFunc adcRead)
        : TemperatureSensor(oid)
        , params_(params)
        , adcRead_(std::move(adcRead)) {}

    double read() override {
        double adcValue = adcRead_();
        if (adcValue < 0 || adcValue > params_.adcMax) return NAN;

        // Calculate RTD resistance from ADC value using voltage divider:
        // V_adc = V_ref * R_rtd / (R_ref + R_rtd)
        // => R_rtd = R_ref * V_adc / (V_ref - V_adc)
        double vadc = adcValue * params_.referenceVoltage / params_.adcMax;
        if (vadc <= 0 || vadc >= params_.referenceVoltage) return NAN;
        double r_rtd = params_.referenceResistor * vadc /
                       (params_.referenceVoltage - vadc);

        // Callendar-Van Dusen equation:
        // For T >= 0: R(T) = R0 * (1 + A*T + B*T^2)
        //   where A = alpha + alpha * alpha * (1 - delta)
        //   Simplified linear approximation: T = (R/R0 - 1) / alpha
        // For better accuracy, use the quadratic formula:
        //   B*T^2 + A*T + (1 - R/R0) = 0
        //   T = (-A + sqrt(A^2 - 4*B*(1 - R/R0))) / (2*B)
        double ratio = r_rtd / params_.nominalResistance;
        double A = params_.alpha;
        double B = -1.466e-6; // Standard PT385 B coefficient

        double temp_c;
        if (ratio >= 1.0) {
            // Temperature >= 0°C: use quadratic
            double discriminant = A * A - 4.0 * B * (1.0 - ratio);
            if (discriminant < 0) return NAN;
            temp_c = (-A + std::sqrt(discriminant)) / (2.0 * B);
        } else {
            // Temperature < 0°C: use linear approximation (simplified)
            temp_c = (ratio - 1.0) / A;
        }

        if (temp_c < params_.minTemp || temp_c > params_.maxTemp) return NAN;
        return temp_c;
    }

    std::string type() const override { return "rtd"; }

    const Params& params() const { return params_; }

private:
    Params params_;
    AdcReadFunc adcRead_;
};

// ============================================================================
// Heater (PWM-controlled with PID)
// ============================================================================

/// @brief PID controller parameters for a heater.
struct HeaterPidParams {
    double Kp = 0.0;    ///< Proportional gain
    double Ki = 0.0;    ///< Integral gain
    double Kd = 0.0;    ///< Derivative gain
    double imax = 0.0;  ///< Integral windup limit
    double pwmMin = 0.0; ///< Minimum PWM (0.0-1.0)
    double pwmMax = 1.0; ///< Maximum PWM (0.0-1.0)
};

/// @brief Heater peripheral: PWM output with PID temperature control.
class Heater {
public:
    using PwmWriteFunc = std::function<void(double)>;
    using SensorReadFunc = std::function<double()>;
    using ShutdownCallback = std::function<void(const std::string&)>;

    Heater(uint8_t oid, PwmWriteFunc pwmWrite, SensorReadFunc sensorRead)
        : oid_(oid)
        , pwmWrite_(std::move(pwmWrite))
        , sensorRead_(std::move(sensorRead)) {
        // Configure the PID controller with default heater limits.
        pid_.setGains(0.0, 0.0, 0.0);
        applyPidLimits();
    }

    uint8_t oid() const { return oid_; }

    /// @brief Set target temperature in °C.
    void setTarget(double target) { target_ = target; }

    /// @brief Get target temperature.
    double target() const { return target_; }

    /// @brief Get current temperature.
    double currentTemp() const { return currentTemp_; }

    /// @brief Set PID parameters.
    void setPidParams(const HeaterPidParams& params) {
        pidParams_ = params;
        pid_.setGains(params.Kp, params.Ki, params.Kd);
        applyPidLimits();
    }

    /// @brief Get PID parameters.
    const HeaterPidParams& pidParams() const { return pidParams_; }

    /// @brief Set safety limits.
    void setSafetyLimits(double minTemp, double maxTemp) {
        minTemp_ = minTemp;
        maxTemp_ = maxTemp;
    }

    /// @brief Set shutdown callback (called when safety limit is violated).
    void setShutdownCallback(ShutdownCallback cb) {
        shutdownCallback_ = std::move(cb);
    }

    /// @brief Set the control interval in seconds.
    void setControlInterval(double interval) { controlInterval_ = interval; }

#if TETHER_ENABLE_PRESSURE_ADVANCE
    /// @brief Flow-adaptive compensation hook.
    /// When set, control() feeds the current flow (from the tracker) into
    /// the controller and adds its feed-forward output to the PID PWM.
    /// Pass nullptr to disable.
    void setFlowCompensation(
        std::shared_ptr<tether::control::extrusion::FlowAdaptiveHeaterController>
            controller,
        std::shared_ptr<tether::klipper::motion::ExtrusionFlowTracker> tracker) {
        flowController_ = std::move(controller);
        flowTracker_ = std::move(tracker);
    }
    void setFlowCompensation(std::nullptr_t) {
        flowController_.reset();
        flowTracker_.reset();
    }

    /// @brief Estimated melt-zone temperature from the flow controller's
    /// internal observer (0.0 if no controller is wired).
    double meltTempEstimate() const {
        return flowController_ ? flowController_->meltTempEst() : 0.0;
    }
    /// @brief Last pre-emphasis PWM contribution (0.0 if none).
    double preEmphasisPwm() const {
        return flowController_ ? flowController_->emphasis().preEmphasisPwm : 0.0;
    }
    /// @brief Last post-emphasis PWM contribution (0.0 if none).
    double postEmphasisPwm() const {
        return flowController_ ? flowController_->emphasis().postEmphasisPwm : 0.0;
    }
#endif

    /// @brief Run one PID control iteration.
    /// @return PWM output (0.0 to 1.0).
    double control() {
        double measured = sensorRead_();
        currentTemp_ = measured;

        // Safety check
        if (!std::isnan(measured)) {
            if (measured < minTemp_ || measured > maxTemp_) {
                if (shutdownCallback_) {
                    shutdownCallback_(std::format("Temperature out of range: {}°C", measured));
                }
                pwmWrite_(0.0);
                return 0.0;
            }
        } else {
            // Sensor read failure
            if (shutdownCallback_) {
                shutdownCallback_("Temperature sensor read failure");
            }
            pwmWrite_(0.0);
            return 0.0;
        }

#if TETHER_ENABLE_PRESSURE_ADVANCE
        // Flow-adaptive path: delegate to the controller, which runs the
        // PID backend internally and adds pre/post-emphasis feed-forward.
        if (flowController_) {
            if (flowTracker_) {
                flowController_->setFlow(
                    flowTracker_->smoothedFlowMm3PerS());
            }
            Control::ControllerInput input{};
            input.reference = target_;
            input.measured = measured;
            input.dt = controlInterval_;
            auto output = flowController_->compute(input);
            lastOutput_ = output;
            double pwm = std::clamp(output.control, pidParams_.pwmMin,
                                    pidParams_.pwmMax);
            pwmWrite_(pwm);
            return pwm;
        }
#endif

        // PID computation via Control::PIDController
        Control::ControllerInput input{};
        input.reference = target_;
        input.measured = measured;
        input.dt = controlInterval_;
        auto output = pid_.compute(input);
        lastOutput_ = output;

        double pwm = std::clamp(output.control, pidParams_.pwmMin, pidParams_.pwmMax);
        pwmWrite_(pwm);
        return pwm;
    }

    /// @brief Manually apply a power level and update the temperature reading.
    /// Used by PID autotune to drive the heater with a fixed duty cycle.
    /// @param power PWM duty cycle to apply (0.0 to 1.0).
    /// @param dt Time step in seconds (unused, kept for API compatibility).
    /// @return The applied PWM output.
    double update(double power, double dt) {
        (void)dt;
        power = std::clamp(power, 0.0, 1.0);
        pwmWrite_(power);
        double measured = sensorRead_();
        if (!std::isnan(measured)) {
            currentTemp_ = measured;
        }
        return power;
    }

    /// @brief Reset PID state.
    void reset() {
        pid_.reset();
        lastOutput_ = {};
        target_ = 0.0;
        pwmWrite_(0.0);
    }

    /// @brief Check if heater is at target temperature (within tolerance).
    bool atTarget(double tolerance = 2.0) const {
        if (target_ <= 0.0) return true;
        return std::abs(currentTemp_ - target_) < tolerance;
    }

    /// @brief Get PID diagnostic values.
    struct PidState {
        double error;
        double integral;
        double derivative;
        double output;
    };

    PidState pidState() const {
        return {lastOutput_.error, lastOutput_.integral,
                lastOutput_.derivative, lastOutput_.control};
    }

private:
    /// @brief Apply integral and output limits from pidParams_ to the PID controller.
    void applyPidLimits() {
        Control::SaturationLimits limits;
        limits.outputMin = pidParams_.pwmMin;
        limits.outputMax = pidParams_.pwmMax;
        limits.integralMin = -pidParams_.imax;
        limits.integralMax = pidParams_.imax;
        pid_.setSaturationLimits(limits);
    }

    uint8_t oid_;
    PwmWriteFunc pwmWrite_;
    SensorReadFunc sensorRead_;
    ShutdownCallback shutdownCallback_;

    double target_ = 0.0;
    double currentTemp_ = NAN;
    double minTemp_ = -50.0;
    double maxTemp_ = 300.0;

    HeaterPidParams pidParams_;
    Control::PIDController pid_;
    Control::ControllerOutput lastOutput_;
    double controlInterval_ = 0.1; // 100ms default
#if TETHER_ENABLE_PRESSURE_ADVANCE
    std::shared_ptr<tether::control::extrusion::FlowAdaptiveHeaterController>
        flowController_;
    std::shared_ptr<tether::klipper::motion::ExtrusionFlowTracker> flowTracker_;
#endif
};

// ============================================================================
// Heater printer object (for UDS status queries)
// ============================================================================

/// @brief Heater status for the UDS object model.
struct HeaterStatus {
    double temperature = 0.0;
    double target = 0.0;
    double power = 0.0;
    std::string state = "off"; ///< "off", "active", "stable"
};

} // namespace tether::klipper::objects
