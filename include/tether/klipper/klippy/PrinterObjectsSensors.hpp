#pragma once

/// @file PrinterObjectsSensors.hpp
/// @brief Sensor printer objects (temperature_sensor, filament sensors, load_cell, etc.)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief The temperature_sensor printer object.
class TemperatureSensorObject : public PrinterObject {
public:
    explicit TemperatureSensorObject(const std::string& name,
                                     std::shared_ptr<objects::TemperatureSensor> sensor)
        : name_(name), sensor_(std::move(sensor)) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (sensor_) {
            add("temperature", JsonValue(sensor_->read()));
        } else {
            add("temperature", JsonValue(temperature_));
        }
        add("measured_min_temp", JsonValue(minTemp_));
        add("measured_max_temp", JsonValue(maxTemp_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "measured_min_temp", "measured_max_temp"};
    }

    void setTemperature(double t) {
        temperature_ = t;
        if (std::isnan(minTemp_) || t < minTemp_) minTemp_ = t;
        if (std::isnan(maxTemp_) || t > maxTemp_) maxTemp_ = t;
    }

private:
    std::string name_;
    std::shared_ptr<objects::TemperatureSensor> sensor_;
    double temperature_ = 0.0;
    double minTemp_ = NAN;
    double maxTemp_ = NAN;
};

/// @brief The filament_switch_sensor printer object.
class FilamentSwitchSensorObject : public PrinterObject {
public:
    explicit FilamentSwitchSensorObject(const std::string& name,
                                        std::shared_ptr<objects::FilamentSensor> sensor)
        : name_(name), sensor_(std::move(sensor)) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (sensor_) {
            add("filament_detected", JsonValue(sensor_->filamentPresent()));
        } else {
            add("filament_detected", JsonValue(filamentDetected_));
        }
        add("enabled", JsonValue(enabled_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"filament_detected", "enabled"};
    }

    void setFilamentDetected(bool d) { filamentDetected_ = d; }
    void setEnabled(bool e) { enabled_ = e; }

private:
    std::string name_;
    std::shared_ptr<objects::FilamentSensor> sensor_;
    bool filamentDetected_ = true;
    bool enabled_ = true;
};

/// @brief Analog input printer object.
class AnalogInObject : public PrinterObject {
public:
    explicit AnalogInObject(std::shared_ptr<objects::AnalogIn> dev,
                            const std::string& name = "analog_pin")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (dev_) {
            add("value", JsonValue(static_cast<double>(dev_->lastSample())));
        }
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"value"};
    }

private:
    std::shared_ptr<objects::AnalogIn> dev_;
    std::string name_;
};

/// @brief Hall filament sensor printer object.
class HallFilamentSensorObject : public PrinterObject {
public:
    explicit HallFilamentSensorObject(std::shared_ptr<objects::HallFilamentSensor> dev,
                                      const std::string& name = "hall_filament_sensor")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (dev_) {
            add("diameter", JsonValue(dev_->diameter()));
            add("filament_present", JsonValue(dev_->withinTolerance()));
        }
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"diameter", "filament_present"};
    }

private:
    std::shared_ptr<objects::HallFilamentSensor> dev_;
    std::string name_;
};

/// @brief Pulse counter printer object.
class PulseCounterObject : public PrinterObject {
public:
    explicit PulseCounterObject(std::shared_ptr<objects::PulseCounter> dev,
                                const std::string& name = "pulse_counter")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (dev_) {
            add("count", JsonValue(static_cast<int64_t>(dev_->count())));
            add("rate", JsonValue(dev_->pulseRate()));
        }
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"count", "rate"};
    }

private:
    std::shared_ptr<objects::PulseCounter> dev_;
    std::string name_;
};

/// @brief Filament motion sensor printer object (filament_motion_sensor).
class FilamentMotionSensorObject : public PrinterObject {
public:
    explicit FilamentMotionSensorObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("filament_detected", JsonValue(filamentDetected_));
        add("enabled", JsonValue(enabled_));
        add("distance", JsonValue(distance_));
        add("detection_distance", JsonValue(detectionDistance_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"filament_detected", "enabled", "distance", "detection_distance"};
    }

    void setFilamentDetected(bool d) { filamentDetected_ = d; }
    void setEnabled(bool e) { enabled_ = e; }
    void setDistance(double d) { distance_ = d; }
    void setDetectionDistance(double d) { detectionDistance_ = d; }

private:
    std::string name_;
    bool filamentDetected_ = true;
    bool enabled_ = true;
    double distance_ = 0.0;
    double detectionDistance_ = 7.0;
};

/// @brief Load cell printer object (load_cell).
class LoadCellObject : public PrinterObject {
public:
    explicit LoadCellObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("load", JsonValue(load_));
        add("tare_value", JsonValue(tareValue_));
        add("threshold", JsonValue(threshold_));
        add("min_load", JsonValue(minLoad_));
        add("max_load", JsonValue(maxLoad_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"load", "tare_value", "threshold", "min_load", "max_load"};
    }

    void setLoad(double l) { load_ = l; }
    void setTareValue(double t) { tareValue_ = t; }
    void setThreshold(double t) { threshold_ = t; }

private:
    std::string name_;
    double load_ = 0.0;
    double tareValue_ = 0.0;
    double threshold_ = 0.0;
    double minLoad_ = 0.0;
    double maxLoad_ = 0.0;
};

/// @brief Angle sensor printer object (angle).
class AngleObject : public PrinterObject {
public:
    explicit AngleObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("angle", JsonValue(angle_));
        add("velocity", JsonValue(velocity_));
        add("error", JsonValue(error_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"angle", "velocity", "error"};
    }

    void setAngle(double a) { angle_ = a; }
    void setVelocity(double v) { velocity_ = v; }
    void setError(double e) { error_ = e; }

private:
    std::string name_;
    double angle_ = 0.0;
    double velocity_ = 0.0;
    double error_ = 0.0;
};

} // namespace tether::klipper::klippy
