#pragma once

/// @file PrinterObjectsFans.hpp
/// @brief Fan printer objects (temperature_fan, controller_fan, heater_fan, fan_generic)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
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

/// @brief Temperature fan printer object (temperature_fan).
class TemperatureFanObject : public PrinterObject {
public:
    explicit TemperatureFanObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("temperature", JsonValue(temperature_));
        s.add("target", JsonValue(target_));
        s.add("speed", JsonValue(speed_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "target", "speed"};
    }

    void setTemperature(double t) { temperature_ = t; }
    void setTarget(double t) { target_ = t; }
    void setSpeed(double s) { speed_ = s; }

private:
    std::string name_;
    double temperature_ = 0.0;
    double target_ = 0.0;
    double speed_ = 0.0;
};

/// @brief Controller fan printer object (controller_fan).
class ControllerFanObject : public PrinterObject {
public:
    explicit ControllerFanObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("speed", JsonValue(speed_));
        s.add("rpm", JsonValue(rpm_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"speed", "rpm"}; }

    void setSpeed(double s) { speed_ = s; }
    void setRpm(double r) { rpm_ = r; }

private:
    std::string name_;
    double speed_ = 0.0;
    double rpm_ = 0.0;
};

/// @brief Heater fan printer object (heater_fan).
class HeaterFanObject : public PrinterObject {
public:
    explicit HeaterFanObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("speed", JsonValue(speed_));
        s.add("rpm", JsonValue(rpm_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"speed", "rpm"}; }

    void setSpeed(double s) { speed_ = s; }
    void setRpm(double r) { rpm_ = r; }

private:
    std::string name_;
    double speed_ = 0.0;
    double rpm_ = 0.0;
};

/// @brief Generic fan printer object (fan_generic).
class FanGenericObject : public PrinterObject {
public:
    explicit FanGenericObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("speed", JsonValue(speed_));
        s.add("rpm", JsonValue(rpm_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"speed", "rpm"}; }

    void setSpeed(double s) { speed_ = s; }
    void setRpm(double r) { rpm_ = r; }

private:
    std::string name_;
    double speed_ = 0.0;
    double rpm_ = 0.0;
};

} // namespace tether::klipper::klippy
