#pragma once

/// @file PrinterObjectsHeaters.hpp
/// @brief Heater printer objects (heater_generic, temperature_probe)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/Thermal.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Generic heater printer object (heater_generic).
class HeaterGenericObject : public PrinterObject {
public:
    explicit HeaterGenericObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("temperature", JsonValue(temperature_));
        s.add("target", JsonValue(target_));
        s.add("power", JsonValue(power_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "target", "power"};
    }

    void setTemperature(double t) { temperature_ = t; }
    void setTarget(double t) { target_ = t; }
    void setPower(double p) { power_ = p; }

private:
    std::string name_;
    double temperature_ = 0.0;
    double target_ = 0.0;
    double power_ = 0.0;
};

/// @brief Temperature probe printer object (temperature_probe).
class TemperatureProbeObject : public PrinterObject {
public:
    explicit TemperatureProbeObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("temperature", JsonValue(temperature_));
        s.add("measured_min_temp", JsonValue(measuredMinTemp_));
        s.add("measured_max_temp", JsonValue(measuredMaxTemp_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "measured_min_temp", "measured_max_temp"};
    }

    void setTemperature(double t) { temperature_ = t; }
    void setMinTemp(double t) { measuredMinTemp_ = t; }
    void setMaxTemp(double t) { measuredMaxTemp_ = t; }

private:
    std::string name_;
    double temperature_ = 0.0;
    double measuredMinTemp_ = 0.0;
    double measuredMaxTemp_ = 0.0;
};

} // namespace tether::klipper::klippy
