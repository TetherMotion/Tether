#pragma once

/// @file PrinterObjectsMisc.hpp
/// @brief Miscellaneous printer objects (gcode_macro, output_pin, servo, menu, etc.)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief The gcode_macro printer object (one per macro).
class GcodeMacroObject : public PrinterObject {
public:
    GcodeMacroObject(const std::string& macroName,
                     std::shared_ptr<GcodeMacroRegistry> registry)
        : name_("gcode_macro " + macroName)
        , macroName_(macroName)
        , registry_(std::move(registry)) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("enabled", JsonValue(true));
        if (registry_) {
            auto* m = registry_->getMacro(macroName_);
            if (m) {
                s.add("description", JsonValue(m->description));
            }
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"enabled", "description"};
    }

private:
    std::string name_;
    std::string macroName_;
    std::shared_ptr<GcodeMacroRegistry> registry_;
};

/// @brief Output pin printer object (output_pin).
class OutputPinObject : public PrinterObject {
public:
    explicit OutputPinObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("value", JsonValue(value_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"value"}; }

    void setValue(double v) { value_ = v; }

private:
    std::string name_;
    double value_ = 0.0;
};

/// @brief PWM tool printer object (pwm_tool).
class PWMToolObject : public PrinterObject {
public:
    explicit PWMToolObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("value", JsonValue(value_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"value"}; }

    void setValue(double v) { value_ = v; }

private:
    std::string name_;
    double value_ = 0.0;
};

/// @brief Servo printer object (servo).
class ServoObject : public PrinterObject {
public:
    explicit ServoObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("angle", JsonValue(angle_));
        s.add("width", JsonValue(width_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"angle", "width"}; }

    void setAngle(double a) { angle_ = a; }
    void setWidth(double w) { width_ = w; }

private:
    std::string name_;
    double angle_ = 0.0;
    double width_ = 0.0;
};

/// @brief Multi-pin printer object (multi_pin).
class MultiPinObject : public PrinterObject {
public:
    explicit MultiPinObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::vector<JsonValue> pinArr;
        for (const auto& p : pins_) pinArr.push_back(JsonValue(p));
        s.add("pins", JsonValue(pinArr));
        s.add("value", JsonValue(value_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"pins", "value"};
    }

    void setPins(const std::vector<std::string>& pins) { pins_ = pins; }
    void setValue(const std::string& v) { value_ = v; }

private:
    std::string name_;
    std::vector<std::string> pins_;
    std::string value_ = "0";
};

/// @brief Button printer object (button).
class ButtonObject : public PrinterObject {
public:
    explicit ButtonObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("state", JsonValue(state_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"state"}; }

    void setState(const std::string& s) { state_ = s; }

private:
    std::string name_;
    std::string state_ = "PRESSED";
};

/// @brief Smart effector printer object (smart_effector).
class SmartEffectorObject : public PrinterObject {
public:
    std::string name() const override { return "smart_effector"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("probe_state", JsonValue(probeState_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"probe_state"}; }

    void setProbeState(const std::string& s) { probeState_ = s; }

private:
    std::string probeState_ = "open";
};

/// @brief CAN bus statistics printer object (canbus_stats).
class CanbusStatsObject : public PrinterObject {
public:
    explicit CanbusStatsObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("rx_error", JsonValue(static_cast<int64_t>(rxError_)));
        s.add("tx_error", JsonValue(static_cast<int64_t>(txError_)));
        s.add("tx_overflow", JsonValue(static_cast<int64_t>(txOverflow_)));
        s.add("rx_overflow", JsonValue(static_cast<int64_t>(rxOverflow_)));
        s.add("bus_state", JsonValue(busState_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"rx_error", "tx_error", "tx_overflow", "rx_overflow", "bus_state"};
    }

    void setRxError(uint32_t e) { rxError_ = e; }
    void setTxError(uint32_t e) { txError_ = e; }
    void setBusState(const std::string& s) { busState_ = s; }

private:
    std::string name_;
    uint32_t rxError_ = 0;
    uint32_t txError_ = 0;
    uint32_t txOverflow_ = 0;
    uint32_t rxOverflow_ = 0;
    std::string busState_ = "active";
};

/// @brief PWM cycle time printer object (pwm_cycle_time).
class PWMCycleTimeObject : public PrinterObject {
public:
    explicit PWMCycleTimeObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("value", JsonValue(value_));
        s.add("cycle_time", JsonValue(cycleTime_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"value", "cycle_time"};
    }

    void setValue(double v) { value_ = v; }
    void setCycleTime(double t) { cycleTime_ = t; }

private:
    std::string name_;
    double value_ = 0.0;
    double cycleTime_ = 0.100;
};

/// @brief Resonance tester printer object (resonance_tester).
class ResonanceTesterObject : public PrinterObject {
public:
    std::string name() const override { return "resonance_tester"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("min_freq", JsonValue(minFreq_));
        s.add("max_freq", JsonValue(maxFreq_));
        s.add("accel_per_hz", JsonValue(accelPerHz_));
        s.add("hz_per_sec", JsonValue(hzPerSec_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"min_freq", "max_freq", "accel_per_hz", "hz_per_sec"};
    }

    void setMinFreq(double f) { minFreq_ = f; }
    void setMaxFreq(double f) { maxFreq_ = f; }
    void setAccelPerHz(double a) { accelPerHz_ = a; }
    void setHzPerSec(double h) { hzPerSec_ = h; }

private:
    double minFreq_ = 5.0;
    double maxFreq_ = 133.5;
    double accelPerHz_ = 75.0;
    double hzPerSec_ = 1.0;
};

/// @brief Palette 2 printer object (palette2).
class Palette2Object : public PrinterObject {
public:
    std::string name() const override { return "palette2"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("connected", JsonValue(connected_));
        s.add("loading", JsonValue(loading_));
        s.add("splicing", JsonValue(splicing_));
        s.add("unload_speed", JsonValue(unloadSpeed_));
        s.add("load_speed", JsonValue(loadSpeed_));
        s.add("ping_load_distance", JsonValue(pingLoadDistance_));
        s.add("long_load_speed", JsonValue(longLoadSpeed_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"connected", "loading", "splicing", "unload_speed",
                "load_speed", "ping_load_distance", "long_load_speed"};
    }

    void setConnected(bool c) { connected_ = c; }
    void setLoading(bool l) { loading_ = l; }
    void setSplicing(bool s) { splicing_ = s; }

private:
    bool connected_ = false;
    bool loading_ = false;
    bool splicing_ = false;
    double unloadSpeed_ = 2.0;
    double loadSpeed_ = 2.0;
    double pingLoadDistance_ = 20.0;
    double longLoadSpeed_ = 1.0;
};

/// @brief Menu printer object (menu).
class MenuObject : public PrinterObject {
public:
    std::string name() const override { return "menu"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("enabled", JsonValue(enabled_));
        s.add("timeout", JsonValue(timeout_));
        s.add("rows", JsonValue(static_cast<int64_t>(rows_)));
        s.add("cols", JsonValue(static_cast<int64_t>(cols_)));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"enabled", "timeout", "rows", "cols"};
    }

    void setEnabled(bool e) { enabled_ = e; }
    void setTimeout(int t) { timeout_ = t; }

private:
    bool enabled_ = false;
    int timeout_ = 0;
    int rows_ = 4;
    int cols_ = 20;
};

/// @brief G-code printer object (gcode).
class GcodeObject : public PrinterObject {
public:
    std::string name() const override { return "gcode"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("commands", JsonValue(static_cast<int64_t>(commands_)));
        s.add("info", JsonValue(info_));
        s.add("config_commands", JsonValue(static_cast<int64_t>(configCommands_)));
        std::vector<JsonValue> posArr;
        for (double v : moveGcodePosition_) posArr.push_back(JsonValue(v));
        s.add("move_gcode_position", JsonValue(posArr));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"commands", "info", "config_commands", "move_gcode_position"};
    }

    void setCommands(uint64_t c) { commands_ = c; }
    void setInfo(const std::string& i) { info_ = i; }

private:
    uint64_t commands_ = 0;
    std::string info_;
    uint64_t configCommands_ = 0;
    std::array<double, 4> moveGcodePosition_ = {0, 0, 0, 0};
};

} // namespace tether::klipper::klippy
