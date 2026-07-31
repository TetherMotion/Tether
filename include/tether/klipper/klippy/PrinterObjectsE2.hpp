#pragma once

/// @file PrinterObjectsE2.hpp
/// @brief Additional printer objects: adxl345, delayed_gcode, save_variables, board_pins.

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

// ============================================================================
// ADXL345 printer object already exists in BedMeshPrinterObject.hpp.
// This file adds: delayed_gcode, save_variables, board_pins.
// ============================================================================

// ============================================================================
// Delayed G-code printer object
// ============================================================================

/// @brief The delayed_gcode printer object for delayed G-code status.
class DelayedGcodeObject : public PrinterObject {
public:
    explicit DelayedGcodeObject(const std::string& name)
        : name_(name) {}

    std::string name() const override { return name_; }

    void setEnabled(bool enabled) { enabled_ = enabled; }
    void setRemainingDuration(double duration) { remainingDuration_ = duration; }
    void setGcode(const std::string& gcode) { gcode_ = gcode; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> s;
        auto emit = [&](const std::string& key, JsonValue val) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), key) != fields.end())
                s[key] = val;
        };
        emit("enabled", JsonValue(enabled_));
        emit("remaining_duration", JsonValue(remainingDuration_));
        emit("gcode", JsonValue(gcode_));
        return s;
    }

    std::vector<std::string> availableFields() const override {
        return {"enabled", "remaining_duration", "gcode"};
    }

private:
    std::string name_;
    bool enabled_ = false;
    double remainingDuration_ = 0.0;
    std::string gcode_;
};

// ============================================================================
// Save variables printer object
// ============================================================================

/// @brief The save_variables printer object for saved variable state.
class SaveVariablesObject : public PrinterObject {
public:
    explicit SaveVariablesObject(const std::string& name = "save_variables")
        : name_(name) {}

    std::string name() const override { return name_; }

    void setVariable(const std::string& key, const std::string& value) {
        variables_[key] = value;
    }

    void removeVariable(const std::string& key) {
        variables_.erase(key);
    }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> s;
        auto emit = [&](const std::string& key, JsonValue val) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), key) != fields.end())
                s[key] = val;
        };
        // Report all variables as a nested object
        std::map<std::string, JsonValue> varsMap;
        for (const auto& [k, v] : variables_) {
            varsMap[k] = JsonValue(v);
        }
        emit("variables", JsonValue(varsMap));
        return s;
    }

    std::vector<std::string> availableFields() const override {
        return {"variables"};
    }

private:
    std::string name_;
    std::map<std::string, std::string> variables_;
};

// ============================================================================
// Board pins printer object
// ============================================================================

/// @brief The board_pins printer object for board pin mapping.
class BoardPinsObject : public PrinterObject {
public:
    explicit BoardPinsObject(const std::string& name = "board_pins")
        : name_(name) {}

    std::string name() const override { return name_; }

    /// @brief Add a pin alias mapping.
    void addAlias(const std::string& alias, const std::string& pin) {
        aliases_[alias] = pin;
    }

    /// @brief Set the MCU name for this board.
    void setMcuName(const std::string& mcu) { mcuName_ = mcu; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> s;
        auto emit = [&](const std::string& key, JsonValue val) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), key) != fields.end())
                s[key] = val;
        };
        emit("mcu", JsonValue(mcuName_));
        // Report aliases as a nested object
        std::map<std::string, JsonValue> aliasMap;
        for (const auto& [alias, pin] : aliases_) {
            aliasMap[alias] = JsonValue(pin);
        }
        emit("aliases", JsonValue(aliasMap));
        return s;
    }

    std::vector<std::string> availableFields() const override {
        return {"mcu", "aliases"};
    }

private:
    std::string name_;
    std::string mcuName_;
    std::map<std::string, std::string> aliases_;
};

} // namespace tether::klipper::klippy
