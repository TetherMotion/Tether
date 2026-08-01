#pragma once

/// @file PrinterObjectsAdvanced.hpp
/// @brief Advanced printer objects (firmware_retraction, exclude_object, z_thermal_adjust)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Printer object for firmware_retraction (G10/G11 state).
class FirmwareRetractionObject : public PrinterObject {
public:
    explicit FirmwareRetractionObject(std::shared_ptr<FirmwareRetraction> fr)
        : fr_(fr) {}

    std::string name() const override { return "firmware_retraction"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (fr_) {
            const auto& p = fr_->params();
            s.add("retract_length", JsonValue(p.retractLength));
            s.add("retract_speed", JsonValue(p.retractSpeed));
            s.add("unretract_extra_length", JsonValue(p.unretractLength));
            s.add("unretract_speed", JsonValue(p.unretractSpeed));
            s.add("z_hop", JsonValue(p.zHop));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"retract_length", "retract_speed",
                "unretract_extra_length", "unretract_speed", "z_hop"};
    }

private:
    std::shared_ptr<FirmwareRetraction> fr_;
};

/// @brief Exclude object printer object (exclude_object).
class ExcludeObjectObject : public PrinterObject {
public:
    std::string name() const override { return "exclude_object"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("current_object", JsonValue(currentObject_));
        std::vector<JsonValue> excluded;
        for (const auto& e : excludedObjects_) excluded.emplace_back(e);
        s.add("excluded_objects", JsonValue(excluded));
        std::vector<JsonValue> objs;
        for (const auto& o : objects_) objs.push_back(JsonValue(o));
        s.add("objects", JsonValue(objs));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"current_object", "excluded_objects", "objects"};
    }

    void setCurrentObject(const std::string& n) { currentObject_ = n; }
    void setExcludedObjects(const std::vector<std::string>& e) { excludedObjects_ = e; }
    void setObjects(const std::vector<std::string>& o) { objects_ = o; }

private:
    std::string currentObject_;
    std::vector<std::string> excludedObjects_;
    std::vector<std::string> objects_;
};

/// @brief Z thermal adjust printer object (z_thermal_adjust).
class ZThermalAdjustObject : public PrinterObject {
public:
    std::string name() const override { return "z_thermal_adjust"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("enabled", JsonValue(enabled_));
        s.add("measured", JsonValue(measured_));
        s.add("adjust", JsonValue(adjust_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"enabled", "measured", "adjust"};
    }

    void setEnabled(bool e) { enabled_ = e; }
    void setMeasured(double m) { measured_ = m; }
    void setAdjust(double a) { adjust_ = a; }

private:
    bool enabled_ = false;
    double measured_ = 0.0;
    double adjust_ = 0.0;
};

} // namespace tether::klipper::klippy
