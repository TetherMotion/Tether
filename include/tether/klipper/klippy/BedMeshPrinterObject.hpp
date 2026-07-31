#pragma once

/// @file BedMeshPrinterObject.hpp
/// @brief BedMesh, QueryEndstops, MotionReport, Adxl345 printer objects for UDS

#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/klippy/KlippyUdsServer.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Printer object exposing bed_mesh status via UDS.
class BedMeshObject : public PrinterObject {
public:
    explicit BedMeshObject(std::shared_ptr<objects::BedMesh> mesh)
        : mesh_(std::move(mesh)) {}

    std::string name() const override { return "bed_mesh"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto addField = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        addField("profile_name", JsonValue(profileName_));
        addField("mesh_min", JsonValue(std::vector<JsonValue>{
            JsonValue(mesh_ ? mesh_->minX() : 0.0),
            JsonValue(mesh_ ? mesh_->minY() : 0.0)
        }));
        addField("mesh_max", JsonValue(std::vector<JsonValue>{
            JsonValue(mesh_ ? mesh_->maxX() : 0.0),
            JsonValue(mesh_ ? mesh_->maxY() : 0.0)
        }));
        addField("matrix", JsonValue(mesh_ && mesh_->isComplete()));
        addField("probed_matrix", JsonValue(mesh_ && mesh_->isComplete()));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"profile_name", "mesh_min", "mesh_max", "matrix", "probed_matrix"};
    }

    void setProfileName(const std::string& name) { profileName_ = name; }

private:
    std::shared_ptr<objects::BedMesh> mesh_;
    std::string profileName_ = "default";
};

/// @brief Printer object exposing endstop states via UDS.
class QueryEndstopsObject : public PrinterObject {
public:
    using EndstopStateFunc = std::function<std::map<std::string, bool>()>;

    explicit QueryEndstopsObject(EndstopStateFunc func)
        : func_(std::move(func)) {}

    std::string name() const override { return "query_endstops"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto states = func_ ? func_() : std::map<std::string, bool>{};
        for (const auto& [axis, triggered] : states) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), axis) != fields.end()) {
                result[axis] = JsonValue(triggered ? "TRIGGERED" : "open");
            }
        }
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"x", "y", "z"};
    }

private:
    EndstopStateFunc func_;
};

/// @brief Printer object exposing motion statistics via UDS.
class MotionReportObject : public PrinterObject {
public:
    std::string name() const override { return "motion_report"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto addField = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        addField("live_position", JsonValue(std::vector<JsonValue>{
            JsonValue(position_[0]), JsonValue(position_[1]),
            JsonValue(position_[2]), JsonValue(position_[3])
        }));
        addField("live_velocity", JsonValue(velocity_));
        addField("live_extruder_velocity", JsonValue(extruderVelocity_));
        addField("steppers", JsonValue(std::vector<JsonValue>{
            JsonValue("stepper_x"), JsonValue("stepper_y"),
            JsonValue("stepper_z"), JsonValue("extruder")
        }));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"live_position", "live_velocity", "live_extruder_velocity", "steppers"};
    }

    void setPosition(const std::array<double, 4>& pos) { position_ = pos; }
    void setVelocity(double v) { velocity_ = v; }
    void setExtruderVelocity(double v) { extruderVelocity_ = v; }

private:
    std::array<double, 4> position_ = {0, 0, 0, 0};
    double velocity_ = 0.0;
    double extruderVelocity_ = 0.0;
};

/// @brief Printer object exposing ADXL345 accelerometer data via UDS.
class Adxl345Object : public PrinterObject {
public:
    explicit Adxl345Object(std::shared_ptr<objects::Adxl345> adxl)
        : adxl_(std::move(adxl)) {}

    std::string name() const override { return "adxl345"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto addField = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        addField("measuring", JsonValue(adxl_ && adxl_->isMeasuring()));
        if (adxl_ && !adxl_->samples().empty()) {
            const auto& acc = adxl_->samples().back();
            addField("last_x", JsonValue(acc.x));
            addField("last_y", JsonValue(acc.y));
            addField("last_z", JsonValue(acc.z));
        }
        addField("samples", JsonValue(static_cast<int64_t>(
            adxl_ ? static_cast<int64_t>(adxl_->samples().size()) : 0)));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"measuring", "last_x", "last_y", "last_z", "samples"};
    }

private:
    std::shared_ptr<objects::Adxl345> adxl_;
};

} // namespace tether::klipper::klippy
