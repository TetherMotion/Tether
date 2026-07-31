#pragma once

/// @file PrinterObjectsProbes.hpp
/// @brief Probe and bed leveling printer objects

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief The probe printer object.
class ProbeObject : public PrinterObject {
public:
    explicit ProbeObject(std::shared_ptr<objects::Probe> probe)
        : probe_(std::move(probe)) {}

    std::string name() const override { return "probe"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (probe_) {
            add("last_query", JsonValue(probe_->triggered()));
        } else {
            add("last_query", JsonValue(lastQuery_));
        }
        add("last_z_result", JsonValue(lastZResult_));
        add("z_offset", JsonValue(probe_ ? probe_->zOffset() : zOffset_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"last_query", "last_z_result", "z_offset"};
    }

    void setLastQuery(bool q) { lastQuery_ = q; }
    void setLastZResult(double z) { lastZResult_ = z; }
    void setZOffset(double z) { zOffset_ = z; }

private:
    std::shared_ptr<objects::Probe> probe_;
    bool lastQuery_ = false;
    double lastZResult_ = NAN;
    double zOffset_ = 0.0;
};

/// @brief BLTouch printer object (bltouch).
class BltouchObject : public PrinterObject {
public:
    std::string name() const override { return "bltouch"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("last_state", JsonValue(lastState_));
        add("last_z_value", JsonValue(lastZValue_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"last_state", "last_z_value"};
    }

    void setLastState(const std::string& s) { lastState_ = s; }
    void setLastZValue(double z) { lastZValue_ = z; }

private:
    std::string lastState_ = "open";
    double lastZValue_ = 0.0;
};

/// @brief Z tilt printer object (z_tilt).
class ZTiltObject : public PrinterObject {
public:
    std::string name() const override { return "z_tilt"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("applied", JsonValue(applied_));
        add("z_positions", JsonValue(zPositions_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"applied", "z_positions"};
    }

    void setApplied(bool a) { applied_ = a; }
    void setZPositions(const std::vector<JsonValue>& z) { zPositions_ = z; }

private:
    bool applied_ = false;
    std::vector<JsonValue> zPositions_;
};

/// @brief Quad gantry level printer object (quad_gantry_level).
class QuadGantryLevelObject : public PrinterObject {
public:
    std::string name() const override { return "quad_gantry_level"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("applied", JsonValue(applied_));
        add("z_values", JsonValue(zValues_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"applied", "z_values"};
    }

    void setApplied(bool a) { applied_ = a; }
    void setZValues(const std::vector<JsonValue>& z) { zValues_ = z; }

private:
    bool applied_ = false;
    std::vector<JsonValue> zValues_;
};

/// @brief Screws tilt adjust printer object (screws_tilt_adjust).
class ScrewsTiltAdjustObject : public PrinterObject {
public:
    std::string name() const override { return "screws_tilt_adjust"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("error", JsonValue(error_));
        add("max_base", JsonValue(maxBase_));
        add("base", JsonValue(base_));
        add("adjusted", JsonValue(adjusted_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"error", "max_base", "base", "adjusted"};
    }

    void setError(bool e) { error_ = e; }
    void setMaxBase(double m) { maxBase_ = m; }
    void setBase(const std::map<std::string, JsonValue>& b) { base_ = b; }
    void setAdjusted(bool a) { adjusted_ = a; }

private:
    bool error_ = false;
    double maxBase_ = 0.0;
    std::map<std::string, JsonValue> base_;
    bool adjusted_ = false;
};

/// @brief Bed screws printer object (bed_screws).
class BedScrewsObject : public PrinterObject {
public:
    std::string name() const override { return "bed_screws"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("state", JsonValue(state_));
        add("current_screw", JsonValue(currentScrew_));
        add("accepted_screws", JsonValue(acceptedScrews_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"state", "current_screw", "accepted_screws"};
    }

    void setState(const std::string& s) { state_ = s; }
    void setCurrentScrew(int s) { currentScrew_ = s; }
    void setAcceptedScrews(int s) { acceptedScrews_ = s; }

private:
    std::string state_ = "";
    int currentScrew_ = -1;
    int acceptedScrews_ = 0;
};

/// @brief Delta calibrate printer object (delta_calibrate).
class DeltaCalibrateObject : public PrinterObject {
public:
    std::string name() const override { return "delta_calibrate"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("radius", JsonValue(radius_));
        add("positions", JsonValue(positions_));
        add("deltas", JsonValue(deltas_));
        add("applied", JsonValue(applied_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"radius", "positions", "deltas", "applied"};
    }

    void setRadius(double r) { radius_ = r; }
    void setApplied(bool a) { applied_ = a; }

private:
    double radius_ = 0.0;
    std::vector<JsonValue> positions_;
    std::vector<JsonValue> deltas_;
    bool applied_ = false;
};

/// @brief Safe Z home printer object (safe_z_home).
class SafeZHomeObject : public PrinterObject {
public:
    std::string name() const override { return "safe_z_home"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("home_xy_position", JsonValue(homeXyPosition_));
        add("z_hop", JsonValue(zHop_));
        add("z_hop_speed", JsonValue(zHopSpeed_));
        add("xy_home_speed", JsonValue(xyHomeSpeed_));
        add("move_to_previous", JsonValue(moveToPrevious_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"home_xy_position", "z_hop", "z_hop_speed", "xy_home_speed", "move_to_previous"};
    }

    void setHomeXyPosition(const std::string& pos) { homeXyPosition_ = pos; }
    void setZHop(double z) { zHop_ = z; }
    void setZHopSpeed(double s) { zHopSpeed_ = s; }
    void setXyHomeSpeed(double s) { xyHomeSpeed_ = s; }
    void setMoveToPrevious(bool m) { moveToPrevious_ = m; }

private:
    std::string homeXyPosition_ = "0, 0";
    double zHop_ = 10.0;
    double zHopSpeed_ = 20.0;
    double xyHomeSpeed_ = 50.0;
    bool moveToPrevious_ = false;
};

/// @brief Bed tilt printer object (bed_tilt).
class BedTiltObject : public PrinterObject {
public:
    std::string name() const override { return "bed_tilt"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("x_adjust", JsonValue(xAdjust_));
        add("y_adjust", JsonValue(yAdjust_));
        add("applied", JsonValue(applied_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"x_adjust", "y_adjust", "applied"};
    }

    void setXAdjust(double x) { xAdjust_ = x; }
    void setYAdjust(double y) { yAdjust_ = y; }
    void setApplied(bool a) { applied_ = a; }

private:
    double xAdjust_ = 0.0;
    double yAdjust_ = 0.0;
    bool applied_ = false;
};

/// @brief Manual probe printer object (manual_probe).
class ManualProbeObject : public PrinterObject {
public:
    std::string name() const override { return "manual_probe"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("is_active", JsonValue(isActive_));
        add("z_position", JsonValue(zPosition_));
        add("z_position_lower", JsonValue(zPositionLower_));
        add("z_position_upper", JsonValue(zPositionUpper_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"is_active", "z_position", "z_position_lower", "z_position_upper"};
    }

    void setActive(bool a) { isActive_ = a; }
    void setZPosition(double z) { zPosition_ = z; }
    void setZPositionLower(double z) { zPositionLower_ = z; }
    void setZPositionUpper(double z) { zPositionUpper_ = z; }

private:
    bool isActive_ = false;
    double zPosition_ = 0.0;
    double zPositionLower_ = 0.0;
    double zPositionUpper_ = 0.0;
};

} // namespace tether::klipper::klippy
