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
        auto s = buildStatus(fields);
        if (probe_) {
            s.add("last_query", JsonValue(probe_->triggered()));
        } else {
            s.add("last_query", JsonValue(lastQuery_));
        }
        s.add("last_z_result", JsonValue(lastZResult_));
        s.add("z_offset", JsonValue(probe_ ? probe_->zOffset() : zOffset_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("last_state", JsonValue(lastState_));
        s.add("last_z_value", JsonValue(lastZValue_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("applied", JsonValue(applied_));
        s.add("z_positions", JsonValue(zPositions_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("applied", JsonValue(applied_));
        s.add("z_values", JsonValue(zValues_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("error", JsonValue(error_));
        s.add("max_base", JsonValue(maxBase_));
        s.add("base", JsonValue(base_));
        s.add("adjusted", JsonValue(adjusted_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("state", JsonValue(state_));
        s.add("current_screw", JsonValue(currentScrew_));
        s.add("accepted_screws", JsonValue(acceptedScrews_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("radius", JsonValue(radius_));
        s.add("positions", JsonValue(positions_));
        s.add("deltas", JsonValue(deltas_));
        s.add("applied", JsonValue(applied_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("home_xy_position", JsonValue(homeXyPosition_));
        s.add("z_hop", JsonValue(zHop_));
        s.add("z_hop_speed", JsonValue(zHopSpeed_));
        s.add("xy_home_speed", JsonValue(xyHomeSpeed_));
        s.add("move_to_previous", JsonValue(moveToPrevious_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("x_adjust", JsonValue(xAdjust_));
        s.add("y_adjust", JsonValue(yAdjust_));
        s.add("applied", JsonValue(applied_));
        return std::move(s).take();
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
        auto s = buildStatus(fields);
        s.add("is_active", JsonValue(isActive_));
        s.add("z_position", JsonValue(zPosition_));
        s.add("z_position_lower", JsonValue(zPositionLower_));
        s.add("z_position_upper", JsonValue(zPositionUpper_));
        return std::move(s).take();
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
