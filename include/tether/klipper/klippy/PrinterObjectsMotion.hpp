#pragma once

/// @file PrinterObjectsMotion.hpp
/// @brief Motion printer objects (skew_correction, input_shaper, pressure_advance, etc.)

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

/// @brief Skew correction printer object (skew_correction).
class SkewCorrectionObject : public PrinterObject {
public:
    std::string name() const override { return "skew_correction"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("xy_skew", JsonValue(xySkew_));
        s.add("xz_skew", JsonValue(xzSkew_));
        s.add("yz_skew", JsonValue(yzSkew_));
        s.add("current_profile", JsonValue(currentProfile_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"xy_skew", "xz_skew", "yz_skew", "current_profile"};
    }

    void setSkew(double xy, double xz, double yz) {
        xySkew_ = xy; xzSkew_ = xz; yzSkew_ = yz;
    }
    void setCurrentProfile(const std::string& p) { currentProfile_ = p; }

private:
    double xySkew_ = 0.0;
    double xzSkew_ = 0.0;
    double yzSkew_ = 0.0;
    std::string currentProfile_ = "";
};

/// @brief Input shaper printer object (input_shaper).
class InputShaperObject : public PrinterObject {
public:
    std::string name() const override { return "input_shaper"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("shaper_freq_x", JsonValue(shaperFreqX_));
        s.add("shaper_type_x", JsonValue(shaperTypeX_));
        s.add("shaper_freq_y", JsonValue(shaperFreqY_));
        s.add("shaper_type_y", JsonValue(shaperTypeY_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"shaper_freq_x", "shaper_type_x", "shaper_freq_y", "shaper_type_y"};
    }

    void setShaperFreqX(double f) { shaperFreqX_ = f; }
    void setShaperTypeX(const std::string& t) { shaperTypeX_ = t; }
    void setShaperFreqY(double f) { shaperFreqY_ = f; }
    void setShaperTypeY(const std::string& t) { shaperTypeY_ = t; }

private:
    double shaperFreqX_ = 0.0;
    std::string shaperTypeX_ = "ei";
    double shaperFreqY_ = 0.0;
    std::string shaperTypeY_ = "ei";
};

#if TETHER_ENABLE_PRESSURE_ADVANCE
/// @brief Pressure advance printer object (pressure_advance).
class PressureAdvanceObject : public PrinterObject {
public:
    std::string name() const override { return "pressure_advance"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("pressure_advance", JsonValue(pressureAdvance_));
        s.add("smooth_time", JsonValue(smoothTime_));
        s.add("model", JsonValue(model_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"pressure_advance", "smooth_time", "model"};
    }

    void setPressureAdvance(double pa) { pressureAdvance_ = pa; }
    void setSmoothTime(double t) { smoothTime_ = t; }
    void setModel(const std::string& m) { model_ = m; }

private:
    double pressureAdvance_ = 0.0;
    double smoothTime_ = 0.040;
    std::string model_ = "linear";
};
#endif // TETHER_ENABLE_PRESSURE_ADVANCE

/// @brief Force move printer object (force_move).
class ForceMoveObject : public PrinterObject {
public:
    std::string name() const override { return "force_move"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("enable_force_move", JsonValue(enableForceMove_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"enable_force_move"};
    }

    void setEnableForceMove(bool e) { enableForceMove_ = e; }

private:
    bool enableForceMove_ = false;
};

/// @brief Dual carriage printer object (dual_carriage).
class DualCarriageObject : public PrinterObject {
public:
    std::string name() const override { return "dual_carriage"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("carriage_0", JsonValue(carriage0_));
        s.add("carriage_1", JsonValue(carriage1_));
        s.add("mode", JsonValue(mode_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"carriage_0", "carriage_1", "mode"};
    }

    void setCarriage0(const std::string& s) { carriage0_ = s; }
    void setCarriage1(const std::string& s) { carriage1_ = s; }
    void setMode(const std::string& m) { mode_ = m; }

private:
    std::string carriage0_ = "PRIMARY";
    std::string carriage1_ = "INACTIVE";
    std::string mode_ = "FULL";
};

/// @brief Extruder stepper printer object (extruder_stepper).
class ExtruderStepperObject : public PrinterObject {
public:
    explicit ExtruderStepperObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("pressure_advance", JsonValue(pressureAdvance_));
        s.add("smooth_time", JsonValue(smoothTime_));
        s.add("motion_queue", JsonValue(motionQueue_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"pressure_advance", "smooth_time", "motion_queue"};
    }

    void setPressureAdvance(double pa) { pressureAdvance_ = pa; }
    void setSmoothTime(double t) { smoothTime_ = t; }
    void setMotionQueue(const std::string& q) { motionQueue_ = q; }

private:
    std::string name_;
    double pressureAdvance_ = 0.0;
    double smoothTime_ = 0.040;
    std::string motionQueue_;
};

/// @brief Manual stepper printer object (manual_stepper).
class ManualStepperObject : public PrinterObject {
public:
    explicit ManualStepperObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("position", JsonValue(position_));
        s.add("velocity", JsonValue(velocity_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"position", "velocity"}; }

    void setPosition(double p) { position_ = p; }
    void setVelocity(double v) { velocity_ = v; }

private:
    std::string name_;
    double position_ = 0.0;
    double velocity_ = 0.0;
};

/// @brief Endstop phase printer object (endstop_phase).
class EndstopPhaseObject : public PrinterObject {
public:
    explicit EndstopPhaseObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("phase", JsonValue(static_cast<int64_t>(phase_)));
        s.add("phases", JsonValue(static_cast<int64_t>(phases_)));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"phase", "phases"}; }

    void setPhase(int p) { phase_ = p; }
    void setPhases(int p) { phases_ = p; }

private:
    std::string name_;
    int phase_ = 0;
    int phases_ = 0;
};

} // namespace tether::klipper::klippy
