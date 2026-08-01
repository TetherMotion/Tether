#pragma once

/// @file PrinterObjectsCore.hpp
/// @brief Core printer objects (extruder, heater_bed, fan, toolhead, etc.)

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

/// @brief The extruder printer object.
/// Exposes hotend temperature, target, power, and pressure advance.
class ExtruderObject : public PrinterObject {
public:
    explicit ExtruderObject(std::shared_ptr<objects::Heater> heater)
        : heater_(std::move(heater)) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (heater_) {
            s.add("temperature", JsonValue(heater_->currentTemp()));
            s.add("target", JsonValue(heater_->target()));
            double power = 0.0;
            auto ps = heater_->pidState();
            power = std::clamp(ps.output, 0.0, 1.0);
            s.add("power", JsonValue(power));
        }
        s.add("pressure_advance", JsonValue(pressureAdvance_));
        s.add("can_extrude", JsonValue(heater_ && heater_->currentTemp() > minExtrudeTemp_));
        s.add("motion_queue", JsonValue(motionQueue_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "target", "power", "pressure_advance",
                "can_extrude", "motion_queue"};
    }

    void setName(const std::string& n) { name_ = n; }
    void setPressureAdvance(double pa) { pressureAdvance_ = pa; }
    void setMinExtrudeTemp(double t) { minExtrudeTemp_ = t; }
    void setMotionQueue(const std::string& q) { motionQueue_ = q; }
    double pressureAdvance() const { return pressureAdvance_; }
    double minExtrudeTemp() const { return minExtrudeTemp_; }
    const std::string& motionQueue() const { return motionQueue_; }

private:
    std::shared_ptr<objects::Heater> heater_;
    std::string name_ = "extruder";
    double pressureAdvance_ = 0.0;
    double minExtrudeTemp_ = 170.0;
    std::string motionQueue_;
};

/// @brief The heater_bed printer object.
class HeaterBedObject : public PrinterObject {
public:
    explicit HeaterBedObject(std::shared_ptr<objects::Heater> heater)
        : heater_(std::move(heater)) {}

    std::string name() const override { return "heater_bed"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (heater_) {
            s.add("temperature", JsonValue(heater_->currentTemp()));
            s.add("target", JsonValue(heater_->target()));
            auto ps = heater_->pidState();
            s.add("power", JsonValue(std::clamp(ps.output, 0.0, 1.0)));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"temperature", "target", "power"};
    }

private:
    std::shared_ptr<objects::Heater> heater_;
};

/// @brief The fan printer object (for part cooling fan).
class FanObject : public PrinterObject {
public:
    explicit FanObject(std::shared_ptr<objects::Fan> fan)
        : fan_(std::move(fan)) {}

    std::string name() const override { return "fan"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (fan_) {
            s.add("speed", JsonValue(fan_->speed()));
        } else {
            s.add("speed", JsonValue(0.0));
        }
        s.add("rpm", JsonValue(rpm_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"speed", "rpm"};
    }

    void setRpm(double rpm) { rpm_ = rpm; }

private:
    std::shared_ptr<objects::Fan> fan_;
    double rpm_ = 0.0;
};

/// @brief The heaters aggregate printer object.
/// Lists all available heaters and sensors.
class HeatersObject : public PrinterObject {
public:
    void addHeater(const std::string& name) { heaters_.push_back(name); }
    void addSensor(const std::string& name) { sensors_.push_back(name); }

    std::string name() const override { return "heaters"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::vector<JsonValue> h;
        for (const auto& n : heaters_) h.emplace_back(n);
        std::vector<JsonValue> sn;
        for (const auto& n : sensors_) sn.emplace_back(n);
        s.add("available_heaters", JsonValue(h));
        s.add("available_sensors", JsonValue(sn));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"available_heaters", "available_sensors"};
    }

private:
    std::vector<std::string> heaters_;
    std::vector<std::string> sensors_;
};

/// @brief The mcu printer object.
/// Exposes MCU version, clock frequency, and statistics.
class McuObject : public PrinterObject {
public:
    std::string name() const override { return "mcu"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("mcu_version", JsonValue(mcuVersion_));
        s.add("mcu_build_version", JsonValue(mcuBuildVersion_));
        s.add("mcu_constants", JsonValue(std::map<std::string, JsonValue>{
            {"FREQ", JsonValue(static_cast<int64_t>(freq_))},
            {"SERIAL", JsonValue(serial_)}
        }));
        s.add("last_stats", JsonValue(std::map<std::string, JsonValue>{
            {"mcu_awake", JsonValue(mcuAwake_)},
            {"bytes_read", JsonValue(static_cast<int64_t>(bytesRead_))},
            {"bytes_write", JsonValue(static_cast<int64_t>(bytesWrite_))},
            {"retransmits", JsonValue(static_cast<int64_t>(retransmits_))},
        }));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"mcu_version", "mcu_build_version", "mcu_constants", "last_stats"};
    }

    void setMcuVersion(const std::string& v) { mcuVersion_ = v; }
    void setMcuBuildVersion(const std::string& v) { mcuBuildVersion_ = v; }
    void setFreq(uint32_t f) { freq_ = f; }
    void setSerial(const std::string& s) { serial_ = s; }
    void setStats(double awake, size_t read, size_t write, size_t retrans) {
        mcuAwake_ = awake; bytesRead_ = read; bytesWrite_ = write; retransmits_ = retrans;
    }

private:
    std::string mcuVersion_ = "tether-mcu-1.0.0";
    std::string mcuBuildVersion_ = "tether-mcu-1.0.0";
    uint32_t freq_ = 48000000;
    std::string serial_ = "";
    double mcuAwake_ = 0.001;
    size_t bytesRead_ = 0;
    size_t bytesWrite_ = 0;
    size_t retransmits_ = 0;
};

/// @brief The stepper_enable printer object.
class StepperEnableObject : public PrinterObject {
public:
    void setStepperEnabled(const std::string& name, bool enabled) {
        steppers_[name] = enabled;
    }

    std::string name() const override { return "stepper_enable"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::vector<JsonValue> stepperList;
        for (const auto& [name, enabled] : steppers_) {
            stepperList.emplace_back(name);
        }
        s.add("steppers", JsonValue(stepperList));
        s.add("enabled", JsonValue(enabled_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"steppers", "enabled"};
    }

    void setEnabled(bool e) { enabled_ = e; }

private:
    std::map<std::string, bool> steppers_;
    bool enabled_ = true;
};

/// @brief The idle_timeout printer object.
class IdleTimeoutObject : public PrinterObject {
public:
    std::string name() const override { return "idle_timeout"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("state", JsonValue(state_));
        s.add("printing_time", JsonValue(printingTime_));
        s.add("timeout", JsonValue(timeout_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"state", "printing_time", "timeout"};
    }

    void setState(const std::string& s) { state_ = s; }
    void setPrintingTime(double t) { printingTime_ = t; }
    void setTimeout(double t) { timeout_ = t; }

private:
    std::string state_ = "Idle";
    double printingTime_ = 0.0;
    double timeout_ = 600.0; // 10 minutes default
};

/// @brief The system_stats printer object.
class SystemStatsObject : public PrinterObject {
public:
    std::string name() const override { return "system_stats"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        s.add("sysload", JsonValue(sysload_));
        s.add("cputime", JsonValue(cputime_));
        s.add("memavail", JsonValue(static_cast<int64_t>(memavail_)));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"sysload", "cputime", "memavail"};
    }

    void setSysload(double load) { sysload_ = load; }
    void setCputime(double t) { cputime_ = t; }
    void setMemavail(size_t m) { memavail_ = m; }

private:
    double sysload_ = 0.0;
    double cputime_ = 0.0;
    size_t memavail_ = 0;
};

/// @brief The print_stats printer object.
/// Tracks print job statistics (filename, duration, filament used, state).
class PrintStatsObject : public PrinterObject {
public:
    explicit PrintStatsObject(std::shared_ptr<VirtualSdcard> sd)
        : sd_(std::move(sd)) {}

    std::string name() const override { return "print_stats"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (sd_) {
            s.add("filename", JsonValue(sd_->filePath()));
            s.add("progress", JsonValue(sd_->progress()));
        } else {
            s.add("filename", JsonValue(filename_));
            s.add("progress", JsonValue(progress_));
        }
        s.add("total_duration", JsonValue(totalDuration_));
        s.add("print_duration", JsonValue(printDuration_));
        s.add("filament_used", JsonValue(filamentUsed_));
        s.add("state", JsonValue(state_));
        s.add("message", JsonValue(message_));
        // info sub-object (SET_PRINT_STATS_INFO)
        std::map<std::string, JsonValue> info;
        info["total_layer"] = JsonValue(infoTotalLayer_);
        info["current_layer"] = JsonValue(infoCurrentLayer_);
        s.add("info", JsonValue(info));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"filename", "total_duration", "print_duration",
                "filament_used", "state", "message", "progress", "info"};
    }

    void setFilename(const std::string& f) { filename_ = f; }
    void setTotalDuration(double d) { totalDuration_ = d; }
    void setPrintDuration(double d) { printDuration_ = d; }
    void setFilamentUsed(double f) { filamentUsed_ = f; }
    void setState(const std::string& s) { state_ = s; }
    void setMessage(const std::string& m) { message_ = m; }
    void setProgress(double p) { progress_ = p; }
    void setInfoTotalLayer(int64_t l) { infoTotalLayer_ = l; }
    void setInfoCurrentLayer(int64_t l) { infoCurrentLayer_ = l; }

private:
    std::shared_ptr<VirtualSdcard> sd_;
    std::string filename_;
    double totalDuration_ = 0.0;
    double printDuration_ = 0.0;
    double filamentUsed_ = 0.0;
    std::string state_ = "standby"; // standby, printing, paused, complete, error, cancelled
    std::string message_;
    double progress_ = 0.0;
    int64_t infoTotalLayer_ = 0;
    int64_t infoCurrentLayer_ = 0;
};

} // namespace tether::klipper::klippy
