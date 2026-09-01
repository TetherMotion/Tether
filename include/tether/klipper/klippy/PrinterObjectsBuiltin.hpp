#pragma once

/// @file PrinterObjectsBuiltin.hpp
/// @brief Built-in printer objects that reference KlippyServer directly.
///
/// These objects (webhooks, gcode_move, toolhead, configfile, pause_resume,
/// virtual_sdcard, display_status) are defined here because they need access
/// to KlippyServer's state.

#include "tether/klipper/klippy/KlippyServer.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// Built-in printer objects
// ============================================================================

/// @brief The webhooks printer object (exposes state and state_message).
class WebhooksObject : public PrinterObject {
public:
    explicit WebhooksObject(KlippyServer& server) : server_(server) {}

    std::string name() const override { return "webhooks"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"state", "state_message"};
    }

private:
    KlippyServer& server_;
};

/// @brief The gcode_move printer object.
class GcodeMoveObject : public PrinterObject {
public:
    std::string name() const override { return "gcode_move"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"absolute_coordinates", "absolute_extrude", "extrude_factor",
                "speed_factor", "speed", "gcode_position", "position",
                "homing_origin", "scale"};
    }

    // Setters for updating state
    void setAbsoluteCoordinates(bool abs) { absoluteCoords_ = abs; }
    void setAbsoluteExtrude(bool abs) { absoluteExtrude_ = abs; }
    void setSpeed(double speed) { speed_ = speed; }
    void setSpeedFactor(double factor) { speedFactor_ = factor; }
    void setExtrudeFactor(double factor) { extrudeFactor_ = factor; }
    void setPosition(const std::array<double, 4>& pos) { position_ = pos; }
    void setGcodePosition(const std::array<double, 4>& pos) { gcodePos_ = pos; }

private:
    bool absoluteCoords_ = true;
    bool absoluteExtrude_ = true;
    double speed_ = 1500.0;
    double speedFactor_ = 1.0;
    double extrudeFactor_ = 1.0;
    std::array<double, 4> position_ = {0, 0, 0, 0};
    std::array<double, 4> gcodePos_ = {0, 0, 0, 0};
};

/// @brief The toolhead printer object.
class ToolheadObject : public PrinterObject {
public:
    std::string name() const override { return "toolhead"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"position", "status", "max_velocity", "max_accel",
                "max_accel_to_decel", "homed_axes", "print_time",
                "estimated_print_time", "extruder", "stalls"};
    }

    void setPosition(const std::array<double, 4>& pos) { position_ = pos; }
    std::array<double, 4> position() const { return position_; }
    void setStatus(const std::string& s) { status_ = s; }
    void setHomedAxes(const std::string& axes) { homedAxes_ = axes; }
    void setExtruder(const std::string& e) { extruder_ = e; }
    void setMaxVelocity(double v) { maxVelocity_ = v; }
    void setMaxAccel(double a) { maxAccel_ = a; }
    void setMaxAccelToDecel(double a) { maxAccelToDecel_ = a; }
    void setPrintTime(double t) { printTime_ = t; }
    void setEstimatedPrintTime(double t) { estimatedPrintTime_ = t; }
    void setStalls(int s) { stalls_ = s; }

private:
    std::array<double, 4> position_ = {0, 0, 0, 0};
    std::string status_ = "Idle";
    std::string homedAxes_ = "";
    std::string extruder_ = "extruder";
    double maxVelocity_ = 500.0;
    double maxAccel_ = 3000.0;
    double maxAccelToDecel_ = 1500.0;
    double printTime_ = 0.0;
    double estimatedPrintTime_ = 0.0;
    int stalls_ = 0;
};

/// @brief The configfile printer object.
class ConfigfileObject : public PrinterObject {
public:
    std::string name() const override { return "configfile"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"path", "save_config_pending", "save_config_pending_items",
                "settings", "config"};
    }

    void setPath(const std::string& p) { path_ = p; }
    void setSaveConfigPending(bool pending) { saveConfigPending_ = pending; }
    void setSaveConfigPendingItems(const std::map<std::string, std::string>& items) {
        saveConfigPendingItems_ = items;
    }

    /// Set the parsed config settings (section name -> key -> value).
    /// Keys should be lowercase as Mainsail expects.
    void setSettings(std::map<std::string, JsonValue> s) { settings_ = std::move(s); }
    void setConfig(std::map<std::string, JsonValue> c) { config_ = std::move(c); }

private:
    std::string path_ = "/etc/tether/printer.cfg";
    bool saveConfigPending_ = false;
    std::map<std::string, std::string> saveConfigPendingItems_;
    // Parsed config: section name (lowercase) -> { key -> value }
    std::map<std::string, JsonValue> settings_;
    std::map<std::string, JsonValue> config_;
};

/// @brief The pause_resume printer object.
class PauseResumeObject : public PrinterObject {
public:
    std::string name() const override { return "pause_resume"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"is_paused"};
    }

    void setPaused(bool p) { isPaused_ = p; }

private:
    bool isPaused_ = false;
};

/// @brief The virtual_sdcard printer object.
///
/// Wraps a VirtualSdcard instance and exposes its status via UDS.
/// If no VirtualSdcard is provided, falls back to manual setters.
class VirtualSdcardObject : public PrinterObject {
public:
    /// @brief Construct with a VirtualSdcard backend.
    explicit VirtualSdcardObject(std::shared_ptr<class VirtualSdcard> sd)
        : sd_(std::move(sd)) {}

    /// @brief Construct as a standalone stub (manual setters).
    VirtualSdcardObject() = default;

    std::string name() const override { return "virtual_sdcard"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"progress", "is_active", "file_position", "file_path",
                "file_size"};
    }

    // Manual setters (for stub mode when no VirtualSdcard is wired)
    void setProgress(double p) { progress_ = p; }
    void setActive(bool a) { isActive_ = a; }
    void setFilePath(const std::string& p) { filePath_ = p; }
    void setFileSize(size_t s) { fileSize_ = s; }
    void setFilePosition(size_t p) { filePosition_ = p; }

private:
    std::shared_ptr<class VirtualSdcard> sd_;
    // Fallback state (used when sd_ is null)
    double progress_ = 0.0;
    bool isActive_ = false;
    std::string filePath_;
    size_t fileSize_ = 0;
    size_t filePosition_ = 0;
};

/// @brief The display_status printer object.
class DisplayStatusObject : public PrinterObject {
public:
    std::string name() const override { return "display_status"; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override;

    std::vector<std::string> availableFields() const override {
        return {"progress", "message"};
    }

    void setProgress(double p) { progress_ = p; }
    void setMessage(const std::string& m) { message_ = m; }

private:
    double progress_ = 0.0;
    std::string message_;
};

} // namespace tether::klipper::klippy
