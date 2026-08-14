/**
 * @file KlippyUdsServerPrinterObjects.cpp
 * @brief Built-in printer object status implementations
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <sys/stat.h>

namespace tether::klipper::klippy {

// ============================================================================
// Built-in printer objects
// ============================================================================

std::map<std::string, JsonValue> WebhooksObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    bool wantState = fields.empty() ||
        std::find(fields.begin(), fields.end(), "state") != fields.end();
    bool wantMsg = fields.empty() ||
        std::find(fields.begin(), fields.end(), "state_message") != fields.end();
    if (wantState) result["state"] = JsonValue(stateToString(server_.state()));
    if (wantMsg) result["state_message"] = JsonValue(server_.stateMessage());
    return result;
}

std::map<std::string, JsonValue> GcodeMoveObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    auto addField = [&](const std::string& name, JsonValue val) {
        if (fields.empty() || std::find(fields.begin(), fields.end(), name) != fields.end())
            result[name] = std::move(val);
    };
    addField("absolute_coordinates", JsonValue(absoluteCoords_));
    addField("absolute_extrude", JsonValue(absoluteExtrude_));
    addField("extrude_factor", JsonValue(extrudeFactor_));
    addField("speed_factor", JsonValue(speedFactor_));
    addField("speed", JsonValue(speed_));
    std::vector<JsonValue> posArr;
    for (double v : position_) posArr.emplace_back(v);
    addField("position", JsonValue(posArr));
    std::vector<JsonValue> gcodeArr;
    for (double v : gcodePos_) gcodeArr.emplace_back(v);
    addField("gcode_position", JsonValue(gcodeArr));
    // homing_origin: the homing offsets (X, Y, Z, 0)
    std::vector<JsonValue> homingArr = {JsonValue(0.0), JsonValue(0.0), JsonValue(0.0), JsonValue(0.0)};
    addField("homing_origin", JsonValue(homingArr));
    // scale: axis scaling factors (default 1.0)
    std::vector<JsonValue> scaleArr = {JsonValue(1.0), JsonValue(1.0), JsonValue(1.0), JsonValue(1.0)};
    addField("scale", JsonValue(scaleArr));
    return result;
}

std::map<std::string, JsonValue> ToolheadObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    auto addField = [&](const std::string& name, JsonValue val) {
        if (fields.empty() || std::find(fields.begin(), fields.end(), name) != fields.end())
            result[name] = std::move(val);
    };
    std::vector<JsonValue> posArr;
    for (double v : position_) posArr.emplace_back(v);
    addField("position", JsonValue(posArr));
    addField("status", JsonValue(status_));
    addField("max_velocity", JsonValue(maxVelocity_));
    addField("max_accel", JsonValue(maxAccel_));
    addField("max_accel_to_decel", JsonValue(maxAccelToDecel_));
    addField("homed_axes", JsonValue(homedAxes_));
    addField("print_time", JsonValue(printTime_));
    addField("estimated_print_time", JsonValue(estimatedPrintTime_));
    addField("extruder", JsonValue(extruder_));
    addField("stalls", JsonValue(static_cast<int64_t>(stalls_)));
    return result;
}

std::map<std::string, JsonValue> ConfigfileObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    auto addField = [&](const std::string& name, JsonValue val) {
        if (fields.empty() || std::find(fields.begin(), fields.end(), name) != fields.end())
            result[name] = std::move(val);
    };
    addField("path", JsonValue(path_));
    addField("save_config_pending", JsonValue(saveConfigPending_));
    // save_config_pending_items: map of section -> pending config changes
    std::map<std::string, JsonValue> pendingItems;
    for (const auto& [section, value] : saveConfigPendingItems_) {
        pendingItems[section] = JsonValue(value);
    }
    addField("save_config_pending_items", JsonValue(pendingItems));
    return result;
}

std::map<std::string, JsonValue> PauseResumeObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    if (fields.empty() || std::find(fields.begin(), fields.end(), "is_paused") != fields.end())
        result["is_paused"] = JsonValue(isPaused_);
    return result;
}

std::map<std::string, JsonValue> VirtualSdcardObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    auto addField = [&](const std::string& name, JsonValue val) {
        if (fields.empty() || std::find(fields.begin(), fields.end(), name) != fields.end())
            result[name] = std::move(val);
    };
    if (sd_) {
        // Delegate to the real VirtualSdcard
        addField("progress", JsonValue(sd_->progress()));
        addField("is_active", JsonValue(sd_->isActive()));
        addField("file_path", JsonValue(sd_->filePath()));
        addField("file_size", JsonValue(static_cast<int64_t>(sd_->fileSize())));
        addField("file_position", JsonValue(static_cast<int64_t>(sd_->filePosition())));
    } else {
        // Fallback to manual state
        addField("progress", JsonValue(progress_));
        addField("is_active", JsonValue(isActive_));
        addField("file_path", JsonValue(filePath_));
        addField("file_size", JsonValue(static_cast<int64_t>(fileSize_)));
        addField("file_position", JsonValue(static_cast<int64_t>(filePosition_)));
    }
    return result;
}

std::map<std::string, JsonValue> DisplayStatusObject::status(
    const std::vector<std::string>& fields) const {
    std::map<std::string, JsonValue> result;
    auto addField = [&](const std::string& name, JsonValue val) {
        if (fields.empty() || std::find(fields.begin(), fields.end(), name) != fields.end())
            result[name] = std::move(val);
    };
    addField("progress", JsonValue(progress_));
    addField("message", JsonValue(message_));
    return result;
}

} // namespace tether::klipper::klippy
