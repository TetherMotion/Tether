/**
 * @file KlippyUdsServerMachineEndpoints.cpp
 * @brief Machine control endpoints and helper methods
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

JsonValue KlippyServer::handleMachineReboot(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    // Initiate shutdown of the printer state machine
    setState(PrinterState::Shutdown, "Machine reboot requested");
    // Call the restart handler if set
    if (restartHandler_) {
        restartHandler_();
    }
    result["result"] = JsonValue("reboot requested");
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineShutdown(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    // Initiate shutdown of the printer state machine
    setState(PrinterState::Shutdown, "Machine shutdown requested");
    // Call the emergency stop handler if set
    if (emergencyStopHandler_) {
        emergencyStopHandler_();
    }
    result["result"] = JsonValue("shutdown requested");
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineUpdateStatus(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> status;
    for (const auto& [component, state] : updateStatus_) {
        status[component] = JsonValue(state);
    }
    result["result"] = JsonValue(status);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineDevicePowerDevices(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> devices;
    for (const auto& [name, dev] : powerDevices_) {
        std::map<std::string, JsonValue> deviceInfo;
        deviceInfo["device"] = JsonValue(name);
        deviceInfo["state"] = JsonValue(dev.state);
        deviceInfo["locked"] = JsonValue(dev.locked);
        devices.push_back(JsonValue(deviceInfo));
    }
    result["result"] = JsonValue(devices);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineDevicePowerState(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    std::string action = params.has("action") && params.find("action")->isString()
        ? params.find("action")->asString() : "";

    if (!device.empty()) {
        auto it = powerDevices_.find(device);
        if (it != powerDevices_.end()) {
            if (action == "on" || action == "off") {
                // Use setPowerDeviceState to properly invoke callbacks
                setPowerDeviceState(device, action);
                result["result"] = JsonValue(action);
            } else {
                result["result"] = JsonValue(it->second.state);
            }
        } else {
            result["error"] = JsonValue("Device not found");
        }
    } else {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        // Return all device states
        std::map<std::string, JsonValue> states;
        for (const auto& [name, dev] : powerDevices_) {
            states[name] = JsonValue(dev.state);
        }
        result["result"] = JsonValue(states);
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handlePrinterInfo(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;
    info["state"] = JsonValue(stateToString(state_));
    info["state_message"] = JsonValue(stateMessage_);
    info["hostname"] = JsonValue("tether-klipper");
    info["software_version"] = JsonValue(config_.softwareVersion);
    info["klipper_path"] = JsonValue(config_.klipperPath);
    info["python_path"] = JsonValue(config_.pythonPath);
    info["log_file"] = JsonValue(config_.logFile);
    info["config_file"] = JsonValue(config_.configFile);
    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyServer::handlePrinterSubscriptions(const JsonValue& params) {
    // Subscriptions are managed by transport layers (UDS, HTTP/WS).
    // This endpoint returns an empty list at the business-logic level.
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(std::vector<JsonValue>{});
    return JsonValue(result);
}

// ============================================================================
// (recordTemperature, recordGcodeResponse, registerPowerDevice,
//  setPowerDeviceState, setUpdateStatus are implemented in KlippyServerCore.cpp)

} // namespace tether::klipper::klippy
