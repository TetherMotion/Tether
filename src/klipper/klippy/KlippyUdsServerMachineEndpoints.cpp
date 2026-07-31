/**
 * @file KlippyUdsServerMachineEndpoints.cpp
 * @brief Machine control endpoints and helper methods
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "UdsConnection_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <set>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace tether::klipper::klippy {

JsonValue KlippyUdsServer::handleMachineReboot(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleMachineShutdown(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleMachineUpdateStatus(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> status;
    for (const auto& [component, state] : updateStatus_) {
        status[component] = JsonValue(state);
    }
    result["result"] = JsonValue(status);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineDevicePowerDevices(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleMachineDevicePowerState(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    std::string action = params.has("action") && params.find("action")->isString()
        ? params.find("action")->asString() : "";

    if (!device.empty()) {
        auto it = powerDevices_.find(device);
        if (it != powerDevices_.end()) {
            if (action == "on" || action == "off") {
                it->second.state = action;
            }
            result["result"] = JsonValue(it->second.state);
        } else {
            result["error"] = JsonValue("Device not found");
        }
    } else {
        // Return all device states
        std::map<std::string, JsonValue> states;
        for (const auto& [name, dev] : powerDevices_) {
            states[name] = JsonValue(dev.state);
        }
        result["result"] = JsonValue(states);
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handlePrinterInfo(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handlePrinterSubscriptions(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> subs;

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& sub : subscriptions_) {
        std::map<std::string, JsonValue> subInfo;
        std::vector<JsonValue> objects;
        for (const auto& [name, fields] : sub.objects) {
            std::map<std::string, JsonValue> objInfo;
            objInfo["name"] = JsonValue(name);
            std::vector<JsonValue> fieldArr;
            for (const auto& f : fields) fieldArr.push_back(JsonValue(f));
            objInfo["fields"] = JsonValue(fieldArr);
            objects.push_back(JsonValue(objInfo));
        }
        subInfo["objects"] = JsonValue(objects);
        subs.push_back(JsonValue(subInfo));
    }
    result["result"] = JsonValue(subs);
    return JsonValue(result);
}

// ============================================================================
// Temperature store, gcode store, power device helpers
// ============================================================================

void KlippyUdsServer::recordTemperature(const std::string& heater,
                                         double temp, double target) {
    auto& entries = tempStore_[heater];
    entries.push_back({temp, target, std::chrono::steady_clock::now()});
    if (entries.size() > kMaxTempStoreEntries) {
        entries.erase(entries.begin());
    }
}

void KlippyUdsServer::recordGcodeResponse(const std::string& msg) {
    gcodeStore_.push_back({msg, std::chrono::steady_clock::now()});
    if (gcodeStore_.size() > kMaxGcodeStoreEntries) {
        gcodeStore_.erase(gcodeStore_.begin());
    }
}

void KlippyUdsServer::registerPowerDevice(const std::string& name,
                                            const std::string& initialState) {
    powerDevices_[name] = {name, initialState, false};
}

bool KlippyUdsServer::setPowerDeviceState(const std::string& name,
                                           const std::string& state) {
    auto it = powerDevices_.find(name);
    if (it == powerDevices_.end()) return false;
    it->second.state = state;
    return true;
}

void KlippyUdsServer::setUpdateStatus(const std::string& component,
                                       const std::string& status) {
    updateStatus_[component] = status;
}

} // namespace tether::klipper::klippy
