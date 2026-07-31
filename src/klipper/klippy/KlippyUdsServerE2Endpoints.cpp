/**
 * @file KlippyUdsServerE2Endpoints.cpp
 * @brief E2 additional Moonraker-compatible endpoint handlers.
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

// ============================================================================
// E2: server/restart — Restart the Moonraker server process
// ============================================================================

JsonValue KlippyUdsServer::handleServerRestart(const JsonValue& params) {
    (void)params;
    // Trigger restart via the restart handler if available
    if (restartHandler_) {
        restartHandler_();
    }
    setState(PrinterState::Startup, "Server restarting");
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    result["message"] = JsonValue("Server restart initiated");
    return JsonValue(result);
}

// ============================================================================
// E2: printer/query_endstops/status — Query endstop states
// ============================================================================

JsonValue KlippyUdsServer::handleQueryEndstopsStatus(const JsonValue& params) {
    (void)params;
    // Query all registered endstop objects
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;

    // Look for query_endstops or endstop objects
    auto it = objects_.find("query_endstops");
    if (it != objects_.end()) {
        auto statusMap = it->second->status({});
        for (const auto& [key, val] : statusMap) {
            result[key] = val;
        }
    } else {
        // No query_endstops object — return empty status
        result["last_queried"] = JsonValue(false);
    }

    std::map<std::string, JsonValue> response;
    response["result"] = JsonValue(result);
    return JsonValue(response);
}

// ============================================================================
// E2: machine/peripherals/usb — List USB devices
// ============================================================================

JsonValue KlippyUdsServer::handleMachinePeripheralsUsb(const JsonValue& params) {
    (void)params;
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> usbDevices;

    // Scan /sys/bus/usb/devices for USB device information
    namespace fs = std::filesystem;
    if (fs::exists("/sys/bus/usb/devices")) {
        for (const auto& entry : fs::directory_iterator("/sys/bus/usb/devices")) {
            auto name = entry.path().filename().string();
            // Only process entries that look like bus-port.subport (contain a ':')
            if (name.find(':') != std::string::npos) continue;
            if (name.empty() || name[0] == '.') continue;

            std::map<std::string, JsonValue> device;
            device["bus"] = JsonValue(name);

            // Read vendor ID
            std::ifstream vendorFile(entry.path() / "idVendor");
            std::string vendorId;
            if (vendorFile) { std::getline(vendorFile, vendorId); }
            if (!vendorId.empty()) device["vendor_id"] = JsonValue(vendorId);

            // Read product ID
            std::ifstream productFile(entry.path() / "idProduct");
            std::string productId;
            if (productFile) { std::getline(productFile, productId); }
            if (!productId.empty()) device["product_id"] = JsonValue(productId);

            // Read product name
            std::ifstream prodNameFile(entry.path() / "product");
            std::string prodName;
            if (prodNameFile) { std::getline(prodNameFile, prodName); }
            if (!prodName.empty()) device["product"] = JsonValue(prodName);

            // Read manufacturer
            std::ifstream manufFile(entry.path() / "manufacturer");
            std::string manuf;
            if (manufFile) { std::getline(manufFile, manuf); }
            if (!manuf.empty()) device["manufacturer"] = JsonValue(manuf);

            if (!vendorId.empty() || !productId.empty()) {
                usbDevices.push_back(JsonValue(device));
            }
        }
    }

    result["usb_devices"] = JsonValue(usbDevices);
    result["count"] = JsonValue(static_cast<int>(usbDevices.size()));

    std::map<std::string, JsonValue> response;
    response["result"] = JsonValue(result);
    return JsonValue(response);
}

// ============================================================================
// E2: machine/peripherals/serial — List serial devices
// ============================================================================

JsonValue KlippyUdsServer::handleMachinePeripheralsSerial(const JsonValue& params) {
    (void)params;
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> serialDevices;

    // Scan /sys/class/tty for serial devices
    namespace fs = std::filesystem;
    if (fs::exists("/sys/class/tty")) {
        for (const auto& entry : fs::directory_iterator("/sys/class/tty")) {
            auto name = entry.path().filename().string();
            if (name.empty()) continue;

            // Check if it's a real serial device (has device symlink or is a ttyUSB/ttyACM)
            bool isSerial = (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0 ||
                             name.find("ttyS") == 0 || name.find("ttyAMA") == 0);

            if (!isSerial) continue;

            std::map<std::string, JsonValue> device;
            device["device"] = JsonValue("/dev/" + name);
            device["name"] = JsonValue(name);

            // Determine device type
            if (name.find("ttyUSB") == 0) {
                device["driver"] = JsonValue("usbserial");
                device["device_type"] = JsonValue("usb");
            } else if (name.find("ttyACM") == 0) {
                device["driver"] = JsonValue("cdc_acm");
                device["device_type"] = JsonValue("usb");
            } else if (name.find("ttyS") == 0) {
                device["driver"] = JsonValue("serial8250");
                device["device_type"] = JsonValue("uart");
            } else if (name.find("ttyAMA") == 0) {
                device["driver"] = JsonValue("amba-pl011");
                device["device_type"] = JsonValue("uart");
            }

            // Check if device node exists
            std::string devPath = "/dev/" + name;
            struct stat st;
            if (stat(devPath.c_str(), &st) == 0) {
                device["path"] = JsonValue(devPath);
                device["available"] = JsonValue(true);
                serialDevices.push_back(JsonValue(device));
            }
        }
    }

    result["serial_devices"] = JsonValue(serialDevices);
    result["count"] = JsonValue(static_cast<int>(serialDevices.size()));

    std::map<std::string, JsonValue> response;
    response["result"] = JsonValue(result);
    return JsonValue(response);
}

// ============================================================================
// E2: machine/update/client — Update a specific client
// ============================================================================

JsonValue KlippyUdsServer::handleMachineUpdateClient(const JsonValue& params) {
    std::string clientName;
    if (params.isObject()) {
        const auto* nameVal = params.find("client");
        if (nameVal && nameVal->isString()) {
            clientName = nameVal->asString();
        }
    }

    if (clientName.empty()) {
        std::map<std::string, JsonValue> err;
        err["error"] = JsonValue("Missing 'client' parameter");
        err["code"] = JsonValue(400);
        return JsonValue(err);
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Update the client's status in updateStatus_
    auto it = updateStatus_.find(clientName);
    if (it == updateStatus_.end()) {
        // Register the client if not already present
        updateStatus_[clientName] = "updating";
    } else {
        it->second = "updating";
    }

    std::map<std::string, JsonValue> result;
    result["client"] = JsonValue(clientName);
    result["status"] = JsonValue("update_initiated");
    result["message"] = JsonValue("Update initiated for client: " + clientName);

    std::map<std::string, JsonValue> response;
    response["result"] = JsonValue(result);
    return JsonValue(response);
}

// ============================================================================
// E2: machine/update/rollback — Rollback a client update
// ============================================================================

JsonValue KlippyUdsServer::handleMachineUpdateRollback(const JsonValue& params) {
    std::string clientName;
    if (params.isObject()) {
        const auto* nameVal = params.find("client");
        if (nameVal && nameVal->isString()) {
            clientName = nameVal->asString();
        }
    }

    if (clientName.empty()) {
        std::map<std::string, JsonValue> err;
        err["error"] = JsonValue("Missing 'client' parameter");
        err["code"] = JsonValue(400);
        return JsonValue(err);
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Rollback the client's status
    auto it = updateStatus_.find(clientName);
    if (it != updateStatus_.end()) {
        it->second = "rolled_back";
    }

    std::map<std::string, JsonValue> result;
    result["client"] = JsonValue(clientName);
    result["status"] = JsonValue("rollback_complete");
    result["message"] = JsonValue("Rollback completed for client: " + clientName);

    std::map<std::string, JsonValue> response;
    response["result"] = JsonValue(result);
    return JsonValue(response);
}

} // namespace tether::klipper::klippy
