/**
 * @file KlippyUdsServerGcodeEndpoints.cpp
 * @brief G-code, query, and print control endpoint handlers
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/KlippyUdsHelpText.hpp"

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
// Core endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleInfo(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    result["state"] = JsonValue(stateToString(state_));
    result["state_message"] = JsonValue(stateMessage_);
    result["hostname"] = JsonValue("localhost");
    result["klipper_path"] = JsonValue(config_.klipperPath);
    result["python_path"] = JsonValue(config_.pythonPath);
    result["process_id"] = JsonValue(static_cast<int64_t>(getpid()));
    result["user_id"] = JsonValue(static_cast<int64_t>(getuid()));
    result["group_id"] = JsonValue(static_cast<int64_t>(getgid()));
    result["log_file"] = JsonValue(config_.logFile);
    result["config_file"] = JsonValue(config_.configFile);
    result["software_version"] = JsonValue(config_.softwareVersion);
    result["cpu_info"] = JsonValue("tether-klippy");

    // Record client_info if present
    if (params.has("client_info")) {
        // Just accept it; no validation needed
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleEmergencyStop(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (emergencyStopHandler_) emergencyStopHandler_();
    }
    setState(PrinterState::Shutdown, "Shutdown due to webhooks request");
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyServer::handleRegisterRemoteMethod(const JsonValue& params) {
    // Remote method registration is handled specially in processConnections()
    // where the connection context is available. This handler is a no-op
    // fallback — the actual registration logic is in the frame processing path.
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyServer::handleListEndpoints(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> endpoints;
    for (const auto& [k, _] : endpoints_) {
        endpoints.emplace_back(k);
    }
    result["endpoints"] = JsonValue(endpoints);
    return JsonValue(result);
}

// ============================================================================
// G-code endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleGcodeHelp(const JsonValue& params) {
    return getGcodeHelpJson();
}

JsonValue KlippyServer::handleGcodeScript(const JsonValue& params) {
    std::string script;
    const auto* scriptVal = params.find("script");
    if (scriptVal && scriptVal->isString()) {
        script = scriptVal->asString();
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        // Record the command in the gcode store
        gcodeStore_.push_back({script, std::chrono::steady_clock::now(), "command"});
        if (gcodeStore_.size() > kMaxGcodeStoreEntries) {
            gcodeStore_.erase(gcodeStore_.begin());
        }
        if (gcodeScriptHandler_) gcodeScriptHandler_(script);
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyServer::handleGcodeRestart(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (restartHandler_) restartHandler_();
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyServer::handleGcodeFirmwareRestart(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (firmwareRestartHandler_) firmwareRestartHandler_();
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyServer::handleGcodeSubscribeOutput(const JsonValue& params) {
    // The connection that sends this becomes a G-code output subscriber.
    // We need the connection context - handled specially in handleRequest.
    // Return empty result.
    return JsonValue(std::map<std::string, JsonValue>{});
}

// ============================================================================
// Query endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleObjectsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> objList;
    for (const auto& [k, _] : objects_) {
        objList.emplace_back(k);
    }
    result["objects"] = JsonValue(objList);
    return JsonValue(result);
}

JsonValue KlippyServer::handleObjectsQuery(const JsonValue& params) {
    std::map<std::string, std::vector<std::string>> queryObjs;
    if (params.has("objects") && params.find("objects")->isObject()) {
        for (const auto& [objName, fieldsVal] : params.find("objects")->asObject()) {
            std::vector<std::string> fields;
            if (fieldsVal.isNull()) {
                // null means all fields
            } else if (fieldsVal.isArray()) {
                for (const auto& f : fieldsVal.asArray()) {
                    if (f.isString()) fields.push_back(f.asString());
                }
            }
            queryObjs[objName] = fields;
        }
    }

    auto status = queryObjects(queryObjs);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> statusJson;
    for (const auto& [objName, fields] : status) {
        std::map<std::string, JsonValue> fieldMap;
        for (const auto& [f, v] : fields) {
            fieldMap[f] = v;
        }
        statusJson[objName] = JsonValue(fieldMap);
    }
    result["status"] = JsonValue(statusJson);
    result["eventtime"] = JsonValue(
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    return JsonValue(result);
}

JsonValue KlippyServer::handleObjectsSubscribe(const JsonValue& params) {
    // Parse requested objects
    std::map<std::string, std::vector<std::string>> queryObjs;
    if (params.has("objects") && params.find("objects")->isObject()) {
        for (const auto& [objName, fieldsVal] : params.find("objects")->asObject()) {
            std::vector<std::string> fields;
            if (fieldsVal.isNull()) {
                // null means all fields
            } else if (fieldsVal.isArray()) {
                for (const auto& f : fieldsVal.asArray()) {
                    if (f.isString()) fields.push_back(f.asString());
                }
            }
            queryObjs[objName] = fields;
        }
    }

    // Get response template
    JsonValue template_ = params.has("response_template")
        ? *params.find("response_template")
        : JsonValue(std::map<std::string, JsonValue>{});

    // Return initial snapshot
    auto status = queryObjects(queryObjs);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> statusJson;
    for (const auto& [objName, fields] : status) {
        std::map<std::string, JsonValue> fieldMap;
        for (const auto& [f, v] : fields) {
            fieldMap[f] = v;
        }
        statusJson[objName] = JsonValue(fieldMap);
    }
    result["status"] = JsonValue(statusJson);
    result["eventtime"] = JsonValue(
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    // Note: actual subscription registration happens in handleRequest
    // where we have access to the connection. For now, return the snapshot.
    return JsonValue(result);
}

// ============================================================================
// Print control endpoint handlers
// ============================================================================

JsonValue KlippyServer::handlePrintStart(const JsonValue& params) {
    std::string filename;
    if (params.isObject()) {
        auto* fn = params.find("filename");
        if (fn && fn->isString()) filename = fn->asString();
    }
    if (printStartHandler_) {
        printStartHandler_();
    }
    setState(PrinterState::Printing, filename.empty() ? "Print started" : "Printing: " + filename);
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyServer::handlePrintCancel(const JsonValue& params) {
    if (printCancelHandler_) {
        printCancelHandler_();
    }
    setState(PrinterState::Ready, "Print cancelled");
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyServer::handlePrintPause(const JsonValue& params) {
    if (printPauseHandler_) {
        printPauseHandler_();
    }
    setState(PrinterState::Paused, "Print paused");
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyServer::handlePrintResume(const JsonValue& params) {
    if (printResumeHandler_) {
        printResumeHandler_();
    }
    setState(PrinterState::Printing, "Print resumed");
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

} // namespace tether::klipper::klippy
