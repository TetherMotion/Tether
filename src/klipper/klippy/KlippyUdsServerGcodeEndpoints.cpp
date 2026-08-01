/**
 * @file KlippyUdsServerGcodeEndpoints.cpp
 * @brief G-code, query, and print control endpoint handlers
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/KlippyUdsHelpText.hpp"
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
// Core endpoint handlers
// ============================================================================

JsonValue KlippyUdsServer::handleInfo(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleEmergencyStop(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (emergencyStopHandler_) emergencyStopHandler_();
    }
    setState(PrinterState::Shutdown, "Shutdown due to webhooks request");
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyUdsServer::handleRegisterRemoteMethod(const JsonValue& params) {
    // Remote method registration is handled specially in processConnections()
    // where the connection context is available. This handler is a no-op
    // fallback — the actual registration logic is in the frame processing path.
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyUdsServer::handleListEndpoints(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleGcodeHelp(const JsonValue& params) {
    return getGcodeHelpJson();
}

JsonValue KlippyUdsServer::handleGcodeScript(const JsonValue& params) {
    std::string script;
    if (params.has("script")) {
        script = params.find("script")->asString();
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (gcodeScriptHandler_) gcodeScriptHandler_(script);
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyUdsServer::handleGcodeRestart(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (restartHandler_) restartHandler_();
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyUdsServer::handleGcodeFirmwareRestart(const JsonValue& params) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (firmwareRestartHandler_) firmwareRestartHandler_();
    }
    return JsonValue(std::map<std::string, JsonValue>{});
}

JsonValue KlippyUdsServer::handleGcodeSubscribeOutput(const JsonValue& params) {
    // The connection that sends this becomes a G-code output subscriber.
    // We need the connection context - handled specially in handleRequest.
    // Return empty result.
    return JsonValue(std::map<std::string, JsonValue>{});
}

// ============================================================================
// Query endpoint handlers
// ============================================================================

JsonValue KlippyUdsServer::handleObjectsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> objList;
    for (const auto& [k, _] : objects_) {
        objList.emplace_back(k);
    }
    result["objects"] = JsonValue(objList);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleObjectsQuery(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleObjectsSubscribe(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handlePrintStart(const JsonValue& params) {
    if (printStartHandler_) {
        printStartHandler_();
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handlePrintCancel(const JsonValue& params) {
    if (printCancelHandler_) {
        printCancelHandler_();
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handlePrintPause(const JsonValue& params) {
    if (printPauseHandler_) {
        printPauseHandler_();
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handlePrintResume(const JsonValue& params) {
    if (printResumeHandler_) {
        printResumeHandler_();
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

} // namespace tether::klipper::klippy
