/**
 * @file KlippyUdsServerGcodeEndpoints.cpp
 * @brief G-code, query, and print control endpoint handlers
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
    std::map<std::string, JsonValue> result;
    // Motion
    result["G0"] = JsonValue("Rapid move");
    result["G1"] = JsonValue("Linear move");
    result["G2"] = JsonValue("Arc move clockwise");
    result["G3"] = JsonValue("Arc move counter-clockwise");
    result["G4"] = JsonValue("Dwell");
    result["G17"] = JsonValue("XY plane select");
    result["G18"] = JsonValue("XZ plane select");
    result["G19"] = JsonValue("YZ plane select");
    result["G20"] = JsonValue("Set units to inches");
    result["G21"] = JsonValue("Set units to millimeters");
    result["G28"] = JsonValue("Home axes");
    result["G29"] = JsonValue("Bed mesh leveling");
    result["G30"] = JsonValue("Probe");
    result["G38"] = JsonValue("Probe toward target");
    result["G60"] = JsonValue("Save position");
    result["G61"] = JsonValue("Restore position");
    result["G90"] = JsonValue("Absolute coordinates");
    result["G91"] = JsonValue("Relative coordinates");
    result["G92"] = JsonValue("Set position");
    // Extrusion
    result["G10"] = JsonValue("Firmware retract");
    result["G11"] = JsonValue("Firmware unretract");
    result["M82"] = JsonValue("Absolute extrusion");
    result["M83"] = JsonValue("Relative extrusion");
    // Temperature
    result["M104"] = JsonValue("Set hotend temperature");
    result["M109"] = JsonValue("Wait for hotend temperature");
    result["M140"] = JsonValue("Set bed temperature");
    result["M190"] = JsonValue("Wait for bed temperature");
    result["M105"] = JsonValue("Get temperatures");
    result["M155"] = JsonValue("Auto temperature reporting");
    // Fan
    result["M106"] = JsonValue("Set fan speed");
    result["M107"] = JsonValue("Fan off");
    // Motors
    result["M17"] = JsonValue("Enable motors");
    result["M18"] = JsonValue("Disable motors");
    result["M84"] = JsonValue("Disable motors");
    // SD card
    result["M20"] = JsonValue("List SD files");
    result["M23"] = JsonValue("Select SD file");
    result["M24"] = JsonValue("Start/resume SD print");
    result["M25"] = JsonValue("Pause SD print");
    result["M27"] = JsonValue("Report SD status");
    // Display
    result["M73"] = JsonValue("Set display progress");
    result["M117"] = JsonValue("Set display message");
    result["M118"] = JsonValue("Output message");
    // Status
    result["M114"] = JsonValue("Get current position");
    result["M119"] = JsonValue("Get endstop status");
    // Sync
    result["M400"] = JsonValue("Wait for moves to finish");
    // Overrides
    result["M220"] = JsonValue("Set speed factor");
    result["M221"] = JsonValue("Set extrude factor");
    // Advanced motion
    result["M205"] = JsonValue("Advanced motion settings");
    result["M900"] = JsonValue("Set pressure advance");
    result["M593"] = JsonValue("Set input shaper");
    // Settings
    result["M500"] = JsonValue("Save settings");
    result["M501"] = JsonValue("Load settings");
    result["M502"] = JsonValue("Reset to factory defaults");
    result["M503"] = JsonValue("Report current settings");
    // Emergency
    result["M112"] = JsonValue("Emergency stop");
    // Nozzle
    result["G12"] = JsonValue("Clean nozzle");
    // Spline
    result["G5"] = JsonValue("Bezier spline move");
    // Firmware info
    result["M115"] = JsonValue("Get firmware version");
    result["M116"] = JsonValue("Wait for all temperatures");
    // Stepper config
    result["M92"] = JsonValue("Set steps per mm");
    result["M350"] = JsonValue("Set microstepping");
    result["M906"] = JsonValue("Set stepper driver current");
    result["M569"] = JsonValue("Set stepper direction");
    // Motion limits
    result["M200"] = JsonValue("Set filament diameter");
    result["M201"] = JsonValue("Set print acceleration");
    result["M203"] = JsonValue("Set max feedrate");
    result["M204"] = JsonValue("Set acceleration");
    // Offsets
    result["M206"] = JsonValue("Set home offset");
    result["M218"] = JsonValue("Set tool offset");
    result["M851"] = JsonValue("Set probe Z offset");
    // Retract
    result["M207"] = JsonValue("Set retract parameters");
    result["M208"] = JsonValue("Set unretract parameters");
    // PID
    result["M301"] = JsonValue("Set hotend PID");
    result["M303"] = JsonValue("PID autotune");
    result["M304"] = JsonValue("Set bed PID");
    // Probe
    result["M401"] = JsonValue("Deploy probe");
    result["M402"] = JsonValue("Stow probe");
    // Bed mesh
    result["M420"] = JsonValue("Enable/disable bed mesh");
    result["M421"] = JsonValue("Set bed mesh point");
    // Backlash
    result["M425"] = JsonValue("Set backlash compensation");
    // Misc
    result["M42"] = JsonValue("Set pin state");
    result["M150"] = JsonValue("Set LED color");
    result["M280"] = JsonValue("Servo control");
    result["M300"] = JsonValue("Beep");
    result["M600"] = JsonValue("Filament change");
    return JsonValue(result);
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
