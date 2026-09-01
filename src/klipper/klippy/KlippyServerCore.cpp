/**
 * @file KlippyServerCore.cpp
 * @brief Core server lifecycle, state management, registration, and endpoint setup
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <set>
#include <sstream>
#include <sys/stat.h>

namespace tether::klipper::klippy {

// ============================================================================
// KlippyServer implementation
// ============================================================================

KlippyServer::KlippyServer(UdsServerConfig cfg)
    : config_(std::move(cfg)) {
    // Wire Spoolman URL from config
    if (!config_.spoolmanUrl.empty()) {
        spoolmanUrl_ = config_.spoolmanUrl;
        spoolmanConnected_ = true;
    }
    // Register core endpoints
    registerEndpoint("info", [this](const JsonValue& p) { return handleInfo(p); });
    registerEndpoint("emergency_stop", [this](const JsonValue& p) { return handleEmergencyStop(p); });
    registerEndpoint("register_remote_method", [this](const JsonValue& p) { return handleRegisterRemoteMethod(p); });
    registerEndpoint("list_endpoints", [this](const JsonValue& p) { return handleListEndpoints(p); });

    // G-code endpoints
    registerEndpoint("gcode/help", [this](const JsonValue& p) { return handleGcodeHelp(p); });
    registerEndpoint("gcode/script", [this](const JsonValue& p) { return handleGcodeScript(p); });
    registerEndpoint("gcode/restart", [this](const JsonValue& p) { return handleGcodeRestart(p); });
    registerEndpoint("gcode/firmware_restart", [this](const JsonValue& p) { return handleGcodeFirmwareRestart(p); });
    registerEndpoint("gcode/subscribe_output", [this](const JsonValue& p) { return handleGcodeSubscribeOutput(p); });

    // Query endpoints
    registerEndpoint("objects/list", [this](const JsonValue& p) { return handleObjectsList(p); });
    registerEndpoint("objects/query", [this](const JsonValue& p) { return handleObjectsQuery(p); });
    registerEndpoint("objects/subscribe", [this](const JsonValue& p) { return handleObjectsSubscribe(p); });

    // Print control endpoints
    registerEndpoint("printer/start", [this](const JsonValue& p) { return handlePrintStart(p); });
    registerEndpoint("printer/cancel", [this](const JsonValue& p) { return handlePrintCancel(p); });
    registerEndpoint("printer/pause", [this](const JsonValue& p) { return handlePrintPause(p); });
    registerEndpoint("printer/resume", [this](const JsonValue& p) { return handlePrintResume(p); });

    // Additional Moonraker-compatible endpoints
    registerEndpoint("server/info", [this](const JsonValue& p) { return handleServerInfo(p); });
    registerEndpoint("server/files/list", [this](const JsonValue& p) { return handleServerFilesList(p); });
    registerEndpoint("server/files/metadata", [this](const JsonValue& p) { return handleServerFilesMetadata(p); });
    registerEndpoint("machine/system_info", [this](const JsonValue& p) { return handleMachineSystemInfo(p); });
    registerEndpoint("machine/procstats", [this](const JsonValue& p) { return handleMachineProcstats(p); });

    // New Moonraker-compatible endpoints
    registerEndpoint("server/temperature_store", [this](const JsonValue& p) { return handleServerTemperatureStore(p); });
    registerEndpoint("server/gcode_store", [this](const JsonValue& p) { return handleServerGcodeStore(p); });
    registerEndpoint("server/files/directory", [this](const JsonValue& p) { return handleServerFilesDirectory(p); });
    registerEndpoint("server/files/get_directory", [this](const JsonValue& p) { return handleServerFilesDirectory(p); });
    registerEndpoint("server/files/move", [this](const JsonValue& p) { return handleServerFilesMove(p); });
    registerEndpoint("server/files/copy", [this](const JsonValue& p) { return handleServerFilesCopy(p); });
    registerEndpoint("server/files/delete", [this](const JsonValue& p) { return handleServerFilesDelete(p); });
    registerEndpoint("server/files/upload", [this](const JsonValue& p) { return handleServerFilesUpload(p); });
    registerEndpoint("machine/reboot", [this](const JsonValue& p) { return handleMachineReboot(p); });
    registerEndpoint("machine/shutdown", [this](const JsonValue& p) { return handleMachineShutdown(p); });
    registerEndpoint("machine/update/status", [this](const JsonValue& p) { return handleMachineUpdateStatus(p); });
    registerEndpoint("machine/device_power/devices", [this](const JsonValue& p) { return handleMachineDevicePowerDevices(p); });
    registerEndpoint("machine/device_power/state", [this](const JsonValue& p) { return handleMachineDevicePowerState(p); });
    registerEndpoint("printer/info", [this](const JsonValue& p) { return handlePrinterInfo(p); });
    registerEndpoint("printer/subscriptions", [this](const JsonValue& p) { return handlePrinterSubscriptions(p); });

    // C1: New Moonraker-compatible endpoints
    registerEndpoint("server/config", [this](const JsonValue& p) { return handleServerConfig(p); });
    registerEndpoint("server/files/roots", [this](const JsonValue& p) { return handleServerFilesRoots(p); });
    registerEndpoint("server/files/create_dir", [this](const JsonValue& p) { return handleServerFilesCreateDir(p); });
    registerEndpoint("server/files/metascan", [this](const JsonValue& p) { return handleServerFilesMetascan(p); });
    registerEndpoint("server/files/thumbnails", [this](const JsonValue& p) { return handleServerFilesThumbnails(p); });
    registerEndpoint("server/logs/rollover", [this](const JsonValue& p) { return handleServerLogsRollover(p); });
    registerEndpoint("server/klippy_log", [this](const JsonValue& p) { return handleServerKlippyLog(p); });
    registerEndpoint("server/moonraker_log", [this](const JsonValue& p) { return handleServerMoonrakerLog(p); });
    registerEndpoint("machine/services/list", [this](const JsonValue& p) { return handleMachineServicesList(p); });
    registerEndpoint("machine/services/restart", [this](const JsonValue& p) { return handleMachineServiceAction(p); });
    registerEndpoint("machine/services/stop", [this](const JsonValue& p) { return handleMachineServiceAction(p); });
    registerEndpoint("machine/services/start", [this](const JsonValue& p) { return handleMachineServiceAction(p); });
    registerEndpoint("machine/update/list", [this](const JsonValue& p) { return handleMachineUpdateList(p); });
    registerEndpoint("machine/update/refresh", [this](const JsonValue& p) { return handleMachineUpdateRefresh(p); });
    registerEndpoint("machine/update/update", [this](const JsonValue& p) { return handleMachineUpdateUpdate(p); });
    registerEndpoint("machine/update/recover", [this](const JsonValue& p) { return handleMachineUpdateRecover(p); });
    registerEndpoint("database/list", [this](const JsonValue& p) { return handleDatabaseList(p); });
    registerEndpoint("database/get", [this](const JsonValue& p) { return handleDatabaseGet(p); });
    registerEndpoint("database/put", [this](const JsonValue& p) { return handleDatabasePut(p); });
    registerEndpoint("database/delete", [this](const JsonValue& p) { return handleDatabaseDelete(p); });
    registerEndpoint("job_queue/status", [this](const JsonValue& p) { return handleJobQueueStatus(p); });
    registerEndpoint("job_queue/post_job", [this](const JsonValue& p) { return handleJobQueuePost(p); });
    registerEndpoint("job_queue/delete_job", [this](const JsonValue& p) { return handleJobQueueDelete(p); });
    registerEndpoint("job_history/list", [this](const JsonValue& p) { return handleJobHistoryList(p); });
    registerEndpoint("job_history/get", [this](const JsonValue& p) { return handleJobHistoryGet(p); });
    registerEndpoint("announcements/list", [this](const JsonValue& p) { return handleAnnouncementsList(p); });
    registerEndpoint("announcements/update", [this](const JsonValue& p) { return handleAnnouncementsUpdate(p); });
    registerEndpoint("announcements/dismiss", [this](const JsonValue& p) { return handleAnnouncementsDismiss(p); });
    registerEndpoint("webcams/list", [this](const JsonValue& p) { return handleWebcamsList(p); });
    registerEndpoint("webcams/get", [this](const JsonValue& p) { return handleWebcamsGet(p); });
    registerEndpoint("webcams/test", [this](const JsonValue& p) { return handleWebcamsTest(p); });
    registerEndpoint("devices/list", [this](const JsonValue& p) { return handleDevicesList(p); });
    registerEndpoint("devices/get", [this](const JsonValue& p) { return handleDevicesGet(p); });

    // E1-High: Additional Moonraker-compatible endpoints
    registerEndpoint("job_queue/pause", [this](const JsonValue& p) { return handleJobQueuePause(p); });
    registerEndpoint("job_queue/start", [this](const JsonValue& p) { return handleJobQueueStart(p); });
    registerEndpoint("job_queue/jump_to", [this](const JsonValue& p) { return handleJobQueueJumpTo(p); });
    registerEndpoint("job_history/delete", [this](const JsonValue& p) { return handleJobHistoryDelete(p); });
    registerEndpoint("webcams/update", [this](const JsonValue& p) { return handleWebcamsUpdate(p); });
    registerEndpoint("webcams/delete", [this](const JsonValue& p) { return handleWebcamsDelete(p); });
    registerEndpoint("machine/device_power/on", [this](const JsonValue& p) { return handleDevicePowerOn(p); });
    registerEndpoint("machine/device_power/off", [this](const JsonValue& p) { return handleDevicePowerOff(p); });
    registerEndpoint("machine/device_power/toggle", [this](const JsonValue& p) { return handleDevicePowerToggle(p); });
    registerEndpoint("machine/system_perms", [this](const JsonValue& p) { return handleMachineSystemPerms(p); });
    registerEndpoint("announcements/feed", [this](const JsonValue& p) { return handleAnnouncementsFeed(p); });
    registerEndpoint("server/files/get", [this](const JsonValue& p) { return handleServerFilesGet(p); });
    registerEndpoint("server/logs/list", [this](const JsonValue& p) { return handleServerLogsList(p); });

    // E1-Low: Access endpoints
    registerEndpoint("access/login", [this](const JsonValue& p) { return handleAccessLogin(p); });
    registerEndpoint("access/logout", [this](const JsonValue& p) { return handleAccessLogout(p); });
    registerEndpoint("access/user", [this](const JsonValue& p) { return handleAccessUser(p); });
    registerEndpoint("access/refresh_jwt", [this](const JsonValue& p) { return handleAccessRefreshJwt(p); });
    registerEndpoint("access/api_key", [this](const JsonValue& p) { return handleAccessApiKey(p); });
    registerEndpoint("access/oneshot_token", [this](const JsonValue& p) { return handleAccessOneshotToken(p); });

    // E1-Low: Bot endpoints
    registerEndpoint("bot/list", [this](const JsonValue& p) { return handleBotList(p); });
    registerEndpoint("bot/get", [this](const JsonValue& p) { return handleBotGet(p); });
    registerEndpoint("bot/update", [this](const JsonValue& p) { return handleBotUpdate(p); });
    registerEndpoint("bot/delete", [this](const JsonValue& p) { return handleBotDelete(p); });

    // E1-Low: Notepad endpoints
    registerEndpoint("notepad/list", [this](const JsonValue& p) { return handleNotepadList(p); });
    registerEndpoint("notepad/get", [this](const JsonValue& p) { return handleNotepadGet(p); });
    registerEndpoint("notepad/put", [this](const JsonValue& p) { return handleNotepadPut(p); });
    registerEndpoint("notepad/delete", [this](const JsonValue& p) { return handleNotepadDelete(p); });

    // E1-Low: Spoolman endpoints
    registerEndpoint("spoolman/info", [this](const JsonValue& p) { return handleSpoolmanInfo(p); });
    registerEndpoint("spoolman/spool_id", [this](const JsonValue& p) { return handleSpoolmanSpoolId(p); });
    registerEndpoint("spoolman/proxy", [this](const JsonValue& p) { return handleSpoolmanProxy(p); });

    // E1-Low: Device CRUD endpoints
    registerEndpoint("devices/create", [this](const JsonValue& p) { return handleDevicesCreate(p); });
    registerEndpoint("devices/delete", [this](const JsonValue& p) { return handleDevicesDelete(p); });

    // E1-Low: Database namespace endpoint
    registerEndpoint("database/ns", [this](const JsonValue& p) { return handleDatabaseNs(p); });

    // E2: Additional Moonraker-compatible endpoints
    registerEndpoint("server/restart", [this](const JsonValue& p) { return handleServerRestart(p); });
    registerEndpoint("printer/query_endstops/status", [this](const JsonValue& p) { return handleQueryEndstopsStatus(p); });
    registerEndpoint("machine/peripherals/usb", [this](const JsonValue& p) { return handleMachinePeripheralsUsb(p); });
    registerEndpoint("machine/peripherals/serial", [this](const JsonValue& p) { return handleMachinePeripheralsSerial(p); });
    registerEndpoint("machine/update/client", [this](const JsonValue& p) { return handleMachineUpdateClient(p); });
    registerEndpoint("machine/update/rollback", [this](const JsonValue& p) { return handleMachineUpdateRollback(p); });

    // Aliases: Mainsail/Moonraker use slightly different names for some
    // endpoints.  Register both forms so the frontend doesn't get 404s.
    registerEndpoint("machine/proc_stats", [this](const JsonValue& p) { return handleMachineProcstats(p); });
    registerEndpoint("server/database/list", [this](const JsonValue& p) { return handleDatabaseList(p); });
    registerEndpoint("server/database/get", [this](const JsonValue& p) { return handleDatabaseGet(p); });
    registerEndpoint("server/database/put", [this](const JsonValue& p) { return handleDatabasePut(p); });
    registerEndpoint("server/database/delete", [this](const JsonValue& p) { return handleDatabaseDelete(p); });
    registerEndpoint("server/database/post_item", [this](const JsonValue& p) { return handleDatabasePut(p); });
    registerEndpoint("server/database/get_item", [this](const JsonValue& p) { return handleDatabaseGet(p); });
    registerEndpoint("server/webcams/list", [this](const JsonValue& p) { return handleWebcamsList(p); });
    registerEndpoint("server/webcams/get", [this](const JsonValue& p) { return handleWebcamsGet(p); });
    registerEndpoint("server/webcams/test", [this](const JsonValue& p) { return handleWebcamsTest(p); });
    registerEndpoint("server/webcams/update", [this](const JsonValue& p) { return handleWebcamsUpdate(p); });
    registerEndpoint("server/webcams/delete", [this](const JsonValue& p) { return handleWebcamsDelete(p); });

    // Printer alias endpoints (Moonraker uses printer/ prefix for some)
    registerEndpoint("printer/emergency_stop", [this](const JsonValue& p) { return handleEmergencyStop(p); });
    registerEndpoint("printer/restart", [this](const JsonValue& p) { return handleGcodeRestart(p); });
    registerEndpoint("printer/firmware_restart", [this](const JsonValue& p) { return handleGcodeFirmwareRestart(p); });
    registerEndpoint("printer/gcode/help", [this](const JsonValue& p) { return handleGcodeHelp(p); });
    registerEndpoint("printer/gcode/script", [this](const JsonValue& p) { return handleGcodeScript(p); });
    registerEndpoint("printer/gcode/subscribe_output", [this](const JsonValue& p) { return handleGcodeSubscribeOutput(p); });
    registerEndpoint("printer/objects/list", [this](const JsonValue& p) { return handleObjectsList(p); });
    registerEndpoint("printer/objects/query", [this](const JsonValue& p) { return handleObjectsQuery(p); });
    registerEndpoint("printer/objects/subscribe", [this](const JsonValue& p) { return handleObjectsSubscribe(p); });
    registerEndpoint("printer/print/start", [this](const JsonValue& p) { return handlePrintStart(p); });
    registerEndpoint("printer/print/cancel", [this](const JsonValue& p) { return handlePrintCancel(p); });
    registerEndpoint("printer/print/pause", [this](const JsonValue& p) { return handlePrintPause(p); });
    registerEndpoint("printer/print/resume", [this](const JsonValue& p) { return handlePrintResume(p); });

    // Register built-in printer objects
    registerObject(std::make_shared<WebhooksObject>(*this));
    registerObject(std::make_shared<GcodeMoveObject>());
    registerObject(std::make_shared<ToolheadObject>());
    registerObject(std::make_shared<ConfigfileObject>());
    registerObject(std::make_shared<PauseResumeObject>());
    registerObject(std::make_shared<VirtualSdcardObject>());
    registerObject(std::make_shared<DisplayStatusObject>());

    // Initialize default file roots
    registerFileRoot("gcodes", fileRoot_, true);
    registerFileRoot("config", "/etc/tether", true);
    registerFileRoot("logs", "/var/log", false);

    // Initialize default services
    registerService("klipper", "active", "running");
    registerService("moonraker", "active", "running");
    registerService("webcamd", "active", "running");
    registerService("klipper-mcu", "active", "running");

    // No default webcams — Mainsail handles an empty webcam list gracefully.
    // Registering a fake URL causes "Failed to fetch" errors in the browser.

    // Initialize emulated server config
    initServerConfig();
}

KlippyServer::~KlippyServer() = default;

bool KlippyServer::loadConfigFile(const std::string& path) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!configParser_.parseFile(path)) {
        stateMessage_ = "Failed to load config: " + path;
        return false;
    }
    // Update configfile object with the path and parsed settings
    auto it = objects_.find("configfile");
    if (it != objects_.end()) {
        auto* cfg = dynamic_cast<ConfigfileObject*>(it->second.get());
        if (cfg) {
            cfg->setPath(path);
            // Build settings/config maps from parsed sections.
            // Mainsail expects: settings["section_name"] = { key: value, ... }
            // Keys are lowercase section names, values are typed (numbers as
            // doubles, booleans as bools, everything else as strings).
            // config uses original section names so frontends like Fluidd can
            // find e.g. "gcode_macro CANCEL_PRINT".
            std::map<std::string, JsonValue> settings;
            std::map<std::string, JsonValue> config;
            for (const auto& section : configParser_.sections()) {
                std::string sectionName = section.name;
                // Lowercase the section name for settings
                std::string lowerName = sectionName;
                std::transform(lowerName.begin(), lowerName.end(),
                              lowerName.begin(), ::tolower);

                std::map<std::string, JsonValue> sectionSettings;
                std::map<std::string, JsonValue> sectionConfig;
                for (const auto& [key, rawValue] : section.values) {
                    // Try to parse as number
                    std::string lowerKey = key;
                    std::transform(lowerKey.begin(), lowerKey.end(),
                                  lowerKey.begin(), ::tolower);

                    // Try double
                    try {
                        size_t pos;
                        double d = std::stod(rawValue, &pos);
                        if (pos == rawValue.size()) {
                            sectionSettings[lowerKey] = JsonValue(d);
                            sectionConfig[lowerKey] = JsonValue(rawValue);
                            continue;
                        }
                    } catch (...) {}

                    // Try bool
                    if (rawValue == "true" || rawValue == "True") {
                        sectionSettings[lowerKey] = JsonValue(true);
                        sectionConfig[lowerKey] = JsonValue(rawValue);
                    } else if (rawValue == "false" || rawValue == "False") {
                        sectionSettings[lowerKey] = JsonValue(false);
                        sectionConfig[lowerKey] = JsonValue(rawValue);
                    } else {
                        sectionSettings[lowerKey] = JsonValue(rawValue);
                        sectionConfig[lowerKey] = JsonValue(rawValue);
                    }
                }
                settings[lowerName] = JsonValue(sectionSettings);
                config[sectionName] = JsonValue(sectionConfig);
            }
            cfg->setSettings(std::move(settings));
            cfg->setConfig(std::move(config));
        }
    }
    return true;
}

bool KlippyServer::loadConfig() {
    return loadConfigFile(config_.configFile);
}

void KlippyServer::setState(PrinterState newState, const std::string& message) {
    std::vector<StateChangeCallback> cbs;
    bool changed = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        PrinterState oldState = state_;

        // Define valid state transitions
        bool valid = false;
        if (newState == state_) {
            // Same state — just update message
            if (!message.empty()) stateMessage_ = message;
            // Not a change, but not invalid either
        } else if (newState == PrinterState::Shutdown) {
            // Any state can transition to Shutdown
            valid = true;
            stateMessage_ = message.empty() ? "Shutdown" : message;
        } else if (state_ == PrinterState::Startup && newState == PrinterState::Ready) {
            valid = true;
            stateMessage_ = message.empty() ? "Printer is ready" : message;
        } else if (state_ == PrinterState::Startup && newState == PrinterState::Error) {
            valid = true;
            stateMessage_ = message.empty() ? "Printer initialization error" : message;
        } else if (state_ == PrinterState::Ready && newState == PrinterState::Printing) {
            valid = true;
            stateMessage_ = message.empty() ? "Printing started" : message;
        } else if (state_ == PrinterState::Printing && newState == PrinterState::Paused) {
            valid = true;
            stateMessage_ = message.empty() ? "Print paused" : message;
        } else if (state_ == PrinterState::Paused && newState == PrinterState::Printing) {
            valid = true;
            stateMessage_ = message.empty() ? "Print resumed" : message;
        } else if (state_ == PrinterState::Printing && newState == PrinterState::Ready) {
            valid = true;
            stateMessage_ = message.empty() ? "Print complete" : message;
        } else if (state_ == PrinterState::Paused && newState == PrinterState::Ready) {
            valid = true;
            stateMessage_ = message.empty() ? "Print cancelled" : message;
        } else if (state_ == PrinterState::Ready && newState == PrinterState::Error) {
            valid = true;
            stateMessage_ = message.empty() ? "Printer error" : message;
        } else if (state_ == PrinterState::Printing && newState == PrinterState::Error) {
            valid = true;
            stateMessage_ = message.empty() ? "Printer error during print" : message;
        } else if (state_ == PrinterState::Paused && newState == PrinterState::Error) {
            valid = true;
            stateMessage_ = message.empty() ? "Printer error while paused" : message;
        } else if (state_ == PrinterState::Error && newState == PrinterState::Ready) {
            // Error recovery — allow transitioning back to Ready
            valid = true;
            stateMessage_ = message.empty() ? "Printer recovered from error" : message;
        } else if (state_ == PrinterState::Error && newState == PrinterState::Startup) {
            // Error recovery — restart
            valid = true;
            stateMessage_ = message.empty() ? "Printer restarting" : message;
        }

        if (valid) {
            state_ = newState;
        }
        changed = (state_ != oldState);
        if (changed) cbs = stateChangeCbs_;
    }
    // Notify observers outside the mutex
    if (changed) {
        for (const auto& cb : cbs) {
            if (cb) cb(state_, stateMessage_);
        }
    }
}

PrinterState KlippyServer::state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_;
}

std::string KlippyServer::stateMessage() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stateMessage_;
}

void KlippyServer::registerEndpoint(const std::string& method, EndpointHandler handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    endpoints_[method] = std::move(handler);
}

JsonValue KlippyServer::callEndpoint(const std::string& method, const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = endpoints_.find(method);
    if (it == endpoints_.end()) {
        std::map<std::string, JsonValue> err;
        err["error"] = JsonValue("Endpoint not found: " + method);
        err["code"] = JsonValue(404);
        return JsonValue(err);
    }
    return it->second(params);
}

void KlippyServer::unregisterEndpoint(const std::string& method) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    endpoints_.erase(method);
}

std::vector<std::string> KlippyServer::listEndpoints() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> result;
    for (const auto& [k, _] : endpoints_) result.push_back(k);
    return result;
}

void KlippyServer::registerObject(std::shared_ptr<PrinterObject> obj) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    objects_[obj->name()] = std::move(obj);
}

void KlippyServer::unregisterObject(const std::string& name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    objects_.erase(name);
}

std::vector<std::string> KlippyServer::listObjects() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> result;
    for (const auto& [k, _] : objects_) result.push_back(k);
    return result;
}

std::map<std::string, std::map<std::string, JsonValue>>
KlippyServer::queryObjects(
    const std::map<std::string, std::vector<std::string>>& objects) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, std::map<std::string, JsonValue>> result;
    for (const auto& [objName, fields] : objects) {
        auto it = objects_.find(objName);
        if (it == objects_.end()) continue;
        result[objName] = it->second->status(fields);
    }
    return result;
}

void KlippyServer::emitGcodeResponse(const std::string& response) {
    std::vector<GcodeResponseCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        // Record in gcode store
        gcodeStore_.push_back({response, std::chrono::steady_clock::now()});
        if (gcodeStore_.size() > kMaxGcodeStoreEntries) {
            gcodeStore_.erase(gcodeStore_.begin());
        }
        cbs = gcodeResponseCbs_;
    }
    // Notify observers outside the mutex
    for (const auto& cb : cbs) {
        if (cb) cb(response);
    }
}

void KlippyServer::setGcodeScriptHandler(std::function<void(const std::string&)> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    gcodeScriptHandler_ = std::move(handler);
}

void KlippyServer::setRestartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    restartHandler_ = std::move(handler);
}

void KlippyServer::setFirmwareRestartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    firmwareRestartHandler_ = std::move(handler);
}

void KlippyServer::setEmergencyStopHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    emergencyStopHandler_ = std::move(handler);
}

void KlippyServer::setPrintStartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printStartHandler_ = std::move(handler);
}

void KlippyServer::setPrintCancelHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printCancelHandler_ = std::move(handler);
}

void KlippyServer::setPrintPauseHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printPauseHandler_ = std::move(handler);
}

void KlippyServer::setPrintResumeHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printResumeHandler_ = std::move(handler);
}

// ============================================================================
// Event callback registration
// ============================================================================

void KlippyServer::addGcodeResponseCallback(GcodeResponseCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    gcodeResponseCbs_.push_back(std::move(cb));
}

void KlippyServer::addStateChangeCallback(StateChangeCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    stateChangeCbs_.push_back(std::move(cb));
}

void KlippyServer::addFilelistChangedCallback(FilelistChangedCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    filelistChangedCbs_.push_back(std::move(cb));
}

void KlippyServer::addHistoryChangedCallback(HistoryChangedCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    historyChangedCbs_.push_back(std::move(cb));
}

void KlippyServer::addJobQueueChangedCallback(JobQueueChangedCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    jobQueueChangedCbs_.push_back(std::move(cb));
}

void KlippyServer::addPowerChangedCallback(PowerChangedCallback cb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    powerChangedCbs_.push_back(std::move(cb));
}

// ============================================================================
// Data recording
// ============================================================================

void KlippyServer::recordTemperature(const std::string& heater, double temp, double target) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    TempStoreEntry entry{temp, target, std::chrono::steady_clock::now()};
    tempStore_[heater].push_back(entry);
    if (tempStore_[heater].size() > kMaxTempStoreEntries) {
        tempStore_[heater].erase(tempStore_[heater].begin());
    }
}

void KlippyServer::recordGcodeResponse(const std::string& msg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    gcodeStore_.push_back({msg, std::chrono::steady_clock::now()});
    if (gcodeStore_.size() > kMaxGcodeStoreEntries) {
        gcodeStore_.erase(gcodeStore_.begin());
    }
}

// ============================================================================
// Registration helpers
// ============================================================================

void KlippyServer::registerPowerDevice(const std::string& name, const std::string& initialState) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    PowerDevice dev{name, initialState, false};
    powerDevices_[name] = dev;
}

bool KlippyServer::setPowerDeviceState(const std::string& name, const std::string& state) {
    std::vector<PowerChangedCallback> cbs;
    bool found = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = powerDevices_.find(name);
        if (it == powerDevices_.end()) return false;
        it->second.state = state;
        found = true;
        cbs = powerChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb(name, state);
    }
    return true;
}

void KlippyServer::setUpdateStatus(const std::string& component, const std::string& status) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    updateStatus_[component] = status;
}

void KlippyServer::setSpoolmanUrl(const std::string& url) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    spoolmanUrl_ = url;
    spoolmanConnected_ = !url.empty();
}

// ============================================================================
// User / token management
// ============================================================================

bool KlippyServer::deleteUser(const std::string& username) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = users_.find(username);
    if (it == users_.end()) return false;
    users_.erase(it);
    return true;
}

std::string KlippyServer::generateOneshotToken() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Remove expired tokens
    auto now = std::chrono::steady_clock::now();
    auto ttl = std::chrono::seconds(kOneshotTokenTtlSeconds);
    oneshotTokens_.erase(
        std::remove_if(oneshotTokens_.begin(), oneshotTokens_.end(),
            [&](const OneshotToken& t) { return now - t.createdAt > ttl; }),
        oneshotTokens_.end());

    std::random_device rd;
    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 16; ++i) {
        ss << std::setw(2) << std::setfill('0') << (rd() & 0xFF);
    }
    std::string token = ss.str();
    oneshotTokens_.push_back({token, now});
    return token;
}

bool KlippyServer::consumeOneshotToken(const std::string& token) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Remove expired tokens
    auto now = std::chrono::steady_clock::now();
    auto ttl = std::chrono::seconds(kOneshotTokenTtlSeconds);
    oneshotTokens_.erase(
        std::remove_if(oneshotTokens_.begin(), oneshotTokens_.end(),
            [&](const OneshotToken& t) { return now - t.createdAt > ttl; }),
        oneshotTokens_.end());

    auto it = std::find_if(oneshotTokens_.begin(), oneshotTokens_.end(),
        [&](const OneshotToken& t) { return t.token == token; });
    if (it == oneshotTokens_.end()) return false;
    oneshotTokens_.erase(it);
    return true;
}

} // namespace tether::klipper::klippy
