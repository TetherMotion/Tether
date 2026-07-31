/**
 * @file KlippyUdsServerCore.cpp
 * @brief Core server lifecycle, state management, registration, and frame processing
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
// KlippyUdsServer implementation
// ============================================================================

KlippyUdsServer::KlippyUdsServer(UdsServerConfig cfg)
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

    // Initialize default webcams
    registerWebcam("default", "http://localhost:8080/?action=stream");

    // Initialize emulated server config
    initServerConfig();
}

KlippyUdsServer::~KlippyUdsServer() {
    stop();
}

bool KlippyUdsServer::loadConfigFile(const std::string& path) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!configParser_.parseFile(path)) {
        stateMessage_ = "Failed to load config: " + path;
        return false;
    }
    // Update configfile object with the path
    auto it = objects_.find("configfile");
    if (it != objects_.end()) {
        auto* cfg = dynamic_cast<ConfigfileObject*>(it->second.get());
        if (cfg) cfg->setPath(path);
    }
    return true;
}

bool KlippyUdsServer::loadConfig() {
    return loadConfigFile(config_.configFile);
}

bool KlippyUdsServer::start() {
    // Remove pre-existing socket file
    ::unlink(config_.socketPath.c_str());

    // Create AF_UNIX SOCK_STREAM socket
    listenFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listenFd_ < 0) return false;

    // Bind
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, config_.socketPath.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(listenFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    // Listen
    if (::listen(listenFd_, config_.backlog) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    // Set non-blocking
    int flags = fcntl(listenFd_, F_GETFL, 0);
    if (flags >= 0) fcntl(listenFd_, F_SETFL, flags | O_NONBLOCK);

    running_ = true;
    lastRefresh_ = std::chrono::steady_clock::now();
    eventThread_ = std::thread([this]() { eventLoop(); });
    return true;
}

void KlippyUdsServer::stop() {
    running_ = false;
    if (eventThread_.joinable()) eventThread_.join();

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    connections_.clear();
    subscriptions_.clear();
    gcodeSubscribers_.clear();
    remoteMethods_.clear();

    if (listenFd_ >= 0) {
        ::close(listenFd_);
        listenFd_ = -1;
    }
    ::unlink(config_.socketPath.c_str());
}

void KlippyUdsServer::setState(PrinterState newState, const std::string& message) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Validate transitions
    if (state_ == PrinterState::Ready && newState == PrinterState::Shutdown) {
        state_ = newState;
        stateMessage_ = message.empty() ? "Shutdown" : message;
    } else if (state_ == PrinterState::Startup && newState == PrinterState::Ready) {
        state_ = newState;
        stateMessage_ = message.empty() ? "Printer is ready" : message;
    } else if (state_ == PrinterState::Startup && newState == PrinterState::Error) {
        state_ = newState;
        stateMessage_ = message.empty() ? "Printer initialization error" : message;
    } else if (state_ == PrinterState::Startup && newState == PrinterState::Shutdown) {
        // Emergency stop during startup is allowed
        state_ = newState;
        stateMessage_ = message.empty() ? "Shutdown" : message;
    } else if (state_ == PrinterState::Error && newState == PrinterState::Shutdown) {
        // No-op: error is already terminal
    } else if (newState == state_) {
        // Same state, just update message
        if (!message.empty()) stateMessage_ = message;
    }
}

PrinterState KlippyUdsServer::state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_;
}

std::string KlippyUdsServer::stateMessage() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stateMessage_;
}

void KlippyUdsServer::registerEndpoint(const std::string& method, EndpointHandler handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    endpoints_[method] = std::move(handler);
}

void KlippyUdsServer::unregisterEndpoint(const std::string& method) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    endpoints_.erase(method);
}

std::vector<std::string> KlippyUdsServer::listEndpoints() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> result;
    for (const auto& [k, _] : endpoints_) result.push_back(k);
    return result;
}

void KlippyUdsServer::registerObject(std::shared_ptr<PrinterObject> obj) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    objects_[obj->name()] = std::move(obj);
}

void KlippyUdsServer::unregisterObject(const std::string& name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    objects_.erase(name);
}

std::vector<std::string> KlippyUdsServer::listObjects() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> result;
    for (const auto& [k, _] : objects_) result.push_back(k);
    return result;
}

std::map<std::string, std::map<std::string, JsonValue>>
KlippyUdsServer::queryObjects(
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

void KlippyUdsServer::invokeRemoteMethod(const std::string& method,
                                          const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = remoteMethods_.find(method);
    if (it == remoteMethods_.end()) return;

    for (auto& reg : it->second) {
        if (reg.conn && !reg.conn->closed()) {
            JsonValue msg = reg.responseTemplate;
            if (msg.isObject() || msg.isNull()) {
                if (!msg.isObject()) msg = JsonValue(std::map<std::string, JsonValue>{});
                msg.asObject()["method"] = JsonValue(method);
                msg.asObject()["params"] = params;
            }
            reg.conn->sendFrame(msg.dump());
        }
    }
    // Clean up closed connections
    auto& regs = it->second;
    regs.erase(std::remove_if(regs.begin(), regs.end(),
        [](const RemoteMethod& r) { return !r.conn || r.conn->closed(); }),
        regs.end());
    if (regs.empty()) remoteMethods_.erase(it);
}

void KlippyUdsServer::emitGcodeResponse(const std::string& response) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Record in gcode store
    gcodeStore_.push_back({response, std::chrono::steady_clock::now()});
    if (gcodeStore_.size() > kMaxGcodeStoreEntries) {
        gcodeStore_.erase(gcodeStore_.begin());
    }
    for (auto* conn : gcodeSubscribers_) {
        if (conn && !conn->closed()) {
            std::map<std::string, JsonValue> msg;
            msg["method"] = JsonValue("process_gcode_response");
            std::map<std::string, JsonValue> params;
            params["response"] = JsonValue(response);
            msg["params"] = JsonValue(params);
            conn->sendFrame(JsonValue(msg).dump());
        }
    }
}

size_t KlippyUdsServer::subscriptionCount() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return subscriptions_.size();
}

void KlippyUdsServer::refreshSubscriptions() {
    subscriptionRefreshTick();
}

void KlippyUdsServer::setGcodeScriptHandler(std::function<void(const std::string&)> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    gcodeScriptHandler_ = std::move(handler);
}

void KlippyUdsServer::setRestartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    restartHandler_ = std::move(handler);
}

void KlippyUdsServer::setFirmwareRestartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    firmwareRestartHandler_ = std::move(handler);
}

void KlippyUdsServer::setEmergencyStopHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    emergencyStopHandler_ = std::move(handler);
}

void KlippyUdsServer::setPrintStartHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printStartHandler_ = std::move(handler);
}

void KlippyUdsServer::setPrintCancelHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printCancelHandler_ = std::move(handler);
}

void KlippyUdsServer::setPrintPauseHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printPauseHandler_ = std::move(handler);
}

void KlippyUdsServer::setPrintResumeHandler(std::function<void()> handler) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    printResumeHandler_ = std::move(handler);
}

// ============================================================================
// Frame processing
// ============================================================================

void KlippyUdsServer::processFrame(UdsConnection& conn, const JsonValue& frame) {
    if (!frame.isObject()) return; // Malformed: top-level must be object

    bool hasId = frame.has("id");
    bool hasMethod = frame.has("method");

    if (hasMethod && hasId) {
        handleRequest(conn, frame);
    } else if (hasMethod && !hasId) {
        handleNotification(conn, frame);
    }
    // If has id but no method: malformed, ignore
}

void KlippyUdsServer::handleRequest(UdsConnection& conn, const JsonValue& frame) {
    const JsonValue& id = *frame.find("id");
    std::string method = frame.find("method")->asString();
    JsonValue params = frame.has("params") ? *frame.find("params") : JsonValue(std::map<std::string, JsonValue>{});

    EndpointHandler handler;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = endpoints_.find(method);
        if (it == endpoints_.end()) {
            sendError(conn, id, "Unknown method: " + method);
            return;
        }
        handler = it->second;
    }

    try {
        JsonValue result = handler(params);
        if (!result.isObject()) {
            result = JsonValue(std::map<std::string, JsonValue>{});
        }
        sendResponse(conn, id, result);
    } catch (const EndpointError& e) {
        sendError(conn, id, e.what());
    } catch (const std::exception& e) {
        sendError(conn, id, e.what());
        // Unhandled exception -> transition to shutdown
        setState(PrinterState::Shutdown, std::string("Internal error: ") + e.what());
    }
}

void KlippyUdsServer::handleNotification(UdsConnection& conn, const JsonValue& frame) {
    // Notifications are fire-and-forget; currently no notifications from client
    // are defined in the protocol.
    (void)conn;
    (void)frame;
}

void KlippyUdsServer::sendResponse(UdsConnection& conn, const JsonValue& id,
                                    const JsonValue& result) {
    std::map<std::string, JsonValue> msg;
    msg["id"] = id;
    msg["result"] = result;
    conn.sendFrame(JsonValue(msg).dump());
}

void KlippyUdsServer::sendError(UdsConnection& conn, const JsonValue& id,
                                 const std::string& message) {
    std::map<std::string, JsonValue> msg;
    msg["id"] = id;
    std::map<std::string, JsonValue> err;
    err["message"] = JsonValue(message);
    err["error"] = JsonValue("WebRequestError");
    msg["error"] = JsonValue(err);
    conn.sendFrame(JsonValue(msg).dump());
}

void KlippyUdsServer::sendPush(UdsConnection& conn, const JsonValue& msg) {
    conn.sendFrame(msg.dump());
}

} // namespace tether::klipper::klippy
