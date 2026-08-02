#pragma once

/// @file KlippyUdsServer.hpp
/// @brief Unix domain socket server implementing the Moonraker-facing API.
///
/// This file defines the main KlippyUdsServer class. Supporting types
/// (JsonValue, PrinterState, PrinterObject, UdsTypes) are in separate headers.

#include "tether/klipper/klippy/JsonValue.hpp"
#include "tether/klipper/klippy/PrinterState.hpp"
#include "tether/klipper/klippy/PrinterObject.hpp"
#include "tether/klipper/klippy/UdsTypes.hpp"

#include "tether/klipper/config/ConfigParser.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Unix domain socket server implementing the Moonraker-facing API.
///
/// The server runs an event loop in a background thread that:
///   - Accepts new connections
///   - Reads frames (ETX-delimited JSON)
///   - Dispatches requests to endpoint handlers
///   - Sends responses and push messages
///   - Periodically refreshes subscriptions (coalescing)
///
/// @section uds_threading Thread safety
///
/// All public methods of KlippyUdsServer are safe to call from any thread.
/// Internal state is protected by `mutex_` (a `std::recursive_mutex`).
/// The event loop runs on `eventThread_` and acquires `mutex_` for each
/// iteration. Endpoint handlers are called with `mutex_` held, so they
/// must not call back into methods that acquire `mutex_` recursively in
/// a way that could deadlock (the recursive mutex allows re-entry, so
/// this is generally safe).
///
/// **Note**: Endpoint handlers that modify external state (e.g.,
/// KlippyInstance state) are responsible for synchronizing that external
/// state. See KlippyInstance's threading model documentation.
class KlippyUdsServer {
public:
    explicit KlippyUdsServer(UdsServerConfig cfg = {});
    ~KlippyUdsServer();

    KlippyUdsServer(const KlippyUdsServer&) = delete;
    KlippyUdsServer& operator=(const KlippyUdsServer&) = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /// @brief Create the socket, bind, and start listening.
    /// @return True on success.
    bool start();

    /// @brief Stop the server, close all connections, remove socket file.
    void stop();

    /// @brief Check if the server is running.
    bool isRunning() const { return running_.load(); }

    // ------------------------------------------------------------------
    // State management
    // ------------------------------------------------------------------

    /// @brief Transition to a new printer state.
    /// Only valid transitions: startup->ready, startup->error, ready->shutdown.
    void setState(PrinterState state, const std::string& message = "");

    /// @brief Get current printer state.
    PrinterState state() const;

    /// @brief Get current state message.
    std::string stateMessage() const;

    // ------------------------------------------------------------------
    // Endpoint registration
    // ------------------------------------------------------------------

    /// @brief Register an endpoint handler.
    /// @param method The method name (e.g. "gcode/script").
    /// @param handler Function that processes params and returns result.
    void registerEndpoint(const std::string& method, EndpointHandler handler);

    /// @brief Unregister an endpoint.
    void unregisterEndpoint(const std::string& method);

    /// @brief List all registered endpoints.
    std::vector<std::string> listEndpoints() const;

    /// @brief Invoke a registered endpoint by name (for testing/direct calls).
    /// @param method The endpoint method name.
    /// @param params The parameters to pass to the handler.
    /// @return The result from the endpoint handler, or error JSON if not found.
    JsonValue callEndpoint(const std::string& method, const JsonValue& params);

    /// @brief Set the Spoolman server URL at runtime.
    void setSpoolmanUrl(const std::string& url) {
        spoolmanUrl_ = url;
        spoolmanConnected_ = !url.empty();
    }

    /// @brief Get the Spoolman server URL.
    std::string spoolmanUrl() const { return spoolmanUrl_; }

    // ------------------------------------------------------------------
    // Printer object registration
    // ------------------------------------------------------------------

    /// @brief Register a printer object for status queries.
    void registerObject(std::shared_ptr<PrinterObject> obj);

    /// @brief Unregister a printer object.
    void unregisterObject(const std::string& name);

    /// @brief List all registered printer object names.
    std::vector<std::string> listObjects() const;

    /// @brief Query object status.
    /// @param objects Map of object name -> field list (empty = all).
    /// @return Status map and eventtime.
    std::map<std::string, std::map<std::string, JsonValue>> queryObjects(
        const std::map<std::string, std::vector<std::string>>& objects) const;

    // ------------------------------------------------------------------
    // Remote methods
    // ------------------------------------------------------------------

    /// @brief Invoke a remote method (Server -> Client push).
    /// @param method Method name.
    /// @param params Parameters to send.
    void invokeRemoteMethod(const std::string& method,
                            const JsonValue& params);

    // ------------------------------------------------------------------
    // G-code output
    // ------------------------------------------------------------------

    /// @brief Emit a G-code response line to all subscribers.
    void emitGcodeResponse(const std::string& response);

    // ------------------------------------------------------------------
    // Subscription management (for testing)
    // ------------------------------------------------------------------

    /// @brief Get number of active subscriptions.
    size_t subscriptionCount() const;

    /// @brief Manually trigger a subscription refresh (for testing).
    void refreshSubscriptions();

    // ------------------------------------------------------------------
    // Configuration access
    // ------------------------------------------------------------------

    const UdsServerConfig& config() const { return config_; }

private:
    // ------------------------------------------------------------------
    // Internal types
    // ------------------------------------------------------------------

    /// @brief Process a single JSON frame from a connection.
    void processFrame(UdsConnection& conn, const JsonValue& frame);

    /// @brief Handle a request (has method + id).
    void handleRequest(UdsConnection& conn, const JsonValue& frame);

    /// @brief Handle a notification (has method, no id).
    void handleNotification(UdsConnection& conn, const JsonValue& frame);

    /// @brief Send a response to a connection.
    void sendResponse(UdsConnection& conn, const JsonValue& id,
                      const JsonValue& result);

    /// @brief Send an error response to a connection.
    void sendError(UdsConnection& conn, const JsonValue& id,
                   const std::string& message);

    /// @brief Send a push message to a connection.
    void sendPush(UdsConnection& conn, const JsonValue& msg);

    // ------------------------------------------------------------------
    // Core endpoint handlers
    // ------------------------------------------------------------------

    JsonValue handleInfo(const JsonValue& params);
    JsonValue handleEmergencyStop(const JsonValue& params);
    JsonValue handleRegisterRemoteMethod(const JsonValue& params);
    JsonValue handleListEndpoints(const JsonValue& params);

    // ------------------------------------------------------------------
    // G-code endpoint handlers
    // ------------------------------------------------------------------

    JsonValue handleGcodeHelp(const JsonValue& params);
    JsonValue handleGcodeScript(const JsonValue& params);
    JsonValue handleGcodeRestart(const JsonValue& params);
    JsonValue handleGcodeFirmwareRestart(const JsonValue& params);
    JsonValue handleGcodeSubscribeOutput(const JsonValue& params);

    // ------------------------------------------------------------------
    // Query endpoint handlers
    // ------------------------------------------------------------------

    JsonValue handleObjectsList(const JsonValue& params);
    JsonValue handleObjectsQuery(const JsonValue& params);
    JsonValue handleObjectsSubscribe(const JsonValue& params);

    // Print control endpoint handlers
    JsonValue handlePrintStart(const JsonValue& params);
    JsonValue handlePrintCancel(const JsonValue& params);
    JsonValue handlePrintPause(const JsonValue& params);
    JsonValue handlePrintResume(const JsonValue& params);

    // Additional Moonraker-compatible endpoint handlers
    JsonValue handleServerInfo(const JsonValue& params);
    JsonValue handleServerFilesList(const JsonValue& params);
    JsonValue handleServerFilesMetadata(const JsonValue& params);
    JsonValue handleMachineSystemInfo(const JsonValue& params);
    JsonValue handleMachineProcstats(const JsonValue& params);

    // New Moonraker-compatible endpoint handlers
    JsonValue handleServerTemperatureStore(const JsonValue& params);
    JsonValue handleServerGcodeStore(const JsonValue& params);
    JsonValue handleServerFilesDirectory(const JsonValue& params);
    JsonValue handleServerFilesMove(const JsonValue& params);
    JsonValue handleServerFilesCopy(const JsonValue& params);
    JsonValue handleServerFilesDelete(const JsonValue& params);
    JsonValue handleServerFilesUpload(const JsonValue& params);
    JsonValue handleMachineReboot(const JsonValue& params);
    JsonValue handleMachineShutdown(const JsonValue& params);
    JsonValue handleMachineUpdateStatus(const JsonValue& params);
    JsonValue handleMachineDevicePowerDevices(const JsonValue& params);
    JsonValue handleMachineDevicePowerState(const JsonValue& params);
    JsonValue handlePrinterInfo(const JsonValue& params);
    JsonValue handlePrinterSubscriptions(const JsonValue& params);

    // C1: New Moonraker-compatible endpoint handlers
    JsonValue handleServerConfig(const JsonValue& params);
    JsonValue handleServerFilesRoots(const JsonValue& params);
    JsonValue handleServerFilesCreateDir(const JsonValue& params);
    JsonValue handleServerFilesMetascan(const JsonValue& params);
    JsonValue handleServerFilesThumbnails(const JsonValue& params);
    JsonValue handleServerLogsRollover(const JsonValue& params);
    JsonValue handleServerKlippyLog(const JsonValue& params);
    JsonValue handleServerMoonrakerLog(const JsonValue& params);
    JsonValue handleMachineServicesList(const JsonValue& params);
    JsonValue handleMachineServiceAction(const JsonValue& params);
    JsonValue handleMachineUpdateList(const JsonValue& params);
    JsonValue handleMachineUpdateRefresh(const JsonValue& params);
    JsonValue handleMachineUpdateUpdate(const JsonValue& params);
    JsonValue handleMachineUpdateRecover(const JsonValue& params);
    JsonValue handleDatabaseList(const JsonValue& params);
    JsonValue handleDatabaseGet(const JsonValue& params);
    JsonValue handleDatabasePut(const JsonValue& params);
    JsonValue handleDatabaseDelete(const JsonValue& params);
    JsonValue handleJobQueueStatus(const JsonValue& params);
    JsonValue handleJobQueuePost(const JsonValue& params);
    JsonValue handleJobQueueDelete(const JsonValue& params);
    JsonValue handleJobHistoryList(const JsonValue& params);
    JsonValue handleJobHistoryGet(const JsonValue& params);
    JsonValue handleAnnouncementsList(const JsonValue& params);
    JsonValue handleAnnouncementsUpdate(const JsonValue& params);
    JsonValue handleAnnouncementsDismiss(const JsonValue& params);
    JsonValue handleWebcamsList(const JsonValue& params);
    JsonValue handleWebcamsGet(const JsonValue& params);
    JsonValue handleWebcamsTest(const JsonValue& params);
    JsonValue handleDevicesList(const JsonValue& params);
    JsonValue handleDevicesGet(const JsonValue& params);

    // E1-High: Additional endpoint handlers
    JsonValue handleJobQueuePause(const JsonValue& params);
    JsonValue handleJobQueueStart(const JsonValue& params);
    JsonValue handleJobQueueJumpTo(const JsonValue& params);
    JsonValue handleJobHistoryDelete(const JsonValue& params);
    JsonValue handleWebcamsUpdate(const JsonValue& params);
    JsonValue handleWebcamsDelete(const JsonValue& params);
    JsonValue handleDevicePowerOn(const JsonValue& params);
    JsonValue handleDevicePowerOff(const JsonValue& params);
    JsonValue handleDevicePowerToggle(const JsonValue& params);
    JsonValue handleMachineSystemPerms(const JsonValue& params);
    JsonValue handleAnnouncementsFeed(const JsonValue& params);
    JsonValue handleServerFilesGet(const JsonValue& params);
    JsonValue handleServerLogsList(const JsonValue& params);

    // E1-Low: Access endpoints
    JsonValue handleAccessLogin(const JsonValue& params);
    JsonValue handleAccessLogout(const JsonValue& params);
    JsonValue handleAccessUser(const JsonValue& params);
    JsonValue handleAccessRefreshJwt(const JsonValue& params);
    JsonValue handleAccessApiKey(const JsonValue& params);
    JsonValue handleAccessOneshotToken(const JsonValue& params);

    // E1-Low: Bot endpoints
    JsonValue handleBotList(const JsonValue& params);
    JsonValue handleBotGet(const JsonValue& params);
    JsonValue handleBotUpdate(const JsonValue& params);
    JsonValue handleBotDelete(const JsonValue& params);

    // E1-Low: Notepad endpoints
    JsonValue handleNotepadList(const JsonValue& params);
    JsonValue handleNotepadGet(const JsonValue& params);
    JsonValue handleNotepadPut(const JsonValue& params);
    JsonValue handleNotepadDelete(const JsonValue& params);

    // E1-Low: Spoolman endpoints
    JsonValue handleSpoolmanInfo(const JsonValue& params);
    JsonValue handleSpoolmanSpoolId(const JsonValue& params);
    JsonValue handleSpoolmanProxy(const JsonValue& params);

    // E1-Low: Device CRUD endpoints
    JsonValue handleDevicesCreate(const JsonValue& params);
    JsonValue handleDevicesDelete(const JsonValue& params);

    // E1-Low: Database namespace endpoint
    JsonValue handleDatabaseNs(const JsonValue& params);

    // E2: Additional Moonraker-compatible endpoints
    JsonValue handleServerRestart(const JsonValue& params);
    JsonValue handleQueryEndstopsStatus(const JsonValue& params);
    JsonValue handleMachinePeripheralsUsb(const JsonValue& params);
    JsonValue handleMachinePeripheralsSerial(const JsonValue& params);
    JsonValue handleMachineUpdateClient(const JsonValue& params);
    JsonValue handleMachineUpdateRollback(const JsonValue& params);

    // ------------------------------------------------------------------
    // Event loop
    // ------------------------------------------------------------------

    void eventLoop();
    void acceptConnection();
    void processConnections();
    void cleanupConnections();

    // ------------------------------------------------------------------
    // Subscription refresh
    // ------------------------------------------------------------------

    void subscriptionRefreshTick();

    // ------------------------------------------------------------------
    // G-code script callback (set by external G-code executor)
    // ------------------------------------------------------------------

public:
    /// @brief Set the G-code script execution callback.
    /// Called when gcode/script endpoint is invoked.
    void setGcodeScriptHandler(std::function<void(const std::string&)> handler);

    /// @brief Set the restart callback.
    void setRestartHandler(std::function<void()> handler);

    /// @brief Set the firmware restart callback.
    void setFirmwareRestartHandler(std::function<void()> handler);

    /// @brief Set the emergency stop callback.
    void setEmergencyStopHandler(std::function<void()> handler);

    /// @brief Set print control callbacks (for printer/start, printer/cancel, etc.)
    void setPrintStartHandler(std::function<void()> handler);
    void setPrintCancelHandler(std::function<void()> handler);
    void setPrintPauseHandler(std::function<void()> handler);
    void setPrintResumeHandler(std::function<void()> handler);

    /// @brief Load printer configuration from a file.
    /// @param path Path to the printer.cfg file.
    /// @return True if loaded successfully.
    bool loadConfigFile(const std::string& path);

    /// @brief Load printer configuration from the config file path
    /// stored in UdsServerConfig::configFile.
    /// @return True if loaded successfully.
    bool loadConfig();

    /// @brief Get the loaded configuration parser.
    const config::ConfigParser& configParser() const { return configParser_; }

    /// @brief Set the VirtualSdcard for file operations.
    void setVirtualSdcard(std::shared_ptr<class VirtualSdcard> sd) { sdcard_ = std::move(sd); }

    /// @brief Set the file root directory for server/files endpoints.
    void setFileRoot(const std::string& root) { fileRoot_ = root; }

    /// @brief Record a temperature reading in the temperature store.
    void recordTemperature(const std::string& heater, double temp, double target);

    /// @brief Record a G-code response in the gcode store.
    void recordGcodeResponse(const std::string& msg);

    /// @brief Register a power device.
    void registerPowerDevice(const std::string& name, const std::string& initialState = "off");

    /// @brief Set a power device state.
    bool setPowerDeviceState(const std::string& name, const std::string& state);

    /// @brief Set update status for a component.
    void setUpdateStatus(const std::string& component, const std::string& status);

    /// @brief Register a webcam.
    void registerWebcam(const std::string& name, const std::string& url,
                         const std::string& service = "mjpegstreamer");

    /// @brief Register a system service.
    void registerService(const std::string& name,
                          const std::string& activeState = "active",
                          const std::string& subState = "running");

    /// @brief Register a file root.
    void registerFileRoot(const std::string& name, const std::string& path,
                           bool writable = true);

    /// @brief Initialize the emulated server config (Moonraker-style config).
    void initServerConfig();

    /// @brief Put a database value.
    void databasePut(const std::string& ns, const std::string& key, const JsonValue& value);

    /// @brief Get a database value.
    std::optional<JsonValue> databaseGet(const std::string& ns, const std::string& key);

    /// @brief Delete a database value.
    bool databaseDelete(const std::string& ns, const std::string& key);

    /// @brief Add a job to the job queue.
    void jobQueueAdd(const std::string& filename);

    /// @brief Add a job history entry.
    int64_t jobHistoryAdd(const std::string& filename, const std::string& status);

    /// @brief Add an announcement.
    void announcementAdd(const std::string& entryId, const std::string& title,
                          const std::string& description, const std::string& severity = "info");

    /// @brief Register a user for access control.
    void registerUser(const std::string& username, const std::string& password,
                       const std::vector<std::string>& permissions = {});

    /// @brief Register a bot.
    void registerBot(const std::string& name, const std::string& type,
                      const std::string& token = "", bool enabled = false);

    /// @brief Set notepad entry.
    void notepadPut(const std::string& key, const std::string& value);

    /// @brief Get notepad entry.
    std::optional<std::string> notepadGet(const std::string& key) const;

    /// @brief Set Spoolman connection state.
    void setSpoolmanConnected(bool connected, const std::string& url = "");

    /// @brief Set current spool ID.
    void setSpoolId(int64_t id);

    /// @brief Add a log file to the log list.
    void addLogFile(const std::string& name, const std::string& path);

    /// @brief Set system permissions for a resource.
    void setSystemPerms(const std::string& resource, const std::vector<std::string>& perms);

private:
    UdsServerConfig config_;

    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread eventThread_;

    mutable std::recursive_mutex mutex_;

    // State
    PrinterState state_ = PrinterState::Startup;
    std::string stateMessage_ = "Printer is not ready";

    // Endpoints
    std::map<std::string, EndpointHandler> endpoints_;

    // Printer objects
    std::map<std::string, std::shared_ptr<PrinterObject>> objects_;

    // Connections
    std::vector<std::unique_ptr<UdsConnection>> connections_;
    int nextConnId_ = 0;

    // Subscriptions (per-connection)
    std::vector<Subscription> subscriptions_;

    // Remote methods (method name -> list of registrations)
    std::map<std::string, std::vector<RemoteMethod>> remoteMethods_;

    // G-code output subscribers (connections with gcode/subscribe_output)
    std::set<UdsConnection*> gcodeSubscribers_;

    // Callbacks
    std::function<void(const std::string&)> gcodeScriptHandler_;
    std::function<void()> restartHandler_;
    std::function<void()> firmwareRestartHandler_;
    std::function<void()> emergencyStopHandler_;
    std::function<void()> printStartHandler_;
    std::function<void()> printCancelHandler_;
    std::function<void()> printPauseHandler_;
    std::function<void()> printResumeHandler_;

    // Configuration
    config::ConfigParser configParser_;

    // VirtualSdcard reference for file operations
    std::shared_ptr<class VirtualSdcard> sdcard_;
    std::string fileRoot_ = "/tmp/tether_sdcard";

    // Temperature store (ring buffer of recent temperature readings)
    struct TempStoreEntry {
        double temperature = 0.0;
        double target = 0.0;
        std::chrono::steady_clock::time_point timestamp;
    };
    std::map<std::string, std::vector<TempStoreEntry>> tempStore_;
    static constexpr size_t kMaxTempStoreEntries = 1200; // 20 min at 1s interval

    // G-code response store (ring buffer of recent responses)
    struct GcodeStoreEntry {
        std::string message;
        std::chrono::steady_clock::time_point timestamp;
    };
    std::vector<GcodeStoreEntry> gcodeStore_;
    static constexpr size_t kMaxGcodeStoreEntries = 1000;

    // Power device state
    struct PowerDevice {
        std::string name;
        std::string state; // "on", "off"
        bool locked = false;
    };
    std::map<std::string, PowerDevice> powerDevices_;

    // Update status
    std::map<std::string, std::string> updateStatus_;

    // C1: Database store (namespace -> key -> value)
    std::map<std::string, std::map<std::string, JsonValue>> database_;

    // C1: Job queue (list of filenames)
    std::vector<std::string> jobQueue_;

    // C1: Job history
    struct JobHistoryEntry {
        int64_t jobId = 0;
        std::string filename;
        std::string status; // "completed", "cancelled", "error", "in_progress"
        double startTime = 0.0;
        double endTime = 0.0;
        double printDuration = 0.0;
        double totalDuration = 0.0;
        double filamentUsed = 0.0;
        int64_t layerCount = 0;
        int64_t firstLayerHeight = 0;
        int64_t firstLayerExtruder = 0;
    };
    std::vector<JobHistoryEntry> jobHistory_;
    int64_t nextJobId_ = 1;

    // C1: Announcements
    struct Announcement {
        std::string entryId;
        std::string title;
        std::string description;
        std::string url;
        std::string date;
        std::string severity; // "info", "warning", "critical"
        bool dismissed = false;
    };
    std::vector<Announcement> announcements_;

    // C1: Webcams
    struct Webcam {
        std::string name;
        std::string url;
        std::string service = "mjpegstreamer";
        bool enabled = true;
        int rotation = 0;
        double aspectRatio = 4.0 / 3.0;
        std::string source = "database";
    };
    std::map<std::string, Webcam> webcams_;

    // C1: Services
    struct Service {
        std::string name;
        std::string activeState = "active";
        std::string subState = "running";
    };
    std::map<std::string, Service> services_;

    // C1: File roots
    struct FileRoot {
        std::string name;
        std::string path;
        bool writable = true;
    };
    std::map<std::string, FileRoot> fileRoots_;

    // C1: Server config (emulated Moonraker config)
    std::map<std::string, JsonValue> serverConfig_;

    // E1: Job queue state
    bool jobQueuePaused_ = false;
    size_t jobQueueCurrentIndex_ = 0;

    // E1: System permissions
    std::map<std::string, std::vector<std::string>> systemPerms_;

    // E1: Access / user management
    struct User {
        std::string username;
        std::string password;
        std::vector<std::string> permissions;
        std::string source = "moonraker";
        std::string jwtSecret;
    };
    std::map<std::string, User> users_;
    std::string apiKey_ = "tether_default_api_key";
    std::vector<std::string> oneshotTokens_;

    // E1: Bot management
    struct Bot {
        std::string name;
        std::string type; // "telegram", "discord", "slack"
        std::string token;
        std::string chatId;
        bool enabled = false;
    };
    std::map<std::string, Bot> bots_;

    // E1: Notepad
    struct NotepadEntry {
        std::string key;
        std::string value;
        int64_t lastModified = 0;
    };
    std::map<std::string, NotepadEntry> notepad_;

    // E1: Spoolman integration
    bool spoolmanConnected_ = false;
    std::string spoolmanUrl_;
    int64_t currentSpoolId_ = 0;
    std::map<std::string, JsonValue> spoolmanInfo_;

    // E1: Log files
    struct LogFile {
        std::string name;
        std::string path;
        int64_t size = 0;
        int64_t modified = 0;
    };
    std::vector<LogFile> logFiles_;

    // Subscription refresh timing
    std::chrono::steady_clock::time_point lastRefresh_;
};

} // namespace tether::klipper::klippy

// Built-in printer objects that reference KlippyUdsServer
#include "tether/klipper/klippy/PrinterObjectsBuiltin.hpp"
