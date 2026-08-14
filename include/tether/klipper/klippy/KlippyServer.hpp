#pragma once

/// @file KlippyServer.hpp
/// @brief Core business-logic layer for the Klipper/Moonraker-compatible API.
///
/// KlippyServer contains all endpoint handlers, data stores, state management,
/// and printer-object registration. It is transport-agnostic: both
/// KlippyUdsServer (Unix domain socket) and KlippyHttpServer (HTTP/WebSocket)
/// delegate to a shared KlippyServer instance.

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
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Core business-logic server for the Klipper/Moonraker-compatible API.
///
/// This class owns all endpoint handlers, data stores (database, job queue,
/// job history, announcements, webcams, services, users, bots, etc.), printer
/// state, printer objects, and configuration. It is independent of any
/// transport (UDS, HTTP, WebSocket).
///
/// @section server_threading Thread safety
///
/// All public methods are safe to call from any thread. Internal state is
/// protected by `mutex_` (a `std::recursive_mutex`). Event callbacks are
/// invoked *outside* the mutex to avoid deadlocks when a transport layer
/// needs to acquire its own lock inside a callback.
class KlippyServer {
public:
    explicit KlippyServer(UdsServerConfig cfg = {});
    ~KlippyServer();

    KlippyServer(const KlippyServer&) = delete;
    KlippyServer& operator=(const KlippyServer&) = delete;

    // ------------------------------------------------------------------
    // State management
    // ------------------------------------------------------------------

    /// @brief Transition to a new printer state.
    void setState(PrinterState state, const std::string& message = "");

    /// @brief Get current printer state.
    PrinterState state() const;

    /// @brief Get current state message.
    std::string stateMessage() const;

    // ------------------------------------------------------------------
    // Endpoint registration
    // ------------------------------------------------------------------

    /// @brief Register an endpoint handler.
    void registerEndpoint(const std::string& method, EndpointHandler handler);

    /// @brief Unregister an endpoint.
    void unregisterEndpoint(const std::string& method);

    /// @brief List all registered endpoints.
    std::vector<std::string> listEndpoints() const;

    /// @brief Invoke a registered endpoint by name.
    /// @return The result from the endpoint handler, or error JSON if not found.
    JsonValue callEndpoint(const std::string& method, const JsonValue& params);

    // ------------------------------------------------------------------
    // Printer object registration
    // ------------------------------------------------------------------

    void registerObject(std::shared_ptr<PrinterObject> obj);
    void unregisterObject(const std::string& name);
    std::vector<std::string> listObjects() const;

    /// @brief Query object status.
    std::map<std::string, std::map<std::string, JsonValue>> queryObjects(
        const std::map<std::string, std::vector<std::string>>& objects) const;

    // ------------------------------------------------------------------
    // G-code output
    // ------------------------------------------------------------------

    /// @brief Emit a G-code response line.
    /// Records in the gcode store and notifies all registered observers.
    void emitGcodeResponse(const std::string& response);

    // ------------------------------------------------------------------
    // Configuration access
    // ------------------------------------------------------------------

    const UdsServerConfig& config() const { return config_; }

    /// @brief Load printer configuration from a file.
    bool loadConfigFile(const std::string& path);

    /// @brief Load printer configuration from the config file path
    /// stored in UdsServerConfig::configFile.
    bool loadConfig();

    /// @brief Get the loaded configuration parser.
    const config::ConfigParser& configParser() const { return configParser_; }

    // ------------------------------------------------------------------
    // External handler callbacks (set by KlippyInstance)
    // ------------------------------------------------------------------

    void setGcodeScriptHandler(std::function<void(const std::string&)> handler);
    void setRestartHandler(std::function<void()> handler);
    void setFirmwareRestartHandler(std::function<void()> handler);
    void setEmergencyStopHandler(std::function<void()> handler);
    void setPrintStartHandler(std::function<void()> handler);
    void setPrintCancelHandler(std::function<void()> handler);
    void setPrintPauseHandler(std::function<void()> handler);
    void setPrintResumeHandler(std::function<void()> handler);

    // ------------------------------------------------------------------
    // File / sdcard configuration
    // ------------------------------------------------------------------

    void setVirtualSdcard(std::shared_ptr<class VirtualSdcard> sd) { sdcard_ = std::move(sd); }
    void setFileRoot(const std::string& root) { fileRoot_ = root; }

    // ------------------------------------------------------------------
    // Data recording
    // ------------------------------------------------------------------

    void recordTemperature(const std::string& heater, double temp, double target);
    void recordGcodeResponse(const std::string& msg);

    // ------------------------------------------------------------------
    // Registration helpers (for KlippyInstance / external setup)
    // ------------------------------------------------------------------

    void registerPowerDevice(const std::string& name, const std::string& initialState = "off");
    bool setPowerDeviceState(const std::string& name, const std::string& state);
    void setUpdateStatus(const std::string& component, const std::string& status);
    void registerWebcam(const std::string& name, const std::string& url,
                         const std::string& service = "mjpegstreamer");
    void registerService(const std::string& name,
                          const std::string& activeState = "active",
                          const std::string& subState = "running");
    void registerFileRoot(const std::string& name, const std::string& path,
                           bool writable = true);
    void initServerConfig();

    // ------------------------------------------------------------------
    // Database operations
    // ------------------------------------------------------------------

    void databasePut(const std::string& ns, const std::string& key, const JsonValue& value);
    std::optional<JsonValue> databaseGet(const std::string& ns, const std::string& key);
    bool databaseDelete(const std::string& ns, const std::string& key);

    // ------------------------------------------------------------------
    // Job queue / history
    // ------------------------------------------------------------------

    void jobQueueAdd(const std::string& filename);
    int64_t jobHistoryAdd(const std::string& filename, const std::string& status);

    // ------------------------------------------------------------------
    // Announcements / users / bots / notepad / spoolman
    // ------------------------------------------------------------------

    void announcementAdd(const std::string& entryId, const std::string& title,
                          const std::string& description, const std::string& severity = "info");

    void registerUser(const std::string& username, const std::string& password,
                       const std::vector<std::string>& permissions = {});

    void registerBot(const std::string& name, const std::string& type,
                      const std::string& token = "", bool enabled = false);

    void notepadPut(const std::string& key, const std::string& value);
    std::optional<std::string> notepadGet(const std::string& key) const;

    void setSpoolmanUrl(const std::string& url);
    std::string spoolmanUrl() const { return spoolmanUrl_; }

    void setSpoolmanConnected(bool connected, const std::string& url = "");
    void setSpoolId(int64_t id);

    void addLogFile(const std::string& name, const std::string& path);
    void setSystemPerms(const std::string& resource, const std::vector<std::string>& perms);

    // ------------------------------------------------------------------
    // Event callbacks (for transport layers: UDS, HTTP, etc.)
    // ------------------------------------------------------------------

    using GcodeResponseCallback = std::function<void(const std::string&)>;
    using StateChangeCallback = std::function<void(PrinterState, const std::string&)>;
    using FilelistChangedCallback = std::function<void(const std::string& action,
                                                       const std::string& path,
                                                       const std::string& root)>;
    using HistoryChangedCallback = std::function<void(const std::string& action, int64_t jobId)>;
    using JobQueueChangedCallback = std::function<void(const std::string& action)>;
    using PowerChangedCallback = std::function<void(const std::string& device, const std::string& state)>;

    void addGcodeResponseCallback(GcodeResponseCallback cb);
    void addStateChangeCallback(StateChangeCallback cb);
    void addFilelistChangedCallback(FilelistChangedCallback cb);
    void addHistoryChangedCallback(HistoryChangedCallback cb);
    void addJobQueueChangedCallback(JobQueueChangedCallback cb);
    void addPowerChangedCallback(PowerChangedCallback cb);

    // ------------------------------------------------------------------
    // Public data types (for transport layers)
    // ------------------------------------------------------------------

    struct JobHistoryEntry {
        int64_t jobId = 0;
        std::string filename;
        std::string status;
        double startTime = 0.0;
        double endTime = 0.0;
        double printDuration = 0.0;
        double totalDuration = 0.0;
        double filamentUsed = 0.0;
        int64_t layerCount = 0;
        int64_t firstLayerHeight = 0;
        int64_t firstLayerExtruder = 0;
    };

    struct User {
        std::string username;
        std::string password;
        std::vector<std::string> permissions;
        std::string source = "moonraker";
        std::string jwtSecret;
    };

    struct LogFile {
        std::string name;
        std::string path;
        int64_t size = 0;
        int64_t modified = 0;
    };

    // ------------------------------------------------------------------
    // Data accessors (for transport layers)
    // ------------------------------------------------------------------

    const std::vector<JobHistoryEntry>& jobHistory() const { return jobHistory_; }
    const std::map<std::string, User>& users() const { return users_; }
    bool deleteUser(const std::string& username);
    const std::vector<LogFile>& logFiles() const { return logFiles_; }

    const std::string& apiKey() const { return apiKey_; }
    void setApiKey(const std::string& key) { apiKey_ = key; }

    std::string generateOneshotToken();
    bool consumeOneshotToken(const std::string& token);

private:
    // ------------------------------------------------------------------
    // Helper templates
    // ------------------------------------------------------------------

    template<typename Func>
    JsonValue withLock(Func&& func) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return JsonValue(func());
    }

    template<typename T>
    JsonValue withLockResult(T&& value) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        std::map<std::string, JsonValue> result;
        result["result"] = JsonValue(std::forward<T>(value));
        return JsonValue(result);
    }

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
    // Internal helpers
    // ------------------------------------------------------------------

    /// @brief Notify filelist change callbacks (called outside mutex).
    void notifyFilelistChanged(const std::string& action,
                               const std::string& path,
                               const std::string& root);

    // ------------------------------------------------------------------
    // Member variables
    // ------------------------------------------------------------------

    UdsServerConfig config_;
    mutable std::recursive_mutex mutex_;

    // State
    PrinterState state_ = PrinterState::Startup;
    std::string stateMessage_ = "Printer is not ready";

    // Endpoints
    std::map<std::string, EndpointHandler> endpoints_;

    // Printer objects
    std::map<std::string, std::shared_ptr<PrinterObject>> objects_;

    // Callbacks (set by KlippyInstance)
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
    static constexpr size_t kMaxTempStoreEntries = 1200;

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
        std::string state;
        bool locked = false;
    };
    std::map<std::string, PowerDevice> powerDevices_;

    // Update status
    std::map<std::string, std::string> updateStatus_;

    // Database store (namespace -> key -> value)
    std::map<std::string, std::map<std::string, JsonValue>> database_;

    // Job queue
    std::vector<std::string> jobQueue_;
    bool jobQueuePaused_ = false;
    size_t jobQueueCurrentIndex_ = 0;

    // Job history
    std::vector<JobHistoryEntry> jobHistory_;
    int64_t nextJobId_ = 1;

    // Announcements
    struct Announcement {
        std::string entryId;
        std::string title;
        std::string description;
        std::string url;
        std::string date;
        std::string severity;
        bool dismissed = false;
    };
    std::vector<Announcement> announcements_;

    // Webcams
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

    // Services
    struct Service {
        std::string name;
        std::string activeState = "active";
        std::string subState = "running";
    };
    std::map<std::string, Service> services_;

    // File roots
    struct FileRoot {
        std::string name;
        std::string path;
        bool writable = true;
    };
    std::map<std::string, FileRoot> fileRoots_;

    // Server config (emulated Moonraker config)
    std::map<std::string, JsonValue> serverConfig_;

    // System permissions
    std::map<std::string, std::vector<std::string>> systemPerms_;

    // Access / user management
    std::map<std::string, User> users_;
    std::string apiKey_ = "tether_default_api_key";
    /// Oneshot tokens with creation timestamp for expiry (5 minute TTL)
    struct OneshotToken {
        std::string token;
        std::chrono::steady_clock::time_point createdAt;
    };
    std::vector<OneshotToken> oneshotTokens_;
    static constexpr int64_t kOneshotTokenTtlSeconds = 300; // 5 minutes

    // Bot management
    struct Bot {
        std::string name;
        std::string type;
        std::string token;
        std::string chatId;
        bool enabled = false;
    };
    std::map<std::string, Bot> bots_;

    // Notepad
    struct NotepadEntry {
        std::string key;
        std::string value;
        int64_t lastModified = 0;
    };
    std::map<std::string, NotepadEntry> notepad_;

    // Spoolman integration
    bool spoolmanConnected_ = false;
    std::string spoolmanUrl_;
    int64_t currentSpoolId_ = 0;
    std::map<std::string, JsonValue> spoolmanInfo_;

    // Log files
    std::vector<LogFile> logFiles_;

    // Event callbacks (for transport layers)
    std::vector<GcodeResponseCallback> gcodeResponseCbs_;
    std::vector<StateChangeCallback> stateChangeCbs_;
    std::vector<FilelistChangedCallback> filelistChangedCbs_;
    std::vector<HistoryChangedCallback> historyChangedCbs_;
    std::vector<JobQueueChangedCallback> jobQueueChangedCbs_;
    std::vector<PowerChangedCallback> powerChangedCbs_;
};

} // namespace tether::klipper::klippy

// Built-in printer objects that reference KlippyServer
#include "tether/klipper/klippy/PrinterObjectsBuiltin.hpp"
