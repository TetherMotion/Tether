#pragma once

/// @file NotificationSink.hpp
/// @brief Abstract interface for receiving push notifications from the server.
///
/// KlippyUdsServer generates events internally (status updates, gcode responses,
/// file changes, etc.). This interface allows external observers (like the
/// WebSocket bridge) to receive those events without being tied to the UDS
/// connection implementation.

#include "tether/klipper/klippy/JsonValue.hpp"

#include <string>

namespace tether::klipper::http {

/// @brief Abstract sink for server-to-client push notifications.
///
/// Implementations receive the same notification events that UDS connections
/// would receive. The WebSocket NotificationBridge implements this to fan out
/// events to connected WebSocket clients.
class NotificationSink {
public:
    virtual ~NotificationSink() = default;

    /// @brief Called when a status update notification should be sent.
    /// @param status The status diff (object name -> field -> value).
    /// @param eventtime The current event time.
    virtual void onStatusUpdate(const klippy::JsonValue& status, double eventtime) = 0;

    /// @brief Called when a G-code response notification should be sent.
    /// @param response The G-code response text.
    virtual void onGcodeResponse(const std::string& response) = 0;

    /// @brief Called when Klippy transitions to ready state.
    virtual void onKlippyReady() = 0;

    /// @brief Called when Klippy shuts down.
    virtual void onKlippyShutdown() = 0;

    /// @brief Called when the connection to Klippy is lost.
    virtual void onKlippyDisconnected() = 0;

    /// @brief Called when the file list changes.
    /// @param action The action (create_file, delete_file, move_file, etc.).
    /// @param path The affected path.
    /// @param root The file root.
    virtual void onFilelistChanged(const std::string& action,
                                    const std::string& path,
                                    const std::string& root) = 0;

    /// @brief Called when job history changes.
    /// @param action The action (added, deleted, etc.).
    /// @param jobId The affected job ID.
    virtual void onHistoryChanged(const std::string& action, int64_t jobId) = 0;

    /// @brief Called when the job queue changes.
    /// @param action The action (added, removed, updated, etc.).
    virtual void onJobQueueChanged(const std::string& action) = 0;

    /// @brief Called when proc stats are updated.
    /// @param stats The stats JSON object.
    virtual void onProcStatsUpdate(const klippy::JsonValue& stats) = 0;

    /// @brief Called when an announcement is updated.
    /// @param announcement The announcement data.
    virtual void onAnnouncementUpdate(const klippy::JsonValue& announcement) = 0;

    /// @brief Called when an announcement is dismissed.
    /// @param entryId The dismissed entry ID.
    virtual void onAnnouncementDismissed(const std::string& entryId) = 0;

    /// @brief Called when webcam configuration changes.
    virtual void onWebcamsChanged() = 0;

    /// @brief Called when a power device state changes.
    /// @param device The device name.
    /// @param state The new state ("on" or "off").
    virtual void onPowerChanged(const std::string& device, const std::string& state) = 0;

    /// @brief Called when the update manager has a response.
    /// @param message The update response message.
    virtual void onUpdateResponse(const klippy::JsonValue& message) = 0;

    /// @brief Called when the update status is refreshed.
    /// @param status The refreshed status.
    virtual void onUpdateRefreshed(const klippy::JsonValue& status) = 0;

    /// @brief Called when a user is created.
    virtual void onUserCreated(const std::string& username) = 0;

    /// @brief Called when a user is deleted.
    virtual void onUserDeleted(const std::string& username) = 0;

    /// @brief Called when a user is logged out.
    virtual void onUserLoggedOut(const std::string& username) = 0;

    /// @brief Called when the active spool is set.
    /// @param spoolId The new spool ID.
    virtual void onActiveSpoolSet(int64_t spoolId) = 0;

    /// @brief Called when Spoolman connection status changes.
    /// @param connected Whether Spoolman is connected.
    virtual void onSpoolmanStatusChanged(bool connected) = 0;

    /// @brief Called when CPU throttled state changes.
    /// @param state The throttled state.
    virtual void onCpuThrottled(int state) = 0;

    /// @brief Called when a service state changes.
    /// @param service The service name.
    /// @param state The new state.
    virtual void onServiceStateChanged(const std::string& service,
                                        const std::string& state) = 0;

    /// @brief Called when a metadata update occurs.
    /// @param filename The file whose metadata was updated.
    virtual void onMetadataUpdate(const std::string& filename) = 0;
};

} // namespace tether::klipper::http
