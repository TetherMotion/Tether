#pragma once

/// @file NotificationBridge.hpp
/// @brief Bridge that receives notifications from KlippyUdsServer and fans
///        them out to WebSocket clients via WsSessionManager.

#include "tether/klipper/http/NotificationSink.hpp"
#include "tether/klipper/http/WsSessionManager.hpp"
#include "tether/klipper/http/ResponseBuilder.hpp"

#include <string>

namespace tether::klipper::http {

/// @brief Implementation of NotificationSink that bridges events to WebSocket sessions.
class NotificationBridge : public NotificationSink {
public:
    explicit NotificationBridge(WsSessionManager& sessions) : sessions_(sessions) {}

    void onStatusUpdate(const klippy::JsonValue& status, double eventtime) override {
        // Moonraker sends params as a JSON array: [status, eventtime]
        std::vector<klippy::JsonValue> params;
        params.push_back(status);
        params.push_back(klippy::JsonValue(eventtime));
        auto msg = buildJsonRpcNotification("notify_status_update",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onGcodeResponse(const std::string& response) override {
        std::map<std::string, klippy::JsonValue> params;
        params["response"] = klippy::JsonValue(response);
        auto msg = buildJsonRpcNotification("notify_gcode_response",
                                            klippy::JsonValue(params));
        auto gcodeSessions = sessions_.getGcodeSubscribedSessions();
        for (const auto& session : gcodeSessions) {
            if (session->conn) {
                session->conn->send(msg);
            }
        }
    }

    void onKlippyReady() override {
        auto msg = buildJsonRpcNotification("notify_klippy_ready",
                                            klippy::JsonValue(std::map<std::string, klippy::JsonValue>{}));
        sessions_.broadcastToIdentified(msg);
    }

    void onKlippyShutdown() override {
        auto msg = buildJsonRpcNotification("notify_klippy_shutdown",
                                            klippy::JsonValue(std::map<std::string, klippy::JsonValue>{}));
        sessions_.broadcastToIdentified(msg);
    }

    void onKlippyDisconnected() override {
        auto msg = buildJsonRpcNotification("notify_klippy_disconnected",
                                            klippy::JsonValue(std::map<std::string, klippy::JsonValue>{}));
        sessions_.broadcastToIdentified(msg);
    }

    void onFilelistChanged(const std::string& action,
                            const std::string& path,
                            const std::string& root) override {
        std::map<std::string, klippy::JsonValue> item;
        item["action"] = klippy::JsonValue(action);
        item["path"] = klippy::JsonValue(path);
        item["root"] = klippy::JsonValue(root);
        std::map<std::string, klippy::JsonValue> params;
        params["item"] = klippy::JsonValue(item);
        auto msg = buildJsonRpcNotification("notify_filelist_changed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onHistoryChanged(const std::string& action, int64_t jobId) override {
        std::map<std::string, klippy::JsonValue> params;
        params["action"] = klippy::JsonValue(action);
        params["job_id"] = klippy::JsonValue(jobId);
        auto msg = buildJsonRpcNotification("notify_history_changed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onJobQueueChanged(const std::string& action) override {
        std::map<std::string, klippy::JsonValue> info;
        info["updated_queue"] = klippy::JsonValue(true);
        info["queue_state"] = klippy::JsonValue("ready");
        std::map<std::string, klippy::JsonValue> params;
        params["action"] = klippy::JsonValue(action);
        params["info"] = klippy::JsonValue(info);
        auto msg = buildJsonRpcNotification("notify_job_queue_changed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onProcStatsUpdate(const klippy::JsonValue& stats) override {
        auto msg = buildJsonRpcNotification("notify_proc_stat_update", stats);
        sessions_.broadcastToIdentified(msg);
    }

    void onAnnouncementUpdate(const klippy::JsonValue& announcement) override {
        std::map<std::string, klippy::JsonValue> params;
        params["entry"] = announcement;
        auto msg = buildJsonRpcNotification("notify_announcement_update",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onAnnouncementDismissed(const std::string& entryId) override {
        std::map<std::string, klippy::JsonValue> params;
        params["entry_id"] = klippy::JsonValue(entryId);
        auto msg = buildJsonRpcNotification("notify_announcement_dismissed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onWebcamsChanged() override {
        auto msg = buildJsonRpcNotification("notify_webcams_changed",
                                            klippy::JsonValue(std::map<std::string, klippy::JsonValue>{}));
        sessions_.broadcastToIdentified(msg);
    }

    void onPowerChanged(const std::string& device, const std::string& state) override {
        std::map<std::string, klippy::JsonValue> device_info;
        device_info["device"] = klippy::JsonValue(device);
        device_info["status"] = klippy::JsonValue(state);
        auto msg = buildJsonRpcNotification("notify_power_changed",
                                            klippy::JsonValue(device_info));
        sessions_.broadcastToIdentified(msg);
    }

    void onUpdateResponse(const klippy::JsonValue& message) override {
        auto msg = buildJsonRpcNotification("notify_update_response", message);
        sessions_.broadcastToIdentified(msg);
    }

    void onUpdateRefreshed(const klippy::JsonValue& status) override {
        auto msg = buildJsonRpcNotification("notify_update_refreshed", status);
        sessions_.broadcastToIdentified(msg);
    }

    void onUserCreated(const std::string& username) override {
        std::map<std::string, klippy::JsonValue> params;
        params["username"] = klippy::JsonValue(username);
        auto msg = buildJsonRpcNotification("notify_user_created",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onUserDeleted(const std::string& username) override {
        std::map<std::string, klippy::JsonValue> params;
        params["username"] = klippy::JsonValue(username);
        auto msg = buildJsonRpcNotification("notify_user_deleted",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onUserLoggedOut(const std::string& username) override {
        std::map<std::string, klippy::JsonValue> params;
        params["username"] = klippy::JsonValue(username);
        auto msg = buildJsonRpcNotification("notify_user_logged_out",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onActiveSpoolSet(int64_t spoolId) override {
        std::map<std::string, klippy::JsonValue> params;
        params["spool_id"] = klippy::JsonValue(spoolId);
        auto msg = buildJsonRpcNotification("notify_active_spool_set",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onSpoolmanStatusChanged(bool connected) override {
        std::map<std::string, klippy::JsonValue> params;
        params["connected"] = klippy::JsonValue(connected);
        auto msg = buildJsonRpcNotification("notify_spoolman_status_changed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onCpuThrottled(int state) override {
        std::map<std::string, klippy::JsonValue> params;
        params["state"] = klippy::JsonValue(static_cast<int64_t>(state));
        auto msg = buildJsonRpcNotification("notify_cpu_throttled",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onServiceStateChanged(const std::string& service,
                                const std::string& state) override {
        std::map<std::string, klippy::JsonValue> params;
        params["service"] = klippy::JsonValue(service);
        params["active_state"] = klippy::JsonValue(state);
        auto msg = buildJsonRpcNotification("notify_service_state_changed",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

    void onMetadataUpdate(const std::string& filename) override {
        std::map<std::string, klippy::JsonValue> params;
        params["filename"] = klippy::JsonValue(filename);
        auto msg = buildJsonRpcNotification("notify_metadata_update",
                                            klippy::JsonValue(params));
        sessions_.broadcastToIdentified(msg);
    }

private:
    WsSessionManager& sessions_;
};

} // namespace tether::klipper::http
