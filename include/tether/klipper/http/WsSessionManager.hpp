#pragma once


/**

 * @file WsSessionManager.hpp

 * @brief Manages WebSocket client sessions and their subscriptions.

 *

 * Each WebSocket connection is tracked as a WsSession with its own

 * object subscriptions, gcode-output subscription status, and client

 * identification info. The NotificationBridge fans out events to

 * sessions via this manager.

 */

#include "tether/klipper/klippy/JsonValue.hpp"
#include "tether/klipper/http/NotificationSink.hpp"

#include <drogon/WebSocketConnection.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace tether::klipper::http {

/**
 * @brief A single WebSocket client session.
 */
struct WsSession {
    int64_t id = 0;
    drogon::WebSocketConnectionPtr conn;
    std::string clientType; // "web", "mobile", "desktop", "display", "agent", "bot", "other"
    std::string clientName;
    std::string version;
    std::map<std::string, std::vector<std::string>> subscriptions; // object -> fields
    bool gcodeSubscribed = false;
    std::map<std::string, std::map<std::string, klippy::JsonValue>> baseline; // for diff
    bool identified = false;
};

/**
 * @brief Manages all active WebSocket sessions.
 *
 * Thread-safe. The WebSocket controller creates/removes sessions, and the
 * NotificationBridge iterates sessions to push notifications.
 */
class WsSessionManager {
public:
    /**
     * @brief Create a new session for a WebSocket connection.
     * @param conn Drogon connection the session wraps
     * @return The session ID.
     */
    int64_t createSession(const drogon::WebSocketConnectionPtr& conn) {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t id = nextId_.fetch_add(1);
        auto session = std::make_shared<WsSession>();
        session->id = id;
        session->conn = conn;
        sessions_[id] = session;
        return id;
    }

    /**
     * @brief Remove a session by ID.
     * @param id Session ID returned by createSession()
     */
    void removeSession(int64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_.erase(id);
    }

    /**
     * @brief Get a session by ID.
     * @param id Session ID to look up
     * @return Session pointer, or nullptr when not found
     */
    std::shared_ptr<WsSession> getSession(int64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it == sessions_.end()) return nullptr;
        return it->second;
    }

    /**
     * @brief Get all sessions (for fan-out).
     * @return Snapshot of every connected session
     */
    std::vector<std::shared_ptr<WsSession>> getAllSessions() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::shared_ptr<WsSession>> result;
        result.reserve(sessions_.size());
        for (const auto& [_, session] : sessions_) {
            result.push_back(session);
        }
        return result;
    }

    /**
     * @brief Get all sessions with object subscriptions.
     * @return Sessions that have at least one printer-object subscription
     */
    std::vector<std::shared_ptr<WsSession>> getSubscribedSessions() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::shared_ptr<WsSession>> result;
        for (const auto& [_, session] : sessions_) {
            if (!session->subscriptions.empty()) {
                result.push_back(session);
            }
        }
        return result;
    }

    /**
     * @brief Get all sessions subscribed to gcode output.
     * @return Sessions receiving gcode-response notifications
     */
    std::vector<std::shared_ptr<WsSession>> getGcodeSubscribedSessions() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::shared_ptr<WsSession>> result;
        for (const auto& [_, session] : sessions_) {
            if (session->gcodeSubscribed) {
                result.push_back(session);
            }
        }
        return result;
    }

    /**
     * @brief Set object subscriptions for a session.
     * @param id Session ID
     * @param subs Map of printer object -> field list to subscribe to;
     *        also clears the diffing baseline
     */
    void setSubscriptions(int64_t id,
                          const std::map<std::string, std::vector<std::string>>& subs) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end()) {
            it->second->subscriptions = subs;
            it->second->baseline.clear();
        }
    }

    /**
     * @brief Set gcode output subscription for a session.
     * @param id Session ID
     * @param subscribed `true` to receive gcode-response notifications
     */
    void setGcodeSubscribed(int64_t id, bool subscribed) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end()) {
            it->second->gcodeSubscribed = subscribed;
        }
    }

    /**
     * @brief Mark a session as identified.
     * @param id Session ID
     * @param clientType Client kind (e.g. "web", "agent")
     * @param clientName Client name reported by the client
     * @param version Client version string
     */
    void setIdentified(int64_t id, const std::string& clientType,
                       const std::string& clientName, const std::string& version) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end()) {
            it->second->identified = true;
            it->second->clientType = clientType;
            it->second->clientName = clientName;
            it->second->version = version;
        }
    }

    /**
     * @brief Update the baseline for a session (for subscription diffing).
     * @param id Session ID
     * @param objName Printer object the fields belong to
     * @param fields New field values merged into the baseline
     */
    void updateBaseline(int64_t id,
                        const std::string& objName,
                        const std::map<std::string, klippy::JsonValue>& fields) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end()) {
            for (const auto& [f, v] : fields) {
                it->second->baseline[objName][f] = v;
            }
        }
    }

    /**
     * @brief Get the baseline for a session.
     * @param id Session ID
     * @return Per-object field baseline; empty when unknown
     */
    std::map<std::string, std::map<std::string, klippy::JsonValue>>
    getBaseline(int64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it == sessions_.end()) return {};
        return it->second->baseline;
    }

    /**
     * @brief Get the subscriptions for a session.
     * @param id Session ID
     * @return Object -> subscribed field list; empty when unknown
     */
    std::map<std::string, std::vector<std::string>> getSubscriptions(int64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it == sessions_.end()) return {};
        return it->second->subscriptions;
    }

    /**
     * @brief Get the number of active sessions.
     * @return Connected session count
     */
    size_t sessionCount() {
        std::lock_guard<std::mutex> lock(mutex_);
        return sessions_.size();
    }

    /**
     * @brief Send a message to a specific session.
     * @param id Session ID (ignored when unknown or disconnected)
     * @param message JSON payload to send
     */
    void sendToSession(int64_t id, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end() && it->second->conn) {
            it->second->conn->send(message);
        }
    }

    /**
     * @brief Broadcast a message to all sessions.
     * @param message JSON payload sent to every connected session
     */
    void broadcast(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [_, session] : sessions_) {
            if (session->conn) {
                session->conn->send(message);
            }
        }
    }

    /**
     * @brief Broadcast a message to all identified sessions.
     * @param message JSON payload sent to sessions that completed
     *        the identify handshake
     */
    void broadcastToIdentified(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [_, session] : sessions_) {
            if (session->identified && session->conn) {
                session->conn->send(message);
            }
        }
    }

private:
    std::mutex mutex_;
    std::map<int64_t, std::shared_ptr<WsSession>> sessions_;
    std::atomic<int64_t> nextId_{1};
};

} // namespace tether::klipper::http
