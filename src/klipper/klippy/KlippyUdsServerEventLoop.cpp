/**
 * @file KlippyUdsServerEventLoop.cpp
 * @brief Event loop, connection processing, and subscription refresh
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "UdsConnection_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace tether::klipper::klippy {

// ============================================================================
// Event loop
// ============================================================================

void KlippyUdsServer::eventLoop() {
    while (running_.load()) {
        acceptConnection();
        processConnections();
        cleanupConnections();

        // Subscription refresh
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastRefresh_);
        if (elapsed.count() >= transportConfig_.refreshIntervalMs) {
            subscriptionRefreshTick();
            lastRefresh_ = now;
        }

        // Small sleep to avoid busy-looping
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void KlippyUdsServer::acceptConnection() {
    if (listenFd_ < 0) return;
    int fd = ::accept(listenFd_, nullptr, nullptr);
    if (fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(transportMutex_);
    auto conn = std::make_unique<UdsConnection>(fd, nextConnId_++);
    conn->setNonBlocking();
    connections_.push_back(std::move(conn));
}

void KlippyUdsServer::processConnections() {
    // Read frames from each connection without holding the lock
    struct ConnFrames {
        UdsConnection* conn;
        std::vector<std::string> frames;
    };
    std::vector<ConnFrames> allFrames;

    {
        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
        for (auto& conn : connections_) {
            if (conn->closed()) continue;
            auto frames = conn->readFrames();
            if (!frames.empty()) {
                allFrames.push_back({conn.get(), std::move(frames)});
            }
        }
    }

    // Process frames (handlers will lock as needed)
    for (auto& cf : allFrames) {
        for (const auto& frameStr : cf.frames) {
            auto parsed = JsonValue::parse(frameStr);
            if (!parsed) continue;

            // Handle special endpoints that need connection context
            if (parsed->has("method")) {
                std::string method = parsed->find("method")->asString();
                if (method == "register_remote_method") {
                    const JsonValue& params = parsed->has("params")
                        ? *parsed->find("params")
                        : JsonValue(std::map<std::string, JsonValue>{});
                    std::string methodName = params.has("remote_method")
                        ? params.find("remote_method")->asString() : "";
                    JsonValue tmpl = params.has("response_template")
                        ? *params.find("response_template")
                        : JsonValue(std::map<std::string, JsonValue>{});
                    if (!methodName.empty()) {
                        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
                        RemoteMethod reg{methodName, tmpl, cf.conn};
                        remoteMethods_[methodName].push_back(reg);
                    }
                    if (parsed->has("id")) {
                        sendResponse(*cf.conn, *parsed->find("id"),
                            JsonValue(std::map<std::string, JsonValue>{}));
                    }
                    continue;
                }
                if (method == "gcode/subscribe_output") {
                    {
                        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
                        gcodeSubscribers_.insert(cf.conn);
                    }
                    if (parsed->has("id")) {
                        sendResponse(*cf.conn, *parsed->find("id"),
                            JsonValue(std::map<std::string, JsonValue>{}));
                    }
                    continue;
                }
                if (method == "objects/subscribe") {
                    const JsonValue& params = parsed->has("params")
                        ? *parsed->find("params")
                        : JsonValue(std::map<std::string, JsonValue>{});
                    Subscription sub;
                    sub.conn = cf.conn;
                    sub.responseTemplate = params.has("response_template")
                        ? *params.find("response_template")
                        : JsonValue(std::map<std::string, JsonValue>{});
                    if (params.has("objects") && params.find("objects")->isObject()) {
                        for (const auto& [objName, fieldsVal] : params.find("objects")->asObject()) {
                            std::vector<std::string> fields;
                            if (fieldsVal.isNull()) {
                                // all fields
                            } else if (fieldsVal.isArray()) {
                                for (const auto& f : fieldsVal.asArray()) {
                                    if (f.isString()) fields.push_back(f.asString());
                                }
                            }
                            sub.objects[objName] = fields;
                        }
                    }
                    // Get initial snapshot from KlippyServer
                    auto status = server().queryObjects(sub.objects);
                    std::map<std::string, JsonValue> result;
                    std::map<std::string, JsonValue> statusJson;
                    for (const auto& [objName, fields] : status) {
                        std::map<std::string, JsonValue> fieldMap;
                        for (const auto& [f, v] : fields) {
                            fieldMap[f] = v;
                            sub.baseline[objName][f] = v;
                        }
                        statusJson[objName] = JsonValue(fieldMap);
                    }
                    result["status"] = JsonValue(statusJson);
                    result["eventtime"] = JsonValue(
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now().time_since_epoch()).count());
                    if (parsed->has("id")) {
                        sendResponse(*cf.conn, *parsed->find("id"), JsonValue(result));
                    }
                    {
                        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
                        subscriptions_.push_back(std::move(sub));
                    }
                    continue;
                }
            }
            processFrame(*cf.conn, *parsed);
        }
    }
}

void KlippyUdsServer::cleanupConnections() {
    std::lock_guard<std::recursive_mutex> lock(transportMutex_);
    // Remove closed connections
    connections_.erase(
        std::remove_if(connections_.begin(), connections_.end(),
            [](const std::unique_ptr<UdsConnection>& c) { return c->closed(); }),
        connections_.end());
    // Clean up subscriptions for closed connections
    subscriptions_.erase(
        std::remove_if(subscriptions_.begin(), subscriptions_.end(),
            [](const Subscription& s) { return !s.conn || s.conn->closed(); }),
        subscriptions_.end());
    // Clean up gcode subscribers
    for (auto it = gcodeSubscribers_.begin(); it != gcodeSubscribers_.end();) {
        if (!*it || (*it)->closed()) it = gcodeSubscribers_.erase(it);
        else ++it;
    }
    // Clean up remote methods
    for (auto it = remoteMethods_.begin(); it != remoteMethods_.end();) {
        auto& regs = it->second;
        regs.erase(std::remove_if(regs.begin(), regs.end(),
            [](const RemoteMethod& r) { return !r.conn || r.conn->closed(); }),
            regs.end());
        if (regs.empty()) it = remoteMethods_.erase(it);
        else ++it;
    }
}

void KlippyUdsServer::subscriptionRefreshTick() {
    // Copy subscription list under lock
    std::vector<Subscription*> activeSubs;
    {
        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
        for (auto& sub : subscriptions_) {
            if (sub.conn && !sub.conn->closed()) {
                activeSubs.push_back(&sub);
            }
        }
    }

    double eventtime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (auto* subPtr : activeSubs) {
        auto& sub = *subPtr;

        // Query current status from KlippyServer
        auto status = server().queryObjects(sub.objects);

        // Compute diff against baseline
        std::map<std::string, JsonValue> diff;
        for (const auto& [objName, fields] : status) {
            std::map<std::string, JsonValue> objDiff;
            for (const auto& [f, v] : fields) {
                auto baseIt = sub.baseline.find(objName);
                if (baseIt == sub.baseline.end() ||
                    baseIt->second.find(f) == baseIt->second.end() ||
                    !(baseIt->second.at(f).dump() == v.dump())) {
                    objDiff[f] = v;
                }
            }
            if (!objDiff.empty()) {
                diff[objName] = JsonValue(objDiff);
            }
        }

        if (diff.empty()) continue;

        // Build push message from template
        JsonValue msg = sub.responseTemplate;
        if (!msg.isObject()) msg = JsonValue(std::map<std::string, JsonValue>{});
        msg.asObject()["params"] = JsonValue(std::map<std::string, JsonValue>{
            {"status", JsonValue(diff)},
            {"eventtime", JsonValue(eventtime)}
        });

        {
            std::lock_guard<std::recursive_mutex> lock(transportMutex_);
            if (sub.conn && !sub.conn->closed()) {
                sub.conn->sendFrame(msg.dump());
            }
        }

        // Update baseline
        for (const auto& [objName, fields] : status) {
            for (const auto& [f, v] : fields) {
                sub.baseline[objName][f] = v;
            }
        }
    }
}

} // namespace tether::klipper::klippy
