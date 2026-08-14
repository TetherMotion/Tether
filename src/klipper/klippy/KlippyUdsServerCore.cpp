/**
 * @file KlippyUdsServerCore.cpp
 * @brief UDS transport: lifecycle, frame processing, and delegation to KlippyServer
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/KlipperLog.hpp"
#include "UdsConnection_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace tether::klipper::klippy {

// ============================================================================
// Construction / destruction
// ============================================================================

KlippyUdsServer::KlippyUdsServer(UdsServerConfig cfg)
    : transportConfig_(std::move(cfg)),
      ownedServer_(std::make_unique<KlippyServer>(transportConfig_)),
      ownsServer_(true) {
    registerUdsCallbacks();
}

KlippyUdsServer::KlippyUdsServer(KlippyServer& server, UdsServerConfig cfg)
    : transportConfig_(std::move(cfg)),
      externalServer_(&server),
      ownsServer_(false) {
    registerUdsCallbacks();
}

KlippyUdsServer::~KlippyUdsServer() {
    stop();
}

void KlippyUdsServer::registerUdsCallbacks() {
    auto& srv = server();
    // Forward gcode responses to UDS gcode subscribers
    srv.addGcodeResponseCallback([this](const std::string& response) {
        std::lock_guard<std::recursive_mutex> lock(transportMutex_);
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
    });
}

// ============================================================================
// Lifecycle
// ============================================================================

bool KlippyUdsServer::start() {
    const auto& cfg = transportConfig_;
    // Remove pre-existing socket file
    ::unlink(cfg.socketPath.c_str());

    // Create AF_UNIX SOCK_STREAM socket
    listenFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listenFd_ < 0) return false;

    // Bind
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, cfg.socketPath.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(listenFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    // Listen
    if (::listen(listenFd_, cfg.backlog) < 0) {
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

    std::lock_guard<std::recursive_mutex> lock(transportMutex_);
    connections_.clear();
    subscriptions_.clear();
    gcodeSubscribers_.clear();
    remoteMethods_.clear();

    if (listenFd_ >= 0) {
        ::close(listenFd_);
        listenFd_ = -1;
    }
    ::unlink(transportConfig_.socketPath.c_str());
}

// ============================================================================
// UDS-specific operations
// ============================================================================

void KlippyUdsServer::invokeRemoteMethod(const std::string& method,
                                          const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(transportMutex_);
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

size_t KlippyUdsServer::subscriptionCount() const {
    std::lock_guard<std::recursive_mutex> lock(transportMutex_);
    return subscriptions_.size();
}

void KlippyUdsServer::refreshSubscriptions() {
    subscriptionRefreshTick();
}

// ============================================================================
// Frame processing
// ============================================================================

void KlippyUdsServer::processFrame(UdsConnection& conn, const JsonValue& frame) {
    if (!frame.isObject()) return;

    bool hasId = frame.has("id");
    bool hasMethod = frame.has("method");

    if (hasMethod && hasId) {
        handleRequest(conn, frame);
    } else if (hasMethod && !hasId) {
        handleNotification(conn, frame);
    }
}

void KlippyUdsServer::handleRequest(UdsConnection& conn, const JsonValue& frame) {
    auto* idPtr = frame.find("id");
    auto* methodPtr = frame.find("method");
    if (!idPtr || !methodPtr) {
        sendError(conn, JsonValue{}, "Missing 'id' or 'method' field");
        return;
    }
    const JsonValue& id = *idPtr;
    std::string method = methodPtr->asString();
    JsonValue params = frame.has("params") ? *frame.find("params") : JsonValue(std::map<std::string, JsonValue>{});

    try {
        JsonValue result = server().callEndpoint(method, params);
        if (!result.isObject()) {
            result = JsonValue(std::map<std::string, JsonValue>{});
        }
        // Check if callEndpoint returned an error (endpoint not found)
        if (result.has("error") && result.find("error")->isString()) {
            sendError(conn, id, result.find("error")->asString());
        } else {
            sendResponse(conn, id, result);
        }
    } catch (const EndpointError& e) {
        sendError(conn, id, e.what());
    } catch (const std::exception& e) {
        sendError(conn, id, e.what());
        server().setState(PrinterState::Shutdown, std::string("Internal error: ") + e.what());
    }
}

void KlippyUdsServer::handleNotification(UdsConnection& conn, const JsonValue& frame) {
    // JSON-RPC notifications (requests without "id") are logged but not processed.
    // Moonraker/klippy don't typically send notifications to the UDS server.
    auto* methodPtr = frame.find("method");
    std::string method = (methodPtr && methodPtr->isString()) ? methodPtr->asString() : "unknown";
    KLIPPER_LOG_INFO("UDS notification received (ignored): " + method);
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
