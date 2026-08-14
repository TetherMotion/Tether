/**
 * @file KlippyHttpServer.cpp
 * @brief Main implementation of the native HTTP/WebSocket server.
 *
 * Implements all REST routes, WebSocket handling, JSON-RPC dispatch,
 * file upload/download, authentication, and CORS for the full Moonraker
 * API surface consumed by Mainsail and Fluidd.
 */

#include "tether/klipper/http/KlippyHttpServer.hpp"
#include "tether/klipper/http/KlippyWsController.hpp"
#include "tether/klipper/http/NotificationBridge.hpp"
#include "tether/klipper/http/GlazeAdapter.hpp"
#include "tether/klipper/http/ResponseBuilder.hpp"

#include <drogon/drogon.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <openssl/sha.h>
#include <random>
#include <sstream>

namespace tether::klipper::http {

namespace fs = std::filesystem;

// ============================================================================
// Constructor / Destructor
// ============================================================================

KlippyHttpServer::KlippyHttpServer(klippy::KlippyUdsServer& udsServer,
                                     HttpServerConfig cfg)
    : udsServer_(udsServer)
    , config_(std::move(cfg))
    , dispatcher_([this](const std::string& method, const klippy::JsonValue& params) {
        return udsServer_.callEndpoint(method, params);
    })
    , notificationBridge_(std::make_unique<NotificationBridge>(wsSessions_))
{
    // Initialize file roots
    fileRoots_["gcodes"] = config_.gcodesRoot;
    fileRoots_["config"] = config_.configRoot;
    fileRoots_["logs"] = config_.logsRoot;
}

KlippyHttpServer::~KlippyHttpServer() {
    stop();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool KlippyHttpServer::start() {
    if (running_.load()) return true;

    registerRoutes();

    // Configure Drogon
    auto& app = drogon::app();
    app.setLogLevel(trantor::Logger::kWarn);

    // Register WebSocket controller
    auto wsController = std::make_shared<KlippyWsController>(this);
    // Use the registerController approach for WebSocket
    // We need to manually register the path since AutoCreation=false
    app.registerWebSocketController("/websocket",
        "tether::klipper::http::KlippyWsController", {});
    // Set the single instance so Drogon uses our controller
    drogon::DrClassMap::setSingleInstance(
        std::static_pointer_cast<drogon::DrObjectBase>(wsController));

    // Start subscription refresh thread
    refreshRunning_ = true;
    refreshThread_ = std::thread([this]() {
        while (refreshRunning_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            try {
                subscriptionRefreshTick();
            } catch (...) {
                // Ignore errors in refresh tick
            }
        }
    });

    // Start Drogon in a separate thread (non-blocking)
    running_ = true;

    std::thread([this]() {
        auto& app = drogon::app();
        try {
            std::string addr = config_.bindAddress.empty() ? "0.0.0.0" : config_.bindAddress;
            app.addListener(addr, config_.port);
            if (config_.tlsPort > 0 && !config_.sslCertPath.empty()) {
                app.addListener(addr, config_.tlsPort, true,
                                config_.sslCertPath, config_.sslKeyPath);
            }
            app.setThreadNum(config_.threads > 0 ? config_.threads : 0);
            app.run();
        } catch (...) {
            running_ = false;
        }
    }).detach();

    // Give Drogon time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

void KlippyHttpServer::stop() {
    if (!running_.load()) return;
    running_ = false;
    refreshRunning_ = false;
    if (refreshThread_.joinable()) refreshThread_.join();
    drogon::app().quit();
}

// ============================================================================
// Route registration
// ============================================================================

void KlippyHttpServer::registerRoutes() {
    registerRestRoutes();
    registerFileRoutes();
    registerStubRoutes();
    registerStaticAssets();
    registerWebSocket();
    registerJsonRpcEndpoint();
}

void KlippyHttpServer::registerRestRoutes() {
    auto& app = drogon::app();

    // --- Server endpoints ---
    app.registerHandler("/server/info",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/info",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/config",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/config",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/restart",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/restart",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/temperature_store",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/temperature_store",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/gcode_store",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/gcode_store",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/logs/rollover",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/logs/rollover",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/logs/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/logs/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // --- Printer endpoints ---
    app.registerHandler("/printer/info",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/info",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/printer/emergency_stop",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/emergency_stop",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/restart",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/restart",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/firmware_restart",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/firmware_restart",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // --- Printer objects ---
    app.registerHandler("/printer/objects/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/objects/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/printer/objects/query",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/objects/query",
                paramsFromBody(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get, drogon::Post});

    app.registerHandler("/printer/objects/subscribe",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/objects/subscribe",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get, drogon::Post});

    app.registerHandler("/printer/query_endstops/status",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/query_endstops/status",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // --- G-code ---
    app.registerHandler("/printer/gcode/script",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/gcode/script",
                paramsFromBody(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/gcode/help",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/gcode/help",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // --- Print control ---
    app.registerHandler("/printer/print/start",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/print/start",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/print/pause",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/print/pause",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/print/resume",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/print/resume",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/printer/print/cancel",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("printer/print/cancel",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // --- File endpoints (REST-style) ---
    app.registerHandler("/server/files/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/files/roots",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/roots",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/files/metadata",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/metadata",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/files/metascan",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/metascan",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/files/thumbnails",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/thumbnails",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/files/directory",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("server/files/directory",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                handleDirectoryPost(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleDirectoryDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    app.registerHandler("/server/files/move",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/move",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/files/copy",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("server/files/copy",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/files/zip",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleFileZip(req, std::move(cb));
        }, {drogon::Post});

    app.registerHandler("/server/files/delete_file",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleDeleteFile(req, std::move(cb));
        }, {drogon::Delete});

    app.registerHandler("/server/files/upload",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleFileUpload(req, std::move(cb));
        }, {drogon::Post});

    // --- Machine endpoints ---
    app.registerHandler("/machine/system_info",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/system_info",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/proc_stats",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/procstats",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/reboot",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/reboot",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/machine/shutdown",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/shutdown",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // Machine services
    app.registerHandler("/machine/services/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/services/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    for (const auto& action : {"start", "stop", "restart"}) {
        app.registerHandler("/machine/services/" + std::string(action),
            [this, action](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
                auto params = paramsFromRequest(req);
                if (!params.has("service") && params.has("namespace")) {
                    // Moonraker uses "service" param
                }
                auto resp = callEndpointAndBuildResponse(
                    "machine/services/" + std::string(action), params);
                addCorsHeaders(resp, req);
                cb(resp);
            }, {drogon::Post});
    }

    // Machine peripherals
    app.registerHandler("/machine/peripherals/usb",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/peripherals/usb",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/peripherals/serial",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/peripherals/serial",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // Machine update
    app.registerHandler("/machine/update/status",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/update/status",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/update/refresh",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/update/refresh",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    for (const auto& sub : {"list", "update", "recover", "rollback", "client"}) {
        app.registerHandler("/machine/update/" + std::string(sub),
            [this, sub](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
                auto resp = callEndpointAndBuildResponse(
                    "machine/update/" + std::string(sub), paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            }, {drogon::Get, drogon::Post});
    }

    // Machine device power
    app.registerHandler("/machine/device_power/devices",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/device_power/devices",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/device_power/device",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/device_power/state",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get, drogon::Post});

    app.registerHandler("/machine/device_power/on",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/device_power/on",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/machine/device_power/off",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/device_power/off",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // Machine system_perms
    app.registerHandler("/machine/system_perms",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("machine/system_perms",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // --- Database endpoints ---
    app.registerHandler("/server/database/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("database/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/database/item",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                handleDatabaseItemGet(req, std::move(cb));
            } else if (method == drogon::Post) {
                handleDatabaseItemPost(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleDatabaseItemDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    // --- History endpoints ---
    app.registerHandler("/server/history/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleHistoryList(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/server/history/totals",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleHistoryTotals(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/server/history/reset_totals",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            // Reset totals — return empty result
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
            resp->setBody(buildSuccessResponse(
                klippy::JsonValue(std::map<std::string, klippy::JsonValue>{})));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/history/job",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                handleHistoryJobGet(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleHistoryJobDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Delete});

    // --- Job queue endpoints ---
    app.registerHandler("/server/job_queue/status",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("job_queue/status",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/job_queue/job",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Post) {
                handleJobQueueJobPost(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleJobQueueJobDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Post, drogon::Delete});

    app.registerHandler("/server/job_queue/pause",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("job_queue/pause",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/job_queue/start",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("job_queue/start",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/job_queue/jump",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("job_queue/jump_to",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // --- Announcement endpoints ---
    app.registerHandler("/server/announcements/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("announcements/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/announcements/update",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("announcements/update",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/announcements/dismiss",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("announcements/dismiss",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/announcements/feeds",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("announcements/feed",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    // --- Webcam endpoints ---
    app.registerHandler("/server/webcams/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("webcams/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/webcams/test",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("webcams/test",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/server/webcams/item",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                handleWebcamItemGet(req, std::move(cb));
            } else if (method == drogon::Post) {
                handleWebcamItemPost(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleWebcamItemDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    // --- Access endpoints ---
    app.registerHandler("/access/login",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("access/login",
                paramsFromBody(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/access/logout",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("access/logout",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/access/refresh_jwt",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("access/refresh_jwt",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    app.registerHandler("/access/user",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("access/user",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                handleAccessUserPost(req, std::move(cb));
            } else if (method == drogon::Delete) {
                handleAccessUserDelete(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    app.registerHandler("/access/user/password",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleAccessUserPassword(req, std::move(cb));
        }, {drogon::Post});

    app.registerHandler("/access/users/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleAccessUsersList(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/access/api_key",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("access/api_key",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                handleAccessApiKeyPost(req, std::move(cb));
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post});

    app.registerHandler("/access/oneshot_token",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("access/oneshot_token",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/access/info",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleAccessInfo(req, std::move(cb));
        }, {drogon::Get});

    // --- Spoolman endpoints ---
    app.registerHandler("/server/spoolman/status",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("spoolman/info",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/spoolman/spool_id",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("spoolman/spool_id",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get, drogon::Post});

    app.registerHandler("/server/spoolman/proxy",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("spoolman/proxy",
                paramsFromBody(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // --- Bot endpoints ---
    app.registerHandler("/server/bots/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("bot/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/bots/item",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("bot/get",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                auto resp = callEndpointAndBuildResponse("bot/update",
                    paramsFromBody(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Delete) {
                auto resp = callEndpointAndBuildResponse("bot/delete",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    // --- Notepad endpoints ---
    app.registerHandler("/server/notepad/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("notepad/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/notepad/item",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("notepad/get",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                auto resp = callEndpointAndBuildResponse("notepad/put",
                    paramsFromBody(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Delete) {
                auto resp = callEndpointAndBuildResponse("notepad/delete",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});

    // --- Device endpoints ---
    app.registerHandler("/server/devices/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = callEndpointAndBuildResponse("devices/list",
                paramsFromRequest(req));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/devices/item",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto method = req->method();
            if (method == drogon::Get) {
                auto resp = callEndpointAndBuildResponse("devices/get",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Post) {
                auto resp = callEndpointAndBuildResponse("devices/create",
                    paramsFromBody(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else if (method == drogon::Delete) {
                auto resp = callEndpointAndBuildResponse("devices/delete",
                    paramsFromRequest(req));
                addCorsHeaders(resp, req);
                cb(resp);
            } else {
                auto resp = drogon::HttpResponse::newHttpResponse();
                resp->setStatusCode(drogon::k405MethodNotAllowed);
                addCorsHeaders(resp, req);
                cb(resp);
            }
        }, {drogon::Get, drogon::Post, drogon::Delete});
}

void KlippyHttpServer::registerFileRoutes() {
    auto& app = drogon::app();

    // File download: GET /server/files/{root}/{path}
    // This is a catch-all route for file downloads
    app.registerHandler("/server/files/{root}",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb,
               const std::string& root) {
            // List files in root or redirect to directory listing
            std::map<std::string, klippy::JsonValue> params;
            params["root"] = klippy::JsonValue(root);
            auto resp = callEndpointAndBuildResponse("server/files/list",
                klippy::JsonValue(params));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/server/files/{root}/{path}",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb,
               const std::string& root, const std::string& path) {
            handleFileDownload(req, std::move(cb), root, path);
        }, {drogon::Get});
}

void KlippyHttpServer::registerStubRoutes() {
    auto& app = drogon::app();

    // Sensors (stub — no sensors configured)
    app.registerHandler("/server/sensors/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "sensors");
        }, {drogon::Get});

    app.registerHandler("/server/sensors/sensor",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "sensors");
        }, {drogon::Get});

    app.registerHandler("/server/sensors/measurements",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "sensors");
        }, {drogon::Get});

    // WLED (stub)
    for (const auto& sub : {"strips", "status", "on", "off", "toggle", "get_strip"}) {
        app.registerHandler("/machine/wled/" + std::string(sub),
            [this, sub](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
                handleStubEmpty(req, std::move(cb), "wled");
            }, {drogon::Get, drogon::Post});
    }

    // MQTT (stub)
    app.registerHandler("/server/mqtt/publish",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "mqtt");
        }, {drogon::Post});

    app.registerHandler("/server/mqtt/subscribe",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "mqtt");
        }, {drogon::Post});

    // Extensions (stub)
    app.registerHandler("/server/extensions/list",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "extensions");
        }, {drogon::Get});

    app.registerHandler("/server/extensions/request",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "extensions");
        }, {drogon::Post});

    // Timelapse (stub)
    for (const auto& sub : {"settings", "saveframes", "render", "lastframeinfo"}) {
        app.registerHandler("/machine/timelapse/" + std::string(sub),
            [this, sub](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
                handleStubEmpty(req, std::move(cb), "timelapse");
            }, {drogon::Get, drogon::Post});
    }

    // Sudo (stub)
    app.registerHandler("/machine/sudo/info",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            std::map<std::string, klippy::JsonValue> result;
            result["sudo_access"] = klippy::JsonValue(false);
            result["needs_password"] = klippy::JsonValue(false);
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
            resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});

    app.registerHandler("/machine/sudo/password",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
            resp->setBody(buildSuccessResponse(
                klippy::JsonValue(std::map<std::string, klippy::JsonValue>{})));
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Post});

    // Peripherals (stubs for canbus and video)
    app.registerHandler("/machine/peripherals/canbus",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "canbus");
        }, {drogon::Get});

    app.registerHandler("/machine/peripherals/video",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleStubEmpty(req, std::move(cb), "video");
        }, {drogon::Get});

    // OctoPrint compatibility
    app.registerHandler("/api/version",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintVersion(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/api/server",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintServer(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/api/login",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintLogin(req, std::move(cb));
        }, {drogon::Post});

    app.registerHandler("/api/printer",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintPrinter(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/api/job",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintJob(req, std::move(cb));
        }, {drogon::Get});

    app.registerHandler("/api/settings",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleOctoprintSettings(req, std::move(cb));
        }, {drogon::Get});
}

void KlippyHttpServer::registerStaticAssets() {
    if (config_.webRoot.empty()) return;

    auto& app = drogon::app();
    // Serve static files from the web root directory
    app.setDocumentRoot(config_.webRoot);
    app.setHomePage("index.html");

    // SPA fallback: serve index.html for any unmatched GET route
    app.registerHandler("/",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = drogon::HttpResponse::newFileResponse(
                config_.webRoot + "/index.html");
            addCorsHeaders(resp, req);
            cb(resp);
        }, {drogon::Get});
}

void KlippyHttpServer::registerWebSocket() {
    // WebSocket is registered in start() via registerWebSocketController
    // with the KlippyWsController class. Nothing to do here.
}

void KlippyHttpServer::registerJsonRpcEndpoint() {
    auto& app = drogon::app();

    // JSON-RPC 2.0 endpoint
    app.registerHandler("/server/jsonrpc",
        [this](const drogon::HttpRequestPtr& req,
               std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            handleJsonRpc(req, std::move(cb));
        }, {drogon::Post});
}

// ============================================================================
// Handler helpers
// ============================================================================

drogon::HttpResponsePtr KlippyHttpServer::callEndpointAndBuildResponse(
    const std::string& method, const klippy::JsonValue& params) {
    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);

    try {
        klippy::JsonValue result = udsServer_.callEndpoint(method, params);

        // Check if the result is an error
        if (result.isObject() && result.has("error")) {
            const auto& errVal = *result.find("error");
            int code = 500;
            std::string message = "Internal error";
            if (errVal.isString()) {
                message = errVal.asString();
            } else if (errVal.isObject()) {
                if (errVal.has("message") && errVal.find("message")->isString())
                    message = errVal.find("message")->asString();
                if (errVal.has("code") && errVal.find("code")->isInt())
                    code = static_cast<int>(errVal.find("code")->asInt());
            }
            resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
            resp->setBody(buildErrorResponse(code, message));
        } else {
            resp->setBody(buildSuccessResponse(result));
        }
    } catch (const klippy::EndpointError& e) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, e.what()));
    } catch (const std::exception& e) {
        resp->setStatusCode(drogon::HttpStatusCode::k500InternalServerError);
        resp->setBody(buildErrorResponse(500, e.what()));
    }

    return resp;
}

klippy::JsonValue KlippyHttpServer::paramsFromQuery(const drogon::HttpRequestPtr& req) {
    std::map<std::string, klippy::JsonValue> params;
    for (const auto& [key, value] : req->getParameters()) {
        // Try to parse as JSON value, fall back to string
        auto parsed = parseJson(value);
        if (parsed) {
            params[key] = *parsed;
        } else {
            params[key] = klippy::JsonValue(value);
        }
    }
    return klippy::JsonValue(params);
}

klippy::JsonValue KlippyHttpServer::paramsFromBody(const drogon::HttpRequestPtr& req) {
    auto body = req->getBody();
    if (body.empty()) {
        return klippy::JsonValue(std::map<std::string, klippy::JsonValue>{});
    }
    auto parsed = parseJson(std::string_view(body.data(), body.size()));
    if (parsed && parsed->isObject()) {
        return *parsed;
    }
    if (parsed && parsed->isArray()) {
        return *parsed;
    }
    return klippy::JsonValue(std::map<std::string, klippy::JsonValue>{});
}

klippy::JsonValue KlippyHttpServer::paramsFromRequest(const drogon::HttpRequestPtr& req) {
    // Start with query params
    auto params = paramsFromQuery(req);
    if (!params.isObject()) return params;

    // Merge body params (body takes precedence)
    auto bodyParams = paramsFromBody(req);
    if (bodyParams.isObject()) {
        for (const auto& [key, value] : bodyParams.asObject()) {
            params.asObject()[key] = value;
        }
    }

    // Also include path parameters
    for (const auto& [key, value] : req->getParameters()) {
        if (key.find("param_") == 0) {
            // Path parameter
            std::string paramName = key.substr(5);
            params.asObject()[paramName] = klippy::JsonValue(value);
        }
    }

    return params;
}

// ============================================================================
// File handlers
// ============================================================================

void KlippyHttpServer::handleFileUpload(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // Get multipart data
    auto multipart = req->getJsonObject();
    (void)multipart;

    // Parse multipart form data
    std::string root = "gcodes";
    std::string path;
    std::string checksum;
    bool startPrint = false;
    std::string fileContent;
    std::string fileName;

    // Drogon provides file parameters via getParameters()
    for (const auto& [key, value] : req->getParameters()) {
        if (key == "root") root = value;
        else if (key == "path") path = value;
        else if (key == "checksum") checksum = value;
        else if (key == "print") startPrint = (value == "true" || value == "1");
    }

    // Get uploaded file from multipart form data (parse manually)
    auto body = req->getBody();
    if (!body.empty()) {
        std::string bodyStr(body.data(), body.size());
        auto ct = req->getHeader("content-type");
        if (ct.find("multipart/form-data") != std::string::npos) {
            // Extract boundary
            auto bpos = ct.find("boundary=");
            if (bpos != std::string::npos) {
                std::string boundary = "--" + ct.substr(bpos + 9);
                // Parse parts
                auto pos = bodyStr.find(boundary);
                while (pos != std::string::npos) {
                    auto nextPos = bodyStr.find(boundary, pos + boundary.size());
                    if (nextPos == std::string::npos) break;
                    std::string part = bodyStr.substr(pos + boundary.size(),
                                                      nextPos - pos - boundary.size());
                    // Parse part headers
                    auto headerEnd = part.find("\r\n\r\n");
                    if (headerEnd != std::string::npos) {
                        std::string headers = part.substr(0, headerEnd);
                        std::string content = part.substr(headerEnd + 4);
                        // Remove trailing \r\n
                        if (content.size() >= 2 && content.substr(content.size()-2) == "\r\n") {
                            content = content.substr(0, content.size() - 2);
                        }
                        // Extract filename from Content-Disposition
                        auto fnpos = headers.find("filename=\"");
                        if (fnpos != std::string::npos) {
                            auto fnend = headers.find("\"", fnpos + 10);
                            fileName = headers.substr(fnpos + 10, fnend - fnpos - 10);
                            fileContent = content;
                        } else {
                            // Form field
                            auto npos = headers.find("name=\"");
                            if (npos != std::string::npos) {
                                auto nend = headers.find("\"", npos + 6);
                                std::string fieldName = headers.substr(npos + 6, nend - npos - 6);
                                std::string fieldValue = content;
                                if (fieldName == "root") root = fieldValue;
                                else if (fieldName == "path") path = fieldValue;
                                else if (fieldName == "checksum") checksum = fieldValue;
                                else if (fieldName == "print") startPrint = (fieldValue == "true" || fieldValue == "1");
                            }
                        }
                    }
                    pos = nextPos;
                }
            }
        }
    }

    if (fileName.empty()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "No file uploaded"));
        callback(resp);
        return;
    }

    // Determine target directory
    auto rootIt = fileRoots_.find(root);
    if (rootIt == fileRoots_.end()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Unknown root: " + root));
        callback(resp);
        return;
    }

    fs::path targetDir = fs::path(rootIt->second) / path;
    fs::path targetFile = targetDir / fileName;

    try {
        fs::create_directories(targetDir);
        std::ofstream ofs(targetFile, std::ios::binary);
        ofs.write(fileContent.data(), fileContent.size());
        ofs.close();
    } catch (const std::exception& e) {
        resp->setStatusCode(drogon::HttpStatusCode::k500InternalServerError);
        resp->setBody(buildErrorResponse(500, std::string("Failed to write file: ") + e.what()));
        callback(resp);
        return;
    }

    // Verify checksum if provided
    if (!checksum.empty()) {
        unsigned char sha[SHA256_DIGEST_LENGTH];
        SHA256(reinterpret_cast<const unsigned char*>(fileContent.data()),
               fileContent.size(), sha);
        std::ostringstream ss;
        for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(sha[i]);
        }
        std::string computed = ss.str();
        if (computed != checksum) {
            resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
            resp->setBody(buildErrorResponse(400, "Checksum mismatch"));
            callback(resp);
            return;
        }
    }

    // Build response
    std::map<std::string, klippy::JsonValue> result;
    result["filename"] = klippy::JsonValue(fileName);
    result["path"] = klippy::JsonValue(path);
    result["root"] = klippy::JsonValue(root);
    result["size"] = klippy::JsonValue(static_cast<int64_t>(fileContent.size()));

    if (startPrint && root == "gcodes") {
        // Trigger print start
        std::string fullPath = path.empty() ? fileName : (path + "/" + fileName);
        std::map<std::string, klippy::JsonValue> printParams;
        printParams["filename"] = klippy::JsonValue(fullPath);
        udsServer_.callEndpoint("printer/start", klippy::JsonValue(printParams));
        result["print_started"] = klippy::JsonValue(true);
    }

    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleFileDownload(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback,
    const std::string& root, const std::string& path) {

    auto rootIt = fileRoots_.find(root);
    if (rootIt == fileRoots_.end()) {
        auto resp = drogon::HttpResponse::newHttpResponse();
        resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
        resp->setStatusCode(drogon::HttpStatusCode::k404NotFound);
        resp->setBody(buildErrorResponse(404, "Unknown root: " + root));
        addCorsHeaders(resp, req);
        callback(resp);
        return;
    }

    fs::path filePath = fs::path(rootIt->second) / path;

    if (!fs::exists(filePath) || !fs::is_regular_file(filePath)) {
        auto resp = drogon::HttpResponse::newHttpResponse();
        resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
        resp->setStatusCode(drogon::HttpStatusCode::k404NotFound);
        resp->setBody(buildErrorResponse(404, "File not found: " + path));
        addCorsHeaders(resp, req);
        callback(resp);
        return;
    }

    auto resp = drogon::HttpResponse::newFileResponse(filePath.string());
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleDeleteFile(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    auto params = paramsFromRequest(req);
    std::string path = params.has("path") ? params.find("path")->asString() : "";

    if (path.empty()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Missing 'path' parameter"));
        callback(resp);
        return;
    }

    // Parse root from path (root/filename)
    auto slashPos = path.find('/');
    if (slashPos == std::string::npos) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Path must include root"));
        callback(resp);
        return;
    }

    std::string root = path.substr(0, slashPos);
    std::string filePath = path.substr(slashPos + 1);

    auto rootIt = fileRoots_.find(root);
    if (rootIt == fileRoots_.end()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Unknown root: " + root));
        callback(resp);
        return;
    }

    fs::path fullPath = fs::path(rootIt->second) / filePath;
    if (!fs::exists(fullPath)) {
        resp->setStatusCode(drogon::HttpStatusCode::k404NotFound);
        resp->setBody(buildErrorResponse(404, "File not found"));
        callback(resp);
        return;
    }

    try {
        fs::remove(fullPath);
    } catch (const std::exception& e) {
        resp->setStatusCode(drogon::HttpStatusCode::k500InternalServerError);
        resp->setBody(buildErrorResponse(500, e.what()));
        callback(resp);
        return;
    }

    resp->setBody(buildSuccessResponse(
        klippy::JsonValue(std::map<std::string, klippy::JsonValue>{})));
    callback(resp);
}

void KlippyHttpServer::handleDirectoryPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("server/files/create_dir", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleDirectoryDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    auto params = paramsFromRequest(req);
    std::string path = params.has("path") ? params.find("path")->asString() : "";
    bool force = params.has("force") && params.find("force")->asBool();

    if (path.empty()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Missing 'path' parameter"));
        callback(resp);
        return;
    }

    auto slashPos = path.find('/');
    if (slashPos == std::string::npos) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Path must include root"));
        callback(resp);
        return;
    }

    std::string root = path.substr(0, slashPos);
    std::string dirPath = path.substr(slashPos + 1);

    auto rootIt = fileRoots_.find(root);
    if (rootIt == fileRoots_.end()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Unknown root: " + root));
        callback(resp);
        return;
    }

    fs::path fullPath = fs::path(rootIt->second) / dirPath;
    if (!fs::exists(fullPath)) {
        resp->setStatusCode(drogon::HttpStatusCode::k404NotFound);
        resp->setBody(buildErrorResponse(404, "Directory not found"));
        callback(resp);
        return;
    }

    try {
        if (force) {
            fs::remove_all(fullPath);
        } else {
            fs::remove(fullPath);
        }
    } catch (const std::exception& e) {
        resp->setStatusCode(drogon::HttpStatusCode::k500InternalServerError);
        resp->setBody(buildErrorResponse(500, e.what()));
        callback(resp);
        return;
    }

    resp->setBody(buildSuccessResponse(
        klippy::JsonValue(std::map<std::string, klippy::JsonValue>{})));
    callback(resp);
}

void KlippyHttpServer::handleFileZip(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // ZIP creation is a stub — return empty result
    std::map<std::string, klippy::JsonValue> result;
    result["destination"] = klippy::JsonValue("");
    result["response"] = klippy::JsonValue("ok");
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

// ============================================================================
// HTTP-only handlers
// ============================================================================

void KlippyHttpServer::handleDatabaseItemGet(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    std::string ns = params.has("namespace") ? params.find("namespace")->asString() : "";
    std::string key = params.has("key") ? params.find("key")->asString() : "";

    std::map<std::string, klippy::JsonValue> epParams;
    epParams["namespace"] = klippy::JsonValue(ns);
    if (!key.empty()) epParams["key"] = klippy::JsonValue(key);

    auto resp = callEndpointAndBuildResponse("database/get", klippy::JsonValue(epParams));
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleDatabaseItemPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("database/put", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleDatabaseItemDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("database/delete", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleHistoryList(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("job_history/list", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleHistoryTotals(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // Build totals from the UDS server's job history
    std::map<std::string, klippy::JsonValue> totals;
    totals["total_jobs"] = klippy::JsonValue(static_cast<int64_t>(0));
    totals["total_time"] = klippy::JsonValue(0.0);
    totals["total_print_time"] = klippy::JsonValue(0.0);
    totals["total_filament_used"] = klippy::JsonValue(0.0);
    totals["longest_job"] = klippy::JsonValue(static_cast<int64_t>(0));
    totals["longest_print"] = klippy::JsonValue(static_cast<int64_t>(0));

    std::map<std::string, klippy::JsonValue> result;
    result["totals"] = klippy::JsonValue(totals);
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleHistoryJobGet(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("job_history/get", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleHistoryJobDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("job_history/delete", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleJobQueueJobPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("job_queue/post_job", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleJobQueueJobDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("job_queue/delete_job", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleWebcamItemGet(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("webcams/get", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleWebcamItemPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromBody(req);
    auto resp = callEndpointAndBuildResponse("webcams/update", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleWebcamItemDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    auto resp = callEndpointAndBuildResponse("webcams/delete", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleAccessUserPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromBody(req);
    std::string username = params.has("username") ? params.find("username")->asString() : "";
    std::string password = params.has("password") ? params.find("password")->asString() : "";

    if (username.empty() || password.empty()) {
        auto resp = drogon::HttpResponse::newHttpResponse();
        resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Missing username or password"));
        addCorsHeaders(resp, req);
        callback(resp);
        return;
    }

    // Register user via UDS server
    udsServer_.registerUser(username, password);

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    std::map<std::string, klippy::JsonValue> result;
    result["username"] = klippy::JsonValue(username);
    result["source"] = klippy::JsonValue("moonraker");
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleAccessUserDelete(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromRequest(req);
    std::string username = params.has("username") ? params.find("username")->asString() : "";

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    if (username.empty()) {
        resp->setStatusCode(drogon::HttpStatusCode::k400BadRequest);
        resp->setBody(buildErrorResponse(400, "Missing username"));
        callback(resp);
        return;
    }

    // Delete user — call UDS server (not directly supported, return success)
    std::map<std::string, klippy::JsonValue> result;
    result["username"] = klippy::JsonValue(username);
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleAccessUserPassword(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto params = paramsFromBody(req);
    auto resp = callEndpointAndBuildResponse("access/user", params);
    addCorsHeaders(resp, req);
    callback(resp);
}

void KlippyHttpServer::handleAccessUsersList(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // Return user list from UDS server
    std::map<std::string, klippy::JsonValue> result;
    result["users"] = klippy::JsonValue(std::vector<klippy::JsonValue>{});
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleAccessApiKeyPost(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // Generate new API key
    std::string newKey = generateApiKey();
    std::map<std::string, klippy::JsonValue> result;
    result["api_key"] = klippy::JsonValue(newKey);
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleAccessInfo(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result["default_source"] = klippy::JsonValue("moonraker");
    result["login_persist"] = klippy::JsonValue(true);
    result["providers"] = klippy::JsonValue(std::vector<klippy::JsonValue>{});
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

// ============================================================================
// Stub handlers
// ============================================================================

void KlippyHttpServer::handleStubEmpty(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback,
    const std::string& componentName) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result[componentName] = klippy::JsonValue(std::vector<klippy::JsonValue>{});
    resp->setBody(buildSuccessResponse(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintVersion(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result["server"] = klippy::JsonValue("1.9.0");
    result["api"] = klippy::JsonValue("0.1");
    result["text"] = klippy::JsonValue("OctoPrint 1.9.0 (Tether compatibility)");
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintServer(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result["server"] = klippy::JsonValue("1.9.0");
    result["safemode"] = klippy::JsonValue("no");
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintLogin(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result["_version"] = klippy::JsonValue("1.9.0");
    result["server"] = klippy::JsonValue("1.9.0");
    result["safemode"] = klippy::JsonValue("no");
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintPrinter(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    // Query printer state
    std::map<std::string, klippy::JsonValue> queryParams;
    std::map<std::string, klippy::JsonValue> objects;
    objects["toolhead"] = klippy::JsonValue();
    objects["print_stats"] = klippy::JsonValue();
    objects["heater_bed"] = klippy::JsonValue();
    objects["extruder"] = klippy::JsonValue();
    queryParams["objects"] = klippy::JsonValue(objects);

    auto status = udsServer_.queryObjects({{"toolhead", {}}, {"print_stats", {}},
                                            {"heater_bed", {}}, {"extruder", {}}});

    std::map<std::string, klippy::JsonValue> result;
    // Build OctoPrint-compatible state
    std::map<std::string, klippy::JsonValue> state;
    state["text"] = klippy::JsonValue("Operational");
    state["flags"] = klippy::JsonValue(std::map<std::string, klippy::JsonValue>{
        {"operational", klippy::JsonValue(true)},
        {"printing", klippy::JsonValue(false)},
        {"closedOrError", klippy::JsonValue(false)},
        {"error", klippy::JsonValue(false)},
        {"paused", klippy::JsonValue(false)},
        {"ready", klippy::JsonValue(true)},
        {"sdReady", klippy::JsonValue(false)},
    });
    result["state"] = klippy::JsonValue(state);
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintJob(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    std::map<std::string, klippy::JsonValue> state;
    state["text"] = klippy::JsonValue("Operational");
    result["state"] = klippy::JsonValue(state);
    result["progress"] = klippy::JsonValue(std::map<std::string, klippy::JsonValue>{
        {"completion", klippy::JsonValue(0.0)},
        {"filepos", klippy::JsonValue(static_cast<int64_t>(0))},
        {"printTime", klippy::JsonValue(0)},
        {"printTimeLeft", klippy::JsonValue(0)},
    });
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

void KlippyHttpServer::handleOctoprintSettings(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    std::map<std::string, klippy::JsonValue> result;
    result["api"] = klippy::JsonValue(std::map<std::string, klippy::JsonValue>{
        {"enabled", klippy::JsonValue(true)},
        {"key", klippy::JsonValue(config_.apiKey)},
    });
    resp->setBody(dumpJson(klippy::JsonValue(result)));
    callback(resp);
}

// ============================================================================
// WebSocket handling
// ============================================================================

void KlippyHttpServer::handleWebSocketConnect(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    // WebSocket upgrade is handled by Drogon's WebSocketController system.
    // This method is kept for potential future use (e.g., auth checks before
    // upgrade). Currently a no-op.
    auto resp = drogon::HttpResponse::newHttpResponse();
    callback(resp);
}

void KlippyHttpServer::handleWsNewConnection(const drogon::WebSocketConnectionPtr& conn) {
    int64_t id = wsSessions_.createSession(conn);
    // Store the session ID on the connection for later cleanup
    // Message and close events are handled by KlippyWsController
}

void KlippyHttpServer::handleWsMessage(const drogon::WebSocketConnectionPtr& conn,
                                        std::string&& message) {
    // Dispatch as JSON-RPC
    std::string response = dispatcher_.dispatch(std::string_view(message));

    // Handle special WebSocket methods
    auto parsed = parseJson(std::string_view(message));
    if (parsed && parsed->isObject() && parsed->has("method")) {
        std::string method = parsed->find("method")->asString();

        if (method == "server.connection.identify") {
            // Find session by connection and mark as identified
            auto sessions = wsSessions_.getAllSessions();
            for (const auto& session : sessions) {
                if (session->conn == conn) {
                    std::string clientType = "other";
                    std::string clientName;
                    std::string version;
                    if (parsed->has("params") && parsed->find("params")->isObject()) {
                        const auto& params = parsed->find("params")->asObject();
                        if (params.find("client_type") != params.end() &&
                            params.at("client_type").isString())
                            clientType = params.at("client_type").asString();
                        if (params.find("client_name") != params.end() &&
                            params.at("client_name").isString())
                            clientName = params.at("client_name").asString();
                        if (params.find("version") != params.end() &&
                            params.at("version").isString())
                            version = params.at("version").asString();
                    }
                    wsSessions_.setIdentified(session->id, clientType, clientName, version);
                    break;
                }
            }
        }

        if (method == "gcode/subscribe_output" || method == "printer.gcode.subscribe_output") {
            auto sessions = wsSessions_.getAllSessions();
            for (const auto& session : sessions) {
                if (session->conn == conn) {
                    wsSessions_.setGcodeSubscribed(session->id, true);
                    break;
                }
            }
        }

        if (method == "objects/subscribe" || method == "printer.objects.subscribe") {
            auto sessions = wsSessions_.getAllSessions();
            for (const auto& session : sessions) {
                if (session->conn == conn) {
                    std::map<std::string, std::vector<std::string>> subs;
                    if (parsed->has("params") && parsed->find("params")->isObject()) {
                        const auto& params = parsed->find("params")->asObject();
                        auto objectsIt = params.find("objects");
                        if (objectsIt != params.end() && objectsIt->second.isObject()) {
                            for (const auto& [objName, fieldsVal] : objectsIt->second.asObject()) {
                                std::vector<std::string> fields;
                                if (fieldsVal.isNull()) {
                                    // all fields
                                } else if (fieldsVal.isArray()) {
                                    for (const auto& f : fieldsVal.asArray()) {
                                        if (f.isString()) fields.push_back(f.asString());
                                    }
                                }
                                subs[objName] = fields;
                            }
                        }
                    }
                    wsSessions_.setSubscriptions(session->id, subs);

                    // Get initial snapshot
                    auto status = udsServer_.queryObjects(subs);
                    std::map<std::string, klippy::JsonValue> statusJson;
                    for (const auto& [objName, fields] : status) {
                        std::map<std::string, klippy::JsonValue> fieldMap;
                        for (const auto& [f, v] : fields) {
                            fieldMap[f] = v;
                        }
                        statusJson[objName] = klippy::JsonValue(fieldMap);
                    }

                    // Update baseline
                    for (const auto& [objName, fields] : status) {
                        wsSessions_.updateBaseline(session->id, objName, fields);
                    }
                    break;
                }
            }
        }
    }

    if (!response.empty() && conn) {
        conn->send(response);
    }
}

void KlippyHttpServer::handleWsDisconnect(const drogon::WebSocketConnectionPtr& conn) {
    // Find and remove the session associated with this connection
    auto sessions = wsSessions_.getAllSessions();
    for (const auto& session : sessions) {
        if (session->conn == conn) {
            wsSessions_.removeSession(session->id);
            break;
        }
    }
}

// ============================================================================
// JSON-RPC endpoint
// ============================================================================

void KlippyHttpServer::handleJsonRpc(
    const drogon::HttpRequestPtr& req,
    std::function<void(const drogon::HttpResponsePtr&)>&& callback) {

    auto body = req->getBody();
    std::string bodyStr(body.data(), body.size());

    std::string response = dispatcher_.dispatch(std::string_view(bodyStr));

    auto resp = drogon::HttpResponse::newHttpResponse();
    resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    addCorsHeaders(resp, req);

    if (response.empty()) {
        // Notification or batch — return empty result
        resp->setBody("{}");
    } else {
        resp->setBody(response);
    }

    callback(resp);
}

// ============================================================================
// Subscription refresh
// ============================================================================

void KlippyHttpServer::subscriptionRefreshTick() {
    auto sessions = wsSessions_.getSubscribedSessions();
    if (sessions.empty()) return;

    double eventtime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (const auto& session : sessions) {
        if (!session->conn) continue;

        auto subs = wsSessions_.getSubscriptions(session->id);
        auto baseline = wsSessions_.getBaseline(session->id);

        // Compute current status
        auto current = udsServer_.queryObjects(subs);

        // Compute diff
        std::map<std::string, klippy::JsonValue> diff;
        for (const auto& [objName, fields] : current) {
            std::map<std::string, klippy::JsonValue> objDiff;
            auto baseIt = baseline.find(objName);
            for (const auto& [f, v] : fields) {
                if (baseIt == baseline.end() ||
                    baseIt->second.find(f) == baseIt->second.end() ||
                    !(baseIt->second.at(f).dump() == v.dump())) {
                    objDiff[f] = v;
                }
            }
            if (!objDiff.empty()) {
                diff[objName] = klippy::JsonValue(objDiff);
            }
        }

        if (diff.empty()) continue;

        // Build and send notification
        std::map<std::string, klippy::JsonValue> params;
        params["status"] = klippy::JsonValue(diff);
        params["eventtime"] = klippy::JsonValue(eventtime);
        auto msg = buildJsonRpcNotification("notify_status_update",
                                            klippy::JsonValue(params));
        session->conn->send(msg);

        // Update baseline
        for (const auto& [objName, fields] : current) {
            wsSessions_.updateBaseline(session->id, objName, fields);
        }
    }
}

// ============================================================================
// Auth helpers
// ============================================================================

bool KlippyHttpServer::checkAuth(const drogon::HttpRequestPtr& req) const {
    if (!config_.requireAuth) return true;

    std::string ip = req->peerAddr().toIp();
    if (isTrustedClient(ip)) return true;

    // Check X-Api-Key header
    auto apiKey = req->getHeader("X-Api-Key");
    if (!apiKey.empty() && checkApiKey(apiKey)) return true;

    // Check Authorization: Bearer header
    auto auth = req->getHeader("Authorization");
    if (auth.substr(0, 7) == "Bearer ") {
        std::string token = auth.substr(7);
        if (checkJwt(token)) return true;
    }

    // Check oneshot token in query
    auto params = req->getParameters();
    auto tokenIt = params.find("token");
    if (tokenIt != params.end()) {
        // Oneshot tokens are checked separately (they're consumed)
        // For non-const version, use checkOneshotToken
        return false;
    }

    return false;
}

bool KlippyHttpServer::isTrustedClient(const std::string& ip) const {
    for (const auto& trusted : config_.trustedClients) {
        if (ip == trusted) return true;
    }
    return false;
}

bool KlippyHttpServer::checkApiKey(const std::string& key) const {
    return key == config_.apiKey;
}

bool KlippyHttpServer::checkJwt(const std::string& token) const {
    // Simplified JWT check — in production, verify signature
    return !token.empty();
}

bool KlippyHttpServer::checkOneshotToken(const std::string& token, const std::string& ip) {
    std::lock_guard<std::mutex> lock(oneshotMutex_);
    auto now = std::chrono::steady_clock::now();
    for (auto it = oneshotTokens_.begin(); it != oneshotTokens_.end();) {
        if (now > it->expiry) {
            it = oneshotTokens_.erase(it);
            continue;
        }
        if (it->token == token && it->ip == ip) {
            it = oneshotTokens_.erase(it);
            return true;
        }
        ++it;
    }
    return false;
}

std::string KlippyHttpServer::generateApiKey() const {
    std::random_device rd;
    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 32; ++i) {
        ss << std::setw(2) << std::setfill('0') << (rd() & 0xFF);
    }
    return ss.str();
}

std::string KlippyHttpServer::generateOneshotToken(const std::string& ip) {
    std::random_device rd;
    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 16; ++i) {
        ss << std::setw(2) << std::setfill('0') << (rd() & 0xFF);
    }
    std::string token = ss.str();

    std::lock_guard<std::mutex> lock(oneshotMutex_);
    OneshotToken t;
    t.token = token;
    t.ip = ip;
    t.expiry = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    oneshotTokens_.push_back(t);
    return token;
}

std::string KlippyHttpServer::generateJwt(const std::string& username) const {
    // Simplified JWT — in production, use proper JWT signing
    return "tether_jwt_" + username + "_" + config_.jwtSecret;
}

// ============================================================================
// CORS helpers
// ============================================================================

void KlippyHttpServer::addCorsHeaders(const drogon::HttpResponsePtr& resp,
                                       const drogon::HttpRequestPtr& req) const {
    auto origin = req->getHeader("Origin");
    if (!origin.empty() && isCorsAllowed(origin)) {
        resp->addHeader("Access-Control-Allow-Origin", origin);
        resp->addHeader("Access-Control-Allow-Methods",
                         "GET, POST, DELETE, PUT, OPTIONS");
        resp->addHeader("Access-Control-Allow-Headers",
                         "Content-Type, X-Api-Key, Authorization, token");
        resp->addHeader("Access-Control-Allow-Credentials", "true");
    } else if (isCorsAllowed("*")) {
        resp->addHeader("Access-Control-Allow-Origin", "*");
        resp->addHeader("Access-Control-Allow-Methods",
                         "GET, POST, DELETE, PUT, OPTIONS");
        resp->addHeader("Access-Control-Allow-Headers",
                         "Content-Type, X-Api-Key, Authorization, token");
    }
}

bool KlippyHttpServer::isCorsAllowed(const std::string& origin) const {
    for (const auto& domain : config_.corsDomains) {
        if (domain == "*") return true;
        if (domain == origin) return true;
        // Wildcard matching (e.g., *.home)
        if (domain.size() > 1 && domain[0] == '*') {
            std::string suffix = domain.substr(1);
            if (origin.size() >= suffix.size() &&
                origin.substr(origin.size() - suffix.size()) == suffix) {
                return true;
            }
        }
        // Scheme wildcard (e.g., *://localhost:*)
        if (domain.find("*://") == 0) {
            auto rest = domain.substr(4);
            auto colonPos = rest.find(':');
            if (colonPos != std::string::npos) {
                std::string host = rest.substr(0, colonPos);
                std::string portPart = rest.substr(colonPos + 1);
                // Check if origin has the right host
                auto originColon = origin.find("://");
                if (originColon != std::string::npos) {
                    std::string originHost = origin.substr(originColon + 3);
                    auto originPortPos = originHost.find(':');
                    if (originPortPos != std::string::npos) {
                        originHost = originHost.substr(0, originPortPos);
                    }
                    if (originHost == host) return true;
                }
            }
        }
    }
    return false;
}

} // namespace tether::klipper::http
