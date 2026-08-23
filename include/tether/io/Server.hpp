/**
 * @file Server.hpp
 * @brief Multi-client server for the Tether IO protocol.
 *
 * The Server listens for incoming connections via an ITransportServer,
 * accepts connections, and spawns a dedicated thread for each client.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Registry.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/Transport.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/Datalogging.hpp"
#include "logging/Logger.hpp"
#include <cstdint>
#include <cstddef>
#include <string_view>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

namespace tether { namespace io {

/// Server configuration.
struct ServerConfig {
    size_t      maxClients   = 4;       ///< Max concurrent sessions
    TimestampFn timestampFn  = nullptr; ///< Required: µs timestamp
    LogFn       logFn        = nullptr; ///< Optional logging callback
    FeatureSet  serverFeatures;         ///< Features to advertise
    InputStreamCreateFn inputStreamCreateFn;
    InputStreamDataFn inputStreamDataFn;
    ReceiveBufferFactory encodedBufferFactory;
    ReceiveBufferFactory decodedBufferFactory;
};

/**
 * @class Server
 * @brief Accepts transport connections and manages concurrent sessions.
 *
 * Usage:
 *  1. Construct with a Registry, transport server, and config.
 *  2. Call start() to begin accepting connections.
 *  3. Call stop() to shut down all sessions.
 */
class Server {
public:
    Server(Registry& registry,
           std::unique_ptr<ITransportServer> transportServer,
           ServerConfig config);
    ~Server();

    /// Start the accept loop. Returns false on failure.
    bool start();

    /// Shut down all sessions and the accept loop.
    void stop();

    /// Number of active sessions.
    size_t activeSessionCount() const;

    /// Returns true if the server is accepting connections.
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Access the server's datalogging recorder (may be null).
    DatalogRecorder* datalogRecorder() { return &datalogRecorder_; }

    /// Publish a log record to all matching client subscriptions.
    void publishLog(LogSeverity severity, std::string_view component,
                    std::string_view message, std::string_view location = {});

private:
    void acceptLoop();
    void cleanupFinishedSessions();

    Registry&    registry_;
    std::unique_ptr<ITransportServer> transportServer_;
    ServerConfig config_;
    std::atomic<bool> running_{false};

    struct SessionInfo {
        std::shared_ptr<Session> session;
        std::thread thread;
    };
    mutable std::mutex sessionsMutex_;
    std::vector<SessionInfo> sessions_;
    std::thread acceptThread_;

    DatalogRecorder datalogRecorder_;
    Tether::Platform::Logger::HandlerId loggerHandlerId_ = 0;
};

}} // namespace tether::io
