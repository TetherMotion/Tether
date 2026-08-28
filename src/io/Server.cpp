/**
 * @file Server.cpp
 * @brief Multi-client server implementation.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Server.hpp"
#include <algorithm>

namespace tether { namespace io {

Server::Server(Registry& registry,
               std::unique_ptr<ITransportServer> transportServer,
               ServerConfig config)
    : registry_(registry)
    , transportServer_(std::move(transportServer))
    , config_(std::move(config))
{}

Server::~Server() {
    stop();
    if (loggerHandlerId_ != 0) {
        Tether::Platform::Logger::instance().removeHandler(loggerHandlerId_);
        loggerHandlerId_ = 0;
    }
}

bool Server::start() {
    if (running_) return false;
    if (!config_.timestampFn) return false;

    if (!transportServer_->start()) return false;

    running_ = true;

    if (loggerHandlerId_ == 0) {
        loggerHandlerId_ = Tether::Platform::Logger::instance().addHandler(
            [this](Tether::Platform::LogLevel level, const char* tag, const char* message) {
                LogSeverity severity = LogSeverity::Info;
                switch (level) {
                    case Tether::Platform::LogLevel::Error: severity = LogSeverity::Error; break;
                    case Tether::Platform::LogLevel::Warn: severity = LogSeverity::Warn; break;
                    case Tether::Platform::LogLevel::Info: severity = LogSeverity::Info; break;
                    case Tether::Platform::LogLevel::Debug: severity = LogSeverity::Debug; break;
                    case Tether::Platform::LogLevel::Verbose: severity = LogSeverity::Trace; break;
                    default: return;
                }
                publishLog(severity, tag ? tag : "", message ? message : "");
            });
    }

    acceptThread_ = std::thread([this]() { acceptLoop(); });

    if (config_.logFn) {
        config_.logFn("TetherIOServer", "Server started (max %zu clients)",
                      config_.maxClients);
    }
    return true;
}

void Server::stop() {
    if (!running_) return;
    running_ = false;

    if (loggerHandlerId_ != 0) {
        Tether::Platform::Logger::instance().removeHandler(loggerHandlerId_);
        loggerHandlerId_ = 0;
    }

    transportServer_->stop();

    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        for (auto& si : sessions_) {
            si.session->requestStop();
        }
    }

    if (acceptThread_.joinable()) {
        acceptThread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        for (auto& si : sessions_) {
            if (si.thread.joinable()) {
                si.thread.join();
            }
        }
        sessions_.clear();
    }

    if (config_.logFn) {
        config_.logFn("TetherIOServer", "Server stopped");
    }
}

size_t Server::activeSessionCount() const {
    std::lock_guard<std::mutex> lock(sessionsMutex_);
    size_t count = 0;
    for (const auto& si : sessions_) {
        if (si.session->isRunning()) ++count;
    }
    return count;
}

void Server::publishLog(LogSeverity severity, std::string_view component,
                        std::string_view message, std::string_view location) {
    std::vector<std::shared_ptr<Session>> sessions;
    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        sessions.reserve(sessions_.size());
        for (const auto& info : sessions_) {
            if (info.session->isRunning()) sessions.push_back(info.session);
        }
    }
    for (const auto& session : sessions) {
        session->publishLog(severity, component, message, location);
    }
}

void Server::acceptLoop() {
    while (running_.load(std::memory_order_relaxed)) {
        cleanupFinishedSessions();

        auto transport = transportServer_->accept();
        if (!transport) {
            if (!running_) break;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(sessionsMutex_);
            if (sessions_.size() >= config_.maxClients) {
                if (config_.logFn) {
                    config_.logFn("TetherIOServer",
                                  "Max clients reached, rejecting");
                }
                transport->close();
                continue;
            }
        }

        auto session = std::make_shared<Session>(
            std::move(transport), registry_,
            config_.timestampFn, config_.logFn,
            &config_.serverFeatures, &datalogRecorder_,
            config_.inputStreamCreateFn, config_.inputStreamDataFn,
            config_.encodedBufferFactory, config_.decodedBufferFactory,
            config_.framing);

        std::shared_ptr<Session> sessionPtr = session;

        SessionInfo si;
        si.session = std::move(session);

        // Add the session to sessions_ and start its thread within the same
        // lock scope.  This prevents activeSessionCount() from observing a
        // session that has already started processing requests but hasn't
        // been added to the list yet (a race that is especially visible
        // under TSAN where thread scheduling timing differs).
        {
            std::lock_guard<std::mutex> lock(sessionsMutex_);
            sessions_.push_back(std::move(si));
            SessionInfo& ref = sessions_.back();
            ref.session->markRunning();
            ref.thread = std::thread([sessionPtr]() { sessionPtr->run(); });
        }
    }
}

void Server::cleanupFinishedSessions() {
    std::vector<SessionInfo> finishedSessions;
    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        auto it = sessions_.begin();
        while (it != sessions_.end()) {
            if (!it->session->isRunning()) {
                finishedSessions.push_back(std::move(*it));
                it = sessions_.erase(it);
            } else {
                ++it;
            }
        }
    }
    // Join outside the lock to avoid holding the mutex while blocking.
    for (auto& si : finishedSessions) {
        if (si.thread.joinable()) {
            si.thread.join();
        }
    }
}

}} // namespace tether::io
