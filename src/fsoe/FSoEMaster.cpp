/**
 * @file FSoEMaster.cpp
 * @brief FSoE Master implementation — manages multiple connections
 */

#include "fsoe/FSoEMaster.hpp"
#include "tether/ethercat/PDORegionManager.hpp"
#include "tether/ethercat/CyclicTaskScheduler.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>

namespace FSoE {

FSoEMaster::FSoEMaster() = default;
FSoEMaster::~FSoEMaster() = default;

int FSoEMaster::addConnection(const MasterConnectionConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Check for duplicate connection ID or slave address
    for (const auto& entry : connections_) {
        if (!entry.connection) continue;
        if (entry.connection->getConfig().connection_id == config.connection_id) {
            return -1;
        }
        if (entry.connection->getConfig().slave_addr == config.slave_addr) {
            return -1;
        }
    }

    auto conn = std::make_unique<FSoEMasterConnection>(config);
    if (!conn->initialize()) {
        return -1;
    }

    int id = next_id_++;
    connections_.push_back({id, std::move(conn)});
    return id;
}

bool FSoEMaster::removeConnection(int connection_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::remove_if(connections_.begin(), connections_.end(),
        [connection_id](const ConnectionEntry& e) {
            return e.connection &&
                   e.connection->getConfig().connection_id == static_cast<uint16_t>(connection_id);
        });

    if (it == connections_.end()) return false;
    connections_.erase(it, connections_.end());
    return true;
}

FSoEMasterConnection* FSoEMaster::getConnection(int connection_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : connections_) {
        if (entry.connection &&
            entry.connection->getConfig().connection_id == static_cast<uint16_t>(connection_id)) {
            return entry.connection.get();
        }
    }
    return nullptr;
}

FSoEMasterConnection* FSoEMaster::getConnectionBySlaveAddr(uint16_t slave_addr)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : connections_) {
        if (entry.connection && entry.connection->getConfig().slave_addr == slave_addr) {
            return entry.connection.get();
        }
    }
    return nullptr;
}

size_t FSoEMaster::getConnectionCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return connections_.size();
}

bool FSoEMaster::startAll()
{
    // Snapshot connection pointers under a short lock, then call
    // startConnection() without holding the lock. This prevents AB-BA
    // deadlock with connection callbacks that may call back into FSoEMaster.
    std::vector<FSoEMasterConnection*> conns;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conns.reserve(connections_.size());
        for (auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    bool all_started = true;
    for (auto* conn : conns) {
        if (!conn->startConnection()) {
            all_started = false;
        }
    }
    return all_started;
}

void FSoEMaster::resetAll()
{
    // Snapshot connection pointers under a short lock, then call
    // resetConnection() without holding the lock. resetConnection() calls
    // transitionTo() which may fire state_change_callback_; if the callback
    // calls back into FSoEMaster, the non-recursive mutex would deadlock.
    std::vector<FSoEMasterConnection*> conns;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conns.reserve(connections_.size());
        for (auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    for (auto* conn : conns) {
        conn->resetConnection();
    }
}

void FSoEMaster::update(uint64_t current_time_ms)
{
    // Snapshot connection pointers under a short lock, then call update()
    // without holding the lock. This prevents deadlock if a connection
    // callback (fail_safe, error, state_change) calls back into FSoEMaster
    // (e.g., anyFailSafe(), allOperational(), getDiagnostics()).
    // FSoEMaster uses a non-recursive std::mutex, so re-entry would deadlock.
    std::vector<FSoEMasterConnection*> conns;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conns.reserve(connections_.size());
        for (auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    for (auto* conn : conns) {
        conn->update(current_time_ms);
    }
}

bool FSoEMaster::allOperational() const
{
    // Snapshot connection pointers under a short lock, then call
    // isOperational() without holding the lock. This prevents AB-BA
    // deadlock: without this, allOperational() holds master.mutex_ while
    // locking conn.mutex_, while a connection callback may hold conn.mutex_
    // and try to lock master.mutex_ (e.g., via anyFailSafe()).
    std::vector<const FSoEMasterConnection*> conns;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (connections_.empty()) return false;
        conns.reserve(connections_.size());
        for (const auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    for (const auto* conn : conns) {
        if (!conn->isOperational()) {
            return false;
        }
    }
    return true;
}

bool FSoEMaster::anyFailSafe() const
{
    // Snapshot connection pointers under a short lock, then call
    // isFailSafe() without holding the lock. Same AB-BA prevention as
    // allOperational().
    std::vector<const FSoEMasterConnection*> conns;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conns.reserve(connections_.size());
        for (const auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    for (const auto* conn : conns) {
        if (conn->isFailSafe()) {
            return true;
        }
    }
    return false;
}

void FSoEMaster::enableInlineMode(EtherCAT::PDORegionManager& region_mgr,
                                  EtherCAT::CyclicTaskScheduler& scheduler)
{
    std::lock_guard<std::mutex> lock(mutex_);
    inline_mode_ = true;

    // Register PDO regions for each connection
    // The actual region offsets depend on the PDO mapping configuration
    // which is set up by the application before calling this method.
    // Here we just mark that inline mode is active; the application
    // is responsible for registering the specific regions via region_mgr.
    (void)region_mgr;
    (void)scheduler;
}

bool FSoEMaster::startDedicatedThread(EtherCAT::IPDOTransport& transport,
                                       uint32_t cycle_period_us)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dedicated_loop_) {
        return false;  // Already running
    }

    dedicated_transport_ = &transport;

    // Create a RealtimeLoop that exchanges FSoE frames each cycle
    // The callback iterates all connections, sending/receiving FSoE PDUs
    // via the IPDOTransport interface.
    //
    // Thread-safety: exchange_fn runs on the RT thread and must not hold
    // mutex_ for the duration of the exchange (stopDedicatedThread() may
    // be waiting for the thread to finish while holding mutex_). Instead,
    // we take a short lock to snapshot the connection pointers, then
    // iterate the snapshot without holding the lock. The connections are
    // owned by unique_ptr in the vector, so the raw pointers remain valid
    // as long as removeConnection() isn't called concurrently (which would
    // be a user error — the application should stop the thread before
    // modifying connections).
    auto exchange_fn = [this]() -> bool {
        if (!dedicated_transport_) return false;

        // Use monotonic clock so watchdog/timeout checks work correctly
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        uint64_t now_ms = static_cast<uint64_t>(ms);

        // Snapshot connection pointers under a short lock
        std::vector<FSoEMasterConnection*> conns;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            conns.reserve(connections_.size());
            for (auto& entry : connections_) {
                if (entry.connection) {
                    conns.push_back(entry.connection.get());
                }
            }
        }

        // Update each connection without holding the lock
        for (auto* conn : conns) {
            conn->update(now_ms);
        }
        return true;
    };

    auto sync_fn = []() -> bool { return true; };
    auto time_fn = []() -> uint64_t {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
    };

    dedicated_loop_ = std::make_unique<EtherCAT::RealtimeLoop>(
        exchange_fn, sync_fn, time_fn,
        EtherCAT::RealtimeLoop::Config::defaults(cycle_period_us, 1));

    return dedicated_loop_->start();
}

void FSoEMaster::stopDedicatedThread()
{
    // Stop the thread BEFORE taking the lock, so exchange_fn (which takes
    // a short lock on mutex_) can finish without deadlocking. The thread
    // owns a reference to this (via the lambda), so it's safe to access
    // dedicated_loop_ without holding the lock — only start/stop/this
    // function modify it.
    std::unique_ptr<EtherCAT::RealtimeLoop> loop_to_stop;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        loop_to_stop = std::move(dedicated_loop_);
        dedicated_transport_ = nullptr;
    }

    // Stop the thread outside the lock to avoid deadlock with exchange_fn
    if (loop_to_stop) {
        loop_to_stop->stop();
        // unique_ptr destructor cleans up here, after stop() has returned
    }
}

bool FSoEMaster::isDedicatedThreadRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dedicated_loop_ != nullptr;
}

std::string FSoEMaster::getDiagnostics() const
{
    // Snapshot connection pointers and metadata under a short lock, then
    // call isOperational()/isFailSafe()/getDiagnostics() without holding
    // the lock. This prevents AB-BA deadlock with connection callbacks.
    std::vector<const FSoEMasterConnection*> conns;
    bool inline_mode;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        inline_mode = inline_mode_;
        conns.reserve(connections_.size());
        for (const auto& entry : connections_) {
            if (entry.connection) {
                conns.push_back(entry.connection.get());
            }
        }
    }

    bool all_op = !conns.empty();
    bool any_fs = false;
    for (const auto* conn : conns) {
        if (!conn->isOperational()) all_op = false;
        if (conn->isFailSafe()) any_fs = true;
    }

    std::string diag;
    diag += "FSoE Master Diagnostics:\n";
    diag += "  Connections: " + std::to_string(conns.size()) + "\n";
    diag += "  All Operational: " + std::string(all_op ? "Yes" : "No") + "\n";
    diag += "  Any Fail-Safe: " + std::string(any_fs ? "Yes" : "No") + "\n";
    diag += "  Mode: " + std::string(inline_mode ? "Inline" : "Dedicated/Manual") + "\n";

    for (size_t i = 0; i < conns.size(); ++i) {
        diag += "\n--- Connection " + std::to_string(i) + " ---\n";
        diag += conns[i]->getDiagnostics();
    }

    return diag;
}

} // namespace FSoE
