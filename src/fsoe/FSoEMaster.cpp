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
    std::lock_guard<std::mutex> lock(mutex_);
    bool all_started = true;
    for (auto& entry : connections_) {
        if (entry.connection && !entry.connection->startConnection()) {
            all_started = false;
        }
    }
    return all_started;
}

void FSoEMaster::resetAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : connections_) {
        if (entry.connection) {
            entry.connection->resetConnection();
        }
    }
}

void FSoEMaster::update(uint64_t current_time_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : connections_) {
        if (entry.connection) {
            entry.connection->update(current_time_ms);
        }
    }
}

bool FSoEMaster::allOperational() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (connections_.empty()) return false;
    for (const auto& entry : connections_) {
        if (!entry.connection || !entry.connection->isOperational()) {
            return false;
        }
    }
    return true;
}

bool FSoEMaster::anyFailSafe() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : connections_) {
        if (entry.connection && entry.connection->isFailSafe()) {
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
    // via the IPDOTransport interface
    auto exchange_fn = [this]() -> bool {
        if (!dedicated_transport_) return false;

        // Use monotonic clock so watchdog/timeout checks work correctly
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        uint64_t now_ms = static_cast<uint64_t>(ms);

        // Use transport to send/receive FSoE frames for each connection
        for (auto& entry : connections_) {
            if (!entry.connection) continue;

            // Update connection state
            // (In a real implementation, this would send FPWR/FPRD datagrams
            // to the FSoE register addresses on the slave)
            // For now, we just call update to keep the state machine running
            entry.connection->update(now_ms);
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
    std::lock_guard<std::mutex> lock(mutex_);
    if (dedicated_loop_) {
        dedicated_loop_->stop();
        dedicated_loop_.reset();
    }
    dedicated_transport_ = nullptr;
}

bool FSoEMaster::isDedicatedThreadRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dedicated_loop_ != nullptr;
}

std::string FSoEMaster::getDiagnostics() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Inline allOperational/anyFailSafe checks to avoid recursive mutex deadlock
    bool all_op = !connections_.empty();
    bool any_fs = false;
    for (const auto& entry : connections_) {
        if (!entry.connection || !entry.connection->isOperational()) {
            all_op = false;
        }
        if (entry.connection && entry.connection->isFailSafe()) {
            any_fs = true;
        }
    }

    std::string diag;
    diag += "FSoE Master Diagnostics:\n";
    diag += "  Connections: " + std::to_string(connections_.size()) + "\n";
    diag += "  All Operational: " + std::string(all_op ? "Yes" : "No") + "\n";
    diag += "  Any Fail-Safe: " + std::string(any_fs ? "Yes" : "No") + "\n";
    diag += "  Mode: " + std::string(inline_mode_ ? "Inline" : "Dedicated/Manual") + "\n";

    for (size_t i = 0; i < connections_.size(); ++i) {
        diag += "\n--- Connection " + std::to_string(i) + " ---\n";
        if (connections_[i].connection) {
            diag += connections_[i].connection->getDiagnostics();
        }
    }

    return diag;
}

} // namespace FSoE
