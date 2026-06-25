/**
 * @file FSoEMaster.hpp
 * @brief FSoE Master — manages multiple FSoE master connections
 *
 * Replaces the old FSoEMaster class. Supports two modes:
 * 1. Inline mode: FSoE PDUs inserted into PDO frames via PDORegionManager
 * 2. Dedicated thread mode: separate RealtimeLoop sending FSoE-only EtherCAT frames
 */

#pragma once

#include "fsoe/FSoEMasterConnection.hpp"
#include <vector>
#include <memory>
#include <string>
#include <mutex>

namespace EtherCAT {
class PDOManager;
class PDORegionManager;
class CyclicTaskScheduler;
class IPDOTransport;
class RealtimeLoop;
} // namespace EtherCAT

namespace FSoE {

class FSoEMaster {
public:
    FSoEMaster();
    ~FSoEMaster();

    FSoEMaster(const FSoEMaster&) = delete;
    FSoEMaster& operator=(const FSoEMaster&) = delete;

    // --- Connection management ---
    int addConnection(const MasterConnectionConfig& config);
    bool removeConnection(int connection_id);
    FSoEMasterConnection* getConnection(int connection_id);
    FSoEMasterConnection* getConnectionBySlaveAddr(uint16_t slave_addr);
    size_t getConnectionCount() const;

    // --- Bulk operations ---
    bool startAll();
    void resetAll();
    void update(uint64_t current_time_ms);
    bool allOperational() const;
    bool anyFailSafe() const;

    // --- Inline mode ---
    // Register FSoE regions with PDORegionManager and cyclic tasks with scheduler.
    // Each connection's FSoE PDU is mapped to a PDO region for its slave.
    void enableInlineMode(EtherCAT::PDORegionManager& region_mgr,
                          EtherCAT::CyclicTaskScheduler& scheduler);

    // --- Dedicated thread mode ---
    // Creates a separate RealtimeLoop that exchanges FSoE frames via IPDOTransport.
    bool startDedicatedThread(EtherCAT::IPDOTransport& transport,
                              uint32_t cycle_period_us = 1000);
    void stopDedicatedThread();
    bool isDedicatedThreadRunning() const;

    // --- Diagnostics ---
    std::string getDiagnostics() const;

private:
    struct ConnectionEntry {
        int id = 0;
        std::unique_ptr<FSoEMasterConnection> connection;
    };

    std::vector<ConnectionEntry> connections_;
    mutable std::mutex mutex_;
    int next_id_ = 1;

    // Dedicated thread mode
    std::unique_ptr<EtherCAT::RealtimeLoop> dedicated_loop_;
    EtherCAT::IPDOTransport* dedicated_transport_ = nullptr;
    bool inline_mode_ = false;
};

} // namespace FSoE
