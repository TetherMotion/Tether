// SPDX-License-Identifier: MIT
/**
 * @file EthercatFleetMaster.hpp
 * @brief EtherCAT master bring-up for the emulated Beckhoff fleet.
 *
 * Drives a tether::ethercat::Master through the full slave lifecycle
 * against the BeckhoffFleet topology over a UdpEthercatLink:
 *
 *   start -> discover (BRD) -> mailbox config -> PREOP
 *         -> SM2/SM3 + fixed PDO registration -> SAFEOP -> OP
 *         -> cyclic exchangeAll()
 *
 * PDO exchange uses Position addressing (APWR/APRD into the slaves'
 * process-data RAM), matching what registerFixedPDOs() configures.
 * A dedicated RX thread pumps master.drainWire() so SDO/mailbox waits
 * and exchange responses are routed while the caller drives cycles.
 *
 * Shared by examples/ethercat_fleet_probe.cpp and the TetherPLC
 * integration test driver.
 */
#pragma once

#include "UdpEthercatLink.hpp"
#include "BeckhoffFleet.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/SlavePdoTypes.hpp"
#include "tether/ethercat/Mailbox.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace tether::examples {

class FleetMaster {
public:
    FleetMaster() = default;
    ~FleetMaster() { close(); }

    FleetMaster(const FleetMaster&) = delete;
    FleetMaster& operator=(const FleetMaster&) = delete;

    /**
     * Open the UDP link and start the master.
     * @param peerHost peer address (slave example, e.g. "127.0.0.1")
     * @param peerPort peer UDP port
     */
    bool open(const std::string& peerHost, uint16_t peerPort) {
        close();
        if (!link_.open(peerHost, peerPort)) return false;
        static const uint8_t kSrcMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        master_.start(link_.interface(), kSrcMac);
        rxRunning_.store(true);
        rxThread_ = std::thread([this]() { rxLoop(); });
        return true;
    }

    void close() {
        exchangeRunning_.store(false);
        if (exchangeThread_.joinable()) exchangeThread_.join();
        rxRunning_.store(false);
        if (rxThread_.joinable()) rxThread_.join();
        if (master_.isRunning()) master_.stop();
        link_.close();
    }

    EtherCAT::Master& master() { return master_; }
    EtherCAT::PDOManager& pdo() { return master_.pdo(); }

    /**
     * Full bring-up against the fleet described by @p infos:
     * discovery, mailbox, PREOP, PDO/SM config, SAFEOP, OP.
     * A cyclic exchange thread runs during and after bring-up; Slave::
     * transitionToOp() requires live PDO traffic, so OP transitions
     * cannot complete without it.
     * @param out  optional sink for the DiscoveredSlave vector (identity
     *             fields are populated for the shallow options used).
     * @return number of slaves discovered, or -1 on failure.
     */
    int bringUp(const std::vector<FleetSlaveInfo>& infos,
                std::vector<EtherCAT::DiscoveredSlave>* out = nullptr) {
        // Shallow SII reads only: the emulated EEPROM carries the identity
        // block, not the category sections deep options would parse.
        auto found = master_.discovery().discover(
            {EtherCAT::DiscoveryOption::VendorId,
             EtherCAT::DiscoveryOption::ProductCode,
             EtherCAT::DiscoveryOption::RevisionNumber,
             EtherCAT::DiscoveryOption::SerialNumber});
        if (found.empty()) {
            std::fprintf(stderr, "fleet-master: no slaves discovered\n");
            return -1;
        }
        if (out) *out = found;
        const uint16_t count = master_.getDiscoveredSlaveCount();
        std::fprintf(stderr, "fleet-master: discovered %u slave(s)\n",
                     static_cast<unsigned>(count));

        // Mailbox (SM0/SM1) + PREOP for every node.
        const EtherCAT::MailboxSyncManagerConfig mboxOut{0x1000, 128};
        const EtherCAT::MailboxSyncManagerConfig mboxIn {0x1080, 128};
        for (uint16_t s = 0; s < count; ++s) {
            auto& sl = master_.slave(s);
            if (sl.configureMailbox(mboxOut, mboxIn, 0x0004)
                    != EtherCAT::SlaveError::Ok) {
                std::fprintf(stderr, "fleet-master: slave %u mailbox config failed\n",
                             static_cast<unsigned>(s));
                return -1;
            }
            if (sl.transitionToPreOp() != EtherCAT::SlaveError::Ok) {
                std::fprintf(stderr, "fleet-master: slave %u PREOP failed\n",
                             static_cast<unsigned>(s));
                return -1;
            }
        }

        // Process data: SM2/SM3 + fixed PDO entries, then SAFEOP.
        for (uint16_t s = 0; s < count; ++s) {
            auto& sl = master_.slave(s);
            const FleetSlaveInfo* inf = findInfo(infos, s);
            const bool hasPdo = inf && (inf->txSize || inf->rxSize);

            if (hasPdo) {
                // SM2 = ProcessOutput (master->slave, RxPDO) at the
                // slave's rxPdoOffset; SM3 = ProcessInput (TxPDO) at
                // txPdoOffset.  Control bytes match the slave defaults.
                if (sl.configurePDOSyncManagers(
                        0x1100, inf->rxSize, 0x44,
                        0x1180, inf->txSize, 0x40)
                    != EtherCAT::SlaveError::Ok) {
                    std::fprintf(stderr,
                        "fleet-master: slave %u SM config failed\n",
                        static_cast<unsigned>(s));
                    return -1;
                }
                EtherCAT::SIIPDOConfig pc{};
                pc.has_rxpdo   = inf->rxSize != 0;
                pc.rxpdo_index = 0x1600;
                pc.rxpdo_size  = inf->rxSize;
                pc.has_txpdo   = inf->txSize != 0;
                pc.txpdo_index = 0x1A00;
                pc.txpdo_size  = inf->txSize;
                if (sl.registerFixedPDOs(pc) != EtherCAT::SlaveError::Ok) {
                    std::fprintf(stderr,
                        "fleet-master: slave %u PDO registration failed\n",
                        static_cast<unsigned>(s));
                    return -1;
                }
            } else {
                // Couplers and other mailbox-only nodes carry no process
                // data; mark PDO handled so SAFEOP/OP transitions pass.
                sl.assumePDOAlreadyConfigured();
            }

            if (sl.transitionToSafeOp() != EtherCAT::SlaveError::Ok) {
                std::fprintf(stderr,
                    "fleet-master: slave %u SAFEOP failed\n",
                    static_cast<unsigned>(s));
                return -1;
            }
        }

        // OP for every node.  transitionToOp() waits up to 100 ms for
        // PDO request/reply counters, so cyclic exchange must already
        // be running while the transitions proceed.
        startExchangeThread();
        for (uint16_t s = 0; s < count; ++s) {
            auto& sl = master_.slave(s);
            if (sl.transitionToOp() != EtherCAT::SlaveError::Ok) {
                std::fprintf(stderr,
                    "fleet-master: slave %u OP failed\n",
                    static_cast<unsigned>(s));
                return -1;
            }
        }
        return static_cast<int>(count);
    }

    /// One cyclic process-data exchange (all configured PDO entries).
    bool exchange() { return pdo().exchangeAll(); }

    /**
     * Stop the background exchange thread started by bringUp() or
     * startExchangeThread().  The caller then drives exchange()
     * synchronously (e.g. to interleave PDO<->file pumping per cycle).
     * Idempotent; exchangeAll() is not thread-safe, so exchange() must
     * not run concurrently with the thread.
     */
    void stopExchangeThread() {
        exchangeRunning_.store(false);
        if (exchangeThread_.joinable()) exchangeThread_.join();
    }

    /**
     * Run exchangeAll() on a dedicated thread at @p period_us intervals
     * until close().  Idempotent; bringUp() starts it automatically
     * before the OP transitions.
     */
    void startExchangeThread(uint32_t period_us = 1000) {
        if (exchangeRunning_.exchange(true)) return;
        exchangeThread_ = std::thread([this, period_us]() {
            while (exchangeRunning_.load(std::memory_order_relaxed)) {
                exchange();
                std::this_thread::sleep_for(std::chrono::microseconds(period_us));
            }
        });
    }

    /// Index of the first enabled PDO entry for @p slave_index and
    /// direction, or -1.  Directions: true = TxPDO (slave->master).
    int findEntry(uint16_t slave_index, bool txpdo) {
        const auto& m = pdo().mapping();
        for (size_t i = 0; i < m.entry_count(); ++i) {
            const auto* e = m.get_entry(i);
            if (e && e->enabled && e->slave_index == slave_index &&
                (e->direction == EtherCAT::PDO::PDODirection::TxPDO) == txpdo)
                return static_cast<int>(i);
        }
        return -1;
    }

private:
    static const FleetSlaveInfo* findInfo(
        const std::vector<FleetSlaveInfo>& infos, uint16_t index) {
        if (index < infos.size() && infos[index].chainIndex == index)
            return &infos[index];
        for (const auto& i : infos)
            if (i.chainIndex == index) return &i;
        return nullptr;
    }

    void rxLoop() {
        while (rxRunning_.load(std::memory_order_relaxed)) {
            master_.drainWire(64);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        master_.drainWire(256);   // final flush
    }

    UdpEthercatLink      link_;
    EtherCAT::Master     master_;
    std::atomic<bool>    rxRunning_{false};
    std::thread          rxThread_;
    std::atomic<bool>    exchangeRunning_{false};
    std::thread          exchangeThread_;
};

} // namespace tether::examples
