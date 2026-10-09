/**
 * @file Master_slave.cpp
 * @brief Master — Slave management, AL state, watchdog and mailbox fallback
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/SlaveRegistry.hpp"
#include "raw/TxFailureDiagnostics.hpp"
#include "raw/WatchdogController.hpp"
#include "raw/MailboxRecovery.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <bit>
#include <vector>
#include <format>
#ifdef __linux__
#include <sys/socket.h>
#include <unistd.h>
#endif
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// Slave Management
// ============================================================================

void Master::initSlaves(uint16_t count)
{
    slaves_->reset(count);

    // Create Slave objects first. Each Slave owns a per-slave SIIManager;
    // binding it to this master and the correct slave index happens here.
    for (uint16_t i = 0; i < count; ++i) {
        auto s = std::make_unique<Slave>(*this, i);
#if TETHER_ENABLE_SII
        s->sii().init(*this, i);
#endif
        slaves_->set(i, std::move(s));
    }

#if TETHER_ENABLE_SII
    // Bulk-prefetch the first 128 words of SII EEPROM for each slave in
    // parallel. Each SIIManager is per-slave and accesses the bus through
    // Master::sendRawFrame, which is protected by send_mutex_.
    std::vector<std::jthread> threads;
    threads.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
        threads.emplace_back([this, i] {
            if (Slave* s = slaves_->slaveAt(i)) {
                s->sii().prefetchWords(0, 128);
            }
        });
    }
    threads.clear();  // join all
#endif

    // Resize filters to current slave count and push per-slave flags.
    debug_flags_.resizeFilters(count);
    updateDebugFlags();
}

void Master::setSlaveName(uint16_t idx, std::string name) {
    slaves_->setName(idx, std::move(name));
}

std::string_view Master::slaveName(uint16_t idx) const {
    return slaves_->name(idx);
}

std::string Master::slaveLogPrefix(uint16_t idx) const {
    return slaves_->logPrefix(idx);
}

bool Master::drainSlaveMailbox(uint16_t slave_index, unsigned int max_drain)
{
    return mailbox_recovery_->drain(slave_index, max_drain);
}

bool Master::resetSlaveMailboxSM1(uint16_t slave_index)
{
    return mailbox_recovery_->resetSM1(slave_index);
}

bool Master::resetSlaveMailboxSM0(uint16_t slave_index)
{
    return mailbox_recovery_->resetSM0(slave_index);
}

void Master::updateDebugFlags()
{
    const uint16_t count = static_cast<uint16_t>(slaves_->size());
    std::vector<EtherCATSlaveDebugFlags> per_slave_flags(count);
    for (uint16_t i = 0; i < count; ++i) {
        per_slave_flags[i] = debug_flags_.computeForSlave(i);
        if (Slave* s = slaves_->slaveAt(i)) {
            s->updateDebugFlags(per_slave_flags[i]);
        }
    }
    {
        std::lock_guard<std::mutex> lock(sdo_managers_mutex_);
        for (uint16_t i = 0; i < count; ++i) {
            if (i < sdo_managers_.size() && sdo_managers_[i]) {
                sdo_managers_[i]->updateDebugFlags(per_slave_flags[i]);
            }
        }
    }
    if (pdo_) {
        pdo_->setDebugFlags(&debug_flags_);
    }
    for (auto& group : pdo_groups_) {
        if (group.pdo) group.pdo->setDebugFlags(&debug_flags_);
    }
    if (logical_addr_mgr_) {
        logical_addr_mgr_->setDebugFlags(&debug_flags_);
    }
    for (auto& group : pdo_groups_) {
        if (group.lam) group.lam->setDebugFlags(&debug_flags_);
    }
}

Slave& Master::slave(uint16_t slave_index)
{
    // Sentinel fallback for invalid index (or null entry from a partially
    // initialized/cleaned-up master) — prevents SIGSEGV during shutdown.
    return slaves_->get(slave_index);
}

#if TETHER_ENABLE_SII

SII::SIIManager& Master::sii(uint16_t slave_index)
{
    return slaves_->sii(slave_index);
}

#endif // TETHER_ENABLE_SII

bool Master::resolvePhysicalSlaveIndex(SlaveAddress slave_address, uint16_t& slave_index_out)
{
    if (!slave_address.isPhysical()) {
        TETHER_LOGE(TAG, "Operation requires a physical slave address");
        return false;
    }

    slave_index_out = slave_address.slavePosition();
    return true;
}

// ============================================================================
// Discovery
// ============================================================================

uint16_t Master::getDiscoveredSlaveCount() const
{
    return slaves_->discovered_count.load(std::memory_order_acquire);
}

// ============================================================================
// Watchdog configuration
// ============================================================================

bool Master::configureWatchdogs(SlaveAddress slave_address,
                                         uint16_t pdi_timeout_100us,
                                         uint16_t pdata_timeout_100us)
{
    return watchdog_->configure(slave_address, pdi_timeout_100us,
                                pdata_timeout_100us);
}

bool Master::disableWatchdogs(SlaveAddress slave_address)
{
    return watchdog_->disable(slave_address);
}

bool Master::readWatchdogStatus(SlaveAddress slave_address,
                                         uint8_t& wd_status,
                                         uint8_t& pdi_cnt,
                                         uint8_t& pdata_cnt)
{
    return watchdog_->readStatus(slave_address, wd_status, pdi_cnt, pdata_cnt);
}

std::string Master::txFailureDiagnostics()
{
#ifdef __linux__
    // The AF_PACKET fd carrying LRW sends — the direct path wires it as
    // iface_.native_handle; under VLAN encapsulation iface_ is the router
    // stub and the cyclic channel's fd is the only wire socket available.
    int fd = -1;
    if (iface_.native_handle) {
        fd = static_cast<int>(reinterpret_cast<intptr_t>(iface_.native_handle));
    } else if (ICyclicChannel* ch = cyclicChannel()) {
        fd = ch->fd();
    }
    std::string report = TxFailureDiagnostics::probe(fd);
    // The kernel's own verdict on the last failed sendto — the probe's
    // socket/interface state explains *why* it happens, the errno is *what*
    // the kernel reported.
    const int e = last_tx_errno_.load(std::memory_order_relaxed);
    if (e != 0) {
        report = std::format("last_send_errno={}({}) | {}",
                             e, strerror(e), report);
    }
    return report;
#else
    return {};
#endif
}

} // namespace EtherCAT
