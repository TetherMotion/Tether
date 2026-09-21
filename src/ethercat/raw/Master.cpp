/**
 * @file Master.cpp
 * @brief Master class implementation
 *
 * Core lifecycle, motion-control loops, frame parsing and sub-manager accessors.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/SlaveStatusPoller.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/ethercat/CoeSDOChannel.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/MasterTransports.hpp"
#include "raw/CyclicDatapath.hpp"
#include "raw/SlaveRegistry.hpp"
#include "raw/MotionLoops.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "tether/platform/CpuIsolation.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>
#ifdef __linux__
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// Utility: getECStateName
// ============================================================================

const char* Master::getECStateName(uint8_t state)
{
    switch (state & 0x0Fu) {
        case 0x01: return "INIT";
        case 0x02: return "PRE_OP";
        case 0x03: return "BOOT";
        case 0x04: return "SAFE_OP";
        case 0x08: return "OP";
        default:   return "UNKNOWN";
    }
}

// adpForSlaveIndex is defined inline in Master.hpp

// ============================================================================
// Constructor / Destructor
// ============================================================================

Master::Master()
    : Master(Config{})
{
}

Master::Master(const Config& config)
    : config_(config)
{
    // Initialize PDO group routing: all slaves default to the default group (-1)
    pdo_group_idx_for_slave_.fill(-1);

    ensureRxQueues();

    // Initialize per-master packet router (required for waiters used by this master)
    if (!packet_router_.init()) {
        TETHER_LOGE(TAG, "Failed to initialize master packet router");
    }

    // Initialize debug gate and wire it to debug flags
    debug_gate_ = std::make_unique<DebugGate>();
    debug_flags_.setGate(debug_gate_.get());
    debug_gate_->setGateChangedCallback([this]() { this->updateDebugFlags(); });

    // Cyclic wire datapath (response slots, channel, process image).
    // Constructed early — transports and loops reach it via datapath_.
    datapath_ = std::make_unique<CyclicDatapath>(*this);
    slaves_   = std::make_unique<SlaveRegistry>(*this);

    // Create instance-based SDO transport (shared across all per-slave CoEManagers)
    sdo_transport_ = makeMasterSDOTransport(*this);

    // Create CoE SDO mailbox channel
    coe_sdo_channel_ = std::make_unique<Raw::CoeSDOChannel>();

    // Create thin wrapper sub-managers so callers may use e.g. master.dc() immediately.
    // Sub-managers are lightweight wrappers that forward to global/free-function
    // implementations that operate on file-scoped DC/SDO/PDO state.
    pdo_transport_ = makeMasterPDOTransport(*this);
    pdo_    = std::make_unique<PDOManager>(*pdo_transport_);
    logical_addr_mgr_ = std::make_unique<LogicalAddressManager>(*pdo_transport_);
    logical_addr_mgr_->setPrefixProvider(
        [this](uint16_t i) { return slaveLogPrefix(i); });
    logical_addr_mgr_->setDebugFlags(&debug_flags_);
    pdo_->setLogicalAddressManager(logical_addr_mgr_.get());
    pdo_->setDebugGate(debug_gate_.get());
    dc_     = std::make_unique<DCManager>(*this);
    foe_    = std::make_unique<FoEManager>(*this);
    voe_    = std::make_unique<VoEManager>(*this);
    eoe_    = std::make_unique<EoEManager>(*this);
    fault_transport_ = makeMasterFaultTransport(*this);
    faults_ = std::make_unique<FaultDetector>(*fault_transport_);
    faults_->setPrefixProvider(
        [this](uint16_t i) { return slaveLogPrefix(i); });
    status_poller_ = std::make_unique<SlaveStatusPoller>(*fault_transport_);
    slave_supervisor_ = std::make_unique<SlaveSupervisor>(*this);
}

Master::~Master()
{
    stop();

    // Clean up per-master packet router
    packet_router_.shutdown();
}


// ============================================================================
// Lifecycle
// ============================================================================

void Master::start(const NetworkInterface& iface, const uint8_t src_mac[6])
{
    iface_ = iface;
    std::memcpy(src_mac_, src_mac, 6);

    // Initialize frame transport (UDP encapsulation + raw sending)
#if TETHER_ENABLE_UDP_ENCAPSULATION
    transport_ = std::make_unique<EtherCATTransport>(&iface_, &config_.udp_encapsulation);
#else
    transport_ = std::make_unique<EtherCATTransport>(&iface_);
#endif

    ensureRxQueues();

    // Initialize per-slave CoEManagers
    for (size_t i = 0; i < sdo_managers_.size(); ++i) {
        if (!sdo_managers_[i]->init()) {
            TETHER_LOGW(TAG, "SDO subsystem failed to initialize for slave {}", i);
        }
    }
    if (!sdo_managers_.empty()) {
        TETHER_LOGI(TAG, "SDO subsystem initialized for {} slave(s)", sdo_managers_.size());
    }

    running_.store(true, std::memory_order_release);

    TETHER_LOGI(TAG, "Master started");
}

void Master::stop()
{
    const bool was_running = running_.load(std::memory_order_acquire);
    requestCancel();  // sets cancel flag + wakes packet router waiters
    stopMotionControlLoop();
    stopAsyncLoop();
    stopCyclicLoop();
    if (slave_supervisor_) {
        slave_supervisor_->stop();
    }
    if (status_poller_) {
        status_poller_->shutdown();
    }
    running_.store(false, std::memory_order_release);

    // Shutdown per-slave CoEManagers
    for (auto& mgr : sdo_managers_) {
        if (mgr) mgr->deinit();
    }

    if (was_running) {
        TETHER_LOGI(TAG, "Master stopped cleanly");
    }
}

void Master::requestCancel()
{
    cancel_requested_.store(true, std::memory_order_release);
    // Wake all threads currently blocked in the packet router so they
    // see the cancellation immediately instead of waiting for their
    // timeout to expire.  This is essential for prompt shutdown when
    // a signal handler calls requestCancel() while SDO/mailbox
    // operations are in flight.
    packet_router_.cancel();
}

bool Master::isCancelRequested() const
{
    return cancel_requested_.load(std::memory_order_acquire);
}

void Master::clearCancel()
{
    cancel_requested_.store(false, std::memory_order_release);
    cancel_warn_logged_.store(false, std::memory_order_release);
    packet_router_.clearCancel();
}

bool Master::isRunning() const
{
    return running_.load(std::memory_order_acquire);
}

// ============================================================================
// Source MAC
// ============================================================================

const uint8_t* Master::getSrcMac() const { return src_mac_; }

const NetworkInterface* Master::networkInterface() const { return &iface_; }




// ============================================================================
// CoE / SDO low-level — delegate to existing Raw:: functions
// ============================================================================

bool Master::coeSdoUpload(uint16_t adp, uint8_t* inout_mbx_cnt,
                                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                                   uint16_t index, uint8_t sub,
                                   uint8_t* out, size_t out_cap, size_t* out_len,
                                   bool diag_enabled,
                                   unsigned int poll_interval_ms,
                                   unsigned int transaction_timeout_ms)
{
    return coe_sdo_channel_->upload(*this, adp, inout_mbx_cnt,
                               mbx_wr_addr, mbx_wr_len,
                               mbx_rd_addr, mbx_rd_len,
                               index, sub, out, out_cap, out_len, diag_enabled,
                               poll_interval_ms, transaction_timeout_ms);
}

bool Master::coeSdoDownload(uint16_t adp, uint8_t* inout_mbx_cnt,
                                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                                     uint16_t index, uint8_t sub,
                                     const uint8_t* data, size_t data_len,
                                     bool diag_enabled,
                                     unsigned int poll_interval_ms,
                                     unsigned int transaction_timeout_ms)
{
    return coe_sdo_channel_->download(*this, adp, inout_mbx_cnt,
                                 mbx_wr_addr, mbx_wr_len,
                                 mbx_rd_addr, mbx_rd_len,
                                 index, sub, data, data_len, diag_enabled,
                                 poll_interval_ms, transaction_timeout_ms);
}

uint32_t Master::lastCoeSdoAbortCode() const {
    return coe_sdo_channel_ ? coe_sdo_channel_->lastAbortCode() : 0u;
}

// ============================================================================
// Test hooks
// ============================================================================

void Master::setAprdTestCallback(AprdTestCb cb) { aprd_cb_ = std::move(cb); }
void Master::setApwrTestCallback(ApwrTestCb cb) { apwr_cb_ = std::move(cb); }

void Master::pushAprdResponse(bool success, uint16_t adp, uint16_t ado,
                                       const void* data, uint16_t len)
{
    AprdResponse r;
    r.success = success; r.adp = adp; r.ado = ado;
    if (data && len > 0)
        r.data.assign(reinterpret_cast<const uint8_t*>(data),
                      reinterpret_cast<const uint8_t*>(data) + len);
    aprd_responses_.push_back(std::move(r));
}

void Master::clearAprdResponses() { aprd_responses_.clear(); }

bool Master::wasFaultDiagnosed(uint16_t slave_index) const
{
    std::lock_guard<std::mutex> _lg(m_diag_mutex_);
    return m_diagnosed_slaves_.find(slave_index) != m_diagnosed_slaves_.end();
}

// ============================================================================
// Sub-manager accessors
// ============================================================================

PDOManager&    Master::pdo()    { return *pdo_; }
LogicalAddressManager& Master::logicalAddressManager() { return *logical_addr_mgr_; }

PDOManager& Master::pdoForSlave(uint16_t slave_index) {
    if (slave_index < PDO::kMaxPDOSlaves) {
        const int idx = pdo_group_idx_for_slave_[slave_index];
        if (idx >= 0 && static_cast<size_t>(idx) < pdo_groups_.size()) {
            return *pdo_groups_[idx].pdo;
        }
    }
    return *pdo_;
}

LogicalAddressManager& Master::logicalAddressManagerForSlave(uint16_t slave_index) {
    if (slave_index < PDO::kMaxPDOSlaves) {
        const int idx = pdo_group_idx_for_slave_[slave_index];
        if (idx >= 0 && static_cast<size_t>(idx) < pdo_groups_.size()) {
            return *pdo_groups_[idx].lam;
        }
    }
    return *logical_addr_mgr_;
}

PDOManager& Master::createPdoGroup(const std::vector<uint16_t>& slave_indices,
                                    uint32_t base_logical_addr) {
    auto& group = pdo_groups_.emplace_back();
    group.transport = makeMasterPDOTransport(*this);
    group.pdo = std::make_unique<PDOManager>(*group.transport);
    group.lam = std::make_unique<LogicalAddressManager>(*group.transport);
    group.lam->setBaseLogicalAddress(base_logical_addr);
    group.lam->setDebugFlags(&debug_flags_);
    group.pdo->init();
    group.lam->init();
    group.pdo->setLogicalAddressManager(group.lam.get());

    // Copy prefix provider for logging
    group.lam->setPrefixProvider([this](uint16_t idx) {
        return slaveLogPrefix(idx);
    });

    // Assign slaves to this group
    const int group_idx = static_cast<int>(pdo_groups_.size() - 1);
    for (uint16_t idx : slave_indices) {
        if (idx < PDO::kMaxPDOSlaves) {
            pdo_group_idx_for_slave_[idx] = group_idx;
            TETHER_LOGI("ec_master", "Slave {} assigned to PDO group {} (base_log=0x{:08X})",
                        idx, group_idx, base_logical_addr);
        }
    }

    return *group.pdo;
}
::EtherCAT::CoE::CoEManager& Master::sdoManager(uint16_t slave_index) {
    std::lock_guard<std::mutex> lock(sdo_managers_mutex_);
    if (slave_index >= sdo_managers_.size()) {
        // Lazily expand the vector if needed
        size_t old_size = sdo_managers_.size();
        sdo_managers_.resize(slave_index + 1);
        for (size_t i = old_size; i <= slave_index; ++i) {
            sdo_managers_[i] = std::make_unique<::EtherCAT::CoE::CoEManager>(
                static_cast<uint16_t>(i), *sdo_transport_);
            sdo_managers_[i]->setLogPrefix(slaveLogPrefix(static_cast<uint16_t>(i)));
            sdo_managers_[i]->setPDOManager(pdo_.get());
            if (running_.load()) {
                sdo_managers_[i]->init();
            }
        }
    }
    return *sdo_managers_[slave_index];
}
DCManager&     Master::dc()     { return *dc_; }
FoEManager&    Master::foe()    { return *foe_; }
VoEManager&    Master::voe()    { return *voe_; }
EoEManager&    Master::eoe()    { return *eoe_; }
FaultDetector& Master::faults() { return *faults_; }

SlaveStatusPoller& Master::statusPoller() { return *status_poller_; }

SlaveSupervisor& Master::slaveSupervisor() { return *slave_supervisor_; }

ConditionalPacketRouter& Master::packetRouter() { return packet_router_; }

Tether::Platform::MessageQueue<RxDatagram>* Master::rxQueue()
{
    return rx_queue_.get();
}

Tether::Platform::MessageQueue<RxDatagram>* Master::txpdoRxQueue()
{
    return txpdo_rx_queue_.get();
}

// ============================================================================
// Statistics
// ============================================================================

#if TETHER_ENABLE_ETHERCAT_STATS
Master::Stats Master::getStats() const
{
    Stats s;
    s.tx_retry_count = tx_retry_count_.load(std::memory_order_relaxed);
    s.tx_fail_count  = tx_fail_count_.load(std::memory_order_relaxed);
    s.rx_frame_count = rx_frame_count_.load(std::memory_order_relaxed);
    s.rx_queue_sent  = rx_queue_sent_.load(std::memory_order_relaxed);
    s.rx_flushed     = total_flushed_.load(std::memory_order_relaxed);
    s.flush_calls    = flush_calls_.load(std::memory_order_relaxed);
    return s;
}
#endif

// ============================================================================
// Internal: queue management
// ============================================================================

void Master::ensureRxQueues()
{
    if (!rx_queue_)
        rx_queue_ = std::make_unique<Tether::Platform::MessageQueue<RxDatagram>>(
            config_.rx_queue_depth);
    if (!txpdo_rx_queue_)
        txpdo_rx_queue_ = std::make_unique<Tether::Platform::MessageQueue<RxDatagram>>(
            config_.txpdo_queue_depth);
}

void Master::flushRxQueue()
{
    if (!rx_queue_) return;
    RxDatagram tmp;
    size_t flushed = 0;
    while (rx_queue_->receive(tmp, 0)) flushed++;
#if TETHER_ENABLE_ETHERCAT_STATS
    total_flushed_.fetch_add(static_cast<uint32_t>(flushed), std::memory_order_relaxed);
    if (flushed) flush_calls_.fetch_add(1, std::memory_order_relaxed);
#else
    (void)flushed;
#endif
}

} // namespace EtherCAT
