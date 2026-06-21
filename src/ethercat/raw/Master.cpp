/**
 * @file Master.cpp
 * @brief Master class implementation
 *
 * Core lifecycle, motion-control loops, frame parsing and sub-manager accessors.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// Global registry of Master instances (host-only helper). This
// allows host-side helpers (examples) to find the master associated with
// a NetworkInterface pointer.
static std::mutex g_master_list_mutex;
static std::vector<Master*> g_master_list;

// TX retry constants
static constexpr int       kMaxTxRetries   = 3;
static constexpr uint32_t  kTxRetryDelayUs = 50;

class IMotionControlLoop {
public:
    virtual ~IMotionControlLoop() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
};

class RealtimeMotionControlLoop final : public IMotionControlLoop {
public:
    RealtimeMotionControlLoop(Master::MotionControlCallback callback,
                              Master::RealtimeMotionLoopConfig config,
                              EtherCAT::DCManager* dc_manager)
        : callback_(std::move(callback))
        , dt_seconds_(static_cast<double>(config.cycle_period_us) / 1000000.0)
        , loop_(
            [this]() { return callback_ ? callback_(dt_seconds_) : true; },
            [this, config, dc_manager]() {
                if (!config.enable_dc_synchronization || dc_manager == nullptr) {
                    return true;
                }
                return dc_manager->get()->sendSyncFrame();
            },
            []() {
                return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            },
            RealtimeLoop::Config::defaults(config.cycle_period_us, config.sync_interval_cycles))
    {
    }

    bool start() override {
        loop_.setPDOEnabled(true);
        return loop_.start();
    }

    void stop() override {
        loop_.stop();
    }

    bool isRunning() const override {
        return loop_.isRunning();
    }

private:
    Master::MotionControlCallback callback_;
    double dt_seconds_;
    RealtimeLoop loop_;
};

class PollingMotionControlLoop final : public IMotionControlLoop {
public:
    PollingMotionControlLoop(Master::MotionControlCallback callback,
                             Master::PollingMotionLoopConfig config,
                             EtherCAT::DCManager* dc_manager)
        : callback_(std::move(callback))
        , config_(config)
        , dc_manager_(dc_manager)
    {
    }

    ~PollingMotionControlLoop() override {
        stop();
    }

    bool start() override {
        if (running_.exchange(true, std::memory_order_acq_rel)) {
            return false;
        }

        thread_ = std::thread([this]() {
            if (config_.request_realtime_priority) {
                (void)Tether::Platform::setCurrentThreadRealtime(-1);
            }

            const auto period = std::chrono::microseconds(config_.cycle_period_us);
            const double dt_seconds = static_cast<double>(config_.cycle_period_us) / 1000000.0;
            auto next_tick = std::chrono::steady_clock::now();
            uint32_t cycle = 0;

            while (running_.load(std::memory_order_acquire)) {
                next_tick += period;
                if (callback_ && !callback_(dt_seconds)) {
                    running_.store(false, std::memory_order_release);
                    break;
                }

                if (config_.enable_dc_synchronization &&
                    dc_manager_ != nullptr &&
                    config_.sync_interval_cycles != 0 &&
                    (++cycle % config_.sync_interval_cycles) == 0) {
                    if (!dc_manager_->get()->sendSyncFrame()) {
                        running_.store(false, std::memory_order_release);
                        break;
                    }
                }

                std::this_thread::sleep_until(next_tick);
            }
        });

        return true;
    }

    void stop() override {
        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool isRunning() const override {
        return running_.load(std::memory_order_acquire);
    }

private:
    Master::MotionControlCallback callback_;
    Master::PollingMotionLoopConfig config_;
    EtherCAT::DCManager* dc_manager_{nullptr};
    std::atomic<bool> running_{false};
    std::thread thread_;
};

// ============================================================================
// Queue-mode RT loop: calls PDOManager::queueCycle() each cycle
// Reuses RealtimeLoop infrastructure (same timer, jitter monitor, DC sync)
// ============================================================================

class QueueMotionControlLoop final : public IMotionControlLoop {
public:
    QueueMotionControlLoop(PDOManager* pdo_manager,
                           Master::RealtimeMotionLoopConfig config,
                           EtherCAT::DCManager* dc_manager)
        : pdo_manager_(pdo_manager)
        , loop_(
            [this]() { return pdo_manager_ ? pdo_manager_->queueCycle() : false; },
            [this, config, dc_manager]() {
                if (!config.enable_dc_synchronization || dc_manager == nullptr) {
                    return true;
                }
                return dc_manager->get()->sendSyncFrame();
            },
            []() {
                return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            },
            RealtimeLoop::Config::defaults(config.cycle_period_us, config.sync_interval_cycles))
    {
    }

    bool start() override {
        loop_.setPDOEnabled(true);
        return loop_.start();
    }

    void stop() override {
        loop_.stop();
    }

    bool isRunning() const override {
        return loop_.isRunning();
    }

private:
    PDOManager* pdo_manager_;
    RealtimeLoop loop_;
};


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
// MasterPDOTransport — adapts Master to IPDOTransport
// ============================================================================

class MasterPDOTransport : public IPDOTransport {
public:
    explicit MasterPDOTransport(Master& master) : master_(master) {}

    bool writeRegister(uint16_t adp, uint16_t ado,
                       const void* data, uint16_t len,
                       unsigned int timeout_ms) override {
        return master_.writeRegister(Master::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool readRegister(uint16_t adp, uint16_t ado,
                      void* data, uint16_t len,
                      unsigned int timeout_ms) override {
        return master_.readRegister(Master::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool sendSingleDatagram(Command cmd, uint8_t idx,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip) override {
        return master_.sendSingleDatagram(cmd, idx, adp, ado, data, datalen, roundtrip);
    }

    size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) override {
        return master_.sendMultiDatagram(specs, count);
    }

    bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                            RxDatagram& out) override {
        return master_.waitForResponseIdx(idx, timeout_ms, out);
    }

    uint8_t allocIdx() override {
        return master_.allocIdx();
    }

    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return Master::adpForSlaveIndex(slave_index);
    }

private:
    Master& master_;
};

// ============================================================================
// MasterFaultTransport — adapts Master to IFaultTransport
// ============================================================================

class MasterFaultTransport : public IFaultTransport {
public:
    explicit MasterFaultTransport(Master& master) : master_(master) {}

    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size) override {
        return master_.readRegister(SlaveAddress(slave_index), reg_addr, data, size, 50);
    }

    bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                       const void* data, uint16_t size) override {
        return master_.writeRegister(SlaveAddress(slave_index), reg_addr, data, size, 50);
    }

    uint64_t getTimestampMs() override {
        auto now = std::chrono::steady_clock::now();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count());
    }

    void delayMs(uint32_t ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

private:
    Master& master_;
};

// ============================================================================
// MasterSDOTransport — adapts Master to ISDOTransport
// ============================================================================

class Master::MasterSDOTransport : public ::EtherCAT::SDO::ISDOTransport {
public:
    explicit MasterSDOTransport(Master& master)
        : master_(master) {}

    bool sdoUpload(uint16_t slave_index, uint8_t* mbx_counter,
                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                   uint16_t index, uint8_t sub,
                   uint8_t* out, size_t out_cap, size_t* out_len,
                   bool diag_enabled = false,
                   unsigned int poll_interval_ms = 5,
                   unsigned int transaction_timeout_ms = 1000) override
    {
        uint16_t adp = Master::adpForSlaveIndex(slave_index);
        return master_.coeSdoUpload(adp, mbx_counter,
                                    mbx_wr_addr, mbx_wr_len,
                                    mbx_rd_addr, mbx_rd_len,
                                    index, sub, out, out_cap, out_len, diag_enabled,
                                    poll_interval_ms, transaction_timeout_ms);
    }

    bool sdoDownload(uint16_t slave_index, uint8_t* mbx_counter,
                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                     uint16_t index, uint8_t sub,
                     const uint8_t* data, size_t data_len,
                     bool diag_enabled = false,
                     unsigned int poll_interval_ms = 5,
                     unsigned int transaction_timeout_ms = 1000) override
    {
        uint16_t adp = Master::adpForSlaveIndex(slave_index);
        return master_.coeSdoDownload(adp, mbx_counter,
                                      mbx_wr_addr, mbx_wr_len,
                                      mbx_rd_addr, mbx_rd_len,
                                      index, sub, data, data_len, diag_enabled,
                                      poll_interval_ms, transaction_timeout_ms);
    }

    uint64_t getMicroseconds() override {
        return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds());
    }

private:
    Master& master_;
};


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
    ensureRxQueues();

    // Initialize per-master packet router (required for waiters used by this master)
    if (!packet_router_.init()) {
        TETHER_LOGE(TAG, "Failed to initialize master packet router");
    }

    // Create instance-based SDO transport (shared across all per-slave CoEManagers)
    sdo_transport_ = std::make_unique<MasterSDOTransport>(*this);

    // Create thin wrapper sub-managers so callers may use e.g. master.dc() immediately.
    // Sub-managers are lightweight wrappers that forward to global/free-function
    // implementations that operate on file-scoped DC/SDO/PDO state.
    pdo_transport_ = std::make_unique<MasterPDOTransport>(*this);
    pdo_    = std::make_unique<PDOManager>(*pdo_transport_);
    logical_addr_mgr_ = std::make_unique<LogicalAddressManager>(*pdo_transport_);
    pdo_->setLogicalAddressManager(logical_addr_mgr_.get());
    dc_     = std::make_unique<DCManager>(*this);
    foe_    = std::make_unique<FoEManager>(*this);
    voe_    = std::make_unique<VoEManager>(*this);
    eoe_    = std::make_unique<EoEManager>(*this);
    fault_transport_ = std::make_unique<MasterFaultTransport>(*this);
    faults_ = std::make_unique<FaultDetector>(*fault_transport_);

    // Register this instance in the global list so host helpers may locate it
    {
        std::lock_guard<std::mutex> lg(g_master_list_mutex);
        g_master_list.push_back(this);
    }
}

Master::~Master()
{
    // Unregister this instance from the global list first
    {
        std::lock_guard<std::mutex> lg(g_master_list_mutex);
        auto it = std::find(g_master_list.begin(), g_master_list.end(), this);
        if (it != g_master_list.end()) g_master_list.erase(it);
    }

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
    iface_ptr_ = &iface;  // Store original pointer for identity comparison
    std::memcpy(src_mac_, src_mac, 6);
    ensureRxQueues();

    // Initialize per-slave CoEManagers
    for (size_t i = 0; i < sdo_managers_.size(); ++i) {
        if (!sdo_managers_[i]->init()) {
            TETHER_LOGW(TAG, "SDO subsystem failed to initialize for slave %zu", i);
        }
    }
    if (!sdo_managers_.empty()) {
        TETHER_LOGI(TAG, "SDO subsystem initialized for %zu slave(s)", sdo_managers_.size());
    }

    running_.store(true, std::memory_order_release);

    TETHER_LOGI(TAG, "Master started");
}

void Master::stop()
{
    requestCancel();
    packet_router_.cancel();
    stopMotionControlLoop();
    running_.store(false, std::memory_order_release);

    // Shutdown per-slave CoEManagers
    for (auto& mgr : sdo_managers_) {
        if (mgr) mgr->deinit();
    }
}

void Master::requestCancel()
{
    cancel_requested_.store(true, std::memory_order_release);
}

bool Master::isCancelRequested() const
{
    return cancel_requested_.load(std::memory_order_acquire);
}

void Master::clearCancel()
{
    cancel_requested_.store(false, std::memory_order_release);
    packet_router_.clearCancel();
}

bool Master::isRunning() const
{
    return running_.load(std::memory_order_acquire);
}

void Master::setMotionControlCallback(MotionControlCallback callback)
{
    motion_control_callback_ = std::move(callback);
}

bool Master::startRealtimeMotionControlLoop()
{
    return startRealtimeMotionControlLoop(RealtimeMotionLoopConfig{});
}

bool Master::startRealtimeMotionControlLoop(const RealtimeMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = std::make_unique<RealtimeMotionControlLoop>(wrapped_callback, config, dc_.get());
    return motion_control_loop_->start();
}

bool Master::startPollingMotionControlLoop()
{
    return startPollingMotionControlLoop(PollingMotionLoopConfig{});
}

bool Master::startPollingMotionControlLoop(const PollingMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = std::make_unique<PollingMotionControlLoop>(wrapped_callback, config, dc_.get());
    return motion_control_loop_->start();
}

void Master::stopMotionControlLoop()
{
    if (motion_control_loop_) {
        motion_control_loop_->stop();
        motion_control_loop_.reset();
    }
}

bool Master::isMotionControlLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}

// ============================================================================
// Queue-mode RT loop
// ============================================================================

bool Master::startQueueModeLoop()
{
    return startQueueModeLoop(RealtimeMotionLoopConfig{});
}

bool Master::startQueueModeLoop(const RealtimeMotionLoopConfig& config)
{
    if (pdo_->getMode() != PDOMode::Queue) {
        TETHER_LOGE(TAG, "PDOManager is not in Queue mode; call configureQueueMode() first");
        return false;
    }

    stopMotionControlLoop();  // Ensure no other loop is running
    clearCancel();
    motion_control_loop_ = std::make_unique<QueueMotionControlLoop>(pdo_.get(), config, dc_.get());
    return motion_control_loop_->start();
}

void Master::stopQueueModeLoop()
{
    stopMotionControlLoop();
}

bool Master::isQueueModeLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}




// ============================================================================
// Source MAC
// ============================================================================

const uint8_t* Master::getSrcMac() const { return src_mac_; }

const NetworkInterface* Master::networkInterface() const { return &iface_; }

Master* Master::findByNetworkInterface(const NetworkInterface* iface)
{
    std::lock_guard<std::mutex> lg(g_master_list_mutex);
    for (auto m : g_master_list) {
        if (m && m->iface_ptr_ == iface) return m;
    }
    return nullptr;
}




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
    return Raw::coe_sdo_upload(*this, adp, inout_mbx_cnt,
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
    return Raw::coe_sdo_download(*this, adp, inout_mbx_cnt,
                                 mbx_wr_addr, mbx_wr_len,
                                 mbx_rd_addr, mbx_rd_len,
                                 index, sub, data, data_len, diag_enabled,
                                 poll_interval_ms, transaction_timeout_ms);
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
::EtherCAT::CoE::CoEManager& Master::sdoManager(uint16_t slave_index) {
    if (slave_index >= sdo_managers_.size()) {
        // Lazily expand the vector if needed
        size_t old_size = sdo_managers_.size();
        sdo_managers_.resize(slave_index + 1);
        for (size_t i = old_size; i <= slave_index; ++i) {
            sdo_managers_[i] = std::make_unique<::EtherCAT::CoE::CoEManager>(
                static_cast<uint16_t>(i), *sdo_transport_);
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
    s.rx_frame_count = rx_frame_count_;
    s.rx_queue_sent  = rx_queue_sent_;
    s.rx_flushed     = total_flushed_;
    s.flush_calls    = flush_calls_;
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
    total_flushed_ += static_cast<uint32_t>(flushed);
    if (flushed) flush_calls_++;
#else
    (void)flushed;
#endif
}

} // namespace EtherCAT
