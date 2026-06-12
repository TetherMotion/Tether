/**
 * @file EtherCATMaster.cpp
 * @brief EtherCATMaster class implementation
 *
 * Core lifecycle, motion-control loops, frame parsing and sub-manager accessors.
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/EtherCATRealtimeLoop.hpp"
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

// Global debug flag for ethercat-statemachine (shared with EtherCATSlave)
extern bool g_debug_statemachine;

// Global debug flags for tx/rx packet logging (shared with EtherCATSlave)
extern bool g_debug_tx_packets;
extern bool g_debug_rx_packets;

// Global debug flags for PDO logging (shared with PDOManager)
extern bool g_debug_rx_pdo;
extern bool g_debug_tx_pdo;

// Global registry of EtherCATMaster instances (host-only helper). This
// allows host-side helpers (examples) to find the master associated with
// a NetworkInterface pointer.
static std::mutex g_master_list_mutex;
static std::vector<EtherCATMaster*> g_master_list;

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
    RealtimeMotionControlLoop(EtherCATMaster::MotionControlCallback callback,
                              EtherCATMaster::RealtimeMotionLoopConfig config,
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
            EtherCATRealtimeLoop::Config::defaults(config.cycle_period_us, config.sync_interval_cycles))
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
    EtherCATMaster::MotionControlCallback callback_;
    double dt_seconds_;
    EtherCATRealtimeLoop loop_;
};

class PollingMotionControlLoop final : public IMotionControlLoop {
public:
    PollingMotionControlLoop(EtherCATMaster::MotionControlCallback callback,
                             EtherCATMaster::PollingMotionLoopConfig config,
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
    EtherCATMaster::MotionControlCallback callback_;
    EtherCATMaster::PollingMotionLoopConfig config_;
    EtherCAT::DCManager* dc_manager_{nullptr};
    std::atomic<bool> running_{false};
    std::thread thread_;
};


// ============================================================================
// Utility: getECStateName
// ============================================================================

const char* EtherCATMaster::getECStateName(uint8_t state)
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

// adpForSlaveIndex is defined inline in EtherCATMaster.hpp

// ============================================================================
// MasterPDOTransport — adapts EtherCATMaster to IPDOTransport
// ============================================================================

class MasterPDOTransport : public IPDOTransport {
public:
    explicit MasterPDOTransport(EtherCATMaster& master) : master_(master) {}

    bool writeRegister(uint16_t adp, uint16_t ado,
                       const void* data, uint16_t len,
                       unsigned int timeout_ms) override {
        return master_.writeRegister(EtherCATMaster::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool readRegister(uint16_t adp, uint16_t ado,
                      void* data, uint16_t len,
                      unsigned int timeout_ms) override {
        return master_.readRegister(EtherCATMaster::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool sendSingleDatagram(Command cmd, uint8_t idx,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip) override {
        return master_.sendSingleDatagram(cmd, idx, adp, ado, data, datalen, roundtrip);
    }

    bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                            RxDatagram& out) override {
        return master_.waitForResponseIdx(idx, timeout_ms, out);
    }

    uint8_t allocIdx() override {
        return master_.allocIdx();
    }

    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return EtherCATMaster::adpForSlaveIndex(slave_index);
    }

private:
    EtherCATMaster& master_;
};

// ============================================================================
// MasterFaultTransport — adapts EtherCATMaster to IFaultTransport
// ============================================================================

class MasterFaultTransport : public IFaultTransport {
public:
    explicit MasterFaultTransport(EtherCATMaster& master) : master_(master) {}

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
    EtherCATMaster& master_;
};

// ============================================================================
// MasterSDOTransport — adapts EtherCATMaster to ISDOTransport
// ============================================================================

class EtherCATMaster::MasterSDOTransport : public ::EtherCAT::SDO::ISDOTransport {
public:
    explicit MasterSDOTransport(EtherCATMaster& master, ::EtherCAT::SDO::SDOManager* mgr = nullptr)
        : master_(master), mgr_(mgr) {}

    void setManager(::EtherCAT::SDO::SDOManager* mgr) { mgr_ = mgr; }

    bool sdoUpload(uint16_t slave_index, uint8_t* mbx_counter,
                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                   uint16_t index, uint8_t sub,
                   uint8_t* out, size_t out_cap, size_t* out_len) override
    {
        uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
        bool diag = (mgr_ && mgr_->isDiagEnabled());
        return master_.coeSdoUpload(adp, mbx_counter,
                                    mbx_wr_addr, mbx_wr_len,
                                    mbx_rd_addr, mbx_rd_len,
                                    index, sub, out, out_cap, out_len, diag);
    }

    bool sdoDownload(uint16_t slave_index, uint8_t* mbx_counter,
                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                     uint16_t index, uint8_t sub,
                     const uint8_t* data, size_t data_len) override
    {
        uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
        bool diag = (mgr_ && mgr_->isDiagEnabled());
        return master_.coeSdoDownload(adp, mbx_counter,
                                      mbx_wr_addr, mbx_wr_len,
                                      mbx_rd_addr, mbx_rd_len,
                                      index, sub, data, data_len, diag);
    }

    uint64_t getMicroseconds() override {
        return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds());
    }

private:
    EtherCATMaster& master_;
    ::EtherCAT::SDO::SDOManager* mgr_;
};


// ============================================================================
// Constructor / Destructor
// ============================================================================

EtherCATMaster::EtherCATMaster()
    : EtherCATMaster(Config{})
{
}

EtherCATMaster::EtherCATMaster(const Config& config)
    : config_(config)
{
    ensureRxQueues();

    // Initialize per-master packet router (required for waiters used by this master)
    if (!packet_router_.init()) {
        TETHER_LOGE(TAG, "Failed to initialize master packet router");
    }

    // Create instance-based SDO manager
    auto* transport_ptr = new MasterSDOTransport(*this);
    sdo_transport_.reset(transport_ptr);
    sdo_manager_ = std::make_unique<::EtherCAT::SDO::SDOManager>(*sdo_transport_);
    transport_ptr->setManager(sdo_manager_.get());

    // Create thin wrapper sub-managers so callers may use e.g. master.dc() immediately.
    // Sub-managers are lightweight wrappers that forward to global/free-function
    // implementations that operate on file-scoped DC/SDO/PDO state.
    pdo_transport_ = std::make_unique<MasterPDOTransport>(*this);
    pdo_    = std::make_unique<PDOManager>(*pdo_transport_);
    logical_addr_mgr_ = std::make_unique<LogicalAddressManager>(*pdo_transport_);
    pdo_->setLogicalAddressManager(logical_addr_mgr_.get());
    sdo_manager_->setPDOManager(pdo_.get());
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

EtherCATMaster::~EtherCATMaster()
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

void EtherCATMaster::start(const NetworkInterface& iface, const uint8_t src_mac[6])
{
    iface_ = iface;
    iface_ptr_ = &iface;  // Store original pointer for identity comparison
    std::memcpy(src_mac_, src_mac, 6);
    ensureRxQueues();

    // Initialize the instance-based SDO subsystem
    if (!sdo_manager_->init()) {
        TETHER_LOGW(TAG, "SDO subsystem failed to initialize");
    } else {
        TETHER_LOGI(TAG, "SDO subsystem initialized");
    }

    running_.store(true, std::memory_order_release);

    TETHER_LOGI(TAG, "Master started");
}

void EtherCATMaster::stop()
{
    stopMotionControlLoop();
    running_.store(false, std::memory_order_release);

    // Shutdown instance-based SDO subsystem
    if (sdo_manager_) {
        sdo_manager_->deinit();
    }
}

bool EtherCATMaster::isRunning() const
{
    return running_.load(std::memory_order_acquire);
}

void EtherCATMaster::setMotionControlCallback(MotionControlCallback callback)
{
    motion_control_callback_ = std::move(callback);
}

bool EtherCATMaster::startRealtimeMotionControlLoop()
{
    return startRealtimeMotionControlLoop(RealtimeMotionLoopConfig{});
}

bool EtherCATMaster::startRealtimeMotionControlLoop(const RealtimeMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    motion_control_loop_ = std::make_unique<RealtimeMotionControlLoop>(motion_control_callback_, config, dc_.get());
    return motion_control_loop_->start();
}

bool EtherCATMaster::startPollingMotionControlLoop()
{
    return startPollingMotionControlLoop(PollingMotionLoopConfig{});
}

bool EtherCATMaster::startPollingMotionControlLoop(const PollingMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    motion_control_loop_ = std::make_unique<PollingMotionControlLoop>(motion_control_callback_, config, dc_.get());
    return motion_control_loop_->start();
}

void EtherCATMaster::stopMotionControlLoop()
{
    if (motion_control_loop_) {
        motion_control_loop_->stop();
        motion_control_loop_.reset();
    }
}

bool EtherCATMaster::isMotionControlLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}




// ============================================================================
// Source MAC
// ============================================================================

const uint8_t* EtherCATMaster::getSrcMac() const { return src_mac_; }

const NetworkInterface* EtherCATMaster::networkInterface() const { return &iface_; }

EtherCATMaster* EtherCATMaster::findByNetworkInterface(const NetworkInterface* iface)
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

bool EtherCATMaster::coeSdoUpload(uint16_t adp, uint8_t* inout_mbx_cnt,
                                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                                   uint16_t index, uint8_t sub,
                                   uint8_t* out, size_t out_cap, size_t* out_len,
                                   bool diag_enabled)
{
    return Raw::coe_sdo_upload(*this, adp, inout_mbx_cnt,
                               mbx_wr_addr, mbx_wr_len,
                               mbx_rd_addr, mbx_rd_len,
                               index, sub, out, out_cap, out_len, diag_enabled);
}

bool EtherCATMaster::coeSdoDownload(uint16_t adp, uint8_t* inout_mbx_cnt,
                                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                                     uint16_t index, uint8_t sub,
                                     const uint8_t* data, size_t data_len,
                                     bool diag_enabled)
{
    return Raw::coe_sdo_download(*this, adp, inout_mbx_cnt,
                                 mbx_wr_addr, mbx_wr_len,
                                 mbx_rd_addr, mbx_rd_len,
                                 index, sub, data, data_len, diag_enabled);
}

// ============================================================================
// Test hooks
// ============================================================================

void EtherCATMaster::setAprdTestCallback(AprdTestCb cb) { aprd_cb_ = std::move(cb); }
void EtherCATMaster::setApwrTestCallback(ApwrTestCb cb) { apwr_cb_ = std::move(cb); }

void EtherCATMaster::pushAprdResponse(bool success, uint16_t adp, uint16_t ado,
                                       const void* data, uint16_t len)
{
    AprdResponse r;
    r.success = success; r.adp = adp; r.ado = ado;
    if (data && len > 0)
        r.data.assign(reinterpret_cast<const uint8_t*>(data),
                      reinterpret_cast<const uint8_t*>(data) + len);
    aprd_responses_.push_back(std::move(r));
}

void EtherCATMaster::clearAprdResponses() { aprd_responses_.clear(); }

bool EtherCATMaster::wasFaultDiagnosed(uint16_t slave_index) const
{
    std::lock_guard<std::mutex> _lg(m_diag_mutex_);
    return m_diagnosed_slaves_.find(slave_index) != m_diagnosed_slaves_.end();
}

// ============================================================================
// Sub-manager accessors
// ============================================================================

PDOManager&    EtherCATMaster::pdo()    { return *pdo_; }
LogicalAddressManager& EtherCATMaster::logicalAddressManager() { return *logical_addr_mgr_; }
::EtherCAT::SDO::SDOManager& EtherCATMaster::sdoManager() { return *sdo_manager_; }
DCManager&     EtherCATMaster::dc()     { return *dc_; }
FoEManager&    EtherCATMaster::foe()    { return *foe_; }
VoEManager&    EtherCATMaster::voe()    { return *voe_; }
EoEManager&    EtherCATMaster::eoe()    { return *eoe_; }
FaultDetector& EtherCATMaster::faults() { return *faults_; }

ConditionalPacketRouter& EtherCATMaster::packetRouter() { return packet_router_; }

Tether::Platform::MessageQueue<RxDatagram>* EtherCATMaster::rxQueue()
{
    return rx_queue_.get();
}

Tether::Platform::MessageQueue<RxDatagram>* EtherCATMaster::txpdoRxQueue()
{
    return txpdo_rx_queue_.get();
}

// ============================================================================
// Statistics
// ============================================================================

#if TETHER_ENABLE_ETHERCAT_STATS
EtherCATMaster::Stats EtherCATMaster::getStats() const
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

void EtherCATMaster::ensureRxQueues()
{
    if (!rx_queue_)
        rx_queue_ = std::make_unique<Tether::Platform::MessageQueue<RxDatagram>>(
            config_.rx_queue_depth);
    if (!txpdo_rx_queue_)
        txpdo_rx_queue_ = std::make_unique<Tether::Platform::MessageQueue<RxDatagram>>(
            config_.txpdo_queue_depth);
}

void EtherCATMaster::flushRxQueue()
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
