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

// TX retry constants
static constexpr int       kMaxTxRetries   = ECAT_TX_MAX_RETRIES;
static constexpr uint32_t  kTxRetryDelayUs = ECAT_TX_RETRY_DELAY_US;

class IMotionControlLoop {
public:
    virtual ~IMotionControlLoop() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual void setShutdownDebug(bool) {}
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

    void setShutdownDebug(bool enabled) override {
        loop_.setShutdownDebug(enabled);
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

    void setShutdownDebug(bool enabled) override {
        loop_.setShutdownDebug(enabled);
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

    size_t preRegisterResponseWaiter(uint8_t idx,
                                     uint8_t* buffer, size_t buffer_size) override {
        return master_.preRegisterResponseWaiter(idx, buffer, buffer_size);
    }

    bool waitForPreRegistered(size_t slot, unsigned int timeout_ms,
                              RxDatagram& out) override {
        WaitResult wr = master_.waitForPreRegistered(slot, timeout_ms);
        if (!wr.success) return false;
        out.idx = wr.idx; out.cmd = wr.cmd; out.adp = wr.adp;
        out.ado = wr.ado;
        out.datalen = static_cast<uint16_t>(wr.data_length);
        out.wkc = wr.wkc;
        return true;
    }

    uint8_t allocIdx() override {
        return master_.allocIdx();
    }

    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return Master::adpForSlaveIndex(slave_index);
    }

    bool isCancelRequested() const override {
        return master_.isCancelRequested();
    }

    size_t maxEtherCATPayloadPerFrame() const override {
        return master_.maxEtherCATPayloadPerFrame();
    }

    // ---- Cyclic fast path ----
    bool supportsCyclicFastPath() const override {
        return master_.supportsCyclicFastPath();
    }
    uint64_t cyclicSlotToken(uint8_t slot) override {
        return master_.cyclicSlotToken(slot);
    }
    bool sendCyclicDatagram(Command cmd, uint8_t slot,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip) override {
        return master_.sendCyclicDatagram(cmd, slot, adp, ado,
                                          data, datalen, roundtrip);
    }
    bool waitCyclicSlot(uint8_t slot, uint64_t token,
                        uint32_t timeout_ns, RxDatagram& out) override {
        return master_.waitCyclicSlot(slot, token, timeout_ns, out);
    }
    bool waitCyclicSlotView(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns,
                            CyclicSlotView& out) override {
        return master_.waitCyclicSlotView(slot, token, timeout_ns, out);
    }
    uint32_t waitCyclicSlotMask(uint32_t slot_mask, const uint64_t* tokens,
                                uint32_t timeout_ns,
                                CyclicSlotView* views) override {
        return master_.waitCyclicSlotMask(slot_mask, tokens, timeout_ns,
                                          views);
    }
    uint8_t* acquireCyclicTxFrame() override {
        return master_.acquireCyclicTxFrame();
    }
    void composeCyclicHeader(uint8_t* frame, Command cmd,
                             uint8_t slot, uint16_t adp,
                             uint16_t ado, uint16_t datalen,
                             bool roundtrip) override {
        master_.composeCyclicHeader(frame, cmd, slot, adp, ado,
                                    datalen, roundtrip);
    }
    bool sendCyclicFrame(uint32_t frame_len) override {
        return master_.sendCyclicFrame(frame_len);
    }
    ICyclicChannel* cyclicChannel() override {
        return master_.cyclicChannel();
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

    bool readSlaveRegister(uint16_t slave_index, uint16_t reg_addr,
                           void* out, uint16_t len,
                           unsigned int timeout_ms) override {
        return master_.readRegister(SlaveAddress(slave_index), reg_addr, out, len, timeout_ms);
    }

    uint32_t lastAbortCode() const override {
        return master_.lastCoeSdoAbortCode();
    }

    bool isCancelRequested() const override {
        return master_.isCancelRequested();
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

    // Create instance-based SDO transport (shared across all per-slave CoEManagers)
    sdo_transport_ = std::make_unique<MasterSDOTransport>(*this);

    // Create CoE SDO mailbox channel
    coe_sdo_channel_ = std::make_unique<Raw::CoeSDOChannel>();

    // Create thin wrapper sub-managers so callers may use e.g. master.dc() immediately.
    // Sub-managers are lightweight wrappers that forward to global/free-function
    // implementations that operate on file-scoped DC/SDO/PDO state.
    pdo_transport_ = std::make_unique<MasterPDOTransport>(*this);
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
    fault_transport_ = std::make_unique<MasterFaultTransport>(*this);
    faults_ = std::make_unique<FaultDetector>(*fault_transport_);
    faults_->setPrefixProvider(
        [this](uint16_t i) { return slaveLogPrefix(i); });
    status_poller_ = std::make_unique<SlaveStatusPoller>(*fault_transport_);
    slave_supervisor_ = std::make_unique<SlaveSupervisor>(*this);

#ifdef __linux__
    // Cyclic-slot deposit notification: the RX parser writes to this fd
    // after depositing a reserved-idx datagram so a cyclic thread blocked
    // in ppoll() wakes immediately.  Non-fatal if unavailable.
    cyclic_notify_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
#endif
}

Master::~Master()
{
    stop();

    // Clean up per-master packet router
    packet_router_.shutdown();

#ifdef __linux__
    if (cyclic_notify_fd_ >= 0) {
        close(cyclic_notify_fd_);
        cyclic_notify_fd_ = -1;
    }
#endif
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
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
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
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
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
// Cyclic executive — deadline-driven fast loop
// ============================================================================

bool Master::startCyclicLoop(const CyclicLoopConfig& config)
{
    stopAsyncLoop();   // mutually exclusive — the user picks the loop model
    stopCyclicLoop();
    stopMotionControlLoop();
    // A legacy DC realtime loop (dc().start()) drives its own PDO exchange —
    // it must not run alongside the cyclic exchange or two LRW streams share
    // the wire.  DC sync frames are driven by the executive's own DC task.
    if (dc_ && dc_->getState() == DC::DCState::Running) dc_->stop();

    // Optional DC auto-init — an explicit dc_config initializes DC here so
    // the app cannot forget the initialize-before-start ordering.  When DC
    // is already initialized the existing setup is kept.
    if (config.dc_config && !(dc_ && dc_->isInitialized())) {
        const uint16_t n = getDiscoveredSlaveCount();
        if (n > 0 && dc().init(*config.dc_config, n)) {
            TETHER_LOGI(TAG, "cyclic loop: DC auto-initialized "
                             "({} slaves)", n);
        } else {
            TETHER_LOGW(TAG, "cyclic loop: dc_config set but DC init failed "
                             "(slaves={}) — sync disabled", n);
        }
    }

    const bool dc_sync_enabled = config.enable_dc_synchronization ||
                                 config.dc_config.has_value();

    // Loud validation — these combinations silently no-op otherwise.
    if (config.motion_in_loop && !motion_control_callback_) {
        TETHER_LOGW(TAG, "cyclic loop: motion_in_loop is set but no motion "
                         "control callback is registered — the MotionControl "
                         "phase will be empty");
    }
    if (config.exchange_placement == ExchangePlacement::SplitLate &&
        config.motion_in_loop) {
        TETHER_LOGW(TAG, "cyclic loop: SplitLate collects after the "
                         "MotionControl phase — in-loop motion reads "
                         "previous-cycle inputs (intended only for external "
                         "motion sources)");
    }
    if (dc_sync_enabled && !(dc_ && dc_->isInitialized())) {
        TETHER_LOGW(TAG, "cyclic loop: DC synchronization enabled but DC was "
                         "never initialized — the DC task will idle (set "
                         "dc_config or call dc().init() first)");
    }

    clearCancel();

    CyclicExecutive::Config exec_cfg = config.exec;
    exec_cfg.cycle_period_us    = config.cycle_period_us;
    exec_cfg.dc_interval_cycles = config.sync_interval_cycles;
    rx_spin_ns_         = config.rx_spin_ns;
    slot_spin_ns_       = config.slot_spin_ns;
    slot_wait_fallback_ = config.slot_wait_fallback;

    // ---- Runtime CPU isolation (opt-in) --------------------------------
    // Claim CPUs for the RT threads before computing affinities.  Claims
    // are released by stopCyclicLoop() and at process exit.
    if (config.cpu_isolation.enabled) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        Tether::Platform::CpuIsolation::Spec spec;
        spec.prefer_isolated = config.cpu_isolation.prefer_isolated;
        spec.avoid_cpu0      = config.cpu_isolation.avoid_cpu0;
        spec.create_cpuset   = config.cpu_isolation.create_cpuset;
        spec.steer_irqs      = config.cpu_isolation.steer_irqs;

        spec.requested_cpu = config.cpu_isolation.cyclic_cpu;
        auto c = iso.claim(spec);
        if (c.valid()) {
            cyclic_cpu_claim_      = c.cpu;
            exec_cfg.cpu_affinity  = c.cpu;
        } else if (config.cpu_isolation.cyclic_cpu >= 0) {
            TETHER_LOGW(TAG, "CPU isolation: cyclic CPU {} claim denied — "
                             "running unpinned",
                        config.cpu_isolation.cyclic_cpu);
        }

        if (exec_cfg.dc_placement ==
            CyclicExecutive::DCPlacement::DedicatedThread) {
            spec.requested_cpu = config.cpu_isolation.dc_cpu;
            auto d = iso.claim(spec);
            if (d.valid()) {
                dc_cpu_claim_            = d.cpu;
                exec_cfg.dc_cpu_affinity = d.cpu;
            } else if (config.cpu_isolation.dc_cpu >= 0) {
                TETHER_LOGW(TAG, "CPU isolation: DC CPU {} claim denied",
                            config.cpu_isolation.dc_cpu);
            }
        }
    }

    // ---- Memory locking (each section independently opt-out-able) ------
    if (config.memory_lock.lock_all_process) {
        Tether::Platform::lockAllMemory();
    }
    if (exec_cfg.stack_prefault_bytes == 0 &&
        config.memory_lock.stack_prefault_bytes > 0) {
        exec_cfg.stack_prefault_bytes =
            config.memory_lock.stack_prefault_bytes;
    }
    if (!config.memory_lock.prefault_stack) {
        exec_cfg.stack_prefault_bytes = 0;
    }

    if (dc_sync_enabled &&
        exec_cfg.dc_placement == CyclicExecutive::DCPlacement::Disabled) {
        // Caller asked for DC sync but left placement at a disabled value —
        // default to the fault-isolated dedicated thread.
        exec_cfg.dc_placement = CyclicExecutive::DCPlacement::DedicatedThread;
    }
    if (!dc_sync_enabled) {
        exec_cfg.dc_placement = CyclicExecutive::DCPlacement::Disabled;
    }
    exec_cfg.jitter    = JitterConfig::defaults(config.cycle_period_us);
    exec_cfg.dc_jitter = JitterConfig::defaults(
        config.cycle_period_us * config.sync_interval_cycles);

    // Exchange: LRW process image via the reserved-slot fast path when the
    // transport supports it, falling back to the router path otherwise.
    // Split placements run the send half at Exchange and the collect half
    // in a later phase — the wire round-trip overlaps the work between.
    const bool split_exchange =
        config.exchange_placement != ExchangePlacement::Atomic;
    const uint32_t cyclic_rx_budget_ns =
        std::min<uint32_t>(200'000, config.cycle_period_us * 900);

    CyclicExecutive::TaskFn exchange_fn = [this, split_exchange,
                                           cyclic_rx_budget_ns]() -> bool {
        bool ok = true;
        if (pdo_) {
            if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
                if (split_exchange) {
                    ok = pdo_->cyclicSend(&process_image_,
                                          cyclic_rx_budget_ns);
                } else {
                    ok = pdo_->exchangeAllLRWCyclic(cyclic_rx_budget_ns,
                                                    &process_image_);
                }
            } else {
                ok = pdo_->exchangeAll();
            }
        }
        for (auto& g : pdo_groups_) {
            if (g.pdo) ok = g.pdo->exchangeAll() && ok;
        }
        return ok;
    };

    CyclicExecutive::TaskFn collect_fn;
    if (split_exchange) {
        collect_fn = [this]() -> bool {
            return !pdo_ || pdo_->cyclicCollect(&process_image_);
        };
    }

    CyclicExecutive::TaskFn dc_fn;
    if (dc_sync_enabled) {
        // Gate on isInitialized(), not Running — under the cyclic executive
        // there is no legacy DC loop to flip the state; initialize() arms
        // the slaves' sync units and this task emits the sync frames.  No
        // reference slave (init ran but nothing DC-capable) → idle quietly.
        dc_fn = [this]() -> bool {
            if (!dc_ || !dc_->isInitialized()) return true;
            EtherCATDC* dc = dc_->get();
            if (!dc || dc->referenceSlave() < 0) return true;
            return dc->sendSyncFrame();
        };
    }

    CyclicExecutive::TimeFunc time_fn;
    if (dc_ && dc_->isInitialized()) {
        time_fn = [this]() -> uint64_t {
            EtherCATDC* dc = dc_->get();
            return dc ? dc->getMasterTimeNs() : 0;
        };
    }

    // ---- Cyclic channel + process image + lockable sections ------------
    setupCyclicDatapath(config.wire_mode, config.image_mode,
                        config.shm_image_name, config.rx_spin_ns,
                        config.slot_spin_ns, config.slot_wait_fallback,
                        config.strict_wkc, config.memory_lock);

    cyclic_loop_ = std::make_unique<CyclicExecutive>(
        std::move(exchange_fn), std::move(dc_fn), std::move(time_fn), exec_cfg);

    if (config.motion_in_loop && motion_control_callback_) {
        const double dt = static_cast<double>(config.cycle_period_us) / 1e6;
        auto cb = motion_control_callback_;
        cyclic_loop_->addTask(TaskPhase::MotionControl,
            [this, cb, dt]() -> bool {
                if (cancel_requested_.load(std::memory_order_acquire)) {
                    return false;
                }
                return cb(dt);
            });
    }

    // Split-phase collect task — the send half already ran at Exchange;
    // collect lands at the configured placement so the wire round-trip
    // overlaps the phases in between.  Split honors the user-supplied
    // collect_phase (Q2); SplitLate pins Diagnostics.
    if (collect_fn) {
        TaskPhase collect_phase =
            (config.exchange_placement == ExchangePlacement::SplitLate)
                ? TaskPhase::Diagnostics
                : config.collect_phase;
        if (collect_phase == TaskPhase::PreExchange ||
            collect_phase == TaskPhase::Exchange) {
            TETHER_LOGW(TAG, "collect_phase must run after Exchange — "
                             "clamping to PostExchange");
            collect_phase = TaskPhase::PostExchange;
        }
        cyclic_loop_->addTask(collect_phase, std::move(collect_fn));
    }

    // The cyclic loop consumes the wire on its own deadline — a
    // triggerSend() here bumps an unconsumed counter; mark the consumer
    // so the producer gets a one-shot warning (Q27).
    process_image_.setSendConsumer(ProcessImage::SendConsumer::Cyclic);
    if (!cyclic_loop_->start()) {
        process_image_.setSendConsumer(ProcessImage::SendConsumer::None);
        return false;
    }
    return true;
}

void Master::stopCyclicLoop()
{
    // The shared datapath may belong to the async loop — only tear it
    // down when the cyclic loop owned it (its thread is dead by then).
    const bool owned_datapath = cyclic_loop_ != nullptr;
    if (cyclic_loop_) {
        cyclic_loop_->stop();
        cyclic_loop_.reset();
        process_image_.setSendConsumer(ProcessImage::SendConsumer::None);
    }
    // Release runtime CPU claims before the threads' affinity becomes
    // meaningless — claims are also auto-released at process exit.
    if (cyclic_cpu_claim_ >= 0 || dc_cpu_claim_ >= 0) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        if (cyclic_cpu_claim_ >= 0) { iso.release(cyclic_cpu_claim_); cyclic_cpu_claim_ = -1; }
        if (dc_cpu_claim_ >= 0)     { iso.release(dc_cpu_claim_);     dc_cpu_claim_ = -1; }
    }
    if (owned_datapath) {
        teardownCyclicDatapath();
    }
}

bool Master::isCyclicLoopRunning() const
{
    return cyclic_loop_ && cyclic_loop_->isRunning();
}

CyclicExecutive::Stats Master::getCyclicLoopStats() const
{
    return cyclic_loop_ ? cyclic_loop_->getStats() : CyclicExecutive::Stats{};
}

// ============================================================================
// Async send-on-change loop — external-producer RxPDO source
// ============================================================================

bool Master::startAsyncLoop(const AsyncLoopConfig& config)
{
    // Mutually exclusive with the cyclic loop — the user picks the model.
    // Async first: its thread blocks on process_image_'s send word, which
    // the shared datapath teardown reconfigures — the thread must be dead
    // before any teardown runs, or waitSend reads unmapped shm.
    stopAsyncLoop();
    stopCyclicLoop();
    stopMotionControlLoop();
    // A running legacy DC loop owns its own PDO exchange — it must not
    // share the wire with the async loop's sends.
    if (dc_ && dc_->getState() == DC::DCState::Running) dc_->stop();
    clearCancel();

    AsyncCyclicLoop::Config ecfg = config.exec;
    ecfg.collect_mode         = config.collect_mode;
    ecfg.collect_period_us    = config.collect_period_us;
    ecfg.min_send_interval_ns = config.min_send_interval_ns;
    ecfg.max_idle_ns          = config.max_idle_ns;
    ecfg.dc_interval_us       = config.enable_dc_synchronization
                                ? config.dc_interval_us : 0;
    rx_spin_ns_         = config.rx_spin_ns;
    slot_spin_ns_       = config.slot_spin_ns;
    slot_wait_fallback_ = config.slot_wait_fallback;

    // ---- Runtime CPU isolation (opt-in; async thread + optional DC) ---
    if (config.cpu_isolation.enabled) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        Tether::Platform::CpuIsolation::Spec spec;
        spec.prefer_isolated = config.cpu_isolation.prefer_isolated;
        spec.avoid_cpu0      = config.cpu_isolation.avoid_cpu0;
        spec.create_cpuset   = config.cpu_isolation.create_cpuset;
        spec.steer_irqs      = config.cpu_isolation.steer_irqs;
        spec.requested_cpu   = config.cpu_isolation.cyclic_cpu;
        auto c = iso.claim(spec);
        if (c.valid()) {
            async_cpu_claim_    = c.cpu;
            ecfg.cpu_affinity   = c.cpu;
        } else if (config.cpu_isolation.cyclic_cpu >= 0) {
            TETHER_LOGW(TAG, "CPU isolation: async CPU {} claim denied — "
                             "running unpinned",
                        config.cpu_isolation.cyclic_cpu);
        }
        if (ecfg.dc_interval_us > 0) {
            spec.requested_cpu = config.cpu_isolation.dc_cpu;
            auto d = iso.claim(spec);
            if (d.valid()) {
                dc_cpu_claim_          = d.cpu;
                ecfg.dc_cpu_affinity   = d.cpu;
            } else if (config.cpu_isolation.dc_cpu >= 0) {
                TETHER_LOGW(TAG, "CPU isolation: async DC CPU {} claim "
                                 "denied", config.cpu_isolation.dc_cpu);
            }
        }
    }

    // ---- Memory locking (each section independently opt-out-able) ------
    if (config.memory_lock.lock_all_process) {
        Tether::Platform::lockAllMemory();
    }
    if (ecfg.stack_prefault_bytes == 0 &&
        config.memory_lock.stack_prefault_bytes > 0) {
        ecfg.stack_prefault_bytes = config.memory_lock.stack_prefault_bytes;
    }
    if (!config.memory_lock.prefault_stack) {
        ecfg.stack_prefault_bytes = 0;
    }

    // ---- Cyclic channel + process image + lockable sections ------------
    setupCyclicDatapath(config.wire_mode, config.image_mode,
                        config.shm_image_name, config.rx_spin_ns,
                        config.slot_spin_ns, config.slot_wait_fallback,
                        config.strict_wkc, config.memory_lock);

    // Send: one RxPDO shot per trigger.  cyclicSend() falls back to the
    // atomic exchange when no logical address manager is configured.
    AsyncCyclicLoop::TaskFn send_fn =
        [this, rx_timeout = config.rx_timeout_ns]() -> bool {
            bool ok = true;
            if (pdo_) {
                if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
                    ok = pdo_->cyclicSend(&process_image_, rx_timeout);
                } else {
                    ok = pdo_->exchangeAll();
                }
            }
            for (auto& g : pdo_groups_) {
                if (g.pdo) ok = g.pdo->exchangeAll() && ok;
            }
            return ok;
        };
    AsyncCyclicLoop::TaskFn collect_fn = [this]() -> bool {
        return !pdo_ || pdo_->cyclicCollect(&process_image_);
    };
    AsyncCyclicLoop::TaskFn dc_fn;
    if (ecfg.dc_interval_us > 0) {
        // Same isInitialized() gate as the cyclic path — the async DC
        // thread owns the sync cadence; no legacy DC loop is required.
        dc_fn = [this]() -> bool {
            if (!dc_ || !dc_->isInitialized()) return true;
            EtherCATDC* dc = dc_->get();
            return dc ? dc->sendSyncFrame() : true;
        };
    }

    async_loop_ = std::make_unique<AsyncCyclicLoop>(
        process_image_, std::move(send_fn), std::move(collect_fn),
        std::move(dc_fn), AsyncCyclicLoop::TimeFunc{}, ecfg);
    // Mark the trigger word's consumer BEFORE start() so a triggerSend()
    // under the wrong loop model warns instead of silently dropping (Q27).
    process_image_.setSendConsumer(ProcessImage::SendConsumer::Async);
    if (!async_loop_->start()) {
        process_image_.setSendConsumer(ProcessImage::SendConsumer::None);
        return false;
    }
    return true;
}

void Master::stopAsyncLoop()
{
    // The shared datapath may belong to the cyclic loop — only tear it
    // down when the async loop owned it (its thread is dead by then).
    if (async_loop_) {
        async_loop_->stop();
        async_loop_.reset();
        process_image_.setSendConsumer(ProcessImage::SendConsumer::None);
        teardownCyclicDatapath();
    }
    if (async_cpu_claim_ >= 0) {
        Tether::Platform::CpuIsolation::instance().release(async_cpu_claim_);
        async_cpu_claim_ = -1;
    }
    if (dc_cpu_claim_ >= 0) {
        Tether::Platform::CpuIsolation::instance().release(dc_cpu_claim_);
        dc_cpu_claim_ = -1;
    }
}

bool Master::isAsyncLoopRunning() const
{
    return async_loop_ && async_loop_->isRunning();
}

AsyncCyclicLoop::Stats Master::getAsyncLoopStats() const
{
    return async_loop_ ? async_loop_->getStats() : AsyncCyclicLoop::Stats{};
}

// ============================================================================
// Shared cyclic datapath bring-up/teardown (cyclic + async loops)
// ============================================================================

void Master::setupCyclicDatapath(CyclicWireMode wire_mode,
                                 ImageMode image_mode,
                                 const std::string& shm_image_name,
                                 uint32_t rx_spin_ns,
                                 uint32_t slot_spin_ns,
                                 CyclicLoopConfig::SlotWaitFallback
                                     slot_fallback,
                                 bool strict_wkc,
                                 const MemoryLockConfig& memlock)
{
    rx_spin_ns_         = rx_spin_ns;
    slot_spin_ns_       = slot_spin_ns;
    slot_wait_fallback_ = slot_fallback;
    // The channel needs a raw AF_PACKET fd — only the direct-EtherCAT path
    // exposes one (iface_.receive/native_handle are stripped for VLAN and
    // absent under UDP encapsulation or polling transports).
    cyclic_channel_.reset();
    active_image_mode_ = ImageMode::Buffered;
    process_image_.configure({});

#ifdef __linux__
    if (iface_.receive && iface_.native_handle &&
        !isUdpEncapsulationEnabled()) {
        const int fd = static_cast<int>(
            reinterpret_cast<intptr_t>(iface_.native_handle));
        sockaddr_ll sll{};
        socklen_t sll_len = sizeof(sll);
        int ifindex = 0;
        if (::getsockname(fd, reinterpret_cast<sockaddr*>(&sll),
                          &sll_len) == 0) {
            ifindex = sll.sll_ifindex;
        }
        CyclicChannelConfig cc;
        cc.ifindex    = ifindex;
        cc.async_fd   = fd;
        cc.wire_mode  = wire_mode;
        cc.rx_spin_ns = rx_spin_ns;
        cyclic_channel_ = createCyclicChannel(cc);
        if (!cyclic_channel_) {
            TETHER_LOGW(TAG, "cyclic channel unavailable — using software "
                             "deposit path (no ring/BPF acceleration)");
        }
    }
#endif

    ImageMode mode = image_mode;
    if (mode == ImageMode::Rotating && !cyclic_channel_) {
        TETHER_LOGW(TAG, "Rotating image mode requires a cyclic channel — "
                         "falling back to Direct");
        mode = ImageMode::Direct;
    }
    if (mode != ImageMode::Buffered && pdo_ &&
        logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        const char* shm = shm_image_name.empty()
                        ? nullptr : shm_image_name.c_str();
        if (pdo_->configureProcessImage(process_image_, mode, shm)) {
            active_image_mode_ = mode;
            TETHER_LOGI(TAG, "process image active: mode={} rx={}B tx={}B{}",
                        static_cast<int>(mode),
                        process_image_.outputBytes(),
                        process_image_.inputBytes(),
                        process_image_.shmBacked() ? " [shm]" : "");
        } else {
            TETHER_LOGW(TAG, "process image configure failed — "
                             "buffered exchange");
        }
    }
    if (pdo_) {
        pdo_->setCyclicStrictWkc(strict_wkc);
    }

    // ---- Memory locking: image + cyclic buffers (opt-out sections) -----
    if (memlock.lock_image && process_image_.configured()) {
        // Lock whatever backing regions the configured mode uses.
        if (uint8_t* w = process_image_.outputWrite())
            Tether::Platform::lockMemory(w, process_image_.imageBytes());
        if (uint8_t* b = process_image_.inputWriteBank())
            Tether::Platform::lockMemory(b, process_image_.imageBytes());
    }
    if (memlock.lock_slots) {
        Tether::Platform::lockMemory(cyclic_slots_.data(),
                                     sizeof(cyclic_slots_));
        Tether::Platform::lockMemory(cyclic_tx_buf_,
                                     sizeof(cyclic_tx_buf_));
    }
}

void Master::teardownCyclicDatapath()
{
    // Drop the held input-view cookie before the channel dies.
    process_image_.configure({});
    // Release ring/bank cookies held by the cyclic slots, then tear down.
    if (cyclic_channel_) {
        for (auto& s : cyclic_slots_) {
            if (s.cookie >= 0) {
                cyclic_channel_->rxRelease(static_cast<uint32_t>(s.cookie));
            }
            s.cookie  = -1;
            s.payload = nullptr;
        }
    }
    cyclic_channel_.reset();
    active_image_mode_ = ImageMode::Buffered;
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
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
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
    group.transport = std::make_unique<MasterPDOTransport>(*this);
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
