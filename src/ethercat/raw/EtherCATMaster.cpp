/**
 * @file EtherCATMaster.cpp
 * @brief EtherCATMaster class implementation
 *
 * Consolidates the transport, runtime, and master-task code that was
 * previously spread across transport.cpp, runtime.cpp, and master.cpp
 * into EtherCATMaster member functions.
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
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
// Helper: determine whether SM0 is configured as write in SII (DEPRECATED/UNUSED).
// EtherCAT standard mandates: SM0=MbxIn(master→slave/write), SM1=MbxOut(slave→master/read).
// Some device SII EEPROMs incorrectly specify reversed directions, but the master
// must ignore those errors and always configure according to the standard.
// This function is retained for diagnostic purposes only.
// ============================================================================
static bool siiMailboxSM0IsWrite(EtherCATMaster& master, uint16_t slave_index)
{
    EtherCAT::SII::SIIData sii;
    if (!EtherCAT::SII::readSII(master, slave_index, sii) || sii.sm_count < 2)
        return false;
    return (sii.sync_managers[0].control_register & EtherCAT::PDO::SM_CTRL_DIR_WRITE) != 0;
}

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
// Slave Management
// ============================================================================

void EtherCATMaster::initSlaves(uint16_t count)
{
    slaves_.clear();
    slaves_.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
        auto s = std::make_unique<EtherCATSlave>(*this, i);
        // Initialize SII cache for each slave
        s->siiCache().init(siiReader(), i);
        slaves_.push_back(std::move(s));
    }
}

EtherCATSlave& EtherCATMaster::slave(uint16_t slave_index)
{
    if (slave_index < slaves_.size()) {
        return *slaves_[slave_index];
    }
    // Return sentinel for invalid index
    if (!non_existing_slave_) {
        non_existing_slave_ = std::make_unique<NonExistingSlave>(*this, slave_index);
    }
    // Update the index so the error message is correct
    non_existing_slave_ = std::make_unique<NonExistingSlave>(*this, slave_index);
    return *non_existing_slave_;
}

SII::SIIReader& EtherCATMaster::siiReader()
{
    if (!sii_reader_) {
        sii_reader_ = std::make_unique<SII::SIIReader>(*this);
    }
    return *sii_reader_;
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

    fmmu::fmmu_set_master(this);

    // Initialize the instance-based SDO subsystem
    if (!sdo_manager_->init()) {
        TETHER_LOGW(TAG, "SDO subsystem failed to initialize");
    } else {
        TETHER_LOGI(TAG, "SDO subsystem initialized");
    }

    running_.store(true, std::memory_order_release);
    master_thread_ = std::thread([this]() { masterTask(); });

    TETHER_LOGI(TAG, "Master task started");
}

void EtherCATMaster::stop()
{
    stopMotionControlLoop();
    running_.store(false, std::memory_order_release);
    if (master_thread_.joinable()) {
        master_thread_.join();
    }
    fmmu::fmmu_set_master(nullptr);

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

bool EtherCATMaster::resolvePhysicalSlaveIndex(SlaveAddress slave_address, uint16_t& slave_index_out)
{
    if (!slave_address.isPhysical()) {
        TETHER_LOGE(TAG, "Operation requires a physical slave address");
        return false;
    }

    slave_index_out = slave_address.slavePosition();
    return true;
}

// ============================================================================
// Frame handling
// ============================================================================

void EtherCATMaster::handleRxFrame(const uint8_t* frame, size_t length)
{
    parseEtherCATFrame(frame, length);
}

// ============================================================================
// Discovery
// ============================================================================

uint16_t EtherCATMaster::getDiscoveredSlaveCount() const
{
    return discovered_slave_count_.load(std::memory_order_acquire);
}

// ============================================================================
// AL state management
// ============================================================================

bool EtherCATMaster::requestSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t state_code)
{
    if (g_debug_statemachine) {
        uint8_t current_state_code = 0;
        readSlaveApplicationLayerState(slave_address, current_state_code);
        const char* current_state_name = getECStateName(current_state_code);
        const char* target_state_name = getECStateName(state_code);
        
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request: Slave %u                                  ║", slave_address.slavePosition());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Current State: %s (0x%02X)", current_state_name, current_state_code);
        TETHER_LOGI(TAG, "║  Target State:  %s (0x%02X)", target_state_name, state_code);
        TETHER_LOGI(TAG, "║  Action:        Writing AL_CONTROL register");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    bool result = writeRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_CONTROL), static_cast<uint16_t>(state_code));
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request Result: %s                                 ║", result ? "SUCCESS" : "FAILED");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return result;
}

bool EtherCATMaster::readSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t& state_code)
{
    uint16_t application_layer_status = 0;
    if (!readRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_STATUS), application_layer_status, 200)) {
        return false;
    }

    state_code = static_cast<uint8_t>(Raw::le16_to_host(application_layer_status) & 0x0F);
    return true;
}

bool EtherCATMaster::transitionSlaveToPreOperational(SlaveAddress slave_address)
{
    return setPreopAndConfirm(slave_address.slavePosition());
}

bool EtherCATMaster::configureProcessDataSyncManagersFromSii(SlaveAddress slave_address)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "configureProcessDataSyncManagersFromSii: invalid slave index %u", slave_index);
        return false;
    }

    // Read full SII data (includes SM category with SM2/SM3)
    EtherCAT::SII::SIIData sii;
    bool sii_valid = EtherCAT::SII::readSII(*this, slave_index, sii);
    if (!sii_valid) {
        TETHER_LOGW(TAG, "configureProcessDataSyncManagersFromSii: SII read failed for slave %u, using fallback", slave_index);
    }

    auto* slave_configs = pdo_->slaveConfigs();
    bool configured_any = false;

    for (size_t i = 2; sii_valid && i < 4 && i < sii.sm_count; i++) {
        const auto& src = sii.sync_managers[i];
        auto& dst = slave_configs[slave_index].sm[i];

        if (src.phys_start_address == 0 && src.length == 0) {
            TETHER_LOGD(TAG, "SM%zu: SII has no data (addr=0 len=0), skipping", i);
            continue;
        }

        dst.phys_start_addr = src.phys_start_address;
        dst.length = src.length;
        dst.control = src.control_register;
        dst.enable = src.isEnabled();
        dst.type = static_cast<PDO::SyncManagerType>(src.sm_type);

        TETHER_LOGI(TAG, "SM%zu from SII: Addr=0x%04X Len=%u Ctrl=0x%02X Type=%s Enable=%s",
                 i, dst.phys_start_addr, dst.length, dst.control,
                 src.getTypeName(), dst.enable ? "yes" : "no");
        configured_any = true;
    }

    if (!configured_any) {
        TETHER_LOGW(TAG, "SII has no SM2/SM3 data for slave %u, trying HW registers", slave_index);

        for (uint8_t sm = 2; sm < 4; sm++) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t buf[8] = {0};
            if (readRegister(SlaveAddress(slave_index), base, buf, sizeof(buf), 200)) {
                uint16_t addr = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
                uint16_t len  = static_cast<uint16_t>(buf[2] | (buf[3] << 8));
                uint8_t  ctrl = buf[4];
                uint8_t  act  = buf[6];

                if (addr != 0) {
                    auto& dst = slave_configs[slave_index].sm[sm];
                    dst.phys_start_addr = addr;
                    dst.length = len;
                    dst.control = ctrl;
                    dst.enable = (act & 0x01) != 0;
                    dst.type = (sm == 2) ? PDO::SyncManagerType::ProcessOutput
                                         : PDO::SyncManagerType::ProcessInput;
                    TETHER_LOGI(TAG, "SM%u from HW regs: Addr=0x%04X Len=%u Ctrl=0x%02X Act=0x%02X",
                             sm, addr, len, ctrl, act);
                    // configured_any = true; // Not used
                }
            }
        }
    }

    if (slave_configs[slave_index].sm[2].phys_start_addr == 0) {
        auto& sm2 = slave_configs[slave_index].sm[2];
        sm2 = PDO::SyncManagerConfig::process_output(0x1800, 0);
        TETHER_LOGW(TAG, "SM2 (RxPDO) using DEFAULT: Addr=0x%04X Ctrl=0x%02X", sm2.phys_start_addr, sm2.control);
    }
    if (slave_configs[slave_index].sm[3].phys_start_addr == 0) {
        auto& sm3 = slave_configs[slave_index].sm[3];
        sm3 = PDO::SyncManagerConfig::process_input(0x1C00, 0);
        TETHER_LOGW(TAG, "SM3 (TxPDO) using DEFAULT: Addr=0x%04X Ctrl=0x%02X", sm3.phys_start_addr, sm3.control);
    }

    pdo_->finalizeMapping(slave_index);

    for (uint8_t sm = 2; sm < 4; sm++) {
        const auto& cfg = slave_configs[slave_index].sm[sm];
        if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);

            uint8_t disable = 0x00;
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 6), &disable, 1, 200);

            uint16_t addr_le = Raw::host_to_le16(cfg.phys_start_addr);
            writeRegister(SlaveAddress(slave_index), base, &addr_le, 2, 200);

            uint16_t len_le = Raw::host_to_le16(cfg.length);
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 2), &len_le, 2, 200);

            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 4), &cfg.control, 1, 200);

            uint8_t activate = cfg.enable ? 0x01 : 0x00;
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 6), &activate, 1, 200);

            TETHER_LOGI(TAG, "Wrote SM%u to slave %u: Addr=0x%04X Len=%u Ctrl=0x%02X Act=0x%02X",
                     sm, slave_index, cfg.phys_start_addr, cfg.length, cfg.control, activate);
        }
    }

    slave_configs[slave_index].configured = true;
    return true;
}

bool EtherCATMaster::sendDatagram(Command cmd, uint8_t idx,
                                  SlaveAddress slave_address, RegisterAddress register_address,
                                  const void* data, uint16_t datalen,
                                  bool roundtrip)
{
    Command routed_command = cmd;
    if (slave_address.isLogical()) {
        switch (cmd) {
            case Command::APRD: routed_command = Command::FPRD; break;
            case Command::APWR: routed_command = Command::FPWR; break;
            case Command::APRW: routed_command = Command::FPRW; break;
            default: break;
        }
    }

    return sendSingleDatagram(routed_command, idx, slave_address.raw(), register_address.raw(), data, datalen, roundtrip);
}

bool EtherCATMaster::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = 1;
    const uint16_t adp = slave_address.raw();
    const uint16_t ado = register_address.raw();
    if (apwr_cb_) return apwr_cb_(adp, ado, data, len, timeout_ms);
    if (ado == Raw::EC_REG_EEPCTL && !aprd_responses_.empty()) return true;

    const uint8_t idx = allocIdx();
    RxDatagram resp{};
    size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
    if (slot >= TransactionRouter::kNumSlots) return false;

    if (!sendDatagram(Command::APWR, idx, slave_address, register_address, data, len, true)) {
        packet_router_.cancelPreRegistered(slot);
        return false;
    }

    WaitResult result = waitForPreRegistered(slot, timeout_ms);
    return result.success && result.wkc > 0;
}

bool EtherCATMaster::writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                   uint16_t value)
{
    const uint16_t little_endian_value = Raw::host_to_le16(value);
    return writeRegister(slave_address, register_address, &little_endian_value, sizeof(little_endian_value), 200);
}

bool EtherCATMaster::readRegister(SlaveAddress slave_address, RegisterAddress register_address,
                                  void* out, uint16_t len,
                                  unsigned int timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = 1;
    const uint16_t adp = slave_address.raw();
    const uint16_t ado = register_address.raw();
    // Debug: show whether a per-instance APRD test callback is present
    if (aprd_cb_) {
        TETHER_LOGD(TAG, "readRegister: using instance aprd_cb_");
        return aprd_cb_(adp, ado, out, len, timeout_ms);
    }

    // EEPROM status short-circuit for tests
    if (ado == 0x0502 && apwr_cb_) {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    }

    // Check queued test responses
    for (auto it = aprd_responses_.begin(); it != aprd_responses_.end(); ++it) {
        if (it->adp == adp && it->ado == ado) {
            AprdResponse resp = *it;
            aprd_responses_.erase(it);
            if (!resp.success) return false;
            if (out && !resp.data.empty()) {
                uint16_t copy_len =
                    static_cast<uint16_t>(std::min<size_t>(len, resp.data.size()));
                std::memcpy(out, resp.data.data(), copy_len);
            }
            return true;
        }
    }
    if (!aprd_responses_.empty()) {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    }

    const uint8_t idx = allocIdx();
    RxDatagram resp{};
    size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
    if (slot >= TransactionRouter::kNumSlots) return false;

    if (!sendDatagram(Command::APRD, idx, slave_address, register_address, nullptr, len, true)) {
        packet_router_.cancelPreRegistered(slot);
        return false;
    }

    WaitResult result = waitForPreRegistered(slot, timeout_ms);
    if (!result.success) return false;

    resp.datalen = static_cast<uint16_t>(result.data_length);
    if (resp.datalen < len) return false;
    if (out && len > 0) std::memcpy(out, resp.data, len);
    return result.wkc > 0;
}

// ============================================================================
// Wait helpers
// ============================================================================

bool EtherCATMaster::waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                         RxDatagram& out)
{
    PacketFilter filter = PacketFilter::byIndex(idx);
    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

bool EtherCATMaster::waitForResponseAdo(uint16_t ado, Command cmd,
                                         unsigned int timeout_ms,
                                         RxDatagram& out)
{
    PacketFilter filter{};
    filter.command   = cmd;
    filter.ado       = ado;
    filter.match_ado = true;

    WaitResult result = packet_router_.waitForPacket(
        filter, out.data, sizeof(out.data), timeout_ms);
    if (result.success) {
        out.idx = result.idx; out.cmd = result.cmd; out.adp = result.adp;
        out.ado = result.ado;
        out.datalen = static_cast<uint16_t>(result.data_length);
        out.wkc = result.wkc;
        return true;
    }
    return false;
}

size_t EtherCATMaster::preRegisterResponseWaiter(uint8_t idx,
                                                  uint8_t* buffer,
                                                  size_t buffer_size)
{
    PacketFilter filter = PacketFilter::byIndex(idx);
    return packet_router_.preRegisterWaiter(filter, buffer, buffer_size);
}

WaitResult EtherCATMaster::waitForPreRegistered(size_t slot, uint32_t timeout_ms)
{
    return packet_router_.waitForPreRegistered(slot, timeout_ms);
}

// ============================================================================
// Index allocation
// ============================================================================

uint8_t EtherCATMaster::allocIdx()
{
    uint8_t idx;
    do { idx = next_idx_.fetch_add(1, std::memory_order_relaxed); }
    while (idx == kFireAndForgetIdx);
    return idx;
}

// Mailbox override helpers (allow application to enforce XML-derived mailbox values)
struct MailboxOverride {
    bool enabled{false};
    uint16_t wr_addr{0};
    uint16_t wr_len{0};
    uint16_t rd_addr{0};
    uint16_t rd_len{0};
    uint16_t proto{0};
};

void EtherCATMaster::setMailboxOverride(SlaveAddress slave_address, uint16_t wr_addr, uint16_t wr_len,
                                       uint16_t rd_addr, uint16_t rd_len, uint16_t proto)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return;
    }

    std::lock_guard<std::mutex> _lg(m_mailbox_override_mutex_);
    if (slave_index >= PDO::kMaxPDOSlaves) return;
    if (m_mailbox_overrides_.size() < PDO::kMaxPDOSlaves) m_mailbox_overrides_.resize(PDO::kMaxPDOSlaves);
    auto& ov = m_mailbox_overrides_[slave_index];
    ov.enabled = true;
    ov.wr_addr = wr_addr;
    ov.wr_len = wr_len;
    ov.rd_addr = rd_addr;
    ov.rd_len = rd_len;
    ov.proto = proto;
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
// Packet debug printer
// ============================================================================

static void printEtherCATFrame(const uint8_t* frame, size_t length, bool is_tx, bool print_ethernet)
{
    using namespace Raw;
    const char* dir = is_tx ? "TX" : "RX";

    if (print_ethernet && length >= sizeof(EtherCAT::EthernetHeader)) {
        const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
        TETHER_LOGI("ec_pkt", "[%s] Ethernet: dst=%02X:%02X:%02X:%02X:%02X:%02X src=%02X:%02X:%02X:%02X:%02X:%02X etherType=0x%04X",
                    dir,
                    eth->dst[0], eth->dst[1], eth->dst[2], eth->dst[3], eth->dst[4], eth->dst[5],
                    eth->src[0], eth->src[1], eth->src[2], eth->src[3], eth->src[4], eth->src[5],
                    bswap16(eth->etherType_be));
    }

    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader)) {
        TETHER_LOGI("ec_pkt", "[%s] Frame too short for EtherCAT header (%u bytes)",
                    dir, static_cast<unsigned>(length));
        return;
    }

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(
        frame + sizeof(EtherCAT::EthernetHeader));
    const uint16_t ec_raw = le16_to_host(ec_hdr->raw_le);
    const uint16_t ec_len = ec_raw & 0x07FFu;
    const uint16_t ec_type = (ec_raw >> 12) & 0x0Fu;

    TETHER_LOGI("ec_pkt", "[%s] EtherCAT Frame: length=%u type=%u", dir, ec_len, ec_type);

    size_t offset = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
    size_t remaining = ec_len;
    uint8_t dg_idx = 0;

    while (remaining >= sizeof(EtherCAT::DatagramHeader) &&
           offset + sizeof(EtherCAT::DatagramHeader) <= length) {
        const auto* dg = reinterpret_cast<const EtherCAT::DatagramHeader*>(frame + offset);
        const uint16_t dg_len_flags = le16_to_host(dg->lenFlags_le);
        const uint16_t datalen = dg_len_flags & 0x07FFu;
        const bool more = (dg_len_flags & 0x8000u) != 0;
        const bool circulating = (dg_len_flags & 0x4000u) != 0;
        const uint16_t adp = le16_to_host(dg->adp_le);
        const uint16_t ado = le16_to_host(dg->ado_le);

        const size_t data_offset = offset + sizeof(EtherCAT::DatagramHeader);
        const size_t wkc_offset = data_offset + datalen;
        uint16_t wkc = 0;
        if (length >= wkc_offset + sizeof(uint16_t)) {
            wkc = le16_to_host(*reinterpret_cast<const uint16_t*>(frame + wkc_offset));
        }

        TETHER_LOGI("ec_pkt", "[%s]   Datagram[%u]: cmd=%s idx=0x%02X adp=0x%04X ado=0x%04X len=%u wkc=%u more=%s circulating=%s",
                    dir, dg_idx,
                    commandToString(dg->cmd),
                    dg->idx,
                    adp, ado,
                    datalen, wkc,
                    more ? "yes" : "no",
                    circulating ? "yes" : "no");

        if (datalen > 0 && length >= data_offset + datalen) {
            constexpr size_t kMaxHexDump = 64;
            const size_t dump_len = (datalen < kMaxHexDump) ? datalen : kMaxHexDump;
            char hexbuf[256];
            size_t pos = 0;
            for (size_t i = 0; i < dump_len && pos + 3 < sizeof(hexbuf); i++) {
                pos += std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "%02X ", frame[data_offset + i]);
            }
            if (datalen > kMaxHexDump) {
                pos += std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "...");
            }
            TETHER_LOGI("ec_pkt", "[%s]     Data (%u/%u bytes): %s",
                        dir, static_cast<unsigned>(dump_len), static_cast<unsigned>(datalen), hexbuf);
        }

        const size_t dg_total = sizeof(EtherCAT::DatagramHeader) + datalen + sizeof(uint16_t);
        if (remaining < dg_total) break;
        remaining -= dg_total;
        offset += dg_total;
        dg_idx++;

        if (!more) break;
    }
}

// ============================================================================
// Transport primitives
// ============================================================================

bool EtherCATMaster::sendRawFrame(const void* buf, size_t len)
{
    if (g_debug_tx_packets) {
        printEtherCATFrame(reinterpret_cast<const uint8_t*>(buf), len, true, false);
    }
    if (iface_.send)
        return iface_.send(reinterpret_cast<const uint8_t*>(buf), len);
    TETHER_LOGE(TAG, "No NetworkInterface registered!");
    return false;
}

bool EtherCATMaster::sendSingleDatagram(Command cmd, uint8_t idx,
                                         uint16_t adp, uint16_t ado,
                                         const void* data, uint16_t datalen,
                                         bool roundtrip)
{
    using namespace Raw;

    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    constexpr size_t kMinEthFrameNoFcs = 60;
    constexpr size_t kMaxEthFrameNoFcs = 1514;

    const size_t required_len =
        sizeof(EtherCATSingleDgramFrameHeader) + datalen + sizeof(uint16_t);
    if (required_len > kMaxEthFrameNoFcs) {
        TETHER_LOGE(TAG, "Datagram too big (datalen=%u required=%u)",
                    datalen, static_cast<unsigned>(required_len));
        return false;
    }

    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;
    uint8_t txbuf[kMaxEthFrameNoFcs] = {0};

    auto* hdr = reinterpret_cast<EtherCATSingleDgramFrameHeader*>(txbuf);
    std::memcpy(hdr->eth.dst, dst_mac, 6);
    std::memcpy(hdr->eth.src, src_mac_, 6);
    hdr->eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);

    const uint16_t payload_len =
        static_cast<uint16_t>(sizeof(EtherCATDatagramHeader) + datalen + sizeof(uint16_t));
    constexpr uint16_t type = 0x1;
    hdr->ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu) | ((type & 0x0Fu) << 12)));

    hdr->dg.cmd    = cmd;
    hdr->dg.idx    = idx;
    hdr->dg.adp_le = host_to_le16(adp);
    hdr->dg.ado_le = host_to_le16(ado);
    const uint16_t flags = roundtrip ? (1u << 14) : 0u;
    hdr->dg.lenFlags.raw_le =
        host_to_le16(static_cast<uint16_t>((datalen & 0x07FFu) | flags));
    hdr->dg.irq_le = host_to_le16(0);

    uint8_t* payload = txbuf + sizeof(EtherCATSingleDgramFrameHeader);
    if (datalen > 0) {
        if (data) std::memcpy(payload, data, datalen);
        else      std::memset(payload, 0, datalen);
    }
    *reinterpret_cast<uint16_t*>(payload + datalen) = host_to_le16(0);

    if (iface_.send) {
        if (g_debug_tx_packets) {
            printEtherCATFrame(txbuf, frame_len, true, false);
        }
        auto& clock = Tether::Platform::Clock::instance();
        int last_errno = 0;
        for (int retry = 0; retry <= kMaxTxRetries; retry++) {
            errno = 0;
            if (iface_.send(txbuf, frame_len)) return true;
            last_errno = errno;
#if TETHER_ENABLE_ETHERCAT_STATS
            tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
#endif
            const int64_t t0 = clock.getMicroseconds();
            while ((clock.getMicroseconds() - t0) <
                   static_cast<int64_t>(kTxRetryDelayUs)) {}
        }

        char msg[256];
        if (last_errno != 0) {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u retries=%d errno=%d:%s)",
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          kMaxTxRetries + 1,
                          last_errno,
                          std::strerror(last_errno));
        } else {
            std::snprintf(msg, sizeof(msg),
                          "NetworkInterface::send failed after retries (cmd=%s idx=%u adp=0x%04X ado=0x%04X datalen=%u frame_len=%u retries=%d errno=0)",
                          commandToString(cmd),
                          static_cast<unsigned>(idx),
                          static_cast<unsigned>(adp),
                          static_cast<unsigned>(ado),
                          static_cast<unsigned>(datalen),
                          static_cast<unsigned>(frame_len),
                          kMaxTxRetries + 1);
        }
        send_fail_log_.logLegacy(1, TAG, msg);
#if TETHER_ENABLE_ETHERCAT_STATS
        tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
#endif
        return false;
    }

    TETHER_LOGE(TAG, "No NetworkInterface available for send");
    return false;
}


void EtherCATMaster::resetIdx()
{
    next_idx_.store(0, std::memory_order_relaxed);
}

// ============================================================================
// Watchdog configuration
// ============================================================================

bool EtherCATMaster::configureWatchdogs(SlaveAddress slave_address,
                                         uint16_t pdi_timeout_100us,
                                         uint16_t pdata_timeout_100us)
{
    if (!writeRegister(slave_address, Raw::EC_REG_WD_DIV, static_cast<uint16_t>(0x09C2))) return false;
    if (!writeRegister(slave_address, Raw::EC_REG_WD_TIME_PDI, pdi_timeout_100us)) return false;
    if (!writeRegister(slave_address, Raw::EC_REG_WD_TIME_PDATA, pdata_timeout_100us)) return false;
    return true;
}

bool EtherCATMaster::disableWatchdogs(SlaveAddress slave_address)
{
    return configureWatchdogs(slave_address, 0, 0);
}

bool EtherCATMaster::readWatchdogStatus(SlaveAddress slave_address,
                                         uint8_t& wd_status,
                                         uint8_t& pdi_cnt,
                                         uint8_t& pdata_cnt)
{
    if (!readRegister(slave_address, Raw::EC_REG_WD_STATUS, wd_status, 200)) return false;
    if (!readRegister(slave_address, Raw::EC_REG_WD_CNT_PDI, pdi_cnt, 200)) return false;
    if (!readRegister(slave_address, Raw::EC_REG_WD_CNT_PDATA, pdata_cnt, 200)) return false;
    return true;
}

// ============================================================================
// SII / EEPROM — delegate to existing Raw:: functions for now
// ============================================================================

bool EtherCATMaster::siiReadString(uint16_t slave_index, uint16_t string_number,
                                    char* out, size_t out_cap)
{
    return Raw::sii_read_string(*this, slave_index, string_number, out, out_cap);
}

bool EtherCATMaster::configureMailboxFromSii(uint16_t slave_index,
                                              uint16_t* wr_addr, uint16_t* wr_len,
                                              uint16_t* rd_addr, uint16_t* rd_len,
                                              uint16_t* mbx_proto)
{
    return Raw::configure_mailbox_from_sii(*this, slave_index,
                                           wr_addr, wr_len, rd_addr, rd_len,
                                           mbx_proto);
}

bool EtherCATMaster::autoConfigureMailbox(SlaveAddress slave_address, Tether::Platform::LogLevel log_level)
{
    const char* local_tag = "autoMbox";
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    uint16_t adp = adpForSlaveIndex(slave_index);
    
    // Log start with requested verbosity
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "======================================================================\n  AUTO-CONFIGURING MAILBOX FOR SLAVE %u (ADP=0x%04X)\n======================================================================",
                    (unsigned)slave_index, adp);
    } else {
        TETHER_LOGI(local_tag, "Auto-configuring mailbox for slave %u...", (unsigned)slave_index);
    }
    
    // Step 1: Read mailbox configuration from SII
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[1/3] Reading mailbox configuration from SII EEPROM...");
    }
    
    bool sii_ok = configureMailboxFromSii(adp, &wr_addr, &wr_len, &rd_addr, &rd_len, &proto);
    
    if (!sii_ok) {
        TETHER_LOGE(local_tag, "Failed to read SII mailbox configuration for slave %u", 
                    (unsigned)slave_index);
        // Note: configureMailboxFromSii already sets defaults on failure, so we can continue
    }
    
    // Step 2: Apply mailbox override to master    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[2/3] Applying mailbox configuration to master override...\n      Send    (S→M, SM0): addr=0x%04X len=%u\n      Receive (M→S, SM1): addr=0x%04X len=%u\n      Protocols: 0x%04X",
                   rd_addr, (unsigned)rd_len, wr_addr, (unsigned)wr_len, proto);
        
        // Decode protocols for verbose logging
        std::string proto_str;
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_AOE)) proto_str += "AoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_EOE)) proto_str += "EoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_COE)) proto_str += "CoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_FOE)) proto_str += "FoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_SOE)) proto_str += "SoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_VOE)) proto_str += "VoE ";
        if (proto_str.empty()) proto_str = "(none)";
        
        TETHER_LOGD(local_tag, "      Supported: %s", proto_str.c_str());
    }
    
    setMailboxOverride(slave_address, wr_addr, wr_len, rd_addr, rd_len, proto);
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "      ✓ Master mailbox override configured");
    }
    
    // Step 3: Configure SDO subsystem mailbox
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[3/3] Configuring SDO subsystem mailbox...");
    }
    
    sdo_manager_->configureSlaveMailbox(slave_index, wr_addr, wr_len, rd_addr, rd_len);
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "      ✓ SDO subsystem mailbox configured");
    }
    
    // Step 4: Verify configuration was applied
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[Verification] Checking SDO subsystem mailbox configuration...");
        
        uint16_t verify_wr = 0, verify_wr_len = 0, verify_rd = 0, verify_rd_len = 0;
        bool verify_ok = sdo_manager_->getSlaveMailbox(slave_index, 
                                                        &verify_wr, &verify_wr_len, 
                                                        &verify_rd, &verify_rd_len);
        
        if (verify_ok) {
            bool match = (verify_wr == wr_addr && verify_wr_len == wr_len && 
                         verify_rd == rd_addr && verify_rd_len == rd_len);
            
            if (match) {
                TETHER_LOGD(local_tag, "      ✓ SDO subsystem mailbox verified: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                           verify_wr, (unsigned)verify_wr_len, verify_rd, (unsigned)verify_rd_len);
            } else {
                TETHER_LOGW(local_tag, "      ⚠ MISMATCH! SDO subsystem has Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u\n      Expected: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                           verify_wr, (unsigned)verify_wr_len, verify_rd, (unsigned)verify_rd_len,
                           wr_addr, (unsigned)wr_len, rd_addr, (unsigned)rd_len);
            }
        } else {
            TETHER_LOGE(local_tag, "      ✗ CRITICAL: SDO subsystem has NO mailbox configuration!\n      This indicates configureSlaveMailbox() failed.");
            return false;
        }
    }
    
    // Final success message
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "======================================================================\n  ✓ MAILBOX AUTO-CONFIGURATION COMPLETE FOR SLAVE %u\n======================================================================",
                    (unsigned)slave_index);
    } else {
        TETHER_LOGI(local_tag, "✓ Mailbox auto-configured for slave %u: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                    (unsigned)slave_index, wr_addr, (unsigned)wr_len, rd_addr, (unsigned)rd_len);
    }
    
    return true;
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

// ============================================================================
// Internal: frame parsing
// ============================================================================

void EtherCATMaster::parseEtherCATFrame(const uint8_t* frame, size_t length)
{
    if (g_debug_rx_packets) {
        printEtherCATFrame(frame, length, false, false);
    }

    using namespace Raw;  // for le16_to_host, Command, RxDatagram, etc.

#if TETHER_ENABLE_ETHERCAT_STATS
    rx_frame_count_++;
#endif

    // Use fully-qualified EtherCAT::EthernetHeader to avoid ambiguity
    // with Raw::EthernetHeader
    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader))
        return;

    const auto* eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(frame);
    if (bswap16(eth->etherType_be) != EtherCAT::kEtherTypeEtherCAT) return;

    const auto* ec_hdr = reinterpret_cast<const EtherCAT::FrameHeader*>(
        frame + sizeof(EtherCAT::EthernetHeader));
    const uint16_t ec_len = le16_to_host(ec_hdr->raw_le) & 0x07FFu;

    if (length < sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader) + ec_len)
        return;

    const size_t payload_offset = sizeof(EtherCAT::EthernetHeader) + sizeof(EtherCAT::FrameHeader);
    const auto* dg = reinterpret_cast<const EtherCAT::DatagramHeader*>(frame + payload_offset);

    const uint16_t ado     = le16_to_host(dg->ado_le);
    const uint16_t adp     = le16_to_host(dg->adp_le);
    const uint16_t datalen = le16_to_host(dg->lenFlags_le) & 0x07FFu;

    const size_t data_offset = payload_offset + sizeof(EtherCAT::DatagramHeader);
    const size_t wkc_offset  = data_offset + datalen;
    if (length < wkc_offset + sizeof(uint16_t)) return;

    const uint16_t wkc =
        le16_to_host(*reinterpret_cast<const uint16_t*>(frame + wkc_offset));

    RxDatagram msg{};
    msg.idx = dg->idx; msg.cmd = dg->cmd; msg.adp = adp; msg.ado = ado;
    msg.datalen = datalen; msg.wkc = wkc;
    if (datalen > 0)
        std::memcpy(msg.data, frame + data_offset,
                    std::min<size_t>(datalen, sizeof(msg.data)));

    if (dg->idx == kFireAndForgetIdx) {
        if (dg->cmd == Command::APRD && txpdo_rx_queue_)
            txpdo_rx_queue_->send(msg, 0);
    } else {
        size_t routed = packet_router_.routePacket(msg);
        if (routed == 0) {
            static uint32_t unrouted_count = 0;
            if (unrouted_count < 10) {
                TETHER_LOGW("ec_rx", "Unrouted pkt idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u",
                         dg->idx, (unsigned)dg->cmd, ado, adp, wkc);
            }
            unrouted_count++;
            if (rx_queue_ && rx_queue_->send(msg, 0)) {
#if TETHER_ENABLE_ETHERCAT_STATS
                rx_queue_sent_++;
#endif
            } else {
                static uint32_t drop_count = 0;
                if (drop_count < 10 || (drop_count % 500 == 0)) {
                    TETHER_LOGW("ec_rx", "RX queue full! Dropped idx=0x%02X cmd=0x%02X ado=0x%04X adp=0x%04X wkc=%u (total dropped: %u)",
                             dg->idx, (unsigned)dg->cmd, ado, adp, wkc, drop_count);
                }
                drop_count++;
            }
        }
    }
}

// ============================================================================
// Internal helpers: scan frame builder
// ============================================================================

static Raw::EtherCATScanFrame buildScanFrame(const uint8_t src_mac[6])
{
    using namespace Raw;
    EtherCATScanFrame frame{};
    const uint8_t dst[6] = {0x01,0x01,0x05,0x00,0x00,0x00};
    std::memcpy(frame.eth.dst, dst, 6);
    std::memcpy(frame.eth.src, src_mac, 6);
    frame.eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);
    constexpr uint16_t payload_len = 10+2+2, type = 0x1;
    frame.ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu)|((type & 0x0Fu)<<12)));
    frame.dg.cmd = Command::BRD;
    frame.dg.adp_le = host_to_le16(0);
    frame.dg.ado_le = host_to_le16(EC_REG_AL_STATUS);
    constexpr uint16_t datalen = 2, rt = (1u<<14);
    frame.dg.lenFlags.raw_le = host_to_le16(static_cast<uint16_t>((datalen&0x07FFu)|rt));
    return frame;
}

// ============================================================================
// Internal: discover slaves
// ============================================================================

bool EtherCATMaster::discoverSlaves()
{
    for (int attempt = 0; attempt < 200; attempt++) {
        if (!running_.load(std::memory_order_acquire)) return false;

        const uint8_t idx = allocIdx();
        auto frame = buildScanFrame(src_mac_);
        frame.dg.idx = idx;

        // Pre-register waiter BEFORE sending to avoid race condition
        RxDatagram resp{};
        size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
        if (slot >= TransactionRouter::kNumSlots) {
            TETHER_LOGW(TAG, "Failed to pre-register waiter for discovery");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        uint8_t txbuf[60] = {0};
        std::memcpy(txbuf, &frame, sizeof(frame));
        if (!sendRawFrame(txbuf, sizeof(txbuf))) {
            packet_router_.cancelPreRegistered(slot);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        WaitResult result = waitForPreRegistered(slot, 300);
        if (result.success && result.wkc > 0) {
            // Copy received data to resp datagram structure
            resp.idx = result.idx;
            resp.cmd = result.cmd;
            resp.adp = result.adp;
            resp.ado = result.ado;
            resp.datalen = result.data_length;
            resp.wkc = result.wkc;

            discovered_slave_count_.store(resp.wkc, std::memory_order_release);
            initSlaves(resp.wkc);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// ============================================================================
// Internal: set PRE_OP and confirm
// ============================================================================

bool EtherCATMaster::setPreopAndConfirm(uint16_t slave_index)
{
    using namespace Raw;

    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u (INIT => PRE_OP)          ║", slave_index);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: INIT => PRE_OP");
        TETHER_LOGI(TAG, "║  Reason:    Automatism - enabling mailbox operations");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - Slave must respond to AL_STATUS register reads");
        TETHER_LOGI(TAG, "║    - Slave must accept AL_CONTROL register writes");
        TETHER_LOGI(TAG, "║  Status:     Starting transition process with retry logic");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }

    // We'll attempt to set PRE_OP multiple times, with an extended wait per attempt.
    // This helps recover from transient conditions where the slave's mailbox or
    // internal state is not yet ready to accept AL state changes.
    const int max_attempts = 3;          // Number of attempts
    const int inner_tries = 200;         // Wait checks per attempt (inner loop)
    const int inner_sleep_ms = 20;       // Delay between checks

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        if (g_debug_statemachine) {
            TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
            TETHER_LOGI(TAG, "║  Attempt %d/%d for Slave %u                                    ║", attempt, max_attempts, slave_index);
            TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
        }
        
        // Read current AL status and decide whether to request PRE_OP
        uint16_t al_le = 0;
        (void)readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, al_le, 200);
        const uint16_t al0 = le16_to_host(al_le);
        const bool has_error = (al0 & 0x0010u) != 0;

        if (g_debug_statemachine) {
            const char* state_name = al_status_get_state_name(al0);
            TETHER_LOGI(TAG, "  Current AL_STATUS: 0x%04X (State=%s, Error=%s)", 
                       al0, state_name, has_error ? "true" : "false");
            TETHER_LOGI(TAG, "  Requesting PRE_OP with error bit: %s", has_error ? "SET" : "CLEAR");
        }

        const uint16_t req = static_cast<uint16_t>(0x0002u | (has_error ? 0x0010u : 0));
        (void)writeRegister(SlaveAddress(slave_index), EC_REG_AL_CONTROL, req);

        if (g_debug_statemachine) {
            TETHER_LOGI(TAG, "  Wrote AL_CONTROL: 0x%04X", req);
            TETHER_LOGI(TAG, "  Waiting for PRE_OP to become active (max %d checks, %dms each)...", 
                       inner_tries, inner_sleep_ms);
        }

        // Wait for PRE_OP to become active
        for (int i = 0; i < inner_tries; i++) {
            uint16_t s_le = 0;
            if (readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, s_le, 200)) {
                if ((le16_to_host(s_le) & 0x000Fu) == 0x0002u) {
                    if (attempt > 1) {
                        TETHER_LOGI(TAG, "setPreop: succeeded on attempt %d", attempt);
                    }
                    if (g_debug_statemachine) {
                        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => PRE_OP SUCCESS                ║", slave_index);
                        TETHER_LOGI(TAG, "║  Confirmed after %d checks on attempt %d/%d                   ║", i+1, attempt, max_attempts);
                        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                    }
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(inner_sleep_ms));
        }

        // If we reached here, the attempt failed — gather diagnostics and retry
        uint16_t s_le_fail = 0;
        if (readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, s_le_fail, 200)) {
            const uint16_t al_raw = le16_to_host(s_le_fail);
            const char* state_name = EtherCAT::al_status_get_state_name(al_raw);
            const bool is_error = EtherCAT::al_status_has_error(al_raw);

            // Try to read AL_STATUS_CODE for more detail
            uint16_t al_code_le = 0;
            const bool have_code = readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS_CODE, al_code_le, 200);
            uint16_t al_code = have_code ? le16_to_host(al_code_le) : 0;

            if (is_error) {
                TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS=0x%04X (State=%s, ERROR=true)",
                         attempt, al_raw, state_name);
            } else {
                TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS=0x%04X (State=%s)",
                         attempt, al_raw, state_name);
            }

            if (have_code) {
                TETHER_LOGW(TAG, "setPreop: AL_STATUS_CODE=0x%04X (%s)", al_code, EtherCAT::getALStatusCodeName(static_cast<EtherCAT::ALStatusCode>(al_code)));

                // Fallback: If the slave reports Invalid Mailbox Configuration on the
                // first attempt to enter PRE_OP, optionally try forcing conservative mailbox
                // settings (ignore SII) and reconfigure SM0/SM1. This helps with
                // devices whose SM0/SM1 registers were cleared (e.g. after a software reset).
                if ((al_code == static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfig) ||
                     al_code == static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfigPreOp)) &&
                    attempt == 1) {
                    if (config_.enable_mailbox_fallback) {
                        TETHER_LOGW(TAG, "setPreop: AL_STATUS_CODE indicates invalid mailbox for slave %u — applying mailbox defaults (enable_mailbox_fallback=true)", slave_index);
                        if (forceMailboxDefaults(slave_index)) {
                            TETHER_LOGI(TAG, "setPreop: mailbox defaults applied for slave %u; retrying PRE_OP", slave_index);
                        } else {
                            TETHER_LOGW(TAG, "setPreop: forceMailboxDefaults failed for slave %u", slave_index);
                        }
                        // Wait for slave to process new SM config
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    } else {
                        TETHER_LOGW(TAG, "setPreop: AL_STATUS_CODE indicates invalid mailbox for slave %u — set enable_mailbox_fallback=true to auto-fix", slave_index);
                    }
                }
            }

            // If AL_STATUS had an error bit set, issue a one-time fault diagnostic
            // dump for this slave so users get actionable guidance.
            if (is_error) {
                std::lock_guard<std::mutex> _lg(m_diag_mutex_);
                if (m_diagnosed_slaves_.find(slave_index) == m_diagnosed_slaves_.end()) {
                    TETHER_LOGI(TAG, "setPreop: issuing one-time fault_diagnose() for slave %u", slave_index);
                    faults_->diagnose(slave_index);
                    m_diagnosed_slaves_.insert(slave_index);
                }
            }
        } else {
            TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS read failed", attempt);
        }

        // Read SM0 (mailbox status) for additional context
        uint8_t sm0 = 0;
        const uint16_t sm0_ado = 0x0805; // SM0 status register
        if (readRegister(SlaveAddress(slave_index), sm0_ado, sm0, 200)) {
            TETHER_LOGW(TAG, "setPreop: SM0=0x%02X (mailbox status)", sm0);
        } else {
            TETHER_LOGW(TAG, "setPreop: SM0 read failed");
        }

        // Backoff before retrying
        const int backoff_ms = 200 * attempt;
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
    }

    TETHER_LOGE(TAG, "setPreopAndConfirm: all %d attempts failed", 3);
    return false;
}

bool EtherCATMaster::forceMailboxDefaults(SlaveAddress slave_address)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    // Conservative hardcoded fallback values with smaller sizes for better compatibility
    // Use 128-byte mailboxes which are more commonly supported by simple devices
    // Standard EtherCAT convention: SM0 (Receive/MbxIn, M→S),
    //                               SM1 (Send/MbxOut, S→M)
    constexpr uint16_t kHardcodedWrAddr = 0x1000;  // Receive/MbxIn (M→S, SM0)
    constexpr uint16_t kHardcodedWrLen = 128;
    constexpr uint16_t kHardcodedRdAddr = 0x1400;  // Send/MbxOut (S→M, SM1)
    constexpr uint16_t kHardcodedRdLen = 128;

    if (slave_index >= PDO::kMaxPDOSlaves) return false;

    // Compute ADP for this slave (0x0000 for slave 0, 0xFFFF for slave 1, etc.)
    uint16_t adp = adpForSlaveIndex(slave_index);

    // When called as a fallback for Invalid Mailbox Configuration errors,
    // skip SII completely and use truly conservative hardcoded defaults.
    // The SII values are likely incorrect if we're in this fallback path.
    TETHER_LOGW(TAG, "forceMailboxDefaults: Using conservative hardcoded mailbox defaults (bypassing potentially incorrect SII)");
    uint16_t wr_addr = kHardcodedWrAddr;
    uint16_t wr_len = kHardcodedWrLen;
    uint16_t rd_addr = kHardcodedRdAddr;
    uint16_t rd_len = kHardcodedRdLen;

    auto* slave_configs = pdo_->slaveConfigs();
    // Standard EtherCAT mailbox SM convention:
    // SM0 = Receive/MbxIn (MASTER→SLAVE, control=0x26)
    // SM1 = Send/MbxOut    (SLAVE→MASTER, control=0x22)
    slave_configs[slave_index].sm[0] = PDO::SyncManagerConfig::mailbox_write(wr_addr, wr_len);  // SM0 = Receive/MbxIn (M→S)
    slave_configs[slave_index].sm[1] = PDO::SyncManagerConfig::mailbox_read(rd_addr, rd_len);   // SM1 = Send/MbxOut (S→M)

    std::vector<PDO::SyncManagerConfig> sm_vec;
    for (int i=0; i<4; ++i) sm_vec.push_back(slave_configs[slave_index].sm[i]);
    
    auto val_res = SyncManagerValidation::validate(sm_vec);
    if (!val_res.valid) {
        TETHER_LOGE(TAG, "SyncManager Validation Failed for slave %u: %s", slave_index, val_res.error_message.c_str());
        return false;
    }

    bool applied = pdo_->configureSlavesSMs(slave_index);
    if (mailbox_fallback_cb_) mailbox_fallback_cb_(slave_index);
    return applied;
}

void EtherCATMaster::masterTask()
{
    ensureRxQueues();
    if (!rx_queue_) { TETHER_LOGE(TAG, "masterTask: no RX queue"); return; }

    TETHER_LOGI(TAG, "Master: discovering slaves...");
    if (!discoverSlaves()) { TETHER_LOGE(TAG, "No slaves found"); return; }

    const uint16_t n = discovered_slave_count_.load(std::memory_order_acquire);
    TETHER_LOGI(TAG, "Found %u slaves", n);

    // Initialize fault detector with discovered slave count
    if (faults_) {
        faults_->init(n);
    }

    // Configure mailbox and PRE_OP for each discovered slave
    for (uint16_t i = 0; i < n; i++) {
        // Force slave through INIT state to reset mailbox SM buffers.
        // If the slave was left in OP/SAFE_OP from a prior dirty session,
        // its SM0 mailbox buffer may contain stale unconsumed data (stat=0x48).
        // The INIT→PRE_OP transition clears this.
        (void)writeRegister(SlaveAddress(i), Raw::EC_REG_AL_CONTROL, static_cast<uint16_t>(0x0011));  // INIT + ACK
        for (int w = 0; w < 20; w++) {
            uint16_t al_le = 0;
            if (readRegister(SlaveAddress(i), Raw::EC_REG_AL_STATUS, al_le, 200)) {
                if ((Raw::le16_to_host(al_le) & 0x000F) == 0x0001) break;  // INIT reached
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // If the application set an explicit mailbox override, use it instead of reading SII
        bool applied_override = false;
        {
            std::lock_guard<std::mutex> _lg(m_mailbox_override_mutex_);
            if (i < static_cast<uint16_t>(m_mailbox_overrides_.size()) && m_mailbox_overrides_[i].enabled) {
                auto& ov = m_mailbox_overrides_[i];
                auto* slave_configs = pdo_->slaveConfigs();
                if (slave_configs) {
                    // Standard EtherCAT SM convention:
                    // SM0 = Receive/MbxIn (M→S), SM1 = Send/MbxOut (S→M)
                    slave_configs[i].sm[0] = PDO::SyncManagerConfig::mailbox_write(ov.wr_addr, ov.wr_len);
                    slave_configs[i].sm[1] = PDO::SyncManagerConfig::mailbox_read(ov.rd_addr, ov.rd_len);
                    const uint8_t* src_mac = src_mac_;

                    std::vector<PDO::SyncManagerConfig> sm_vec;
                    for (int k=0; k<4; ++k) sm_vec.push_back(slave_configs[i].sm[k]);
                    auto val_res = SyncManagerValidation::validate(sm_vec);

                    if (!val_res.valid) {
                        TETHER_LOGE(TAG, "Slave %u: Override SyncManager Validation Failed: %s", (unsigned)i, val_res.error_message.c_str());
                    } else if (src_mac && pdo_->configureSlavesSMs(i)) {
                        TETHER_LOGI(TAG, "Slave %u: Applied XML mailbox override wr=0x%04X/%u rd=0x%04X/%u proto=0x%04X",
                                 (unsigned)i, ov.wr_addr, (unsigned)ov.wr_len, ov.rd_addr, (unsigned)ov.rd_len, ov.proto);
                        applied_override = true;
                    } else {
                        TETHER_LOGW(TAG, "Slave %u: Failed to apply XML mailbox override", (unsigned)i);
                    }
                }
            }
        }

        if (!applied_override) {
            uint16_t wa=0,wl=0,ra=0,rl=0,mp=0;
            if (configureMailboxFromSii(i,&wa,&wl,&ra,&rl,&mp)) {
                TETHER_LOGI(TAG, "Slave %u: Mailbox from SII wr=0x%04X/%u rd=0x%04X/%u", i, wa, (unsigned)wl, ra, (unsigned)rl);

                // Standard EtherCAT SM convention:
                // SM0 = Receive/MbxIn (M→S), SM1 = Send/MbxOut (S→M)
                auto* slave_configs = pdo_->slaveConfigs();
                if (slave_configs) {
                    slave_configs[i].sm[0] = PDO::SyncManagerConfig::mailbox_write(wa, wl);
                    slave_configs[i].sm[1] = PDO::SyncManagerConfig::mailbox_read(ra, rl);
                    pdo_->configureSlavesSMs(i);
                }
                // NOTE: SDO mailbox configuration is provided by the PDO SyncManager
                // configuration written above (SM0/SM1). The SDO subsystem now prefers
                // the PDO-configured SyncManagers, so an explicit call to
                // configureSlaveMailbox() here would be redundant and has been
                // removed to avoid duplication of configuration sources.
            } else {
                TETHER_LOGW(TAG, "Slave %u: Failed to configure mailbox from SII", i);
            }
        }

        if (!setPreopAndConfirm(i)) {
            TETHER_LOGE(TAG, "Slave %u: Failed to set PRE_OP", i);
        } else {
            TETHER_LOGI(TAG, "Slave %u: PRE_OP confirmed", i);

            // Configure process-data SyncManagers (SM2/SM3) from SII / HW registers
            if (!configureProcessDataSyncManagersFromSii(SlaveAddress(i))) {
                TETHER_LOGW(TAG, "Slave %u: Failed to configure process-data SMs (SM2/SM3) from SII", i);
            }
        }
    }

    // Also emit a concise discovered-slaves summary for diagnostics
    logDiscoveredSlavesSummary(TAG);
}

void EtherCATMaster::logDiscoveredSlavesSummary(const char* tag)
{
    const uint16_t n = getDiscoveredSlaveCount();
    TETHER_LOGI(tag, "Discovered %u slave(s)", n);

    for (uint16_t i = 0; i < n; ++i) {
        EtherCAT::SII::SIIData sii_data;
        if (EtherCAT::SII::readSII(*this, i, sii_data)) {
            // Detailed summary handled by SII module
            EtherCAT::SII::logSIISummary(sii_data, i, tag);
        }
        else {
            // Fallback: try reading identity only
            EtherCAT::SII::SIIIdentity id;
            if (EtherCAT::SII::readSIIIdentity(*this, i, id)) {
                char name_buf[64] = {0};
                if (!siiReadString(i, 1, name_buf, sizeof(name_buf)) &&
                    !siiReadString(i, 2, name_buf, sizeof(name_buf)) &&
                    !siiReadString(i, 3, name_buf, sizeof(name_buf))) {
                    strncpy(name_buf, "<unknown>", sizeof(name_buf));
                }

                TETHER_LOGI(tag, "Slave %u @ ADP=0x%04X Vendor=0x%08" PRIx32 " Product=0x%08" PRIx32 " Name='%s'",
                         i, adpForSlaveIndex(i), id.vendor_id, id.product_code, name_buf);
            }
            else {
                TETHER_LOGW(tag, "Slave %u @ ADP=0x%04X: unable to read SII/identity", i, adpForSlaveIndex(i));
            }
        }
    }
}

} // namespace EtherCAT
