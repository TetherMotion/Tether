/**
 * @file DCClass.cpp
 * @brief Implementation of class-based EtherCAT Distributed Clock synchronization
 *
 * All network I/O goes through IDCTransport — no direct Raw:: or global
 * dependency.  The realtime loop is delegated to RealtimeLoop.
 */

#include "tether/ethercat/DCClass.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"

#include <cstring>
#include <algorithm>

namespace EtherCAT {

static const char* TAG = "DC sync";

// DC register enums moved to the public header `include/tether/ethercat/DCClass.hpp`.

// ============================================================================
// Byte-order helpers (little-endian ↔ host, identity on x86/ESP32)
// ============================================================================
// ============================================================================
// Byte-order helpers (little-endian ↔ host, identity on x86/ESP32)
// ============================================================================

static inline uint32_t host_to_le32(uint32_t val) { return val; }
static inline uint32_t le32_to_host(uint32_t val) { return val; }

// ============================================================================
// Constructor / Destructor
// ============================================================================

// NoopDCTransport is defined in the public header (EtherCATDC.hpp) and held
// as a private base of NoDistributedClockConfigured (base-from-member idiom),
// so no process-global sentinel instance is required here.

// ============================================================================
// NoDistributedClockConfigured (sentinel)
// ============================================================================

NoDistributedClockConfigured::NoDistributedClockConfigured()
    : NoopDCTransport()
    , EtherCATDC(static_cast<NoopDCTransport&>(*this), 1, nullptr)
{
    // Keep the base class in Disabled state; do not initialize.
}

void NoDistributedClockConfigured::logCritical(const char* method) const {
    TETHER_LOGE(TAG, "CRITICAL: %s() called on uninitialized Distributed Clock. Call dc().init() first.", method);
}

bool NoDistributedClockConfigured::start() { logCritical("start"); return false; }
bool NoDistributedClockConfigured::start(std::function<bool()> ) { logCritical("start"); return false; }
bool NoDistributedClockConfigured::init() { logCritical("init"); return false; }
void NoDistributedClockConfigured::stop() { logCritical("stop"); }

DCState NoDistributedClockConfigured::getState() const { logCritical("getState"); return DCState::Disabled; }
DCLoopStats NoDistributedClockConfigured::getStats() const { logCritical("getStats"); return DCLoopStats{}; }

void NoDistributedClockConfigured::forceSync() { logCritical("forceSync"); }
void NoDistributedClockConfigured::setPDOEnabled(bool) { logCritical("setPDOEnabled"); }
bool NoDistributedClockConfigured::isPDOEnabled() const { logCritical("isPDOEnabled"); return false; }

bool NoDistributedClockConfigured::isSlaveSupported(uint16_t) const { logCritical("isSlaveSupported"); return false; }
int64_t NoDistributedClockConfigured::getSlaveOffset(uint16_t) const { logCritical("getSlaveOffset"); return 0; }

void NoDistributedClockConfigured::readSyncConfig(uint16_t) { logCritical("readSyncConfig"); }
bool NoDistributedClockConfigured::reconfigureSync(uint16_t) { logCritical("reconfigureSync"); return false; }

uint64_t NoDistributedClockConfigured::getMasterTimeNs() { logCritical("getMasterTimeNs"); return 0; }

bool NoDistributedClockConfigured::readRegister(uint16_t, DCRegisters, void*, uint16_t, unsigned int) { logCritical("readRegister"); return false; }
bool NoDistributedClockConfigured::writeRegister(uint16_t, DCRegisters, const void*, uint16_t, unsigned int) { logCritical("writeRegister"); return false; }

bool NoDistributedClockConfigured::sendSyncFrame() { logCritical("sendSyncFrame"); return false; }

EtherCATDC::EtherCATDC(IDCTransport& transport,
                       uint16_t slave_count,
                       const DCConfig* config)
    : config_(config ? *config : DCConfig::defaults())
    , slave_count_(std::min(slave_count, static_cast<uint16_t>(kMaxDCSlaves)))
    , transport_(transport)
{
    if (slave_count_ == 0) {
        TETHER_LOGE(TAG, "Invalid parameters: slaves=%u", slave_count_);
        state_.store(DCState::Error, std::memory_order_release);
        return;
    }

    std::memset(slaves_, 0, sizeof(slaves_));
}

EtherCATDC::~EtherCATDC() {
    stop();
}

/// No-op PDO exchange (used when no callback is provided)
static bool ExchangeNoop_impl() { return true; }

// ============================================================================
// Public API
// ============================================================================

bool EtherCATDC::start() {
    return start(nullptr);
}

bool EtherCATDC::start(std::function<bool()> pdo_exchange_fn) {
    const DCState current_state = state_.load(std::memory_order_acquire);

    if (current_state == DCState::Error) {
        TETHER_LOGE(TAG, "Cannot start: DC in error state");
        return false;
    }

    if (current_state == DCState::Disabled && !initialized_.load(std::memory_order_acquire)) {
        TETHER_LOGW(TAG, "Cannot start: DC not initialized");
        return false;
    }

    if (realtime_loop_ && realtime_loop_->isRunning()) {
        TETHER_LOGW(TAG, "DC already running");
        return false;
    }

    // Store PDO exchange callback for potential restart
    pdo_exchange_fn_ = std::move(pdo_exchange_fn);

    // Build the realtime loop with DC sync and (optional) PDO exchange callbacks.
    // Use the factory method so jitter thresholds are auto-derived from the
    // actual cycle/sync periods rather than hardcoded defaults.
    RealtimeLoop::Config loop_cfg =
        RealtimeLoop::Config::defaults(config_.cycle_period_us,
                                                config_.sync_interval_cycles);

    auto sync_fn = [this]() { return sendSyncFrame(); };
    auto time_fn = [this]() { return getMasterTimeNs(); };

    realtime_loop_ = std::make_unique<RealtimeLoop>(
        pdo_exchange_fn_ ? pdo_exchange_fn_ : RealtimeLoop::ExchangeFunc(ExchangeNoop_impl),
        sync_fn, time_fn, loop_cfg);

    if (!realtime_loop_->start()) {
        TETHER_LOGE(TAG, "Failed to start realtime loop");
        realtime_loop_.reset();
        return false;
    }

    state_.store(DCState::Running, std::memory_order_release);

    TETHER_LOGI(TAG, "DC realtime loop started (period=%u us)", config_.cycle_period_us);
    return true;
}

void EtherCATDC::stop() {
    if (realtime_loop_) {
        // Snapshot loop stats before destroying the loop
        auto loop_stats = realtime_loop_->getStats();
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.cycle_count     = loop_stats.cycle_count;
            stats_.sync_count      = loop_stats.sync_count;
            stats_.pdo_error_count = loop_stats.pdo_error_count;
            stats_.max_jitter_us   = loop_stats.max_jitter_us;
            stats_.avg_jitter_us   = loop_stats.avg_jitter_us;
        }

        realtime_loop_->stop();
        realtime_loop_.reset();
    }

    // Transition to Disabled. The 'initialized_' flag remains set so
    // callers can call start() again without re-initializing.
    state_.store(DCState::Disabled, std::memory_order_release);

    TETHER_LOGI(TAG, "DC realtime loop stopped");
}

DCLoopStats EtherCATDC::getStats() const {
    DCLoopStats result{};

    if (realtime_loop_) {
        // Live stats from running loop
        auto loop_stats = realtime_loop_->getStats();
        result.cycle_count     = loop_stats.cycle_count;
        result.sync_count      = loop_stats.sync_count;
        result.pdo_error_count = loop_stats.pdo_error_count;
        result.max_jitter_us   = loop_stats.max_jitter_us;
        result.avg_jitter_us   = loop_stats.avg_jitter_us;
    }

    // DC-specific stats (includes saved loop stats after stop)
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (!realtime_loop_) {
        // Loop stopped — use saved snapshot
        result.cycle_count     = stats_.cycle_count;
        result.sync_count      = stats_.sync_count;
        result.pdo_error_count = stats_.pdo_error_count;
        result.max_jitter_us   = stats_.max_jitter_us;
        result.avg_jitter_us   = stats_.avg_jitter_us;
    }
    result.last_drift_ns      = stats_.last_drift_ns;
    result.last_master_time_ns = stats_.last_master_time_ns;
    return result;
}

void EtherCATDC::forceSync() {
    sendSyncFrame();
}

void EtherCATDC::setPDOEnabled(bool enable) {
    if (realtime_loop_) realtime_loop_->setPDOEnabled(enable);
}

bool EtherCATDC::isPDOEnabled() const {
    return realtime_loop_ ? realtime_loop_->isPDOEnabled() : false;
}

bool EtherCATDC::isSlaveSupported(uint16_t slave_index) const {
    if (slave_index >= slave_count_) {
        return false;
    }
    return slaves_[slave_index].dc_supported;
}

int64_t EtherCATDC::getSlaveOffset(uint16_t slave_index) const {
    if (slave_index >= slave_count_) {
        return 0;
    }
    return slaves_[slave_index].offset_to_master_ns;
}

uint64_t EtherCATDC::getMasterTimeNs() {
    return transport_.getMasterTimeNs();
}

// ============================================================================
// Internal Implementation
// ============================================================================

bool EtherCATDC::init() {
    if (state_.load(std::memory_order_acquire) == DCState::Error) {
        initialized_.store(false, std::memory_order_release);
        return false;
    }

    const bool ok = initialize();
    initialized_.store(ok, std::memory_order_release);

    // init() probes/configures slaves but should leave the instance
    // in Disabled state until start() is called.
    if (state_.load(std::memory_order_acquire) != DCState::Error) {
        state_.store(DCState::Disabled, std::memory_order_release);
    }

    return ok;
}

bool EtherCATDC::initialize() {
    state_.store(DCState::Initializing, std::memory_order_release);
    
    if (dc_debug_) {
        TETHER_LOGI(TAG, "Reading DC capabilities from %u slaves...", slave_count_);
    }

    uint16_t dc_capable_count = 0;
    for (uint16_t i = 0; i < slave_count_; i++) {
        if (readSlaveCapabilities(i)) {
            dc_capable_count++;

            // Calculate propagation delay
            if (!calcPropagationDelay(i)) {
                TETHER_LOGW(TAG, "Slave[%u]: propagation delay calculation failed", i);
            }

            // Calculate offset from master
            const uint64_t master_time = getMasterTimeNs();
            slaves_[i].offset_to_master_ns =
                static_cast<int64_t>(slaves_[i].system_time_ns) - static_cast<int64_t>(master_time);

            if (dc_debug_) {
                TETHER_LOGI(TAG, "Slave[%u]: offset=%lld ns, delay=%u ns",
                         i, (long long)slaves_[i].offset_to_master_ns,
                         slaves_[i].propagation_delay_ns);
            }
            
            // Write system time offset
            if (!writeSystemTimeOffset(i, -slaves_[i].offset_to_master_ns)) {
                TETHER_LOGW(TAG, "Slave[%u]: failed to write time offset", i);
            }
            
            // Configure SYNC signals
            if (!configureSyncSignals(i)) {
                TETHER_LOGW(TAG, "Slave[%u]: SYNC configuration failed", i);
            }
        } else {
            TETHER_LOGW(TAG, "Slave[%u]: DC capability read failed or not supported", i);
        }
    }
    
    if (dc_debug_) {
        TETHER_LOGI(TAG, "Found %u DC-capable slaves out of %u total",
                 dc_capable_count, slave_count_);
    }
    
    if (dc_capable_count == 0) {
        TETHER_LOGW(TAG, "No DC-capable slaves found, DC sync disabled");
        state_.store(DCState::Disabled, std::memory_order_release);
        return false;
    }

    // Initialization succeeded; remain stopped until start() is called.
    state_.store(DCState::Disabled, std::memory_order_release);
    return true;
}


// ============================================================================
// Protocol Implementation - Instance Methods
// ============================================================================

bool EtherCATDC::readSlaveCapabilities(uint16_t slave_index) {
    if (slave_index >= slave_count_) {
        return false;
    }

    SlaveTimeInfo& info = slaves_[slave_index];

    if (dc_debug_) {
        TETHER_LOGI(TAG, "Slave[%u]: Reading DC System Time (reg=0x0910)...", slave_index);
    }
    
    uint8_t sysTime[8] = {0};
    if (!transport_.readRegister(slave_index, toUInt16(DCRegisters::DCSysTime), sysTime, sizeof(sysTime), 200)) {
        TETHER_LOGW(TAG, "Slave[%u]: DC System Time read FAILED", slave_index);
        return false;
    }

    uint64_t sys_time = 0;
    for (int i = 7; i >= 0; i--) {
        sys_time = (sys_time << 8) | sysTime[i];
    }

    info.system_time_ns = sys_time;
    info.dc_supported = (sys_time != 0);

    return info.dc_supported;
}

bool EtherCATDC::calcPropagationDelay(uint16_t slave_index) {
    if (slave_index >= slave_count_) {
        return false;
    }
    
    slaves_[slave_index].propagation_delay_ns = (slave_index == 0) ? 0 : 150 * slave_index;
    return true;
}

bool EtherCATDC::writeSystemTimeOffset(uint16_t slave_index, int64_t offset) {
    if (slave_index >= slave_count_) {
        return false;
    }

    uint8_t offset_bytes[8];
    uint64_t offset_u = static_cast<uint64_t>(offset);
    for (int i = 0; i < 8; i++) {
        offset_bytes[i] = static_cast<uint8_t>(offset_u & 0xFF);
        offset_u >>= 8;
    }

    return transport_.writeRegister(slave_index, toUInt16(DCRegisters::DCSysOffset),
                                     offset_bytes, sizeof(offset_bytes), 200);
}

// --------------------------------------------------------------------
// Convenience wrapper: read a DC register using the scoped enum
// --------------------------------------------------------------------
bool EtherCATDC::readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                              unsigned int timeout_ms) {
    if (slave_index >= slave_count_) return false;
    return transport_.readRegister(slave_index, toUInt16(reg), data, size, timeout_ms);
}

// --------------------------------------------------------------------
// Convenience wrapper: write a DC register using the scoped enum
// --------------------------------------------------------------------
bool EtherCATDC::writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                               unsigned int timeout_ms) {
    if (slave_index >= slave_count_) return false;
    return transport_.writeRegister(slave_index, toUInt16(reg), data, size, timeout_ms);
}


bool EtherCATDC::updateSyncStartTime() {
    for (uint16_t i = 0; i < slave_count_; i++) {
        if (!slaves_[i].dc_supported || !slaves_[i].dc_active) continue;

        // Skip SYNC0 start time for slaves in the sync_disabled filter —
        // they don't have SYNC0 enabled so the start time is irrelevant.
        bool sync_disabled = false;
        for (uint16_t idx : config_.sync_disabled_slaves) {
            if (idx == i) { sync_disabled = true; break; }
        }
        if (sync_disabled) continue;

        // Read the slave's current local DC System Time
        uint8_t sysTime[8] = {0};
        if (!transport_.readRegister(i, toUInt16(DCRegisters::DCSysTime), sysTime, sizeof(sysTime), 200)) {
            TETHER_LOGW(TAG, "Slave[%u]: Failed to read DC SysTime", i);
            continue;
        }

        uint64_t slave_time = 0;
        for (int j = 7; j >= 0; j--) {
            slave_time = (slave_time << 8) | sysTime[j];
        }

        // Set start time = current slave time + 10 cycle times (well in the future)
        uint64_t start_time = slave_time + config_.sync0_cycle_time_ns * 10;

        // Align to cycle boundary
        if (config_.sync0_cycle_time_ns > 0) {
            start_time = ((start_time / config_.sync0_cycle_time_ns) + 1)
                         * config_.sync0_cycle_time_ns;
        }

        // Apply SYNC0 shift (offset from cycle boundary)
        if (config_.sync0_shift_ns > 0) {
            start_time += static_cast<uint64_t>(config_.sync0_shift_ns);
        }

        uint8_t start_bytes[8];
        uint64_t st = start_time;
        for (int j = 0; j < 8; j++) {
            start_bytes[j] = static_cast<uint8_t>(st & 0xFF);
            st >>= 8;
        }

        if (!transport_.writeRegister(i, toUInt16(DCRegisters::DCStart0),
                                       start_bytes, sizeof(start_bytes), 200)) {
            TETHER_LOGW(TAG, "Slave[%u]: Failed to write SYNC0 start time", i);
            continue;
        }

        if (dc_debug_) {
            TETHER_LOGI(TAG, "Slave[%u]: SYNC0 start=%llu (delta=%llu ns)",
                     i, (unsigned long long)start_time,
                     (unsigned long long)(start_time - slave_time));
        }
    }
    return true;
}

bool EtherCATDC::configureSyncSignals(uint16_t slave_index) {
    if (slave_index >= slave_count_) {
        return false;
    }

    // Check if this slave is in the sync_disabled_slaves filter.
    // These slaves still get DC time sync (offset/delay compensation) but
    // do NOT get SYNC0/SYNC1 signal activation.
    bool sync_disabled = false;
    for (uint16_t idx : config_.sync_disabled_slaves) {
        if (idx == slave_index) {
            sync_disabled = true;
            break;
        }
    }

    if (sync_disabled) {
        // Deactivate SYNC signals for this slave but still mark it as
        // DC-active so time sync datagrams continue.
        uint8_t sync_act = DC_SYNCACT_ENA | DC_SYNCACT_AUTO_ACT;
        transport_.writeRegister(slave_index, toUInt16(DCRegisters::DCSyncAct),
                                  &sync_act, sizeof(sync_act), 200);
        slaves_[slave_index].dc_active = true;
        if (dc_debug_) {
            TETHER_LOGI(TAG, "Slave[%u]: SYNC0/SYNC1 disabled by filter "
                         "(time sync still active)", slave_index);
        }
        return true;
    }

    // Build sync activation register value from config flags
    uint8_t sync_act = DC_SYNCACT_ENA | DC_SYNCACT_AUTO_ACT;
    if (config_.enable_sync0 && config_.sync0_cycle_time_ns > 0) {
        sync_act |= DC_SYNCACT_SYNC0_ENA;
        // Write SYNC0 cycle time
        uint32_t cycle0_le = host_to_le32(config_.sync0_cycle_time_ns);
        transport_.writeRegister(slave_index, toUInt16(DCRegisters::DCCycle0),
                                  &cycle0_le, sizeof(cycle0_le), 200);
    }

    if (config_.enable_sync1 && config_.sync1_cycle_time_ns > 0) {
        sync_act |= DC_SYNCACT_SYNC1_ENA;
        // Write SYNC1 cycle time
        uint32_t cycle1_le = host_to_le32(config_.sync1_cycle_time_ns);
        transport_.writeRegister(slave_index, toUInt16(DCRegisters::DCCycle1),
                                  &cycle1_le, sizeof(cycle1_le), 200);
    }

    // Activate SYNC with the configured signals and Auto-Activation bit set.
    transport_.writeRegister(slave_index, toUInt16(DCRegisters::DCSyncAct),
                              &sync_act, sizeof(sync_act), 200);

    // Mark active BEFORE writing start time (updateSyncStartTime checks dc_active)
    slaves_[slave_index].dc_active = true;

    // Set the SYNC0 start time AFTER activating the sync unit.
    updateSyncStartTime();

    return true;
}

bool EtherCATDC::sendSyncFrame() {
    const uint64_t master_ns = getMasterTimeNs();
    bool all_ok = true;
    for (uint16_t i = 0; i < slave_count_; i++) {
        if (slaves_[i].dc_supported && slaves_[i].dc_active) {
            if (!transport_.sendSyncDatagram(i, toUInt16(DCRegisters::DCSysTime),
                                              &master_ns, sizeof(master_ns))) {
                all_ok = false;
            }
        }
    }
    return all_ok;
}

void EtherCATDC::readSyncConfig(uint16_t slave_index) {
    if (slave_index >= slave_count_) {
        return;
    }
    
    uint32_t sync0_cycle_le = 0;
    if (transport_.readRegister(slave_index, toUInt16(DCRegisters::DCCycle0),
                                 &sync0_cycle_le, sizeof(sync0_cycle_le), 200)) {
        uint32_t sync0_ns = le32_to_host(sync0_cycle_le);
        if (dc_debug_) {
            TETHER_LOGI(TAG, "Slave[%u] SYNC0 cycle: %lu ns", slave_index, (unsigned long)sync0_ns);
        }
    }
}

bool EtherCATDC::reconfigureSync(uint16_t slave_index) {
    return configureSyncSignals(slave_index);
}

} // namespace EtherCAT
