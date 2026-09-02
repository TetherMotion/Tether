#include "tether/ethercat/DCManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/Raw.hpp"
#include "tether/platform/EspCompat.hpp"

#include <cstring>

namespace EtherCAT {

static const char* TAG = "DC sync";

DCManager::DCManager(Master& master)
    : master_(master), sentinel_(std::make_unique<NoDistributedClockConfigured>())
{
}

// Helper to convert DC::DCConfig to EtherCAT::DCConfig
static EtherCAT::DCConfig convertConfig(const DC::DCConfig& dc_config) {
    return EtherCAT::DCConfig{
        .cycle_period_us = dc_config.cycle_period_us,
        .sync_interval_cycles = dc_config.sync_interval_cycles,
        .sync0_cycle_time_ns = dc_config.sync0_cycle_time_ns,
        .sync1_cycle_time_ns = dc_config.sync1_cycle_time_ns,
        .sync0_shift_ns = dc_config.sync0_shift_ns,
        .enable_sync0 = dc_config.enable_sync0,
        .enable_sync1 = dc_config.enable_sync1,
        .sync_disabled_slaves = dc_config.sync_disabled_slaves
    };
}

// Helper to convert EtherCAT::DCState to DC::DCState
static DC::DCState convertState(EtherCAT::DCState state) {
    switch (state) {
        case EtherCAT::DCState::Disabled:         return DC::DCState::Disabled;
        case EtherCAT::DCState::Initializing:     return DC::DCState::Initializing;
        case EtherCAT::DCState::PropagationCalc:  return DC::DCState::PropagationCalc;
        case EtherCAT::DCState::DriftCompensation: return DC::DCState::DriftCompensation;
        case EtherCAT::DCState::Running:          return DC::DCState::Running;
        case EtherCAT::DCState::Error:            return DC::DCState::Error;
        default:                                   return DC::DCState::Error;
    }
}

// Helper to convert EtherCAT::DCLoopStats to DC::DCLoopStats
static DC::DCLoopStats convertStats(const EtherCAT::DCLoopStats& stats) {
    return DC::DCLoopStats{
        .cycle_count = stats.cycle_count,
        .sync_count = stats.sync_count,
        .pdo_error_count = stats.pdo_error_count,
        .max_jitter_us = stats.max_jitter_us,
        .avg_jitter_us = stats.avg_jitter_us,
        .last_drift_ns = stats.last_drift_ns,
        .last_master_time_ns = stats.last_master_time_ns
    };
}

bool DCManager::init(const DC::DCConfig& config, uint16_t slave_count)
{
    if (slave_count == 0) {
        TETHER_LOGW(TAG, "DCManager::init: invalid slave count (0)");
        return false;
    }

    try {
        // Convert DC::DCConfig to EtherCAT::DCConfig
        EtherCAT::DCConfig class_config = convertConfig(config);
        
        // Create transport adapter (owned by DCManager)
        transport_ = std::make_unique<RawDCTransport>(master_);

        // Create EtherCATDC instance via transport
        dc_instance_ = std::make_unique<EtherCATDC>(*transport_, slave_count, &class_config);
        // Propagate the master's --debug dc flag to the DC instance.
        dc_instance_->setDebugLogging(master_.debugFlags().dc);
        // Perform explicit initialization step (reads capabilities). Return true
        // if instance created even if initialization was incomplete.
        bool init_ok = dc_instance_->init();
        if (!init_ok) {
            TETHER_LOGW(TAG, "DCManager: DC initialization incomplete after init() call");
        }
        TETHER_LOGI(TAG, "DCManager: created EtherCATDC instance (slaves={})", (unsigned)slave_count);
        return true;
    } catch (const std::exception& ex) {
        TETHER_LOGW(TAG, "DCManager: failed to create EtherCATDC: {}", ex.what());
        dc_instance_.reset();
        return false;
    }
}

DCManager::~DCManager()
{
    if (dc_instance_) {
        dc_instance_->stop();
        dc_instance_.reset();
    }
    transport_.reset();
}

bool DCManager::start()
{
    if (!dc_instance_) {
        TETHER_LOGW(TAG, "DCManager::start: DC not initialized (call init() first)");
        return false;
    }
    // Automatically wire in the master's PDO physical exchange so the
    // realtime loop drives actual process data, not a noop.  Exchange
    // the default group plus any additional PDO groups.
    // Pass kMaxPDOSlaves so exchangePhysical scans all configured slaves
    // (not just index 0 — needed for slaves at index > 0).
    auto pdo_fn = [this]() -> bool {
        bool ok = master_.pdo().exchangePhysical(EtherCAT::PDO::kMaxPDOSlaves);
        for (auto& group : master_.pdoGroups()) {
            if (group.pdo) {
                ok = group.pdo->exchangeAll() && ok;
            }
        }
        return ok;
    };
    return dc_instance_->start(std::move(pdo_fn));
}

bool DCManager::start(std::function<bool()> pdo_exchange_fn)
{
    if (!dc_instance_) {
        TETHER_LOGW(TAG, "DCManager::start: DC not initialized (call init() first)");
        return false;
    }
    return dc_instance_->start(std::move(pdo_exchange_fn));
}

void DCManager::stop()
{
    if (dc_instance_) {
        dc_instance_->stop();
    }
}

DC::DCState DCManager::getState() const
{
    if (!dc_instance_) {
        return DC::DCState::Disabled;
    }
    return convertState(dc_instance_->getState());
}

DC::DCLoopStats DCManager::getStats() const
{
    if (!dc_instance_) {
        return DC::DCLoopStats{};
    }
    return convertStats(dc_instance_->getStats());
}

void DCManager::forceSync()
{
    if (dc_instance_) {
        dc_instance_->forceSync();
    }
}

void DCManager::setPDOEnabled(bool en)
{
    if (dc_instance_) {
        dc_instance_->setPDOEnabled(en);
    }
}

bool DCManager::reconfigureSync(uint16_t slave_index)
{
    if (!dc_instance_) {
        TETHER_LOGW(TAG, "DCManager::reconfigureSync: DC not initialized");
        return false;
    }
    return dc_instance_->reconfigureSync(slave_index);
}

} // namespace EtherCAT
