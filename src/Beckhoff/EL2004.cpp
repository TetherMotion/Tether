/**
 * @file EL2004.cpp
 * @brief Implementation of the Beckhoff EL2004 digital-output driver.
 *
 * See EL2004.hpp for the API documentation.  This file contains the
 * EL2004-specific bring-up sequence that works around three framework
 * assumptions which do not hold for this terminal:
 *
 *   1. No mailbox.  The EL2004's only sync manager is SM channel 0 carrying
 *      process outputs — configureMailbox() would overwrite those registers.
 *      assumeMailboxAlreadyConfigured() is used to satisfy the PRE-OP gate.
 *   2. Process outputs on SM0, not SM2.  The PDOManager bookkeeping expects
 *      RxPDO data at SM2/TxPDO at SM3.  The real config is registered under
 *      its actual channel index (so configureSlavesSMs() writes the correct
 *      ESC register block) and only the physical base address is mirrored
 *      into sm[2] for finalizeMapping()'s offset computation.
 *   3. No TxPDO.  Slave::transitionToOp() waits for a reply counter that an
 *      output-only slave can never increment, so OP is requested directly.
 *
 * SII quirk: the terminal reports SM length 0 in its EEPROM.  The real size
 * is the sum of the RxPDO entry bit lengths (4 x 1 bit -> 1 byte).
 */

#include "tether/Beckhoff/EL2004.hpp"

#include <bit>
#include <chrono>

#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "EL2004";

// ESI fallbacks (Beckhoff EL2xxx.xml), used when SII data is unavailable.
constexpr uint8_t  kFallbackSmCtrl = 0x44;  // buffered, ECAT-write, no watchdog
constexpr uint16_t kFallbackSmAddr = 0x0F00;
constexpr uint16_t kFallbackSmLen  = 1;

const char* EL2004::errorToString(Error e) {
    switch (e) {
        case Error::Ok:                    return "Ok";
        case Error::NoDeviceFound:         return "No EL2004 found on the bus";
        case Error::NotAnEL2004:           return "Slave is not an EL2004 (vendor/product mismatch)";
        case Error::SlaveIndexOutOfRange:  return "Slave index exceeds the PDO manager's slave table";
        case Error::SmConfigFailed:        return "Failed to write sync-manager registers";
        case Error::PdoRegistrationFailed: return "RxPDO buffer registration failed";
        case Error::PreOpFailed:           return "PRE-OP transition failed";
        case Error::SafeOpFailed:          return "SAFE-OP transition failed";
        case Error::OpRequestFailed:       return "OP request could not be sent";
        case Error::OpTimeout:             return "Slave did not reach OP in time";
        case Error::LoopStartFailed:       return "Master realtime loop failed to start";
        case Error::Cancelled:             return "Operation cancelled";
        default:                           return "Unknown error";
    }
}

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

EL2004::EL2004(Master& master, uint16_t slave_index)
    : master_(&master), slave_index_(slave_index),
      state_(std::make_unique<std::atomic<uint8_t>>(0)) {}

EL2004::EL2004(Master& master, const DiscoveredSlave& slave)
    : EL2004(master, slave.index) {
    info_ = slave;
}

EL2004::~EL2004() {
    stop();
}

EL2004::Result<EL2004> EL2004::findFirst(Master& master) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, scan);
}

EL2004::Result<EL2004> EL2004::findFirst(
    Master& master, std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s)) return EL2004(master, s);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

void EL2004::resolveOutputSm() {
#if TETHER_ENABLE_SII
    // The SII SM category is positional: entry index == SM channel.
    // The EL2004 has exactly one SM (channel 0, process outputs @ 0x0F00).
    if (info_ && info_->sync_managers) {
        for (size_t i = 0; i < info_->sync_managers->size() && i < 4; ++i) {
            const auto& sm = (*info_->sync_managers)[i];
            if (sm.sm_type == SII::SM_TYPE_PROCESS_OUT) {
                sm_channel_ = static_cast<uint8_t>(i);
                sm_addr_    = sm.phys_start_address;
                sm_len_     = sm.length;
                sm_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
                break;
            }
        }
    }

    // EL terminals report SM length 0 in SII — derive the real size from the
    // RxPDO entries (4 x 1 bit = 1 byte for the EL2004).
    if (sm_len_ == 0 && info_ && info_->rx_pdos) {
        size_t total_bits = 0;
        for (const auto& pdo : *info_->rx_pdos) {
            total_bits += pdo.totalBits();
        }
        sm_len_ = static_cast<uint16_t>((total_bits + 7) / 8);
    }
#endif
    if (sm_len_ == 0) sm_len_ = kFallbackSmLen;
}

EL2004::Result<> EL2004::configure() {
    if (configured_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    // Fetch SII data (identity + SM + PDO categories) if the caller did not
    // supply a DiscoveredSlave — or supplied a shallow scan result.
    bool need_sii = true;
#if TETHER_ENABLE_SII
    need_sii = !(info_ && info_->sync_managers);
#endif
    if (need_sii) {
        info_ = master_->discovery().discoverOne(
            slave_index_,
            {DiscoveryOption::VendorId, DiscoveryOption::ProductCode,
             DiscoveryOption::DeviceNames, DiscoveryOption::SyncManagers,
             DiscoveryOption::RxPDOs});
    }

    auto& sl = master_->slave(slave_index_);

    if (info_) {
        if (info_->vendor_id && info_->product_code && !matches(*info_)) {
            TETHER_LOGE(TAG, "{}: not an EL2004 (vendor=0x{:08X} product=0x{:08X})",
                        master_->slaveLogPrefix(slave_index_).c_str(),
                        *info_->vendor_id, *info_->product_code);
            return std::unexpected(Error::NotAnEL2004);
        }
        if (info_->device_name) sl.setName(*info_->device_name);
        else                    sl.setName("EL2004");
    }

    resolveOutputSm();
    TETHER_LOGI(TAG, "{}: output SM channel {} @ 0x{:04X} len {} ctrl 0x{:02X}",
                master_->slaveLogPrefix(slave_index_).c_str(), sm_channel_,
                sm_addr_, sm_len_, sm_ctrl_);

    // The EL2004 has no mailbox — satisfy the PRE-OP gate without touching
    // SM0/SM1 (SM0 carries the process outputs on this terminal).
    sl.assumeMailboxAlreadyConfigured();

    // Register the process-data SM under its actual channel index so
    // configureSlavesSMs() writes the right ESC register block
    // (0x0800 + channel*8).  sm[2] only receives the physical base address
    // for finalizeMapping()'s RxPDO offset bookkeeping — its type stays
    // Unused so no SM2 registers are written (the EL2004 has no SM2).
    auto* cfgs = master_->pdo().slaveConfigs();
    cfgs[slave_index_].sm[sm_channel_] =
        PDO::SyncManagerConfig::process_output(sm_addr_, sm_len_);
    cfgs[slave_index_].sm[sm_channel_].control =
        std::bit_cast<SyncManager::SMControlReg>(sm_ctrl_);
    if (sm_channel_ != 2) {
        cfgs[slave_index_].sm[2].phys_start_addr = sm_addr_;
    }
    if (!master_->pdo().configureSlavesSMs(slave_index_)) {
        return std::unexpected(Error::SmConfigFailed);
    }

    if (sl.transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }

    // All four channels share one byte; bit N = channel N.
    // Position addressing (APWR straight into the SM buffer) needs no FMMU.
    int entry = master_->pdo().mapping().add_rxpdo(
        slave_index_, state_.get(), sm_len_, kRxPdoIndex,
        PDO::PDOAddressMode::Position);
    if (entry < 0) {
        return std::unexpected(Error::PdoRegistrationFailed);
    }
    master_->pdo().finalizeMapping(slave_index_);

    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }

    configured_ = true;
    return {};
}

EL2004::Result<> EL2004::requestOp(int timeout_ms) {
    if (!master_->requestSlaveApplicationLayerState(
            SlaveAddress(slave_index_),
            static_cast<uint8_t>(SlaveState::OP) | 0x10)) {
        return std::unexpected(Error::OpRequestFailed);
    }
    if (master_->isCancelRequested()) {
        return std::unexpected(Error::Cancelled);
    }
    if (!waitAlState(SlaveState::OP, timeout_ms)) {
        auto& sl = master_->slave(slave_index_);
        sl.readALStatusCode(last_al_status_code_);
        if (master_->isCancelRequested()) {
            return std::unexpected(Error::Cancelled);
        }
        TETHER_LOGE(TAG, "{}: OP not confirmed (AL status: {} 0x{:04X})",
                    master_->slaveLogPrefix(slave_index_).c_str(),
                    getALStatusCodeName(last_al_status_code_),
                    last_al_status_code_);
        return std::unexpected(Error::OpTimeout);
    }
    return {};
}

EL2004::Result<> EL2004::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;

    if (opts.manage_realtime_loop && !master_->isMotionControlLoopRunning()) {
        master_->setMotionControlCallback(
            [this](double) { master_->pdo().exchangeAll(); return true; });
        Master::RealtimeMotionLoopConfig cfg;
        cfg.cycle_period_us           = opts.cycle_period_us;
        cfg.enable_dc_synchronization = false;
        if (!master_->startRealtimeMotionControlLoop(cfg)) {
            return std::unexpected(Error::LoopStartFailed);
        }
        loop_started_ = true;
    }

    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — outputs live",
                master_->slaveLogPrefix(slave_index_).c_str());
    return {};
}

void EL2004::stop() {
    allOff();
    if (loop_started_) {
        // Let a few cycles push the zeroed byte to the terminal.
        Tether::Platform::Clock::instance().delayMilliseconds(20);
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

void EL2004::set(size_t channel, bool on) {
    if (channel >= kNumChannels) return;
    const uint8_t mask = static_cast<uint8_t>(1u << channel);
    if (on) state_->fetch_or(mask, std::memory_order_relaxed);
    else    state_->fetch_and(static_cast<uint8_t>(~mask & 0x0F),
                              std::memory_order_relaxed);
}

bool EL2004::get(size_t channel) const {
    if (channel >= kNumChannels) return false;
    return (raw() >> channel) & 1u;
}

void EL2004::setChannels(Channels bits) {
    setRaw(static_cast<uint8_t>(bits.to_ulong() & 0x0F));
}

void EL2004::setRaw(uint8_t byte) {
    state_->store(static_cast<uint8_t>(byte & 0x0F),
                  std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

SlaveState EL2004::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool EL2004::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
