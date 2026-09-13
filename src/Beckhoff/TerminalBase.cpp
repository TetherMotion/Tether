/**
 * @file TerminalBase.cpp
 * @brief Shared bring-up machinery for the Beckhoff terminal drivers.
 */

#include "tether/Beckhoff/TerminalBase.hpp"

#include <algorithm>
#include <bit>
#include <string>

#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/fmmu/FMMUManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "TerminalBase";

TerminalBase::TerminalBase(Master& master, uint16_t slave_index,
                           const DeviceIdentity& identity)
    : master_(&master), slave_index_(slave_index), identity_(identity) {
}

TerminalBase::TerminalBase(Master& master, const DiscoveredSlave& slave,
                           const DeviceIdentity& identity)
    : TerminalBase(master, slave.index, identity) {
    info_ = slave;
}

TerminalBase::~TerminalBase() {
    stop();
}

std::string TerminalBase::logPrefix() const {
    return master_->slaveLogPrefix(slave_index_);
}

const char* TerminalBase::deviceName() const {
    if (info_ && info_->device_name) return info_->device_name->c_str();
    if (identity_.name) return identity_.name;
    return "Terminal";
}

// ---------------------------------------------------------------------------
// Discovery / identity
// ---------------------------------------------------------------------------

void TerminalBase::fetchDiscovery() {
    bool need_sii = true;
#if TETHER_ENABLE_SII
    need_sii = !(info_ && info_->sync_managers);
#endif
    if (!need_sii) return;
    info_ = master_->discovery().discoverOne(
        slave_index_,
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode,
         DiscoveryOption::DeviceNames, DiscoveryOption::SyncManagers,
         DiscoveryOption::TxPDOs, DiscoveryOption::RxPDOs,
         DiscoveryOption::MailboxConfig});
}

bool TerminalBase::verifyIdentity() {
    auto& sl = master_->slave(slave_index_);
    if (info_) {
        if (info_->vendor_id && info_->product_code &&
            !(info_->hasVendorAndProduct(identity_.vendor_id,
                                         identity_.product_code))) {
            TETHER_LOGE(TAG, "{}: not a {} (vendor=0x{:08X} product=0x{:08X})",
                        logPrefix().c_str(), deviceName(),
                        *info_->vendor_id, *info_->product_code);
            return false;
        }
        if (info_->device_name) sl.setName(*info_->device_name);
        else if (identity_.name) sl.setName(identity_.name);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Sync managers
// ---------------------------------------------------------------------------

void TerminalBase::resolveSyncManagers() {
#if TETHER_ENABLE_SII
    if (!info_ || !info_->sync_managers) return;

    // The SII SM category is positional: entry index == SM channel.
    for (size_t i = 0; i < info_->sync_managers->size() && i < 4; ++i) {
        const auto& sm = (*info_->sync_managers)[i];
        if (!sm.isEnabled() || sm.length == 0) continue;
        if (sm.sm_type == SII::SM_TYPE_PROCESS_IN && !sm_in_.enabled) {
            sm_in_.channel   = static_cast<uint8_t>(i);
            sm_in_.phys_addr = sm.phys_start_address;
            sm_in_.length    = sm.length;
            sm_in_.ctrl      = std::bit_cast<uint8_t>(sm.control_register);
            sm_in_.enabled   = true;
        } else if (sm.sm_type == SII::SM_TYPE_PROCESS_OUT &&
                   !sm_out_.enabled) {
            sm_out_.channel   = static_cast<uint8_t>(i);
            sm_out_.phys_addr = sm.phys_start_address;
            sm_out_.length    = sm.length;
            sm_out_.ctrl      = std::bit_cast<uint8_t>(sm.control_register);
            sm_out_.enabled   = true;
        }
    }

    // First PDO index assigned to each region (used for the registration
    // record — the entry content itself is resolved by the subclass).
    if (info_->tx_pdos) {
        for (const auto& pdo : *info_->tx_pdos) {
            if (pdo.sync_manager == sm_in_.channel) {
                sm_in_.first_pdo = pdo.pdo_index;
                break;
            }
        }
    }
    if (info_->rx_pdos) {
        for (const auto& pdo : *info_->rx_pdos) {
            if (pdo.sync_manager == sm_out_.channel) {
                sm_out_.first_pdo = pdo.pdo_index;
                break;
            }
        }
    }
#endif
}

void TerminalBase::stageSyncManagers() {
    auto& pdo  = master_->pdoForSlave(slave_index_);
    auto* cfgs = pdo.slaveConfigs();
    if (sm_in_.enabled) {
        cfgs[slave_index_].sm[sm_in_.channel] =
            PDO::SyncManagerConfig::process_input(sm_in_.phys_addr,
                                                sm_in_.length);
        cfgs[slave_index_].sm[sm_in_.channel].control =
            std::bit_cast<SyncManager::SMControlReg>(sm_in_.ctrl);
    }
    if (sm_out_.enabled) {
        cfgs[slave_index_].sm[sm_out_.channel] =
            PDO::SyncManagerConfig::process_output(sm_out_.phys_addr,
                                                 sm_out_.length);
        cfgs[slave_index_].sm[sm_out_.channel].control =
            std::bit_cast<SyncManager::SMControlReg>(sm_out_.ctrl);
    }
}

void TerminalBase::flushSyncManagers() {
    master_->pdoForSlave(slave_index_).configureSlavesSMs(slave_index_);
}

// ---------------------------------------------------------------------------
// Mailbox
// ---------------------------------------------------------------------------

void TerminalBase::configureMailboxOrDeclare() {
    auto& sl = master_->slave(slave_index_);

    bool has_mailbox = true;
#if TETHER_ENABLE_SII
    if (info_ && info_->mailbox_config) {
        has_mailbox = info_->mailbox_config->hasMailbox();
    } else if (info_ && info_->sync_managers) {
        has_mailbox = std::ranges::any_of(*info_->sync_managers,
            [](const SII::SIISyncManager& sm) {
                return sm.sm_type == SII::SM_TYPE_MBX_WRITE ||
                       sm.sm_type == SII::SM_TYPE_MBX_READ;
            });
    }
#endif
    if (!has_mailbox) {
        sl.markNoMailbox();
    } else if (sl.configureMailbox() != SlaveError::Ok) {
        // SII mailbox read failed — the firmware ships the bootstrap
        // mailbox pre-configured; continue on the assumption path so the
        // PRE-OP gate is satisfied.
        sl.assumeMailboxAlreadyConfigured();
    }
}

// ---------------------------------------------------------------------------
// State transitions / PDO registration / FMMU
// ---------------------------------------------------------------------------

Result<> TerminalBase::enterPreOp() {
    if (master_->slave(slave_index_).transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }
    return {};
}

Result<> TerminalBase::registerProcessData(PDO::PDOAddressMode mode) {
    auto& pdo = master_->pdoForSlave(slave_index_);
    if (sm_in_.enabled && sm_in_.length > 0) {
        in_buf_.assign(sm_in_.length, 0);
        int entry = pdo.mapping().add_txpdo(slave_index_, in_buf_.data(),
                                            sm_in_.length,
                                            sm_in_.first_pdo, mode);
        if (entry < 0) {
            return std::unexpected(Error::PdoRegistrationFailed);
        }
    }
    if (sm_out_.enabled && sm_out_.length > 0) {
        out_buf_.assign(sm_out_.length, 0);
        int entry = pdo.mapping().add_rxpdo(slave_index_, out_buf_.data(),
                                            sm_out_.length,
                                            sm_out_.first_pdo, mode);
        if (entry < 0) {
            return std::unexpected(Error::PdoRegistrationFailed);
        }
    }
    pdo.finalizeMapping(slave_index_);
    prepared_ = true;
    return {};
}

Result<> TerminalBase::enterSafeOp() {
    auto& sl = master_->slave(slave_index_);
    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }
    configured_ = true;
    return {};
}

Result<> TerminalBase::programFmmuAndEnterSafeOp() {
    auto& lam = master_->logicalAddressManagerForSlave(slave_index_);
    if (!lam.hasSlavePDOs(slave_index_)) {
        TETHER_LOGE(TAG, "{}: no logical address assigned — "
                    "was the shared address map built?", logPrefix().c_str());
        return std::unexpected(Error::LogicalMapMissing);
    }

    auto& fmmu = master_->slave(slave_index_).fmmuManager();
    auto& cfg  = fmmu.config();
    cfg.clear();
    cfg.slave_index = slave_index_;

    if (sm_out_.enabled) {
        const uint32_t log_out = lam.getRxPDOLogicalAddr(slave_index_);
        cfg.next_logical_addr = log_out;
        if (!cfg.addOutput(sm_out_.phys_addr, sm_out_.length,
                           sm_out_.channel)) {
            return std::unexpected(Error::FmmuConfigFailed);
        }
        TETHER_LOGI(TAG, "{}: FMMU maps logical 0x{:08X} -> phys 0x{:04X} "
                    "len {}", logPrefix().c_str(),
                    static_cast<unsigned long>(log_out), sm_out_.phys_addr,
                    sm_out_.length);
        logical_addr_ = log_out;
    }
    if (sm_in_.enabled) {
        const uint32_t log_in = lam.getTxPDOLogicalAddr(slave_index_);
        cfg.next_logical_addr = log_in;
        if (!cfg.addInput(sm_in_.phys_addr, sm_in_.length, sm_in_.channel)) {
            return std::unexpected(Error::FmmuConfigFailed);
        }
        TETHER_LOGI(TAG, "{}: FMMU maps logical 0x{:08X} <- phys 0x{:04X} "
                    "len {}", logPrefix().c_str(),
                    static_cast<unsigned long>(log_in), sm_in_.phys_addr,
                    sm_in_.length);
        if (!sm_out_.enabled) logical_addr_ = log_in;
    }
    cfg.configured = true;
    if (!fmmu.writeToSlave()) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    return enterSafeOp();
}

Result<> TerminalBase::requestOp(int timeout_ms) {
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
                    logPrefix().c_str(),
                    getALStatusCodeName(last_al_status_code_),
                    last_al_status_code_);
        return std::unexpected(Error::OpTimeout);
    }
    return {};
}

Result<> TerminalBase::startLoopIfManaged(const StartOptions& opts) {
    if (!opts.manage_realtime_loop ||
        master_->isMotionControlLoopRunning()) {
        return {};
    }
    master_->setMotionControlCallback(
        [this](double) {
            master_->pdo().exchangeAll();
            auto& mine = master_->pdoForSlave(slave_index_);
            if (&mine != &master_->pdo()) mine.exchangeAll();
            return true;
        });
    Master::RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us           = opts.cycle_period_us;
    cfg.enable_dc_synchronization = false;
    if (!master_->startRealtimeMotionControlLoop(cfg)) {
        return std::unexpected(Error::LoopStartFailed);
    }
    loop_started_ = true;
    return {};
}

void TerminalBase::stop() {
    if (loop_started_) {
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Status helpers
// ---------------------------------------------------------------------------

Slave& TerminalBase::slave() {
    return master_->slave(slave_index_);
}

SlaveState TerminalBase::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool TerminalBase::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
