/**
 * @file InputTerminal.cpp
 * @brief Implementation of the generic packed-bit input terminal driver.
 *
 * Input-direction mirror of OutputTerminal.cpp — see InputTerminal.hpp for
 * the API documentation.  The bring-up sequence works around three
 * framework assumptions which do not hold for this terminal family
 * (single "Inputs" SM — usually channel 0 — no mailbox, no RxPDO):
 *
 *   1. No mailbox.  The terminal's only sync manager carries process
 *      inputs — configureMailbox() would overwrite those registers.
 *      markNoMailbox() declares the absence (detected from SII) so the
 *      framework skips all mailbox handling while the PRE-OP gate is
 *      satisfied vacuously.
 *   2. Process inputs on a non-SM3 channel.  The PDOManager bookkeeping
 *      expects TxPDO data at SM3.  The real config is registered under
 *      its actual channel index (so configureSlavesSMs() writes the
 *      correct ESC register block); afterwards the channel-0 SM is
 *      *mirrored* into sm[3] bookkeeping with enable=false — enough for
 *      finalizeMapping() and LogicalAddressManager::buildAddressMap(),
 *      while no SM3 register writes or verification happen (the terminal
 *      has no SM3).
 *   3. No RxPDO.  Slave::transitionToOp() waits for a request counter
 *      that an input-only slave can never increment, so OP is requested
 *      directly.
 *
 * SII quirk: these terminals report SM length 0 in their EEPROM.  The
 * real size is the sum of the TxPDO entry bit lengths (4 x 1 bit ->
 * 1 byte for the EL1014, 8 x 1 bit -> 1 byte for the EL1008, ...).
 */

#include "tether/Beckhoff/InputTerminal.hpp"

#include <algorithm>
#include <bit>
#include <chrono>

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

static const char* TAG = "InputTerminal";

// ESI fallbacks (Beckhoff EL1xxx.xml), used when SII data is unavailable.
constexpr uint8_t  kFallbackSmCtrl = 0x00;  // buffered, ECAT-read, no watchdog
constexpr uint16_t kFallbackSmAddr = 0x1000;
constexpr uint16_t kFallbackSmLen  = 1;

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

InputTerminal::InputTerminal(Master& master, uint16_t slave_index,
                         const DeviceIdentity& identity)
    : master_(&master), slave_index_(slave_index), identity_(identity),
      state_(std::make_unique<std::atomic<uint64_t>>(0)) {
    num_inputs_ = identity.num_bits;
}

InputTerminal::InputTerminal(Master& master, const DiscoveredSlave& slave,
                         const DeviceIdentity& identity)
    : InputTerminal(master, slave.index, identity) {
    info_ = slave;
}

InputTerminal::~InputTerminal() {
    stop();
}

Result<InputTerminal> InputTerminal::findFirst(Master& master,
                                           const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<InputTerminal> InputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return InputTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

const char* InputTerminal::deviceName() const {
    if (info_ && info_->device_name) return info_->device_name->c_str();
    if (identity_.name) return identity_.name;
    return "InputTerminal";
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

void InputTerminal::resolveInputSm() {
#if TETHER_ENABLE_SII
    // The SII SM category is positional: entry index == SM channel.
    // This terminal family has exactly one SM (channel 0, process inputs
    // @ 0x1000).
    if (info_ && info_->sync_managers) {
        for (size_t i = 0; i < info_->sync_managers->size() && i < 4; ++i) {
            const auto& sm = (*info_->sync_managers)[i];
            if (sm.sm_type == SII::SM_TYPE_PROCESS_IN) {
                sm_channel_ = static_cast<uint8_t>(i);
                sm_addr_    = sm.phys_start_address;
                sm_len_     = sm.length;
                sm_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
                break;
            }
        }
    }

    // EL terminals report SM length 0 in SII — derive the real size from
    // the TxPDO entries (N x 1 bit).  The first TxPDO index is kept for
    // mapping bookkeeping.
    if (info_ && info_->tx_pdos && !info_->tx_pdos->empty()) {
        size_t total_bits = 0;
        for (const auto& pdo : *info_->tx_pdos) {
            total_bits += pdo.totalBits();
        }
        if (sm_len_ == 0) {
            sm_len_ = static_cast<uint16_t>((total_bits + 7) / 8);
        }
        txpdo_index_ = (*info_->tx_pdos)[0].pdo_index;
        if (num_inputs_ == 0) {
            num_inputs_ = std::min(total_bits, kMaxBits);
        }
    }
#endif
    if (sm_len_ == 0) sm_len_ = kFallbackSmLen;
    if (num_inputs_ == 0) {
        num_inputs_ = std::min<size_t>(size_t(sm_len_) * 8, kMaxBits);
    }
    if (num_inputs_ > kMaxBits) num_inputs_ = kMaxBits;
}

Result<> InputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

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
             DiscoveryOption::TxPDOs, DiscoveryOption::MailboxConfig});
    }

    auto& sl = master_->slave(slave_index_);

    if (info_) {
        if (info_->vendor_id && info_->product_code &&
            !matches(*info_, identity_)) {
            TETHER_LOGE(TAG, "{}: not a {} (vendor=0x{:08X} product=0x{:08X})",
                        master_->slaveLogPrefix(slave_index_).c_str(),
                        deviceName(),
                        *info_->vendor_id, *info_->product_code);
            return std::unexpected(Error::WrongDevice);
        }
        if (info_->device_name) sl.setName(*info_->device_name);
        else if (identity_.name) sl.setName(identity_.name);
    }

    resolveInputSm();
    TETHER_LOGI(TAG, "{}: input SM channel {} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} input bits)",
                master_->slaveLogPrefix(slave_index_).c_str(), sm_channel_,
                sm_addr_, sm_len_, sm_ctrl_, num_inputs_);

    // Mailbox handling: this terminal family has no mailbox — its only
    // sync manager carries process data.  Ask the SII when readable
    // (the Mailbox category is authoritative; a mailbox-typed SM entry
    // works as fallback); on SII failure the family property holds.
    // markNoMailbox() satisfies the PRE-OP gate while suppressing every
    // mailbox interaction (drain, SM0/SM1 validation, SDO setup).
    bool has_mailbox = false;
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
    if (has_mailbox) {
        // Not expected for this family — satisfy the gate via the
        // firmware-pre-configured assumption instead of declaring absence.
        sl.assumeMailboxAlreadyConfigured();
    } else {
        sl.markNoMailbox();
    }

    // Register the process-data SM under its actual channel index so
    // configureSlavesSMs() writes the right ESC register block
    // (0x0800 + channel*8).
    auto& pdo   = master_->pdoForSlave(slave_index_);
    auto* cfgs  = pdo.slaveConfigs();
    cfgs[slave_index_].sm[sm_channel_] =
        PDO::SyncManagerConfig::process_input(sm_addr_, sm_len_);
    cfgs[slave_index_].sm[sm_channel_].control =
        std::bit_cast<SyncManager::SMControlReg>(sm_ctrl_);
    if (!pdo.configureSlavesSMs(slave_index_)) {
        return std::unexpected(Error::SmConfigFailed);
    }

    if (sl.transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }

    // Mirror the real input SM into the sm[3] bookkeeping slot AFTER the
    // register write (configureSlavesSMs writes every non-Unused channel):
    // enable=false keeps it out of register writes and SM verification, but
    // type=ProcessInput lets finalizeMapping() record txpdo_size — which
    // LogicalAddressManager::buildAddressMap() needs to assign this slave a
    // range in the shared logical address space.  (Skipped when the real
    // input SM already is channel 3 — then sm[3] holds the live config.)
    if (sm_channel_ != 3) {
        cfgs[slave_index_].sm[3].type           = PDO::SyncManagerType::ProcessInput;
        cfgs[slave_index_].sm[3].phys_start_addr = sm_addr_;
        cfgs[slave_index_].sm[3].enable         = false;
    }

    int entry = pdo.mapping().add_txpdo(
        slave_index_, state_.get(), sm_len_, txpdo_index_, mode);
    if (entry < 0) {
        return std::unexpected(Error::PdoRegistrationFailed);
    }
    pdo.finalizeMapping(slave_index_);

    prepared_ = true;
    return {};
}

Result<> InputTerminal::enterSafeOp() {
    auto& sl = master_->slave(slave_index_);
    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }
    configured_ = true;
    return {};
}

Result<> InputTerminal::configure() {
    // Standalone: position addressing (APRD straight from the SM buffer)
    // needs no FMMU and no logical map.
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return enterSafeOp();
}

Result<> InputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> InputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }

    // The chain must have built the logical map by now — look up the
    // address assigned to this slave in its covering LAM.
    auto& lam = master_->logicalAddressManagerForSlave(slave_index_);
    if (!lam.hasSlavePDOs(slave_index_)) {
        TETHER_LOGE(TAG, "{}: no logical address assigned — "
                    "was the shared address map built?",
                    master_->slaveLogPrefix(slave_index_).c_str());
        return std::unexpected(Error::LogicalMapMissing);
    }
    const uint32_t log = lam.getTxPDOLogicalAddr(slave_index_);

    // Program the input FMMU: map our physical input SM buffer onto the
    // logical input range.  (addInput() defaults to sm_index 3; this
    // family's input SM is channel 0, so pass the real channel.)
    auto& fmmu = master_->slave(slave_index_).fmmuManager();
    auto& cfg  = fmmu.config();
    cfg.clear();
    cfg.slave_index       = slave_index_;
    cfg.next_logical_addr = log;
    if (!cfg.addInput(sm_addr_, sm_len_, sm_channel_)) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    cfg.configured = true;
    if (!fmmu.writeToSlave()) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    logical_addr_ = log;
    TETHER_LOGI(TAG, "{}: FMMU maps phys 0x{:04X} len {} -> logical 0x{:08X}",
                master_->slaveLogPrefix(slave_index_).c_str(),
                sm_addr_, sm_len_, static_cast<unsigned long>(log));

    return enterSafeOp();
}

Result<> InputTerminal::requestOp(int timeout_ms) {
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

Result<> InputTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;

    if (opts.manage_realtime_loop && !master_->isMotionControlLoopRunning()) {
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
    }

    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — inputs live",
                master_->slaveLogPrefix(slave_index_).c_str());
    return {};
}

void InputTerminal::stop() {
    if (loop_started_) {
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

bool InputTerminal::bit(size_t bit) const {
    if (bit >= num_inputs_) return false;
    return (bits() >> bit) & 1u;
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

SlaveState InputTerminal::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool InputTerminal::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
