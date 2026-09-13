/**
 * @file PackedOutput.cpp
 * @brief Implementation of the generic packed-bit output terminal driver.
 *
 * See PackedOutput.hpp for the API documentation.  The bring-up sequence
 * works around three framework assumptions which do not hold for this
 * terminal family (single "Outputs" SM — usually channel 0 — no mailbox,
 * no TxPDO):
 *
 *   1. No mailbox.  The terminal's only sync manager carries process
 *      outputs — configureMailbox() would overwrite those registers.
 *      markNoMailbox() declares the absence (detected from SII) so the
 *      framework skips all mailbox handling while the PRE-OP gate is
 *      satisfied vacuously.
 *   2. Process outputs on a non-SM2 channel.  The PDOManager bookkeeping
 *      expects RxPDO data at SM2.  The real config is registered under its
 *      actual channel index (so configureSlavesSMs() writes the correct
 *      ESC register block); afterwards the channel-0 SM is *mirrored* into
 *      sm[2] bookkeeping with enable=false — enough for finalizeMapping()
 *      and LogicalAddressManager::buildAddressMap(), while no SM2 register
 *      writes or verification happen (the terminal has no SM2).
 *   3. No TxPDO.  Slave::transitionToOp() waits for a reply counter that an
 *      output-only slave can never increment, so OP is requested directly.
 *
 * SII quirk: these terminals report SM length 0 in their EEPROM.  The real
 * size is the sum of the RxPDO entry bit lengths (4 x 1 bit -> 1 byte for
 * the EL2004, 8 x 1 bit -> 1 byte for the EL2008, 32 x 1 bit -> 4 bytes
 * for the EL2407, ...).
 */

#include "tether/Beckhoff/PackedOutput.hpp"

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

static const char* TAG = "PackedOutput";

// ESI fallbacks (Beckhoff EL2xxx.xml), used when SII data is unavailable.
constexpr uint8_t  kFallbackSmCtrl = 0x44;  // buffered, ECAT-write, no watchdog
constexpr uint16_t kFallbackSmAddr = 0x0F00;
constexpr uint16_t kFallbackSmLen  = 1;

const char* errorToString(Error e) {
    switch (e) {
        case Error::Ok:                    return "Ok";
        case Error::NoDeviceFound:         return "No matching device found on the bus";
        case Error::WrongDevice:           return "Slave is not the expected device (vendor/product mismatch)";
        case Error::SlaveIndexOutOfRange:  return "Slave index exceeds the PDO manager's slave table";
        case Error::SmConfigFailed:        return "Failed to write sync-manager registers";
        case Error::PdoRegistrationFailed: return "RxPDO buffer registration failed";
        case Error::FmmuConfigFailed:      return "Failed to program the output FMMU";
        case Error::LogicalMapMissing:     return "No logical address assigned to this slave";
        case Error::PreOpFailed:           return "PRE-OP transition failed";
        case Error::SafeOpFailed:          return "SAFE-OP transition failed";
        case Error::OpRequestFailed:       return "OP request could not be sent";
        case Error::OpTimeout:             return "Slave did not reach OP in time";
        case Error::LoopStartFailed:       return "Master realtime loop failed to start";
        case Error::TooManyBits:           return "Chain exceeds the MaxBits capacity";
        case Error::Cancelled:             return "Operation cancelled";
        default:                           return "Unknown error";
    }
}

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

PackedOutput::PackedOutput(Master& master, uint16_t slave_index,
                           const DeviceIdentity& identity)
    : master_(&master), slave_index_(slave_index), identity_(identity),
      state_(std::make_unique<std::atomic<uint64_t>>(0)) {
    num_outputs_ = identity.num_outputs;
}

PackedOutput::PackedOutput(Master& master, const DiscoveredSlave& slave,
                           const DeviceIdentity& identity)
    : PackedOutput(master, slave.index, identity) {
    info_ = slave;
}

PackedOutput::~PackedOutput() {
    stop();
}

Result<PackedOutput> PackedOutput::findFirst(Master& master,
                                             const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<PackedOutput> PackedOutput::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return PackedOutput(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

const char* PackedOutput::deviceName() const {
    if (info_ && info_->device_name) return info_->device_name->c_str();
    if (identity_.name) return identity_.name;
    return "PackedOutput";
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

void PackedOutput::resolveOutputSm() {
#if TETHER_ENABLE_SII
    // The SII SM category is positional: entry index == SM channel.
    // This terminal family has exactly one SM (channel 0, process outputs
    // @ 0x0F00).
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

    // EL terminals report SM length 0 in SII — derive the real size from
    // the RxPDO entries (N x 1 bit).  The first RxPDO index is kept for
    // mapping bookkeeping.
    if (info_ && info_->rx_pdos && !info_->rx_pdos->empty()) {
        size_t total_bits = 0;
        for (const auto& pdo : *info_->rx_pdos) {
            total_bits += pdo.totalBits();
        }
        if (sm_len_ == 0) {
            sm_len_ = static_cast<uint16_t>((total_bits + 7) / 8);
        }
        rxpdo_index_ = (*info_->rx_pdos)[0].pdo_index;
        if (num_outputs_ == 0) {
            num_outputs_ = std::min(total_bits, kMaxBits);
        }
    }
#endif
    if (sm_len_ == 0) sm_len_ = kFallbackSmLen;
    if (num_outputs_ == 0) {
        num_outputs_ = std::min<size_t>(size_t(sm_len_) * 8, kMaxBits);
    }
    if (num_outputs_ > kMaxBits) num_outputs_ = kMaxBits;
}

Result<> PackedOutput::prepare(PDO::PDOAddressMode mode) {
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
             DiscoveryOption::RxPDOs, DiscoveryOption::MailboxConfig});
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

    resolveOutputSm();
    TETHER_LOGI(TAG, "{}: output SM channel {} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} output bits)",
                master_->slaveLogPrefix(slave_index_).c_str(), sm_channel_,
                sm_addr_, sm_len_, sm_ctrl_, num_outputs_);

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
        PDO::SyncManagerConfig::process_output(sm_addr_, sm_len_);
    cfgs[slave_index_].sm[sm_channel_].control =
        std::bit_cast<SyncManager::SMControlReg>(sm_ctrl_);
    if (!pdo.configureSlavesSMs(slave_index_)) {
        return std::unexpected(Error::SmConfigFailed);
    }

    if (sl.transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }

    // Mirror the real output SM into the sm[2] bookkeeping slot AFTER the
    // register write (configureSlavesSMs writes every non-Unused channel):
    // enable=false keeps it out of register writes and SM verification, but
    // type=ProcessOutput lets finalizeMapping() record rxpdo_size — which
    // LogicalAddressManager::buildAddressMap() needs to assign this slave a
    // range in the shared logical address space.  (Skipped when the real
    // output SM already is channel 2 — then sm[2] holds the live config.)
    if (sm_channel_ != 2) {
        cfgs[slave_index_].sm[2].type           = PDO::SyncManagerType::ProcessOutput;
        cfgs[slave_index_].sm[2].phys_start_addr = sm_addr_;
        cfgs[slave_index_].sm[2].enable         = false;
    }

    int entry = pdo.mapping().add_rxpdo(
        slave_index_, state_.get(), sm_len_, rxpdo_index_, mode);
    if (entry < 0) {
        return std::unexpected(Error::PdoRegistrationFailed);
    }
    pdo.finalizeMapping(slave_index_);

    prepared_ = true;
    return {};
}

Result<> PackedOutput::enterSafeOp() {
    auto& sl = master_->slave(slave_index_);
    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }
    configured_ = true;
    return {};
}

Result<> PackedOutput::configure() {
    // Standalone: position addressing (APWR straight into the SM buffer)
    // needs no FMMU and no logical map.
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return enterSafeOp();
}

Result<> PackedOutput::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> PackedOutput::mapLogicalAndEnterSafeOp() {
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
    const uint32_t log = lam.getRxPDOLogicalAddr(slave_index_);

    // Program the output FMMU: map our logical range onto the physical
    // output SM buffer.  (configureManual() hardcodes sm_index 2; this
    // family's output SM is channel 0, so the config is built directly.)
    auto& fmmu = master_->slave(slave_index_).fmmuManager();
    auto& cfg  = fmmu.config();
    cfg.clear();
    cfg.slave_index       = slave_index_;
    cfg.next_logical_addr = log;
    if (!cfg.addOutput(sm_addr_, sm_len_, sm_channel_)) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    cfg.configured = true;
    if (!fmmu.writeToSlave()) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    logical_addr_ = log;
    TETHER_LOGI(TAG, "{}: FMMU maps logical 0x{:08X} -> phys 0x{:04X} len {}",
                master_->slaveLogPrefix(slave_index_).c_str(),
                static_cast<unsigned long>(log), sm_addr_, sm_len_);

    return enterSafeOp();
}

Result<> PackedOutput::requestOp(int timeout_ms) {
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

Result<> PackedOutput::start(const StartOptions& opts) {
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
    TETHER_LOGI(TAG, "{}: in OP — outputs live",
                master_->slaveLogPrefix(slave_index_).c_str());
    return {};
}

void PackedOutput::stop() {
    allOff();
    if (loop_started_) {
        // Let a few cycles push the zeroed field to the terminal.
        Tether::Platform::Clock::instance().delayMilliseconds(20);
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

void PackedOutput::setBit(size_t bit, bool on) {
    if (bit >= num_outputs_) return;
    const uint64_t mask = uint64_t{1} << bit;
    if (on) state_->fetch_or(mask, std::memory_order_relaxed);
    else    state_->fetch_and(~mask, std::memory_order_relaxed);
}

bool PackedOutput::bit(size_t bit) const {
    if (bit >= num_outputs_) return false;
    return (bits() >> bit) & 1u;
}

void PackedOutput::setBits(uint64_t value) {
    state_->store(value & mask(), std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

SlaveState PackedOutput::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool PackedOutput::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
