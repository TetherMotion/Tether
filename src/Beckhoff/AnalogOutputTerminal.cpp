/**
 * @file AnalogOutputTerminal.cpp
 * @brief Implementation of the generic analog-output terminal driver.
 *
 * See AnalogOutputTerminal.hpp for the API documentation.  Bring-up
 * mirrors AnalogInputTerminal with reversed process-data roles: SM2 is
 * the output image, and an enabled SM3 (EL407x status, EL4374 inputs)
 * is registered + mapped as a readable scratch region.
 */

#include "tether/Beckhoff/AnalogOutputTerminal.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstring>

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

static const char* TAG = "AnalogOutputTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

AnalogOutputTerminal::AnalogOutputTerminal(Master& master, uint16_t slave_index,
                                           const DeviceIdentity& identity)
    : master_(&master), slave_index_(slave_index), identity_(identity) {
}

AnalogOutputTerminal::AnalogOutputTerminal(Master& master,
                                           const DiscoveredSlave& slave,
                                           const DeviceIdentity& identity)
    : AnalogOutputTerminal(master, slave.index, identity) {
    info_ = slave;
}

AnalogOutputTerminal::~AnalogOutputTerminal() {
    stop();
}

Result<AnalogOutputTerminal> AnalogOutputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<AnalogOutputTerminal> AnalogOutputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return AnalogOutputTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

const char* AnalogOutputTerminal::deviceName() const {
    if (info_ && info_->device_name) return info_->device_name->c_str();
    if (identity_.name) return identity_.name;
    return "AnalogOutputTerminal";
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

#if TETHER_ENABLE_SII
namespace {

/// True for a channel-value PDO entry: any entry of at least 16 bit.
/// Value object indices vary by family/revision (0x7xxx, 0x6411, 0x300x),
/// so width alone is the robust discriminator; the last such entry in a
/// PDO is the channel value.
inline bool isAnalogValueEntry(const SII::SIIPDOEntry& e) {
    return e.bit_length >= 16;
}

} // anonymous namespace
#endif

bool AnalogOutputTerminal::resolveLayout() {
#if TETHER_ENABLE_SII
    // --- Process-data sync managers -----------------------------------------
    // The SII SM category is positional: entry index == SM channel.
    if (info_ && info_->sync_managers) {
        for (size_t i = 0; i < info_->sync_managers->size() && i < 4; ++i) {
            const auto& sm = (*info_->sync_managers)[i];
            if (sm.sm_type == SII::SM_TYPE_PROCESS_OUT && sm.isEnabled() &&
                sm.length > 0) {
                sm_out_channel_ = static_cast<uint8_t>(i);
                sm_out_addr_    = sm.phys_start_address;
                sm_out_len_     = sm.length;
                sm_out_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
            } else if (sm.sm_type == SII::SM_TYPE_PROCESS_IN &&
                       sm.isEnabled() && sm.length > 0) {
                sm_in_enabled_ = true;
                sm_in_addr_    = sm.phys_start_address;
                sm_in_len_     = sm.length;
                sm_in_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
            }
        }
    }

    // --- Per-channel layout from the RxPDOs assigned to the output SM ------
    if (info_ && info_->rx_pdos && sm_out_len_ > 0) {
        channels_.clear();
        uint32_t running_bit = 0;
        bool     first       = true;

        for (const auto& pdo : *info_->rx_pdos) {
            if (pdo.sync_manager != sm_out_channel_) continue;
            if (first) { first_rxpdo_ = pdo.pdo_index; first = false; }

            uint32_t entry_bit = 0;
            const SII::SIIPDOEntry* value_entry = nullptr;
            uint32_t value_entry_bit = 0;
            for (const auto& e : pdo.entries) {
                if (isAnalogValueEntry(e)) {
                    value_entry     = &e;
                    value_entry_bit = running_bit + entry_bit;
                }
                entry_bit += e.bit_length;
            }

            if (value_entry) {
                if ((running_bit % 8) || (value_entry_bit % 8) ||
                    (pdo.totalBits() % 8)) {
                    TETHER_LOGE(TAG, "{}: non-byte-aligned PDO layout in "
                                "0x{:04X} — unsupported",
                                master_->slaveLogPrefix(slave_index_).c_str(),
                                pdo.pdo_index);
                    channels_.clear();
                    return false;
                }
                ChannelLayout ch{};
                ch.value_off  = static_cast<uint16_t>(value_entry_bit / 8);
                ch.value_bits = value_entry->bit_length > 32
                              ? 32 : value_entry->bit_length;
                channels_.push_back(ch);
            }
            running_bit += pdo.totalBits();
        }

        const uint32_t image_bytes = running_bit / 8;
        if (channels_.empty() || image_bytes > sm_out_len_) {
            TETHER_LOGE(TAG, "{}: RxPDO layout resolves to {}B over {} "
                        "channel(s) but SM{} holds {}B — unsupported "
                        "terminal shape",
                        master_->slaveLogPrefix(slave_index_).c_str(),
                        image_bytes, channels_.size(), sm_out_channel_,
                        sm_out_len_);
            channels_.clear();
            return false;
        }
    }

    // TxPDO index bookkeeping (only relevant when SM3 is enabled).
    if (sm_in_enabled_ && info_ && info_->tx_pdos) {
        for (const auto& pdo : *info_->tx_pdos) {
            if (pdo.sync_manager == 3) { first_txpdo_ = pdo.pdo_index; break; }
        }
    }
#endif
    // Fallback when SII resolution was impossible: dominant Beckhoff shape —
    // identity num_bits channels of 2B each on SM2 @ 0x1100.
    if (sm_out_len_ == 0 && identity_.num_bits > 0 &&
        identity_.num_bits <= kMaxChannels) {
        sm_out_len_     = static_cast<uint16_t>(identity_.num_bits * 2);
        sm_out_addr_    = 0x1100;
        sm_out_ctrl_    = 0x24;
        sm_out_channel_ = 2;
        for (uint16_t ch = 0; ch < identity_.num_bits; ++ch) {
            channels_.push_back(
                ChannelLayout{static_cast<uint16_t>(ch * 2), 16});
        }
    }
    return sm_out_len_ > 0 && !channels_.empty() &&
           channels_.size() <= kMaxChannels &&
           sm_out_len_ <= kMaxImageBytes &&
           sm_in_len_ <= kMaxImageBytes;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> AnalogOutputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    bool need_sii = true;
#if TETHER_ENABLE_SII
    need_sii = !(info_ && info_->sync_managers && info_->rx_pdos);
#endif
    if (need_sii) {
        info_ = master_->discovery().discoverOne(
            slave_index_,
            {DiscoveryOption::VendorId, DiscoveryOption::ProductCode,
             DiscoveryOption::DeviceNames, DiscoveryOption::SyncManagers,
             DiscoveryOption::TxPDOs, DiscoveryOption::RxPDOs,
             DiscoveryOption::MailboxConfig});
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

    if (!resolveLayout()) {
        TETHER_LOGE(TAG, "{}: no usable analog-output layout "
                    "(SM2 disabled, zero-length, or unrecognized PDO shape)",
                    master_->slaveLogPrefix(slave_index_).c_str());
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: output SM{} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} analog channel(s)){}",
                master_->slaveLogPrefix(slave_index_).c_str(),
                sm_out_channel_, sm_out_addr_, sm_out_len_, sm_out_ctrl_,
                channels_.size(),
                sm_in_enabled_ ? " +SM3 inputs" : "");

    // Register the process-data SMs under their actual channel indices so
    // the upcoming configureSlavesSMs() — invoked inside configureMailbox —
    // writes all four ESC register blocks in one pass.
    auto& pdo   = master_->pdoForSlave(slave_index_);
    auto* cfgs  = pdo.slaveConfigs();
    cfgs[slave_index_].sm[sm_out_channel_] =
        PDO::SyncManagerConfig::process_output(sm_out_addr_, sm_out_len_);
    cfgs[slave_index_].sm[sm_out_channel_].control =
        std::bit_cast<SyncManager::SMControlReg>(sm_out_ctrl_);
    if (sm_in_enabled_) {
        cfgs[slave_index_].sm[3] =
            PDO::SyncManagerConfig::process_input(sm_in_addr_, sm_in_len_);
        cfgs[slave_index_].sm[3].control =
            std::bit_cast<SyncManager::SMControlReg>(sm_in_ctrl_);
    }

    // Mailbox: these are CoE devices — configure SM0/SM1 + the SDO channel
    // from SII.
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
        sl.assumeMailboxAlreadyConfigured();
    }

    // Write the process-data SM registers unconditionally — configureMailbox
    // already wrote them on success, but a failed mailbox config or a
    // declared no-mailbox variant must still get SM2/SM3.
    pdo.configureSlavesSMs(slave_index_);

    if (sl.transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }

    out_buf_.assign(sm_out_len_, 0);
    int entry = pdo.mapping().add_rxpdo(
        slave_index_, out_buf_.data(), sm_out_len_, first_rxpdo_, mode);
    if (entry < 0) {
        return std::unexpected(Error::PdoRegistrationFailed);
    }
    if (sm_in_enabled_) {
        in_buf_.assign(sm_in_len_, 0);
        entry = pdo.mapping().add_txpdo(
            slave_index_, in_buf_.data(), sm_in_len_, first_txpdo_, mode);
        if (entry < 0) {
            return std::unexpected(Error::PdoRegistrationFailed);
        }
    }
    pdo.finalizeMapping(slave_index_);

    prepared_ = true;
    return {};
}

Result<> AnalogOutputTerminal::enterSafeOp() {
    auto& sl = master_->slave(slave_index_);
    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }
    configured_ = true;
    return {};
}

Result<> AnalogOutputTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return enterSafeOp();
}

Result<> AnalogOutputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> AnalogOutputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }

    auto& lam = master_->logicalAddressManagerForSlave(slave_index_);
    if (!lam.hasSlavePDOs(slave_index_)) {
        TETHER_LOGE(TAG, "{}: no logical address assigned — "
                    "was the shared address map built?",
                    master_->slaveLogPrefix(slave_index_).c_str());
        return std::unexpected(Error::LogicalMapMissing);
    }
    const uint32_t log_out = lam.getRxPDOLogicalAddr(slave_index_);

    auto& fmmu = master_->slave(slave_index_).fmmuManager();
    auto& cfg  = fmmu.config();
    cfg.clear();
    cfg.slave_index       = slave_index_;
    cfg.next_logical_addr = log_out;
    if (!cfg.addOutput(sm_out_addr_, sm_out_len_, sm_out_channel_)) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    if (sm_in_enabled_) {
        const uint32_t log_in = lam.getTxPDOLogicalAddr(slave_index_);
        cfg.next_logical_addr = log_in;
        if (!cfg.addInput(sm_in_addr_, sm_in_len_, 3)) {
            return std::unexpected(Error::FmmuConfigFailed);
        }
    }
    cfg.configured = true;
    if (!fmmu.writeToSlave()) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    logical_addr_ = log_out;
    TETHER_LOGI(TAG, "{}: FMMU maps logical 0x{:08X} -> phys 0x{:04X} len {}",
                master_->slaveLogPrefix(slave_index_).c_str(),
                static_cast<unsigned long>(log_out), sm_out_addr_,
                sm_out_len_);

    return enterSafeOp();
}

Result<> AnalogOutputTerminal::requestOp(int timeout_ms) {
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

Result<> AnalogOutputTerminal::start(const StartOptions& opts) {
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

void AnalogOutputTerminal::stop() {
    if (loop_started_) {
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

void AnalogOutputTerminal::setValue(size_t channel, int32_t value) {
    if (channel >= channels_.size()) return;
    const auto& ch = channels_[channel];
    if (ch.value_off + ch.value_bits / 8 > out_buf_.size()) return;
    uint8_t* p = out_buf_.data() + ch.value_off;
    if (ch.value_bits > 16) {
        std::memcpy(p, &value, 4);
    } else {
        const int16_t v = static_cast<int16_t>(value);
        std::memcpy(p, &v, 2);
    }
}

int32_t AnalogOutputTerminal::value(size_t channel) const {
    if (channel >= channels_.size()) return 0;
    const auto& ch = channels_[channel];
    if (ch.value_off + ch.value_bits / 8 > out_buf_.size()) return 0;
    const uint8_t* p = out_buf_.data() + ch.value_off;
    if (ch.value_bits > 16) {
        int32_t v;
        std::memcpy(&v, p, 4);
        return v;
    }
    int16_t v;
    std::memcpy(&v, p, 2);
    return v;
}

void AnalogOutputTerminal::allZero() {
    std::ranges::fill(out_buf_, uint8_t{0});
}

size_t AnalogOutputTerminal::channelBits(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    return channels_[ch].value_bits;
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

SlaveState AnalogOutputTerminal::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool AnalogOutputTerminal::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
