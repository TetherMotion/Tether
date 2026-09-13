/**
 * @file AnalogInputTerminal.cpp
 * @brief Implementation of the generic analog-input terminal driver.
 *
 * See AnalogInputTerminal.hpp for the API documentation.  Compared to the
 * digital terminal family the bring-up differs in two ways:
 *
 *   1. These are mailbox devices — configureMailbox() sets up SM0/SM1
 *      and the CoE SDO channel from SII.  No SDO traffic is generated:
 *      the factory-default PDO assignment already covers all channels.
 *   2. Process data sits on the standard channels (SM3 inputs — and SM2
 *      outputs when the terminal enables one), so no SM bookkeeping
 *      mirroring is needed.  An enabled SM2 is registered + mapped as a
 *      scratch region to keep the cyclic exchange's working counter
 *      satisfied; its contents are not interpreted.
 */

#include "tether/Beckhoff/AnalogInputTerminal.hpp"

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

static const char* TAG = "AnalogInputTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

AnalogInputTerminal::AnalogInputTerminal(Master& master, uint16_t slave_index,
                                         const DeviceIdentity& identity)
    : master_(&master), slave_index_(slave_index), identity_(identity) {
}

AnalogInputTerminal::AnalogInputTerminal(Master& master,
                                         const DiscoveredSlave& slave,
                                         const DeviceIdentity& identity)
    : AnalogInputTerminal(master, slave.index, identity) {
    info_ = slave;
}

AnalogInputTerminal::~AnalogInputTerminal() {
    stop();
}

Result<AnalogInputTerminal> AnalogInputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<AnalogInputTerminal> AnalogInputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return AnalogInputTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

const char* AnalogInputTerminal::deviceName() const {
    if (info_ && info_->device_name) return info_->device_name->c_str();
    if (identity_.name) return identity_.name;
    return "AnalogInputTerminal";
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

#if TETHER_ENABLE_SII
namespace {

/// True for a channel-value PDO entry: any entry of at least 16 bit.
/// Value object indices vary by family/revision (0x6xxx, 0x6411, 0x300x)
/// and status bits are always sub-16-bit entries, so width alone is the
/// robust discriminator; the last such entry in a PDO is the channel value.
inline bool isAnalogValueEntry(const SII::SIIPDOEntry& e) {
    return e.bit_length >= 16;
}

} // anonymous namespace
#endif

bool AnalogInputTerminal::resolveLayout() {
#if TETHER_ENABLE_SII
    // --- Process-data sync managers -----------------------------------------
    // The SII SM category is positional: entry index == SM channel.
    if (info_ && info_->sync_managers) {
        for (size_t i = 0; i < info_->sync_managers->size() && i < 4; ++i) {
            const auto& sm = (*info_->sync_managers)[i];
            if (sm.sm_type == SII::SM_TYPE_PROCESS_IN && sm.isEnabled() &&
                sm.length > 0) {
                sm_in_channel_ = static_cast<uint8_t>(i);
                sm_in_addr_    = sm.phys_start_address;
                sm_in_len_     = sm.length;
                sm_in_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
            } else if (sm.sm_type == SII::SM_TYPE_PROCESS_OUT &&
                       sm.isEnabled() && sm.length > 0) {
                sm_out_enabled_ = true;
                sm_out_addr_    = sm.phys_start_address;
                sm_out_len_     = sm.length;
                sm_out_ctrl_    = std::bit_cast<uint8_t>(sm.control_register);
            }
        }
    }

    // --- Per-channel layout from the TxPDOs assigned to the input SM --------
    // The default assignment is exactly the set of TxPDOs whose SII record
    // names the input sync manager.  PDOs without a value entry (status-only
    // blocks like the EL3356's "RMB Status") act as a prefix of the next
    // channel; a PDO carrying the value closes the channel.
    if (info_ && info_->tx_pdos && sm_in_len_ > 0) {
        channels_.clear();
        uint32_t channel_start_bit = 0;
        uint32_t running_bit       = 0;
        bool     first             = true;

        for (const auto& pdo : *info_->tx_pdos) {
            if (pdo.sync_manager != sm_in_channel_) continue;
            if (first) { first_txpdo_ = pdo.pdo_index; first = false; }

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
                const uint32_t status_bits = value_entry_bit - channel_start_bit;
                ch.status_off = status_bits >= 16
                              ? static_cast<int16_t>(channel_start_bit / 8)
                              : -1;
                channels_.push_back(ch);
                channel_start_bit = running_bit + pdo.totalBits();
            }
            running_bit += pdo.totalBits();
        }

        // Sanity: the resolved image must fit the SM length.
        const uint32_t image_bytes = running_bit / 8;
        if (channels_.empty() || image_bytes > sm_in_len_) {
            TETHER_LOGE(TAG, "{}: TxPDO layout resolves to {}B over {} channel(s)"
                        " but SM{} holds {}B — unsupported terminal shape",
                        master_->slaveLogPrefix(slave_index_).c_str(),
                        image_bytes, channels_.size(), sm_in_channel_,
                        sm_in_len_);
            channels_.clear();
            return false;
        }
    }

    // RxPDO index bookkeeping (only relevant when SM2 is enabled).
    if (sm_out_enabled_ && info_ && info_->rx_pdos) {
        for (const auto& pdo : *info_->rx_pdos) {
            if (pdo.sync_manager == 2) { first_rxpdo_ = pdo.pdo_index; break; }
        }
    }
#endif
    // Fallback when SII resolution was impossible (build without SII, or an
    // ESI variant not yet in the registry): assume the dominant Beckhoff
    // shape — identity num_bits channels of 4B each (16-bit status + 16-bit
    // value) on SM3 @ 0x1180.
    if (sm_in_len_ == 0 && identity_.num_bits > 0 &&
        identity_.num_bits <= kMaxChannels) {
        sm_in_len_    = static_cast<uint16_t>(identity_.num_bits * 4);
        sm_in_addr_   = 0x1180;
        sm_in_ctrl_   = 0x20;
        sm_in_channel_ = 3;
        for (uint16_t ch = 0; ch < identity_.num_bits; ++ch) {
            channels_.push_back(
                ChannelLayout{static_cast<uint16_t>(ch * 4 + 2), 16,
                              static_cast<int16_t>(ch * 4)});
        }
    }
    return sm_in_len_ > 0 && !channels_.empty() &&
           channels_.size() <= kMaxChannels &&
           sm_in_len_ <= kMaxImageBytes &&
           sm_out_len_ <= kMaxImageBytes;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> AnalogInputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    // Fetch SII data (identity + SM + PDO + mailbox categories) if the
    // caller did not supply a deep-scan DiscoveredSlave.
    bool need_sii = true;
#if TETHER_ENABLE_SII
    need_sii = !(info_ && info_->sync_managers && info_->tx_pdos);
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
        TETHER_LOGE(TAG, "{}: no usable analog-input layout "
                    "(SM3 disabled, zero-length, or unrecognized PDO shape)",
                    master_->slaveLogPrefix(slave_index_).c_str());
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: input SM{} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} analog channel(s)){}",
                master_->slaveLogPrefix(slave_index_).c_str(), sm_in_channel_,
                sm_in_addr_, sm_in_len_, sm_in_ctrl_, channels_.size(),
                sm_out_enabled_ ? " +SM2 outputs" : "");

    // Register the process-data SMs under their actual channel indices so
    // the upcoming configureSlavesSMs() — invoked inside configureMailbox —
    // writes all four ESC register blocks in one pass.
    auto& pdo   = master_->pdoForSlave(slave_index_);
    auto* cfgs  = pdo.slaveConfigs();
    cfgs[slave_index_].sm[sm_in_channel_] =
        PDO::SyncManagerConfig::process_input(sm_in_addr_, sm_in_len_);
    cfgs[slave_index_].sm[sm_in_channel_].control =
        std::bit_cast<SyncManager::SMControlReg>(sm_in_ctrl_);
    if (sm_out_enabled_) {
        cfgs[slave_index_].sm[2] =
            PDO::SyncManagerConfig::process_output(sm_out_addr_, sm_out_len_);
        cfgs[slave_index_].sm[2].control =
            std::bit_cast<SyncManager::SMControlReg>(sm_out_ctrl_);
    }

    // Mailbox: these are CoE devices — configure SM0/SM1 + the SDO channel
    // from SII.  (If SII ever reported no mailbox on a family variant, the
    // no-mailbox declaration keeps the PRE-OP gate satisfied.)
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

    // Write the process-data SM registers unconditionally — configureMailbox
    // already wrote them on success, but a failed mailbox config or a
    // declared no-mailbox variant must still get SM2/SM3.
    pdo.configureSlavesSMs(slave_index_);

    if (sl.transitionToPreOp() != SlaveError::Ok) {
        return std::unexpected(Error::PreOpFailed);
    }

    // PDO buffer registration — one logical PDO covering the whole SM image
    // per direction; the channel layout indexes into it.
    in_buf_.assign(sm_in_len_, 0);
    int entry = pdo.mapping().add_txpdo(
        slave_index_, in_buf_.data(), sm_in_len_, first_txpdo_, mode);
    if (entry < 0) {
        return std::unexpected(Error::PdoRegistrationFailed);
    }
    if (sm_out_enabled_) {
        out_buf_.assign(sm_out_len_, 0);
        entry = pdo.mapping().add_rxpdo(
            slave_index_, out_buf_.data(), sm_out_len_, first_rxpdo_, mode);
        if (entry < 0) {
            return std::unexpected(Error::PdoRegistrationFailed);
        }
    }
    pdo.finalizeMapping(slave_index_);

    prepared_ = true;
    return {};
}

Result<> AnalogInputTerminal::enterSafeOp() {
    auto& sl = master_->slave(slave_index_);
    sl.assumePDOAlreadyConfigured();
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        return std::unexpected(Error::SafeOpFailed);
    }
    configured_ = true;
    return {};
}

Result<> AnalogInputTerminal::configure() {
    // Standalone: position addressing (APRD straight out of the SM buffer)
    // needs no FMMU and no logical map.
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return enterSafeOp();
}

Result<> AnalogInputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> AnalogInputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }

    // The chain must have built the logical map by now — look up the
    // addresses assigned to this slave in its covering LAM.
    auto& lam = master_->logicalAddressManagerForSlave(slave_index_);
    if (!lam.hasSlavePDOs(slave_index_)) {
        TETHER_LOGE(TAG, "{}: no logical address assigned — "
                    "was the shared address map built?",
                    master_->slaveLogPrefix(slave_index_).c_str());
        return std::unexpected(Error::LogicalMapMissing);
    }
    const uint32_t log_in = lam.getTxPDOLogicalAddr(slave_index_);

    auto& fmmu = master_->slave(slave_index_).fmmuManager();
    auto& cfg  = fmmu.config();
    cfg.clear();
    cfg.slave_index       = slave_index_;
    cfg.next_logical_addr = log_in;
    if (!cfg.addInput(sm_in_addr_, sm_in_len_, sm_in_channel_)) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    if (sm_out_enabled_) {
        const uint32_t log_out = lam.getRxPDOLogicalAddr(slave_index_);
        cfg.next_logical_addr = log_out;
        if (!cfg.addOutput(sm_out_addr_, sm_out_len_, 2)) {
            return std::unexpected(Error::FmmuConfigFailed);
        }
    }
    cfg.configured = true;
    if (!fmmu.writeToSlave()) {
        return std::unexpected(Error::FmmuConfigFailed);
    }
    logical_addr_ = log_in;
    TETHER_LOGI(TAG, "{}: FMMU maps logical 0x{:08X} <- phys 0x{:04X} len {}",
                master_->slaveLogPrefix(slave_index_).c_str(),
                static_cast<unsigned long>(log_in), sm_in_addr_, sm_in_len_);

    return enterSafeOp();
}

Result<> AnalogInputTerminal::requestOp(int timeout_ms) {
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

Result<> AnalogInputTerminal::start(const StartOptions& opts) {
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

void AnalogInputTerminal::stop() {
    if (loop_started_) {
        master_->stopMotionControlLoop();
        loop_started_ = false;
    }
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

int32_t AnalogInputTerminal::value(size_t channel) const {
    if (channel >= channels_.size()) return 0;
    const auto& ch = channels_[channel];
    if (ch.value_off + ch.value_bits / 8 > in_buf_.size()) return 0;
    const uint8_t* p = in_buf_.data() + ch.value_off;
    if (ch.value_bits > 16) {
        int32_t v;
        std::memcpy(&v, p, 4);
        return v;
    }
    int16_t v;
    std::memcpy(&v, p, 2);
    return v;
}

uint16_t AnalogInputTerminal::status(size_t channel) const {
    if (channel >= channels_.size()) return 0;
    const auto& ch = channels_[channel];
    if (ch.status_off < 0 ||
        static_cast<size_t>(ch.status_off) + 2 > in_buf_.size()) return 0;
    uint16_t s;
    std::memcpy(&s, in_buf_.data() + ch.status_off, 2);
    return s;
}

bool AnalogInputTerminal::hasStatus(size_t channel) const {
    if (channel >= channels_.size()) return false;
    return channels_[channel].status_off >= 0;
}

size_t AnalogInputTerminal::channelBits(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    return channels_[ch].value_bits;
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

SlaveState AnalogInputTerminal::alState() {
    uint8_t state = 0;
    if (!master_->readSlaveApplicationLayerState(SlaveAddress(slave_index_),
                                                state)) {
        return SlaveState::INIT;
    }
    return static_cast<SlaveState>(state & 0x0F);
}

bool AnalogInputTerminal::waitAlState(SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master_->isCancelRequested()) return false;
        if (alState() == target) return true;
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

} // namespace Beckhoff

} // namespace EtherCAT
