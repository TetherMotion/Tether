/**
 * @file PwmTerminal.cpp
 * @brief Implementation of the EL25xx PWM-terminal driver.
 *
 * Channel fields are resolved from the SII PDO list by (object index,
 * subindex): duty values on 0x7000+0x10*n:17, control bits and status
 * on the same channel objects.
 */

#include "tether/Beckhoff/PwmTerminal.hpp"

#include <algorithm>
#include <cstring>

#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "PwmTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

PwmTerminal::PwmTerminal(Master& master, uint16_t slave_index,
                         const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

PwmTerminal::PwmTerminal(Master& master, const DiscoveredSlave& slave,
                         const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

PwmTerminal::~PwmTerminal() = default;

Result<PwmTerminal> PwmTerminal::findFirst(Master& master,
                                           const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<PwmTerminal> PwmTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return PwmTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool PwmTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        const auto& rx = *info_->rx_pdos;
        const uint8_t oc = sm_out_.channel;
        const uint8_t ic = sm_in_.enabled ? sm_in_.channel : 0xFF;
        const bool have_tx = info_ && info_->tx_pdos && sm_in_.enabled;
        const auto tx = have_tx
            ? std::span<const SII::SIIPDO>(*info_->tx_pdos)
            : std::span<const SII::SIIPDO>{};

        auto outField = [&](uint16_t idx, int sub) {
            Field f;
            if (!detail::findValueEntry(rx, oc, idx, sub, 0, 8,
                                        f.bit_off, f.bit_len)) {
                f.bit_off = kInvalid; f.bit_len = 0;
            }
            return f;
        };
        auto outBit = [&](uint16_t idx, int sub) {
            uint32_t off = kInvalid;
            detail::findEntryBitOffset(rx, oc, idx, sub, 0, off);
            return off;
        };
        auto inBitOf = [&](uint16_t idx, int sub) {
            uint32_t off = kInvalid;
            if (have_tx) {
                detail::findEntryBitOffset(tx, ic, idx, sub, 0, off);
            }
            return off;
        };

        for (size_t c = 0; c < kMaxChannels; ++c) {
            const uint16_t obj = static_cast<uint16_t>(0x7000 + 0x10 * c);
            Field duty = outField(obj, 0x11);
            if (!duty.present()) break;
            Channel ch{};
            ch.duty    = duty;
            ch.enable  = outBit(obj, 0x06);
            ch.reset   = outBit(obj, 0x07);
            ch.dither  = outBit(obj, 0x01);
            const uint16_t sobj = static_cast<uint16_t>(0x6000 + 0x10 * c);
            ch.warning = inBitOf(sobj, 0x06);
            ch.err     = inBitOf(sobj, 0x07);
            ch.din     = inBitOf(sobj, 0x01);
            channels_.push_back(ch);
        }
        master_gain_ = outField(0xF719, 0x11);

        if (channels_.empty()) {
            TETHER_LOGE(TAG, "{}: no PWM output fields in the SM2 image",
                        logPrefix().c_str());
            return false;
        }
    }
#endif

    // Fallback without SII: num_bits carries the channel count —
    // N channels of 2-byte duty values, no control bits, no inputs.
    if (!sm_out_.enabled) {
        const uint16_t ch = identity_.num_bits ? identity_.num_bits : 2;
        sm_out_ = {2, 0x1100, static_cast<uint16_t>(2 * ch), 0x24, true,
                   0x1600};
        for (uint16_t c = 0; c < ch; ++c) {
            Channel chn{};
            chn.duty = {static_cast<uint32_t>(c * 16), 16};
            channels_.push_back(chn);
        }
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1180, 0, 0x20, false, 0};
    }
    return sm_out_.enabled && sm_out_.length <= 512 && !channels_.empty();
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> PwmTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: PWM out SM{} {}B / in SM{} {}B ({} ch)",
                logPrefix().c_str(), sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length, channels_.size());

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> PwmTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> PwmTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> PwmTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> PwmTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> PwmTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Bit access
// ---------------------------------------------------------------------------

bool PwmTerminal::inBit(uint32_t bit_off) const {
    return bit_off != kInvalid && detail::imageBit(in_buf_, bit_off);
}

void PwmTerminal::setOutBit(uint32_t bit_off, bool v) {
    if (bit_off != kInvalid) detail::setImageBit(out_buf_, bit_off, v);
}

// ---------------------------------------------------------------------------
// Duty cycle
// ---------------------------------------------------------------------------

void PwmTerminal::setDuty(size_t ch, uint16_t raw) {
    if (ch >= channels_.size()) return;
    const Field& f = channels_[ch].duty;
    if (!f.present()) return;
    const uint16_t v = std::min<uint16_t>(raw, kDutyFullScale);
    const size_t byte = f.bit_off / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(out_buf_.data() + byte, &v, 2);
    }
}

void PwmTerminal::setDutyFraction(size_t ch, float fraction) {
    const float clamped = std::clamp(fraction, 0.0f, 1.0f);
    setDuty(ch, static_cast<uint16_t>(clamped * kDutyFullScale + 0.5f));
}

uint16_t PwmTerminal::duty(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    const Field& f = channels_[ch].duty;
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    uint16_t v = 0;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&v, out_buf_.data() + byte, 2);
    }
    return v;
}

void PwmTerminal::allOff() {
    for (size_t c = 0; c < channels_.size(); ++c) setDuty(c, 0);
}

// ---------------------------------------------------------------------------
// Control bits
// ---------------------------------------------------------------------------

void PwmTerminal::setEnable(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].enable, on);
}
void PwmTerminal::setReset(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].reset, on);
}
void PwmTerminal::setEnableDithering(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].dither, on);
}
bool PwmTerminal::hasEnable(size_t ch) const {
    return ch < channels_.size() && channels_[ch].enable != kInvalid;
}
bool PwmTerminal::hasDithering(size_t ch) const {
    return ch < channels_.size() && channels_[ch].dither != kInvalid;
}

void PwmTerminal::setMasterGain(uint16_t gain) {
    if (!master_gain_.present()) return;
    const uint16_t v = std::min<uint16_t>(gain, kDutyFullScale);
    const size_t byte = master_gain_.bit_off / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(out_buf_.data() + byte, &v, 2);
    }
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

bool PwmTerminal::warning(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].warning);
}
bool PwmTerminal::error(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].err);
}
bool PwmTerminal::digitalInput(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].din);
}
bool PwmTerminal::hasStatus(size_t ch) const {
    return ch < channels_.size() &&
           (channels_[ch].warning != kInvalid || channels_[ch].err != kInvalid);
}

} // namespace Beckhoff

} // namespace EtherCAT
