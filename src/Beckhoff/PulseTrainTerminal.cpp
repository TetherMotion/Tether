/**
 * @file PulseTrainTerminal.cpp
 * @brief Implementation of the EL2521/EL2522 pulse-train driver.
 *
 * See PulseTrainTerminal.hpp for the API documentation.  All fields are
 * resolved from the SII PDO entries by (object index, subindex).
 * Per-channel ENC objects are found by shape (an object with a 1-bit
 * entry at :03 "set counter" and a value at :17, distinct from the PTO
 * object) — the index varies between EL2521-0124 (0x7010) and EL2522
 * (0x7020+0x10*ch).
 */

#include "tether/Beckhoff/PulseTrainTerminal.hpp"

#include <algorithm>
#include <cstring>

#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "PulseTrainTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

PulseTrainTerminal::PulseTrainTerminal(Master& master, uint16_t slave_index,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

PulseTrainTerminal::PulseTrainTerminal(Master& master,
                                       const DiscoveredSlave& slave,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

PulseTrainTerminal::~PulseTrainTerminal() = default;

Result<PulseTrainTerminal> PulseTrainTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<PulseTrainTerminal> PulseTrainTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) {
            return PulseTrainTerminal(master, s, identity);
        }
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool PulseTrainTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        const auto& rx = *info_->rx_pdos;
        const uint8_t oc = sm_out_.channel;
        const bool have_tx = info_ && info_->tx_pdos && sm_in_.enabled;
        const uint8_t ic = sm_in_.channel;
        const auto tx = have_tx
            ? std::span<const SII::SIIPDO>(*info_->tx_pdos)
            : std::span<const SII::SIIPDO>{};

        auto fld = [](std::span<const SII::SIIPDO> pdos, uint8_t ch,
                      uint16_t idx, int sub, uint8_t minb = 1) {
            Field f;
            if (!detail::findValueEntry(pdos, ch, idx, sub, 0, minb,
                                        f.bit_off, f.bit_len)) {
                f.bit_off = kInvalid; f.bit_len = 0;
            }
            return f;
        };
        auto bit1 = [&](std::span<const SII::SIIPDO> pdos, uint8_t ch,
                        uint16_t idx, int sub) {
            // Only matches genuine 1-bit entries (16-bit words sharing
            // the subindex are rejected via bit_len).
            Field f = fld(pdos, ch, idx, sub, 1);
            if (f.present() && f.bit_len == 1) return f.bit_off;
            return kInvalid;
        };

        // Objects already claimed by an earlier channel — the ENC
        // candidates overlap the PTO objects of other channels
        // (EL2522 ch2's PTO is 0x7010, inside the scan range).
        std::vector<uint16_t> claimed_out, claimed_in;

        // Returns the object index of the ENC control object for this
        // channel: a 0x70xx object (not the PTO object) with a 1-bit
        // entry at :03 (set counter) and a value at :17 — and no :01
        // (PTO :01 = frequency select, PLS :01 = enable).
        auto findEncCtrlObj = [&](uint16_t pto_obj) -> uint16_t {
            for (uint16_t x = 0x7010; x < 0x7080; x += 0x10) {
                if (x == pto_obj) continue;
                if (std::ranges::find(claimed_out, x) != claimed_out.end()) {
                    continue;
                }
                if (bit1(rx, oc, x, 3) != kInvalid &&
                    bit1(rx, oc, x, 1) == kInvalid &&
                    fld(rx, oc, x, 0x11, 8).present()) {
                    claimed_out.push_back(x);
                    return x;
                }
            }
            return 0;
        };
        // ENC status object: a 0x60xx object (not the PTO status) with a
        // value entry at :17 (counter value).
        auto findEncStatObj = [&](uint16_t pto_stat) -> uint16_t {
            if (!have_tx) return 0;
            for (uint16_t x = 0x6010; x < 0x6080; x += 0x10) {
                if (x == pto_stat) continue;
                if (std::ranges::find(claimed_in, x) != claimed_in.end()) {
                    continue;
                }
                if (fld(tx, ic, x, 0x11, 8).present()) {
                    claimed_in.push_back(x);
                    return x;
                }
            }
            return 0;
        };

        for (size_t c = 0; c < kMaxChannels; ++c) {
            const uint16_t pto = static_cast<uint16_t>(0x7000 + 0x10 * c);
            const uint16_t pst = static_cast<uint16_t>(0x6000 + 0x10 * c);
            Channel ch{};

            ch.freq_sel   = bit1(rx, oc, pto, 0x01);
            ch.dis_ramp   = bit1(rx, oc, pto, 0x02);
            ch.go_counter = bit1(rx, oc, pto, 0x03);
            ch.auto_dir   = bit1(rx, oc, pto, 0x04);
            ch.fwd        = bit1(rx, oc, pto, 0x05);
            ch.rev        = bit1(rx, oc, pto, 0x06);
            ch.freq       = fld(rx, oc, pto, 0x11, 8);
            ch.target     = fld(rx, oc, pto, 0x12, 8);
            ch.raw_ctrl   = fld(rx, oc, pto, 0x01, 8);
            ch.raw_data   = fld(rx, oc, pto, 0x02, 8);

            const uint16_t enc = findEncCtrlObj(pto);
            if (enc) {
                ch.enc_set_cnt = bit1(rx, oc, enc, 0x03);
                ch.enc_set_val = fld(rx, oc, enc, 0x11, 8);
            }

            if (have_tx) {
                ch.sel_ack     = bit1(tx, ic, pst, 0x01);
                ch.ramp_active = bit1(tx, ic, pst, 0x02);
                ch.err         = bit1(tx, ic, pst, 0x07);
                ch.sync_err    = bit1(tx, ic, pst, 0x14);

                const uint16_t est = findEncStatObj(pst);
                if (est) {
                    ch.set_done  = bit1(tx, ic, est, 0x03);
                    ch.underflow = bit1(tx, ic, est, 0x04);
                    ch.overflow  = bit1(tx, ic, est, 0x05);
                    ch.counter   = fld(tx, ic, est, 0x11, 8);
                    ch.latch     = fld(tx, ic, est, 0x12, 8);
                }
            }

            // A channel exists when any PTO or legacy field resolved.
            const bool found = ch.freq.present() || ch.raw_ctrl.present() ||
                               ch.enc_set_val.present() ||
                               ch.sel_ack != kInvalid;
            if (!found) break;
            channels_.push_back(ch);
        }

        if (channels_.empty()) {
            TETHER_LOGE(TAG, "{}: no PTO/ENC fields in the process image",
                        logPrefix().c_str());
            return false;
        }
    }
#endif

    // Fallback without SII: legacy EL2521 shape — 4B out (Ctrl, Data),
    // 4B in (Status, Data In).
    if (!sm_out_.enabled) {
        const uint16_t ch = identity_.num_bits ? identity_.num_bits : 1;
        sm_out_ = {2, 0x1100, static_cast<uint16_t>(4 * ch), 0x24, true,
                   0x1600};
        sm_in_  = {3, 0x1180, static_cast<uint16_t>(4 * ch), 0x20, true,
                   0x1A00};
        for (uint16_t c = 0; c < ch; ++c) {
            Channel chn{};
            chn.raw_ctrl = {static_cast<uint32_t>(c * 32), 16};
            chn.raw_data = {static_cast<uint32_t>(c * 32 + 16), 16};
            channels_.push_back(chn);
        }
    }
    return sm_out_.enabled && sm_out_.length <= 512 && !channels_.empty();
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> PulseTrainTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: PTO out SM{} {}B / in SM{} {}B ({} ch)",
                logPrefix().c_str(), sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length, channels_.size());

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> PulseTrainTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> PulseTrainTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> PulseTrainTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> PulseTrainTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> PulseTrainTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Access helpers
// ---------------------------------------------------------------------------

bool PulseTrainTerminal::inBit(uint32_t bit_off) const {
    if (bit_off == kInvalid) return false;
    const size_t byte = bit_off / 8;
    if (byte >= in_buf_.size()) return false;
    return (in_buf_[byte] >> (bit_off % 8)) & 1;
}

void PulseTrainTerminal::setOutBit(uint32_t bit_off, bool v) {
    if (bit_off == kInvalid) return;
    const size_t byte = bit_off / 8;
    if (byte >= out_buf_.size()) return;
    if (v) out_buf_[byte] |=  static_cast<uint8_t>(1u << (bit_off % 8));
    else   out_buf_[byte] &= ~static_cast<uint8_t>(1u << (bit_off % 8));
}

int64_t PulseTrainTerminal::inSigned(const Field& f) const {
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    if (byte + (f.bit_len + 7) / 8 > in_buf_.size()) return 0;
    uint64_t raw = 0;
    std::memcpy(&raw, in_buf_.data() + byte, (f.bit_len + 7) / 8);
    if (f.bit_len < 64) raw &= (uint64_t{1} << f.bit_len) - 1;
    const uint64_t sign = uint64_t{1} << (f.bit_len - 1);
    return static_cast<int64_t>((raw ^ sign) - sign);
}

void PulseTrainTerminal::outSigned(const Field& f, int32_t v) {
    if (!f.present()) return;
    const size_t byte = f.bit_off / 8;
    if (byte + (f.bit_len + 7) / 8 > out_buf_.size()) return;
    if (f.bit_len > 16) {
        std::memcpy(out_buf_.data() + byte, &v, 4);
    } else {
        const int16_t s = static_cast<int16_t>(v);
        std::memcpy(out_buf_.data() + byte, &s, 2);
    }
}

uint16_t PulseTrainTerminal::inWord(const Field& f) const {
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    uint16_t w = 0;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

void PulseTrainTerminal::outWord(const Field& f, uint16_t v) {
    if (!f.present()) return;
    const size_t byte = f.bit_off / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(out_buf_.data() + byte, &v, 2);
    }
}

// ---------------------------------------------------------------------------
// PTO control
// ---------------------------------------------------------------------------

void PulseTrainTerminal::setFrequency(size_t ch, uint16_t freq) {
    if (ch < channels_.size()) outWord(channels_[ch].freq, freq);
}
uint16_t PulseTrainTerminal::frequency(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    const Field& f = channels_[ch].freq;
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    uint16_t v = 0;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&v, out_buf_.data() + byte, 2);
    }
    return v;
}
bool PulseTrainTerminal::hasFrequency(size_t ch) const {
    return ch < channels_.size() && channels_[ch].freq.present();
}

void PulseTrainTerminal::setFrequencySelect(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].freq_sel, on);
}
void PulseTrainTerminal::setDisableRamp(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].dis_ramp, on);
}
void PulseTrainTerminal::setGoCounter(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].go_counter, on);
}
void PulseTrainTerminal::setAutoDirection(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].auto_dir, on);
}
void PulseTrainTerminal::setForward(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].fwd, on);
}
void PulseTrainTerminal::setReverse(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].rev, on);
}

void PulseTrainTerminal::setTarget(size_t ch, int32_t target) {
    if (ch < channels_.size()) outSigned(channels_[ch].target, target);
}
bool PulseTrainTerminal::hasTarget(size_t ch) const {
    return ch < channels_.size() && channels_[ch].target.present();
}

uint16_t PulseTrainTerminal::rawCtrl(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    const Field& f = channels_[ch].raw_ctrl;
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    uint16_t w = 0;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}
void PulseTrainTerminal::setRawCtrl(size_t ch, uint16_t w) {
    if (ch < channels_.size()) outWord(channels_[ch].raw_ctrl, w);
}
uint16_t PulseTrainTerminal::rawData(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    const Field& f = channels_[ch].raw_data;
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    uint16_t w = 0;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}
void PulseTrainTerminal::setRawData(size_t ch, uint16_t w) {
    if (ch < channels_.size()) outWord(channels_[ch].raw_data, w);
}
bool PulseTrainTerminal::hasRawInterface(size_t ch) const {
    return ch < channels_.size() && channels_[ch].raw_ctrl.present();
}

// ---------------------------------------------------------------------------
// ENC control
// ---------------------------------------------------------------------------

void PulseTrainTerminal::setEncSetCounter(size_t ch, bool on) {
    if (ch < channels_.size()) setOutBit(channels_[ch].enc_set_cnt, on);
}
void PulseTrainTerminal::setEncCounterValue(size_t ch, int32_t v) {
    if (ch < channels_.size()) outSigned(channels_[ch].enc_set_val, v);
}
bool PulseTrainTerminal::hasEnc(size_t ch) const {
    return ch < channels_.size() &&
           (channels_[ch].enc_set_cnt != kInvalid ||
            channels_[ch].counter.present());
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

bool PulseTrainTerminal::selAck(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].sel_ack);
}
bool PulseTrainTerminal::rampActive(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].ramp_active);
}
bool PulseTrainTerminal::error(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].err);
}
bool PulseTrainTerminal::syncError(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].sync_err);
}
bool PulseTrainTerminal::setCounterDone(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].set_done);
}
bool PulseTrainTerminal::counterUnderflow(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].underflow);
}
bool PulseTrainTerminal::counterOverflow(size_t ch) const {
    return ch < channels_.size() && inBit(channels_[ch].overflow);
}
int32_t PulseTrainTerminal::counterValue(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    return static_cast<int32_t>(inSigned(channels_[ch].counter));
}
int32_t PulseTrainTerminal::latchValue(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    return static_cast<int32_t>(inSigned(channels_[ch].latch));
}

} // namespace Beckhoff

} // namespace EtherCAT
