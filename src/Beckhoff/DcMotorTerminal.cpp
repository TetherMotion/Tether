/**
 * @file DcMotorTerminal.cpp
 * @brief Implementation of the EL73xx DC-motor (H-bridge) terminal driver.
 *
 * See DcMotorTerminal.hpp for the API documentation.  Axes are discovered
 * by scanning the SM2 PDO entries for 0x70xx objects carrying a "Velocity"
 * value entry (subindex 0x21); encoder objects pair to axes in index
 * order.  All field offsets are resolved by (object index, subindex).
 */

#include "tether/Beckhoff/DcMotorTerminal.hpp"

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

static const char* TAG = "DcMotorTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

DcMotorTerminal::DcMotorTerminal(Master& master, uint16_t slave_index,
                                 const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

DcMotorTerminal::DcMotorTerminal(Master& master,
                                 const DiscoveredSlave& slave,
                                 const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

DcMotorTerminal::~DcMotorTerminal() = default;

Result<DcMotorTerminal> DcMotorTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<DcMotorTerminal> DcMotorTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return DcMotorTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool DcMotorTerminal::resolveLayout() {
    resolveSyncManagers();
    axes_.clear();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled && sm_out_.length > 0) {
        const auto& rx = *info_->rx_pdos;
        const uint8_t ch = sm_out_.channel;

        // Collect every 0x70xx object index present in the SM2 image.
        std::vector<uint16_t> objs;
        for (const auto& pdo : rx) {
            if (pdo.sync_manager != ch) continue;
            for (const auto& e : pdo.entries) {
                if (e.index >= 0x7000 && e.index <= 0x70FF &&
                    std::ranges::find(objs, e.index) == objs.end()) {
                    objs.push_back(e.index);
                }
            }
        }
        std::ranges::sort(objs);

        // Classify: a MOT object carries a >=16-bit Velocity entry at
        // subindex 0x21; an ENC object carries "Set counter" (bit :03)
        // or "Set counter value" (:11, >=16 bit) without a velocity.
        std::vector<uint16_t> mot_objs, enc_objs;
        for (const uint16_t o : objs) {
            uint32_t off; uint8_t len;
            if (detail::findValueEntry(rx, ch, o, 0x21, 0, 16, off, len)) {
                mot_objs.push_back(o);
            } else {
                uint32_t b;
                if (detail::findEntryBitOffset(rx, ch, o, 3, 0, b) ||
                    detail::findValueEntry(rx, ch, o, 0x11, 0, 16, b, len)) {
                    enc_objs.push_back(o);
                }
            }
        }

        axes_.resize(mot_objs.size());
        for (size_t i = 0; i < mot_objs.size(); ++i) {
            Axis& a = axes_[i];
            a.mot_obj = mot_objs[i];
            if (i < enc_objs.size()) a.enc_obj = enc_objs[i];

            auto bit = [&](uint16_t idx, int sub, uint32_t& out) {
                out = kInvalid;
                detail::findEntryBitOffset(rx, ch, idx, sub, 0, out);
            };
            bit(a.mot_obj, 1, a.off_enable);
            bit(a.mot_obj, 2, a.off_reset);
            bit(a.mot_obj, 3, a.off_rtq);
            detail::findValueEntry(rx, ch, a.mot_obj, 0x21, 0, 16,
                                   a.velocity.bit_off, a.velocity.bit_len);

            if (a.enc_obj) {
                bit(a.enc_obj, 2, a.off_latch_pos);
                bit(a.enc_obj, 3, a.off_set_counter);
                bit(a.enc_obj, 4, a.off_latch_neg);
                detail::findValueEntry(rx, ch, a.enc_obj, 0x11, 0, 16,
                                       a.set_counter_val.bit_off,
                                       a.set_counter_val.bit_len);
            }
            a.stat_obj = static_cast<uint16_t>(a.mot_obj - 0x1000);
            a.enc_stat_obj =
                static_cast<uint16_t>(a.enc_obj ? a.enc_obj - 0x1000 : 0);
        }

        if (axes_.empty()) {
            TETHER_LOGE(TAG, "{}: no MOT velocity objects in the SM2 "
                        "image — not a DC-motor terminal",
                        logPrefix().c_str());
            return false;
        }
    }

    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        const auto& tx = *info_->tx_pdos;
        const uint8_t ch = sm_in_.channel;
        for (auto& a : axes_) {
            detail::findEntryBitOffset(tx, ch, a.stat_obj, -1, 0,
                                       a.off_status);
            detail::findEntryBitOffset(tx, ch, 0x1C32, 32, 0,
                                       a.off_sync_err);
            if (a.enc_stat_obj) {
                detail::findEntryBitOffset(tx, ch, a.enc_stat_obj, -1, 0,
                                           a.off_enc_status);
                detail::findValueEntry(tx, ch, a.enc_stat_obj, 0x11, 0, 16,
                                       a.counter.bit_off, a.counter.bit_len);
                detail::findValueEntry(tx, ch, a.enc_stat_obj, 0x12, 0, 16,
                                       a.latch.bit_off, a.latch.bit_len);
            }
        }
    }
#endif

    // Fallback without SII: the fixed EL7332 2-axis shape —
    //   out: per axis [ctrl_lo, ctrl_hi, vel_lo, vel_hi]
    //   in : per axis [stat_lo, stat_hi, sync/toggle byte]
    if (!sm_out_.enabled) {
        const uint8_t n =
            identity_.num_bits ? identity_.num_bits : 2;
        sm_out_ = {2, 0x1100,
                   static_cast<uint16_t>(4 * n), 0x24, true, 0x1600};
        axes_.resize(n);
        for (uint8_t i = 0; i < n; ++i) {
            Axis& a    = axes_[i];
            a.mot_obj  = static_cast<uint16_t>(0x7000 + 0x10 * i);
            a.off_enable    = uint32_t(i) * 32 + 0;
            a.off_reset     = uint32_t(i) * 32 + 1;
            a.off_rtq       = uint32_t(i) * 32 + 2;
            a.velocity      = {uint32_t(i) * 32 + 16, 16};
            a.stat_obj      = static_cast<uint16_t>(0x6000 + 0x10 * i);
            a.enc_obj       = 0;
            a.enc_stat_obj  = 0;
        }
    }
    if (!sm_in_.enabled) {
        const uint16_t n = static_cast<uint16_t>(axes_.size());
        sm_in_ = {3, 0x1180,
                  static_cast<uint16_t>(3 * n), 0x20, true, 0x1A00};
        for (uint16_t i = 0; i < n; ++i) {
            axes_[i].off_status   = i * 24;
            axes_[i].off_sync_err = i * 24 + 16;
        }
    }
    return sm_in_.enabled && sm_out_.enabled && !axes_.empty() &&
           axes_.size() <= kMaxAxes &&
           sm_in_.length <= 512 && sm_out_.length <= 512;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> DcMotorTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: MOT interface {} axes, out SM{} {}B / in SM{} {}B",
                logPrefix().c_str(), axes_.size(),
                sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> DcMotorTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> DcMotorTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> DcMotorTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> DcMotorTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> DcMotorTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Bit/field access helpers
// ---------------------------------------------------------------------------

bool DcMotorTerminal::inBit(uint32_t bit_off) const {
    return bit_off != kInvalid && detail::imageBit(in_buf_, bit_off);
}

void DcMotorTerminal::setOutBit(uint32_t bit_off, bool v) {
    if (bit_off != kInvalid) detail::setImageBit(out_buf_, bit_off, v);
}

int64_t DcMotorTerminal::inSigned64(uint32_t bit_off, uint8_t bits) const {
    if (bit_off == kInvalid || bits == 0) return 0;
    const size_t byte = bit_off / 8;
    if (byte + (bits + 7) / 8 > in_buf_.size()) return 0;
    return detail::signExtendLE(in_buf_.data() + byte, bits);
}

int32_t DcMotorTerminal::inSigned(uint32_t bit_off, uint8_t bits) const {
    return static_cast<int32_t>(inSigned64(bit_off, bits));
}

void DcMotorTerminal::outSigned(uint32_t bit_off, uint8_t bits, int32_t v) {
    if (bit_off == kInvalid || bits == 0) return;
    const size_t byte = bit_off / 8;
    if (byte + (bits + 7) / 8 > out_buf_.size()) return;
    if (bits > 16) {
        std::memcpy(out_buf_.data() + byte, &v, 4);
    } else {
        const int16_t s = static_cast<int16_t>(v);
        std::memcpy(out_buf_.data() + byte, &s, 2);
    }
}

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------

void DcMotorTerminal::setEnable(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_enable, on);
}
void DcMotorTerminal::setReset(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_reset, on);
}
void DcMotorTerminal::setReduceTorque(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_rtq, on);
}
void DcMotorTerminal::setVelocity(size_t axis, int16_t v) {
    if (auto* a = axisAt(axis)) {
        outSigned(a->velocity.bit_off, a->velocity.bit_len, v);
    }
}

uint16_t DcMotorTerminal::motControlWord(size_t axis) const {
    const auto* a = axisAt(axis);
    if (!a || a->off_enable == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = a->off_enable / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}

// ---------------------------------------------------------------------------
// Motor status — MOT status bit ordinals inside the packed word starting
// at the first status entry (subs 1..7 map to ordinals 0..6, subs 12/13
// to ordinals 11/12 — identical to the POS STM status layout).
// ---------------------------------------------------------------------------

bool DcMotorTerminal::readyToEnable(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 0);
}
bool DcMotorTerminal::ready(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 1);
}
bool DcMotorTerminal::warning(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 2);
}
bool DcMotorTerminal::error(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 3);
}
bool DcMotorTerminal::movingPositive(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 4);
}
bool DcMotorTerminal::movingNegative(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 5);
}
bool DcMotorTerminal::torqueReduced(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 6);
}
bool DcMotorTerminal::digitalInput1(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 11);
}
bool DcMotorTerminal::digitalInput2(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_status + 12);
}
bool DcMotorTerminal::syncError(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && inBit(a->off_sync_err);
}

uint16_t DcMotorTerminal::motStatusWord(size_t axis) const {
    const auto* a = axisAt(axis);
    if (!a || a->off_status == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = a->off_status / 8;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

// ---------------------------------------------------------------------------
// Optional encoder channel
// ---------------------------------------------------------------------------

bool DcMotorTerminal::hasEncoder(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && a->enc_stat_obj != 0;
}

int32_t DcMotorTerminal::counterValue(size_t axis) const {
    const auto* a = axisAt(axis);
    return a ? inSigned(a->counter.bit_off, a->counter.bit_len) : 0;
}
int32_t DcMotorTerminal::latchValue(size_t axis) const {
    const auto* a = axisAt(axis);
    return a ? inSigned(a->latch.bit_off, a->latch.bit_len) : 0;
}

// ENC status ordinals: :01 latch-C-valid(unused), :02 latch valid,
// :03 set-counter done, :04 underflow, :05 overflow.
bool DcMotorTerminal::latchValid(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && a->off_enc_status != kInvalid &&
           inBit(a->off_enc_status + 1);
}
bool DcMotorTerminal::setCounterDone(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && a->off_enc_status != kInvalid &&
           inBit(a->off_enc_status + 2);
}
bool DcMotorTerminal::counterUnderflow(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && a->off_enc_status != kInvalid &&
           inBit(a->off_enc_status + 3);
}
bool DcMotorTerminal::counterOverflow(size_t axis) const {
    const auto* a = axisAt(axis);
    return a && a->off_enc_status != kInvalid &&
           inBit(a->off_enc_status + 4);
}

void DcMotorTerminal::setCounterValue(size_t axis, int32_t v) {
    if (auto* a = axisAt(axis)) {
        outSigned(a->set_counter_val.bit_off, a->set_counter_val.bit_len, v);
    }
}
void DcMotorTerminal::setSetCounter(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_set_counter, on);
}
void DcMotorTerminal::setLatchExtPos(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_latch_pos, on);
}
void DcMotorTerminal::setLatchExtNeg(size_t axis, bool on) {
    if (auto* a = axisAt(axis)) setOutBit(a->off_latch_neg, on);
}

uint16_t DcMotorTerminal::encStatusWord(size_t axis) const {
    const auto* a = axisAt(axis);
    if (!a || a->off_enc_status == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = a->off_enc_status / 8;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

} // namespace Beckhoff

} // namespace EtherCAT
