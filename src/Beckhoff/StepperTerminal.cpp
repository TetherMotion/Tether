/**
 * @file StepperTerminal.cpp
 * @brief Implementation of the EL703x/EL704x stepper-terminal driver.
 *
 * See StepperTerminal.hpp for the API documentation.  All field offsets
 * are resolved from the SII PDO entries by (object index, subindex) —
 * the compact (16-bit) and extended (32-bit) PDO variants and the
 * EL7031-without-latch-C vs. EL7041 layout differences are handled by
 * the resolution, not by hard-coded offsets.
 */

#include "tether/Beckhoff/StepperTerminal.hpp"

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

static const char* TAG = "StepperTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

StepperTerminal::StepperTerminal(Master& master, uint16_t slave_index,
                                 const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

StepperTerminal::StepperTerminal(Master& master,
                                 const DiscoveredSlave& slave,
                                 const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

StepperTerminal::~StepperTerminal() = default;

Result<StepperTerminal> StepperTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<StepperTerminal> StepperTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return StepperTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution — every field located by (object index, subindex)
// ---------------------------------------------------------------------------

bool StepperTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled && sm_out_.length > 0) {
        const auto& rx = *info_->rx_pdos;
        const uint8_t ch = sm_out_.channel;
        auto bit = [&](uint16_t idx, int sub, uint32_t& out) {
            return detail::findEntryBitOffset(rx, ch, idx, sub, 0, out);
        };
        auto val = [&](uint16_t idx, int sub, Field& f, uint8_t minb = 16) {
            f.bit_off = kInvalid;
            f.bit_len = 0;
            return detail::findValueEntry(rx, ch, idx, sub, 0, minb,
                                          f.bit_off, f.bit_len);
        };

        bit(0x7000, -1, off_enc_ctrl_);
        bit(0x7000, 1, off_enc_latch_c_);    // absent on EL703x
        bit(0x7000, 2, off_latch_pos_);
        bit(0x7000, 3, off_set_counter_);
        bit(0x7000, 4, off_latch_neg_);
        val(0x7000, 0x11, fld_set_counter_val_);
        bit(0x7010, 1, off_stm_enable_);
        bit(0x7010, 2, off_stm_reset_);
        bit(0x7010, 3, off_stm_rtq_);
        val(0x7010, 0x21, fld_velocity_);
        val(0x7010, 0x11, fld_pos_target_, 32);   // only when PDO 0x1603 mapped

        if (off_stm_enable_ == kInvalid || fld_velocity_.bit_len == 0) {
            TETHER_LOGE(TAG, "{}: no STM control/velocity field in the "
                        "SM2 image — not a POS-interface stepper",
                        logPrefix().c_str());
            return false;
        }
    }

    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        const auto& tx = *info_->tx_pdos;
        const uint8_t ch = sm_in_.channel;
        auto bit = [&](uint16_t idx, int sub, uint32_t& out) {
            return detail::findEntryBitOffset(tx, ch, idx, sub, 0, out);
        };
        auto val = [&](uint16_t idx, int sub, Field& f) {
            f.bit_off = kInvalid;
            f.bit_len = 0;
            return detail::findValueEntry(tx, ch, idx, sub, 0, 16,
                                          f.bit_off, f.bit_len);
        };

        bit(0x6000, -1, off_enc_status_);
        bit(0x6000, 2, off_latch_valid_);
        bit(0x6000, 3, off_set_done_);
        bit(0x6000, 4, off_underflow_);
        bit(0x6000, 5, off_overflow_);
        val(0x6000, 0x11, fld_counter_);
        val(0x6000, 0x12, fld_latch_);
        bit(0x6010, -1, off_stm_status_);
        bit(0x1C32, 32, off_sync_err_);

        if (off_stm_status_ == kInvalid) {
            TETHER_LOGE(TAG, "{}: no STM status field in the SM3 image",
                        logPrefix().c_str());
            return false;
        }
    }
#endif

    // Fallback without SII: the fixed 8B/8B default shape.
    //   out: [enc_ctrl_lo, enc_ctrl_hi, setval_lo, setval_hi,
    //         stm_ctrl_lo, stm_ctrl_hi, vel_lo, vel_hi]
    //   in : [enc_stat_lo, enc_stat_hi, cnt_lo, cnt_hi,
    //         latch_lo, latch_hi, stm_stat_lo, stm_stat_hi]
    if (!sm_out_.enabled) {
        sm_out_ = {2, 0x1100, 8, 0x24, true, 0x1600};
        off_enc_ctrl_    = 0;
        off_latch_pos_   = 1;
        off_set_counter_ = 2;
        off_latch_neg_   = 3;
        off_enc_latch_c_ = kInvalid;
        fld_set_counter_val_ = {16, 16};
        off_stm_enable_  = 32;
        off_stm_reset_   = 33;
        off_stm_rtq_     = 34;
        fld_velocity_    = {48, 16};
        fld_pos_target_  = {};
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1180, 8, 0x20, true, 0x1A00};
        off_enc_status_  = 0;
        off_latch_valid_ = 1;
        off_set_done_    = 2;
        off_underflow_   = 3;
        off_overflow_    = 4;
        fld_counter_     = {16, 16};
        fld_latch_       = {32, 16};
        off_stm_status_  = 48;
        off_sync_err_    = kInvalid;
    }
    return sm_in_.enabled && sm_out_.enabled &&
           sm_in_.length <= 512 && sm_out_.length <= 512 &&
           off_stm_enable_ != kInvalid && off_stm_status_ != kInvalid;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> StepperTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: POS interface out SM{} {}B / in SM{} {}B",
                logPrefix().c_str(), sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> StepperTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> StepperTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> StepperTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> StepperTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> StepperTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Bit/field access helpers
// ---------------------------------------------------------------------------

bool StepperTerminal::inBit(uint32_t bit_off) const {
    return bit_off != kInvalid && detail::imageBit(in_buf_, bit_off);
}

void StepperTerminal::setOutBit(uint32_t bit_off, bool v) {
    if (bit_off != kInvalid) detail::setImageBit(out_buf_, bit_off, v);
}

int64_t StepperTerminal::inSigned64(uint32_t bit_off, uint8_t bits) const {
    if (bit_off == kInvalid || bits == 0) return 0;
    const size_t byte = bit_off / 8;
    if (byte + (bits + 7) / 8 > in_buf_.size()) return 0;
    return detail::signExtendLE(in_buf_.data() + byte, bits);
}

int32_t StepperTerminal::inSigned(uint32_t bit_off, uint8_t bits) const {
    return static_cast<int32_t>(inSigned64(bit_off, bits));
}

void StepperTerminal::outSigned(uint32_t bit_off, uint8_t bits, int32_t v) {
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
// Drive control
// ---------------------------------------------------------------------------

void StepperTerminal::setEnable(bool on)      { setOutBit(off_stm_enable_, on); }
void StepperTerminal::setReset(bool on)       { setOutBit(off_stm_reset_, on); }
void StepperTerminal::setReduceTorque(bool on){ setOutBit(off_stm_rtq_, on); }

void StepperTerminal::setVelocity(int16_t v) {
    outSigned(fld_velocity_.bit_off, fld_velocity_.bit_len, v);
}

void StepperTerminal::setTargetPosition(int32_t pos) {
    outSigned(fld_pos_target_.bit_off, fld_pos_target_.bit_len, pos);
}

void StepperTerminal::setCounterValue(int32_t v) {
    outSigned(fld_set_counter_val_.bit_off, fld_set_counter_val_.bit_len, v);
}

void StepperTerminal::setSetCounter(bool on)  { setOutBit(off_set_counter_, on); }
void StepperTerminal::setLatchExtPos(bool on) { setOutBit(off_latch_pos_, on); }
void StepperTerminal::setLatchExtNeg(bool on) { setOutBit(off_latch_neg_, on); }
void StepperTerminal::setLatchC(bool on)      { setOutBit(off_enc_latch_c_, on); }

uint16_t StepperTerminal::stmControlWord() const {
    if (off_stm_enable_ == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = off_stm_enable_ / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}

uint16_t StepperTerminal::encControlWord() const {
    if (off_enc_ctrl_ == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = off_enc_ctrl_ / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}

// ---------------------------------------------------------------------------
// Drive status
// ---------------------------------------------------------------------------

// STM status bit ordinals inside the packed word starting at the first
// 0x6010 entry (see the ESI entry order: subs 1..7 then 12,13 at 11,12).
bool StepperTerminal::readyToEnable()  const { return inBit(off_stm_status_ + 0); }
bool StepperTerminal::ready()          const { return inBit(off_stm_status_ + 1); }
bool StepperTerminal::warning()        const { return inBit(off_stm_status_ + 2); }
bool StepperTerminal::error()          const { return inBit(off_stm_status_ + 3); }
bool StepperTerminal::movingPositive() const { return inBit(off_stm_status_ + 4); }
bool StepperTerminal::movingNegative() const { return inBit(off_stm_status_ + 5); }
bool StepperTerminal::torqueReduced()  const { return inBit(off_stm_status_ + 6); }
bool StepperTerminal::digitalInput1()  const { return inBit(off_stm_status_ + 11); }
bool StepperTerminal::digitalInput2()  const { return inBit(off_stm_status_ + 12); }

bool StepperTerminal::latchValid()       const { return inBit(off_latch_valid_); }
bool StepperTerminal::setCounterDone()   const { return inBit(off_set_done_); }
bool StepperTerminal::counterUnderflow() const { return inBit(off_underflow_); }
bool StepperTerminal::counterOverflow()  const { return inBit(off_overflow_); }
bool StepperTerminal::syncError()        const { return inBit(off_sync_err_); }

int32_t StepperTerminal::counterValue() const {
    return inSigned(fld_counter_.bit_off, fld_counter_.bit_len);
}

int32_t StepperTerminal::latchValue() const {
    return inSigned(fld_latch_.bit_off, fld_latch_.bit_len);
}

uint16_t StepperTerminal::stmStatusWord() const {
    if (off_stm_status_ == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = off_stm_status_ / 8;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

uint16_t StepperTerminal::encStatusWord() const {
    if (off_enc_status_ == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = off_enc_status_ / 8;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

} // namespace Beckhoff

} // namespace EtherCAT
