/**
 * @file CommTerminal.cpp
 * @brief Implementation of the byte-FIFO communication-terminal driver.
 *
 * Per-channel resolution: within each process PDO the first entry is
 * the control/status register (8 or 16 bit); every following 8-bit
 * entry is one FIFO data byte.  Channel i pairs the i-th SM2 FIFO PDO
 * with the i-th SM3 FIFO PDO.
 */

#include "tether/Beckhoff/CommTerminal.hpp"

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

static const char* TAG = "CommTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

CommTerminal::CommTerminal(Master& master, uint16_t slave_index,
                           const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

CommTerminal::CommTerminal(Master& master, const DiscoveredSlave& slave,
                           const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

CommTerminal::~CommTerminal() = default;

Result<CommTerminal> CommTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<CommTerminal> CommTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return CommTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool CommTerminal::resolveLayout() {
    resolveSyncManagers();
    chans_.clear();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled && sm_out_.length > 0) {
        for (const auto& f : detail::resolveFifoChannels(
                 *info_->rx_pdos, sm_out_.channel, kMaxFifoBytes)) {
            Channel c;
            c.ctrl_off  = f.ctrl_off;
            c.ctrl_bits = f.ctrl_bits;
            c.dout_off  = f.data_off;
            c.dout_len  = f.data_len;
            chans_.push_back(c);
        }
    }
    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        const auto ins = detail::resolveFifoChannels(
            *info_->tx_pdos, sm_in_.channel, kMaxFifoBytes);
        for (size_t ci = 0; ci < ins.size(); ++ci) {
            if (ci >= chans_.size()) chans_.emplace_back();
            chans_[ci].stat_off  = ins[ci].ctrl_off;
            chans_[ci].stat_bits = ins[ci].ctrl_bits;
            chans_[ci].din_off   = ins[ci].data_off;
            chans_[ci].din_len   = ins[ci].data_len;
        }
    }
#endif

    if (!sm_out_.enabled) {
        // Fallback without SII: one channel, 16-bit ctrl + N data bytes.
        sm_out_ = {2, 0x1100, 24, 0x24, true, 0x1600};
        Channel c;
        c.ctrl_off  = 0; c.ctrl_bits = 16;
        c.dout_off  = 2; c.dout_len  = 22;
        chans_.push_back(c);
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1180, 24, 0x20, true, 0x1A00};
        if (chans_.empty()) chans_.push_back({});
        chans_[0].stat_off  = 0; chans_[0].stat_bits = 16;
        chans_[0].din_off   = 2; chans_[0].din_len   = 22;
    }
    if (chans_.empty()) {
        TETHER_LOGE(TAG, "{}: no FIFO channels in the process image",
                    logPrefix().c_str());
        return false;
    }
    return sm_in_.enabled && sm_out_.enabled &&
           sm_in_.length <= 512 && sm_out_.length <= 512;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> CommTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: {} FIFO channels, out SM{} {}B / in SM{} {}B",
                logPrefix().c_str(), chans_.size(),
                sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> CommTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> CommTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> CommTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> CommTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> CommTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Channel access
// ---------------------------------------------------------------------------

size_t CommTerminal::txCapacity(size_t ch) const {
    return ch < chans_.size() ? chans_[ch].dout_len : 0;
}
size_t CommTerminal::rxCapacity(size_t ch) const {
    return ch < chans_.size() ? chans_[ch].din_len : 0;
}

uint16_t CommTerminal::ctrlWord(size_t ch) const {
    if (ch >= chans_.size() || chans_[ch].ctrl_bits == 0) return 0;
    const auto& c = chans_[ch];
    const size_t len = (c.ctrl_bits + 7) / 8;
    if (c.ctrl_off + len > out_buf_.size()) return 0;
    uint16_t w = 0;
    std::memcpy(&w, out_buf_.data() + c.ctrl_off, len);
    return w;
}

void CommTerminal::setCtrlWord(size_t ch, uint16_t w) {
    if (ch >= chans_.size() || chans_[ch].ctrl_bits == 0) return;
    const auto& c = chans_[ch];
    const size_t len = (c.ctrl_bits + 7) / 8;
    if (c.ctrl_off + len > out_buf_.size()) return;
    std::memcpy(out_buf_.data() + c.ctrl_off, &w, len);
}

bool CommTerminal::ctrlBit(size_t ch, uint8_t bit) const {
    if (ch >= chans_.size() || bit >= chans_[ch].ctrl_bits) return false;
    return detail::imageBit(out_buf_, chans_[ch].ctrl_off * 8 + bit);
}

void CommTerminal::setCtrlBit(size_t ch, uint8_t bit, bool v) {
    if (ch >= chans_.size() || bit >= chans_[ch].ctrl_bits) return;
    detail::setImageBit(out_buf_, chans_[ch].ctrl_off * 8 + bit, v);
}

uint16_t CommTerminal::statusWord(size_t ch) const {
    if (ch >= chans_.size() || chans_[ch].stat_bits == 0) return 0;
    const auto& c = chans_[ch];
    const size_t len = (c.stat_bits + 7) / 8;
    if (c.stat_off + len > in_buf_.size()) return 0;
    uint16_t w = 0;
    std::memcpy(&w, in_buf_.data() + c.stat_off, len);
    return w;
}

bool CommTerminal::statusBit(size_t ch, uint8_t bit) const {
    if (ch >= chans_.size() || bit >= chans_[ch].stat_bits) return false;
    return detail::imageBit(in_buf_, chans_[ch].stat_off * 8 + bit);
}

// ---------------------------------------------------------------------------
// FIFO data
// ---------------------------------------------------------------------------

size_t CommTerminal::writeData(size_t ch, std::span<const uint8_t> src) {
    if (ch >= chans_.size()) return 0;
    const auto& c = chans_[ch];
    const size_t n = std::min(src.size(), static_cast<size_t>(c.dout_len));
    if (c.dout_off + n > out_buf_.size()) return 0;
    std::memcpy(out_buf_.data() + c.dout_off, src.data(), n);
    return n;
}

std::span<const uint8_t> CommTerminal::dataIn(size_t ch) const {
    if (ch >= chans_.size()) return {};
    const auto& c = chans_[ch];
    if (c.din_off + c.din_len > in_buf_.size()) return {};
    return {in_buf_.data() + c.din_off, c.din_len};
}

size_t CommTerminal::readData(size_t ch, std::span<uint8_t> dst) const {
    const auto view = dataIn(ch);
    const size_t n = std::min(dst.size(), view.size());
    std::memcpy(dst.data(), view.data(), n);
    return n;
}

} // namespace Beckhoff

} // namespace EtherCAT
