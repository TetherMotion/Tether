/**
 * @file AnalogOutputTerminal.cpp
 * @brief Implementation of the generic analog-output terminal driver.
 *
 * See AnalogOutputTerminal.hpp for the API documentation.  Bring-up
 * machinery lives in TerminalBase; this file adds the per-channel value
 * offsets inside the SM2 image and the setValue() accessors.
 */

#include "tether/Beckhoff/AnalogOutputTerminal.hpp"

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

static const char* TAG = "AnalogOutputTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

AnalogOutputTerminal::AnalogOutputTerminal(Master& master,
                                           uint16_t slave_index,
                                           const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

AnalogOutputTerminal::AnalogOutputTerminal(Master& master,
                                           const DiscoveredSlave& slave,
                                           const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

AnalogOutputTerminal::~AnalogOutputTerminal() = default;

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

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool AnalogOutputTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled && sm_out_.length > 0) {
        std::vector<detail::ResolvedChannel> resolved;
        uint32_t image_bits = 0;
        if (!detail::resolveValueChannels(*info_->rx_pdos, sm_out_.channel,
                                          16, resolved, image_bits)) {
            TETHER_LOGE(TAG, "{}: non-byte-aligned PDO layout — unsupported",
                        logPrefix().c_str());
            return false;
        }
        if (resolved.empty() || image_bits / 8 > sm_out_.length) {
            TETHER_LOGE(TAG, "{}: RxPDO layout resolves to {}B over {} "
                        "channel(s) but SM{} holds {}B — unsupported "
                        "terminal shape",
                        logPrefix().c_str(), image_bits / 8,
                        resolved.size(), sm_out_.channel, sm_out_.length);
            return false;
        }
        channels_.clear();
        for (const auto& rc : resolved) {
            const auto& v = rc.values.back();
            channels_.push_back(ChannelLayout{v.byte_off, v.bit_len});
        }
    }
#endif
    // Fallback when SII resolution was impossible: dominant Beckhoff
    // shape — identity num_bits channels of 2B each on SM2 @ 0x1100.
    if (!sm_out_.enabled && identity_.num_bits > 0 &&
        identity_.num_bits <= kMaxChannels) {
        sm_out_.channel   = 2;
        sm_out_.phys_addr = 0x1100;
        sm_out_.ctrl      = 0x24;
        sm_out_.length    = static_cast<uint16_t>(identity_.num_bits * 2);
        sm_out_.enabled   = true;
        sm_out_.first_pdo = 0x1600;
        for (uint16_t ch = 0; ch < identity_.num_bits; ++ch) {
            channels_.push_back(
                ChannelLayout{static_cast<uint16_t>(ch * 2), 16});
        }
    }
    return sm_out_.enabled && sm_out_.length > 0 && !channels_.empty() &&
           channels_.size() <= kMaxChannels &&
           sm_out_.length <= kMaxImageBytes &&
           sm_in_.length <= kMaxImageBytes;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> AnalogOutputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        TETHER_LOGE(TAG, "{}: no usable analog-output layout "
                    "(SM2 disabled, zero-length, or unrecognized PDO shape)",
                    logPrefix().c_str());
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: output SM{} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} analog channel(s)){}",
                logPrefix().c_str(), sm_out_.channel,
                sm_out_.phys_addr, sm_out_.length, sm_out_.ctrl,
                channels_.size(),
                sm_in_.enabled ? " +SM3 inputs" : "");

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> AnalogOutputTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> AnalogOutputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> AnalogOutputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> AnalogOutputTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> AnalogOutputTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — outputs live", logPrefix().c_str());
    return {};
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

} // namespace Beckhoff

} // namespace EtherCAT
