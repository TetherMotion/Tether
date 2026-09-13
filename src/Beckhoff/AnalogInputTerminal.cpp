/**
 * @file AnalogInputTerminal.cpp
 * @brief Implementation of the generic analog-input terminal driver.
 *
 * See AnalogInputTerminal.hpp for the API documentation.  The bring-up
 * machinery lives in TerminalBase; this file only adds the analog
 * channel-layout resolution (per-channel value/status offsets inside the
 * SM3 image) and the value accessors.
 */

#include "tether/Beckhoff/AnalogInputTerminal.hpp"

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

static const char* TAG = "AnalogInputTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

AnalogInputTerminal::AnalogInputTerminal(Master& master,
                                         uint16_t slave_index,
                                         const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

AnalogInputTerminal::AnalogInputTerminal(Master& master,
                                         const DiscoveredSlave& slave,
                                         const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

AnalogInputTerminal::~AnalogInputTerminal() = default;

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

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool AnalogInputTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        std::vector<detail::ResolvedChannel> resolved;
        uint32_t image_bits = 0;
        if (!detail::resolveValueChannels(*info_->tx_pdos, sm_in_.channel,
                                          16, resolved, image_bits)) {
            TETHER_LOGE(TAG, "{}: non-byte-aligned PDO layout — unsupported",
                        logPrefix().c_str());
            return false;
        }
        if (resolved.empty() || image_bits / 8 > sm_in_.length) {
            TETHER_LOGE(TAG, "{}: TxPDO layout resolves to {}B over {} "
                        "channel(s) but SM{} holds {}B — unsupported "
                        "terminal shape",
                        logPrefix().c_str(), image_bits / 8,
                        resolved.size(), sm_in_.channel, sm_in_.length);
            return false;
        }
        channels_.clear();
        for (const auto& rc : resolved) {
            // The channel value is the last >=16-bit entry of the PDO.
            const auto& v = rc.values.back();
            channels_.push_back(ChannelLayout{v.byte_off, v.bit_len,
                                              rc.status_off});
        }
    }
#endif
    // Fallback when SII resolution was impossible (build without SII, or an
    // ESI variant not yet in the registry): assume the dominant Beckhoff
    // shape — identity num_bits channels of 4B each (16-bit status +
    // 16-bit value) on SM3 @ 0x1180.
    if (!sm_in_.enabled && identity_.num_bits > 0 &&
        identity_.num_bits <= kMaxChannels) {
        sm_in_.channel   = 3;
        sm_in_.phys_addr = 0x1180;
        sm_in_.ctrl      = 0x20;
        sm_in_.length    = static_cast<uint16_t>(identity_.num_bits * 4);
        sm_in_.enabled   = true;
        sm_in_.first_pdo = 0x1A00;
        for (uint16_t ch = 0; ch < identity_.num_bits; ++ch) {
            channels_.push_back(
                ChannelLayout{static_cast<uint16_t>(ch * 4 + 2), 16,
                              static_cast<int16_t>(ch * 4)});
        }
    }
    return sm_in_.enabled && sm_in_.length > 0 && !channels_.empty() &&
           channels_.size() <= kMaxChannels &&
           sm_in_.length <= kMaxImageBytes &&
           sm_out_.length <= kMaxImageBytes;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> AnalogInputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        TETHER_LOGE(TAG, "{}: no usable analog-input layout "
                    "(SM3 disabled, zero-length, or unrecognized PDO shape)",
                    logPrefix().c_str());
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: input SM{} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} analog channel(s)){}",
                logPrefix().c_str(), sm_in_.channel,
                sm_in_.phys_addr, sm_in_.length, sm_in_.ctrl,
                channels_.size(),
                sm_out_.enabled ? " +SM2 outputs" : "");

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> AnalogInputTerminal::configure() {
    // Standalone: position addressing (APRD straight out of the SM buffer)
    // needs no FMMU and no logical map.
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> AnalogInputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> AnalogInputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> AnalogInputTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> AnalogInputTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — inputs live", logPrefix().c_str());
    return {};
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

} // namespace Beckhoff

} // namespace EtherCAT
