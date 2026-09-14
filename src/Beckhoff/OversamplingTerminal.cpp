/**
 * @file OversamplingTerminal.cpp
 * @brief Implementation of the oversampling measurement driver.
 *
 * Channel/sample resolution delegates to
 * detail::resolveValueChannels() with min_value_bits = 16 — every
 * >=16-bit entry in a channel PDO is one sample of the current cycle.
 */

#include "tether/Beckhoff/OversamplingTerminal.hpp"

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

static const char* TAG = "OversamplingTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

OversamplingTerminal::OversamplingTerminal(Master& master,
                                           uint16_t slave_index,
                                           const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

OversamplingTerminal::OversamplingTerminal(
    Master& master, const DiscoveredSlave& slave,
    const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

OversamplingTerminal::~OversamplingTerminal() = default;

Result<OversamplingTerminal> OversamplingTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<OversamplingTerminal> OversamplingTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) {
            return OversamplingTerminal(master, s, identity);
        }
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool OversamplingTerminal::resolveLayout() {
    resolveSyncManagers();
    channels_.clear();

#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        std::vector<detail::ResolvedChannel> chs;
        uint32_t image_bits = 0;
        if (!detail::resolveValueChannels(*info_->tx_pdos,
                                          sm_in_.channel, 16,
                                          chs, image_bits)) {
            TETHER_LOGE(TAG, "{}: misaligned SM3 image",
                        logPrefix().c_str());
            return false;
        }
        if (chs.size() > kMaxChannels) {
            TETHER_LOGE(TAG, "{}: {} channels exceeds the {} limit",
                        logPrefix().c_str(), chs.size(), kMaxChannels);
            return false;
        }
        for (const auto& c : chs) {
            Channel ch;
            for (const auto& v : c.values) {
                if (ch.samples.size() >= kMaxSamples) break;
                ch.samples.push_back({v.byte_off, v.bit_len});
            }
            ch.status_off = c.status_off;
            channels_.push_back(std::move(ch));
        }
        if (channels_.empty()) {
            TETHER_LOGE(TAG, "{}: no sample channels in the SM3 image",
                        logPrefix().c_str());
            return false;
        }
    }
#endif

    if (!sm_in_.enabled) {
        // Fallback without SII: `num_bits` channels of one 16-bit sample
        // plus a 16-bit status each — the EL3702 default shape.
        const uint8_t n = identity_.num_bits ? identity_.num_bits : 2;
        sm_in_ = {3, 0x1180,
                  static_cast<uint16_t>(n * 4), 0x20, true, 0x1A00};
        channels_.resize(n);
        for (uint8_t i = 0; i < n; ++i) {
            channels_[i].samples.push_back(
                {static_cast<uint16_t>(i * 4 + 2), 16});
            channels_[i].status_off = static_cast<int16_t>(i * 4);
        }
    }
    // Oversampling terminals may still map a small control SM2 (e.g.
    // range/filter selects) — leave it enabled when present, otherwise
    // declare a zero-length image.
    if (!sm_out_.enabled) {
        sm_out_ = {2, 0x1100, 0, 0x24, false, 0x1600};
    }
    return sm_in_.enabled && sm_in_.length <= 4096;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> OversamplingTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: {} ch x {} samples, in SM{} {}B",
                logPrefix().c_str(), channels_.size(),
                channels_.empty() ? 0 : channels_[0].samples.size(),
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> OversamplingTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> OversamplingTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> OversamplingTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> OversamplingTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> OversamplingTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Sample access
// ---------------------------------------------------------------------------

size_t OversamplingTerminal::samplesPerCycle(size_t ch) const {
    return ch < channels_.size() ? channels_[ch].samples.size() : 0;
}

uint8_t OversamplingTerminal::sampleBits(size_t ch) const {
    return (ch < channels_.size() && !channels_[ch].samples.empty())
               ? channels_[ch].samples[0].bit_len
               : 0;
}

int32_t OversamplingTerminal::sample(size_t ch, size_t i) const {
    if (ch >= channels_.size() || i >= channels_[ch].samples.size()) {
        return 0;
    }
    const auto& s = channels_[ch].samples[i];
    const size_t len = (s.bit_len + 7) / 8;
    if (s.byte_off + len > in_buf_.size()) return 0;
    return static_cast<int32_t>(
        detail::signExtendLE(in_buf_.data() + s.byte_off, s.bit_len));
}

size_t OversamplingTerminal::samples(size_t ch,
                                     std::span<int32_t> dst) const {
    if (ch >= channels_.size()) return 0;
    const size_t n = std::min(dst.size(), channels_[ch].samples.size());
    for (size_t i = 0; i < n; ++i) dst[i] = sample(ch, i);
    return n;
}

uint16_t OversamplingTerminal::statusWord(size_t ch) const {
    if (ch >= channels_.size() || channels_[ch].status_off < 0) {
        return 0xffff;
    }
    const auto byte = static_cast<size_t>(channels_[ch].status_off);
    if (byte + 2 > in_buf_.size()) return 0xffff;
    uint16_t w;
    std::memcpy(&w, in_buf_.data() + byte, 2);
    return w;
}

bool OversamplingTerminal::hasStatus(size_t ch) const {
    return ch < channels_.size() && channels_[ch].status_off >= 0;
}

} // namespace Beckhoff

} // namespace EtherCAT
