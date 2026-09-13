/**
 * @file PositionInputTerminal.cpp
 * @brief Implementation of the generic position/encoder input driver.
 *
 * See PositionInputTerminal.hpp for the API documentation.  Bring-up
 * machinery lives in TerminalBase; this file resolves the per-channel
 * position/latch/status offsets from the SII TxPDO list and implements
 * the 16…64-bit field readers.
 */

#include "tether/Beckhoff/PositionInputTerminal.hpp"

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

static const char* TAG = "PositionInputTerminal";

namespace {

/// Read up to 64 bits little-endian at `p`, zero-extended.
inline uint64_t readRaw(const uint8_t* p, uint8_t bits) {
    uint64_t raw = 0;
    const size_t n = (bits + 7) / 8;
    std::memcpy(&raw, p, n > 8 ? 8 : n);
    if (bits < 64) raw &= (uint64_t{1} << bits) - 1;
    return raw;
}

/// readRaw + sign extension at `bits` width.
inline int64_t readSigned(const uint8_t* p, uint8_t bits) {
    const uint64_t raw = readRaw(p, bits);
    if (bits >= 64) return static_cast<int64_t>(raw);
    const uint64_t sign = uint64_t{1} << (bits - 1);
    return static_cast<int64_t>((raw ^ sign) - sign);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

PositionInputTerminal::PositionInputTerminal(Master& master,
                                             uint16_t slave_index,
                                             const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

PositionInputTerminal::PositionInputTerminal(Master& master,
                                             const DiscoveredSlave& slave,
                                             const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

PositionInputTerminal::~PositionInputTerminal() = default;

Result<PositionInputTerminal> PositionInputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<PositionInputTerminal> PositionInputTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) {
            return PositionInputTerminal(master, s, identity);
        }
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool PositionInputTerminal::resolveLayout() {
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
            ChannelLayout ch{};
            ch.status_off = rc.status_off;
            for (const auto& v : rc.values) {
                ch.fields.push_back(
                    ChannelLayout::Field{v.byte_off, v.bit_len,
                                         v.index, v.subindex});
            }
            channels_.push_back(std::move(ch));
        }
    }
#endif
    // Fallback without SII data: identity num_bits channels of 8B each
    // (32-bit status + 32-bit counter — the dominant EL51xx shape).
    if (!sm_in_.enabled && identity_.num_bits > 0 &&
        identity_.num_bits <= kMaxChannels) {
        sm_in_.channel   = 3;
        sm_in_.phys_addr = 0x1180;
        sm_in_.ctrl      = 0x20;
        sm_in_.length    = static_cast<uint16_t>(identity_.num_bits * 8);
        sm_in_.enabled   = true;
        sm_in_.first_pdo = 0x1A00;
        for (uint16_t ch = 0; ch < identity_.num_bits; ++ch) {
            ChannelLayout c{};
            c.status_off = static_cast<int16_t>(ch * 8);
            c.fields.push_back(ChannelLayout::Field{
                static_cast<uint16_t>(ch * 8 + 4), 32, 0x6000, 0x11});
            channels_.push_back(std::move(c));
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

Result<> PositionInputTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};

    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        TETHER_LOGE(TAG, "{}: no usable position-input layout "
                    "(process-input SM disabled, zero-length, or "
                    "unrecognized PDO shape)", logPrefix().c_str());
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: input SM{} @ 0x{:04X} len {} ctrl 0x{:02X}"
                " ({} position channel(s)){}",
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

Result<> PositionInputTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> PositionInputTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> PositionInputTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> PositionInputTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> PositionInputTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — positions live", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

size_t PositionInputTerminal::valueCount(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    return channels_[ch].fields.size();
}

int64_t PositionInputTerminal::value(size_t ch, size_t i) const {
    if (ch >= channels_.size() || i >= channels_[ch].fields.size()) return 0;
    const auto& f = channels_[ch].fields[i];
    if (f.byte_off + (f.bit_len + 7) / 8 > in_buf_.size()) return 0;
    return readSigned(in_buf_.data() + f.byte_off, f.bit_len);
}

uint64_t PositionInputTerminal::rawValue(size_t ch, size_t i) const {
    if (ch >= channels_.size() || i >= channels_[ch].fields.size()) return 0;
    const auto& f = channels_[ch].fields[i];
    if (f.byte_off + (f.bit_len + 7) / 8 > in_buf_.size()) return 0;
    return readRaw(in_buf_.data() + f.byte_off, f.bit_len);
}

uint16_t PositionInputTerminal::status(size_t ch) const {
    if (ch >= channels_.size()) return 0;
    const auto& c = channels_[ch];
    if (c.status_off < 0 ||
        static_cast<size_t>(c.status_off) + 2 > in_buf_.size()) return 0;
    uint16_t s;
    std::memcpy(&s, in_buf_.data() + c.status_off, 2);
    return s;
}

bool PositionInputTerminal::hasStatus(size_t ch) const {
    if (ch >= channels_.size()) return false;
    return channels_[ch].status_off >= 0;
}

size_t PositionInputTerminal::channelBits(size_t ch) const {
    if (ch >= channels_.size() || channels_[ch].fields.empty()) return 0;
    return channels_[ch].fields[0].bit_len;
}

std::pair<uint16_t, uint8_t> PositionInputTerminal::valueObject(
    size_t ch, size_t i) const {
    if (ch >= channels_.size() || i >= channels_[ch].fields.size()) {
        return {0, 0};
    }
    const auto& f = channels_[ch].fields[i];
    return {f.index, f.subindex};
}

} // namespace Beckhoff

} // namespace EtherCAT
