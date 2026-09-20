/**
 * @file CombinedIoTerminal.cpp
 * @brief Implementation of the combined packed-bit I/O driver.
 */

#include "tether/Beckhoff/CombinedIoTerminal.hpp"

#include <cstring>

#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "CombinedIoTerminal";

CombinedIoTerminal::CombinedIoTerminal(Master& master, uint16_t slave_index,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

CombinedIoTerminal::CombinedIoTerminal(Master& master,
                                       const DiscoveredSlave& slave,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

CombinedIoTerminal::~CombinedIoTerminal() = default;

Result<CombinedIoTerminal> CombinedIoTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<CombinedIoTerminal> CombinedIoTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return CombinedIoTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

bool CombinedIoTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    uint32_t dummy = 0;
    if (info_ && info_->tx_pdos && sm_in_.enabled) {
        detail::resolveBitChannels(*info_->tx_pdos, sm_in_.channel,
                                   in_offs_, dummy);
    }
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        detail::resolveBitChannels(*info_->rx_pdos, sm_out_.channel,
                                   out_offs_, dummy);
    }
#endif

    // No-SII fallback: num_bits packs (out << 8 | in), both packed
    // contiguously from bit 0 of their image.
    if (sm_in_.enabled && in_offs_.empty()) {
        const size_t n = identity_.num_bits & 0xFF;
        for (size_t i = 0; i < n; ++i) in_offs_.push_back(i);
    }
    if (sm_out_.enabled && out_offs_.empty()) {
        const size_t n = (identity_.num_bits >> 8) & 0xFF;
        for (size_t i = 0; i < n; ++i) out_offs_.push_back(i);
    }
    return sm_in_.enabled || sm_out_.enabled;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> CombinedIoTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);
    if (!resolveLayout())  return std::unexpected(Error::SmConfigFailed);

    TETHER_LOGI(TAG, "{}: {} in / {} out bits, SM{} {}B / SM{} {}B",
                logPrefix().c_str(), in_offs_.size(), out_offs_.size(),
                sm_in_.channel, sm_in_.length,
                sm_out_.channel, sm_out_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> CombinedIoTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> CombinedIoTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> CombinedIoTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> CombinedIoTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> CombinedIoTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Packed-bit access
// ---------------------------------------------------------------------------

bool CombinedIoTerminal::input(size_t i) const {
    return i < in_offs_.size() && detail::imageBit(in_buf_, in_offs_[i]);
}

void CombinedIoTerminal::setOutput(size_t i, bool on) {
    if (i < out_offs_.size()) detail::setImageBit(out_buf_, out_offs_[i], on);
}

bool CombinedIoTerminal::output(size_t i) const {
    return i < out_offs_.size() && detail::imageBit(out_buf_, out_offs_[i]);
}

void CombinedIoTerminal::allOutputsOff() {
    std::memset(out_buf_.data(), 0, out_buf_.size());
}

// ---------------------------------------------------------------------------
// Raw field access
// ---------------------------------------------------------------------------

bool CombinedIoTerminal::findInputField(uint16_t index, int subindex,
                                        uint32_t occurrence,
                                        uint32_t& bit_off) const {
#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled) {
        return detail::findEntryBitOffset(*info_->tx_pdos, sm_in_.channel,
                                          index, subindex, occurrence,
                                          bit_off);
    }
#endif
    (void)index; (void)subindex; (void)occurrence; (void)bit_off;
    return false;
}

bool CombinedIoTerminal::findOutputField(uint16_t index, int subindex,
                                         uint32_t occurrence,
                                         uint32_t& bit_off) const {
#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        return detail::findEntryBitOffset(*info_->rx_pdos, sm_out_.channel,
                                          index, subindex, occurrence,
                                          bit_off);
    }
#endif
    (void)index; (void)subindex; (void)occurrence; (void)bit_off;
    return false;
}

bool CombinedIoTerminal::fieldIn(uint16_t index, int subindex,
                               uint32_t occurrence, uint64_t& value) const {
    uint32_t bit_off = 0;
    uint8_t  bit_len = 0;
#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled &&
        detail::findValueEntry(*info_->tx_pdos, sm_in_.channel, index,
                               subindex, occurrence, 2, bit_off, bit_len) &&
        bit_off % 8 == 0 && bit_len <= 64) {
        const size_t byte = bit_off / 8;
        const size_t n = (bit_len + 7) / 8;
        if (byte + n > in_buf_.size()) return false;
        value = 0;
        std::memcpy(&value, in_buf_.data() + byte, n);
        if (bit_len % 8) value &= (uint64_t{1} << bit_len) - 1;
        return true;
    }
#endif
    (void)index; (void)subindex; (void)occurrence;
    value = 0;
    return false;
}

bool CombinedIoTerminal::fieldOut(uint16_t index, int subindex,
                                  uint32_t occurrence, uint64_t value) {
    uint32_t bit_off = 0;
    uint8_t  bit_len = 0;
#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled &&
        detail::findValueEntry(*info_->rx_pdos, sm_out_.channel, index,
                               subindex, occurrence, 2, bit_off, bit_len) &&
        bit_off % 8 == 0 && bit_len <= 64 && bit_len % 8 == 0) {
        const size_t byte = bit_off / 8;
        const size_t n = bit_len / 8;
        if (byte + n > out_buf_.size()) return false;
        std::memcpy(out_buf_.data() + byte, &value, n);
        return true;
    }
#endif
    (void)index; (void)subindex; (void)occurrence; (void)value;
    return false;
}

} // namespace Beckhoff
} // namespace EtherCAT
