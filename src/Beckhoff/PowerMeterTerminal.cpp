/**
 * @file PowerMeterTerminal.cpp
 * @brief Implementation of the EL34xx power-measurement terminal driver.
 *
 * See PowerMeterTerminal.hpp for the family model.  Phase count comes
 * from the number of status objects (0x6000/0x6010/... low nibble 0);
 * semantic value fields are resolved by probing the per-family object
 * layout — family B (V/I + P/Q/S/PF split objects) is tried first,
 * then family A (EL3403 single object), then the index-addressed
 * family C selectors.
 */

#include "tether/Beckhoff/PowerMeterTerminal.hpp"

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

static const char* TAG = "PowerMeterTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

PowerMeterTerminal::PowerMeterTerminal(Master& master, uint16_t slave_index,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

PowerMeterTerminal::PowerMeterTerminal(Master& master,
                                       const DiscoveredSlave& slave,
                                       const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

PowerMeterTerminal::~PowerMeterTerminal() = default;

Result<PowerMeterTerminal> PowerMeterTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<PowerMeterTerminal> PowerMeterTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return PowerMeterTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool PowerMeterTerminal::resolveLayout() {
    resolveSyncManagers();
    phases_.clear();
    selectors_.clear();

#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled && sm_in_.length > 0) {
        const auto& tx = *info_->tx_pdos;
        const uint8_t ch = sm_in_.channel;

        /// Fill `f` when a >=8-bit entry (index,sub) exists in the image.
        auto resolveIn = [&](uint16_t idx, int sub, uint32_t occ,
                             Field& f) {
            f = {};
            if (detail::findValueEntry(tx, ch, idx, sub, occ, 8,
                                       f.bit_off, f.bit_len)) {
                f.index    = idx;
                f.subindex = static_cast<uint8_t>(sub);
            }
        };

        // Phase count = highest phase-indexed 0x60xx object + 1
        // (phase i lives at 0x6000 + 0x10*i .. 0x600F + 0x10*i).
        int n_phases = 0;
        for (const auto& pdo : tx) {
            if (pdo.sync_manager != ch) continue;
            for (const auto& e : pdo.entries) {
                if (e.index >= 0x6000 && e.index < 0x6100) {
                    const int ph = (e.index - 0x6000) / 0x10 + 1;
                    n_phases = std::max(n_phases, ph);
                }
            }
        }
        n_phases = std::min<int>(n_phases, kMaxPhases);
        if (n_phases == 0 && identity_.num_bits) {
            n_phases = std::min<int>(identity_.num_bits, kMaxPhases);
        }
        phases_.resize(n_phases);

        for (int p = 0; p < n_phases; ++p) {
            Phase& ph  = phases_[p];
            const uint16_t base =
                static_cast<uint16_t>(0x6000 + 0x10 * p);
            ph.stat_obj = base;

            uint32_t off;
            if (detail::findEntryBitOffset(tx, ch, base, -1, 0, off)) {
                ph.off_status = off;
            }
            // Sync error: sub :0E inside the status object (EL3403/
            // EL3413 style), or 0x1C32:32 elsewhere in the PDO.
            if (!detail::findEntryBitOffset(tx, ch, base, 0x0E, 0,
                                            ph.off_sync)) {
                detail::findEntryBitOffset(tx, ch, 0x1C32, 32,
                                           static_cast<uint32_t>(p),
                                           ph.off_sync);
            }

            // Family B: split V/I object (base+1) and power object
            // (base+2); both use subindex 0x11+ for 32-bit REAL values.
            Field v, i;
            resolveIn(base + 1, 0x11, 0, v);
            resolveIn(base + 1, 0x12, 0, i);
            if (v.bit_len >= 16 && i.bit_len >= 16) {
                ph.voltage = v; ph.current = i;
                resolveIn(base + 2, 0x11, 0, ph.active_power);
                resolveIn(base + 2, 0x12, 0, ph.apparent_power);
                resolveIn(base + 2, 0x13, 0, ph.reactive_power);
                resolveIn(base + 2, 0x14, 0, ph.power_factor);
            } else {
                // Family A (EL3403): Current(:17), Voltage(:18),
                // ActivePower(:19) inside the status object itself.
                resolveIn(base, 0x11, 0, ph.current);
                resolveIn(base, 0x12, 0, ph.voltage);
                resolveIn(base, 0x13, 0, ph.active_power);
            }
        }

        // Family C selectors: RX 0x70xx objects carrying an 8- or 16-bit
        // "Index" field (sub 1 for EL3413, subs 0x11.. for EL3475).
        if (info_ && info_->rx_pdos && sm_out_.enabled &&
            sm_out_.length > 0) {
            const auto& rx = *info_->rx_pdos;
            const uint8_t rch = sm_out_.channel;
            std::vector<uint16_t> sel_objs;
            for (const auto& pdo : rx) {
                if (pdo.sync_manager != rch) continue;
                for (const auto& e : pdo.entries) {
                    if (e.index >= 0x7000 && e.index <= 0x71FF &&
                        e.index != 0 &&
                        std::ranges::find(sel_objs, e.index) ==
                            sel_objs.end()) {
                        sel_objs.push_back(e.index);
                    }
                }
            }
            std::ranges::sort(sel_objs);
            for (const uint16_t o : sel_objs) {
                Selector s;
                // 8-bit Index at :01 (EL3413/3433) or 16-bit index
                // fields at :11..:14 (EL3475 multi-index).
                if (!detail::findValueEntry(rx, rch, o, 0x01, 0, 8,
                                            s.index_out.bit_off,
                                            s.index_out.bit_len)) {
                    detail::findValueEntry(rx, rch, o, 0x11, 0, 8,
                                           s.index_out.bit_off,
                                           s.index_out.bit_len);
                }
                if (s.index_out.bit_len == 0) continue;
                s.index_out.index = o;
                s.index_out.subindex = 1;
                // Result object: the 0x60xx object whose (sub 0x13)
                // is a >=16-bit value — EL3413 uses 0x6030:19 Value.
                for (const auto& pdo : tx) {
                    if (pdo.sync_manager != ch) continue;
                    for (const auto& e : pdo.entries) {
                        if (e.index >= 0x6030 && e.index < 0x6100 &&
                            e.subindex == 0x13 && e.bit_length >= 16) {
                            detail::findValueEntry(
                                tx, ch, e.index, 0x13, 0, 16,
                                s.value_in.bit_off, s.value_in.bit_len);
                            s.value_in.index = e.index;
                            s.value_in.subindex = 0x13;
                            detail::findValueEntry(
                                tx, ch, e.index, 0x11, 0, 8,
                                s.index_echo.bit_off,
                                s.index_echo.bit_len);
                            s.index_echo.index = e.index;
                            detail::findValueEntry(
                                tx, ch, e.index, 0x12, 0, 8,
                                s.channel_echo.bit_off,
                                s.channel_echo.bit_len);
                            s.channel_echo.index = e.index;
                        }
                    }
                }
                selectors_.push_back(s);
            }
        }
    }
#endif

    if (!sm_out_.enabled) {
        // Index-addressed devices still need the RX image even without
        // SII — declare a small scratch out image.
        sm_out_ = {2, 0x1100, 4, 0x24, true, 0x1600};
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1180, 64, 0x20, true, 0x1A00};
    }
    return sm_in_.enabled && sm_in_.length <= 512 &&
           sm_out_.length <= 512;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> PowerMeterTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: {} phases, {} index selectors, "
                "out SM{} {}B / in SM{} {}B",
                logPrefix().c_str(), phases_.size(), selectors_.size(),
                sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> PowerMeterTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> PowerMeterTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> PowerMeterTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> PowerMeterTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> PowerMeterTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Field access
// ---------------------------------------------------------------------------

uint64_t PowerMeterTerminal::inRaw(const Field& f, bool& ok) const {
    ok = false;
    if (f.bit_off == kInvalid || f.bit_len == 0) return 0;
    if (f.bit_off % 8 != 0) return 0;   // only byte-aligned fields
    const size_t byte = f.bit_off / 8;
    const size_t len  = (f.bit_len + 7) / 8;
    if (byte + len > in_buf_.size()) return 0;
    uint64_t raw = 0;
    std::memcpy(&raw, in_buf_.data() + byte, len);
    ok = true;
    return raw;
}

std::optional<float> PowerMeterTerminal::inReal(const Field& f) const {
    bool ok;
    const uint64_t raw = inRaw(f, ok);
    if (!ok) return std::nullopt;
    if (f.bit_len == 32) {
        const uint32_t u = static_cast<uint32_t>(raw);
        float v;
        std::memcpy(&v, &u, 4);
        return v;
    }
    // Non-REAL widths: the sign-extended integer value as float.
    return static_cast<float>(detail::signExtend64(raw, f.bit_len));
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

uint16_t PowerMeterTerminal::statusWord(size_t phase) const {
    if (phase >= phases_.size() ||
        phases_[phase].off_status == kInvalid) return 0;
    uint16_t w = 0;
    const size_t byte = phases_[phase].off_status / 8;
    if (byte + 2 <= in_buf_.size()) {
        std::memcpy(&w, in_buf_.data() + byte, 2);
    }
    return w;
}

bool PowerMeterTerminal::syncError(size_t phase) const {
    if (phase >= phases_.size()) return false;
    const uint32_t off = phases_[phase].off_sync;
    return off != kInvalid && detail::imageBit(in_buf_, off);
}

// ---------------------------------------------------------------------------
// Measurements
// ---------------------------------------------------------------------------

std::optional<float> PowerMeterTerminal::voltage(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].voltage)
                                  : std::nullopt;
}
std::optional<float> PowerMeterTerminal::current(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].current)
                                  : std::nullopt;
}
std::optional<float> PowerMeterTerminal::activePower(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].active_power)
                                  : std::nullopt;
}
std::optional<float> PowerMeterTerminal::apparentPower(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].apparent_power)
                                  : std::nullopt;
}
std::optional<float> PowerMeterTerminal::reactivePower(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].reactive_power)
                                  : std::nullopt;
}
std::optional<float> PowerMeterTerminal::powerFactor(size_t phase) const {
    return phase < phases_.size() ? inReal(phases_[phase].power_factor)
                                  : std::nullopt;
}

// ---------------------------------------------------------------------------
// Index-addressed selectors (family C)
// ---------------------------------------------------------------------------

void PowerMeterTerminal::setIndexSelector(size_t sel, uint8_t index) {
    if (sel >= selectors_.size()) return;
    const Field& f = selectors_[sel].index_out;
    if (f.bit_off == kInvalid || f.bit_off % 8 != 0) return;
    const size_t byte = f.bit_off / 8;
    if (byte >= out_buf_.size()) return;
    out_buf_[byte] = index;
    if (f.bit_len > 8 && byte + 1 < out_buf_.size()) out_buf_[byte + 1] = 0;
}

std::optional<float> PowerMeterTerminal::indexedValue(size_t sel) const {
    if (sel >= selectors_.size()) return std::nullopt;
    return inReal(selectors_[sel].value_in);
}

std::optional<uint8_t> PowerMeterTerminal::indexedIndexEcho(size_t sel) const {
    if (sel >= selectors_.size()) return std::nullopt;
    bool ok;
    const uint64_t raw = inRaw(selectors_[sel].index_echo, ok);
    if (!ok) return std::nullopt;
    return static_cast<uint8_t>(raw);
}

std::optional<uint8_t> PowerMeterTerminal::indexedChannelEcho(size_t sel) const {
    if (sel >= selectors_.size()) return std::nullopt;
    bool ok;
    const uint64_t raw = inRaw(selectors_[sel].channel_echo, ok);
    if (!ok) return std::nullopt;
    return static_cast<uint8_t>(raw);
}

// ---------------------------------------------------------------------------
// Generic (object, subindex) access
// ---------------------------------------------------------------------------

std::optional<float> PowerMeterTerminal::valueReal(uint16_t index, int subindex,
                                                 uint32_t occurrence) const {
#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled) {
        Field f;
        if (!detail::findValueEntry(*info_->tx_pdos, sm_in_.channel,
                                    index, subindex, occurrence, 8,
                                    f.bit_off, f.bit_len)) {
            return std::nullopt;
        }
        f.index = index;
        return inReal(f);
    }
#endif
    (void)index; (void)subindex; (void)occurrence;
    return std::nullopt;
}

bool PowerMeterTerminal::valueRaw(uint16_t index, int subindex,
                                  uint32_t occurrence, uint64_t& out) const {
#if TETHER_ENABLE_SII
    if (info_ && info_->tx_pdos && sm_in_.enabled) {
        Field f;
        if (!detail::findValueEntry(*info_->tx_pdos, sm_in_.channel,
                                    index, subindex, occurrence, 8,
                                    f.bit_off, f.bit_len)) {
            return false;
        }
        bool ok;
        out = inRaw(f, ok);
        return ok;
    }
#endif
    (void)index; (void)subindex; (void)occurrence; (void)out;
    return false;
}

bool PowerMeterTerminal::fieldOut(uint16_t index, int subindex,
                                  uint32_t occurrence, uint64_t value) {
#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        uint32_t off; uint8_t len;
        if (!detail::findValueEntry(*info_->rx_pdos, sm_out_.channel,
                                    index, subindex, occurrence, 8,
                                    off, len)) return false;
        if (off % 8 != 0) return false;
        const size_t byte = off / 8;
        const size_t n = (len + 7) / 8;
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
