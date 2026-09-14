/**
 * @file PowerMeterTerminal.hpp
 * @brief Beckhoff EL34xx three-phase power-measurement terminals
 *
 * The EL34xx family measures grid/power quantities (voltage, current,
 * active/apparent/reactive power, power factor, frequency, energy).
 * The ESI shows three process-data sub-shapes:
 *
 *   Family A — fixed per-phase PDOs (EL3403):
 *     phase i object 0x6000+i*0x10: Current(:17), Voltage(:18),
 *     Active power(:19), all 32-bit REAL (float), plus index echo.
 *
 *   Family B — grouped per-phase PDOs (EL3423, EL3443, EL3453, EL3483):
 *     status object  0x6000+i*0x10 (over/undervoltage, overcurrent, ...)
 *     V/I object     0x6001+i*0x10: Voltage(:17), Current(:18)
 *     power object   0x6002+i*0x10: Active(:17), Apparent(:18),
 *                                  Reactive(:19), PowerFactor(:20)
 *     zero-crossing  0x6006+i*0x10: 64-bit timestamp (where mapped)
 *
 *   Family C — index-addressed (EL3413, EL3433, EL3475):
 *     RX carries per-selector Index (+Channel) objects 0x7000+i*0x10;
 *     the TX result PDO echoes Index/Channel/Value for the requested
 *     measurement.  Status PDOs per phase as in family B.
 *
 * All offsets resolve from the SII PDO entries by (object, subindex);
 * the family is detected by probing which objects carry 32-bit value
 * entries.  EKM1101 (energy coupler) shares the shape.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known power-measurement terminals (from the Beckhoff ESIs)
// ============================================================================
// num_bits holds the phase count.  All ESI-verified for shape; none
// verified on hardware yet — supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL3403{0x00000002, 0x0D4B3052, 3, "EL3403"};
inline constexpr DeviceIdentity EL3413{0x00000002, 0x0D553052, 3, "EL3413"};
inline constexpr DeviceIdentity EL3423{0x00000002, 0x0D5F3052, 3, "EL3423"};
inline constexpr DeviceIdentity EL3433{0x00000002, 0x0D693052, 3, "EL3433"};
inline constexpr DeviceIdentity EL3443{0x00000002, 0x0D733052, 3, "EL3443"};
inline constexpr DeviceIdentity EL3453{0x00000002, 0x0D7D3052, 3, "EL3453"};
inline constexpr DeviceIdentity EL3475{0x00000002, 0x0D933052, 6, "EL3475"};
inline constexpr DeviceIdentity EL3483{0x00000002, 0x0D9B3052, 3, "EL3483"};
// EKM1101 energy-measurement coupler (status + packed fields).
inline constexpr DeviceIdentity EKM1101{0x00000002, 0x4FE17CD9, 1, "EKM1101"};

/// Every known power-measurement terminal.
inline constexpr std::array kPowerMeterTerminals{
    EL3403, EL3413, EL3423, EL3433, EL3443, EL3453, EL3475, EL3483,
    EKM1101,
};

} // namespace Devices

// ============================================================================
// PowerMeterTerminal
// ============================================================================

class PowerMeterTerminal : public TerminalBase {
public:
    static constexpr size_t kMaxPhases = 8;

    // -- Construction / factories ----------------------------------------------

    PowerMeterTerminal(Master& master, uint16_t slave_index,
                       const DeviceIdentity& identity);
    PowerMeterTerminal(Master& master, const DiscoveredSlave& slave,
                       const DeviceIdentity& identity);

    ~PowerMeterTerminal() override;

    PowerMeterTerminal(PowerMeterTerminal&&) noexcept            = default;
    PowerMeterTerminal& operator=(PowerMeterTerminal&&) noexcept = default;
    PowerMeterTerminal(const PowerMeterTerminal&)                = delete;
    PowerMeterTerminal& operator=(const PowerMeterTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<PowerMeterTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);
    static Result<PowerMeterTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Bring-up ------------------------------------------------------------------

    /// Standalone bring-up to SAFE-OP (position addressing).
    Result<> configure();

    /// configure() then OP (+ managed realtime loop by default).
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /// Chained-operation contract (shared logical address space).
    Result<> prepareForLogicalExchange();
    Result<> mapLogicalAndEnterSafeOp();
    Result<> requestOp(int timeout_ms);

    // -- Phases / status --------------------------------------------------------------

    /// Number of resolved measurement phases (0 before prepare()).
    size_t phaseCount() const { return phases_.size(); }

    /// Raw per-phase status word (16 bit) — over/undervoltage,
    /// overcurrent, guard warnings, inaccurate-flags.
    uint16_t statusWord(size_t phase) const;

    /// Sync-error flag inside the phase status PDO, when mapped.
    bool syncError(size_t phase) const;

    // -- Measurements (families A and B) -----------------------------------------------
    // Beckhoff power values are 32-bit REAL (IEEE-754 float).

    std::optional<float> voltage(size_t phase)       const;
    std::optional<float> current(size_t phase)       const;
    std::optional<float> activePower(size_t phase)   const;
    std::optional<float> apparentPower(size_t phase) const;
    std::optional<float> reactivePower(size_t phase) const;
    std::optional<float> powerFactor(size_t phase)   const;

    // -- Index-addressed measurements (family C) -----------------------------------------

    /// Number of index selectors (RX "Index" fields, EL3413-class).
    size_t indexSelectors() const { return selectors_.size(); }

    /// Write the 8-bit measurement index for selector `sel`.
    void setIndexSelector(size_t sel, uint8_t index);

    /// The value returned for selector `sel` (32-bit REAL), and the
    /// echoed Index/Channel bytes when mapped.
    std::optional<float>   indexedValue(size_t sel)   const;
    std::optional<uint8_t> indexedIndexEcho(size_t sel) const;
    std::optional<uint8_t> indexedChannelEcho(size_t sel) const;

    // -- Generic access ------------------------------------------------------------------

    /// Read a 32-bit REAL input entry by (object, subindex, occurrence).
    std::optional<float> valueReal(uint16_t index, int subindex,
                                   uint32_t occurrence = 0) const;
    /// Read any input entry zero-extended (up to 64 bit).
    bool valueRaw(uint16_t index, int subindex, uint32_t occurrence,
                  uint64_t& out) const;
    /// Write an output entry (up to 32 bit, byte-aligned).
    bool fieldOut(uint16_t index, int subindex, uint32_t occurrence,
                  uint64_t value);

private:
    static constexpr uint32_t kInvalid = ~0u;

    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
        uint16_t index   = 0;
        uint8_t  subindex = 0;
    };

    /// Per-phase resolved layout.
    struct Phase {
        uint16_t stat_obj   = 0;        ///< 0x6000+i*0x10 status object
        uint32_t off_status = kInvalid; ///< first status bit
        uint32_t off_sync   = kInvalid; ///< sync-error bit (varies)
        // semantic value fields (family A/B)
        Field voltage, current, active_power,
              apparent_power, reactive_power, power_factor;
    };

    /// Index-addressed selector (family C).
    struct Selector {
        Field    index_out;     ///< RX 8-bit "Index" field
        Field    value_in;      ///< TX 32-bit result value
        Field    index_echo;    ///< TX 8-bit echoed index
        Field    channel_echo;  ///< TX 8-bit echoed channel
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    std::optional<float> inReal(const Field& f) const;
    uint64_t inRaw(const Field& f, bool& ok) const;

    std::vector<Phase>    phases_;
    std::vector<Selector> selectors_;
};

} // namespace Beckhoff

} // namespace EtherCAT
