/**
 * @file OversamplingTerminal.hpp
 * @brief Beckhoff oversampling measurement terminals (EL37xx, ELM3xxx)
 *
 * Oversampling terminals transfer N samples per channel per EtherCAT
 * cycle (DC-synchronized).  Their process image is a status prefix plus
 * a block of equal-width sample entries per channel:
 *
 *   EL3702/EL3742 : 2 ch, timestamped analog input
 *   EL3773/EL3783 : 3-phase power oversampling (50+ samples/ch/cycle)
 *   ELM30xx–ELM37xx : premium multi-range measurement (±30V, ±20mA,
 *                     TC, RTD, IEPE) with large per-channel sample blocks
 *
 * The driver resolves the layout through the shared
 * PdoChannelLayout::resolveValueChannels() machinery: each channel PDO
 * contributes one status prefix and K >=16-bit value entries — the K
 * samples of one cycle.  Non-value fields (timestamps, cycle counters)
 * remain reachable via value()/statusWord() plus rawInput().
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
// Known oversampling terminals (from the Beckhoff ESIs)
// ============================================================================
// num_bits holds the channel count.  All ESI-verified for shape; none
// verified on hardware yet — supported, not verified yet.

namespace Devices {

// EL37xx — timestamped / oversampling analog inputs.
inline constexpr DeviceIdentity EL3773{0x00000002, 0x0EBD3052, 3, "EL3773"};
inline constexpr DeviceIdentity EL3783{0x00000002, 0x0EC73052, 3, "EL3783"};

// ELM3xxx premium measurement modules (Beckhoff ELM product range).
inline constexpr DeviceIdentity ELM3002{0x00000002, 0x50216DA9, 2, "ELM3002"};
inline constexpr DeviceIdentity ELM3004{0x00000002, 0x50216DC9, 4, "ELM3004"};
inline constexpr DeviceIdentity ELM3102{0x00000002, 0x502173E9, 2, "ELM3102"};
inline constexpr DeviceIdentity ELM3104{0x00000002, 0x50217409, 4, "ELM3104"};
inline constexpr DeviceIdentity ELM3142{0x00000002, 0x50217669, 2, "ELM3142"};
inline constexpr DeviceIdentity ELM3144{0x00000002, 0x50217689, 4, "ELM3144"};
inline constexpr DeviceIdentity ELM3146{0x00000002, 0x502176A9, 6, "ELM3146"};
inline constexpr DeviceIdentity ELM3148{0x00000002, 0x502176C9, 8, "ELM3148"};
inline constexpr DeviceIdentity ELM3244{0x00000002, 0x50217CC9, 4, "ELM3244"};
inline constexpr DeviceIdentity ELM3246{0x00000002, 0x50217CE9, 6, "ELM3246"};
inline constexpr DeviceIdentity ELM3344{0x00000002, 0x50218309, 4, "ELM3344"};
inline constexpr DeviceIdentity ELM3348{0x00000002, 0x50218349, 8, "ELM3348"};
inline constexpr DeviceIdentity ELM3502{0x00000002, 0x50218CE9, 2, "ELM3502"};
inline constexpr DeviceIdentity ELM3504{0x00000002, 0x50218D09, 4, "ELM3504"};
inline constexpr DeviceIdentity ELM3602{0x00000002, 0x50219329, 2, "ELM3602"};
inline constexpr DeviceIdentity ELM3604{0x00000002, 0x50219349, 4, "ELM3604"};
inline constexpr DeviceIdentity ELM3702{0x00000002, 0x50219969, 2, "ELM3702"};
inline constexpr DeviceIdentity ELM3704{0x00000002, 0x50219989, 4, "ELM3704"};

/// Every known oversampling terminal.
inline constexpr std::array kOversamplingTerminals{
    EL3773, EL3783,
    ELM3002, ELM3004, ELM3102, ELM3104, ELM3142, ELM3144, ELM3146,
    ELM3148, ELM3244, ELM3246, ELM3344, ELM3348, ELM3502, ELM3504,
    ELM3602, ELM3604, ELM3702, ELM3704,
};

} // namespace Devices

// ============================================================================
// OversamplingTerminal
// ============================================================================

class OversamplingTerminal : public TerminalBase {
public:
    static constexpr size_t kMaxChannels  = 16;
    static constexpr size_t kMaxSamples   = 1024;   ///< per channel/cycle

    // -- Construction / factories ----------------------------------------------

    OversamplingTerminal(Master& master, uint16_t slave_index,
                         const DeviceIdentity& identity);
    OversamplingTerminal(Master& master, const DiscoveredSlave& slave,
                         const DeviceIdentity& identity);

    ~OversamplingTerminal() override;

    OversamplingTerminal(OversamplingTerminal&&) noexcept            = default;
    OversamplingTerminal& operator=(OversamplingTerminal&&) noexcept = default;
    OversamplingTerminal(const OversamplingTerminal&)                = delete;
    OversamplingTerminal& operator=(const OversamplingTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<OversamplingTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);
    static Result<OversamplingTerminal> findFirst(
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

    // -- Samples -----------------------------------------------------------------------

    /// Number of resolved channels (0 before prepare()).
    size_t channels() const { return channels_.size(); }

    /// Samples per channel per cycle (the oversampling factor).
    size_t samplesPerCycle(size_t ch) const;

    /// Bit width of the channel's sample entries (16/24/32).
    uint8_t sampleBits(size_t ch) const;

    /**
     * @brief Sample `i` of channel `ch` from the latest cycle,
     *        sign-extended to int32.
     */
    int32_t sample(size_t ch, size_t i) const;

    /// Copy the whole channel's sample block into `dst` (sign-extended).
    /// @return samples written.
    size_t samples(size_t ch, std::span<int32_t> dst) const;

    /// Per-channel status word (16 bit), 0xffff when absent.
    uint16_t statusWord(size_t ch) const;
    bool hasStatus(size_t ch) const;

private:
    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    /// One resolved sample field inside the input image.
    struct Sample {
        uint16_t byte_off = 0;
        uint8_t  bit_len  = 0;
    };
    struct Channel {
        std::vector<Sample> samples;
        int16_t status_off = -1;
    };

    std::vector<Channel> channels_;
};

} // namespace Beckhoff

} // namespace EtherCAT
