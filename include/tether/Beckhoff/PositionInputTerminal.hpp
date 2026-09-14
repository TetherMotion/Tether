/**
 * @file PositionInputTerminal.hpp
 * @brief Generic driver for Beckhoff position/encoder input terminals
 *        (the EL5xxx family)
 *
 * PositionInputTerminal reads any EL5xxx terminal whose process inputs
 * live on an enabled process-input SM as a sequence of per-channel PDOs
 * carrying >=16-bit value entries — SSI (EL500x), SinCos (EL5021),
 * EnDat/BiSS (EL503x/EL5042), displacement (EL5072) and incremental
 * counters (EL51xx).  The per-channel layout — position offset/width,
 * latch fields, status prefix — is derived from the SII TxPDO list at
 * bring-up, so one implementation covers 16-bit compact counter PDOs,
 * 32-bit counters with latch, and 64-bit BiSS/EnDat position words.
 *
 * Each channel's value entries are exposed in PDO order:
 *   value(ch,0) = position/counter     value(ch,1) = first latch
 * Terminals that additionally enable a process-output SM (counter preset
 * words, ENC control) get that region mapped so the cyclic exchange's
 * working counter stays satisfied; rawOutput() exposes it for drivers
 * that need the preset/control fields.
 *
 * @code
 *   auto enc = PositionInputTerminal::findFirst(master, Devices::EL5101);
 *   enc->start();
 *   int64_t pos = enc->position(0);     // signed counter
 *   int64_t lat = enc->latch(0);        // latch value (0 if none)
 *
 *   MultiPositionInputTerminal<> encs(master);
 *   encs.detect();
 *   encs.start();
 *   int64_t p = encs.position(2);       // flat channel index
 * @endcode
 *
 * Channel numbering is 0-based.  All values are raw field-bus integers;
 * unit scaling (counts/revolution, micrometres, ...) is left to the
 * application.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/IPositionInputTerminal.hpp"
#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known position/encoder terminals (from the Beckhoff EL5xxx ESIs)
// ============================================================================
// Every entry below has the PositionInputTerminal shape: mailbox +
// enabled process-input SM whose default TxPDOs carry one or more
// >=16-bit value entries per channel.  num_bits holds the physical
// channel count (used by the no-SII fallback layout).  Product codes
// verified against Beckhoff EL5xxx.xml.
//
// None of these have been exercised on real hardware yet — supported,
// not verified yet.

namespace Devices {

// -- SSI absolute encoders ---------------------------------------------------
inline constexpr DeviceIdentity EL5001{0x00000002, 0x13893052, 1, "EL5001"};
inline constexpr DeviceIdentity EL5002{0x00000002, 0x138A3052, 2, "EL5002"};

// -- SinCos / EnDat / BiSS / displacement -------------------------------------
inline constexpr DeviceIdentity EL5021{0x00000002, 0x139D3052, 1, "EL5021"};
inline constexpr DeviceIdentity EL5031{0x00000002, 0x13A73052, 1, "EL5031"};
inline constexpr DeviceIdentity EL5032{0x00000002, 0x13A83052, 2, "EL5032"};
inline constexpr DeviceIdentity EL5042{0x00000002, 0x13B23052, 2, "EL5042"};
inline constexpr DeviceIdentity EL5072{0x00000002, 0x13D03052, 2, "EL5072"};

// -- Incremental encoders ------------------------------------------------------
inline constexpr DeviceIdentity EL5101{0x00000002, 0x13ED3052, 1, "EL5101"};
inline constexpr DeviceIdentity EL5102{0x00000002, 0x13EE3052, 2, "EL5102"};
inline constexpr DeviceIdentity EL5112{0x00000002, 0x13F83052, 1, "EL5112"};
inline constexpr DeviceIdentity EL5122{0x00000002, 0x14023052, 2, "EL5122"};
inline constexpr DeviceIdentity EL5131{0x00000002, 0x140B3052, 1, "EL5131"};
inline constexpr DeviceIdentity EL5151{0x00000002, 0x141F3052, 1, "EL5151"};
inline constexpr DeviceIdentity EL5152{0x00000002, 0x14203052, 2, "EL5152"};

/// Every known position-input terminal — the default detection set used
/// by MultiPositionInputTerminal::detect().  All entries verified against
/// the ESIs for shape (mailbox + process-input SM + >=16-bit value
/// entries); none verified on hardware yet.

// -- EP/ER/EJ variants — same electronics under 0x2852/0x4052/0x4852 ------------
inline constexpr DeviceIdentity EJ5002{0x00000002, 0x138A2852, 2, "EJ5002"};
inline constexpr DeviceIdentity EJ5021{0x00000002, 0x139D2852, 1, "EJ5021"};
inline constexpr DeviceIdentity EJ5042{0x00000002, 0x13B22852, 2, "EJ5042"};
inline constexpr DeviceIdentity EJ5101{0x00000002, 0x13ED2852, 1, "EJ5101"};
inline constexpr DeviceIdentity EJ5112{0x00000002, 0x13F82852, 2, "EJ5112"};
inline constexpr DeviceIdentity EJ5151{0x00000002, 0x141F2852, 1, "EJ5151"};
inline constexpr DeviceIdentity EJ5152{0x00000002, 0x14202852, 2, "EJ5152"};
inline constexpr DeviceIdentity EP5001{0x00000002, 0x13894052, 1, "EP5001"};
inline constexpr DeviceIdentity EP5101{0x00000002, 0x13ED4052, 1, "EP5101"};
inline constexpr DeviceIdentity EP5151{0x00000002, 0x141F4052, 1, "EP5151"};
inline constexpr DeviceIdentity ER5101{0x00000002, 0x13ED4852, 1, "ER5101"};
inline constexpr DeviceIdentity ER5151{0x00000002, 0x141F4852, 1, "ER5151"};

// -- EL15xx up/down counters — counter value + status in, set-counter out --------
// Same value-channel shape as the encoder terminals (32-bit counter +
// latch); the RxPDO (set counter value + control bits) is mapped as the
// output scratch buffer.
inline constexpr DeviceIdentity EL1502{0x00000002, 0x05DE3052, 2, "EL1502"};
inline constexpr DeviceIdentity EL1512{0x00000002, 0x05E83052, 2, "EL1512"};

// -- EPP/ELX encoder boxes ----------------------------------------------------------
// Module-style ESIs (no explicit PDO list); SII resolves the same
// encoder layout on hardware.
inline constexpr DeviceIdentity EPP5001{0x00000002, 0x6476FE99, 1, "EPP5001"};
inline constexpr DeviceIdentity EPP5101{0x00000002, 0x647704D9, 1, "EPP5101"};
inline constexpr DeviceIdentity EPP5151{0x00000002, 0x647707F9, 1, "EPP5151"};
inline constexpr DeviceIdentity ELX5151{0x00000002, 0x970C3FF9, 1, "ELX5151"};

inline constexpr std::array kPositionInputTerminals{
    EL5001, EL5002, EL5021, EL5031, EL5032, EL5042, EL5072,
    EL5101, EL5102, EL5112, EL5122, EL5131, EL5151, EL5152,
    EJ5002, EJ5021, EJ5042, EJ5101, EJ5112, EJ5151, EJ5152,
    EP5001, EP5101, EP5151, ER5101, ER5151,
    EL1502, EL1512, EPP5001, EPP5101, EPP5151, ELX5151,
};

} // namespace Devices

// ============================================================================
// PositionInputTerminal — generic SII-driven position input driver
// ============================================================================

class PositionInputTerminal : public TerminalBase,
                              public IPositionInputTerminal {
public:
    /// Maximum channels supported per device.
    static constexpr size_t kMaxChannels = 16;

    /// Maximum process-image bytes per device (input or output side).
    static constexpr size_t kMaxImageBytes = 512;

    // -- Construction ----------------------------------------------------------

    /// Bind by bus position; SII data is read lazily in configure().
    PositionInputTerminal(Master& master, uint16_t slave_index,
                          const DeviceIdentity& identity);

    /// Bind to an already-discovered slave — reuses its SII data.
    PositionInputTerminal(Master& master, const DiscoveredSlave& slave,
                          const DeviceIdentity& identity);

    ~PositionInputTerminal() override;

    PositionInputTerminal(PositionInputTerminal&&) noexcept            = default;
    PositionInputTerminal& operator=(PositionInputTerminal&&) noexcept = default;
    PositionInputTerminal(const PositionInputTerminal&)                = delete;
    PositionInputTerminal& operator=(const PositionInputTerminal&)     = delete;

    // -- Factories ---------------------------------------------------------------

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus and return a driver bound to the first slave matching
    /// `identity` (shallow vendor/product discovery).
    static Result<PositionInputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);

    /// Like findFirst(master, identity) but reuses an existing scan.
    static Result<PositionInputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Standalone bring-up ------------------------------------------------------

    /// Configure the terminal up to SAFE-OP using position addressing.
    Result<> configure();

    /// configure() then OP (+ the managed realtime loop by default).
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    // -- IPositionInputTerminal ----------------------------------------------------

    uint16_t slaveIndex() const override {
        return TerminalBase::slaveIndex();
    }
    size_t channelCount() const override { return channels_.size(); }
    const char* deviceName() const override {
        return TerminalBase::deviceName();
    }

    size_t   valueCount(size_t ch) const override;
    int64_t  value(size_t ch, size_t i) const override;
    uint64_t rawValue(size_t ch, size_t i) const override;
    uint16_t status(size_t ch) const override;
    bool     hasStatus(size_t ch) const override;
    size_t   channelBits(size_t ch) const override;

    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Extra queries ---------------------------------------------------------------

    /// Object index/subindex of value field `i` of channel `ch`
    /// (0/0 when out of range) — identifies what a value *is* on devices
    /// carrying several (counter vs. latch vs. timestamp).
    std::pair<uint16_t, uint8_t> valueObject(size_t ch, size_t i) const;

private:
    /// Per-channel layout resolved from the SII TxPDO list.
    struct ChannelLayout {
        /// All >=16-bit value entries in PDO order (position first).
        struct Field {
            uint16_t byte_off;
            uint8_t  bit_len;   ///< 16…64
            uint16_t index;
            uint8_t  subindex;
        };
        std::vector<Field> fields;
        int16_t status_off = -1;
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    std::vector<ChannelLayout> channels_;
};

} // namespace Beckhoff

} // namespace EtherCAT
