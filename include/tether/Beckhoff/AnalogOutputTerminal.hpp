/**
 * @file AnalogOutputTerminal.hpp
 * @brief Generic driver for Beckhoff-style analog-output terminals
 *
 * AnalogOutputTerminal drives any terminal whose process outputs live on
 * SM2 as a sequence of per-channel RxPDOs carrying one 16- or 32-bit
 * value entry — the "SM0/SM1 = mailbox, SM2 = Outputs" family of the
 * Beckhoff EL4xxx ESI (see the Devices registry below).  The per-channel
 * layout is derived from the SII RxPDO list at bring-up, so one
 * implementation covers 1-, 2-, 4-, 8-channel terminals and both the
 * 16-bit (EL40xx/EL41xx) and 32-bit (EL407x) value widths.  Value object
 * indices differ by revision (0x7xxx, 0x6411, 0x300x) — the driver keys
 * on entry width, not index.
 *
 * Terminals that additionally enable an SM3 process-input channel
 * (EL407x status words, EL4374's two analog inputs) get that region
 * mapped as well so the cyclic exchange's working counter stays
 * satisfied; the bytes are readable via rawInput() for advanced use.
 *
 * The mailbox is fully configured from SII (these are CoE devices), but
 * no SDO traffic is performed: the default PDO assignment already
 * covers all channels.
 *
 * @code
 *   // Standalone — position addressing, one frame per terminal:
 *   auto el = AnalogOutputTerminal::findFirst(master, Devices::EL4134);
 *   el->start();
 *   el->setValue(0, 16384);              // ~+5 V on a ±10 V terminal
 *
 *   // Chained — one shared logical address space, one LRW frame/cycle:
 *   MultiAnalogOutputTerminal<> outs(master);
 *   outs.detect();
 *   outs.start();
 *   outs.setValue(9, -10000);            // flat channel index
 * @endcode
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 * All values are raw field-bus integers — the terminal's configured
 * scaling maps them to volts/amps (±32767 = full scale, e.g. ±10 V).
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/IAnalogOutputTerminal.hpp"
#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

// ============================================================================
// Known analog-output terminals (from the Beckhoff EL4xxx ESI)
// ============================================================================
// Every entry below has the AnalogOutputTerminal shape: SM0/SM1 mailbox,
// an enabled SM2 "Outputs" channel whose default-assigned RxPDOs carry
// one 16- or 32-bit value entry per channel.  num_bits holds the channel
// count.  Product codes verified against Beckhoff EL4xxx.xml.
//
// None of these have been exercised on real hardware yet — supported,
// not verified yet.

namespace Devices {

// -- EL40xx — voltage outputs (0-10 V / ±10 V), 16-bit -------------------------
inline constexpr DeviceIdentity EL4001{0x00000002, 0x0FA13052, 1, "EL4001"};
inline constexpr DeviceIdentity EL4002{0x00000002, 0x0FA23052, 2, "EL4002"};
inline constexpr DeviceIdentity EL4004{0x00000002, 0x0FA43052, 4, "EL4004"};
inline constexpr DeviceIdentity EL4008{0x00000002, 0x0FA83052, 8, "EL4008"};
inline constexpr DeviceIdentity EL4011{0x00000002, 0x0FAB3052, 1, "EL4011"};
inline constexpr DeviceIdentity EL4012{0x00000002, 0x0FAC3052, 2, "EL4012"};
inline constexpr DeviceIdentity EL4014{0x00000002, 0x0FAE3052, 4, "EL4014"};
inline constexpr DeviceIdentity EL4018{0x00000002, 0x0FB23052, 8, "EL4018"};
inline constexpr DeviceIdentity EL4021{0x00000002, 0x0FB53052, 1, "EL4021"};
inline constexpr DeviceIdentity EL4022{0x00000002, 0x0FB63052, 2, "EL4022"};
inline constexpr DeviceIdentity EL4024{0x00000002, 0x0FB83052, 4, "EL4024"};
inline constexpr DeviceIdentity EL4028{0x00000002, 0x0FBC3052, 8, "EL4028"};
inline constexpr DeviceIdentity EL4031{0x00000002, 0x0FBF3052, 1, "EL4031"};
inline constexpr DeviceIdentity EL4032{0x00000002, 0x0FC03052, 2, "EL4032"};
inline constexpr DeviceIdentity EL4034{0x00000002, 0x0FC23052, 4, "EL4034"};
inline constexpr DeviceIdentity EL4038{0x00000002, 0x0FC63052, 8, "EL4038"};

// -- EL407x — combined voltage/current outputs, 32-bit, +SM3 status ------------
inline constexpr DeviceIdentity EL4072{0x00000002, 0x0FE83052, 2, "EL4072"};
inline constexpr DeviceIdentity EL4074{0x00000002, 0x0FEA3052, 4, "EL4074"};
inline constexpr DeviceIdentity EL4078{0x00000002, 0x0FEE3052, 8, "EL4078"};

// -- EL41xx — current outputs (0/4-20 mA), 16-bit -------------------------------
inline constexpr DeviceIdentity EL4102{0x00000002, 0x10063052, 2, "EL4102"};
inline constexpr DeviceIdentity EL4104{0x00000002, 0x10083052, 4, "EL4104"};
inline constexpr DeviceIdentity EL4112{0x00000002, 0x10103052, 2, "EL4112"};
inline constexpr DeviceIdentity EL4114{0x00000002, 0x10123052, 4, "EL4114"};
inline constexpr DeviceIdentity EL4122{0x00000002, 0x101A3052, 2, "EL4122"};
inline constexpr DeviceIdentity EL4124{0x00000002, 0x101C3052, 4, "EL4124"};
inline constexpr DeviceIdentity EL4132{0x00000002, 0x10243052, 2, "EL4132"};
inline constexpr DeviceIdentity EL4134{0x00000002, 0x10263052, 4, "EL4134"};

// -- EL43xx — combined 2×AI + 2×AO ---------------------------------------------
inline constexpr DeviceIdentity EL4374{0x00000002, 0x11163052, 2, "EL4374"};

// -- EP/ER/EJ variants — same electronics, value-only SM2 images ----------------
// All ESI-verified for shape; none verified on hardware yet.
inline constexpr DeviceIdentity EJ4002{0x00000002, 0x0FA22852, 2, "EJ4002"};
inline constexpr DeviceIdentity EJ4004{0x00000002, 0x0FA42852, 4, "EJ4004"};
inline constexpr DeviceIdentity EJ4008{0x00000002, 0x0FA82852, 8, "EJ4008"};
inline constexpr DeviceIdentity EJ4018{0x00000002, 0x0FB22852, 8, "EJ4018"};
inline constexpr DeviceIdentity EJ4024{0x00000002, 0x0FB82852, 4, "EJ4024"};
inline constexpr DeviceIdentity EJ4132{0x00000002, 0x10242852, 2, "EJ4132"};
inline constexpr DeviceIdentity EJ4134{0x00000002, 0x10262852, 4, "EJ4134"};

inline constexpr DeviceIdentity EP4174{0x00000002, 0x104E4052, 4, "EP4174"};
inline constexpr DeviceIdentity EP4374{0x00000002, 0x11164052, 4, "EP4374"};
inline constexpr DeviceIdentity ER4174{0x00000002, 0x104E4852, 4, "ER4174"};
inline constexpr DeviceIdentity ER4374{0x00000002, 0x11164852, 4, "ER4374"};

// -- EL47xx — oversampling-capable analog outputs ------------------------------------
// Default mapping is plain value-per-channel (no oversampling PDOs).
inline constexpr DeviceIdentity EL4712{0x00000002, 0x12683052, 2, "EL4712"};
inline constexpr DeviceIdentity EL4732{0x00000002, 0x127C3052, 2, "EL4732"};

// -- EP/EPP analog output boxes --------------------------------------------------------
// EPP entries are module-style devices (no explicit PDO list in the
// ESI); the SII resolves the same value-per-channel layout.
inline constexpr DeviceIdentity EP4304{0x00000002, 0x10D04052, 4, "EP4304"};
inline constexpr DeviceIdentity EP4314{0x00000002, 0x10DA4052, 4, "EP4314"};
inline constexpr DeviceIdentity EPP4304{0x00000002, 0x6476D309, 4, "EPP4304"};
inline constexpr DeviceIdentity EPP4314{0x00000002, 0x6476D3A9, 4, "EPP4314"};
inline constexpr DeviceIdentity EPP4374{0x00000002, 0x6476D769, 4, "EPP4374"};

// -- Ex-i intrinsically safe analog outputs ---------------------------------------------
inline constexpr DeviceIdentity ELX4154{0x00000002, 0x970C01A9, 4, "ELX4154"};
inline constexpr DeviceIdentity ELX4181{0x00000002, 0x970C0359, 1, "ELX4181"};

/// Every known analog-output terminal — the default detection set used by
/// MultiAnalogOutputTerminal::detect().  All entries verified against the
/// ESI for shape (mailbox + SM2 outputs + one ≤32-bit value entry per
/// channel PDO); none verified on hardware yet.
inline constexpr std::array kAnalogOutputTerminals{
    EL4001, EL4002, EL4004, EL4008, EL4011, EL4012, EL4014, EL4018,
    EL4021, EL4022, EL4024, EL4028, EL4031, EL4032, EL4034, EL4038,
    EL4072, EL4074, EL4078,
    EL4102, EL4104, EL4112, EL4114, EL4122, EL4124, EL4132, EL4134,
    EL4374,
    EJ4002, EJ4004, EJ4008, EJ4018, EJ4024, EJ4132, EJ4134,
    EP4174, EP4374, ER4174, ER4374,
    EL4712, EL4732,
    EP4304, EP4314, EPP4304, EPP4314, EPP4374,
    ELX4154, ELX4181,
};

} // namespace Devices

// ============================================================================
// AnalogOutputTerminal — generic SII-driven analog output driver
// ============================================================================

class AnalogOutputTerminal : public TerminalBase,
                             public IAnalogOutputTerminal {
public:
    /// Maximum channels supported per device.
    static constexpr size_t kMaxChannels = 64;

    /// Maximum process-image bytes per device (either direction).
    static constexpr size_t kMaxImageBytes = 512;

    // -- Construction ----------------------------------------------------------

    /**
     * @brief Bind the driver to a slave by bus position.
     * @param master       Started master (must outlive this object).
     * @param slave_index  Zero-based bus position of the terminal.
     * @param identity     Expected device identity (vendor/product).
     */
    AnalogOutputTerminal(Master& master, uint16_t slave_index,
                         const DeviceIdentity& identity);

    /**
     * @brief Bind the driver to an already-discovered slave — reuses the
     *        DiscoveredSlave's SII data so configure() need not re-read
     *        the EEPROM.  Identity is checked in configure().
     */
    AnalogOutputTerminal(Master& master, const DiscoveredSlave& slave,
                         const DeviceIdentity& identity);

    ~AnalogOutputTerminal() override;

    AnalogOutputTerminal(AnalogOutputTerminal&&) noexcept            = default;
    AnalogOutputTerminal& operator=(AnalogOutputTerminal&&) noexcept = default;
    AnalogOutputTerminal(const AnalogOutputTerminal&)                = delete;
    AnalogOutputTerminal& operator=(const AnalogOutputTerminal&)     = delete;

    // -- Factories ---------------------------------------------------------------

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /**
     * @brief Scan the bus and return a driver bound to the first slave
     *        matching `identity`.  Runs a shallow vendor/product discovery.
     * @return The driver, or Error::NoDeviceFound.
     */
    static Result<AnalogOutputTerminal> findFirst(Master& master,
                                                  const DeviceIdentity& identity);

    /// Like findFirst(master, identity) but reuses an existing scan.
    static Result<AnalogOutputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Standalone bring-up (position addressing, no FMMU) ---------------------

    /**
     * @brief Configure the terminal up to SAFE-OP using position addressing:
     *        SII resolution, mailbox config, process-SM registers, PRE-OP,
     *        RxPDO registration, SAFE-OP.  Idempotent.
     */
    Result<> configure();

    /**
     * @brief Bring the terminal fully up: configure() then OP.  With the
     *        default StartOptions this also starts the master's realtime
     *        loop — after start() returns, setValue() writes reach the
     *        terminal every cycle.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    // -- IAnalogOutputTerminal ----------------------------------------------------

    uint16_t slaveIndex() const override {
        return TerminalBase::slaveIndex();
    }
    size_t   channelCount() const override { return channels_.size(); }
    const char* deviceName() const override {
        return TerminalBase::deviceName();
    }

    void    setValue(size_t channel, int32_t value) override;
    int32_t value(size_t channel) const override;
    void    allZero() override;

    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Status --------------------------------------------------------------------

    /// Bit width of channel `ch`'s value field (16 or 32; 0 when the
    /// layout is not resolved yet or `ch` is out of range).
    size_t channelBits(size_t ch) const;

private:
    /// Per-channel layout resolved from the SII RxPDO list.
    struct ChannelLayout {
        uint16_t value_off;   ///< byte offset of the value in the SM2 image
        uint8_t  value_bits;  ///< value width: 16 or 32
    };

    /// Resolve the per-channel value offsets from the bound discovery
    /// data / SII (SM regions themselves come from the base).
    bool resolveLayout();

    /// Shared bring-up: SII, identity check, mailbox config, SM registers,
    /// PRE-OP, PDO buffer registration in the given address mode.
    /// Ends before SAFE-OP.
    Result<> prepare(PDO::PDOAddressMode mode);

    std::vector<ChannelLayout> channels_;
};

} // namespace Beckhoff

} // namespace EtherCAT
