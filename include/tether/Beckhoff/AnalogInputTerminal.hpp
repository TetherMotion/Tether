/**
 * @file AnalogInputTerminal.hpp
 * @brief Generic driver for Beckhoff-style analog-input terminals
 *
 * AnalogInputTerminal reads any terminal whose process inputs live on
 * SM3 as a sequence of per-channel PDOs carrying a 16- or 32-bit value
 * entry — the "SM0/SM1 = mailbox, SM3 = Inputs" family of the Beckhoff
 * EL3xxx ESI (see the Devices registry below).  The per-channel layout
 * (value offset, width, status offset) is derived from the SII TxPDO
 * list at bring-up, so one implementation covers 1-, 2-, 4-, 8-channel
 * terminals in both the 4-byte standard (status + int16) and the 6-byte
 * high-resolution (status + int32) shapes.  Channels whose default PDO
 * carries no status field report hasStatus() == false.
 *
 * Terminals that additionally enable an SM2 process-output channel
 * (thermocouple cold-junction compensation words, EL3356/EL3681 control
 * words, ...) get that region mapped as well so the cyclic exchange's
 * working counter stays satisfied; the bytes are not interpreted.
 *
 * The mailbox is fully configured from SII (these are CoE devices), but
 * no SDO traffic is performed: the default PDO assignment already
 * covers all channels.
 *
 * @code
 *   // Standalone — position addressing, one frame per terminal:
 *   auto el = AnalogInputTerminal::findFirst(master, Devices::EL3104);
 *   el->start();
 *   int32_t raw = el->value(0);          // ±32767 = ±10 V
 *
 *   // Chained — one shared logical address space, one LRW frame/cycle:
 *   MultiAnalogInputTerminal<> ins(master);
 *   ins.detect();                        // every known analog input terminal
 *   ins.start();
 *   int32_t v = ins.value(5);            // flat channel index
 * @endcode
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 * All values are raw field-bus integers — applying the terminal's
 * configured scaling (±10 V → ±32767, 0-20 mA → 0-32767, 0.1 °C/digit
 * for RTD/TC, ...) is left to the application.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/IAnalogInputTerminal.hpp"
#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode, kMaxPDOSlaves
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

// ============================================================================
// Known analog-input terminals (from the Beckhoff EL3xxx ESIs)
// ============================================================================
// Every entry below has the AnalogInputTerminal shape: SM0/SM1 mailbox,
// an enabled SM3 "Inputs" channel whose default-assigned TxPDOs carry
// one 16- or 32-bit value entry per channel.  num_bits holds the channel
// count.  Product codes verified against Beckhoff EL30xx–EL3xxx.xml.
//
// None of these have been exercised on real hardware yet — supported,
// not verified yet.

namespace Devices {

// -- EL30xx — single-ended analog inputs -------------------------------------
inline constexpr DeviceIdentity EL3001{0x00000002, 0x0BB93052, 1, "EL3001"};
inline constexpr DeviceIdentity EL3002{0x00000002, 0x0BBA3052, 2, "EL3002"};
inline constexpr DeviceIdentity EL3004{0x00000002, 0x0BBC3052, 4, "EL3004"};
inline constexpr DeviceIdentity EL3008{0x00000002, 0x0BC03052, 8, "EL3008"};
inline constexpr DeviceIdentity EL3011{0x00000002, 0x0BC33052, 1, "EL3011"};
inline constexpr DeviceIdentity EL3012{0x00000002, 0x0BC43052, 2, "EL3012"};
inline constexpr DeviceIdentity EL3014{0x00000002, 0x0BC63052, 4, "EL3014"};
inline constexpr DeviceIdentity EL3021{0x00000002, 0x0BCD3052, 1, "EL3021"};
inline constexpr DeviceIdentity EL3022{0x00000002, 0x0BCE3052, 2, "EL3022"};
inline constexpr DeviceIdentity EL3024{0x00000002, 0x0BD03052, 4, "EL3024"};
inline constexpr DeviceIdentity EL3041{0x00000002, 0x0BE13052, 1, "EL3041"};
inline constexpr DeviceIdentity EL3042{0x00000002, 0x0BE23052, 2, "EL3042"};
inline constexpr DeviceIdentity EL3044{0x00000002, 0x0BE43052, 4, "EL3044"};
inline constexpr DeviceIdentity EL3048{0x00000002, 0x0BE83052, 8, "EL3048"};
inline constexpr DeviceIdentity EL3051{0x00000002, 0x0BEB3052, 1, "EL3051"};
inline constexpr DeviceIdentity EL3052{0x00000002, 0x0BEC3052, 2, "EL3052"};
inline constexpr DeviceIdentity EL3054{0x00000002, 0x0BEE3052, 4, "EL3054"};
inline constexpr DeviceIdentity EL3058{0x00000002, 0x0BF23052, 8, "EL3058"};
inline constexpr DeviceIdentity EL3061{0x00000002, 0x0BF53052, 1, "EL3061"};
inline constexpr DeviceIdentity EL3062{0x00000002, 0x0BF63052, 2, "EL3062"};
inline constexpr DeviceIdentity EL3064{0x00000002, 0x0BF83052, 4, "EL3064"};
inline constexpr DeviceIdentity EL3068{0x00000002, 0x0BFC3052, 8, "EL3068"};
inline constexpr DeviceIdentity EL3072{0x00000002, 0x0C003052, 2, "EL3072"};
inline constexpr DeviceIdentity EL3074{0x00000002, 0x0C023052, 4, "EL3074"};
inline constexpr DeviceIdentity EL3078{0x00000002, 0x0C063052, 8, "EL3078"};

// -- EL31xx — differential analog inputs --------------------------------------
inline constexpr DeviceIdentity EL3101{0x00000002, 0x0C1D3052, 1, "EL3101"};
inline constexpr DeviceIdentity EL3102{0x00000002, 0x0C1E3052, 2, "EL3102"};
inline constexpr DeviceIdentity EL3104{0x00000002, 0x0C203052, 4, "EL3104"};
inline constexpr DeviceIdentity EL3111{0x00000002, 0x0C273052, 1, "EL3111"};
inline constexpr DeviceIdentity EL3112{0x00000002, 0x0C283052, 2, "EL3112"};
inline constexpr DeviceIdentity EL3114{0x00000002, 0x0C2A3052, 4, "EL3114"};
inline constexpr DeviceIdentity EL3121{0x00000002, 0x0C313052, 1, "EL3121"};
inline constexpr DeviceIdentity EL3122{0x00000002, 0x0C323052, 2, "EL3122"};
inline constexpr DeviceIdentity EL3124{0x00000002, 0x0C343052, 4, "EL3124"};
inline constexpr DeviceIdentity EL3141{0x00000002, 0x0C453052, 1, "EL3141"};
inline constexpr DeviceIdentity EL3142{0x00000002, 0x0C463052, 2, "EL3142"};
inline constexpr DeviceIdentity EL3144{0x00000002, 0x0C483052, 4, "EL3144"};
inline constexpr DeviceIdentity EL3151{0x00000002, 0x0C4F3052, 1, "EL3151"};
inline constexpr DeviceIdentity EL3152{0x00000002, 0x0C503052, 2, "EL3152"};
inline constexpr DeviceIdentity EL3154{0x00000002, 0x0C523052, 4, "EL3154"};
inline constexpr DeviceIdentity EL3161{0x00000002, 0x0C593052, 1, "EL3161"};
inline constexpr DeviceIdentity EL3162{0x00000002, 0x0C5A3052, 2, "EL3162"};
inline constexpr DeviceIdentity EL3164{0x00000002, 0x0C5C3052, 4, "EL3164"};
inline constexpr DeviceIdentity EL3172{0x00000002, 0x0C643052, 2, "EL3172"};
inline constexpr DeviceIdentity EL3174{0x00000002, 0x0C663052, 4, "EL3174"};
inline constexpr DeviceIdentity EL3182{0x00000002, 0x0C6E3052, 2, "EL3182"};
inline constexpr DeviceIdentity EL3184{0x00000002, 0x0C703052, 4, "EL3184"};

// -- EL32xx — RTD / potentiometer ---------------------------------------------
inline constexpr DeviceIdentity EL3201{0x00000002, 0x0C813052, 1, "EL3201"};
inline constexpr DeviceIdentity EL3202{0x00000002, 0x0C823052, 2, "EL3202"};
inline constexpr DeviceIdentity EL3204{0x00000002, 0x0C843052, 4, "EL3204"};
inline constexpr DeviceIdentity EL3208{0x00000002, 0x0C883052, 8, "EL3208"};
inline constexpr DeviceIdentity EL3214{0x00000002, 0x0C8E3052, 4, "EL3214"};
inline constexpr DeviceIdentity EL3218{0x00000002, 0x0C923052, 8, "EL3218"};
inline constexpr DeviceIdentity EL3255{0x00000002, 0x0CB73052, 5, "EL3255"};

// -- EL33xx — thermocouple / resistor bridge ----------------------------------
// EL3311/3312/3314 also enable an SM2 output channel (cold-junction
// compensation words) — mapped and left untouched by this driver.
inline constexpr DeviceIdentity EL3311{0x00000002, 0x0CEF3052, 1, "EL3311"};
inline constexpr DeviceIdentity EL3312{0x00000002, 0x0CF03052, 2, "EL3312"};
inline constexpr DeviceIdentity EL3314{0x00000002, 0x0CF23052, 4, "EL3314"};
inline constexpr DeviceIdentity EL3318{0x00000002, 0x0CF63052, 8, "EL3318"};
inline constexpr DeviceIdentity EL3351{0x00000002, 0x0D173052, 2, "EL3351"};
inline constexpr DeviceIdentity EL3356{0x00000002, 0x0D1C3052, 1, "EL3356"};

// -- EL34xx — distributed power measurement (clean per-channel layout) --------
inline constexpr DeviceIdentity EL3444{0x00000002, 0x0D743052, 4, "EL3444"};
inline constexpr DeviceIdentity EL3446{0x00000002, 0x0D763052, 6, "EL3446"};

// -- EL36xx — high-resolution / specialty analog inputs ------------------------
inline constexpr DeviceIdentity EL3602{0x00000002, 0x0E123052, 2, "EL3602"};
inline constexpr DeviceIdentity EL3611{0x00000002, 0x0E1B3052, 1, "EL3611"};
inline constexpr DeviceIdentity EL3612{0x00000002, 0x0E1C3052, 2, "EL3612"};
inline constexpr DeviceIdentity EL3621{0x00000002, 0x0E253052, 1, "EL3621"};
inline constexpr DeviceIdentity EL3681{0x00000002, 0x0E613052, 1, "EL3681"};
inline constexpr DeviceIdentity EL3692{0x00000002, 0x0E6C3052, 2, "EL3692"};

// -- EL37xx — multi-function measurement ----------------------------------------
inline constexpr DeviceIdentity EL3751{0x00000002, 0x0EA73052, 1, "EL3751"};

// -- EP/ER/EJ variants — same electronics, value+status SM3 images -------------
// Product-code suffixes: 0x4052=EP box, 0x4852=ER box, 0x2852=EJ module.
// All ESI-verified for shape; none verified on hardware yet.
inline constexpr DeviceIdentity EJ3004{0x00000002, 0x0BBC2852, 4, "EJ3004"};
inline constexpr DeviceIdentity EJ3008{0x00000002, 0x0BC02852, 8, "EJ3008"};
inline constexpr DeviceIdentity EJ3048{0x00000002, 0x0BE82852, 8, "EJ3048"};
inline constexpr DeviceIdentity EJ3058{0x00000002, 0x0BF22852, 8, "EJ3058"};
inline constexpr DeviceIdentity EJ3068{0x00000002, 0x0BFC2852, 8, "EJ3068"};
inline constexpr DeviceIdentity EJ3104{0x00000002, 0x0C202852, 4, "EJ3104"};
inline constexpr DeviceIdentity EJ3108{0x00000002, 0x0C242852, 8, "EJ3108"};
inline constexpr DeviceIdentity EJ3114{0x00000002, 0x0C2A2852, 4, "EJ3114"};
inline constexpr DeviceIdentity EJ3124{0x00000002, 0x0C342852, 4, "EJ3124"};
inline constexpr DeviceIdentity EJ3148{0x00000002, 0x0C4C2852, 8, "EJ3148"};
inline constexpr DeviceIdentity EJ3202{0x00000002, 0x0C822852, 2, "EJ3202"};
inline constexpr DeviceIdentity EJ3214{0x00000002, 0x0C8E2852, 4, "EJ3214"};
inline constexpr DeviceIdentity EJ3255{0x00000002, 0x0CB72852, 5, "EJ3255"};
inline constexpr DeviceIdentity EJ3314{0x00000002, 0x0CF22852, 4, "EJ3314"};
inline constexpr DeviceIdentity EJ3318{0x00000002, 0x0CF62852, 8, "EJ3318"};

inline constexpr DeviceIdentity EP3048{0x00000002, 0x0BE84052, 8, "EP3048"};
inline constexpr DeviceIdentity EP3162{0x00000002, 0x0C5A4052, 2, "EP3162"};
inline constexpr DeviceIdentity EP3174{0x00000002, 0x0C664052, 4, "EP3174"};
inline constexpr DeviceIdentity EP3182{0x00000002, 0x0C6E4052, 2, "EP3182"};
inline constexpr DeviceIdentity EP3184{0x00000002, 0x0C704052, 4, "EP3184"};
inline constexpr DeviceIdentity EP3204{0x00000002, 0x0C844052, 4, "EP3204"};
inline constexpr DeviceIdentity EP3314{0x00000002, 0x0CF24052, 4, "EP3314"};
inline constexpr DeviceIdentity EP3351{0x00000002, 0x0D174052, 1, "EP3351"};
inline constexpr DeviceIdentity EP3356{0x00000002, 0x0D1C4052, 1, "EP3356"};
inline constexpr DeviceIdentity EP3632{0x00000002, 0x0E304052, 2, "EP3632"};
inline constexpr DeviceIdentity EP3744{0x00000002, 0x0EA04052, 4, "EP3744"};
inline constexpr DeviceIdentity EP3751{0x00000002, 0x0EA74052, 1, "EP3751"};
inline constexpr DeviceIdentity EP3752{0x00000002, 0x0EA84052, 2, "EP3752"};

inline constexpr DeviceIdentity ER3174{0x00000002, 0x0C664852, 4, "ER3174"};
inline constexpr DeviceIdentity ER3184{0x00000002, 0x0C704852, 4, "ER3184"};
inline constexpr DeviceIdentity ER3204{0x00000002, 0x0C844852, 4, "ER3204"};
inline constexpr DeviceIdentity ER3314{0x00000002, 0x0CF24852, 4, "ER3314"};

/// Every known analog-input terminal — the default detection set used by
/// MultiAnalogInputTerminal::detect().  All entries verified against the
/// ESIs for shape (mailbox + SM3 inputs + per-channel ≤32-bit value
/// entries); none verified on hardware yet.
inline constexpr std::array kAnalogInputTerminals{
    EL3001, EL3002, EL3004, EL3008, EL3011, EL3012, EL3014,
    EL3021, EL3022, EL3024, EL3041, EL3042, EL3044, EL3048,
    EL3051, EL3052, EL3054, EL3058, EL3061, EL3062, EL3064, EL3068,
    EL3072, EL3074, EL3078,
    EL3101, EL3102, EL3104, EL3111, EL3112, EL3114,
    EL3121, EL3122, EL3124, EL3141, EL3142, EL3144,
    EL3151, EL3152, EL3154, EL3161, EL3162, EL3164,
    EL3172, EL3174, EL3182, EL3184,
    EL3201, EL3202, EL3204, EL3208, EL3214, EL3218, EL3255,
    EL3311, EL3312, EL3314, EL3318, EL3351, EL3356,
    EL3444, EL3446,
    EL3602, EL3611, EL3612, EL3621, EL3681, EL3692,
    EL3751,
    EJ3004, EJ3008, EJ3048, EJ3058, EJ3068, EJ3104, EJ3108,
    EJ3114, EJ3124, EJ3148, EJ3202, EJ3214, EJ3255, EJ3314, EJ3318,
    EP3048, EP3162, EP3174, EP3182, EP3184, EP3204, EP3314,
    EP3351, EP3356, EP3632, EP3744, EP3751, EP3752,
    ER3174, ER3184, ER3204, ER3314,
};

} // namespace Devices

// ============================================================================
// AnalogInputTerminal — generic SII-driven analog input driver
// ============================================================================

class AnalogInputTerminal : public TerminalBase,
                            public IAnalogInputTerminal {
public:
    /// Maximum channels supported per device (EL3403-class devices exceed
    /// this and are intentionally not in the registry).
    static constexpr size_t kMaxChannels = 64;

    /// Maximum process-image bytes per device (input or output side).
    static constexpr size_t kMaxImageBytes = 512;

    // -- Construction ----------------------------------------------------------

    /**
     * @brief Bind the driver to a slave by bus position.
     * @param master       Started master (must outlive this object).
     * @param slave_index  Zero-based bus position of the terminal.
     * @param identity     Expected device identity (vendor/product).
     *
     * The slave's identity and SII data are read lazily in configure()
     * (via discovery().discoverOne()), so construction never performs bus I/O.
     */
    AnalogInputTerminal(Master& master, uint16_t slave_index,
                        const DeviceIdentity& identity);

    /**
     * @brief Bind the driver to an already-discovered slave.
     *
     * Reuses the DiscoveredSlave's SII data (sync managers, PDOs, name) so
     * configure() does not need to re-read the EEPROM.  Identity is checked
     * in configure() — a vendor/product mismatch fails with
     * Error::WrongDevice.
     */
    AnalogInputTerminal(Master& master, const DiscoveredSlave& slave,
                        const DeviceIdentity& identity);

    ~AnalogInputTerminal() override;

    AnalogInputTerminal(AnalogInputTerminal&&) noexcept            = default;
    AnalogInputTerminal& operator=(AnalogInputTerminal&&) noexcept = default;
    AnalogInputTerminal(const AnalogInputTerminal&)                = delete;
    AnalogInputTerminal& operator=(const AnalogInputTerminal&)     = delete;

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
    static Result<AnalogInputTerminal> findFirst(Master& master,
                                                 const DeviceIdentity& identity);

    /// Like findFirst(master, identity) but reuses an existing scan.
    static Result<AnalogInputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Standalone bring-up (position addressing, no FMMU) --------------------

    /**
     * @brief Configure the terminal up to SAFE-OP using position addressing.
     *
     * Steps: read SII (unless supplied via the DiscoveredSlave constructor),
     * verify identity, configure the mailbox (SM0/SM1 + CoE), program the
     * process-data sync managers, enter PRE-OP, register the PDO buffers
     * (position addressing — no FMMU needed), enter SAFE-OP.
     * Idempotent: repeated calls are no-ops once configured.
     */
    Result<> configure();

    /**
     * @brief Bring the terminal fully up: configure() then OP.
     *
     * With the default StartOptions this also starts the master's realtime
     * loop with a default exchange callback — after start() returns the
     * inputs are live and value()/status() reflect the terminal's inputs.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    // -- IAnalogInputTerminal ----------------------------------------------------

    uint16_t slaveIndex() const override {
        return TerminalBase::slaveIndex();
    }
    size_t   channelCount() const override { return channels_.size(); }
    const char* deviceName() const override {
        return TerminalBase::deviceName();
    }

    int32_t  value(size_t channel) const override;
    uint16_t status(size_t channel) const override;
    bool     hasStatus(size_t channel) const override;

    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Status -------------------------------------------------------------------

    /// Bit width of channel `ch`'s value field (16 or 32; 0 when the
    /// layout is not resolved yet or `ch` is out of range).
    size_t channelBits(size_t ch) const;

private:
    /// Per-channel layout resolved from the SII TxPDO list.
    struct ChannelLayout {
        uint16_t value_off;   ///< byte offset of the value in the SM3 image
        uint8_t  value_bits;  ///< value width: 16 or 32
        int16_t  status_off;  ///< byte offset of the status word, -1 = none
    };

    /// Resolve the per-channel value/status offsets from the bound
    /// discovery data / SII (SM regions themselves come from the base).
    /// Returns false when the slave does not match the terminal shape.
    bool resolveLayout();

    /// Shared bring-up: SII, identity check, mailbox config, SM registers,
    /// PRE-OP, PDO buffer registration in the given address mode.
    /// Ends before SAFE-OP so the chain can interpose FMMU programming.
    Result<> prepare(PDO::PDOAddressMode mode);

    std::vector<ChannelLayout> channels_;
};

} // namespace Beckhoff

} // namespace EtherCAT
