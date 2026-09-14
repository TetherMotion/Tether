/**
 * @file InputTerminal.hpp
 * @brief Generic driver for Beckhoff-style packed-bit input terminals
 *
 * InputTerminal reads any terminal whose process inputs are N one-bit
 * channels packed into a single sync manager — the whole "SM0 = Inputs,
 * one input FMMU, no mailbox" family of the Beckhoff EL1xxx ESI (see the
 * Devices registry below).  The channel count comes from the
 * DeviceIdentity (or, when declared as 0, from the SII TxPDO bit sum), so
 * one implementation covers 1-, 2-, 4-, 8- ... 64-channel terminals.
 *
 * @code
 *   // Standalone — position addressing, one frame per terminal:
 *   auto el = InputTerminal::findFirst(master, Devices::EL1008);
 *   el->start();
 *   bool on = el->bit(0);
 *
 *   // Chained — one shared logical address space, one LRW frame per cycle:
 *   MultiInputTerminal<> ins(master);              // see MultiInputTerminal.hpp
 *   ins.detect();                          // every known input terminal
 *   ins.start();
 *   auto field = ins.bits();               // flat std::bitset
 * @endcode
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 * Bits are stored little-endian: bit N maps to byte N/8, bit N%8 of the
 * process-data field — matching the PDO entry order in the ESI.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

#include "tether/Beckhoff/IInputTerminal.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode, kMaxPDOSlaves
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

// ============================================================================
// Known packed-input terminals (from the Beckhoff EL1xxx ESI)
// ============================================================================
// Every entry below has exactly one "Inputs" sync manager, one "Inputs"
// FMMU, N x 1-bit TxPDOs and no mailbox — i.e. the InputTerminal shape.
// Product codes verified against Beckhoff EL1xxx.xml.

namespace Devices {

inline constexpr DeviceIdentity EL1002{0x00000002, 0x03EA3052, 2,  "EL1002"};
inline constexpr DeviceIdentity EL1004{0x00000002, 0x03EC3052, 4,  "EL1004"};
inline constexpr DeviceIdentity EL1008{0x00000002, 0x03F03052, 8,  "EL1008"};
inline constexpr DeviceIdentity EL1012{0x00000002, 0x03F43052, 2,  "EL1012"};
inline constexpr DeviceIdentity EL1014{0x00000002, 0x03F63052, 4,  "EL1014"};
inline constexpr DeviceIdentity EL1018{0x00000002, 0x03FA3052, 8,  "EL1018"};
inline constexpr DeviceIdentity EL1024{0x00000002, 0x04003052, 4,  "EL1024"};
inline constexpr DeviceIdentity EL1034{0x00000002, 0x040A3052, 4,  "EL1034"};
inline constexpr DeviceIdentity EL1052{0x00000002, 0x041C3052, 4,  "EL1052"};
inline constexpr DeviceIdentity EL1054{0x00000002, 0x041E3052, 8,  "EL1054"};
inline constexpr DeviceIdentity EL1084{0x00000002, 0x043C3052, 4,  "EL1084"};
inline constexpr DeviceIdentity EL1088{0x00000002, 0x04403052, 8,  "EL1088"};
inline constexpr DeviceIdentity EL1094{0x00000002, 0x04463052, 4,  "EL1094"};
inline constexpr DeviceIdentity EL1098{0x00000002, 0x044A3052, 8,  "EL1098"};
inline constexpr DeviceIdentity EL1104{0x00000002, 0x04503052, 4,  "EL1104"};
inline constexpr DeviceIdentity EL1114{0x00000002, 0x045A3052, 4,  "EL1114"};
inline constexpr DeviceIdentity EL1124{0x00000002, 0x04643052, 4,  "EL1124"};
inline constexpr DeviceIdentity EL1134{0x00000002, 0x046E3052, 4,  "EL1134"};
inline constexpr DeviceIdentity EL1144{0x00000002, 0x04783052, 4,  "EL1144"};
inline constexpr DeviceIdentity EL1202{0x00000002, 0x04B23052, 2,  "EL1202"};
inline constexpr DeviceIdentity EL1382{0x00000002, 0x05663052, 4,  "EL1382"};
inline constexpr DeviceIdentity EL1409{0x00000002, 0x05813052, 16, "EL1409"};
inline constexpr DeviceIdentity EL1429{0x00000002, 0x05953052, 16, "EL1429"};
inline constexpr DeviceIdentity EL1489{0x00000002, 0x05D13052, 16, "EL1489"};
inline constexpr DeviceIdentity EL1702{0x00000002, 0x06A63052, 2,  "EL1702"};
inline constexpr DeviceIdentity EL1712{0x00000002, 0x06B03052, 2,  "EL1712"};
inline constexpr DeviceIdentity EL1722{0x00000002, 0x06BA3052, 2,  "EL1722"};
inline constexpr DeviceIdentity EL1804{0x00000002, 0x070C3052, 4,  "EL1804"};
inline constexpr DeviceIdentity EL1808{0x00000002, 0x07103052, 8,  "EL1808"};
inline constexpr DeviceIdentity EL1809{0x00000002, 0x07113052, 16, "EL1809"};
inline constexpr DeviceIdentity EL1814{0x00000002, 0x07163052, 4,  "EL1814"};
inline constexpr DeviceIdentity EL1819{0x00000002, 0x071B3052, 16, "EL1819"};
inline constexpr DeviceIdentity EL1862{0x00000002, 0x07463052, 16, "EL1862"};
inline constexpr DeviceIdentity EL1872{0x00000002, 0x07503052, 16, "EL1872"};
inline constexpr DeviceIdentity EL1889{0x00000002, 0x07613052, 16, "EL1889"};
inline constexpr DeviceIdentity EL1899{0x00000002, 0x076B3052, 16, "EL1899"};

// --- EP/ER/EJ field-box/plug-in variants with packed input images ---
// ESI-verified packed-bit input images; supported, not verified on
// hardware yet.  EP/ER/EJ 2339/2349 and EJ2819 are mixed I/O boxes —
// their input half is mapped here, the output half via OutputTerminal.
inline constexpr DeviceIdentity EP1819{0x00000002, 0x071B4052, 16, "EP1819"};
inline constexpr DeviceIdentity EP2339I{0x00000002, 0x09234052, 8,  "EP2339"};
inline constexpr DeviceIdentity EP2349I{0x00000002, 0x092D4052, 8,  "EP2349"};
inline constexpr DeviceIdentity ER2339I{0x00000002, 0x09234852, 8,  "ER2339"};
inline constexpr DeviceIdentity ER2349I{0x00000002, 0x092D4852, 8,  "ER2349"};
inline constexpr DeviceIdentity EJ2819I{0x00000002, 0x0B032852, 16, "EJ2819"};

// --- 32-channel and specialty inputs ------------------------------------------
inline constexpr DeviceIdentity EL1417{0x00000002, 0x05893052, 32, "EL1417"};

// --- EJ plug-in digital inputs (0x2852 suffix) ---------------------------------
inline constexpr DeviceIdentity EJ1008{0x00000002, 0x03F02852, 8,  "EJ1008"};
inline constexpr DeviceIdentity EJ1128{0x00000002, 0x04682852, 8,  "EJ1128"};
inline constexpr DeviceIdentity EJ1809{0x00000002, 0x07112852, 16, "EJ1809"};
inline constexpr DeviceIdentity EJ1819{0x00000002, 0x071B2852, 16, "EJ1819"};
inline constexpr DeviceIdentity EJ1889{0x00000002, 0x07612852, 16, "EJ1889"};

// --- EP/ER digital input boxes ---------------------------------------------------
inline constexpr DeviceIdentity EP1008{0x00000002, 0x03F04052, 8,  "EP1008"};
inline constexpr DeviceIdentity EP1018{0x00000002, 0x03FA4052, 8,  "EP1018"};
inline constexpr DeviceIdentity EP1098{0x00000002, 0x044A4052, 8,  "EP1098"};
inline constexpr DeviceIdentity EP1809{0x00000002, 0x07114052, 16, "EP1809"};
inline constexpr DeviceIdentity EP1816{0x00000002, 0x07184052, 16, "EP1816"};
inline constexpr DeviceIdentity ER1008{0x00000002, 0x03F04852, 8,  "ER1008"};
inline constexpr DeviceIdentity ER1018{0x00000002, 0x03FA4852, 8,  "ER1018"};
inline constexpr DeviceIdentity ER1098{0x00000002, 0x044A4852, 8,  "ER1098"};
inline constexpr DeviceIdentity ER1809{0x00000002, 0x07114852, 16, "ER1809"};
inline constexpr DeviceIdentity ER1819{0x00000002, 0x071B4852, 16, "ER1819"};

// --- EtherCAT P (EPP) digital input boxes ---------------------------------------
// The ESI defines these as module-style devices without explicit PDO
// lists; on hardware the SII resolves the same packed-bit layout.
inline constexpr DeviceIdentity EPP1004{0x00000002, 0x647604C9, 4,  "EPP1004"};
inline constexpr DeviceIdentity EPP1008{0x00000002, 0x64760509, 8,  "EPP1008"};
inline constexpr DeviceIdentity EPP1018{0x00000002, 0x647605A9, 8,  "EPP1018"};
inline constexpr DeviceIdentity EPP1098{0x00000002, 0x64760AA9, 8,  "EPP1098"};
inline constexpr DeviceIdentity EPP1111{0x00000002, 0x64760B79, 4,  "EPP1111"};
inline constexpr DeviceIdentity EPP1809{0x00000002, 0x647637B9, 16, "EPP1809"};
inline constexpr DeviceIdentity EPP1819{0x00000002, 0x647637B9, 16, "EPP1819"};

// --- Timestamped / XFC digital inputs -------------------------------------------
// The input bits lead the image; latch/event timestamps are extra fields
// (readable via the image, used by DC applications).  num_bits is the
// physical channel count.
inline constexpr DeviceIdentity EL1252{0x00000002, 0x04E43052, 2,  "EL1252"};
inline constexpr DeviceIdentity EL1254{0x00000002, 0x04E63052, 2,  "EL1254"};
inline constexpr DeviceIdentity EL1258{0x00000002, 0x04EA3052, 8,  "EL1258"};
inline constexpr DeviceIdentity EL1259{0x00000002, 0x04EB3052, 8,  "EL1259"};
inline constexpr DeviceIdentity EL1262{0x00000002, 0x04EE3052, 2,  "EL1262"};
inline constexpr DeviceIdentity EL1264{0x00000002, 0x04F03052, 4,  "EL1264"};
inline constexpr DeviceIdentity EJ1254{0x00000002, 0x04E62852, 2,  "EJ1254"};
inline constexpr DeviceIdentity EP1258{0x00000002, 0x04EA4052, 8,  "EP1258"};
inline constexpr DeviceIdentity ER1258{0x00000002, 0x04EA4852, 8,  "ER1258"};
inline constexpr DeviceIdentity EPP1258{0x00000002, 0x64761209, 8,  "EPP1258"};

// --- Ex-i intrinsically safe inputs (NAMUR; error bits interleaved) -------------
// num_bits counts every 1-bit entry — input and per-channel error/diag
// bits alternate (ch1 in, ch1 err, ch2 in, ch2 err, ...).
inline constexpr DeviceIdentity ELX1052{0x00000002, 0x970B3FC9, 4,  "ELX1052"};
inline constexpr DeviceIdentity ELX1054{0x00000002, 0x970B3FE9, 12, "ELX1054"};
inline constexpr DeviceIdentity ELX1058{0x00000002, 0x970B4029, 16, "ELX1058"};
inline constexpr DeviceIdentity EPX1058{0x00000002, 0x98092829, 16, "EPX1058"};

// --- Power supply / system status terminals -------------------------------------
// One or two packed status bits (PowerOK, overload, undervoltage, fuse).
inline constexpr DeviceIdentity EL9110{0x00000002, 0x23963052, 1,  "EL9110"};
inline constexpr DeviceIdentity EL9160{0x00000002, 0x23C83052, 1,  "EL9160"};
inline constexpr DeviceIdentity EL9210{0x00000002, 0x23FA3052, 2,  "EL9210"};
inline constexpr DeviceIdentity EL9260{0x00000002, 0x242C3052, 2,  "EL9260"};
inline constexpr DeviceIdentity EL9410{0x00000002, 0x24C23052, 2,  "EL9410"};
inline constexpr DeviceIdentity EL9505{0x00000002, 0x25213052, 2,  "EL9505"};
inline constexpr DeviceIdentity EL9508{0x00000002, 0x25243052, 2,  "EL9508"};
inline constexpr DeviceIdentity EL9510{0x00000002, 0x25263052, 2,  "EL9510"};
inline constexpr DeviceIdentity EL9512{0x00000002, 0x25283052, 2,  "EL9512"};
inline constexpr DeviceIdentity EL9515{0x00000002, 0x252B3052, 2,  "EL9515"};
inline constexpr DeviceIdentity EL9520{0x00000002, 0x25303052, 1,  "EL9520"};
inline constexpr DeviceIdentity EL9540{0x00000002, 0x25443052, 2,  "EL9540"};
inline constexpr DeviceIdentity EL9550{0x00000002, 0x254E3052, 4,  "EL9550"};
inline constexpr DeviceIdentity EL9560{0x00000002, 0x25583052, 2,  "EL9560"};
inline constexpr DeviceIdentity EJ9505{0x00000002, 0x25212852, 2,  "EJ9505"};
inline constexpr DeviceIdentity ELX9410{0x00000002, 0x970D4A29, 2, "ELX9410"};
inline constexpr DeviceIdentity ELX9560{0x00000002, 0x970D5389, 2, "ELX9560"};

/// Every known packed-input terminal — the default detection set used by
/// MultiInputTerminal::detect().
inline constexpr std::array kInputTerminals{
    EL1002, EL1004, EL1008, EL1012, EL1014, EL1018, EL1024, EL1034,
    EL1052, EL1054, EL1084, EL1088, EL1094, EL1098, EL1104, EL1114,
    EL1124, EL1134, EL1144, EL1202, EL1382, EL1409, EL1429, EL1489,
    EL1702, EL1712, EL1722, EL1804, EL1808, EL1809, EL1814, EL1819,
    EL1862, EL1872, EL1889, EL1899,
    EP1819, EP2339I, EP2349I, ER2339I, ER2349I, EJ2819I,
    EJ1008, EJ1128, EJ1809, EJ1819, EJ1889,
    EP1008, EP1018, EP1098, EP1809, EP1816,
    ER1008, ER1018, ER1098, ER1809, ER1819,
    EPP1004, EPP1008, EPP1018, EPP1098, EPP1111, EPP1809, EPP1819,
    EL1252, EL1254, EL1258, EL1259, EL1262, EL1264,
    EJ1254, EP1258, ER1258, EPP1258,
    ELX1052, ELX1054, ELX1058, EPX1058,
    EL9110, EL9160, EL9210, EL9260, EL9410, EL9505, EL9508, EL9510,
    EL9512, EL9515, EL9520, EL9540, EL9550, EL9560, EJ9505,
    ELX9410, ELX9560, EL1417,
};

} // namespace Devices

// ============================================================================
// InputTerminal — generic single-field input terminal driver
// ============================================================================

class InputTerminal : public IInputTerminal {
public:
    /// Maximum input bits supported per device (one 64-bit field).
    static constexpr size_t kMaxBits = 64;

    // -- Construction ----------------------------------------------------------

    /**
     * @brief Bind the driver to a slave by bus position.
     * @param master       Started master (must outlive this object).
     * @param slave_index  Zero-based bus position of the terminal.
     * @param identity     Expected device identity (vendor/product/width).
     *
     * The slave's identity and SII data are read lazily in configure()
     * (via discovery().discoverOne()), so construction never performs bus I/O.
     */
    InputTerminal(Master& master, uint16_t slave_index,
                const DeviceIdentity& identity);

    /**
     * @brief Bind the driver to an already-discovered slave.
     *
     * Reuses the DiscoveredSlave's SII data (sync managers, PDOs, name) so
     * configure() does not need to re-read the EEPROM.  Identity is checked
     * in configure() — a vendor/product mismatch fails with
     * Error::WrongDevice.
     */
    InputTerminal(Master& master, const DiscoveredSlave& slave,
                const DeviceIdentity& identity);

    ~InputTerminal() override;

    InputTerminal(InputTerminal&&) noexcept            = default;
    InputTerminal& operator=(InputTerminal&&) noexcept = default;
    InputTerminal(const InputTerminal&)                = delete;
    InputTerminal& operator=(const InputTerminal&)     = delete;

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
    static Result<InputTerminal> findFirst(Master& master,
                                         const DeviceIdentity& identity);

    /// Like findFirst(master, identity) but reuses an existing scan.
    static Result<InputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Standalone bring-up (position addressing, no FMMU) --------------------

    /**
     * @brief Configure the terminal up to SAFE-OP using position addressing.
     *
     * Steps: read SII (unless supplied via the DiscoveredSlave constructor),
     * verify identity, program the process-input sync manager, enter
     * PRE-OP, register the input PDO buffer (position addressing, one
     * APRD read per cycle — no FMMU needed), enter SAFE-OP.
     * Idempotent: repeated calls are no-ops once configured.
     */
    Result<> configure();

    /**
     * @brief Bring the terminal fully up: configure() then OP.
     *
     * With the default StartOptions this also starts the master's realtime
     * loop with a default exchange callback — after start() returns the
     * inputs are live and bit()/bits() reflect the terminal's inputs.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /**
     * @brief Stop the master's realtime loop if this driver started it.
     *        Safe to call multiple times.
     */
    void stop();

    // -- IInputTerminal --------------------------------------------------------

    uint16_t slaveIndex() const override { return slave_index_; }
    size_t   bitCount() const override   { return num_inputs_; }
    const char* deviceName() const override;

    bool     bit(size_t bit) const override;
    uint64_t bits() const override {
        return state_->load(std::memory_order_relaxed) & mask();
    }

    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Status -------------------------------------------------------------------

    /// True after configure()/mapLogicalAndEnterSafeOp() succeeded.
    bool configured() const { return configured_; }

    /// Live-read the application-layer (ESM) state register.
    SlaveState alState();

    /// True when the terminal is in OP.
    bool operational() { return alState() == SlaveState::OP; }

    /// AL status code captured by the last failed state transition (0 = none).
    uint16_t lastAlStatusCode() const { return last_al_status_code_; }

    /// The declared/resolved device identity.
    const DeviceIdentity& identity() const { return identity_; }

    /// Logical address assigned to this device's input field (chained
    /// operation only; 0 when unmapped).
    uint32_t logicalAddress() const { return logical_addr_; }

private:
    /// Poll AL status until `target` or timeout; honours master cancellation.
    bool waitAlState(SlaveState target, int timeout_ms);

    /// Resolve the process-input SM (channel, address, length, control)
    /// and the input bit width from the bound discovery data / SII.
    void resolveInputSm();

    /// Shared bring-up: SII, identity check, input SM registers, PRE-OP,
    /// TxPDO registration in the given address mode, finalizeMapping().
    /// Ends before SAFE-OP so the chain can interpose FMMU programming.
    Result<> prepare(PDO::PDOAddressMode mode);

    /// assumePDOAlreadyConfigured() + SAFE-OP transition.
    Result<> enterSafeOp();

    /// Bit mask covering num_inputs_ bits.
    uint64_t mask() const {
        return num_inputs_ >= kMaxBits ? ~uint64_t{0}
                                       : (uint64_t{1} << num_inputs_) - 1;
    }

    Master*                        master_;
    uint16_t                       slave_index_;
    DeviceIdentity                 identity_;
    std::optional<DiscoveredSlave> info_;

    /// Registered PDO buffer — the cyclic exchange writes the latest input
    /// field into it.  Heap-allocated so the pointer stays valid when the
    /// driver object is moved (e.g. returned from findFirst()).
    /// Bit N maps to input N (little-endian byte order).
    std::unique_ptr<std::atomic<uint64_t>> state_;

    size_t   num_inputs_    = 0;    // resolved bit width
    uint8_t  sm_channel_    = 0;
    uint16_t sm_addr_       = 0x1000;
    uint16_t sm_len_        = 1;
    uint8_t  sm_ctrl_       = 0x00;
    uint16_t txpdo_index_   = 0x1A00;
    bool     prepared_      = false;  // prepare() done (any mode)
    bool     configured_    = false;  // reached SAFE-OP
    bool     loop_started_  = false;  // we started the master's RT loop
    uint32_t logical_addr_  = 0;
    uint16_t last_al_status_code_ = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
