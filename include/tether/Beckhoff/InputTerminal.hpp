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

/// Every known packed-input terminal — the default detection set used by
/// MultiInputTerminal::detect().
inline constexpr std::array kInputTerminals{
    EL1002, EL1004, EL1008, EL1012, EL1014, EL1018, EL1024, EL1034,
    EL1052, EL1054, EL1084, EL1088, EL1094, EL1098, EL1104, EL1114,
    EL1124, EL1134, EL1144, EL1202, EL1382, EL1409, EL1429, EL1489,
    EL1702, EL1712, EL1722, EL1804, EL1808, EL1809, EL1814, EL1819,
    EL1862, EL1872, EL1889, EL1899,
    EP1819, EP2339I, EP2349I, ER2339I, ER2349I, EJ2819I,
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
