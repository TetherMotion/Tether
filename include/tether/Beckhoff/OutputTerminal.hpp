/**
 * @file OutputTerminal.hpp
 * @brief Generic driver for Beckhoff-style packed-bit output terminals
 *
 * OutputTerminal drives any terminal whose process outputs are N one-bit
 * channels packed into a single sync manager — the whole "SM0 = Outputs,
 * one output FMMU, no mailbox" family of the Beckhoff EL2xxx ESI (see the
 * Devices registry below).  The channel count comes from the
 * DeviceIdentity (or, when declared as 0, from the SII RxPDO bit sum), so
 * one implementation covers 1-, 2-, 4-, 8- ... 64-channel terminals.
 *
 * @code
 *   // Standalone — position addressing, one frame per terminal:
 *   auto el = OutputTerminal::findFirst(master, Devices::EL2008);
 *   el->start();
 *   el->setBit(0, true);
 *
 *   // Chained — one shared logical address space, one LRW frame per cycle:
 *   MultiOutputTerminal<> outs(master);            // see MultiOutputTerminal.hpp
 *   outs.detect();                         // every known output terminal
 *   outs.start();
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

#include "tether/Beckhoff/IOutputTerminal.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode, kMaxPDOSlaves
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

// ============================================================================
// Known packed-output terminals (from the Beckhoff EL2xxx ESI)
// ============================================================================
// Every entry below was verified against Beckhoff EL2xxx.xml to share the
// EL2004's exact shape: exactly one "Outputs" sync manager, one "Outputs"
// FMMU, N x 1-bit RxPDOs, no mailbox, no inputs.
//
//   verified on hardware        : EL2004 (5 modules, examples/beckhoff_output_toggle)
//   supported, not verified yet : all other entries
//
// Caveats:
//   - EL2602 / EL2622: the -0010 variants (same product code) add an
//     "Inputs" SM carrying diagnostic data — outputs still work, the
//     input SM is simply unused.
//   - EL2262: only the base revision has this exact shape; newer
//     revisions add a second "Outputs" SM and would only be driven on the
//     first SM.
//   - EL2202 is DC-capable — works without DC configuration (free-run).
//
// Same family, different shape (not supported by this driver):
//   EL2014/EL2044/EL2212/EL2258/EL2819/EL2869 (mailbox + inputs),
//   EL2032/EL2034/EL2838/EL2878-0005 (extra diagnostic-input SM),
//   EL2252 (adds status TxPDOs), EL2409/EL2489/EL2809/EL2872/EL2889
//   (two output SMs), EL2602-0010/EL2622-0010 are covered by the caveat
//   above.

namespace Devices {

inline constexpr DeviceIdentity EL2002{0x00000002, 0x07D23052, 2,  "EL2002"};
inline constexpr DeviceIdentity EL2004{0x00000002, 0x07D43052, 4,  "EL2004"};
inline constexpr DeviceIdentity EL2008{0x00000002, 0x07D83052, 8,  "EL2008"};
inline constexpr DeviceIdentity EL2022{0x00000002, 0x07E63052, 2,  "EL2022"};
inline constexpr DeviceIdentity EL2024{0x00000002, 0x07E83052, 4,  "EL2024"};
inline constexpr DeviceIdentity EL2042{0x00000002, 0x07FA3052, 2,  "EL2042"};
inline constexpr DeviceIdentity EL2084{0x00000002, 0x08243052, 4,  "EL2084"};
inline constexpr DeviceIdentity EL2088{0x00000002, 0x08283052, 8,  "EL2088"};
inline constexpr DeviceIdentity EL2124{0x00000002, 0x084C3052, 4,  "EL2124"};
inline constexpr DeviceIdentity EL2202{0x00000002, 0x089A3052, 2,  "EL2202"};
inline constexpr DeviceIdentity EL2262{0x00000002, 0x08D63052, 2,  "EL2262"};
inline constexpr DeviceIdentity EL2407{0x00000002, 0x09673052, 32, "EL2407"};
inline constexpr DeviceIdentity EL2602{0x00000002, 0x0A2A3052, 2,  "EL2602"};
inline constexpr DeviceIdentity EL2612{0x00000002, 0x0A343052, 2,  "EL2612"};
inline constexpr DeviceIdentity EL2622{0x00000002, 0x0A3E3052, 2,  "EL2622"};
inline constexpr DeviceIdentity EL2624{0x00000002, 0x0A403052, 4,  "EL2624"};
inline constexpr DeviceIdentity EL2634{0x00000002, 0x0A4A3052, 4,  "EL2634"};
inline constexpr DeviceIdentity EL2652{0x00000002, 0x0A5C3052, 2,  "EL2652"};
inline constexpr DeviceIdentity EL2712{0x00000002, 0x0A983052, 2,  "EL2712"};
inline constexpr DeviceIdentity EL2722{0x00000002, 0x0AA23052, 2,  "EL2722"};
inline constexpr DeviceIdentity EL2732{0x00000002, 0x0AAC3052, 2,  "EL2732"};
inline constexpr DeviceIdentity EL2784{0x00000002, 0x0AE03052, 4,  "EL2784"};
inline constexpr DeviceIdentity EL2788{0x00000002, 0x0AE43052, 8,  "EL2788"};
inline constexpr DeviceIdentity EL2794{0x00000002, 0x0AEA3052, 4,  "EL2794"};
inline constexpr DeviceIdentity EL2798{0x00000002, 0x0AEE3052, 8,  "EL2798"};
inline constexpr DeviceIdentity EL2808{0x00000002, 0x0AF83052, 8,  "EL2808"};
inline constexpr DeviceIdentity EL2828{0x00000002, 0x0B0C3052, 8,  "EL2828"};

// --- EP/ER/EJ field-box/plug-in variants with packed output images ---
// Same electronics as the EL counterparts under different product-code
// suffixes (0x4052=EP, 0x4852=ER, 0x2852=EJ).  ESI-verified packed-bit
// output images; supported, not verified on hardware yet.
inline constexpr DeviceIdentity EP2816{0x00000002, 0x0B004052, 16, "EP2816"};
inline constexpr DeviceIdentity EP2339{0x00000002, 0x09234052, 8,  "EP2339"};
inline constexpr DeviceIdentity EP2349{0x00000002, 0x092D4052, 8,  "EP2349"};
inline constexpr DeviceIdentity ER2339{0x00000002, 0x09234852, 8,  "ER2339"};
inline constexpr DeviceIdentity ER2349{0x00000002, 0x092D4852, 8,  "ER2349"};
inline constexpr DeviceIdentity EJ2819{0x00000002, 0x0B032852, 16, "EJ2819"};
inline constexpr DeviceIdentity EP6228{0x00000002, 0x18544052, 8,  "EP6228"};

// --- Extended EL2xxx outputs (16-bit wide, two-SM layouts resolved via SII) -----
inline constexpr DeviceIdentity EL2409{0x00000002, 0x09693052, 16, "EL2409"};
inline constexpr DeviceIdentity EL2489{0x00000002, 0x09B93052, 16, "EL2489"};
inline constexpr DeviceIdentity EL2809{0x00000002, 0x0AF93052, 16, "EL2809"};
inline constexpr DeviceIdentity EL2872{0x00000002, 0x0B383052, 16, "EL2872"};
inline constexpr DeviceIdentity EL2889{0x00000002, 0x0B493052, 16, "EL2889"};

// --- Timestamped / XFC digital outputs ------------------------------------------
// Output bits lead the RxPDO image; start-time/cycle fields follow.
inline constexpr DeviceIdentity EL2252{0x00000002, 0x08CC3052, 2,  "EL2252"};
inline constexpr DeviceIdentity EL2258{0x00000002, 0x08D23052, 8,  "EL2258"};

// --- EJ plug-in digital outputs (0x2852) ------------------------------------------
inline constexpr DeviceIdentity EJ2008{0x00000002, 0x07D82852, 8,  "EJ2008"};
inline constexpr DeviceIdentity EJ2128{0x00000002, 0x08502852, 8,  "EJ2128"};
inline constexpr DeviceIdentity EJ2809{0x00000002, 0x0AF92852, 16, "EJ2809"};
inline constexpr DeviceIdentity EJ2889{0x00000002, 0x0B492852, 16, "EJ2889"};

// --- EP/ER digital output boxes ------------------------------------------------------
inline constexpr DeviceIdentity EP2001{0x00000002, 0x07D14052, 8,  "EP2001"};
inline constexpr DeviceIdentity EP2008{0x00000002, 0x07D84052, 8,  "EP2008"};
inline constexpr DeviceIdentity EP2028{0x00000002, 0x07EC4052, 8,  "EP2028"};
inline constexpr DeviceIdentity EP2624{0x00000002, 0x0A404052, 4,  "EP2624"};
inline constexpr DeviceIdentity EP2809{0x00000002, 0x0AF94052, 16, "EP2809"};
inline constexpr DeviceIdentity ER2008{0x00000002, 0x07D84852, 8,  "ER2008"};
inline constexpr DeviceIdentity ER2028{0x00000002, 0x07EC4852, 8,  "ER2028"};
inline constexpr DeviceIdentity ER2624{0x00000002, 0x0A404852, 4,  "ER2624"};
inline constexpr DeviceIdentity ER2809{0x00000002, 0x0AF94852, 16, "ER2809"};

// --- EtherCAT P (EPP) output boxes ----------------------------------------------------
inline constexpr DeviceIdentity EPP2008{0x00000002, 0x64764389, 8,  "EPP2008"};
inline constexpr DeviceIdentity EPP2028{0x00000002, 0x647644C9, 8,  "EPP2028"};
inline constexpr DeviceIdentity EPP2304{0x00000002, 0x64765609, 4,  "EPP2304"};
inline constexpr DeviceIdentity EPP2624{0x00000002, 0x64766A09, 4,  "EPP2624"};
inline constexpr DeviceIdentity EPP2809{0x00000002, 0x64767599, 16, "EPP2809"};
inline constexpr DeviceIdentity EPP2816{0x00000002, 0x64767609, 16, "EPP2816"};
inline constexpr DeviceIdentity EPP2817{0x00000002, 0x64767619, 16, "EPP2817"};

// --- Ex-i intrinsically safe outputs --------------------------------------------------
inline constexpr DeviceIdentity ELX2002{0x00000002, 0x970B7B29, 2,  "ELX2002"};
inline constexpr DeviceIdentity ELX2008{0x00000002, 0x970B7B89, 8,  "ELX2008"};
inline constexpr DeviceIdentity ELX2792{0x00000002, 0x970BAC89, 2,  "ELX2792"};
inline constexpr DeviceIdentity EPX2004{0x00000002, 0x98096349, 4,  "EPX2004"};

/// Every known packed-output terminal — the default detection set used by
/// MultiOutputTerminal::detect().
inline constexpr std::array kOutputTerminals{
    EL2002, EL2004, EL2008, EL2022, EL2024, EL2042, EL2084, EL2088,
    EL2124, EL2202, EL2262, EL2407, EL2602, EL2612, EL2622, EL2624,
    EL2634, EL2652, EL2712, EL2722, EL2732, EL2784, EL2788, EL2794,
    EL2798, EL2808, EL2828,
    EP2816, EP2339, EP2349, ER2339, ER2349, EJ2819, EP6228,
    EL2409, EL2489, EL2809, EL2872, EL2889, EL2252, EL2258,
    EJ2008, EJ2128, EJ2809, EJ2889,
    EP2001, EP2008, EP2028, EP2624, EP2809,
    ER2008, ER2028, ER2624, ER2809,
    EPP2008, EPP2028, EPP2304, EPP2624, EPP2809, EPP2816, EPP2817,
    ELX2002, ELX2008, ELX2792, EPX2004,
};

} // namespace Devices

// ============================================================================
// OutputTerminal — generic single-field output terminal driver
// ============================================================================

class OutputTerminal : public IOutputTerminal {
public:
    /// Maximum output bits supported per device (one 64-bit field).
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
    OutputTerminal(Master& master, uint16_t slave_index,
                 const DeviceIdentity& identity);

    /**
     * @brief Bind the driver to an already-discovered slave.
     *
     * Reuses the DiscoveredSlave's SII data (sync managers, PDOs, name) so
     * configure() does not need to re-read the EEPROM.  Identity is checked
     * in configure() — a vendor/product mismatch fails with
     * Error::WrongDevice.
     */
    OutputTerminal(Master& master, const DiscoveredSlave& slave,
                 const DeviceIdentity& identity);

    ~OutputTerminal() override;

    OutputTerminal(OutputTerminal&&) noexcept            = default;
    OutputTerminal& operator=(OutputTerminal&&) noexcept = default;
    OutputTerminal(const OutputTerminal&)                = delete;
    OutputTerminal& operator=(const OutputTerminal&)     = delete;

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
    static Result<OutputTerminal> findFirst(Master& master,
                                          const DeviceIdentity& identity);

    /// Like findFirst(master, identity) but reuses an existing scan.
    static Result<OutputTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Standalone bring-up (position addressing, no FMMU) --------------------

    /**
     * @brief Configure the terminal up to SAFE-OP using position addressing.
     *
     * Steps: read SII (unless supplied via the DiscoveredSlave constructor),
     * verify identity, program the process-output sync manager, enter
     * PRE-OP, register the output PDO buffer (position addressing, one
     * APWR write per cycle — no FMMU needed), enter SAFE-OP.
     * Idempotent: repeated calls are no-ops once configured.
     */
    Result<> configure();

    /**
     * @brief Bring the terminal fully up: configure() then OP.
     *
     * With the default StartOptions this also starts the master's realtime
     * loop with a default exchange callback — after start() returns the
     * outputs are live and setters take effect on the next cycle.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /**
     * @brief Switch all outputs off and, if this driver started the master's
     *        realtime loop, stop it.  Safe to call multiple times.
     */
    void stop();

    // -- IOutputTerminal -------------------------------------------------------

    uint16_t slaveIndex() const override { return slave_index_; }
    size_t   bitCount() const override   { return num_outputs_; }
    const char* deviceName() const override;

    void     setBit(size_t bit, bool on) override;
    bool     bit(size_t bit) const override;
    void     setBits(uint64_t bits) override;
    uint64_t bits() const override {
        return state_->load(std::memory_order_relaxed) & mask();
    }
    void     allOff() override { setBits(0); }

    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Convenience ------------------------------------------------------------

    /// Set exactly one output bit on, all others off.
    void setOnly(size_t bit) {
        setBits(bit < num_outputs_ ? (uint64_t{1} << bit) : 0);
    }

    /// All output bits on.
    void allOn() { setBits(mask()); }

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

    /// Logical address assigned to this device's output field (chained
    /// operation only; 0 when unmapped).
    uint32_t logicalAddress() const { return logical_addr_; }

private:
    /// Poll AL status until `target` or timeout; honours master cancellation.
    bool waitAlState(SlaveState target, int timeout_ms);

    /// Resolve the process-output SM (channel, address, length, control)
    /// and the output bit width from the bound discovery data / SII.
    void resolveOutputSm();

    /// Shared bring-up: SII, identity check, output SM registers, PRE-OP,
    /// PDO registration in the given address mode, finalizeMapping().
    /// Ends before SAFE-OP so the chain can interpose FMMU programming.
    Result<> prepare(PDO::PDOAddressMode mode);

    /// assumePDOAlreadyConfigured() + SAFE-OP transition.
    Result<> enterSafeOp();

    /// Bit mask covering num_outputs_ bits.
    uint64_t mask() const {
        return num_outputs_ >= kMaxBits ? ~uint64_t{0}
                                        : (uint64_t{1} << num_outputs_) - 1;
    }

    Master*                        master_;
    uint16_t                       slave_index_;
    DeviceIdentity                 identity_;
    std::optional<DiscoveredSlave> info_;

    /// Registered PDO buffer.  Heap-allocated so the pointer stays valid
    /// when the driver object is moved (e.g. returned from findFirst()).
    /// Bit N maps to output N (little-endian byte order).
    /// PDO state word — points at the mapping entry's storage after
    /// prepare(), or at *state_local_ before registration (Q1).
    /// Same lifetime rule as TerminalBase::out_buf_: entry removal
    /// invalidates it.
    /// unique_ptr keeps the terminal movable.
    std::unique_ptr<std::atomic<uint64_t>> state_local_ =
        std::make_unique<std::atomic<uint64_t>>(0);
    std::atomic<uint64_t>* state_ = state_local_.get();

    size_t   num_outputs_   = 0;    // resolved bit width
    uint8_t  sm_channel_    = 0;
    uint16_t sm_addr_       = 0x0F00;
    uint16_t sm_len_        = 1;
    uint8_t  sm_ctrl_       = 0x44;
    uint16_t rxpdo_index_   = 0x1600;
    bool     prepared_      = false;  // prepare() done (any mode)
    bool     configured_    = false;  // reached SAFE-OP
    bool     loop_started_  = false;  // we started the master's RT loop
    uint32_t logical_addr_  = 0;
    uint16_t last_al_status_code_ = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
