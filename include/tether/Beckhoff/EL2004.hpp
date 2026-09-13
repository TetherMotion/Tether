/**
 * @file EL2004.hpp
 * @brief Easy-to-use driver for the Beckhoff EL2004 4-channel digital output terminal
 *
 * The EL2004 (vendor 0x00000002, product 0x07D43052) is a mailbox-less,
 * output-only EtherCAT terminal: four 24V/0.5A channels packed as four bits
 * into a single process-data byte.  Unlike most slaves its process-output
 * data lives on sync-manager channel 0 (there are no mailbox SMs at all),
 * which makes the generic mailbox/PDO bring-up path unusable.  This driver
 * hides all of that: construct it, call start(), set bits.
 *
 * @code
 *   // Simplest possible usage — find and drive the first EL2004 on the bus:
 *   auto el = EL2004::findFirst(master);
 *   if (!el) { ... el.error() ... }
 *   el->start();                    // PRE-OP -> SAFE-OP -> OP (+ RT loop)
 *   el->set(0, true);               // channel 1 ON  (0-based channel index!)
 *   el->setChannels(0b0101);        // channels 1+3 ON
 *   el->allOff();
 * @endcode
 *
 * Output setters only touch the registered PDO buffer — the master's cyclic
 * exchange publishes them on the next cycle.  They are safe to call from any
 * thread (including while the realtime loop is running); each call is a
 * single-byte lock-free store.
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 *
 * For chains with several EL2004 terminals see MultiEL2004.hpp, which exposes
 * one flat bit interface across all of them.
 */

#pragma once

#include <atomic>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <span>

#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

// ============================================================================
// EL2004 — single-terminal driver
// ============================================================================

class EL2004 {
public:
    // -- Device identity (from the Beckhoff EL2xxx ESI) ----------------------

    static constexpr uint32_t kVendorId    = 0x00000002;  ///< Beckhoff Automation
    static constexpr uint32_t kProductCode = 0x07D43052;  ///< EL2004
    static constexpr size_t   kNumChannels = 4;           ///< digital outputs
    static constexpr uint16_t kRxPdoIndex  = 0x1600;      ///< first of 0x1600-0x1603

    /// Channel bitmask type — bit N corresponds to output channel N.
    using Channels = std::bitset<kNumChannels>;

    // -- Errors --------------------------------------------------------------

    enum class Error : uint8_t {
        Ok = 0,
        NoDeviceFound,          ///< findFirst()/detect(): no EL2004 on the bus
        NotAnEL2004,            ///< slave identity does not match vendor/product
        SlaveIndexOutOfRange,   ///< index exceeds the PDO manager's slave table
        SmConfigFailed,         ///< could not write the sync-manager registers
        PdoRegistrationFailed,  ///< add_rxpdo() rejected the buffer
        PreOpFailed,            ///< INIT -> PRE-OP transition failed
        SafeOpFailed,           ///< PRE-OP -> SAFE-OP transition failed
        OpRequestFailed,        ///< SAFE-OP -> OP request could not be sent
        OpTimeout,              ///< slave did not reach OP in time
        LoopStartFailed,        ///< master realtime loop failed to start
        Cancelled,              ///< aborted via master.requestCancel()
    };

    /// Expected-like result type used by all fallible operations.
    template <typename T = void>
    using Result = std::expected<T, Error>;

    /// Human-readable description of an Error value.
    static const char* errorToString(Error e);

    // -- Start options ---------------------------------------------------------

    struct StartOptions {
        /// When true (default), the driver installs a default motion-control
        /// callback that runs `pdo().exchangeAll()` and starts the master's
        /// realtime loop if it is not already running.  Set false when the
        /// application manages its own cyclic PDO exchange — the driver then
        /// only requests OP and assumes outputs are being exchanged.
        bool     manage_realtime_loop = true;
        /// Cycle period used when the driver starts the RT loop (microseconds).
        uint32_t cycle_period_us      = 1000;
        /// How long to wait for the OP state confirmation (milliseconds).
        int      op_timeout_ms        = 5000;
    };

    // -- Construction ----------------------------------------------------------

    /**
     * @brief Bind the driver to a slave by bus position.
     * @param master       Started master (must outlive this object).
     * @param slave_index  Zero-based bus position of the EL2004.
     *
     * The slave's identity and SII data are read lazily in configure()
     * (via discovery().discoverOne()), so construction never performs bus I/O.
     */
    EL2004(Master& master, uint16_t slave_index);

    /**
     * @brief Bind the driver to an already-discovered slave.
     *
     * Reuses the DiscoveredSlave's SII data (sync managers, PDOs, name) so
     * configure() does not need to re-read the EEPROM.  Identity is checked
     * in configure() — a positive vendor/product mismatch fails with
     * Error::NotAnEL2004.
     */
    EL2004(Master& master, const DiscoveredSlave& slave);

    ~EL2004();

    EL2004(EL2004&&) noexcept            = default;
    EL2004& operator=(EL2004&&) noexcept = default;
    EL2004(const EL2004&)                = delete;
    EL2004& operator=(const EL2004&)     = delete;

    // -- Factories ---------------------------------------------------------------

    /// True when `s` carries the EL2004 vendor/product identity.
    static bool matches(const DiscoveredSlave& s) {
        return s.hasVendorAndProduct(kVendorId, kProductCode);
    }

    /**
     * @brief Scan the bus and return a driver bound to the first EL2004 found.
     *
     * Runs a shallow discovery (vendor + product ID only); the selected
     * slave's SII is read later by configure().
     * @return The driver, or Error::NoDeviceFound.
     */
    static Result<EL2004> findFirst(Master& master);

    /**
     * @brief Like findFirst(master) but reuses an existing discovery result
     *        instead of scanning the bus again.
     */
    static Result<EL2004> findFirst(Master& master,
                                    std::span<const DiscoveredSlave> scan);

    // -- Bring-up ----------------------------------------------------------------

    /**
     * @brief Configure the terminal up to SAFE-OP.
     *
     * Steps: read SII (unless supplied via the DiscoveredSlave constructor),
     * verify identity, program the process-output sync manager (channel 0 @
     * 0x0F00 per ESI — the EL2004 has no mailbox SMs), enter PRE-OP, register
     * the 1-byte RxPDO buffer (position addressing, no FMMU needed), enter
     * SAFE-OP.  Idempotent: repeated calls are no-ops once configured.
     */
    Result<> configure();

    /**
     * @brief Bring the terminal fully up: configure() then OP.
     *
     * With the default StartOptions this also starts the master's realtime
     * loop with a default exchangeAll() callback — after start() returns the
     * outputs are live and setters take effect on the next cycle.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /**
     * @brief Request the SAFE-OP -> OP transition and wait for confirmation.
     *
     * Low-level step used by start() and by MultiEL2004 (which shares one RT
     * loop across modules).  Output-only terminals can never satisfy
     * Slave::transitionToOp()'s TxPDO reply-counter check, so the OP request
     * is issued directly and AL status is polled.
     */
    Result<> requestOp(int timeout_ms = 5000);

    /**
     * @brief Switch all outputs off and, if this driver started the master's
     *        realtime loop, stop it.  Safe to call multiple times.
     */
    void stop();

    // -- Output access -----------------------------------------------------------
    // All setters take effect on the next PDO exchange cycle.  They are
    // lock-free single-byte stores — safe from any thread, including while
    // the realtime loop runs.

    /// Set a single channel (0-3).  Out-of-range indices are ignored.
    void set(size_t channel, bool on);

    /// Current requested state of a channel (0-3); false if out of range.
    bool get(size_t channel) const;

    /// Set exactly one channel on, all others off.  Out-of-range indices
    /// are ignored.
    void setOnly(size_t channel) {
        if (channel < kNumChannels) setChannels(Channels{}.set(channel));
    }

    /// Write all four channels at once (bit N = channel N).
    void setChannels(Channels bits);

    /// Currently requested channel states (bit N = channel N).
    Channels channels() const { return Channels(raw()); }

    /// All four channels on.
    void allOn()  { setRaw(0x0F); }

    /// All four channels off.
    void allOff() { setRaw(0x00); }

    /// Raw output byte (only the low 4 bits are used).
    void    setRaw(uint8_t byte);
    uint8_t raw() const { return state_->load(std::memory_order_relaxed); }

    // -- Status ------------------------------------------------------------------

    /// Bus position this driver is bound to.
    uint16_t slaveIndex() const { return slave_index_; }

    /// True after configure() succeeded.
    bool configured() const { return configured_; }

    /// Live-read the application-layer (ESM) state register.
    SlaveState alState();

    /// True when the terminal is in OP.
    bool operational() { return alState() == SlaveState::OP; }

    /// AL status code captured by the last failed state transition (0 = none).
    uint16_t lastAlStatusCode() const { return last_al_status_code_; }

private:
    /// Poll AL status until `target` or timeout; honours master cancellation.
    bool waitAlState(SlaveState target, int timeout_ms);

    /// Resolve the process-output SM (channel, address, length, control)
    /// from the bound discovery data, with ESI fallbacks.
    void resolveOutputSm();

    Master*                        master_;
    uint16_t                       slave_index_;
    std::optional<DiscoveredSlave> info_;

    /// Registered PDO byte.  Heap-allocated so the pointer stays valid when
    /// the driver object is moved (e.g. returned from findFirst()).
    std::unique_ptr<std::atomic<uint8_t>> state_;

    uint8_t  sm_channel_    = 0;
    uint16_t sm_addr_       = 0x0F00;
    uint16_t sm_len_        = 1;
    uint8_t  sm_ctrl_       = 0x44;
    bool     configured_    = false;
    bool     loop_started_  = false;   // we started the master's RT loop
    uint16_t last_al_status_code_ = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
