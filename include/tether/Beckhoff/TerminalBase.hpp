/**
 * @file TerminalBase.hpp
 * @brief Shared bring-up machinery for the Beckhoff terminal drivers
 *
 * Every process-data terminal driver in this library performs the same
 * sequence: read SII (identity, sync managers, PDOs, mailbox), verify
 * identity, program the process-data sync managers, configure or
 * declare-skip the mailbox, transition PRE-OP, register PDO buffers,
 * optionally program an FMMU from a shared logical address map, and
 * transition SAFE-OP -> OP.  TerminalBase implements that machinery once;
 * the concrete drivers (analog, position, stepper, safety, ...) only add
 * their process-image interpretation.
 *
 * The class is protected-inheritance building material — it is not a
 * public interface on its own.  Subclasses compose the protected step
 * functions into their prepare()/mapLogicalAndEnterSafeOp() contract.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/TerminalTypes.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;
class Slave;

namespace Beckhoff {

class TerminalBase {
public:
    // -- Shared queries --------------------------------------------------------

    /// Zero-based bus position.
    uint16_t slaveIndex() const { return slave_index_; }

    /// Device name (SII name when discovered, else registry name).
    const char* deviceName() const;

    /// True after the SAFE-OP transition succeeded.
    bool configured() const { return configured_; }

    /// Live-read the application-layer (ESM) state register.
    SlaveState alState();

    /// True when the terminal is in OP.
    bool operational() { return alState() == SlaveState::OP; }

    /// AL status code captured by the last failed state transition (0 = none).
    uint16_t lastAlStatusCode() const { return last_al_status_code_; }

    /// The declared device identity.
    const DeviceIdentity& identity() const { return identity_; }

    /// Logical address assigned to this device (chained operation only).
    uint32_t logicalAddress() const { return logical_addr_; }

    /// Raw input process image (SM3 region) as last exchanged.
    std::span<const uint8_t> rawInput() const { return in_buf_; }

    /// Raw output process image (SM2 region) — what will be sent next cycle.
    std::span<const uint8_t> rawOutput() const { return out_buf_; }

    /// Writable view of the output process image (scratch/control regions).
    std::span<uint8_t> rawOutput() { return out_buf_; }

    /// The underlying slave object — for SDO configuration (operating
    /// modes, encoder coding, PTO ramp settings, ...) before start().
    Slave& slave();

    /// Stop the master's realtime loop if this driver started it.
    void stop();

    virtual ~TerminalBase();

    TerminalBase(TerminalBase&&) noexcept            = default;
    TerminalBase& operator=(TerminalBase&&) noexcept = default;
    TerminalBase(const TerminalBase&)                = delete;
    TerminalBase& operator=(const TerminalBase&)     = delete;

protected:
    /// A resolved process-data sync-manager region.
    struct SmRegion {
        uint8_t  channel   = 0;      ///< SM channel index (2 or 3 normally)
        uint16_t phys_addr = 0;      ///< physical start address in the ESC
        uint16_t length    = 0;      ///< region length in bytes
        uint8_t  ctrl      = 0;      ///< SM control byte from SII
        bool     enabled   = false;  ///< region exists and is enabled
        uint16_t first_pdo = 0;      ///< first PDO index assigned to this SM
    };

    TerminalBase(Master& master, uint16_t slave_index,
                 const DeviceIdentity& identity);

    /// Bind to an already-discovered slave — reuses its SII data.
    TerminalBase(Master& master, const DiscoveredSlave& slave,
                 const DeviceIdentity& identity);

    // -- Bring-up steps (composed by subclasses) --------------------------------

    /**
     * @brief Populate `info_` with a deep discovery (vendor, product, names,
     *        sync managers, TxPDOs, RxPDOs, mailbox config) when the bound
     *        DiscoveredSlave did not already carry that data.
     */
    void fetchDiscovery();

    /**
     * @brief Verify the discovered identity against `identity_` and publish
     *        the device name onto the slave object.  When `info_` carries no
     *        vendor/product data the check is skipped (trusting the caller).
     * @return false on a vendor/product mismatch (logs WrongDevice context).
     */
    bool verifyIdentity();

    /**
     * @brief Resolve the enabled process-data sync managers from SII into
     *        `sm_in_`/`sm_out_` (channel, physical address, length, control
     *        byte, first assigned PDO index).
     */
    void resolveSyncManagers();

    /// Write all enabled process-data SM regions into the slave's
    /// SyncManagerConfig table (they are flushed to the ESC by the
    /// following configureMailbox()/flushSyncManagers() call).
    void stageSyncManagers();

    /**
     * @brief Configure the mailbox from SII; when SII reports none, declare
     *        the slave mailbox-less; on mailbox-config failure fall back to
     *        assumeMailboxAlreadyConfigured().  Always satisfies the PRE-OP
     *        mailbox gate afterwards.
     */
    void configureMailboxOrDeclare();

    /// Flush all staged sync-manager registers to the ESC.
    void flushSyncManagers();

    /// INIT -> PRE-OP transition.
    Result<> enterPreOp();

    /**
     * @brief Size the PDO buffers to the resolved SM regions and register
     *        them in the given address mode, then finalizeMapping().
     *        A region is registered only when enabled && length > 0.
     */
    Result<> registerProcessData(PDO::PDOAddressMode mode);

    /// assumePDOAlreadyConfigured() + PRE-OP -> SAFE-OP.
    Result<> enterSafeOp();

    /**
     * @brief Program one FMMU per enabled direction from the shared logical
     *        address map, then enter SAFE-OP.
     *
     * `logical_addr_` is set to the *output* region's logical address when
     * present, else the input region's.
     */
    Result<> programFmmuAndEnterSafeOp();

    /// SAFE-OP -> OP request + wait, honouring cancellation.
    Result<> requestOp(int timeout_ms);

    /**
     * @brief Start the master's realtime loop with a default
     *        exchange-all callback unless it is already running or
     *        `opts.manage_realtime_loop` is false.
     * @return Ok, or Error::LoopStartFailed.
     */
    Result<> startLoopIfManaged(const StartOptions& opts);

    /// Poll AL status until `target` or timeout; honours master cancellation.
    bool waitAlState(SlaveState target, int timeout_ms);

    /// Log-prefixed identity string ("<name> (#N)") for messages.
    std::string logPrefix() const;

    // -- State -----------------------------------------------------------------

    Master*                        master_;
    uint16_t                       slave_index_;
    DeviceIdentity                 identity_;
    std::optional<DiscoveredSlave> info_;

    /// Registered PDO buffers.  Heap-allocated so the pointers stay valid
    /// when the driver object is moved.
    std::vector<uint8_t> in_buf_;    ///< SM3 process image
    std::vector<uint8_t> out_buf_;   ///< SM2 process image

    SmRegion sm_in_;                 ///< process-input region (SM3 normally)
    SmRegion sm_out_;                ///< process-output region (SM2 normally)

    bool     prepared_      = false;  // PDO buffers registered (any mode)
    bool     configured_    = false;  // reached SAFE-OP
    bool     loop_started_  = false;  // we started the master's RT loop
    uint32_t logical_addr_  = 0;
    uint16_t last_al_status_code_ = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
