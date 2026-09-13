/**
 * @file ChainableOutput.hpp
 * @brief Contract for bit-oriented digital-output terminals that can be
 *        chained into one shared logical address space
 *
 * Any device implementing IChainableOutput can join a MultiOutput chain:
 * the chain assigns every device a contiguous logical address range inside
 * a single PDO group's logical address space, so one LRW datagram per cycle
 * exchanges the outputs of the whole chain.
 *
 * The chain bring-up is split into two phases so that the chain controller
 * can interpose the logical-map build between PDO registration and FMMU
 * programming:
 *
 *   1. prepareForLogicalExchange() — per device: resolve SII data, write the
 *      process-output sync manager, enter PRE-OP, register the PDO entry in
 *      logical-address mode with the PDOManager covering the slave, and
 *      finalize the mapping.
 *   2. (chain)  — builds the shared logical address map once across all
 *      devices.
 *   3. mapLogicalAndEnterSafeOp() — per device: look up the assigned logical
 *      address in the covering LogicalAddressManager, program the device's
 *      output FMMU, enter SAFE-OP.
 *   4. requestOp() — per device: SAFE-OP -> OP.
 *
 * Devices also expose a flat, position-independent bit interface
 * (bitCount/setBit/bit/setBits/bits/allOff) so the chain can present all
 * outputs as one contiguous bit field.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Shared error type
// ============================================================================

enum class Error : uint8_t {
    Ok = 0,
    NoDeviceFound,          ///< detect()/findFirst(): no matching device on the bus
    WrongDevice,            ///< slave identity does not match vendor/product
    SlaveIndexOutOfRange,   ///< index exceeds the PDO manager's slave table
    SmConfigFailed,         ///< could not write the sync-manager registers
    PdoRegistrationFailed,  ///< add_rxpdo() rejected the buffer
    FmmuConfigFailed,       ///< could not program the output FMMU
    LogicalMapMissing,      ///< no logical address was assigned to this slave
    PreOpFailed,            ///< INIT -> PRE-OP transition failed
    SafeOpFailed,           ///< PRE-OP -> SAFE-OP transition failed
    OpRequestFailed,        ///< SAFE-OP -> OP request could not be sent
    OpTimeout,              ///< slave did not reach OP in time
    LoopStartFailed,        ///< master realtime loop failed to start
    TooManyBits,            ///< chain exceeds the MaxBits capacity
    Cancelled,              ///< aborted via master.requestCancel()
};

/// Human-readable description of an Error value.
const char* errorToString(Error e);

/// Expected-like result type used by all fallible operations.
template <typename T = void>
using Result = std::expected<T, Error>;

// ============================================================================
// Device identity
// ============================================================================

/**
 * @brief Static description of a supported packed-bit terminal
 *        (input or output direction).
 *
 * `vendor_id`/`product_code` are matched against discovery results.
 * `num_bits` declares the packed bit width; when it is 0 the width is
 * derived from the SII PDO bit sum during bring-up (MultiOutput/
 * MultiInput need a non-zero declared width to lay out the flat bit
 * space before start()).
 */
struct DeviceIdentity {
    uint32_t    vendor_id;
    uint32_t    product_code;
    uint16_t    num_bits    = 0;
    const char* name        = nullptr;
};

// ============================================================================
// Start options (shared by single-device and Multi bring-up)
// ============================================================================

struct StartOptions {
    /// When true (default), the driver installs a motion-control callback
    /// that exchanges the PDO manager(s) covering the device(s) and starts
    /// the master's realtime loop if it is not already running.  Set false
    /// when the application manages its own cyclic exchange.
    bool     manage_realtime_loop = true;
    /// Cycle period used when the driver starts the RT loop (microseconds).
    uint32_t cycle_period_us      = 1000;
    /// How long to wait for the OP state confirmation (milliseconds).
    int      op_timeout_ms        = 5000;
    /// Base of the chain's logical address space (MultiOutput only).
    /// 0 = auto-select a non-overlapping range.
    uint32_t base_logical_addr    = 0;
};

// ============================================================================
// IChainableOutput — the contract a device must implement to join a chain
// ============================================================================

/**
 * @brief A single output field — one contiguous block of bits written to
 *        one device within a shared logical address space.
 *
 * Implement this interface for devices that are not PackedOutput-shaped
 * (e.g. devices with a mailbox or multiple process-data SMs) and register
 * them with MultiOutput via a DeviceMatcher factory or attach().
 */
class IChainableOutput {
public:
    virtual ~IChainableOutput() = default;

    // -- Topology / identity -------------------------------------------------

    /// Zero-based bus position of the bound slave.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of output bits this device contributes to the chain.
    virtual size_t bitCount() const = 0;

    /// Human-readable device name (identity name or discovered name).
    virtual const char* deviceName() const = 0;

    // -- Bit access ----------------------------------------------------------
    // Setters only touch the registered PDO buffer; the cyclic exchange
    // publishes them on the next cycle.

    /// Set one output bit (0-based within this device).
    virtual void setBit(size_t bit, bool on) = 0;

    /// Current requested state of an output bit.
    virtual bool bit(size_t bit) const = 0;

    /// Write all output bits at once (bit N = output N).
    virtual void setBits(uint64_t bits) = 0;

    /// Currently requested output bits (bit N = output N).
    virtual uint64_t bits() const = 0;

    /// All output bits off.
    virtual void allOff() = 0;

    // -- Chained bring-up (called by MultiOutput) ----------------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, process-output SM registers, PRE-OP, and
     *        registration of the PDO entry in logical-address mode with the
     *        PDOManager covering this slave.
     *
     * The chain calls this for every device BEFORE building the shared
     * logical address map.
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program this device's output FMMU with the logical address the
     *        covering LogicalAddressManager assigned to this slave, then
     *        enter SAFE-OP.
     *
     * The chain calls this for every device AFTER the shared logical
     * address map has been built.
     */
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /**
     * @brief Request the SAFE-OP -> OP transition and wait for confirmation.
     */
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
