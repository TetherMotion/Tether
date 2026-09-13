/**
 * @file ChainableInput.hpp
 * @brief Contract for bit-oriented digital-input terminals that can be
 *        chained into one shared logical address space
 *
 * Mirror of ChainableOutput.hpp for the input direction.  Any device
 * implementing IChainableInput can join a MultiInput chain: the chain
 * assigns every device a contiguous logical address range inside a single
 * PDO group's logical address space, so one LRW datagram per cycle
 * delivers the inputs of the whole chain.
 *
 * Bring-up uses the same two-phase split as the output side (see
 * ChainableOutput.hpp): prepareForLogicalExchange() per device, then the
 * chain builds the shared logical map, then mapLogicalAndEnterSafeOp()
 * programs each input FMMU, then requestOp().
 *
 * Input bits are read-only from the application's point of view — the
 * registered PDO buffer is updated by the cyclic exchange, so bit()/bits()
 * simply reflect the most recent received state.  Devices expose a flat,
 * position-independent bit interface (bitCount/bit/bits) so the chain can
 * present all inputs as one contiguous bit field.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/Beckhoff/ChainableOutput.hpp"   // Error, Result, DeviceIdentity, StartOptions

namespace EtherCAT {
namespace Beckhoff {

// ============================================================================
// IChainableInput — the contract a device must implement to join a chain
// ============================================================================

/**
 * @brief A single input field — one contiguous block of bits read from
 *        one device within a shared logical address space.
 *
 * Implement this interface for devices that are not PackedInput-shaped
 * (e.g. devices with a mailbox or multiple process-data SMs) and register
 * them with MultiInput via a DeviceMatcher factory or attach().
 */
class IChainableInput {
public:
    virtual ~IChainableInput() = default;

    // -- Topology / identity -------------------------------------------------

    /// Zero-based bus position of the bound slave.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of input bits this device contributes to the chain.
    virtual size_t bitCount() const = 0;

    /// Human-readable device name (identity name or discovered name).
    virtual const char* deviceName() const = 0;

    // -- Bit access ----------------------------------------------------------
    // Reads observe the registered PDO buffer as last written by the cyclic
    // exchange — they reflect the inputs as of the most recent cycle.

    /// Current state of an input bit (0-based within this device).
    virtual bool bit(size_t bit) const = 0;

    /// Current input bits (bit N = input N).
    virtual uint64_t bits() const = 0;

    // -- Chained bring-up (called by MultiInput) -----------------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, process-input sync manager registers, PRE-OP,
     *        and registration of the TxPDO entry in logical-address mode
     *        with the PDOManager covering this slave.
     *
     * The chain calls this for every device BEFORE building the shared
     * logical address map.
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program this device's input FMMU with the logical address the
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
