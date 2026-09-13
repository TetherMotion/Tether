/**
 * @file IOutputTerminal.hpp
 * @brief Contract for bit-oriented digital-output terminals that can be
 *        chained into one shared logical address space
 *
 * Any device implementing IOutputTerminal can join a MultiOutputTerminal
 * chain: the chain assigns every device a contiguous logical address range
 * inside a single PDO group's logical address space, so one LRW datagram
 * per cycle exchanges the outputs of the whole chain.
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

#include "tether/Beckhoff/TerminalTypes.hpp"

namespace EtherCAT {
namespace Beckhoff {

// ============================================================================
// IOutputTerminal — the contract a device must implement to join a chain
// ============================================================================

/**
 * @brief A single output field — one contiguous block of bits written to
 *        one device within a shared logical address space.
 *
 * Implement this interface for devices that are not OutputTerminal-shaped
 * (e.g. devices with a mailbox or multiple process-data SMs) and register
 * them with MultiOutputTerminal via a DeviceMatcher factory or attach().
 */
class IOutputTerminal {
public:
    virtual ~IOutputTerminal() = default;

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

    // -- Chained bring-up (called by MultiOutputTerminal) --------------------

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
