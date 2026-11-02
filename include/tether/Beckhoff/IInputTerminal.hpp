/**
 * @file IInputTerminal.hpp
 * @brief Contract for bit-oriented digital-input terminals that can be
 *        chained into one shared logical address space
 *
 * Mirror of IOutputTerminal.hpp for the input direction.  Any device
 * implementing IInputTerminal can join a MultiInputTerminal chain: the chain
 * assigns every device a contiguous logical address range inside a single
 * PDO group's logical address space, so one LRW datagram per cycle
 * delivers the inputs of the whole chain.
 *
 * Bring-up uses the same two-phase split as the output side (see
 * IOutputTerminal.hpp): prepareForLogicalExchange() per device, then the
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

#include "tether/Beckhoff/TerminalTypes.hpp"   // Error, Result, DeviceIdentity, StartOptions

namespace EtherCAT {
namespace Beckhoff {

// ============================================================================
// IInputTerminal — the contract a device must implement to join a chain
// ============================================================================

/**
 * @brief A single input field — one contiguous block of bits read from
 *        one device within a shared logical address space.
 *
 * Implement this interface for devices that are not InputTerminal-shaped
 * (e.g. devices with a mailbox or multiple process-data SMs) and register
 * them with MultiInputTerminal via a DeviceMatcher factory or attach().
 */
class IInputTerminal {
public:
    virtual ~IInputTerminal() = default;

    // -- Topology / identity -------------------------------------------------

    /**
     * @brief Zero-based bus position of the bound slave.
     * @return slave position on the EtherCAT bus.
     */
    virtual uint16_t slaveIndex() const = 0;

    /**
     * @brief Number of input bits this device contributes to the chain.
     * @return input channel count.
     */
    virtual size_t bitCount() const = 0;

    /**
     * @brief Human-readable device name (identity name or discovered name).
     * @return static string valid for the device's lifetime.
     */
    virtual const char* deviceName() const = 0;

    // -- Bit access ----------------------------------------------------------
    // Reads observe the registered PDO buffer as last written by the cyclic
    // exchange — they reflect the inputs as of the most recent cycle.

    /**
     * @brief Current state of an input bit (0-based within this device).
     * @param bit  Channel index; must be < bitCount().
     * @return sampled bit value from the last cyclic exchange.
     */
    virtual bool bit(size_t bit) const = 0;

    /**
     * @brief Current input bits (bit N = input N).
     * @return all channel states packed into a 64-bit mask.
     */
    virtual uint64_t bits() const = 0;

    // -- Chained bring-up (called by MultiInputTerminal) -----------------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, process-input sync manager registers, PRE-OP,
     *        and registration of the TxPDO entry in logical-address mode
     *        with the PDOManager covering this slave.
     *
     * The chain calls this for every device BEFORE building the shared
     * logical address map.
     *
     * @return success or the first failing step's error.
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program this device's input FMMU with the logical address the
     *        covering LogicalAddressManager assigned to this slave, then
     *        enter SAFE-OP.
     *
     * The chain calls this for every device AFTER the shared logical
     * address map has been built.
     *
     * @return success or the FMMU/state-transition error.
     */
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /**
     * @brief Request the SAFE-OP -> OP transition and wait for confirmation.
     * @param timeout_ms  Maximum wait for the OP state in milliseconds.
     * @return success, or timeout/error when the slave refuses OP.
     */
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
