/**
 * @file ICombinedIoTerminal.hpp
 * @brief Contract for combined packed-bit I/O devices that can be
 *        chained into one shared logical address space
 *
 * Any device implementing ICombinedIoTerminal can join a
 * MultiCombinedIoTerminal chain: the chain assigns every device a
 * contiguous logical address range inside a single PDO group's logical
 * address space, so one LRW datagram per cycle exchanges the inputs and
 * outputs of the whole chain.
 *
 * Devices carry two independent bit-channel directions: input(i) reads
 * the i-th 1-bit process-input entry, setOutput(i,on) writes the i-th
 * 1-bit process-output entry.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/Beckhoff/TerminalTypes.hpp"

namespace EtherCAT {
namespace Beckhoff {

class ICombinedIoTerminal {
public:
    virtual ~ICombinedIoTerminal() = default;

    // -- Identity --------------------------------------------------------------

    /// Zero-based bus position of this device.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of 1-bit input/output channels on this device.
    virtual size_t inputChannelCount()  const = 0;
    virtual size_t outputChannelCount() const = 0;

    /// Device name for logging/UI.
    virtual const char* deviceName() const = 0;

    // -- I/O ---------------------------------------------------------------------

    /// Latest state of packed input bit `i`.
    virtual bool input(size_t i) const = 0;

    /// Write packed output bit `i`.
    virtual void setOutput(size_t i, bool on) = 0;

    /// Read-back of packed output bit `i`.
    virtual bool output(size_t i) const = 0;

    /// Clear the whole output image.
    virtual void allOutputsOff() = 0;

    // -- Chained bring-up (called by MultiCombinedIoTerminal) ---------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, mailbox + process sync-manager registers,
     *        PRE-OP, PDO buffer registration in Logical mode and
     *        finalizeMapping().
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program the FMMUs from the assigned logical addresses and
     *        enter SAFE-OP.  Requires the chain to have built the
     *        logical address map first.
     */
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /// Request the OP state and wait for confirmation (or cancellation).
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff
} // namespace EtherCAT
