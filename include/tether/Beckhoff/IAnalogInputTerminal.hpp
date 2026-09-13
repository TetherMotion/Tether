/**
 * @file IAnalogInputTerminal.hpp
 * @brief Contract for analog-input terminals that can be chained into
 *        one shared logical address space
 *
 * Any device implementing IAnalogInputTerminal can join a
 * MultiAnalogInputTerminal chain: the chain assigns every device a
 * contiguous logical address range inside a single PDO group's logical
 * address space, so one LRW datagram per cycle reads the inputs of the
 * whole chain.
 *
 * Unlike the digital IInputTerminal family these are mailbox devices —
 * bring-up includes SM0/SM1 mailbox configuration — and channels carry
 * 16- or 32-bit analog values plus a per-channel status word instead of
 * single bits.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/Beckhoff/TerminalTypes.hpp"

namespace EtherCAT {
namespace Beckhoff {

class IAnalogInputTerminal {
public:
    virtual ~IAnalogInputTerminal() = default;

    // -- Identity --------------------------------------------------------------

    /// Zero-based bus position of this device.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of analog input channels on this device.
    virtual size_t channelCount() const = 0;

    /// Device name for logging/UI.
    virtual const char* deviceName() const = 0;

    // -- Inputs -----------------------------------------------------------------

    /**
     * @brief Latest value of `channel` (raw field-bus units).
     *
     * Returns the channel's native signed integer as transmitted in the
     * PDO — typically ±32767 = full scale for 16-bit terminals (an EL3104
     * reports 0x7FFF at +10 V), sign-extended int32 for wider channels.
     * Reflects the last completed cyclic exchange; out-of-range channels
     * return 0.
     */
    virtual int32_t value(size_t channel) const = 0;

    /**
     * @brief Raw status word of `channel` (0 when the channel's default
     *        PDO carries no status field).
     *
     * Bit layout is terminal-family specific — commonly bit0 underrange,
     * bit1 overrange, bit6 error, and the TxPDO toggle in the high byte.
     * Consult the terminal documentation.
     */
    virtual uint16_t status(size_t channel) const = 0;

    /// True when `channel` has a status field in its process image.
    virtual bool hasStatus(size_t channel) const = 0;

    // -- Chained bring-up (called by MultiAnalogInputTerminal) -------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, mailbox + process sync-manager registers,
     *        PRE-OP, TxPDO buffer registration in Logical mode and
     *        finalizeMapping().
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program the input FMMU from the assigned logical address and
     *        enter SAFE-OP.  Requires the chain to have built the logical
     *        address map first.
     */
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /// Request the OP state and wait for confirmation (or cancellation).
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff
} // namespace EtherCAT
