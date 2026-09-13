/**
 * @file IAnalogOutputTerminal.hpp
 * @brief Contract for analog-output terminals that can be chained into
 *        one shared logical address space
 *
 * Any device implementing IAnalogOutputTerminal can join a
 * MultiAnalogOutputTerminal chain: the chain assigns every device a
 * contiguous logical address range inside a single PDO group's logical
 * address space, so one LRW datagram per cycle exchanges the outputs of
 * the whole chain.
 *
 * Unlike the digital IOutputTerminal family these are mailbox devices —
 * bring-up includes SM0/SM1 mailbox configuration — and channels carry
 * 16- or 32-bit analog values instead of single bits.
 *
 * Implement this interface for devices that are not
 * AnalogOutputTerminal-shaped (e.g. multi-value or oversampling
 * terminals) and register them with MultiAnalogOutputTerminal via a
 * DeviceMatcher factory or attach().
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/Beckhoff/TerminalTypes.hpp"

namespace EtherCAT {
namespace Beckhoff {

class IAnalogOutputTerminal {
public:
    virtual ~IAnalogOutputTerminal() = default;

    // -- Identity --------------------------------------------------------------

    /// Zero-based bus position of this device.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of analog output channels on this device.
    virtual size_t channelCount() const = 0;

    /// Device name for logging/UI.
    virtual const char* deviceName() const = 0;

    // -- Outputs ----------------------------------------------------------------

    /**
     * @brief Set one channel's output value (raw field-bus units).
     *
     * The value is written into the PDO buffer and reaches the terminal on
     * the next cyclic exchange.  The raw encoding is the terminal's native
     * signed integer (typically ±32767 = full scale; e.g. an EL4134 writes
     * 0x7FFF for +10 V).  For channels wider than 16 bit the full int32 is
     * used.  Out-of-range channel indices are ignored.
     */
    virtual void setValue(size_t channel, int32_t value) = 0;

    /// Last value written to `channel` (0 initially).
    virtual int32_t value(size_t channel) const = 0;

    /// All output channels to 0.
    virtual void allZero() = 0;

    // -- Chained bring-up (called by MultiAnalogOutputTerminal) ------------------

    /**
     * @brief Everything up to (but excluding) the SAFE-OP transition:
     *        SII resolution, mailbox + process sync-manager registers,
     *        PRE-OP, RxPDO buffer registration in Logical mode and
     *        finalizeMapping().
     *
     * After this call the covering PDO manager knows this slave's
     * rxpdo_size, so the chain can build the shared logical address map.
     */
    virtual Result<> prepareForLogicalExchange() = 0;

    /**
     * @brief Program the output FMMU from the assigned logical address and
     *        enter SAFE-OP.  Requires the chain to have built the logical
     *        address map first.
     */
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /// Request the OP state and wait for confirmation (or cancellation).
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff
} // namespace EtherCAT
