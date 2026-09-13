/**
 * @file IPositionInputTerminal.hpp
 * @brief Chainable interface for Beckhoff position/encoder input terminals
 *        (the EL5xxx family)
 *
 * One "channel" is one position counter — an SSI encoder input, an
 * incremental-encoder channel, a BiSS/EnDat axis — resolved from the
 * SII TxPDO list at bring-up.  Each channel exposes its value entries in
 * PDO order: value(0) is the position/counter, value(1) the latch (touch
 * probe) value where the PDO carries one; some devices expose additional
 * latch or timestamp fields (valueCount() > 2).
 *
 * Two reading conventions are offered because the families differ:
 *   - value()       sign-extends the field (incremental counters go
 *                   negative; the right choice for counting axes)
 *   - rawValue()    zero-extends it (SSI/BiSS singleturn data is unsigned)
 *
 * Channel numbering is 0-based throughout.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/Beckhoff/TerminalTypes.hpp"

namespace EtherCAT {
namespace Beckhoff {

class IPositionInputTerminal {
public:
    virtual ~IPositionInputTerminal() = default;

    /// Zero-based bus position.
    virtual uint16_t slaveIndex() const = 0;

    /// Number of position channels this terminal exposes.
    virtual size_t channelCount() const = 0;

    /// Device name (SII name when discovered, else the registry name).
    virtual const char* deviceName() const = 0;

    /// Number of >=16-bit value fields channel `ch` carries (0 when the
    /// layout is unresolved or `ch` is out of range).
    virtual size_t valueCount(size_t ch) const = 0;

    /**
     * @brief i-th value field of channel `ch`, sign-extended to int64.
     *
     * The right interpretation for counting axes (incremental counters
     * wrap and go negative).  0 when out of range.
     */
    virtual int64_t value(size_t ch, size_t i) const = 0;

    /**
     * @brief i-th value field of channel `ch`, zero-extended to uint64.
     *
     * The right interpretation for absolute encoders (SSI/BiSS/EnDat
     * position words are unsigned).  0 when out of range.
     */
    virtual uint64_t rawValue(size_t ch, size_t i) const = 0;

    /// Position/counter of channel `ch` — value(ch, 0).
    int64_t position(size_t ch) const { return value(ch, 0); }

    /// Raw position word — rawValue(ch, 0).
    uint64_t rawPosition(size_t ch) const { return rawValue(ch, 0); }

    /// First latch (touch-probe) value — value(ch, 1); 0 when absent.
    int64_t latch(size_t ch) const { return value(ch, 1); }

    /// Raw first latch — rawValue(ch, 1); 0 when absent.
    uint64_t rawLatch(size_t ch) const { return rawValue(ch, 1); }

    /// Whether channel `ch` carries a latch field (valueCount() > 1).
    bool hasLatch(size_t ch) const { return valueCount(ch) > 1; }

    /// Raw status word of channel `ch` (the packed status bits preceding
    /// the value fields, when at least 16 bits wide); 0 when absent.
    virtual uint16_t status(size_t ch) const = 0;

    /// Whether channel `ch` carries a status prefix.
    virtual bool hasStatus(size_t ch) const = 0;

    /// Bit width of the position field of channel `ch` (16…64; 0 when
    /// unresolved or out of range).
    virtual size_t channelBits(size_t ch) const = 0;

    // -- Chained operation ------------------------------------------------------

    /// Prepare for chained logical exchange (SII + mailbox + SM + PRE-OP
    /// + PDO registration in logical mode; ends before SAFE-OP).
    virtual Result<> prepareForLogicalExchange() = 0;

    /// Program the input FMMU from the shared logical map and enter
    /// SAFE-OP.
    virtual Result<> mapLogicalAndEnterSafeOp() = 0;

    /// SAFE-OP -> OP transition.
    virtual Result<> requestOp(int timeout_ms) = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
