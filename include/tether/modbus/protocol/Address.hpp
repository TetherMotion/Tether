/**
 * @file Address.hpp
 * @brief Table-qualified address and half-open range types.
 *
 * Offsets are always wire (0-based). PLC-notation conversion lives in
 * typed/AddressNotation.hpp; this header is protocol-layer only.
 */
#pragma once

#include <cstdint>
#include <optional>

#include "tether/modbus/protocol/TableId.hpp"

namespace tether::modbus {

/**@brief Table-qualified 0-based element address. */
struct Address {
    TableId  table = TableId::HoldingRegister;
    uint16_t offset = 0;
    /// Bit index when parsed from a `addr.N` notation (0xFF = none).
    uint8_t  bit = 0xFF;

    constexpr bool hasBit() const { return bit != 0xFF; }
    constexpr uint16_t bitMask() const {
        return hasBit() ? static_cast<uint16_t>(1u << bit) : 0;
    }
    constexpr bool operator==(const Address&) const = default;
    constexpr bool operator<(const Address& o) const {
        if (table != o.table) return static_cast<int>(table) < static_cast<int>(o.table);
        return offset < o.offset;
    }
};

/**@brief Half-open [offset, offset+qty) range in one table. */
struct Range {
    TableId  table  = TableId::HoldingRegister;
    uint16_t offset = 0;
    uint16_t qty    = 1;

    /// Exclusive end bound (may equal 65536).
    constexpr uint32_t end() const { return uint32_t(offset) + qty; }
    constexpr bool contains(uint16_t off) const {
        return off >= offset && off < end();
    }
    constexpr bool intersects(const Range& o) const {
        return table == o.table && offset < o.end() && o.offset < end();
    }
    constexpr bool operator==(const Range&) const = default;

    constexpr static std::optional<Range> make(TableId t, uint16_t off,
                                               uint16_t qty) {
        if (qty == 0 || uint32_t(off) + qty > 65536) return std::nullopt;
        return Range{t, off, qty};
    }
};

} // namespace tether::modbus
