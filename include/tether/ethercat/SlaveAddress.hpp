/**
 * @file SlaveAddress.hpp
 * @brief EtherCAT slave addressing types (physical / configured / logical).
 *
 * Split out of Master.hpp.
 */

#pragma once

#include <cstdint>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

struct PhysicalAddress {
    constexpr explicit PhysicalAddress(unsigned int slave_position_in)
        : slave_position(static_cast<uint16_t>(slave_position_in)) {}

    constexpr uint16_t raw() const { return static_cast<uint16_t>(0u - slave_position); }
    constexpr uint16_t slavePosition() const { return slave_position; }

private:
    uint16_t slave_position;
};

struct LogicalAddress {
    constexpr explicit LogicalAddress(unsigned int configured_address_in)
        : configured_address(static_cast<uint16_t>(configured_address_in)) {}

    constexpr uint16_t raw() const { return configured_address; }

private:
    uint16_t configured_address;
};

class SlaveAddress {
public:
    enum class Kind : uint8_t {
        Physical,
        Logical,
    };

    constexpr SlaveAddress(unsigned int slave_position_in)
        : kind_(Kind::Physical), value_(static_cast<uint16_t>(slave_position_in)) {}

    constexpr SlaveAddress(PhysicalAddress physical_address)
        : kind_(Kind::Physical), value_(physical_address.slavePosition()) {}

    constexpr SlaveAddress(LogicalAddress logical_address)
        : kind_(Kind::Logical), value_(logical_address.raw()) {}

    constexpr bool isPhysical() const { return kind_ == Kind::Physical; }
    constexpr bool isLogical() const { return kind_ == Kind::Logical; }
    constexpr uint16_t raw() const {
        return isPhysical() ? static_cast<uint16_t>(0u - value_) : value_;
    }
    constexpr uint16_t slavePosition() const { return value_; }
    constexpr Kind kind() const { return kind_; }

private:
    Kind kind_;
    uint16_t value_;
};

struct RegisterAddress {
    constexpr explicit RegisterAddress(uint16_t value_in) : value(value_in) {}
    constexpr uint16_t raw() const { return value; }

private:
    uint16_t value;
};

} // namespace EtherCAT
