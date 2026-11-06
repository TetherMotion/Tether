/**
 * @file AddressNotation.hpp
 * @brief Parse/write the address notations: "40001", "3x0005", "hr:10",
 *        "coil.3", "0x00FF", "40001.2" (bit-qualified).
 */
#pragma once

#include <string>
#include <string_view>

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/protocol/Address.hpp"

namespace tether::modbus {

/**
 * Notations (all → Address{table, offset[, bit]}):
 *   "40012"      5/6-digit PLC → table by first digit(s), 0-based offset
 *   "30012.3"    same, bit-qualified
 *   "hr:17"/"holding:17" / "ir:4" / "coil:9" / "di:2"
 *   "4x17" "3x17" "1x17" "0x17" (x = separator)
 *   "0x1F"       hex → holding register
 *   "17"         bare decimal → holding register
 */
Expected<Address> parseAddress(std::string_view text);
std::string       formatAddress(const Address& a);      // "hr:17" style
std::string       formatPlcAddress(const Address& a);   // "40018" style

} // namespace tether::modbus
