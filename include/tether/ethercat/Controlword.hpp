/**
 * @file Controlword.hpp
 * @brief Helpers to describe/format CiA 402 ControlWord values
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace EtherCAT {

/**
 * @brief Produce a short, human-readable description of a CiA 402 ControlWord.
 *
 * Returns a std::string containing the formatted description. Example output:
 *   "0x000F (EnableOperation) [SwitchOn,EnVolt,!QStop,EnOp]"
 */
std::string describeControlword(uint16_t cw);

} // namespace EtherCAT
