// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

/**
 * @file SDO.hpp
 * @brief CiA 301 SDO (Service Data Object) object-dictionary definitions
 *
 * Contains the SDO Server/Client parameter base indices and subindex
 * definitions.  New code should use `EtherCAT::SDO` constants; backward
 * compatible `CiA301` aliases are provided at the end of this header.
 */

namespace EtherCAT { namespace SDO {

/// SDO Server Parameters (0x1200 - 0x127F)
constexpr uint16_t kSDOServerParameterBase = 0x1200;

/// SDO Client Parameters (0x1280 - 0x12FF)
constexpr uint16_t kSDOClientParameterBase = 0x1280;

namespace ParameterSub {
    constexpr uint8_t NumberOfEntries       = 0x00;
    constexpr uint8_t CobIdClientToServer   = 0x01;
    constexpr uint8_t CobIdServerToClient   = 0x02;
    constexpr uint8_t NodeIdOfServer        = 0x03;
}

}} // namespace EtherCAT::SDO

// ---------------------------------------------------------------------------
// Backward-compatible CiA301 aliases
// ---------------------------------------------------------------------------

namespace CiA301 {

constexpr uint16_t SDOServerParameterBase = EtherCAT::SDO::kSDOServerParameterBase;
constexpr uint16_t SDOClientParameterBase = EtherCAT::SDO::kSDOClientParameterBase;

// Keep the original sub-namespace name for existing callers
namespace SDOParameterSub = EtherCAT::SDO::ParameterSub;

} // namespace CiA301
