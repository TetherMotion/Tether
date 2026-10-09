/**
 * @file FoEWireFormat.hpp
 * @brief FoE wire-format structures.
 *
 * Split out of FoE.hpp.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
namespace FoE {

// ============================================================================
// Wire Format Structures
// ============================================================================

/**
 * @brief FoE header structure
 */
struct __attribute__((packed)) FoEHeader {
    uint8_t opcode;         ///< FoE opcode
    uint8_t reserved;       ///< Reserved (0)
    uint32_t packet_no_le;  ///< Packet/block number (little-endian)
};
static_assert(sizeof(FoEHeader) == 6, "FoEHeader must be 6 bytes");

/**
 * @brief FoE error response structure
 */
struct __attribute__((packed)) FoEErrorResponse {
    uint8_t opcode;         ///< Always 5 (ERROR)
    uint8_t reserved;
    uint32_t error_code_le; ///< Error code
};
static_assert(sizeof(FoEErrorResponse) == 6, "FoEErrorResponse must be 6 bytes");

} // namespace FoE
} // namespace EtherCAT
