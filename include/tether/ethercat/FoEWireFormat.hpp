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
struct FoEHeader {
    uint8_t opcode; /**< FoE opcode */
    uint8_t reserved; /**< Reserved (0) */
    uint32_t packet_no_le; /**< Packet/block number (little-endian) */
} __attribute__((packed));
static_assert(sizeof(FoEHeader) == 6, "FoEHeader must be 6 bytes");

/**
 * @brief FoE error response structure
 */
struct FoEErrorResponse {
    uint8_t opcode; /**< Always 5 (ERROR) */
    uint8_t reserved;
    uint32_t error_code_le; /**< Error code */
} __attribute__((packed));
static_assert(sizeof(FoEErrorResponse) == 6, "FoEErrorResponse must be 6 bytes");

} // namespace FoE
} // namespace EtherCAT
