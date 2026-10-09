/**
 * @file EoEWireFormat.hpp
 * @brief EoE wire-format structures.
 *
 * Split out of EoE.hpp.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
namespace EoE {

// ============================================================================
// Wire Format Structures
// ============================================================================

/**
 * @brief EoE header structure
 * 
 * 4-byte header at the start of each EoE mailbox message.
 */
struct __attribute__((packed)) EoEHeader {
    // First 16 bits
    uint16_t fragment_number : 6;   ///< Fragment number (0-63)
    uint16_t offset_buffer : 6;     ///< Offset or buffer
    uint16_t frame_number : 4;      ///< Frame number for reassembly
    
    // Second 16 bits
    uint16_t complete : 1;          ///< Last fragment flag
    uint16_t port : 4;              ///< Port number
    uint16_t time_appended : 1;     ///< Timestamp appended
    uint16_t time_request : 1;      ///< Timestamp requested
    uint16_t reserved : 5;          ///< Reserved
    uint16_t frame_type : 4;        ///< Frame type (EoEFrameType)
};
static_assert(sizeof(EoEHeader) == 4, "EoEHeader must be 4 bytes");

/**
 * @brief EoE Set IP request structure
 */
struct __attribute__((packed)) EoESetIPRequest {
    uint32_t flags;             ///< Which parameters to set
    uint32_t ip_address;        ///< IP address
    uint32_t subnet_mask;       ///< Subnet mask
    uint32_t gateway;           ///< Default gateway
    uint32_t dns_server;        ///< DNS server
    char dns_name[32];          ///< DNS name
};

/**
 * @brief EoE Set IP flags
 */
enum class EoEIPFlags : uint32_t {
    MAC_INCLUDED    = 0x01,
    IP_INCLUDED     = 0x02,
    SUBNET_INCLUDED = 0x04,
    GATEWAY_INCLUDED = 0x08,
    DNS_INCLUDED    = 0x10,
    DNS_NAME_INCLUDED = 0x20,
};

} // namespace EoE

} // namespace EtherCAT
