/**
 * @file RawWireFormat.hpp
 * @brief EtherCAT wire-format structures (packed)
 *
 * Extracted from internal.hpp. Contains all packed protocol structures:
 * - EthernetHeader, EtherCATFrameHeader, EtherCATDatagramHeader
 * - EepromCmd, SyncManagerRegs, MbxHeader, CoeHeader
 * - SDO request/response structures
 * - EtherCATScanFrame
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "tether/platform/Platform.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SMRegisters.hpp"

namespace EtherCAT {
namespace Raw {

using EtherCATCommand = EtherCAT::Command;
using RxDatagram = EtherCAT::RxDatagram;

// ============================================================================
// Byte Order Utilities
// ============================================================================

static inline uint16_t bswap16(uint16_t v) { return __builtin_bswap16(v); }
static inline uint32_t bswap32(uint32_t v) { return __builtin_bswap32(v); }

static inline uint16_t be16(uint16_t host) { return bswap16(host); }
static inline uint16_t host_to_be16(uint16_t host) { return be16(host); }
static inline uint32_t host_to_be32(uint32_t host) { return bswap32(host); }
static inline uint16_t le16_to_host(uint16_t le) { return le; }
static inline uint16_t host_to_le16(uint16_t host) { return host; }
static inline uint32_t le32_to_host(uint32_t le) { return le; }
static inline uint32_t host_to_le32(uint32_t host) { return host; }

// ============================================================================
// EtherType
// ============================================================================

enum class EtherType : uint16_t {
    EtherCAT = 0x88A4,
    IPv4     = 0x0800,
};

constexpr uint16_t kEtherTypeIPv4 = 0x0800;

// ============================================================================
// EtherCAT-over-UDP encapsulation (ETG.1000.3)
// EtherCAT frames tunneled via IPv4/UDP, destination port 0x88A4 (34980).
// ============================================================================

constexpr uint16_t kEtherCATOverUdpPort = 0x88A4; // 34980
constexpr size_t   kUdpEncapOverhead    = 28;     // IPv4 header (20) + UDP header (8)

struct __attribute__((packed)) IPv4Header {
    uint8_t  version_ihl;       // 0x45 (version 4, IHL 5 = 20 bytes)
    uint8_t  tos;               // 0
    uint16_t total_length_be;   // IP header + UDP header + EtherCAT payload
    uint16_t identification_be; // Per-packet incrementing ID
    uint16_t flags_fragment_be; // 0x4000 = Don't Fragment, no offset
    uint8_t  ttl;               // 64
    uint8_t  protocol;          // 17 = UDP
    uint16_t checksum_be;       // Header checksum (computed)
    uint32_t src_ip_be;         // Source IP (network byte order)
    uint32_t dst_ip_be;         // Destination IP (network byte order)
};
static_assert(sizeof(IPv4Header) == 20, "IPv4Header must be 20 bytes");

struct __attribute__((packed)) UDPHeader {
    uint16_t src_port_be;  // Source port (network byte order)
    uint16_t dst_port_be;  // Destination port (network byte order)
    uint16_t length_be;    // UDP header + payload (network byte order)
    uint16_t checksum_be;  // 0 = not computed (optional for IPv4 UDP)
};
static_assert(sizeof(UDPHeader) == 8, "UDPHeader must be 8 bytes");

// ============================================================================
// Wire-Format Structures (Packed)
// ============================================================================

struct __attribute__((packed)) EthernetHeader {
    uint8_t dst[6];
    uint8_t src[6];
    uint16_t etherType_be;
};
static_assert(sizeof(EthernetHeader) == 14, "EthernetHeader must be 14 bytes");

struct __attribute__((packed)) EtherCATFrameHeaderBits {
    uint16_t length : 11;
    uint16_t reserved : 1;
    uint16_t type : 4;
};
static_assert(sizeof(EtherCATFrameHeaderBits) == 2, "EtherCATFrameHeaderBits must be 2 bytes");

union __attribute__((packed)) EtherCATFrameHeader {
    EtherCATFrameHeaderBits bits;
    uint16_t raw_le;
};
static_assert(sizeof(EtherCATFrameHeader) == 2, "EtherCATFrameHeader must be 2 bytes");

struct __attribute__((packed)) EtherCATDatagramLenFlagsBits {
    uint16_t length : 11;
    uint16_t reserved : 3;
    uint16_t c : 1;
    uint16_t m : 1;
};
static_assert(sizeof(EtherCATDatagramLenFlagsBits) == 2, "EtherCATDatagramLenFlagsBits must be 2 bytes");

union __attribute__((packed)) EtherCATDatagramLenFlags {
    EtherCATDatagramLenFlagsBits bits;
    uint16_t raw_le;
};
static_assert(sizeof(EtherCATDatagramLenFlags) == 2, "EtherCATDatagramLenFlags must be 2 bytes");

struct __attribute__((packed)) EtherCATDatagramHeader {
    EtherCATCommand cmd;
    uint8_t idx;
    uint16_t adp_le;
    uint16_t ado_le;
    EtherCATDatagramLenFlags lenFlags;
    uint16_t irq_le;
};
static_assert(sizeof(EtherCATDatagramHeader) == 10, "EtherCATDatagramHeader must be 10 bytes");

struct __attribute__((packed)) EtherCATSingleDgramFrameHeader {
    EthernetHeader eth;
    EtherCATFrameHeader ec;
    EtherCATDatagramHeader dg;
};

static constexpr size_t kMaxEtherCATPayloadPerFrame = 1498;
static constexpr size_t kDatagramOverhead = sizeof(EtherCATDatagramHeader) + sizeof(uint16_t);

struct __attribute__((packed)) EepromCmd {
    uint16_t comm_le;
    uint16_t addr_le;
    uint16_t d2_le;
};
static_assert(sizeof(EepromCmd) == 6, "EepromCmd must be 6 bytes");

struct __attribute__((packed)) SyncManagerRegs {
    uint16_t physStart_le;
    uint16_t length_le;
    EtherCAT::SyncManager::SMControlReg  control;
    EtherCAT::SyncManager::SMStatusReg   status;
    EtherCAT::SyncManager::SMActivateReg activate;
    EtherCAT::SyncManager::SMPDICtrlReg  pdiControl;
};
static_assert(sizeof(SyncManagerRegs) == 8, "SyncManagerRegs must be 8 bytes");

struct __attribute__((packed)) MbxHeader {
    uint16_t length_le;
    uint16_t address_le;
    uint8_t priority;
    uint8_t mbxtype;
};
static_assert(sizeof(MbxHeader) == 6, "MbxHeader must be 6 bytes");

static inline uint8_t mbx_type_with_cnt(uint8_t type_low_nibble, uint8_t cnt) {
    return static_cast<uint8_t>(((cnt & 0x07u) << 4) | (type_low_nibble & 0x0Fu));
}

struct __attribute__((packed)) CoeHeader {
    uint16_t raw_le;
};
static_assert(sizeof(CoeHeader) == 2, "CoeHeader must be 2 bytes");

static inline uint16_t coe_make_raw(uint16_t number, uint8_t service) {
    return static_cast<uint16_t>((number & 0x01FFu) | ((static_cast<uint16_t>(service) & 0x0Fu) << 12));
}

struct __attribute__((packed)) SdoInitUploadReq {
    uint8_t cmd;
    uint16_t index_le;
    uint8_t sub;
    uint8_t reserved[4];
};
static_assert(sizeof(SdoInitUploadReq) == 8, "SdoInitUploadReq must be 8 bytes");

struct __attribute__((packed)) SdoInitUploadRes {
    uint8_t cmd;
    uint16_t index_le;
    uint8_t sub;
    uint32_t data_or_size_le;
};
static_assert(sizeof(SdoInitUploadRes) == 8, "SdoInitUploadRes must be 8 bytes");

struct __attribute__((packed)) SdoInitDownloadReq {
    uint8_t cmd;
    uint16_t index_le;
    uint8_t sub;
    uint32_t data_le;
};
static_assert(sizeof(SdoInitDownloadReq) == 8, "SdoInitDownloadReq must be 8 bytes");

struct __attribute__((packed)) SdoInitDownloadRes {
    uint8_t cmd;
    uint16_t index_le;
    uint8_t sub;
    uint8_t reserved[4];
};
static_assert(sizeof(SdoInitDownloadRes) == 8, "SdoInitDownloadRes must be 8 bytes");

struct __attribute__((packed)) SdoAbort {
    uint8_t cmd;
    uint16_t index_le;
    uint8_t sub;
    uint32_t abortCode_le;
};
static_assert(sizeof(SdoAbort) == 8, "SdoAbort must be 8 bytes");

struct __attribute__((packed)) EtherCATScanFrame {
    EthernetHeader eth;
    EtherCATFrameHeader ec;
    EtherCATDatagramHeader dg;
    uint8_t data[2];
    uint16_t wkc_le;
};
static_assert(sizeof(EtherCATScanFrame) == 14 + 2 + 10 + 2 + 2, "Unexpected EtherCATScanFrame size");

} // namespace Raw
} // namespace EtherCAT
