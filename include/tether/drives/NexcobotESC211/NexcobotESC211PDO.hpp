/**
 * @file NexcobotESC211PDO.hpp
 * @brief Packed wire-format structs for the ESC211 FSoE SafetyPDU PDOs
 *
 * The ESC211 carries all FSoE channels inside two flat 496-byte PDO
 * regions:
 *
 *   0x6000 (RxPDU, slave→master): PDO map 0x1600 = 16 × 248-bit entries
 *   0x7000 (TxPDU, master→slave): PDO map 0x1A00 = 16 × 248-bit entries
 *
 * Each PDO entry carves a 248-bit (31-byte) section out of the flat PDU.
 * The raw PDU contains all FSoE channels packed together; the decoded
 * objects (0x7100/0x7101 FSoE0 frame/safedata, 0x7110/0x7111 FSoE1
 * frame/safedata, ...) are views into this data.  (The ESI datatype for
 * the SDO objects declares them differently; the wire image carved by
 * the PDO map is authoritative here.)
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace EtherCAT {
namespace Drives {
namespace NexcobotESC211PDO {

/// Maximum number of FSoE channels defined by the ESC211 object
/// dictionary (FSoE0–FSoE7: 0x6100–0x6170 / 0x7100–0x7170).
inline constexpr size_t kMaxFSoEChannels = 8;

/// Flat SafetyPDU layout: 16 PDO entries of 248 bits (31 bytes) each.
inline constexpr size_t kPduSections    = 16;
inline constexpr size_t kPduSectionSize = 31;   ///< 248 bits
inline constexpr size_t kPduTotalBytes  = kPduSections * kPduSectionSize;  // 496

/// 0x6000 — FSoE SafetyPDU RxPDO (slave→master, 496 bytes flat).
struct RxPDO_0x6000_SafetyPDU {
    uint8_t data[kPduTotalBytes];
} __attribute__((packed));
static_assert(sizeof(RxPDO_0x6000_SafetyPDU) == kPduTotalBytes,
              "RxPDO_0x6000_SafetyPDU size mismatch");

/// 0x7000 — FSoE SafetyPDU TxPDO (master→slave, 496 bytes flat).
struct TxPDO_0x7000_SafetyPDU {
    uint8_t data[kPduTotalBytes];
} __attribute__((packed));
static_assert(sizeof(TxPDO_0x7000_SafetyPDU) == kPduTotalBytes,
              "TxPDO_0x7000_SafetyPDU size mismatch");

} // namespace NexcobotESC211PDO
} // namespace Drives
} // namespace EtherCAT
