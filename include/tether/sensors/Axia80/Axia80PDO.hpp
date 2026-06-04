/**
 * @file Axia80PDO.hpp
 * @brief ATI Axia80 Force/Torque Sensor PDO Layout Definitions
 *
 * Provides packed structs and compile-time PDO descriptors for the
 * Axia80's fixed slave-defined PDOs.
 *
 * TxPDO (0x1A00): 6×DINT F/T readings + UDINT status + UDINT counter (32 bytes)
 * RxPDO (0x1601): 2×UDINT control registers (8 bytes)
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
namespace Sensors {
namespace Axia80_pdo {

// ============================================================================
// TxPDO (0x1A00) — Slave → Master, 32 bytes
// ============================================================================

struct Axia80_TxPDO {
    int32_t  fx = 0;       ///< Force X (counts)
    int32_t  fy = 0;       ///< Force Y (counts)
    int32_t  fz = 0;       ///< Force Z (counts)
    int32_t  tx = 0;       ///< Torque X (counts)
    int32_t  ty = 0;       ///< Torque Y (counts)
    int32_t  tz = 0;       ///< Torque Z (counts)
    uint32_t status = 0;   ///< Status code (0x6010)
    uint32_t counter = 0;  ///< Sample counter (0x6020)
} __attribute__((packed));

static_assert(sizeof(Axia80_TxPDO) == 32, "Axia80_TxPDO size mismatch");

// ============================================================================
// RxPDO (0x1601) — Master → Slave, 8 bytes
// ============================================================================

struct Axia80_RxPDO {
    uint32_t control1 = 0; ///< Control register 1 (0x7010.1)
    uint32_t control2 = 0; ///< Control register 2 (0x7010.2)
} __attribute__((packed));

static_assert(sizeof(Axia80_RxPDO) == 8, "Axia80_RxPDO size mismatch");

// ============================================================================
// PDO Descriptors (constexpr objects, matching DynaDrive pattern)
// ============================================================================

struct PDODescriptor {
    uint16_t index;
    uint16_t size;
};

static constexpr PDODescriptor TxPDO_1A00 = { 0x1A00, sizeof(Axia80_TxPDO) };
static constexpr PDODescriptor RxPDO_1601 = { 0x1601, sizeof(Axia80_RxPDO) };

} // namespace Axia80_pdo
} // namespace Sensors
} // namespace EtherCAT
