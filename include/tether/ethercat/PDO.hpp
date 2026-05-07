// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

/**
 * @file PDO.hpp
 * @brief CiA 301 PDO (Process Data Object) object-dictionary definitions
 *
 * This header contains the CiA 301 object-dictionary indices and subindex
 * definitions for PDO communication and mapping parameters.  The definitions
 * are exposed under `EtherCAT::PDO` (preferred for new code) and backward
 * compatible aliases are provided in the `CiA301` namespace.
 */

namespace EtherCAT { namespace PDO {

// ---------------------------------------------------------------------------
// RxPDO Communication Parameters (0x1400 - 0x15FF)
// ---------------------------------------------------------------------------

/// RxPDO Communication Parameter Base (0x1400 - 0x15FF)
constexpr uint16_t kRxPDOCommParamBase  = 0x1400;
constexpr uint16_t kRxPDOCommParamCount = 512;  // 0x1400 - 0x15FF

namespace CommParamSub {
    constexpr uint8_t NumberOfEntries       = 0x00;
    constexpr uint8_t CobId                 = 0x01;
    constexpr uint8_t TransmissionType      = 0x02;
    constexpr uint8_t InhibitTime           = 0x03;
    constexpr uint8_t Reserved              = 0x04;
    constexpr uint8_t EventTimer            = 0x05;
    constexpr uint8_t SyncStartValue        = 0x06;
}

// ---------------------------------------------------------------------------
// RxPDO Mapping Parameters (0x1600 - 0x17FF)
// ---------------------------------------------------------------------------

/// RxPDO Mapping Parameter Base (0x1600 - 0x17FF)
constexpr uint16_t kRxPDOMappingBase   = 0x1600;
constexpr uint16_t kRxPDOMappingCount  = 512;  // 0x1600 - 0x17FF

namespace MappingSub {
    constexpr uint8_t NumberOfMappedObjects = 0x00;
    constexpr uint8_t MappedObject1         = 0x01;
    constexpr uint8_t MappedObject2         = 0x02;
    constexpr uint8_t MappedObject3         = 0x03;
    constexpr uint8_t MappedObject4         = 0x04;
    constexpr uint8_t MappedObject5         = 0x05;
    constexpr uint8_t MappedObject6         = 0x06;
    constexpr uint8_t MappedObject7         = 0x07;
    constexpr uint8_t MappedObject8         = 0x08;
}

// ---------------------------------------------------------------------------
// TxPDO Communication & Mapping Parameters (0x1800 - 0x1BFF)
// ---------------------------------------------------------------------------

/// TxPDO Communication Parameter Base (0x1800 - 0x19FF)
constexpr uint16_t kTxPDOCommParamBase  = 0x1800;
constexpr uint16_t kTxPDOCommParamCount = 512;  // 0x1800 - 0x19FF

/// TxPDO Mapping Parameter Base (0x1A00 - 0x1BFF)
constexpr uint16_t kTxPDOMappingBase    = 0x1A00;
constexpr uint16_t kTxPDOMappingCount   = 512;  // 0x1A00 - 0x1BFF

}} // namespace EtherCAT::PDO

// ---------------------------------------------------------------------------
// Backward-compatible CiA301 aliases
// ---------------------------------------------------------------------------

namespace CiA301 {

// OD indices / counts (kept as-original names for backward compatibility)
constexpr uint16_t RxPDOCommParamBase  = EtherCAT::PDO::kRxPDOCommParamBase;
constexpr uint16_t RxPDOCommParamCount = EtherCAT::PDO::kRxPDOCommParamCount;

// Subindex namespace alias
namespace PDOCommParamSub = EtherCAT::PDO::CommParamSub;

constexpr uint16_t RxPDOMappingBase   = EtherCAT::PDO::kRxPDOMappingBase;
constexpr uint16_t RxPDOMappingCount  = EtherCAT::PDO::kRxPDOMappingCount;

// Subindex namespace alias
namespace PDOMappingSub = EtherCAT::PDO::MappingSub;

constexpr uint16_t TxPDOCommParamBase  = EtherCAT::PDO::kTxPDOCommParamBase;
constexpr uint16_t TxPDOCommParamCount = EtherCAT::PDO::kTxPDOCommParamCount;
constexpr uint16_t TxPDOMappingBase    = EtherCAT::PDO::kTxPDOMappingBase;
constexpr uint16_t TxPDOMappingCount   = EtherCAT::PDO::kTxPDOMappingCount;

} // namespace CiA301
