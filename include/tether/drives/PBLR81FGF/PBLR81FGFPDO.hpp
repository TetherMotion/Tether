/**
 * @file PBLR81FGFPDO.hpp
 * @brief Compile-time PDO layout descriptors for PBLR81FGF drive
 *
 * The PBLR81FGF uses a very simple fixed mapping: one RxPDO (0x1600) and
 * one TxPDO (0x1A00), each containing seven entries totalling 128 bits
 * (16 bytes).  Offsets and sizes are expressed as
 * `constexpr` so that they can be used in static_asserts, tests and
 * compile-time buffer layout calculations.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>
#include <string>
#include <sstream>

#include "tether/profiles/cia402/CiA402Drive.hpp"
// standard CiA402 parameter definitions are shared across all drives
#include "profiles/cia402/60xx-Parameters.hpp"

// generic PDO helpers
#include "tether/utils/PDO.hpp"


namespace EtherCAT {
namespace Drives {
namespace PBLR81FGF {

/// One field inside a PDO, referencing a parameter entry.
struct PDOField {
    const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry* entry;
    uint16_t offset;
    uint8_t size;
    const char* description;
};

/// Description of a whole PDO (index, size and pointer to field list).
struct PDO {
    uint16_t index;
    uint16_t size;
    const PDOField* fields;
    size_t field_count;
};

// helper to construct a PDO descriptor from a std::array of fields
static constexpr PDO makePDO(uint16_t idx, uint16_t sz, const PDOField* flds, size_t count) {
    return { idx, sz, flds, count };
}

// ---------------------------------------------------------------------------
// TxPDO 0x1A00 (slave → master, 16 bytes)
//  statusword (0x6041)           2
//  position actual value (0x6064)4
//  velocity actual value (0x606C)4
//  torque actual value (0x6077)   2
//  error code (0x603F)            2
//  mode display (0x6061)          1
//  reserved                       1
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,7> TxPDO_1A00_Fields = {{
    { &CiA402::Parameters60xx::StatusWord,           0u, 2, "Statusword" },
    { &CiA402::Parameters60xx::PositionFeedback,     2u, 4, "PositionActualValue" },
    { &CiA402::Parameters60xx::ActualSpeed,          6u, 4, "VelocityActualValue" },
    { &CiA402::Parameters60xx::ActualTorque,        10u, 2, "TorqueActualValue" },
    { &CiA402::Parameters60xx::ErrorCode,           12u, 2, "ErrorCode" },
    { &CiA402::Parameters60xx::ModeDisplay,         14u, 1, "ModeDisplay" },
    { nullptr,                                     15u, 1, "Reserved" },
}};

static constexpr PDO TxPDO_1A00 = makePDO(0x1A00u, 16u,
                                          TxPDO_1A00_Fields.data(),
                                          TxPDO_1A00_Fields.size());
static_assert(TxPDO_1A00.field_count == TxPDO_1A00_Fields.size(), "TxPDO1A00 field count mismatch");

// ---------------------------------------------------------------------------
// RxPDO 0x1600 (master → slave, 16 bytes)
//  control word (0x6040)        2
//  target position (0x607A)      4
//  target velocity (0x60FF)      4
//  target torque (0x6071)        2
//  max torque (0x6072)           2
//  modes of operation (0x6060)   1
//  reserved                      1
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,7> RxPDO_1600_Fields = {{
    { &CiA402::Parameters60xx::ControlWord,            0u, 2, "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition,         2u, 4, "TargetPosition" },
    { &CiA402::Parameters60xx::TargetVelocity,         6u, 4, "TargetVelocity" },
    { &CiA402::Parameters60xx::TargetTorque,          10u, 2, "TargetTorque" },
    { &CiA402::Parameters60xx::MaxTorque,             12u, 2, "MaxTorque" },
    { &CiA402::Parameters60xx::OperationMode,         14u, 1, "ModesOfOperation" },
    { nullptr,                                       15u, 1, "Reserved" },
}};

static constexpr PDO RxPDO_1600 = makePDO(0x1600u, 16u,
                                          RxPDO_1600_Fields.data(),
                                          RxPDO_1600_Fields.size());
static_assert(RxPDO_1600.field_count == RxPDO_1600_Fields.size(), "RxPDO1600 field count mismatch");

// convenience descriptor vectors
inline const std::vector<const PDO*> kAllPDOs = {
    &RxPDO_1600,
    &TxPDO_1A00,
};

inline const std::vector<const PDO*> kRxPDOs = {
    &RxPDO_1600,
};

inline const std::vector<const PDO*> kTxPDOs = {
    &TxPDO_1A00,
};

/// @brief Find a PDO descriptor by its index value (PBLR81FGF-specific).
inline constexpr const PDO* findPDOByIndex(uint16_t idx) noexcept
{
    return EtherCAT::Utils::findPDOByIndex(kAllPDOs, idx);
}

// ----------------------------------------------------------------------------
// Utility helpers
// ----------------------------------------------------------------------------

/**
 * @brief Dump contents of the current Rx/Tx PDO buffer using compile-time
 *        descriptor information.
 *
 * The function selects the appropriate descriptor based on the PDO index
 * stored in the drive object.  It then iterates through each field, reading
 * the value from the raw buffer and appending a human-readable line with
 * the field name and value.  If the index is unknown, the returned string
 * will still include the numeric PDO index.
 */
inline std::string dumpUsingDescriptors(CiA402Drive& drive, bool tx)
{
    uint16_t idx = tx ? drive.getTxPDOIndex() : drive.getRxPDOIndex();
    uint16_t size = tx ? drive.getTxPDOSize() : drive.getRxPDOSize();

    const PDO* desc = nullptr;
    switch (idx) {
        case RxPDO_1600.index: desc = &RxPDO_1600; break;
        case TxPDO_1A00.index: desc = &TxPDO_1A00; break;
        default:
            break;
    }

    std::ostringstream oss;
    oss << "PDO 0x" << std::hex << std::uppercase << idx << std::nouppercase
        << " size=" << std::dec << size;

    if (desc) {
        const uint8_t* buf = reinterpret_cast<const uint8_t*>(
            tx ? drive.getTxPDOBuffer() : drive.getRxPDOBuffer());
        for (size_t i = 0; i < desc->field_count; ++i) {
            const PDOField& f = desc->fields[i];
            oss << "\n  " << f.description << "@" << f.offset << " = 0x";
            // read little-endian value up to 8 bytes
            uint64_t v = 0;
            for (uint8_t b = 0; b < f.size && f.offset + b < size; ++b) {
                v |= static_cast<uint64_t>(buf[f.offset + b]) << (8 * b);
            }
            oss << std::hex << v << std::dec;
        }
    }

    return oss.str();
}

// ---------------------------------------------------------------------------
// packed structs matching the only two PDOs
// ---------------------------------------------------------------------------

struct PBLR81FGF_RxPDO_1600 {
    uint16_t controlword;
    int32_t  target_position;
    int32_t  target_velocity;
    int16_t  target_torque;
    uint16_t max_torque;
    int8_t   modes_of_operation;
    uint8_t  reserved;
} __attribute__((packed));
static_assert(sizeof(PBLR81FGF_RxPDO_1600) == RxPDO_1600.size,
              "PBLR81FGF_RxPDO_1600 struct size must match PDO 0x1600 size");

struct PBLR81FGF_TxPDO_1A00 {
    uint16_t statusword;
    int32_t  position_actual;
    int32_t  velocity_actual;
    int16_t  torque_actual;
    uint16_t error_code;
    int8_t   modes_of_operation_display;
    uint8_t  reserved;
} __attribute__((packed));
static_assert(sizeof(PBLR81FGF_TxPDO_1A00) == TxPDO_1A00.size,
              "PBLR81FGF_TxPDO_1A00 struct size must match PDO 0x1A00 size");


} // namespace PBLR81FGF
} // namespace Drives
} // namespace EtherCAT
