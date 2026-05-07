/**
 * @file AS715NPDO.hpp
 * @brief Compile-time PDO layout descriptors for ANCTL AS715N (0x1702 / 0x1B02)
 *
 * Provides a small, generic `PDODescriptor` and two constexpr descriptor lists
 * describing the fixed slave-defined PDOs used by the AS715N drive.  All
 * offsets/sizes/indices are expressed as `constexpr` so they can be used in
 * static_asserts, tests and compile-time buffer layout calculations.
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

// we reference constants by fully qualifying to avoid polluting namespace

// ----------------------------------------------------------------------------
// New PDO API (similar style to parameters API)
// ----------------------------------------------------------------------------

namespace EtherCAT {
namespace Drives {
namespace AS715N_pdo {

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




// ---------------------------------------------------------------------------
// AS715N — slave-defined RxPDO 0x1702 (19 bytes total)
// Layout (sequential):
//  0x1702[0] Controlword             : 0x6040 (2 bytes)
//  0x1702[2] TargetPosition          : 0x607A (4 bytes)
//  0x1702[6] TargetVelocity          : 0x60FF (4 bytes)
//  0x1702[10] TargetTorque           : 0x6071 (2 bytes)
//  0x1702[12] ModesOfOperation       : 0x6060 (1 byte)
//  0x1702[13] TouchProbeFunction     : 0x60B8 (2 bytes)
//  0x1702[15] MaxProfileVelocity     : 0x607F (4 bytes)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,7> RxPDO_1702_Fields = {{
    { &CiA402::Parameters60xx::ControlWord,                 0u, 2,  "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition,              2u, 4,  "TargetPosition" },
    { &CiA402::Parameters60xx::TargetVelocity,              6u, 4,  "TargetVelocity" },
    { &CiA402::Parameters60xx::TargetTorque,               10u, 2,  "TargetTorque" },
    { &CiA402::Parameters60xx::OperationMode,              12u, 1,  "ModesOfOperation" },
    { &CiA402::Parameters60xx::TouchProbeFunction,         13u, 2,  "TouchProbeFunction" },
    { &CiA402::Parameters60xx::MaxSpeed,                   15u, 4,  "MaxProfileVelocity" },
}};

// helper to construct a PDO descriptor from a std::array of fields
static constexpr PDO makePDO(uint16_t idx, uint16_t sz, const PDOField* flds, size_t count) {
    return { idx, sz, flds, count };
}

static constexpr PDO RxPDO_1702 = makePDO(0x1702u, 19u,
                                          RxPDO_1702_Fields.data(),
                                          RxPDO_1702_Fields.size());

static_assert(RxPDO_1702.field_count == RxPDO_1702_Fields.size(), "RxPDO1702 field count mismatch");


// ---------------------------------------------------------------------------
// AS715N — slave-defined TxPDO 0x1B02 (25 bytes total)
// Layout (sequential):
//  0x1B02[0]  ManufacturerErr/ErrCode  : 0x603F (2 bytes)
//  0x1B02[2]  Statusword               : 0x6041 (2 bytes)
//  0x1B02[4]  PositionActualValue      : 0x6064 (4 bytes)
//  0x1B02[8]  TorqueActualValue        : 0x6077 (2 bytes)
//  0x1B02[10] ModesOfOperationDisplay : 0x6061 (1 byte)
//  0x1B02[11] TouchProbeStatus         : 0x60B9 (2 bytes)
//  0x1B02[13] TouchProbe1PosEdge      : 0x60BA (4 bytes)
//  0x1B02[17] TouchProbe2PosEdge      : 0x60BC (4 bytes)
//  0x1B02[21] DigitalInputs            : 0x60FD (4 bytes)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// AS715N — slave-defined TxPDO 0x1B02 (25 bytes total)
// Layout (sequential):
//  0x1B02[0]  ManufacturerErr/ErrCode  : 0x603F (2 bytes)
//  0x1B02[2]  Statusword               : 0x6041 (2 bytes)
//  0x1B02[4]  PositionActualValue      : 0x6064 (4 bytes)
//  0x1B02[8]  TorqueActualValue        : 0x6077 (2 bytes)
//  0x1B02[10] ModesOfOperationDisplay : 0x6061 (1 byte)
//  0x1B02[11] TouchProbeStatus         : 0x60B9 (2 bytes)
//  0x1B02[13] TouchProbe1PosEdge      : 0x60BA (4 bytes)
//  0x1B02[17] TouchProbe2PosEdge      : 0x60BC (4 bytes)
//  0x1B02[21] DigitalInputs            : 0x60FD (4 bytes)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,9> TxPDO_1B02_Fields = {{
    { &CiA402::Parameters60xx::ErrorCode,               0u, 2,  "ManufacturerError" },
    { &CiA402::Parameters60xx::StatusWord,              2u, 2,  "Statusword" },
    { &CiA402::Parameters60xx::PositionFeedback,        4u, 4,  "PositionActualValue" },
    { &CiA402::Parameters60xx::ActualTorque,            8u, 2,  "TorqueActualValue" },
    { &CiA402::Parameters60xx::ModeDisplay,             10u, 1,  "ModesOfOperationDisplay" },
    { &CiA402::Parameters60xx::TouchProbeStatus,        11u, 2,  "TouchProbeStatus" },
    { &CiA402::Parameters60xx::TouchProbe1PosEdge,      13u, 4,  "TouchProbe1PosEdge" },
    { &CiA402::Parameters60xx::TouchProbe2PosEdge,      17u, 4,  "TouchProbe2PosEdge" },
    { &CiA402::Parameters60xx::DIStatus,                21u, 4,  "DigitalInputs" },
}};

static constexpr PDO TxPDO_1B02 = makePDO(0x1B02u, 25u,
                                           TxPDO_1B02_Fields.data(),
                                           TxPDO_1B02_Fields.size());

static_assert(TxPDO_1B02.field_count == TxPDO_1B02_Fields.size(), "TxPDO1B02 field count mismatch");


// ---------------------------------------------------------------------------
// Convenience named offsets (compile-time)
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// AS715N — slave DEFAULT RxPDO 0x1701 (12 bytes total)
// RxPDO 0x1701: Controlword(2) + TargetPosition(4) + TouchProbeFunction(2) + PhysicalOutputs(4)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,4> RxPDO_1701_Fields = {{
    { &CiA402::Parameters60xx::ControlWord, 0u, 2, "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition, 2u, 4, "TargetPosition" },
    { &CiA402::Parameters60xx::TouchProbeFunction, 6u, 2, "TouchProbeFunction" },
    { nullptr, 8u, 4, "ForcedPhysicalDO" },   // 60FE:1
}};

static constexpr PDO RxPDO_1701 = makePDO(0x1701u, 12u,
                                           RxPDO_1701_Fields.data(),
                                           RxPDO_1701_Fields.size());

static_assert(RxPDO_1701.field_count == RxPDO_1701_Fields.size(), "RxPDO1701 field count mismatch");


// ---------------------------------------------------------------------------
// AS715N — slave DEFAULT TxPDO 0x1B01 (28 bytes total)
// TxPDO 0x1B01: ErrCode(2) + Statusword(2) + PosActual(4) + Torque(2) + FollowErr(4)
//               + TPStatus(2) + TP1(4) + TP2(4) + DI(4)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,9> TxPDO_1B01_Fields = {{
    { &CiA402::Parameters60xx::ErrorCode,               0u, 2,  "ManufacturerError" },
    { &CiA402::Parameters60xx::StatusWord,              2u, 2,  "Statusword" },
    { &CiA402::Parameters60xx::PositionFeedback,        4u, 4,  "PositionActualValue" },
    { &CiA402::Parameters60xx::ActualTorque,            8u, 2,  "TorqueActualValue" },
    { &CiA402::Parameters60xx::FollowingErrorActual,   10u, 4,  "FollowingError" },
    { &CiA402::Parameters60xx::TouchProbeFunction,     14u, 2,  "TouchProbeStatus" },
    { &CiA402::Parameters60xx::TouchProbe1PosEdge,     16u, 4,  "TouchProbe1PosEdge" },
    { &CiA402::Parameters60xx::TouchProbe2PosEdge,     20u, 4,  "TouchProbe2PosEdge" },
    { &CiA402::Parameters60xx::DIStatus,               24u, 4,  "DigitalInputs" },
}};

static constexpr PDO TxPDO_1B01 = makePDO(0x1B01u, 28u,
                                           TxPDO_1B01_Fields.data(),
                                           TxPDO_1B01_Fields.size());

static_assert(TxPDO_1B01.field_count == TxPDO_1B01_Fields.size(), "TxPDO1B01 field count mismatch");



// ---------------------------------------------------------------------------
// AS715N — alternative RxPDO 0x1703 (7 entries, 17 bytes)
// RxPDO 0x1703: Controlword(2) + TargetPosition(4) + TargetVelocity(4)
//               + TargetTorque(2) + Mode(1) + TouchProbeFunction(2)
//               + PositiveTorqueLimit (60E0, 2)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,7> RxPDO_1703_Fields = {{
    { &CiA402::Parameters60xx::ControlWord,             0u, 2,  "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition,          2u, 4,  "TargetPosition" },
    { &CiA402::Parameters60xx::TargetVelocity,          6u, 4,  "TargetVelocity" },
    { &CiA402::Parameters60xx::TargetTorque,           10u, 2,  "TargetTorque" },
    { &CiA402::Parameters60xx::OperationMode,          12u, 1,  "ModesOfOperation" },
    { &CiA402::Parameters60xx::TouchProbeFunction,     13u, 2,  "TouchProbeFunction" },
    { &CiA402::Parameters60xx::PositiveTorqueLimit,    15u, 2,  "PositiveTorqueLimit" },
}};

static constexpr PDO RxPDO_1703 = makePDO(0x1703u, 17u,
                                          RxPDO_1703_Fields.data(),
                                          RxPDO_1703_Fields.size());

static_assert(RxPDO_1703.field_count == RxPDO_1703_Fields.size(), "RxPDO1703 field count mismatch");

// ---------------------------------------------------------------------------
// AS715N — alternative TxPDO 0x1B03 (10 entries, 29 bytes)
// TxPDO 0x1B03: ErrCode(2) + Statusword(2) + PosActual(4) + Torque(2)
//               + PositionDeviation(4) + ModeDisp(1) + TPStatus(2)
//               + TP1(4) + TP2(4) + DI(4)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,10> TxPDO_1B03_Fields = {{
    { &CiA402::Parameters60xx::ErrorCode,  0u, 2,  "ErrorCode" },
    { &CiA402::Parameters60xx::StatusWord,  2u, 2,  "Statusword" },
    { &CiA402::Parameters60xx::PositionFeedback,  4u, 4,  "PositionActualValue" },
    { &CiA402::Parameters60xx::ActualTorque,  8u, 2,  "TorqueActualValue" },
    { &CiA402::Parameters60xx::FollowingErrorActual, 10u, 4,  "PositionDeviation" },
    { &CiA402::Parameters60xx::ModeDisplay,          14u, 1,  "ModeDisplay" },
    { &CiA402::Parameters60xx::TouchProbeFunction,   15u, 2,  "TouchProbeStatus" },
    { &CiA402::Parameters60xx::TouchProbe1PosEdge,   17u, 4,  "TouchProbe1PosEdge" },
    { &CiA402::Parameters60xx::TouchProbe2PosEdge,   21u, 4,  "TouchProbe2PosEdge" },
    { &CiA402::Parameters60xx::DIStatus,             25u, 4,  "DigitalInputs" },
}};

static constexpr PDO TxPDO_1B03 = makePDO(0x1B03u, 29u,
                                          TxPDO_1B03_Fields.data(),
                                          TxPDO_1B03_Fields.size());

static_assert(TxPDO_1B03.field_count == TxPDO_1B03_Fields.size(), "TxPDO1B03 field count mismatch");



// ---------------------------------------------------------------------------
// AS715N — alternative RxPDO 0x1704 (nine entries, 23 bytes)
// RxPDO 0x1704: Controlword(2) + TargetPosition(4) + TargetVelocity(4)
//               + TargetTorque(2) + Mode selection(1) + TouchProbeFunction(2)
//               + MaxProfileVelocity(4) + PosTorqueLimit(2) + NegTorqueLimit(2)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,9> RxPDO_1704_Fields = {{
    { &CiA402::Parameters60xx::ControlWord, 0u, 2,  "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition, 2u, 4,  "TargetPosition" },
    { &CiA402::Parameters60xx::TargetVelocity, 6u, 4,  "TargetVelocity" },
    { &CiA402::Parameters60xx::TargetTorque, 10u, 2,  "TargetTorque" },
    { &CiA402::Parameters60xx::OperationMode, 12u, 1,  "ModesOfOperation" },
    { &CiA402::Parameters60xx::TouchProbeFunction, 13u, 2,  "TouchProbeFunction" },
    { &CiA402::Parameters60xx::MaxSpeed, 15u, 4,  "MaxProfileVelocity" },
    { &CiA402::Parameters60xx::PositiveTorqueLimit,19u, 2,  "PositiveTorqueLimit" },
    { &CiA402::Parameters60xx::NegativeTorqueLimit,21u, 2,  "NegativeTorqueLimit" },
}};

static constexpr PDO RxPDO_1704 = makePDO(0x1704u, 23u,
                                          RxPDO_1704_Fields.data(),
                                          RxPDO_1704_Fields.size());

static_assert(RxPDO_1704.field_count == RxPDO_1704_Fields.size(), "RxPDO1704 field count mismatch");


// ---------------------------------------------------------------------------
// AS715N — alternative RxPDO 0x1705 (eight entries, 19 bytes)
// RxPDO 0x1705: Controlword(2) + TargetPosition(4) + TargetVelocity(4)
//               + Mode selection(1) + TouchProbeFunction(2)
//               + PositiveTorqueLimit(2) + NegativeTorqueLimit(2) + TorqueOffset(2)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,8> RxPDO_1705_Fields = {{
    { &CiA402::Parameters60xx::ControlWord, 0u, 2,  "Controlword" },
    { &CiA402::Parameters60xx::TargetPosition, 2u, 4,  "TargetPosition" },
    { &CiA402::Parameters60xx::TargetVelocity, 6u, 4,  "TargetVelocity" },
    { &CiA402::Parameters60xx::OperationMode, 10u, 1, "ModesOfOperation" },
    { &CiA402::Parameters60xx::TouchProbeFunction,11u, 2, "TouchProbeFunction" },
    { &CiA402::Parameters60xx::PositiveTorqueLimit,13u, 2, "PositiveTorqueLimit" },
    { &CiA402::Parameters60xx::NegativeTorqueLimit,15u, 2, "NegativeTorqueLimit" },
    { &CiA402::Parameters60xx::TorqueOffset,17u, 2, "TorqueOffset" },
}};

static constexpr PDO RxPDO_1705 = makePDO(0x1705u, 19u,
                                          RxPDO_1705_Fields.data(),
                                          RxPDO_1705_Fields.size());

static_assert(RxPDO_1705.field_count == RxPDO_1705_Fields.size(), "RxPDO1705 field count mismatch");

// ---------------------------------------------------------------------------
// AS715N — alternative TxPDO 0x1B04 (ten entries, 29 bytes)
// TxPDO 0x1B04: ErrCode(2) + Statusword(2) + PosActual(4) + Torque(2)
//               + ModeDisplay(1) + PositionDeviation(4) + TPStatus(2)
//               + TP1(4) + TP2(4) + SpeedFeedback(4)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField,10> TxPDO_1B04_Fields = {{
    { &CiA402::Parameters60xx::ErrorCode,  0u, 2,  "ErrorCode" },
    { &CiA402::Parameters60xx::StatusWord,  2u, 2,  "Statusword" },
    { &CiA402::Parameters60xx::PositionFeedback,  4u, 4,  "PositionActualValue" },
    { &CiA402::Parameters60xx::ActualTorque,  8u, 2,  "TorqueActualValue" },
    { &CiA402::Parameters60xx::ModeDisplay, 10u, 1,  "ModesOfOperationDisp" },
    { &CiA402::Parameters60xx::FollowingErrorActual, 11u, 4,  "PositionDeviation" },
    { &CiA402::Parameters60xx::TouchProbeStatus, 15u, 2,  "TouchProbeStatus" },
    { &CiA402::Parameters60xx::TouchProbe1PosEdge, 17u, 4,  "TouchProbe1PosEdge" },
    { &CiA402::Parameters60xx::TouchProbe2PosEdge, 21u, 4,  "TouchProbe2PosEdge" },
    { &CiA402::Parameters60xx::ActualSpeed, 25u, 4,  "SpeedFeedback" },
}};

static constexpr PDO TxPDO_1B04 = makePDO(0x1B04u, 29u,
                                          TxPDO_1B04_Fields.data(),
                                          TxPDO_1B04_Fields.size());

static_assert(TxPDO_1B04.field_count == TxPDO_1B04_Fields.size(), "TxPDO1B04 field count mismatch");

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

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
        case RxPDO_1705.index: desc = &RxPDO_1705; break;
        case RxPDO_1702.index: desc = &RxPDO_1702; break;
        case RxPDO_1701.index: desc = &RxPDO_1701; break;
        case RxPDO_1703.index: desc = &RxPDO_1703; break;

        case TxPDO_1B04.index: desc = &TxPDO_1B04; break;
        case TxPDO_1B02.index: desc = &TxPDO_1B02; break;
        case TxPDO_1B01.index: desc = &TxPDO_1B01; break;
        case TxPDO_1B03.index: desc = &TxPDO_1B03; break;

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
// Unified PDO lists
// ---------------------------------------------------------------------------

inline const std::vector<const PDO*> kAllPDOs = {
    &RxPDO_1701,
    &TxPDO_1B01,
    &RxPDO_1702,
    &TxPDO_1B02,
    &RxPDO_1703,
    &TxPDO_1B03,
    &RxPDO_1704,
    &RxPDO_1705,
    &TxPDO_1B04,
};

inline const std::vector<const PDO*> kRxPDOs = {
    &RxPDO_1701,
    &RxPDO_1702,
    &RxPDO_1703,
    &RxPDO_1704,
    &RxPDO_1705,
};

inline const std::vector<const PDO*> kTxPDOs = {
    &TxPDO_1B01,
    &TxPDO_1B02,
    &TxPDO_1B03,
    &TxPDO_1B04,
};


/// @brief Find a PDO descriptor by its index value (AS715N-specific wrapper).
///
/// Internally forwards to the generic helper in `EtherCAT::Utils`.  Keeping
/// a small drive-specific function ensures existing code continues to compile
/// unmodified.
inline constexpr const PDO* findPDOByIndex(uint16_t idx) noexcept
{
    // generic helper accepts any range of pointers.
    return EtherCAT::Utils::findPDOByIndex(kAllPDOs, idx);
}



// ===========================================================================
// Packed PDO structs — direct memory-mapped access to PDO buffers
// ===========================================================================

/**
 * @brief RxPDO 0x1705 struct (master → slave, 19 bytes)
 *
 * Fields exactly match the slave-defined PDO 0x1705 layout.
 * Use with CiA402Drive::rxPDO<AS715N_RxPDO_1705>() to get a typed pointer
 * to the RxPDO buffer.
 */
struct AS715N_RxPDO_1705 {
    uint16_t controlword;           ///< 0x6040 Controlword
    int32_t  target_position;       ///< 0x607A Target Position (encoder counts)
    int32_t  target_velocity;       ///< 0x60FF Target Velocity (counts/s)
    int8_t   modes_of_operation;    ///< 0x6060 Modes of Operation (8 = CSP)
    uint16_t touch_probe_function;  ///< 0x60B8 Touch Probe Function
    uint16_t positive_torque_limit; ///< 0x60E0 Positive Torque Limit (‰ of rated)
    uint16_t negative_torque_limit; ///< 0x60E1 Negative Torque Limit (‰ of rated)
    int16_t  torque_offset;         ///< 0x60B2 Torque Offset (‰ of rated)
} __attribute__((packed));

static_assert(sizeof(AS715N_RxPDO_1705) == RxPDO_1705.size,
              "AS715N_RxPDO_1705 struct size must match PDO 0x1705 size");

/**
 * @brief TxPDO 0x1B04 struct (slave → master, 29 bytes)
 *
 * Fields exactly match the slave-defined PDO 0x1B04 layout.
 * Use with CiA402Drive::txPDO<AS715N_TxPDO_1B04>() to get a typed pointer
 * to the TxPDO buffer.
 */
struct AS715N_TxPDO_1B04 {
    uint16_t error_code;                 ///< 0x603F Manufacturer Error / Error Code
    uint16_t statusword;                 ///< 0x6041 Statusword
    int32_t  position_actual;            ///< 0x6064 Position Actual Value
    int16_t  torque_actual;              ///< 0x6077 Torque Actual Value (‰ of rated)
    int8_t   modes_of_operation_display; ///< 0x6061 Modes of Operation Display
    int32_t  position_deviation;         ///< 0x60F4 Following Error / Position Deviation
    uint16_t touch_probe_status;         ///< 0x60B9 Touch Probe Status
    int32_t  touch_probe_pos1;           ///< 0x60BA Touch Probe 1 Pos Edge
    int32_t  touch_probe_pos2;           ///< 0x60BC Touch Probe 2 Pos Edge
    int32_t  speed_feedback;             ///< 0x606C Speed Feedback (counts/s)
} __attribute__((packed));

static_assert(sizeof(AS715N_TxPDO_1B04) == TxPDO_1B04.size,
              "AS715N_TxPDO_1B04 struct size must match PDO 0x1B04 size");

/**
 * @brief RxPDO 0x1702 struct (master → slave, 19 bytes)
 */
struct AS715N_RxPDO_1702 {
    uint16_t controlword;           ///< 0x6040 Controlword
    int32_t  target_position;       ///< 0x607A Target Position
    int32_t  target_velocity;       ///< 0x60FF Target Velocity
    int16_t  target_torque;         ///< 0x6071 Target Torque
    int8_t   modes_of_operation;    ///< 0x6060 Modes of Operation
    uint16_t touch_probe_function;  ///< 0x60B8 Touch Probe Function
    uint32_t max_profile_velocity;  ///< 0x607F Max Profile Velocity
} __attribute__((packed));

static_assert(sizeof(AS715N_RxPDO_1702) == RxPDO_1702.size,
              "AS715N_RxPDO_1702 struct size must match PDO 0x1702 size");

/**
 * @brief TxPDO 0x1B02 struct (slave → master, 25 bytes)
 */
struct AS715N_TxPDO_1B02 {
    uint16_t error_code;                 ///< 0x603F Error Code
    uint16_t statusword;                 ///< 0x6041 Statusword
    int32_t  position_actual;            ///< 0x6064 Position Actual Value
    int16_t  torque_actual;              ///< 0x6077 Torque Actual Value
    int8_t   modes_of_operation_display; ///< 0x6061 Modes of Operation Display
    uint16_t touch_probe_status;         ///< 0x60B9 Touch Probe Status
    int32_t  touch_probe_pos1;           ///< 0x60BA Touch Probe 1 Pos Edge
    int32_t  touch_probe_pos2;           ///< 0x60BC Touch Probe 2 Pos Edge
    uint32_t digital_inputs;             ///< 0x60FD Digital Inputs
} __attribute__((packed));

static_assert(sizeof(AS715N_TxPDO_1B02) == TxPDO_1B02.size,
              "AS715N_TxPDO_1B02 struct size must match PDO 0x1B02 size");

} // namespace AS715N_pdo
} // namespace Drives
} // namespace EtherCAT
