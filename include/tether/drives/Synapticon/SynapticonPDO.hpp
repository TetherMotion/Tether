/**
 * @file SynapticonPDO.hpp
 * @brief Synapticon SOMANET CiA402 drive — PDO layout definitions
 *
 * Packed structs and constexpr descriptors matching the PDO mappings defined
 * in the SOMANET_CiA_402_v5.1.9.xml ESI file.
 *
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │  PDO LAYOUT — CRITICAL INFORMATION                                   │
 * ├──────────────────────────────────────────────────────────────────────┤
 * │                                                                      │
 * │  The SOMANET drive uses ESI-defined PDO sizes:                       │
 * │    0x1600 = 19 bytes, 0x1601 = 8 bytes, 0x1602 = 8 bytes            │
 * │    0x1A00 = 13 bytes, 0x1A01 = 12 bytes, 0x1A02 = 4 bytes,         │
 * │    0x1A03 = 18 bytes                                                │
 * │    0x1700 = 11 bytes (FSoE RxPDO), 0x1B00 = 31 bytes (FSoE TxPDO)  │
 * │                                                                      │
 * │  COMBINED PDO CONFIGURATION (FSoE + motion):                         │
 * │  ┌─────────────────────────────────────────────────────────────┐    │
 * │  │ SM2 (Rx): [0x1700 (11B)][0x1600 (19B)][0x1601][0x1602]    │    │
 * │  │ SM3 (Tx): [0x1B00 (31B)][0x1A00 (13B)][0x1A01][0x1A02]    │    │
 * │  │                       [0x1A03]                             │    │
 * │  └─────────────────────────────────────────────────────────────┘    │
 * │                                                                      │
 * │  FSoE PDOs COME FIRST in the assignment order.  This is critical     │
 * │  because the Synapticon Circulo EtherCAT chip has a bug where the    │
 * │  last word in the SM buffer is zeroed.  If the FSoE PDO were last,   │
 * │  the ConnectionID (the final word of the FSoE frame) would be        │
 * │  zeroed and the slave would reject every frame.  By placing the      │
 * │  motion PDO last, the zeroed word falls on motion data, not the      │
 * │  FSoE ConnectionID.                                                  │
 * │  See: https://doc.synapticon.com/circulo_safe_motion/smm/            │
 * │       ecat_fsoe_issues.htm                                           │
 * │                                                                      │
 * │  ALL PDOs (including 0x1700/0x1B00) are written explicitly to        │
 * │  0x1C12/0x1C13.  The `fixed` flag is always false.                  │
 * │                                                                      │
 * │  SM REGISTER LENGTH = totalLength() (all PDOs).                      │
 * │  FMMU LENGTH = totalLength() (all PDOs).                             │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * ESI-defined PDO layouts (for application struct compatibility):
 *
 * RxPDOs (master -> slave, SM2, ControlByte 0x64):
 *   0x1600  RxPDO Mapping 1  (19 bytes) — controlword, mode, torque, position, velocity, torque offset, tuning cmd
 *   0x1601  RxPDO Mapping 2  (8 bytes)  — physical outputs, bit mask
 *   0x1602  RxPDO Mapping 3  (8 bytes)  — user MOSI, velocity offset
 *
 * TxPDOs (slave -> master, SM3, ControlByte 0x20):
 *   0x1A00  TxPDO Mapping 1  (13 bytes) — statusword, mode display, position, velocity, torque
 *   0x1A01  TxPDO Mapping 2  (12 bytes) — analog inputs 1-4, tuning status
 *   0x1A02  TxPDO Mapping 3  (4 bytes)  — digital inputs
 *   0x1A03  TxPDO Mapping 4  (18 bytes) — user MISO, timestamp, position demand, velocity demand, torque demand
 *
 * FSoE RxPDO (master -> slave, SM2):
 *   0x1700  Control (PLC to Drive)  (11 bytes) — FSoE command, STO/SS1/SS2/SOS/SBC bits, SLS instances, CRCs, ConnectionID
 *
 * FSoE TxPDO (slave -> master, SM3):
 *   0x1B00  Status (Drive to PLC)   (31 bytes) — FSoE command, safety state bits, safe position/velocity, CRCs, ConnectionID
 */

#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "tether/ethercat/PDOMappingConfig.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace Synapticon_pdo {

// ============================================================================
// RxPDO 0x1600 — Master -> Slave, 14 bytes
// ============================================================================

struct SOMANET_RxPDO_1600 {
    uint16_t controlword;           ///< 0x6040 Controlword
    int8_t   modes_of_operation;    ///< 0x6060 Modes of operation
    int16_t  target_torque;         ///< 0x6071 Target torque
    int32_t  target_position;       ///< 0x607A Target position
    int32_t  target_velocity;       ///< 0x60FF Target velocity
    int16_t  torque_offset;         ///< 0x60B2 Torque offset
    uint32_t tuning_command;        ///< 0x2701 Tuning command
} __attribute__((packed));

static_assert(sizeof(SOMANET_RxPDO_1600) == 19, "SOMANET_RxPDO_1600 size mismatch");

// ============================================================================
// RxPDO 0x1601 — Master -> Slave, 8 bytes
// ============================================================================

struct SOMANET_RxPDO_1601 {
    uint32_t physical_outputs;      ///< 0x60FE:1 Physical outputs
    uint32_t bit_mask;              ///< 0x60FE:2 Bit mask
} __attribute__((packed));

static_assert(sizeof(SOMANET_RxPDO_1601) == 8, "SOMANET_RxPDO_1601 size mismatch");

// ============================================================================
// RxPDO 0x1602 — Master -> Slave, 8 bytes
// ============================================================================

struct SOMANET_RxPDO_1602 {
    uint32_t user_mosi;             ///< 0x2703 User MOSI
    int32_t  velocity_offset;       ///< 0x60B1 Velocity offset
} __attribute__((packed));

static_assert(sizeof(SOMANET_RxPDO_1602) == 8, "SOMANET_RxPDO_1602 size mismatch");

// ============================================================================
// TxPDO 0x1A00 — Slave -> Master, 11 bytes
// ============================================================================

struct SOMANET_TxPDO_1A00 {
    uint16_t statusword;                    ///< 0x6041 Statusword
    int8_t   modes_of_operation_display;    ///< 0x6061 Modes of operation display
    int32_t  position_actual;               ///< 0x6064 Position actual value
    int32_t  velocity_actual;               ///< 0x606C Velocity actual value
    int16_t  torque_actual;                 ///< 0x6077 Torque actual value
} __attribute__((packed));

static_assert(sizeof(SOMANET_TxPDO_1A00) == 13, "SOMANET_TxPDO_1A00 size mismatch");

// ============================================================================
// TxPDO 0x1A01 — Slave -> Master, 12 bytes
// ============================================================================

struct SOMANET_TxPDO_1A01 {
    uint16_t analog_input_1;        ///< 0x2401 Analog input 1
    uint16_t analog_input_2;        ///< 0x2402 Analog input 2
    uint16_t analog_input_3;        ///< 0x2403 Analog input 3
    uint16_t analog_input_4;        ///< 0x2404 Analog input 4
    uint32_t tuning_status;         ///< 0x2702 Tuning status
} __attribute__((packed));

static_assert(sizeof(SOMANET_TxPDO_1A01) == 12, "SOMANET_TxPDO_1A01 size mismatch");

// ============================================================================
// TxPDO 0x1A02 — Slave -> Master, 4 bytes
// ============================================================================

struct SOMANET_TxPDO_1A02 {
    uint32_t digital_inputs;        ///< 0x60FD Digital inputs
} __attribute__((packed));

static_assert(sizeof(SOMANET_TxPDO_1A02) == 4, "SOMANET_TxPDO_1A02 size mismatch");

// ============================================================================
// TxPDO 0x1A03 — Slave -> Master, 18 bytes
// ============================================================================

struct SOMANET_TxPDO_1A03 {
    uint32_t user_miso;             ///< 0x2704 User MISO
    uint32_t timestamp;             ///< 0x20F0 Timestamp
    int32_t  position_demand;       ///< 0x60FC Position demand internal value
    int32_t  velocity_demand;       ///< 0x606B Velocity demand value
    int16_t  torque_demand;         ///< 0x6074 Torque demand
} __attribute__((packed));

static_assert(sizeof(SOMANET_TxPDO_1A03) == 18, "SOMANET_TxPDO_1A03 size mismatch");

// ============================================================================
// PDO Descriptors (constexpr objects, matching AS715N/DynaDrive pattern)
// ============================================================================

struct PDODescriptor {
    uint16_t index;
    uint16_t size;
};

static constexpr PDODescriptor RxPDO_1600 = { 0x1600, sizeof(SOMANET_RxPDO_1600) };
static constexpr PDODescriptor RxPDO_1601 = { 0x1601, sizeof(SOMANET_RxPDO_1601) };
static constexpr PDODescriptor RxPDO_1602 = { 0x1602, sizeof(SOMANET_RxPDO_1602) };

static constexpr PDODescriptor TxPDO_1A00 = { 0x1A00, sizeof(SOMANET_TxPDO_1A00) };
static constexpr PDODescriptor TxPDO_1A01 = { 0x1A01, sizeof(SOMANET_TxPDO_1A01) };
static constexpr PDODescriptor TxPDO_1A02 = { 0x1A02, sizeof(SOMANET_TxPDO_1A02) };
static constexpr PDODescriptor TxPDO_1A03 = { 0x1A03, sizeof(SOMANET_TxPDO_1A03) };

// ============================================================================
// FSoE RxPDO 0x1700 — Master -> Slave, 11 bytes
// Control (PLC to Drive) — FSoE safety command frame
//
// ESI layout (88 bits = 11 bytes):
//   Byte 0:    FSoE Command (8 bits)
//   Bytes 1-3: Safety flags (24 bits used, 8 padding)
//     bit 0:  STO          (0x6640:0)
//     bit 1:  SS1          (0x6650:1)
//     bit 2:  SS2          (0x6670:1)
//     bit 3:  SOS          (0x6668:1)
//     bits 4-6: reserved
//     bit 7:  Error ack    (0x6632:0)
//     bit 8:  SLS inst 1   (0x6690:1)
//     bit 9:  SLS inst 2   (0x6690:2)
//     bit 10: SLS inst 3   (0x6690:3)
//     bit 11: SLS inst 4   (0x6690:4)
//     bit 12: Restart ack  (0x6630:0)
//     bit 13: SBC command  (0x6660:0)
//     bit 14: Reset pos    (0x26A0:0)
//     bit 15: reserved
//     bits 16-23: reserved
//   Bytes 4-5: FSoE CRC_0  (16 bits)
//   Byte 6:    reserved (8 bits)
//   Byte 7:    Safe outputs (2 bits used + 6 padding)
//     bit 0: Safe output 1 (0x26F0:1)
//     bit 1: Safe output 2 (0x26F0:2)
//   Bytes 8-9: FSoE CRC_1  (16 bits)
//   Bytes 10-11: FSoE ConnectionID (16 bits)
// ============================================================================

struct SOMANET_RxPDO_1700 {
    uint8_t  fsoe_command;          ///< 0x6770:1 FSoE Command
    uint16_t safety_flags;          ///< Bit-packed: STO/SS1/SS2/SOS/SLS/SBC/ResetPos
    uint16_t fsoe_crc_0;            ///< 0x6770:3 FSoE CRC_0
    uint8_t  reserved_byte;         ///< Padding
    uint8_t  safe_outputs;          ///< Bit 0: Safe output 1, bit 1: Safe output 2
    uint16_t fsoe_crc_1;            ///< 0x6770:4 FSoE CRC_1
    uint16_t fsoe_connection_id;    ///< 0x6770:2 FSoE ConnectionID

    // Safety flag bit positions in safety_flags field
    static constexpr uint16_t kSTO             = 1u << 0;
    static constexpr uint16_t kSS1             = 1u << 1;
    static constexpr uint16_t kSS2             = 1u << 2;
    static constexpr uint16_t kSOS             = 1u << 3;
    static constexpr uint16_t kErrorAck        = 1u << 7;
    static constexpr uint16_t kSLS_Instance1   = 1u << 8;
    static constexpr uint16_t kSLS_Instance2   = 1u << 9;
    static constexpr uint16_t kSLS_Instance3   = 1u << 10;
    static constexpr uint16_t kSLS_Instance4   = 1u << 11;
    static constexpr uint16_t kRestartAck      = 1u << 12;
    static constexpr uint16_t kSBCCommand      = 1u << 13;
    static constexpr uint16_t kResetPosition   = 1u << 14;

    // Safe output bit positions in safe_outputs field
    static constexpr uint8_t kSafeOutput1 = 1u << 0;
    static constexpr uint8_t kSafeOutput2 = 1u << 1;
} __attribute__((packed));

static_assert(sizeof(SOMANET_RxPDO_1700) == 11, "SOMANET_RxPDO_1700 size mismatch");

// ============================================================================
// FSoE TxPDO 0x1B00 — Slave -> Master, 31 bytes
// Status (Drive to PLC) — FSoE safety status frame
//
// ESI layout (248 bits = 31 bytes):
//   Byte 0:     FSoE Command (8 bits)
//   Bytes 1-3:  Safety state flags (16 bits used + 8 padding)
//     bit 0:  STO state        (0x6640:0)
//     bit 3:  SOS state        (0x6668:1)
//     bit 7:  Error state      (0x6632:0)
//     bit 8:  SS1 state        (0x6650:1)
//     bit 9:  SS2 state        (0x6670:1)
//     bit 12: SLS inst 1       (0x6690:1)
//     bit 13: SLS inst 2       (0x6690:2)
//     bit 14: SLS inst 3       (0x6690:3)
//     bit 15: SLS inst 4       (0x6690:4)
//   Bytes 4-5:  FSoE CRC_0 (16 bits)
//   Bytes 6-7:  Diagnostic flags (16 bits used)
//     bit 0: Restart ack req   (0x6630:0)
//     bit 1: SBC state         (0x6660:0)
//     bit 2: Temp warning      (0x2600:0)
//     bit 3: Safe pos valid    (0x2601:0)
//     bit 4: Safe speed valid  (0x2602:0)
//     bit 8: Safe input 1      (0x2603:1)
//     bit 9: Safe input 2      (0x2603:2)
//     bit 10: Safe input 3     (0x2603:3)
//     bit 11: Safe input 4     (0x2603:4)
//     bit 12: Safe out mon 1   (0x2604:1)
//     bit 13: Safe out mon 2   (0x2604:2)
//     bit 14: Analog diag      (0x2605:1)
//     bit 15: Analog valid     (0x2605:2)
//   Bytes 8-9:  FSoE CRC_1 (16 bits)
//   Bytes 10-11: Safe position actual (16 bits)
//   Bytes 12-13: FSoE CRC_2 (16 bits)
//   Bytes 14-15: Safe position actual duplicate (16 bits)
//   Bytes 16-17: FSoE CRC_3 (16 bits)
//   Bytes 18-19: Safe velocity actual (16 bits)
//   Bytes 20-21: FSoE CRC_4 (16 bits)
//   Bytes 22-23: Safe velocity actual duplicate (16 bits)
//   Bytes 24-25: FSoE CRC_5 (16 bits)
//   Bytes 26-27: Safe analog value (16 bits)
//   Bytes 28-29: FSoE CRC_6 (16 bits)
//   Bytes 30-31: FSoE ConnectionID (16 bits)
// ============================================================================

struct SOMANET_TxPDO_1B00 {
    uint8_t  fsoe_command;              ///< 0x6760:1 FSoE Command
    uint16_t safety_state_flags;        ///< Bit-packed: STO/SOS/SS1/SS2/SLS/Error
    uint16_t fsoe_crc_0;                ///< 0x6760:3 FSoE CRC_0
    uint16_t diagnostic_flags;          ///< Bit-packed: diag/safe I/O status
    uint16_t fsoe_crc_1;                ///< 0x6760:4 FSoE CRC_1
    uint16_t safe_position_actual;      ///< 0x6611:0 Safe position actual value
    uint16_t fsoe_crc_2;                ///< 0x6760:5 FSoE CRC_2
    uint16_t safe_position_actual_dup;  ///< Safe position actual value (duplicate)
    uint16_t fsoe_crc_3;                ///< 0x6760:6 FSoE CRC_3
    uint16_t safe_velocity_actual;      ///< 0x6613:0 Safe velocity actual value
    uint16_t fsoe_crc_4;                ///< 0x6760:7 FSoE CRC_4
    uint16_t safe_velocity_actual_dup;  ///< Safe velocity actual value (duplicate)
    uint16_t fsoe_crc_5;                ///< 0x6760:8 FSoE CRC_5
    uint16_t safe_analog_value;         ///< 0x2605:3 Safe analog value (scaled)
    uint16_t fsoe_crc_6;                ///< 0x6760:9 FSoE CRC_6
    uint16_t fsoe_connection_id;        ///< 0x6760:2 FSoE ConnectionID

    // Safety state flag bit positions
    static constexpr uint16_t kSTOState        = 1u << 0;
    static constexpr uint16_t kSOSState        = 1u << 3;
    static constexpr uint16_t kErrorState      = 1u << 7;
    static constexpr uint16_t kSS1State        = 1u << 8;
    static constexpr uint16_t kSS2State        = 1u << 9;
    static constexpr uint16_t kSLSInstance1    = 1u << 12;
    static constexpr uint16_t kSLSInstance2    = 1u << 13;
    static constexpr uint16_t kSLSInstance3    = 1u << 14;
    static constexpr uint16_t kSLSInstance4    = 1u << 15;

    // Diagnostic flag bit positions
    static constexpr uint16_t kRestartAckReq       = 1u << 0;
    static constexpr uint16_t kSBCState            = 1u << 1;
    static constexpr uint16_t kTemperatureWarning  = 1u << 2;
    static constexpr uint16_t kSafePositionValid   = 1u << 3;
    static constexpr uint16_t kSafeSpeedValid      = 1u << 4;
    static constexpr uint16_t kSafeInput1          = 1u << 8;
    static constexpr uint16_t kSafeInput2          = 1u << 9;
    static constexpr uint16_t kSafeInput3          = 1u << 10;
    static constexpr uint16_t kSafeInput4          = 1u << 11;
    static constexpr uint16_t kSafeOutputMonitor1  = 1u << 12;
    static constexpr uint16_t kSafeOutputMonitor2  = 1u << 13;
    static constexpr uint16_t kAnalogDiagActive    = 1u << 14;
    static constexpr uint16_t kAnalogValueValid    = 1u << 15;
} __attribute__((packed));

static_assert(sizeof(SOMANET_TxPDO_1B00) == 31, "SOMANET_TxPDO_1B00 size mismatch");

static constexpr PDODescriptor RxPDO_1700 = { 0x1700, sizeof(SOMANET_RxPDO_1700) };
static constexpr PDODescriptor TxPDO_1B00 = { 0x1B00, sizeof(SOMANET_TxPDO_1B00) };

// ============================================================================
// Sync Manager constants (from ESI Sm elements)
// ============================================================================

/// SM2 control byte: Buffered | Write | Watchdog | RepeatReq = 0x64
constexpr uint8_t kSM2ControlByte = 0x64;
/// SM3 control byte: Buffered | Read | Watchdog = 0x20
constexpr uint8_t kSM3ControlByte = 0x20;

/// SM2 physical start address (from ESI)
constexpr uint16_t kSM2PhysAddr = 0x1800;
/// SM3 physical start address (from ESI)
constexpr uint16_t kSM3PhysAddr = 0x1C00;

/// Total SM2 size: 19 + 8 + 8 = 35 bytes (matches ESI DefaultSize)
constexpr uint16_t kSM2TotalSize = RxPDO_1600.size + RxPDO_1601.size + RxPDO_1602.size;
/// Total SM3 size: 13 + 12 + 4 + 18 = 47 bytes (matches ESI DefaultSize)
constexpr uint16_t kSM3TotalSize = TxPDO_1A00.size + TxPDO_1A01.size + TxPDO_1A02.size + TxPDO_1A03.size;

static_assert(kSM2TotalSize == 35, "SM2 total size must match ESI DefaultSize");
static_assert(kSM3TotalSize == 47, "SM3 total size must match ESI DefaultSize");

/// FSoE PDO offset within SM2 — FSoE comes FIRST (offset 0).
/// This is critical: the Synapticon Circulo ESC has a bug where the last
/// word in the SM buffer is zeroed.  If FSoE were last, the ConnectionID
/// (final word of the FSoE frame) would be zeroed and the slave would
/// reject every frame.  Motion PDOs come after FSoE so the zeroed word
/// falls on motion data.
/// See: https://doc.synapticon.com/circulo_safe_motion/smm/ecat_fsoe_issues.htm
constexpr uint16_t kFSoERxPDOOffset = 0;
/// Motion PDO offset within SM2 (after FSoE RxPDO)
constexpr uint16_t kMotionRxPDOOffset = RxPDO_1700.size;   // 11
/// FSoE PDO offset within SM3 — FSoE comes FIRST (offset 0).
constexpr uint16_t kFSoETxPDOOffset = 0;
/// Motion PDO offset within SM3 (after FSoE TxPDO)
constexpr uint16_t kMotionTxPDOOffset = TxPDO_1B00.size;   // 31

/// Combined SM2 length: FSoE + motion PDOs
constexpr uint16_t kSM2CombinedSize = RxPDO_1700.size + kSM2TotalSize;   // 11 + 35 = 46
/// Combined SM3 length: FSoE + motion PDOs
constexpr uint16_t kSM3CombinedSize = TxPDO_1B00.size + kSM3TotalSize;   // 31 + 47 = 78

// ============================================================================
// Multi-PDO Assignment Builders
// ============================================================================
//
// These functions build Slave::MultiPDOAssignment configurations using the
// PDO mappings defined in the SOMANET_CiA_402_v5.1.9.xml ESI file.
//
// The standard CiA 402 configuration assigns all three RxPDOs to SM2 and all
// four TxPDOs to SM3, giving the full process data image:
//   SM2 (35 bytes): 0x1600 + 0x1601 + 0x1602
//   SM3 (47 bytes): 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03
//
// The FSoE configuration assigns the safety PDOs:
//   SM2 (11 bytes): 0x1700
//   SM3 (31 bytes): 0x1B00
//
// The combined configuration assigns both FSoE and standard PDOs, with
// FSoE PDOs FIRST (critical for the Synapticon ESC bug — see comment above):
//   SM2 (46 bytes): 0x1700 + 0x1600 + 0x1601 + 0x1602
//   SM3 (78 bytes): 0x1B00 + 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03

/// Build a MultiPDOAssignment with all standard CiA 402 PDOs (no FSoE).
/// SM2: 0x1600 + 0x1601 + 0x1602 (35 bytes)
/// SM3: 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03 (47 bytes)
inline Slave::MultiPDOAssignment makeStandardPDOAssignment() {
    Slave::MultiPDOAssignment assignment;

    // SM2 — Outputs (master -> slave)
    Slave::MultiPDOAssignment::SMConfig sm2;
    sm2.sm_index = 2;
    sm2.phys_start_addr = kSM2PhysAddr;
    sm2.control_byte = kSM2ControlByte;
    sm2.pdo_mappings = {
        {RxPDO_1600.index, RxPDO_1600.size},
        {RxPDO_1601.index, RxPDO_1601.size},
        {RxPDO_1602.index, RxPDO_1602.size},
    };
    assignment.sm_configs.push_back(std::move(sm2));

    // SM3 — Inputs (slave -> master)
    Slave::MultiPDOAssignment::SMConfig sm3;
    sm3.sm_index = 3;
    sm3.phys_start_addr = kSM3PhysAddr;
    sm3.control_byte = kSM3ControlByte;
    sm3.pdo_mappings = {
        {TxPDO_1A00.index, TxPDO_1A00.size},
        {TxPDO_1A01.index, TxPDO_1A01.size},
        {TxPDO_1A02.index, TxPDO_1A02.size},
        {TxPDO_1A03.index, TxPDO_1A03.size},
    };
    assignment.sm_configs.push_back(std::move(sm3));

    return assignment;
}

/// Build a MultiPDOAssignment with only FSoE safety PDOs.
/// SM2: 0x1700 (11 bytes)
/// SM3: 0x1B00 (31 bytes)
///
/// FSoE PDOs are written explicitly to 0x1C12/0x1C13 (fixed=false).
inline Slave::MultiPDOAssignment makeFSoEPDOAssignment() {
    Slave::MultiPDOAssignment assignment;

    // SM2 — FSoE Control (master -> slave)
    Slave::MultiPDOAssignment::SMConfig sm2;
    sm2.sm_index = 2;
    sm2.phys_start_addr = kSM2PhysAddr;
    sm2.control_byte = kSM2ControlByte;
    sm2.pdo_mappings = {
        {RxPDO_1700.index, RxPDO_1700.size, false},   // not fixed — written explicitly
    };
    assignment.sm_configs.push_back(std::move(sm2));

    // SM3 — FSoE Status (slave -> master)
    Slave::MultiPDOAssignment::SMConfig sm3;
    sm3.sm_index = 3;
    sm3.phys_start_addr = kSM3PhysAddr;
    sm3.control_byte = kSM3ControlByte;
    sm3.pdo_mappings = {
        {TxPDO_1B00.index, TxPDO_1B00.size, false},   // not fixed — written explicitly
    };
    assignment.sm_configs.push_back(std::move(sm3));

    return assignment;
}

/// Build a MultiPDOAssignment with FSoE + motion PDOs.
///
/// FSoE PDOs come FIRST, then motion PDOs.  This ordering is critical
/// because the Synapticon Circulo EtherCAT chip has a bug where the last
/// word in the SM buffer is zeroed.  If the FSoE PDO were last, the
/// ConnectionID (the final word of the FSoE frame) would be zeroed and
/// the slave would reject every frame.  By placing motion PDOs last, the
/// zeroed word falls on motion data, not the FSoE ConnectionID.
/// See: https://doc.synapticon.com/circulo_safe_motion/smm/ecat_fsoe_issues.htm
///
/// SM2 (Rx): [0x1700 (11B)][0x1600 (19B)][0x1601 (8B)][0x1602 (8B)] = 46 bytes
/// SM3 (Tx): [0x1B00 (31B)][0x1A00 (13B)][0x1A01 (12B)][0x1A02 (4B)][0x1A03 (18B)] = 78 bytes
///
/// ALL PDOs (including FSoE) are written explicitly to 0x1C12/0x1C13.
/// SM register length = totalLength() (all PDOs).
/// FMMU length = totalLength() (all PDOs).
inline Slave::MultiPDOAssignment makeCombinedPDOAssignment() {
    Slave::MultiPDOAssignment assignment;

    // SM2 — FSoE Control + Outputs (master -> slave)
    // FSoE PDO first, then motion PDOs
    Slave::MultiPDOAssignment::SMConfig sm2;
    sm2.sm_index = 2;
    sm2.phys_start_addr = kSM2PhysAddr;
    sm2.control_byte = kSM2ControlByte;
    sm2.pdo_mappings = {
        {RxPDO_1700.index, RxPDO_1700.size, false},   // FSoE first (11B)
        {RxPDO_1600.index, RxPDO_1600.size, false},   // 19B
        {RxPDO_1601.index, RxPDO_1601.size, false},   // 8B
        {RxPDO_1602.index, RxPDO_1602.size, false},   // 8B
    };
    assignment.sm_configs.push_back(std::move(sm2));

    // SM3 — FSoE Status + Inputs (slave -> master)
    // FSoE PDO first, then motion PDOs
    Slave::MultiPDOAssignment::SMConfig sm3;
    sm3.sm_index = 3;
    sm3.phys_start_addr = kSM3PhysAddr;
    sm3.control_byte = kSM3ControlByte;
    sm3.pdo_mappings = {
        {TxPDO_1B00.index, TxPDO_1B00.size, false},   // FSoE first (31B)
        {TxPDO_1A00.index, TxPDO_1A00.size, false},   // 13B
        {TxPDO_1A01.index, TxPDO_1A01.size, false},   // 12B
        {TxPDO_1A02.index, TxPDO_1A02.size, false},   // 4B
        {TxPDO_1A03.index, TxPDO_1A03.size, false},   // 18B
    };
    assignment.sm_configs.push_back(std::move(sm3));

    return assignment;
}

/// Build a MultiPDOAssignment with a minimal subset: only RxPDO 0x1600 and TxPDO 0x1A00.
/// This matches the CST (Cyclic Sync Torque) mode configuration.
/// SM2: 0x1600 (19 bytes)
/// SM3: 0x1A00 (13 bytes)
inline Slave::MultiPDOAssignment makeCSTModePDOAssignment() {
    Slave::MultiPDOAssignment assignment;

    Slave::MultiPDOAssignment::SMConfig sm2;
    sm2.sm_index = 2;
    sm2.phys_start_addr = kSM2PhysAddr;
    sm2.control_byte = kSM2ControlByte;
    sm2.pdo_mappings = {
        {RxPDO_1600.index, RxPDO_1600.size},
    };
    assignment.sm_configs.push_back(std::move(sm2));

    Slave::MultiPDOAssignment::SMConfig sm3;
    sm3.sm_index = 3;
    sm3.phys_start_addr = kSM3PhysAddr;
    sm3.control_byte = kSM3ControlByte;
    sm3.pdo_mappings = {
        {TxPDO_1A00.index, TxPDO_1A00.size},
    };
    assignment.sm_configs.push_back(std::move(sm3));

    return assignment;
}

/// Build a MultiPDOAssignment from an explicit list of RxPDO and TxPDO indices.
/// Only known PDO indices (from the ESI) are included; unknown indices are skipped.
///
/// ALL PDOs (including 0x1700/0x1B00) are written explicitly to 0x1C12/0x1C13
/// with fixed=false.  The drive accepts the full ESI PDO assignment.
///
/// @param rxpdo_indices  PDO indices to assign to SM2 (outputs)
/// @param txpdo_indices  PDO indices to assign to SM3 (inputs)
inline Slave::MultiPDOAssignment makePDOAssignment(
    std::initializer_list<uint16_t> rxpdo_indices,
    std::initializer_list<uint16_t> txpdo_indices) {

    Slave::MultiPDOAssignment assignment;

    // SM2 — Outputs
    if (rxpdo_indices.size() > 0) {
        Slave::MultiPDOAssignment::SMConfig sm2;
        sm2.sm_index = 2;
        sm2.phys_start_addr = kSM2PhysAddr;
        sm2.control_byte = kSM2ControlByte;
        for (uint16_t idx : rxpdo_indices) {
            uint16_t sz = 0;
            switch (idx) {
                case 0x1600: sz = RxPDO_1600.size; break;
                case 0x1601: sz = RxPDO_1601.size; break;
                case 0x1602: sz = RxPDO_1602.size; break;
                case 0x1700: sz = RxPDO_1700.size; break;
                default: continue;  // skip unknown
            }
            sm2.pdo_mappings.push_back({idx, sz, false});  // fixed=false
        }
        if (!sm2.pdo_mappings.empty()) {
            assignment.sm_configs.push_back(std::move(sm2));
        }
    }

    // SM3 — Inputs
    if (txpdo_indices.size() > 0) {
        Slave::MultiPDOAssignment::SMConfig sm3;
        sm3.sm_index = 3;
        sm3.phys_start_addr = kSM3PhysAddr;
        sm3.control_byte = kSM3ControlByte;
        for (uint16_t idx : txpdo_indices) {
            uint16_t sz = 0;
            switch (idx) {
                case 0x1A00: sz = TxPDO_1A00.size; break;
                case 0x1A01: sz = TxPDO_1A01.size; break;
                case 0x1A02: sz = TxPDO_1A02.size; break;
                case 0x1A03: sz = TxPDO_1A03.size; break;
                case 0x1B00: sz = TxPDO_1B00.size; break;
                default: continue;  // skip unknown
            }
            sm3.pdo_mappings.push_back({idx, sz, false});  // fixed=false
        }
        if (!sm3.pdo_mappings.empty()) {
            assignment.sm_configs.push_back(std::move(sm3));
        }
    }

    return assignment;
}

} // namespace Synapticon_pdo
} // namespace Drives
} // namespace EtherCAT
