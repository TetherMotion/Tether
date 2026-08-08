/**
 * @file SynapticonPDO.hpp
 * @brief Synapticon SOMANET CiA402 drive — PDO layout definitions
 *
 * Packed structs and constexpr descriptors matching the PDO mappings defined
 * in the SOMANET_CiA_402_v5.1.9.xml ESI file.
 *
 * RxPDOs (master -> slave, SM2):
 *   0x1600  RxPDO Mapping 1  (19 bytes) — controlword, mode, torque, position, velocity, torque offset, tuning cmd
 *   0x1601  RxPDO Mapping 2  (8 bytes)  — physical outputs, bit mask
 *   0x1602  RxPDO Mapping 3  (8 bytes)  — user MOSI, velocity offset
 *
 * TxPDOs (slave -> master, SM3):
 *   0x1A00  TxPDO Mapping 1  (13 bytes) — statusword, mode display, position, velocity, torque
 *   0x1A01  TxPDO Mapping 2  (12 bytes) — analog inputs 1-4, tuning status
 *   0x1A02  TxPDO Mapping 3  (4 bytes)  — digital inputs
 *   0x1A03  TxPDO Mapping 4  (18 bytes) — user MISO, timestamp, position demand, velocity demand, torque demand
 */

#pragma once

#include <cstdint>

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

} // namespace Synapticon_pdo
} // namespace Drives
} // namespace EtherCAT
