/**
 * @file NexcobotESC211PDO.hpp
 * @brief Compile-time PDO layout descriptors and packed structs for
 *        Nexcobot ESC211 (0x1600, 0x1601, 0x1A00, 0x1A01, 0x1A02, 0x1A03)
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>

#include "tether/drives/NexcobotESC211/Registers/FSOERx.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOETx.hpp"
#include "tether/drives/NexcobotESC211/Registers/SafetyStatus.hpp"
#include "tether/drives/NexcobotESC211/Registers/RSAPMonitoring.hpp"
#include "tether/utils/PDO.hpp"

namespace EtherCAT {
namespace Drives {
namespace NexcobotESC211_pdo {

namespace Reg = EtherCAT::Drives::Registers::NexcobotESC211;

struct PDOField {
    const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry* entry;
    uint16_t offset;
    uint8_t size;
    const char* description;
};

struct PDO {
    uint16_t index;
    uint16_t size;
    const PDOField* fields;
    size_t field_count;
};

static constexpr PDO makePDO(uint16_t idx, uint16_t sz, const PDOField* flds, size_t count) {
    return { idx, sz, flds, count };
}

// ---------------------------------------------------------------------------
// RxPDO 0x1600 — FSOE Rx buffers (496 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 16> RxPDO_1600_Fields = {{
    { &Reg::FSOERx::FSOERxPDU_1,    0u, 31, "FSOE_1" },
    { &Reg::FSOERx::FSOERxPDU_2,   31u, 31, "FSOE_2" },
    { &Reg::FSOERx::FSOERxPDU_3,   62u, 31, "FSOE_3" },
    { &Reg::FSOERx::FSOERxPDU_4,   93u, 31, "FSOE_4" },
    { &Reg::FSOERx::FSOERxPDU_5,  124u, 31, "FSOE_5" },
    { &Reg::FSOERx::FSOERxPDU_6,  155u, 31, "FSOE_6" },
    { &Reg::FSOERx::FSOERxPDU_7,  186u, 31, "FSOE_7" },
    { &Reg::FSOERx::FSOERxPDU_8,  217u, 31, "FSOE_8" },
    { &Reg::FSOERx::FSOERxPDU_9,  248u, 31, "FSOE_9" },
    { &Reg::FSOERx::FSOERxPDU_10, 279u, 31, "FSOE_10" },
    { &Reg::FSOERx::FSOERxPDU_11, 310u, 31, "FSOE_11" },
    { &Reg::FSOERx::FSOERxPDU_12, 341u, 31, "FSOE_12" },
    { &Reg::FSOERx::FSOERxPDU_13, 372u, 31, "FSOE_13" },
    { &Reg::FSOERx::FSOERxPDU_14, 403u, 31, "FSOE_14" },
    { &Reg::FSOERx::FSOERxPDU_15, 434u, 31, "FSOE_15" },
    { &Reg::FSOERx::FSOERxPDU_16, 465u, 31, "FSOE_16" },
}};

static constexpr PDO RxPDO_1600 = makePDO(0x1600u, 496u,
                                            RxPDO_1600_Fields.data(),
                                            RxPDO_1600_Fields.size());
static_assert(RxPDO_1600.field_count == RxPDO_1600_Fields.size(),
              "RxPDO1600 field count mismatch");

// ---------------------------------------------------------------------------
// RxPDO 0x1601 — OutputCounter + SAFE_DO (8 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 2> RxPDO_1601_Fields = {{
    { &Reg::FSOETx::OutputCounter, 0u, 4, "OutputCounter" },
    { &Reg::FSOETx::SAFE_DO,       4u, 4, "SAFE_DO" },
}};

static constexpr PDO RxPDO_1601 = makePDO(0x1601u, 8u,
                                            RxPDO_1601_Fields.data(),
                                            RxPDO_1601_Fields.size());
static_assert(RxPDO_1601.field_count == RxPDO_1601_Fields.size(),
              "RxPDO1601 field count mismatch");

// ---------------------------------------------------------------------------
// TxPDO 0x1A00 — FSOE Tx buffers (496 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 16> TxPDO_1A00_Fields = {{
    { &Reg::FSOETx::FSOETxPDU_1,    0u, 31, "FSOE_1" },
    { &Reg::FSOETx::FSOETxPDU_2,   31u, 31, "FSOE_2" },
    { &Reg::FSOETx::FSOETxPDU_3,   62u, 31, "FSOE_3" },
    { &Reg::FSOETx::FSOETxPDU_4,   93u, 31, "FSOE_4" },
    { &Reg::FSOETx::FSOETxPDU_5,  124u, 31, "FSOE_5" },
    { &Reg::FSOETx::FSOETxPDU_6,  155u, 31, "FSOE_6" },
    { &Reg::FSOETx::FSOETxPDU_7,  186u, 31, "FSOE_7" },
    { &Reg::FSOETx::FSOETxPDU_8,  217u, 31, "FSOE_8" },
    { &Reg::FSOETx::FSOETxPDU_9,  248u, 31, "FSOE_9" },
    { &Reg::FSOETx::FSOETxPDU_10, 279u, 31, "FSOE_10" },
    { &Reg::FSOETx::FSOETxPDU_11, 310u, 31, "FSOE_11" },
    { &Reg::FSOETx::FSOETxPDU_12, 341u, 31, "FSOE_12" },
    { &Reg::FSOETx::FSOETxPDU_13, 372u, 31, "FSOE_13" },
    { &Reg::FSOETx::FSOETxPDU_14, 403u, 31, "FSOE_14" },
    { &Reg::FSOETx::FSOETxPDU_15, 434u, 31, "FSOE_15" },
    { &Reg::FSOETx::FSOETxPDU_16, 465u, 31, "FSOE_16" },
}};

static constexpr PDO TxPDO_1A00 = makePDO(0x1A00u, 496u,
                                            TxPDO_1A00_Fields.data(),
                                            TxPDO_1A00_Fields.size());
static_assert(TxPDO_1A00.field_count == TxPDO_1A00_Fields.size(),
              "TxPDO1A00 field count mismatch");

// ---------------------------------------------------------------------------
// TxPDO 0x1A01 — Input / DO / DI monitors (28 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 7> TxPDO_1A01_Fields = {{
    { &Reg::FSOERx::InputCounter,  0u, 4, "InputCounter" },
    { &Reg::FSOERx::SAFE_DI,      4u, 4, "SAFE_DI" },
    { &Reg::FSOERx::PowerStatus,  8u, 4, "Power_Status" },
    { &Reg::FSOERx::DOMonitor,   12u, 4, "DO_Monitor" },
    { &Reg::FSOERx::DOValueActual, 16u, 4, "DO_Valu" },
    { &Reg::FSOERx::DIValue,     20u, 4, "DI_Valu" },
    { &Reg::FSOERx::DOCommand,   24u, 4, "DO_Command" },
}};

static constexpr PDO TxPDO_1A01 = makePDO(0x1A01u, 28u,
                                            TxPDO_1A01_Fields.data(),
                                            TxPDO_1A01_Fields.size());
static_assert(TxPDO_1A01.field_count == TxPDO_1A01_Fields.size(),
              "TxPDO1A01 field count mismatch");

// ---------------------------------------------------------------------------
// TxPDO 0x1A02 — RSAP Info / Safety Status (30 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 22> TxPDO_1A02_Fields = {{
    { &Reg::SafetyStatus::RSAPStatus,                    0u,  1, "SAFP Operation State" },
    { &Reg::SafetyStatus::RSAPInformation1,              1u,  1, "SAFP Information 1" },
    { &Reg::SafetyStatus::RSAPFaultAndDiscrepancy,       2u,  1, "SAFP Information 2" },
    { &Reg::SafetyStatus::SafetyInputDiscrepancy,        3u,  1, "Input Discrepancy" },
    { &Reg::SafetyStatus::EmergencyStopState,            4u,  1, "Emergency Stop Input State" },
    { &Reg::SafetyStatus::ProtectiveStopState,           5u,  1, "Protective Stop Input State" },
    { &Reg::SafetyStatus::CollaborativeInputState,       6u,  1, "Collaborative Input State" },
    { &Reg::SafetyStatus::SafetyInputSummary,            7u,  1, "Input State" },
    { &Reg::SafetyStatus::OutputDiscrepancyMonitor,       8u,  2, "Output Discrepancy" },
    { &Reg::SafetyStatus::OutputStateMonitor,           10u,  2, "Output State" },
    { &Reg::SafetyStatus::SafetyFunctionDiscrepancy,    12u,  1, "SAFP Limiting Functions State Discrepancy" },
    { &Reg::SafetyStatus::SafetyFunctionSummary,         13u,  1, "SAFP Limiting Functions State" },
    { &Reg::SafetyStatus::EndpointManualReducedSpeed,    14u,  1, "Endpoint Manual Reduced Speed State" },
    { &Reg::SafetyStatus::SafetyTCPManualReducedSpeed,   15u,  2, "Safety TCP Manual Reduced Speed State" },
    { &Reg::SafetyStatus::SafetyTCPSpeedState,           17u,  2, "Safety TCP Speed State" },
    { &Reg::SafetyStatus::SafetyTCPForceState,           19u,  2, "Safety TCP Force State" },
    { &Reg::SafetyStatus::CartesianPositionState,        21u,  2, "Safety TCP Cartesian Position State" },
    { &Reg::SafetyStatus::AxisPositionState,             23u,  1, "Axis Position State" },
    { &Reg::SafetyStatus::AxisSpeedState,                24u,  1, "Axis Speed State" },
    { &Reg::SafetyStatus::AxisForceState,                25u,  1, "Axis Force State" },
    { &Reg::SafetyStatus::RSAPStateMirror,               26u,  2, "SAFP State" },
    { &Reg::SafetyStatus::ErrorCodeMirror,               28u,  2, "Error Code" },
}};

static constexpr PDO TxPDO_1A02 = makePDO(0x1A02u, 30u,
                                            TxPDO_1A02_Fields.data(),
                                            TxPDO_1A02_Fields.size());
static_assert(TxPDO_1A02.field_count == TxPDO_1A02_Fields.size(),
              "TxPDO1A02 field count mismatch");

// ---------------------------------------------------------------------------
// TxPDO 0x1A03 — RSAP Debug (44 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 11> TxPDO_1A03_Fields = {{
    { &Reg::RSAPMonitoring::RSAPTCP1MonitoringVelocity,  0u,  4, "RSAP Calculate TCP Monitoring Velocity" },
    { &Reg::RSAPMonitoring::RSAPTCP1PositionX,           4u,  4, "RSAP Calculate TCP Position X" },
    { &Reg::RSAPMonitoring::RSAPTCP1PositionY,           8u,  4, "RSAP Calculate TCP Position Y" },
    { nullptr,                                           12u,  4, "RSAP Calculate TCP Position Z" },
    { nullptr,                                           16u,  4, "RSAP Calculate TCP Velocity" },
    { nullptr,                                           20u,  4, "RSAP Calculate TCP Force" },
    { nullptr,                                           24u,  4, "RSAP Calculate Elbow Position X" },
    { nullptr,                                           28u,  4, "RSAP Calculate Elbow Position Y" },
    { nullptr,                                           32u,  4, "RSAP Calculate Elbow Position Z" },
    { nullptr,                                           36u,  4, "RSAP Calculate Elbow Velocity" },
    { nullptr,                                           40u,  4, "RSAP Calculate Elbow Force" },
}};

static constexpr PDO TxPDO_1A03 = makePDO(0x1A03u, 44u,
                                            TxPDO_1A03_Fields.data(),
                                            TxPDO_1A03_Fields.size());
static_assert(TxPDO_1A03.field_count == TxPDO_1A03_Fields.size(),
              "TxPDO1A03 field count mismatch");

// ---------------------------------------------------------------------------
// Descriptor vectors
// ---------------------------------------------------------------------------

inline const std::vector<const PDO*> kAllPDOs = {
    &RxPDO_1600, &RxPDO_1601,
    &TxPDO_1A00, &TxPDO_1A01, &TxPDO_1A02, &TxPDO_1A03,
};

inline const std::vector<const PDO*> kRxPDOs = {
    &RxPDO_1600, &RxPDO_1601,
};

inline const std::vector<const PDO*> kTxPDOs = {
    &TxPDO_1A00, &TxPDO_1A01, &TxPDO_1A02, &TxPDO_1A03,
};

inline constexpr const PDO* findPDOByIndex(uint16_t idx) noexcept
{
    return EtherCAT::Utils::findPDOByIndex(kAllPDOs, idx);
}

// ===========================================================================
// Packed PDO structs
// ===========================================================================

struct NexcobotESC211_RxPDO_1600 {
    uint8_t fsoe[16][31];
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_RxPDO_1600) == RxPDO_1600.size,
              "NexcobotESC211_RxPDO_1600 size mismatch");

struct NexcobotESC211_RxPDO_1601 {
    uint32_t output_counter;
    uint32_t safe_do;
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_RxPDO_1601) == RxPDO_1601.size,
              "NexcobotESC211_RxPDO_1601 size mismatch");

struct NexcobotESC211_TxPDO_1A00 {
    uint8_t fsoe[16][31];
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A00) == TxPDO_1A00.size,
              "NexcobotESC211_TxPDO_1A00 size mismatch");

struct NexcobotESC211_TxPDO_1A01 {
    uint32_t input_counter;
    uint32_t safe_di;
    uint32_t power_status;
    uint32_t do_monitor;
    uint32_t do_valu;
    uint32_t di_valu;
    uint32_t do_command;
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A01) == TxPDO_1A01.size,
              "NexcobotESC211_TxPDO_1A01 size mismatch");

struct NexcobotESC211_TxPDO_1A02 {
    uint8_t  safp_operation_state;
    uint8_t  safp_information_1;
    uint8_t  safp_information_2;
    uint8_t  input_discrepancy;
    uint8_t  emergency_stop_input_state;
    uint8_t  protective_stop_input_state;
    uint8_t  collaborative_input_state;
    uint8_t  input_state;
    uint16_t output_discrepancy;
    uint16_t output_state;
    uint8_t  safp_limiting_functions_state_discrepancy;
    uint8_t  safp_limiting_functions_state;
    uint8_t  endpoint_manual_reduced_speed_state;
    uint16_t safety_tcp_manual_reduced_speed_state;
    uint16_t safety_tcp_speed_state;
    uint16_t safety_tcp_force_state;
    uint16_t safety_tcp_cartesian_position_state;
    uint8_t  axis_position_state;
    uint8_t  axis_speed_state;
    uint8_t  axis_force_state;
    int16_t  safp_state;
    int16_t  error_code;
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A02) == TxPDO_1A02.size,
              "NexcobotESC211_TxPDO_1A02 size mismatch");

struct NexcobotESC211_TxPDO_1A03 {
    uint32_t tcp_monitoring_velocity;
    int32_t  tcp_position_x;
    int32_t  tcp_position_y;
    int32_t  tcp_position_z;
    uint32_t tcp_velocity;
    uint32_t tcp_force;
    int32_t  elbow_position_x;
    int32_t  elbow_position_y;
    int32_t  elbow_position_z;
    uint32_t elbow_velocity;
    uint32_t elbow_force;
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A03) == TxPDO_1A03.size,
              "NexcobotESC211_TxPDO_1A03 size mismatch");

} // namespace NexcobotESC211_pdo
} // namespace Drives
} // namespace EtherCAT
