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
// TxPDO 0x1A02 — RSAP Info / Safety Status (ESI v0.9: 46 entries, 55 B)
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 46> TxPDO_1A02_Fields = {{
    { &Reg::SafetyStatus::RSAPState,                    0u,  1, "RSAP state" },
    { &Reg::SafetyStatus::MonitoringSubState,           1u,  1, "Sub state in MONITORING" },
    { &Reg::SafetyStatus::ErrorCode,                    2u,  2, "Error code" },
    { &Reg::SafetyStatus::OperationMode,                4u,  1, "Operation mode" },
    { &Reg::SafetyStatus::StopState,                    5u,  1, "Stop state" },
    { &Reg::SafetyStatus::StopCategory,                 6u,  1, "Stop category" },
    { &Reg::SafetyStatus::FaultAndViolationStatus,      7u,  1, "Fault and violation status" },
    { &Reg::SafetyStatus::DriveStatus,                  8u,  1, "Drive status" },
    { &Reg::SafetyStatus::DrivePositionValidStatus,     9u,  1, "Drive position valid status" },
    { &Reg::SafetyStatus::DriveVelocityValidStatus,    10u,  1, "Drive velocity valid status" },
    { &Reg::SafetyStatus::DriveErrorStatus,            11u,  1, "Drive error status" },
    { &Reg::SafetyStatus::DriveSTOValidStatus,         12u,  1, "Drive STO valid status" },
    { &Reg::SafetyStatus::DriveSOSValidStatus,         13u,  1, "Drive SOS valid status" },
    { &Reg::SafetyStatus::Drive1UserBitStatus,         14u,  1, "Drive 1 user defined bit status" },
    { &Reg::SafetyStatus::Drive2UserBitStatus,         15u,  1, "Drive 2 user defined bit status" },
    { &Reg::SafetyStatus::Drive3UserBitStatus,         16u,  1, "Drive 3 user defined bit status" },
    { &Reg::SafetyStatus::Drive4UserBitStatus,         17u,  1, "Drive 4 user defined bit status" },
    { &Reg::SafetyStatus::Drive5UserBitStatus,         18u,  1, "Drive 5 user defined bit status" },
    { &Reg::SafetyStatus::Drive6UserBitStatus,         19u,  1, "Drive 6 user defined bit status" },
    { &Reg::SafetyStatus::Drive7UserBitStatus,         20u,  1, "Drive 7 user defined bit status" },
    { &Reg::SafetyStatus::InputDiscrepancyStatus,      21u,  2, "Input discrepancy status" },
    { &Reg::SafetyStatus::EmergencyStopInputState,     23u,  1, "Emergency stop input state" },
    { &Reg::SafetyStatus::NormalStopInputState,        24u,  1, "Normal stop input state" },
    { &Reg::SafetyStatus::ProtectiveStopInputState,    25u,  1, "Protective stop input state" },
    { &Reg::SafetyStatus::EnablingDeviceInputState,    26u,  1, "Enabling device input state" },
    { &Reg::SafetyStatus::OperationModeInputState,     27u,  1, "Operation mode input state" },
    { &Reg::SafetyStatus::ResetInputState,             28u,  1, "Reset input state" },
    { &Reg::SafetyStatus::CollaborativeInputState,     29u,  1, "Collaborative input state" },
    { &Reg::SafetyStatus::HGCInputState,               30u,  1, "HGC input state" },
    { &Reg::SafetyStatus::MonitoredPositionInputState, 31u,  1, "Monitored position input state" },
    { &Reg::SafetyStatus::SSMInputState,               32u,  1, "SSM input state" },
    { &Reg::SafetyStatus::OutputDiscrepancyStatus,     33u,  2, "Output discrepancy status" },
    { &Reg::SafetyStatus::SafetyOutputState,           35u,  2, "Safety output state" },
    { &Reg::SafetyStatus::SafetyControlFunctionState,  37u,  2, "Safety control function state" },
    { &Reg::SafetyStatus::SafetyLimitFunctionState,    39u,  2, "Safety limit function state" },
    { &Reg::SafetyStatus::AxisPositionLimitStatus,     41u,  1, "Axis position limit status" },
    { &Reg::SafetyStatus::TCPPositionLimitStatus,      42u,  2, "TCP position limit status" },
    { &Reg::SafetyStatus::EndpointPositionLimitStatus, 44u,  1, "Endpoint position limit status" },
    { &Reg::SafetyStatus::AxisSpeedLimitStatus,        45u,  1, "Axis speed limit status" },
    { &Reg::SafetyStatus::TCPSpeedLimitStatus,         46u,  2, "TCP speed limit status" },
    { &Reg::SafetyStatus::EndpointSpeedLimitStatus,    48u,  1, "Endpoint speed limit status" },
    { &Reg::SafetyStatus::AxisTorqueLimitStatus,       49u,  1, "Axis torque limit status" },
    { &Reg::SafetyStatus::TCPForceLimitStatus,         50u,  2, "TCP force limit status" },
    { &Reg::SafetyStatus::Endpoint2ForceLimitState,    52u,  1, "Endpoint 2 force limit state" },
    { &Reg::SafetyStatus::TCP0OrientationLimitState,   53u,  1, "TCP 0 orientation limit state" },
    { &Reg::SafetyStatus::TCP0RobotPowerLimitState,    54u,  1, "TCP 0 robot power limit state" },
}};

static constexpr PDO TxPDO_1A02 = makePDO(0x1A02u, 55u,
                                            TxPDO_1A02_Fields.data(),
                                            TxPDO_1A02_Fields.size());
static_assert(TxPDO_1A02.field_count == TxPDO_1A02_Fields.size(),
              "TxPDO1A02 field count mismatch");

// ---------------------------------------------------------------------------
// TxPDO 0x1A03 — RSAP Debug (ESI v0.9: 104 entries, 416 B)
// The debug payload is a device-defined blob of per-TCP/drive monitoring
// values (0x4100-0x4115 records); modelled as raw byte ranges.
// ---------------------------------------------------------------------------

static constexpr std::array<PDOField, 2> TxPDO_1A03_Fields = {{
    { nullptr,   0u, 255, "RSAP debug data [0:255]" },
    { nullptr, 255u, 161, "RSAP debug data [255:416]" },
}};

static constexpr PDO TxPDO_1A03 = makePDO(0x1A03u, 416u,
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

// ESI v0.9 layout — see TxPDO_1A02_Fields for field offsets.
struct NexcobotESC211_TxPDO_1A02 {
    uint8_t  rsap_state;
    uint8_t  monitoring_sub_state;
    int16_t  error_code;
    uint8_t  operation_mode;
    uint8_t  stop_state;
    uint8_t  stop_category;
    uint8_t  fault_and_violation_status;
    uint8_t  drive_status;
    uint8_t  drive_valid_status[5];
    uint8_t  drive_user_bit_status[7];
    uint16_t input_discrepancy_status;
    uint8_t  emergency_stop_input_state;
    uint8_t  normal_stop_input_state;
    uint8_t  protective_stop_input_state;
    uint8_t  enabling_device_input_state;
    uint8_t  operation_mode_input_state;
    uint8_t  reset_input_state;
    uint8_t  collaborative_input_state;
    uint8_t  hgc_input_state;
    uint8_t  monitored_position_input_state;
    uint8_t  ssm_input_state;
    uint16_t output_discrepancy_status;
    uint16_t safety_output_state;
    uint16_t safety_control_function_state;
    uint16_t safety_limit_function_state;
    uint8_t  axis_position_limit_status;
    uint16_t tcp_position_limit_status;
    uint8_t  endpoint_position_limit_status;
    uint8_t  axis_speed_limit_status;
    uint16_t tcp_speed_limit_status;
    uint8_t  endpoint_speed_limit_status;
    uint8_t  axis_torque_limit_status;
    uint16_t tcp_force_limit_status;
    uint8_t  endpoint2_force_limit_state;
    uint8_t  tcp0_orientation_limit_state;
    uint8_t  tcp0_robot_power_limit_state;
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A02) == TxPDO_1A02.size,
              "NexcobotESC211_TxPDO_1A02 size mismatch");

// ESI v0.9: 416-byte device-defined RSAP debug blob (0x4100-0x4115 records).
struct NexcobotESC211_TxPDO_1A03 {
    uint8_t data[416];
} __attribute__((packed));

static_assert(sizeof(NexcobotESC211_TxPDO_1A03) == TxPDO_1A03.size,
              "NexcobotESC211_TxPDO_1A03 size mismatch");

} // namespace NexcobotESC211_pdo
} // namespace Drives
} // namespace EtherCAT
