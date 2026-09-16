#pragma once

#include <cstdint>
#include <span>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace SafetyStatus {

static constexpr uint16_t RSAPStatusIndex                  = 0x4001;
static constexpr uint16_t RSAPInformation1Index            = 0x4002;
static constexpr uint16_t RSAPFaultAndDiscrepancyIndex     = 0x4003;
static constexpr uint16_t SafetyInputDiscrepancyIndex      = 0x4004;
static constexpr uint16_t EmergencyStopStateIndex          = 0x4005;
static constexpr uint16_t ProtectiveStopStateIndex         = 0x4006;
static constexpr uint16_t CollaborativeInputStateIndex     = 0x4007;
static constexpr uint16_t SafetyInputSummaryIndex          = 0x4008;
static constexpr uint16_t OutputDiscrepancyMonitorIndex    = 0x4009;
static constexpr uint16_t OutputStateMonitorIndex         = 0x400A;
static constexpr uint16_t SafetyFunctionDiscrepancyIndex    = 0x400B;
static constexpr uint16_t SafetyFunctionSummaryIndex      = 0x400C;
static constexpr uint16_t EndpointManualReducedSpeedIndex  = 0x400D;
static constexpr uint16_t SafetyTCPManualReducedSpeedIndex = 0x400E;
static constexpr uint16_t SafetyTCPSpeedStateIndex         = 0x400F;
static constexpr uint16_t SafetyTCPForceStateIndex         = 0x4010;
static constexpr uint16_t CartesianPositionStateIndex      = 0x4011;
static constexpr uint16_t AxisPositionStateIndex           = 0x4012;
static constexpr uint16_t AxisSpeedStateIndex              = 0x4013;
static constexpr uint16_t AxisForceStateIndex              = 0x4014;
static constexpr uint16_t RSAPStateMirrorIndex             = 0x4015;
static constexpr uint16_t ErrorCodeMirrorIndex             = 0x4016;

// ---------------------------------------------------------------------------
// Helper macro to reduce boilerplate for single-subindex USINT objects
// ---------------------------------------------------------------------------

#define NEXCOBOT_USINT_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), \
        .subindex = 0x00, \
        .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = (DESC), \
    }

#define NEXCOBOT_UINT_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), \
        .subindex = 0x00, \
        .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = (DESC), \
    }

#define NEXCOBOT_INT_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), \
        .subindex = 0x00, \
        .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = -32768, \
        .max_value = 32767, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = (DESC), \
    }

NEXCOBOT_USINT_REG(RSAPStatus,                RSAPStatusIndex,                  "RSAP Status");
NEXCOBOT_USINT_REG(RSAPInformation1,          RSAPInformation1Index,            "RSAP Information 1");
NEXCOBOT_USINT_REG(RSAPFaultAndDiscrepancy,   RSAPFaultAndDiscrepancyIndex,     "RSAP Fault and Discrepancy");
NEXCOBOT_USINT_REG(SafetyInputDiscrepancy,    SafetyInputDiscrepancyIndex,      "Safety Input Discrepancy");
NEXCOBOT_USINT_REG(EmergencyStopState,        EmergencyStopStateIndex,          "Emergency Stop State");
NEXCOBOT_USINT_REG(ProtectiveStopState,       ProtectiveStopStateIndex,         "Protective Stop State");
NEXCOBOT_USINT_REG(CollaborativeInputState,   CollaborativeInputStateIndex,     "Collaborative Input State");
NEXCOBOT_USINT_REG(SafetyInputSummary,        SafetyInputSummaryIndex,          "Safety Input Summary");

NEXCOBOT_UINT_REG(OutputDiscrepancyMonitor,  OutputDiscrepancyMonitorIndex,    "Output Discrepancy Monitor");
NEXCOBOT_UINT_REG(OutputStateMonitor,         OutputStateMonitorIndex,         "Output State Monitor");

NEXCOBOT_USINT_REG(SafetyFunctionDiscrepancy, SafetyFunctionDiscrepancyIndex,   "Safety Function Discrepancy");
NEXCOBOT_USINT_REG(SafetyFunctionSummary,     SafetyFunctionSummaryIndex,       "Safety Function Summary");
NEXCOBOT_USINT_REG(EndpointManualReducedSpeed, EndpointManualReducedSpeedIndex, "Endpoint Manual Reduced Speed State");

NEXCOBOT_UINT_REG(SafetyTCPManualReducedSpeed, SafetyTCPManualReducedSpeedIndex, "Safety TCP Manual Reduced Speed State");
NEXCOBOT_UINT_REG(SafetyTCPSpeedState,        SafetyTCPSpeedStateIndex,         "Safety TCP Speed State");
NEXCOBOT_UINT_REG(SafetyTCPForceState,        SafetyTCPForceStateIndex,         "Safety TCP Force State");
NEXCOBOT_UINT_REG(CartesianPositionState,     CartesianPositionStateIndex,      "Cartesian Position State");

NEXCOBOT_USINT_REG(AxisPositionState,          AxisPositionStateIndex,           "Axis Position State");
NEXCOBOT_USINT_REG(AxisSpeedState,             AxisSpeedStateIndex,              "Axis Speed State");
NEXCOBOT_USINT_REG(AxisForceState,             AxisForceStateIndex,              "Axis Force State");

NEXCOBOT_INT_REG(RSAPStateMirror,              RSAPStateMirrorIndex,             "RSAP State Mirror");
NEXCOBOT_INT_REG(ErrorCodeMirror,              ErrorCodeMirrorIndex,             "Error Code Mirror");

#undef NEXCOBOT_USINT_REG
#undef NEXCOBOT_UINT_REG
#undef NEXCOBOT_INT_REG

inline const RegisterList kRegisterList = {
    &RSAPStatus,
    &RSAPInformation1,
    &RSAPFaultAndDiscrepancy,
    &SafetyInputDiscrepancy,
    &EmergencyStopState,
    &ProtectiveStopState,
    &CollaborativeInputState,
    &SafetyInputSummary,
    &OutputDiscrepancyMonitor,
    &OutputStateMonitor,
    &SafetyFunctionDiscrepancy,
    &SafetyFunctionSummary,
    &EndpointManualReducedSpeed,
    &SafetyTCPManualReducedSpeed,
    &SafetyTCPSpeedState,
    &SafetyTCPForceState,
    &CartesianPositionState,
    &AxisPositionState,
    &AxisSpeedState,
    &AxisForceState,
    &RSAPStateMirror,
    &ErrorCodeMirror,
};

// ---------------------------------------------------------------------------
// Bit/field labels for the 0x4001-0x4004 status objects.  Used by
// RSAPInformationProcessor to decode triggered bits into human-readable names.
// ---------------------------------------------------------------------------

using ::EtherCAT::Utils::BitLabel;

// 0x4001: RSAP Status (single-bit flags)
static constexpr uint8_t kInMonitoring              = 1u << 0;
static constexpr uint8_t kInCollaborativeOperation  = 1u << 1;
static constexpr uint8_t kInRecovery                = 1u << 2;
static constexpr uint8_t kInDemandState             = 1u << 3;
static constexpr uint8_t kInSafeState               = 1u << 4;
static constexpr uint8_t kInInitialReset            = 1u << 5;

inline BitLabel kRSAPStatusBitLabels[] = {
    BitLabel::bitIfOn("InMonitoring",            kInMonitoring),
    BitLabel::bitIfOn("InCollaborativeOperation", kInCollaborativeOperation),
    BitLabel::bitIfOn("InRecovery",              kInRecovery),
    BitLabel::bitIfOn("InDemandState",           kInDemandState),
    BitLabel::bitIfOn("InSafeState",             kInSafeState),
    BitLabel::bitIfOn("InInitialReset",          kInInitialReset),
};

// 0x4002: RSAP Information 1 (multi-bit fields)
static constexpr uint8_t kRSAPInformation1_OperationModeMask = 0x07u;
static constexpr uint8_t kRSAPInformation1_StopTypeStateMask = 0x18u;
static constexpr uint8_t kRSAPInformation1_StopCategoryMask  = 0xE0u;

static constexpr uint8_t kRSAPInformation1_OperationModeShift = 0u;
static constexpr uint8_t kRSAPInformation1_StopTypeStateShift = 3u;
static constexpr uint8_t kRSAPInformation1_StopCategoryShift  = 5u;

// 0x4003: RSAP Fault and Discrepancy (single-bit flags)
static constexpr uint8_t kFaultReset                = 1u << 0;
static constexpr uint8_t kDecelerationViolation     = 1u << 1;
static constexpr uint8_t kStandstillViolation       = 1u << 2;
static constexpr uint8_t kCrossCheckDataDiscrepancy = 1u << 3;
static constexpr uint8_t kSignalDiscrepancy         = 1u << 4;

inline BitLabel kRSAPFaultAndDiscrepancyBitLabels[] = {
    BitLabel::bitIfOn("FaultReset",                kFaultReset),
    BitLabel::bitIfOn("DecelerationViolation",     kDecelerationViolation),
    BitLabel::bitIfOn("StandstillViolation",       kStandstillViolation),
    BitLabel::bitIfOn("CrossCheckDataDiscrepancy", kCrossCheckDataDiscrepancy),
    BitLabel::bitIfOn("SignalDiscrepancy",         kSignalDiscrepancy),
};

// 0x4004: Safety Input Discrepancy (single-bit flags)
static constexpr uint8_t kBasicEmergencyStopInputDiscrepancy = 1u << 0;
static constexpr uint8_t kProtectiveStopInputDiscrepancy     = 1u << 1;
static constexpr uint8_t kCollaborativeInputDiscrepancy      = 1u << 2;
static constexpr uint8_t kEnablingSwitchInputDiscrepancy     = 1u << 3;
static constexpr uint8_t kHandGuidingInputDiscrepancy        = 1u << 4;
static constexpr uint8_t kOperationModeInputDiscrepancy      = 1u << 5;
static constexpr uint8_t kSafeHoldingInputDiscrepancy        = 1u << 6;
static constexpr uint8_t kResetInputDiscrepancy              = 1u << 7;

inline BitLabel kSafetyInputDiscrepancyBitLabels[] = {
    BitLabel::bitIfOn("BasicEmergencyStopInputDiscrepancy", kBasicEmergencyStopInputDiscrepancy),
    BitLabel::bitIfOn("ProtectiveStopInputDiscrepancy",     kProtectiveStopInputDiscrepancy),
    BitLabel::bitIfOn("CollaborativeInputDiscrepancy",      kCollaborativeInputDiscrepancy),
    BitLabel::bitIfOn("EnablingSwitchInputDiscrepancy",     kEnablingSwitchInputDiscrepancy),
    BitLabel::bitIfOn("HandGuidingInputDiscrepancy",        kHandGuidingInputDiscrepancy),
    BitLabel::bitIfOn("OperationModeInputDiscrepancy",      kOperationModeInputDiscrepancy),
    BitLabel::bitIfOn("SafeHoldingInputDiscrepancy",        kSafeHoldingInputDiscrepancy),
    BitLabel::bitIfOn("ResetInputDiscrepancy",              kResetInputDiscrepancy),
};

// 0x4005: Emergency Stop State
inline BitLabel kEmergencyStopStateBitLabels[] = {
    BitLabel::bitIfOn("BasicEmergencyStopState", 1u << 0),
    BitLabel::bitIfOn("EmergencyStop1State",     1u << 1),
    BitLabel::bitIfOn("EmergencyStop2State",     1u << 2),
    BitLabel::bitIfOn("EmergencyStop3State",     1u << 3),
    BitLabel::bitIfOn("EmergencyStop4State",     1u << 4),
    BitLabel::bitIfOn("EmergencyStop5State",     1u << 5),
    BitLabel::bitIfOn("EmergencyStop6State",     1u << 6),
    BitLabel::bitIfOn("EmergencyStop7State",     1u << 7),
};

// 0x4006: Protective Stop State
inline BitLabel kProtectiveStopStateBitLabels[] = {
    BitLabel::bitIfOn("ProtectiveStop1State", 1u << 0),
    BitLabel::bitIfOn("ProtectiveStop2State", 1u << 1),
    BitLabel::bitIfOn("ProtectiveStop3State", 1u << 2),
    BitLabel::bitIfOn("ProtectiveStop4State", 1u << 3),
    BitLabel::bitIfOn("ProtectiveStop5State", 1u << 4),
    BitLabel::bitIfOn("ProtectiveStop6State", 1u << 5),
    BitLabel::bitIfOn("ProtectiveStop7State", 1u << 6),
    BitLabel::bitIfOn("ProtectiveStop8State", 1u << 7),
};

// 0x4007: Collaborative Input State
inline BitLabel kCollaborativeInputStateBitLabels[] = {
    BitLabel::bitIfOn("Collaborative1State", 1u << 0),
    BitLabel::bitIfOn("Collaborative2State", 1u << 1),
    BitLabel::bitIfOn("Collaborative3State", 1u << 2),
    BitLabel::bitIfOn("Collaborative4State", 1u << 3),
    BitLabel::bitIfOn("Collaborative5State", 1u << 4),
    BitLabel::bitIfOn("Collaborative6State", 1u << 5),
    BitLabel::bitIfOn("Collaborative7State", 1u << 6),
    BitLabel::bitIfOn("Collaborative8State", 1u << 7),
};

// 0x4008: Safety Input Summary
inline BitLabel kSafetyInputSummaryBitLabels[] = {
    BitLabel::bitIfOn("EmergencyStopState",      1u << 0),
    BitLabel::bitIfOn("ProtectiveStopState",     1u << 1),
    BitLabel::bitIfOn("CollaborativeState",      1u << 2),
    BitLabel::bitIfOn("EnablingSwitchState",     1u << 3),
    BitLabel::bitIfOn("HandGuidingState",        1u << 4),
    BitLabel::bitIfOn("OperationModeInputState", 1u << 5),
    BitLabel::bitIfOn("SafeHoldingState",        1u << 6),
    BitLabel::bitIfOn("ResetState",              1u << 7),
};

// 0x4009: Output Discrepancy Monitor
inline BitLabel kOutputDiscrepancyMonitorBitLabels[] = {
    BitLabel::bitIfOn("EmergencyStopOutputDiscrepancy",         1u << 0),
    BitLabel::bitIfOn("ProtectiveStopOutputDiscrepancy",        1u << 1),
    BitLabel::bitIfOn("EnablingSwitchOutputDiscrepancy",        1u << 2),
    BitLabel::bitIfOn("OperationModeOutputDiscrepancy",         1u << 3),
    BitLabel::bitIfOn("ResetOutputDiscrepancy",                 1u << 4),
    BitLabel::bitIfOn("CollaborativeOutputDiscrepancy",         1u << 5),
    BitLabel::bitIfOn("StandstillMonitoringOutputDiscrepancy",  1u << 6),
    BitLabel::bitIfOn("RobotMovingOutputDiscrepancy",           1u << 7),
    BitLabel::bitIfOn("RecoveryStateOutputDiscrepancy",         1u << 8),
    BitLabel::bitIfOn("InT2ModeOutputDiscrepancy",              1u << 9),
    BitLabel::bitIfOn("InExtModeOutputDiscrepancy",             1u << 10),
    BitLabel::bitIfOn("SafeHomeOutputDiscrepancy",              1u << 11),
};

// 0x400A: Output State Monitor
inline BitLabel kOutputStateMonitorBitLabels[] = {
    BitLabel::bitIfOn("EmergencyStopOutputState",         1u << 0),
    BitLabel::bitIfOn("ProtectiveStopOutputState",        1u << 1),
    BitLabel::bitIfOn("EnablingSwitchOutputState",        1u << 2),
    BitLabel::bitIfOn("OperationModeOutputState",         1u << 3),
    BitLabel::bitIfOn("ResetOutputState",                 1u << 4),
    BitLabel::bitIfOn("CollaborativeOutputState",         1u << 5),
    BitLabel::bitIfOn("StandstillMonitoringOutputState",  1u << 6),
    BitLabel::bitIfOn("RobotMovingOutputState",           1u << 7),
    BitLabel::bitIfOn("RecoveryStateOutputState",         1u << 8),
    BitLabel::bitIfOn("InT2ModeOutputState",              1u << 9),
    BitLabel::bitIfOn("InExtModeOutputState",             1u << 10),
    BitLabel::bitIfOn("SafeHomeOutputState",              1u << 11),
};

// 0x400B: Safety Function Discrepancy
inline BitLabel kSafetyFunctionDiscrepancyBitLabels[] = {
    BitLabel::bitIfOn("EndpointManualReducedSpeedStateDiscrepancy",      1u << 0),
    BitLabel::bitIfOn("SafetyTCPManualReducedSpeedStateDiscrepancy",     1u << 1),
    BitLabel::bitIfOn("SafetyTCPSpeedStateDiscrepancy",                  1u << 2),
    BitLabel::bitIfOn("SafetyTCPForceStateDiscrepancy",                  1u << 3),
    BitLabel::bitIfOn("SafetyTCPCartesianPositionStateDiscrepancy",      1u << 4),
    BitLabel::bitIfOn("AxisPositionStateDiscrepancy",                    1u << 5),
    BitLabel::bitIfOn("AxisSpeedStateDiscrepancy",                       1u << 6),
    BitLabel::bitIfOn("AxisForceStateDiscrepancy",                       1u << 7),
};

// 0x400C: Safety Function Summary
inline BitLabel kSafetyFunctionSummaryBitLabels[] = {
    BitLabel::bitIfOn("EndpointManualReducedSpeedState",      1u << 0),
    BitLabel::bitIfOn("SafetyTCPManualReducedSpeedState",     1u << 1),
    BitLabel::bitIfOn("SafetyTCPSpeedState",                  1u << 2),
    BitLabel::bitIfOn("SafetyTCPForceState",                  1u << 3),
    BitLabel::bitIfOn("SafetyTCPCartesianPositionState",      1u << 4),
    BitLabel::bitIfOn("AxisPositionState",                    1u << 5),
    BitLabel::bitIfOn("AxisSpeedState",                       1u << 6),
    BitLabel::bitIfOn("AxisForceState",                       1u << 7),
};

// 0x400D: Endpoint Manual Reduced Speed State
inline BitLabel kEndpointManualReducedSpeedStateBitLabels[] = {
    BitLabel::bitIfOn("Endpoint1MRS", 1u << 0),
    BitLabel::bitIfOn("Endpoint2MRS", 1u << 1),
    BitLabel::bitIfOn("Endpoint3MRS", 1u << 2),
    BitLabel::bitIfOn("Endpoint4MRS", 1u << 3),
    BitLabel::bitIfOn("Endpoint5MRS", 1u << 4),
    BitLabel::bitIfOn("Endpoint6MRS", 1u << 5),
};

// 0x400E: Safety TCP Manual Reduced Speed State
inline BitLabel kSafetyTCPManualReducedSpeedStateBitLabels[] = {
    BitLabel::bitIfOn("TCP0MRS", 1u << 0),
    BitLabel::bitIfOn("TCP1MRS", 1u << 1),
    BitLabel::bitIfOn("TCP2MRS", 1u << 2),
    BitLabel::bitIfOn("TCP3MRS", 1u << 3),
    BitLabel::bitIfOn("TCP4MRS", 1u << 4),
    BitLabel::bitIfOn("TCP5MRS", 1u << 5),
    BitLabel::bitIfOn("TCP6MRS", 1u << 6),
    BitLabel::bitIfOn("TCP7MRS", 1u << 7),
    BitLabel::bitIfOn("TCP8MRS", 1u << 8),
};

// 0x400F: Safety TCP Speed State
inline BitLabel kSafetyTCPSpeedStateBitLabels[] = {
    BitLabel::bitIfOn("TCP0Speed", 1u << 0),
    BitLabel::bitIfOn("TCP1Speed", 1u << 1),
    BitLabel::bitIfOn("TCP2Speed", 1u << 2),
    BitLabel::bitIfOn("TCP3Speed", 1u << 3),
    BitLabel::bitIfOn("TCP4Speed", 1u << 4),
    BitLabel::bitIfOn("TCP5Speed", 1u << 5),
    BitLabel::bitIfOn("TCP6Speed", 1u << 6),
    BitLabel::bitIfOn("TCP7Speed", 1u << 7),
    BitLabel::bitIfOn("TCP8Speed", 1u << 8),
    BitLabel::bitIfOn("Endpoint2TCPSpeed", 1u << 9),
};

// 0x4010: Safety TCP Force State
inline BitLabel kSafetyTCPForceStateBitLabels[] = {
    BitLabel::bitIfOn("TCP0Force", 1u << 0),
    BitLabel::bitIfOn("TCP1Force", 1u << 1),
    BitLabel::bitIfOn("TCP2Force", 1u << 2),
    BitLabel::bitIfOn("TCP3Force", 1u << 3),
    BitLabel::bitIfOn("TCP4Force", 1u << 4),
    BitLabel::bitIfOn("TCP5Force", 1u << 5),
    BitLabel::bitIfOn("TCP6Force", 1u << 6),
    BitLabel::bitIfOn("TCP7Force", 1u << 7),
    BitLabel::bitIfOn("TCP8Force", 1u << 8),
    BitLabel::bitIfOn("Endpoint2ElbowForce", 1u << 9),
};

// 0x4011: Cartesian Position State
inline BitLabel kCartesianPositionStateBitLabels[] = {
    BitLabel::bitIfOn("TCP0CartPos", 1u << 0),
    BitLabel::bitIfOn("TCP1CartPos", 1u << 1),
    BitLabel::bitIfOn("TCP2CartPos", 1u << 2),
    BitLabel::bitIfOn("TCP3CartPos", 1u << 3),
    BitLabel::bitIfOn("TCP4CartPos", 1u << 4),
    BitLabel::bitIfOn("TCP5CartPos", 1u << 5),
    BitLabel::bitIfOn("TCP6CartPos", 1u << 6),
    BitLabel::bitIfOn("TCP7CartPos", 1u << 7),
    BitLabel::bitIfOn("TCP8CartPos", 1u << 8),
    BitLabel::bitIfOn("Endpoint2CartPos", 1u << 9),
};

// 0x4012: Axis Position State
inline BitLabel kAxisPositionStateBitLabels[] = {
    BitLabel::bitIfOn("Axis1Pos", 1u << 0),
    BitLabel::bitIfOn("Axis2Pos", 1u << 1),
    BitLabel::bitIfOn("Axis3Pos", 1u << 2),
    BitLabel::bitIfOn("Axis4Pos", 1u << 3),
    BitLabel::bitIfOn("Axis5Pos", 1u << 4),
    BitLabel::bitIfOn("Axis6Pos", 1u << 5),
};

// 0x4013: Axis Speed State
inline BitLabel kAxisSpeedStateBitLabels[] = {
    BitLabel::bitIfOn("Axis1Speed", 1u << 0),
    BitLabel::bitIfOn("Axis2Speed", 1u << 1),
    BitLabel::bitIfOn("Axis3Speed", 1u << 2),
    BitLabel::bitIfOn("Axis4Speed", 1u << 3),
    BitLabel::bitIfOn("Axis5Speed", 1u << 4),
    BitLabel::bitIfOn("Axis6Speed", 1u << 5),
};

// 0x4014: Axis Force State
inline BitLabel kAxisForceStateBitLabels[] = {
    BitLabel::bitIfOn("Axis1Force", 1u << 0),
    BitLabel::bitIfOn("Axis2Force", 1u << 1),
    BitLabel::bitIfOn("Axis3Force", 1u << 2),
    BitLabel::bitIfOn("Axis4Force", 1u << 3),
    BitLabel::bitIfOn("Axis5Force", 1u << 4),
    BitLabel::bitIfOn("Axis6Force", 1u << 5),
};

/// Returns the set of bit labels for a bitfield object in 0x4001-0x4014,
/// or an empty span for objects without defined labels.
inline std::span<const BitLabel> bitLabelsFor(uint16_t index) {
    switch (index) {
        case RSAPStatusIndex:              return std::span(kRSAPStatusBitLabels);
        case RSAPFaultAndDiscrepancyIndex: return std::span(kRSAPFaultAndDiscrepancyBitLabels);
        case SafetyInputDiscrepancyIndex:  return std::span(kSafetyInputDiscrepancyBitLabels);
        case EmergencyStopStateIndex:      return std::span(kEmergencyStopStateBitLabels);
        case ProtectiveStopStateIndex:     return std::span(kProtectiveStopStateBitLabels);
        case CollaborativeInputStateIndex: return std::span(kCollaborativeInputStateBitLabels);
        case SafetyInputSummaryIndex:      return std::span(kSafetyInputSummaryBitLabels);
        case OutputDiscrepancyMonitorIndex: return std::span(kOutputDiscrepancyMonitorBitLabels);
        case OutputStateMonitorIndex:      return std::span(kOutputStateMonitorBitLabels);
        case SafetyFunctionDiscrepancyIndex: return std::span(kSafetyFunctionDiscrepancyBitLabels);
        case SafetyFunctionSummaryIndex:   return std::span(kSafetyFunctionSummaryBitLabels);
        case EndpointManualReducedSpeedIndex: return std::span(kEndpointManualReducedSpeedStateBitLabels);
        case SafetyTCPManualReducedSpeedIndex: return std::span(kSafetyTCPManualReducedSpeedStateBitLabels);
        case SafetyTCPSpeedStateIndex:     return std::span(kSafetyTCPSpeedStateBitLabels);
        case SafetyTCPForceStateIndex:     return std::span(kSafetyTCPForceStateBitLabels);
        case CartesianPositionStateIndex:  return std::span(kCartesianPositionStateBitLabels);
        case AxisPositionStateIndex:       return std::span(kAxisPositionStateBitLabels);
        case AxisSpeedStateIndex:          return std::span(kAxisSpeedStateBitLabels);
        case AxisForceStateIndex:          return std::span(kAxisForceStateBitLabels);
    }
    return {};
}

} // namespace SafetyStatus
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
