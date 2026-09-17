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

// ---------------------------------------------------------------------------
// RSAP information objects — ESC211 ESI v0.9 dictionary (0x4000-0x4023).
// These objects are also mapped cyclically into TxPDO 0x1A02
// "TxPDO-Map-RSAP-Info" (46 entries, 55 bytes).
// ---------------------------------------------------------------------------

static constexpr uint16_t RSAPStateIndex                   = 0x4000;
static constexpr uint16_t MonitoringSubStateIndex          = 0x4001;
static constexpr uint16_t ErrorCodeIndex                   = 0x4002;
static constexpr uint16_t OperationModeIndex               = 0x4003;
static constexpr uint16_t StopStateIndex                   = 0x4004;
static constexpr uint16_t StopCategoryIndex                = 0x4005;
static constexpr uint16_t FaultAndViolationStatusIndex     = 0x4006;
static constexpr uint16_t DriveStatusIndex                 = 0x4007;
static constexpr uint16_t DriveValidStatusIndex            = 0x4008;
static constexpr uint16_t DriveUserBitStatusIndex          = 0x4009;
static constexpr uint16_t InputDiscrepancyStatusIndex      = 0x400A;
static constexpr uint16_t EmergencyStopInputStateIndex     = 0x400B;
static constexpr uint16_t NormalStopInputStateIndex        = 0x400C;
static constexpr uint16_t ProtectiveStopInputStateIndex    = 0x400D;
static constexpr uint16_t EnablingDeviceInputStateIndex    = 0x400E;
static constexpr uint16_t OperationModeInputStateIndex     = 0x400F;
static constexpr uint16_t ResetInputStateIndex             = 0x4010;
static constexpr uint16_t CollaborativeInputStateIndex     = 0x4011;
static constexpr uint16_t HGCInputStateIndex               = 0x4012;
static constexpr uint16_t MonitoredPositionInputStateIndex = 0x4013;
static constexpr uint16_t SSMInputStateIndex               = 0x4014;
static constexpr uint16_t OutputDiscrepancyStatusIndex     = 0x4015;
static constexpr uint16_t SafetyOutputStateIndex           = 0x4016;
static constexpr uint16_t SafetyControlFunctionStateIndex  = 0x4017;
static constexpr uint16_t SafetyLimitFunctionStateIndex    = 0x4018;
static constexpr uint16_t AxisPositionLimitStatusIndex     = 0x4019;
static constexpr uint16_t TCPPositionLimitStatusIndex      = 0x401A;
static constexpr uint16_t EndpointPositionLimitStatusIndex = 0x401B;
static constexpr uint16_t AxisSpeedLimitStatusIndex        = 0x401C;
static constexpr uint16_t TCPSpeedLimitStatusIndex         = 0x401D;
static constexpr uint16_t EndpointSpeedLimitStatusIndex    = 0x401E;
static constexpr uint16_t AxisTorqueLimitStatusIndex       = 0x401F;
static constexpr uint16_t TCPForceLimitStatusIndex         = 0x4020;
static constexpr uint16_t Endpoint2ForceLimitStateIndex    = 0x4021;
static constexpr uint16_t TCP0OrientationLimitStateIndex   = 0x4022;
static constexpr uint16_t TCP0RobotPowerLimitStateIndex    = 0x4023;

// ---------------------------------------------------------------------------
// Helper macros to reduce boilerplate
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

#define NEXCOBOT_USINT_SUB_REG(NAME, IDX, SUB, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), \
        .subindex = (SUB), \
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

// Scalar state objects
NEXCOBOT_USINT_REG(RSAPState,                 RSAPStateIndex,               "RSAP state");
NEXCOBOT_USINT_REG(MonitoringSubState,        MonitoringSubStateIndex,    "Sub state in MONITORING state");
NEXCOBOT_INT_REG(ErrorCode,                   ErrorCodeIndex,             "Error code");
NEXCOBOT_USINT_REG(OperationMode,             OperationModeIndex,         "Operation mode");
NEXCOBOT_USINT_REG(StopState,                 StopStateIndex,             "Stop state");
NEXCOBOT_USINT_REG(StopCategory,              StopCategoryIndex,          "Stop category");
NEXCOBOT_USINT_REG(FaultAndViolationStatus,   FaultAndViolationStatusIndex, "Fault and violation status");
NEXCOBOT_USINT_REG(DriveStatus,               DriveStatusIndex,           "Drive status");

// 0x4008: Drive valid status (record, sub 1..5)
NEXCOBOT_USINT_REG(DriveValidStatusCount,     DriveValidStatusIndex,      "Drive valid status count");
NEXCOBOT_USINT_SUB_REG(DrivePositionValidStatus, DriveValidStatusIndex, 0x01, "Drive position valid status");
NEXCOBOT_USINT_SUB_REG(DriveVelocityValidStatus, DriveValidStatusIndex, 0x02, "Drive velocity valid status");
NEXCOBOT_USINT_SUB_REG(DriveErrorStatus,         DriveValidStatusIndex, 0x03, "Drive error status");
NEXCOBOT_USINT_SUB_REG(DriveSTOValidStatus,      DriveValidStatusIndex, 0x04, "Drive STO valid status");
NEXCOBOT_USINT_SUB_REG(DriveSOSValidStatus,      DriveValidStatusIndex, 0x05, "Drive SOS valid status");

// 0x4009: Drive user defined bit status (record, sub 1..7)
NEXCOBOT_USINT_REG(DriveUserBitStatusCount,   DriveUserBitStatusIndex,    "Drive user defined bit status count");
NEXCOBOT_USINT_SUB_REG(Drive1UserBitStatus,   DriveUserBitStatusIndex, 0x01, "Drive 1 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive2UserBitStatus,   DriveUserBitStatusIndex, 0x02, "Drive 2 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive3UserBitStatus,   DriveUserBitStatusIndex, 0x03, "Drive 3 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive4UserBitStatus,   DriveUserBitStatusIndex, 0x04, "Drive 4 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive5UserBitStatus,   DriveUserBitStatusIndex, 0x05, "Drive 5 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive6UserBitStatus,   DriveUserBitStatusIndex, 0x06, "Drive 6 user defined bit status");
NEXCOBOT_USINT_SUB_REG(Drive7UserBitStatus,   DriveUserBitStatusIndex, 0x07, "Drive 7 user defined bit status");

// Input status / discrepancy objects
NEXCOBOT_UINT_REG(InputDiscrepancyStatus,     InputDiscrepancyStatusIndex,  "Input discrepancy status");
NEXCOBOT_USINT_REG(EmergencyStopInputState,   EmergencyStopInputStateIndex, "Emergency stop input state");
NEXCOBOT_USINT_REG(NormalStopInputState,      NormalStopInputStateIndex,    "Normal stop input state");
NEXCOBOT_USINT_REG(ProtectiveStopInputState,  ProtectiveStopInputStateIndex, "Protective stop input state");
NEXCOBOT_USINT_REG(EnablingDeviceInputState,  EnablingDeviceInputStateIndex, "Enabling device input state");
NEXCOBOT_USINT_REG(OperationModeInputState,   OperationModeInputStateIndex, "Operation mode input state");
NEXCOBOT_USINT_REG(ResetInputState,           ResetInputStateIndex,         "Reset input state");
NEXCOBOT_USINT_REG(CollaborativeInputState,   CollaborativeInputStateIndex, "Collaborative input state");
NEXCOBOT_USINT_REG(HGCInputState,             HGCInputStateIndex,           "HGC input state");
NEXCOBOT_USINT_REG(MonitoredPositionInputState, MonitoredPositionInputStateIndex, "Monitored position input state");
NEXCOBOT_USINT_REG(SSMInputState,             SSMInputStateIndex,           "SSM input state");

// Output / function / limit status objects
NEXCOBOT_UINT_REG(OutputDiscrepancyStatus,    OutputDiscrepancyStatusIndex, "Output discrepancy status");
NEXCOBOT_UINT_REG(SafetyOutputState,          SafetyOutputStateIndex,       "Safety output state");
NEXCOBOT_UINT_REG(SafetyControlFunctionState, SafetyControlFunctionStateIndex, "Safety control function state");
NEXCOBOT_UINT_REG(SafetyLimitFunctionState,   SafetyLimitFunctionStateIndex, "Safety limit function state");
NEXCOBOT_USINT_REG(AxisPositionLimitStatus,   AxisPositionLimitStatusIndex, "Axis position limit status");
NEXCOBOT_UINT_REG(TCPPositionLimitStatus,     TCPPositionLimitStatusIndex,  "TCP position limit status");
NEXCOBOT_USINT_REG(EndpointPositionLimitStatus, EndpointPositionLimitStatusIndex, "Endpoint position limit status");
NEXCOBOT_USINT_REG(AxisSpeedLimitStatus,      AxisSpeedLimitStatusIndex,    "Axis speed limit status");
NEXCOBOT_UINT_REG(TCPSpeedLimitStatus,        TCPSpeedLimitStatusIndex,     "TCP speed limit status");
NEXCOBOT_USINT_REG(EndpointSpeedLimitStatus,  EndpointSpeedLimitStatusIndex, "Endpoint speed limit status");
NEXCOBOT_USINT_REG(AxisTorqueLimitStatus,     AxisTorqueLimitStatusIndex,   "Axis torque limit status");
NEXCOBOT_UINT_REG(TCPForceLimitStatus,        TCPForceLimitStatusIndex,     "TCP force limit status");
NEXCOBOT_USINT_REG(Endpoint2ForceLimitState,  Endpoint2ForceLimitStateIndex, "Endpoint 2 force limit state");
NEXCOBOT_USINT_REG(TCP0OrientationLimitState, TCP0OrientationLimitStateIndex, "TCP 0 orientation limit state");
NEXCOBOT_USINT_REG(TCP0RobotPowerLimitState,  TCP0RobotPowerLimitStateIndex, "TCP 0 robot power limit state");

#undef NEXCOBOT_USINT_REG
#undef NEXCOBOT_USINT_SUB_REG
#undef NEXCOBOT_UINT_REG
#undef NEXCOBOT_INT_REG

inline const RegisterList kRegisterList = {
    &RSAPState,
    &MonitoringSubState,
    &ErrorCode,
    &OperationMode,
    &StopState,
    &StopCategory,
    &FaultAndViolationStatus,
    &DriveStatus,
    &DriveValidStatusCount,
    &DrivePositionValidStatus,
    &DriveVelocityValidStatus,
    &DriveErrorStatus,
    &DriveSTOValidStatus,
    &DriveSOSValidStatus,
    &DriveUserBitStatusCount,
    &Drive1UserBitStatus,
    &Drive2UserBitStatus,
    &Drive3UserBitStatus,
    &Drive4UserBitStatus,
    &Drive5UserBitStatus,
    &Drive6UserBitStatus,
    &Drive7UserBitStatus,
    &InputDiscrepancyStatus,
    &EmergencyStopInputState,
    &NormalStopInputState,
    &ProtectiveStopInputState,
    &EnablingDeviceInputState,
    &OperationModeInputState,
    &ResetInputState,
    &CollaborativeInputState,
    &HGCInputState,
    &MonitoredPositionInputState,
    &SSMInputState,
    &OutputDiscrepancyStatus,
    &SafetyOutputState,
    &SafetyControlFunctionState,
    &SafetyLimitFunctionState,
    &AxisPositionLimitStatus,
    &TCPPositionLimitStatus,
    &EndpointPositionLimitStatus,
    &AxisSpeedLimitStatus,
    &TCPSpeedLimitStatus,
    &EndpointSpeedLimitStatus,
    &AxisTorqueLimitStatus,
    &TCPForceLimitStatus,
    &Endpoint2ForceLimitState,
    &TCP0OrientationLimitState,
    &TCP0RobotPowerLimitState,
};

// ---------------------------------------------------------------------------
// Bit/field labels for the bitfield objects.  Used by
// RSAPInformationProcessor to decode set bits into human-readable names.
// The ESI does not publish bit-level semantics; labels below reuse the
// vendor-documented bit layouts for objects whose names carried over
// from the pre-v0.9 dictionary.  Unlabeled objects print "bit N".
// ---------------------------------------------------------------------------

using ::EtherCAT::Utils::BitLabel;

// 0x4006: Fault and violation status
inline BitLabel kFaultAndViolationStatusBitLabels[] = {
    BitLabel::bitIfOn("FaultReset",                1u << 0),
    BitLabel::bitIfOn("DecelerationViolation",     1u << 1),
    BitLabel::bitIfOn("StandstillViolation",       1u << 2),
    BitLabel::bitIfOn("CrossCheckDataDiscrepancy", 1u << 3),
    BitLabel::bitIfOn("SignalDiscrepancy",         1u << 4),
};

// 0x4007: Drive status / 0x4008: Drive valid status / per-drive bits
inline BitLabel kDriveStatusBitLabels[] = {
    BitLabel::bitIfOn("Drive1", 1u << 0),
    BitLabel::bitIfOn("Drive2", 1u << 1),
    BitLabel::bitIfOn("Drive3", 1u << 2),
    BitLabel::bitIfOn("Drive4", 1u << 3),
    BitLabel::bitIfOn("Drive5", 1u << 4),
    BitLabel::bitIfOn("Drive6", 1u << 5),
    BitLabel::bitIfOn("Drive7", 1u << 6),
};

// 0x400A: Input discrepancy status
inline BitLabel kInputDiscrepancyStatusBitLabels[] = {
    BitLabel::bitIfOn("BasicEmergencyStopInputDiscrepancy", 1u << 0),
    BitLabel::bitIfOn("ProtectiveStopInputDiscrepancy",     1u << 1),
    BitLabel::bitIfOn("CollaborativeInputDiscrepancy",      1u << 2),
    BitLabel::bitIfOn("EnablingSwitchInputDiscrepancy",     1u << 3),
    BitLabel::bitIfOn("HandGuidingInputDiscrepancy",        1u << 4),
    BitLabel::bitIfOn("OperationModeInputDiscrepancy",      1u << 5),
    BitLabel::bitIfOn("SafeHoldingInputDiscrepancy",        1u << 6),
    BitLabel::bitIfOn("ResetInputDiscrepancy",              1u << 7),
};

// 0x400B: Emergency stop input state
inline BitLabel kEmergencyStopInputStateBitLabels[] = {
    BitLabel::bitIfOn("BasicEmergencyStopState", 1u << 0),
    BitLabel::bitIfOn("EmergencyStop1State",     1u << 1),
    BitLabel::bitIfOn("EmergencyStop2State",     1u << 2),
    BitLabel::bitIfOn("EmergencyStop3State",     1u << 3),
    BitLabel::bitIfOn("EmergencyStop4State",     1u << 4),
    BitLabel::bitIfOn("EmergencyStop5State",     1u << 5),
    BitLabel::bitIfOn("EmergencyStop6State",     1u << 6),
    BitLabel::bitIfOn("EmergencyStop7State",     1u << 7),
};

// 0x400D: Protective stop input state
inline BitLabel kProtectiveStopInputStateBitLabels[] = {
    BitLabel::bitIfOn("ProtectiveStop1State", 1u << 0),
    BitLabel::bitIfOn("ProtectiveStop2State", 1u << 1),
    BitLabel::bitIfOn("ProtectiveStop3State", 1u << 2),
    BitLabel::bitIfOn("ProtectiveStop4State", 1u << 3),
    BitLabel::bitIfOn("ProtectiveStop5State", 1u << 4),
    BitLabel::bitIfOn("ProtectiveStop6State", 1u << 5),
    BitLabel::bitIfOn("ProtectiveStop7State", 1u << 6),
    BitLabel::bitIfOn("ProtectiveStop8State", 1u << 7),
};

// 0x4011: Collaborative input state
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

// 0x4015: Output discrepancy status
inline BitLabel kOutputDiscrepancyStatusBitLabels[] = {
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

// 0x4016: Safety output state
inline BitLabel kSafetyOutputStateBitLabels[] = {
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

// 0x4018: Safety limit function state
inline BitLabel kSafetyLimitFunctionStateBitLabels[] = {
    BitLabel::bitIfOn("EndpointManualReducedSpeedState",      1u << 0),
    BitLabel::bitIfOn("SafetyTCPManualReducedSpeedState",     1u << 1),
    BitLabel::bitIfOn("SafetyTCPSpeedState",                  1u << 2),
    BitLabel::bitIfOn("SafetyTCPForceState",                  1u << 3),
    BitLabel::bitIfOn("SafetyTCPCartesianPositionState",      1u << 4),
    BitLabel::bitIfOn("AxisPositionState",                    1u << 5),
    BitLabel::bitIfOn("AxisSpeedState",                       1u << 6),
    BitLabel::bitIfOn("AxisForceState",                       1u << 7),
};

// 0x4019: Axis position limit status
inline BitLabel kAxisPositionLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Axis1Pos", 1u << 0),
    BitLabel::bitIfOn("Axis2Pos", 1u << 1),
    BitLabel::bitIfOn("Axis3Pos", 1u << 2),
    BitLabel::bitIfOn("Axis4Pos", 1u << 3),
    BitLabel::bitIfOn("Axis5Pos", 1u << 4),
    BitLabel::bitIfOn("Axis6Pos", 1u << 5),
};

// 0x401A: TCP position limit status
inline BitLabel kTCPPositionLimitStatusBitLabels[] = {
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

// 0x401B: Endpoint position limit status
inline BitLabel kEndpointPositionLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Endpoint1Pos", 1u << 0),
    BitLabel::bitIfOn("Endpoint2Pos", 1u << 1),
    BitLabel::bitIfOn("Endpoint3Pos", 1u << 2),
    BitLabel::bitIfOn("Endpoint4Pos", 1u << 3),
    BitLabel::bitIfOn("Endpoint5Pos", 1u << 4),
    BitLabel::bitIfOn("Endpoint6Pos", 1u << 5),
};

// 0x401C: Axis speed limit status
inline BitLabel kAxisSpeedLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Axis1Speed", 1u << 0),
    BitLabel::bitIfOn("Axis2Speed", 1u << 1),
    BitLabel::bitIfOn("Axis3Speed", 1u << 2),
    BitLabel::bitIfOn("Axis4Speed", 1u << 3),
    BitLabel::bitIfOn("Axis5Speed", 1u << 4),
    BitLabel::bitIfOn("Axis6Speed", 1u << 5),
};

// 0x401D: TCP speed limit status
inline BitLabel kTCPSpeedLimitStatusBitLabels[] = {
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

// 0x401E: Endpoint speed limit status
inline BitLabel kEndpointSpeedLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Endpoint1Speed", 1u << 0),
    BitLabel::bitIfOn("Endpoint2Speed", 1u << 1),
    BitLabel::bitIfOn("Endpoint3Speed", 1u << 2),
    BitLabel::bitIfOn("Endpoint4Speed", 1u << 3),
    BitLabel::bitIfOn("Endpoint5Speed", 1u << 4),
    BitLabel::bitIfOn("Endpoint6Speed", 1u << 5),
};

// 0x401F: Axis torque limit status
inline BitLabel kAxisTorqueLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Axis1Torque", 1u << 0),
    BitLabel::bitIfOn("Axis2Torque", 1u << 1),
    BitLabel::bitIfOn("Axis3Torque", 1u << 2),
    BitLabel::bitIfOn("Axis4Torque", 1u << 3),
    BitLabel::bitIfOn("Axis5Torque", 1u << 4),
    BitLabel::bitIfOn("Axis6Torque", 1u << 5),
};

// 0x4020: TCP force limit status
inline BitLabel kTCPForceLimitStatusBitLabels[] = {
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

/// Returns the set of bit labels for a bitfield object in 0x4006-0x4023,
/// or an empty span for objects without defined labels.
inline std::span<const BitLabel> bitLabelsFor(uint16_t index) {
    switch (index) {
        case FaultAndViolationStatusIndex:     return std::span(kFaultAndViolationStatusBitLabels);
        case DriveStatusIndex:                 return std::span(kDriveStatusBitLabels);
        case DriveValidStatusIndex:            return std::span(kDriveStatusBitLabels);
        case InputDiscrepancyStatusIndex:      return std::span(kInputDiscrepancyStatusBitLabels);
        case EmergencyStopInputStateIndex:     return std::span(kEmergencyStopInputStateBitLabels);
        case ProtectiveStopInputStateIndex:    return std::span(kProtectiveStopInputStateBitLabels);
        case CollaborativeInputStateIndex:     return std::span(kCollaborativeInputStateBitLabels);
        case OutputDiscrepancyStatusIndex:     return std::span(kOutputDiscrepancyStatusBitLabels);
        case SafetyOutputStateIndex:           return std::span(kSafetyOutputStateBitLabels);
        case SafetyLimitFunctionStateIndex:    return std::span(kSafetyLimitFunctionStateBitLabels);
        case AxisPositionLimitStatusIndex:     return std::span(kAxisPositionLimitStatusBitLabels);
        case TCPPositionLimitStatusIndex:      return std::span(kTCPPositionLimitStatusBitLabels);
        case EndpointPositionLimitStatusIndex: return std::span(kEndpointPositionLimitStatusBitLabels);
        case AxisSpeedLimitStatusIndex:        return std::span(kAxisSpeedLimitStatusBitLabels);
        case TCPSpeedLimitStatusIndex:         return std::span(kTCPSpeedLimitStatusBitLabels);
        case EndpointSpeedLimitStatusIndex:    return std::span(kEndpointSpeedLimitStatusBitLabels);
        case AxisTorqueLimitStatusIndex:       return std::span(kAxisTorqueLimitStatusBitLabels);
        case TCPForceLimitStatusIndex:         return std::span(kTCPForceLimitStatusBitLabels);
    }
    return {};
}

} // namespace SafetyStatus
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
