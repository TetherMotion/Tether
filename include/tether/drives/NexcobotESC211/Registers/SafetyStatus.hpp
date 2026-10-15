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
// Bit assignments follow the vendor's "ESC CoE Object" manual (0x4000 tab)
// — they supersede the earlier labels inferred from the pre-v0.9 packed
// RSAP Information object.  Unlabeled objects print "bit N".
//
// Labels are deliberately terse — every display path already names the
// containing object (the RSAP panel's row label, or the "0xXXXX:sub
// <object name>" prefix in decodeAll/decodeTriggered): input-state chips
// are "In1".."In8" (the row says "EStop"/"PStop"/...), discrepancy/state
// chips carry only the function abbreviation, and limit chips carry just
// the element index ("Ax3", "TCP0", "EP2").
//
// Note on record objects: the manual lists record data entries starting at
// "Sub 0"; on the device (verified) sub0 is the USINT entry count and data
// lives at sub 1..N, e.g. 0x4008:0=5, 0x4009:0=7.
// ---------------------------------------------------------------------------

using ::EtherCAT::Utils::BitLabel;

// 0x4001: Sub state in MONITORING state (bitmask; bits may combine —
// Calibration State triggered means the ESC211 safety function is invalid)
inline BitLabel kMonitoringSubStateBitLabels[] = {
    BitLabel::bitIfOn("Normal",        1u << 0),
    BitLabel::bitIfOn("Collab", 1u << 1),
    BitLabel::bitIfOn("Recover",      1u << 2),
    BitLabel::bitIfOn("Demand",        1u << 3),
    BitLabel::bitIfOn("InitRst",  1u << 4),
    BitLabel::bitIfOn("Calib",   1u << 5),
};

// 0x4006: Fault and violation status
// (manual: 0x4017 bits 1,2,3,4,6,7,8 causing demand state sets bit 4;
//          0x4018 bits 0,1,2,3,4,5,6,7,9 causing demand state sets bit 5)
inline BitLabel kFaultAndViolationStatusBitLabels[] = {
    BitLabel::bitIfOn("FltRst",                 1u << 0),
    BitLabel::bitIfOn("DecelViol",      1u << 1),
    BitLabel::bitIfOn("Stndstill",        1u << 2),
    BitLabel::bitIfOn("DioDisc",             1u << 3),
    BitLabel::bitIfOn("CtrlFunc",   1u << 4),
    BitLabel::bitIfOn("LimFunc",  1u << 5),
    BitLabel::bitIfOn("InitRst",      1u << 6),
};

// 0x4007: Drive status — summary bitfield of the aggregated drive state
// (NOT per-drive; per-drive detail is in the 0x4008 sub-indices)
inline BitLabel kDriveStatusBitLabels[] = {
    BitLabel::bitIfOn("PosValid", 1u << 0),
    BitLabel::bitIfOn("VelValid", 1u << 1),
    BitLabel::bitIfOn("Error",    1u << 2),
    BitLabel::bitIfOn("STO",    1u << 3),
    BitLabel::bitIfOn("SOS",    1u << 4),
};

// 0x4008:1-5: per-drive status bitfields — every sub-index uses the same
// Drive1-7 bit layout (sub1 position valid, sub2 velocity valid,
// sub3 error, sub4 STO, sub5 SOS)
inline BitLabel kDriveValidStatusBitLabels[] = {
    BitLabel::bitIfOn("D1", 1u << 0),
    BitLabel::bitIfOn("D2", 1u << 1),
    BitLabel::bitIfOn("D3", 1u << 2),
    BitLabel::bitIfOn("D4", 1u << 3),
    BitLabel::bitIfOn("D5", 1u << 4),
    BitLabel::bitIfOn("D6", 1u << 5),
    BitLabel::bitIfOn("D7", 1u << 6),
};

// 0x4009:1-7: Drive N user defined bit status — user defined bits 1-8
inline BitLabel kDriveUserBitStatusBitLabels[] = {
    BitLabel::bitIfOn("U1", 1u << 0),
    BitLabel::bitIfOn("U2", 1u << 1),
    BitLabel::bitIfOn("U3", 1u << 2),
    BitLabel::bitIfOn("U4", 1u << 3),
    BitLabel::bitIfOn("U5", 1u << 4),
    BitLabel::bitIfOn("U6", 1u << 5),
    BitLabel::bitIfOn("U7", 1u << 6),
    BitLabel::bitIfOn("U8", 1u << 7),
};

// 0x400A: Input discrepancy status
inline BitLabel kInputDiscrepancyStatusBitLabels[] = {
    BitLabel::bitIfOn("EStop",    1u << 0),
    BitLabel::bitIfOn("NStop",       1u << 1),
    BitLabel::bitIfOn("PStop",   1u << 2),
    BitLabel::bitIfOn("EnaDev",   1u << 3),
    BitLabel::bitIfOn("OpMode",    1u << 4),
    BitLabel::bitIfOn("Reset",            1u << 5),
    BitLabel::bitIfOn("Collab",    1u << 6),
    BitLabel::bitIfOn("HGC",              1u << 7),
    BitLabel::bitIfOn("MonPos",1u << 8),
    BitLabel::bitIfOn("SSM",              1u << 9),
};

// 0x400B: Emergency stop input state (bit value 0: not triggered [high
// signal] / 1: triggered [low signal])
inline BitLabel kEmergencyStopInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
    BitLabel::bitIfOn("In3", 1u << 2),
    BitLabel::bitIfOn("In4", 1u << 3),
    BitLabel::bitIfOn("In5", 1u << 4),
    BitLabel::bitIfOn("In6", 1u << 5),
    BitLabel::bitIfOn("In7", 1u << 6),
    BitLabel::bitIfOn("In8", 1u << 7),
};

// 0x400C: Normal stop input state (0: not triggered [high] / 1: triggered [low])
inline BitLabel kNormalStopInputStateBitLabels[] = {
    BitLabel::bitIfOn("In", 1u << 0),
};

// 0x400D: Protective stop input state (bit value 0: not triggered [high]
// signal] / 1: triggered [low signal])
inline BitLabel kProtectiveStopInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
    BitLabel::bitIfOn("In3", 1u << 2),
    BitLabel::bitIfOn("In4", 1u << 3),
    BitLabel::bitIfOn("In5", 1u << 4),
    BitLabel::bitIfOn("In6", 1u << 5),
    BitLabel::bitIfOn("In7", 1u << 6),
    BitLabel::bitIfOn("In8", 1u << 7),
};

// 0x400E: Enabling device input state (bit value 0: not triggered [OFF]
// / 1: triggered [ON])
inline BitLabel kEnablingDeviceInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
    BitLabel::bitIfOn("In3", 1u << 2),
    BitLabel::bitIfOn("In4", 1u << 3),
};

// 0x400F: Operation mode input state (bit value 0: not triggered [low]
// / 1: triggered [high])
inline BitLabel kOperationModeInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
};

// 0x4010: Reset input state (bit value 0: low signal / 1: high signal)
inline BitLabel kResetInputStateBitLabels[] = {
    BitLabel::bitIfOn("Safe",            1u << 0),
    BitLabel::bitIfOn("Comm", 1u << 1),
};

// 0x4011: Collaborative input state (bit value 0: not triggered [low]
// / 1: triggered [high])
inline BitLabel kCollaborativeInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
    BitLabel::bitIfOn("In3", 1u << 2),
    BitLabel::bitIfOn("In4", 1u << 3),
    BitLabel::bitIfOn("In5", 1u << 4),
    BitLabel::bitIfOn("In6", 1u << 5),
    BitLabel::bitIfOn("In7", 1u << 6),
    BitLabel::bitIfOn("In8", 1u << 7),
};

// 0x4012: HGC (hand guiding) input state (bit value 0: not triggered [low]
// / 1: triggered [high])
inline BitLabel kHGCInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
};

// 0x4013: Monitored position input state (bit value 0: not triggered [high]
// / 1: triggered [low])
inline BitLabel kMonitoredPositionInputStateBitLabels[] = {
    BitLabel::bitIfOn("In1", 1u << 0),
    BitLabel::bitIfOn("In2", 1u << 1),
};

// 0x4014: SSM (safe speed monitor) input state — reduce-speed and
// protective-stop inputs interleaved per channel (bit value 0: not
// triggered [high] / 1: triggered [low])
inline BitLabel kSSMInputStateBitLabels[] = {
    BitLabel::bitIfOn("RedSpd1",    1u << 0),
    BitLabel::bitIfOn("PStop1", 1u << 1),
    BitLabel::bitIfOn("RedSpd2",    1u << 2),
    BitLabel::bitIfOn("PStop2", 1u << 3),
    BitLabel::bitIfOn("RedSpd3",    1u << 4),
    BitLabel::bitIfOn("PStop3", 1u << 5),
    BitLabel::bitIfOn("RedSpd4",    1u << 6),
    BitLabel::bitIfOn("PStop4", 1u << 7),
};

// 0x4015: Output discrepancy status
inline BitLabel kOutputDiscrepancyStatusBitLabels[] = {
    BitLabel::bitIfOn("EStop",          1u << 0),
    BitLabel::bitIfOn("NStop",             1u << 1),
    BitLabel::bitIfOn("PStop",         1u << 2),
    BitLabel::bitIfOn("EnaDev",         1u << 3),
    BitLabel::bitIfOn("OpMode",          1u << 4),
    BitLabel::bitIfOn("Reset",                  1u << 5),
    BitLabel::bitIfOn("Collab",          1u << 6),
    BitLabel::bitIfOn("Stndstill",             1u << 7),
    BitLabel::bitIfOn("Moving",            1u << 8),
    BitLabel::bitIfOn("Recover",          1u << 9),
    BitLabel::bitIfOn("T2",               1u << 10),
    BitLabel::bitIfOn("Ext",              1u << 11),
    BitLabel::bitIfOn("MonPos",      1u << 12),
    BitLabel::bitIfOn("RedZone",   1u << 13),
    BitLabel::bitIfOn("PStpZone",1u << 14),
    BitLabel::bitIfOn("SafeHome",               1u << 15),
};

// 0x4016: Safety output state (bit value 0: not triggered / 1: triggered)
inline BitLabel kSafetyOutputStateBitLabels[] = {
    BitLabel::bitIfOn("EStop",          1u << 0),
    BitLabel::bitIfOn("NStop",             1u << 1),
    BitLabel::bitIfOn("PStop",         1u << 2),
    BitLabel::bitIfOn("EnaDev",         1u << 3),
    BitLabel::bitIfOn("OpMode",          1u << 4),
    BitLabel::bitIfOn("Reset",                  1u << 5),
    BitLabel::bitIfOn("Collab",          1u << 6),
    BitLabel::bitIfOn("Stndstill",             1u << 7),
    BitLabel::bitIfOn("Moving",            1u << 8),
    BitLabel::bitIfOn("Recover",          1u << 9),
    BitLabel::bitIfOn("T2",               1u << 10),
    BitLabel::bitIfOn("Ext",              1u << 11),
    BitLabel::bitIfOn("MonPos",      1u << 12),
    BitLabel::bitIfOn("RedZone",   1u << 13),
    BitLabel::bitIfOn("PStpZone",1u << 14),
    BitLabel::bitIfOn("SafeHome",               1u << 15),
};

// 0x4017: Safety control function state (bit value 0: not triggered /
// 1: triggered).  Manual: bits 1,2,3,4,6,7,8 causing demand state also set
// 0x4006 bit 4 (Control function triggered).
inline BitLabel kSafetyControlFunctionStateBitLabels[] = {
    BitLabel::bitIfOn("Reset",           1u << 0),
    BitLabel::bitIfOn("EStop",   1u << 1),
    BitLabel::bitIfOn("PStop",  1u << 2),
    BitLabel::bitIfOn("OpMode",   1u << 3),
    BitLabel::bitIfOn("EnaDev",  1u << 4),
    BitLabel::bitIfOn("Collab",   1u << 5),
    BitLabel::bitIfOn("HGC",             1u << 6),
    BitLabel::bitIfOn("MonPos",1u << 7),
    BitLabel::bitIfOn("SDrive",    1u << 8),
};

// 0x4018: Safety limit function state (bit value 0: not triggered /
// 1: triggered).  Manual: bits 0,1,2,3,4,5,6,7,9,10 causing demand state
// also set 0x4006 bit 5 (Limiting function triggered).
inline BitLabel kSafetyLimitFunctionStateBitLabels[] = {
    BitLabel::bitIfOn("AxPos",           1u << 0),
    BitLabel::bitIfOn("TCPPos",            1u << 1),
    BitLabel::bitIfOn("Orient",         1u << 2),
    BitLabel::bitIfOn("AxSpd",              1u << 3),
    BitLabel::bitIfOn("TCPSpd",               1u << 4),
    BitLabel::bitIfOn("AxTrq",             1u << 5),
    BitLabel::bitIfOn("TCPFrc",               1u << 6),
    BitLabel::bitIfOn("Power",             1u << 7),
    BitLabel::bitIfOn("PwrFrc",          1u << 8),
    BitLabel::bitIfOn("SSM",1u << 9),
    BitLabel::bitIfOn("StndMon",        1u << 10),
    BitLabel::bitIfOn("SafeHome",                    1u << 11),
};

// 0x4019: Axis position limit status (bit value 0: within limit /
// 1: over limit)
inline BitLabel kAxisPositionLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Ax1", 1u << 0),
    BitLabel::bitIfOn("Ax2", 1u << 1),
    BitLabel::bitIfOn("Ax3", 1u << 2),
    BitLabel::bitIfOn("Ax4", 1u << 3),
    BitLabel::bitIfOn("Ax5", 1u << 4),
    BitLabel::bitIfOn("Ax6", 1u << 5),
    BitLabel::bitIfOn("Ax7", 1u << 6),
};

// 0x401A: TCP position limit status (bit value 0: not in Cart. zone /
// 1: in Cart. zone)
inline BitLabel kTCPPositionLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("TCP0", 1u << 0),
    BitLabel::bitIfOn("TCP1", 1u << 1),
    BitLabel::bitIfOn("TCP2", 1u << 2),
    BitLabel::bitIfOn("TCP3", 1u << 3),
    BitLabel::bitIfOn("TCP4", 1u << 4),
    BitLabel::bitIfOn("TCP5", 1u << 5),
    BitLabel::bitIfOn("TCP6", 1u << 6),
    BitLabel::bitIfOn("TCP7", 1u << 7),
    BitLabel::bitIfOn("TCP8", 1u << 8),
};

// 0x401B: Endpoint position limit status (bit value 0: not in Cart. zone /
// 1: in Cart. zone)
inline BitLabel kEndpointPositionLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("EP1", 1u << 0),
    BitLabel::bitIfOn("EP2", 1u << 1),
    BitLabel::bitIfOn("EP3", 1u << 2),
    BitLabel::bitIfOn("EP4", 1u << 3),
    BitLabel::bitIfOn("EP5", 1u << 4),
    BitLabel::bitIfOn("EP6", 1u << 5),
    BitLabel::bitIfOn("EP7", 1u << 6),
};

// 0x401C: Axis speed limit status (bit value 0: under limit / 1: over limit)
inline BitLabel kAxisSpeedLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Ax1", 1u << 0),
    BitLabel::bitIfOn("Ax2", 1u << 1),
    BitLabel::bitIfOn("Ax3", 1u << 2),
    BitLabel::bitIfOn("Ax4", 1u << 3),
    BitLabel::bitIfOn("Ax5", 1u << 4),
    BitLabel::bitIfOn("Ax6", 1u << 5),
    BitLabel::bitIfOn("Ax7", 1u << 6),
};

// 0x401D: TCP speed limit status (bit value 0: under limit / 1: over limit)
inline BitLabel kTCPSpeedLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("TCP0", 1u << 0),
    BitLabel::bitIfOn("TCP1", 1u << 1),
    BitLabel::bitIfOn("TCP2", 1u << 2),
    BitLabel::bitIfOn("TCP3", 1u << 3),
    BitLabel::bitIfOn("TCP4", 1u << 4),
    BitLabel::bitIfOn("TCP5", 1u << 5),
    BitLabel::bitIfOn("TCP6", 1u << 6),
    BitLabel::bitIfOn("TCP7", 1u << 7),
    BitLabel::bitIfOn("TCP8", 1u << 8),
};

// 0x401E: Endpoint speed limit status (bit value 0: under limit /
// 1: over limit)
inline BitLabel kEndpointSpeedLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("EP1", 1u << 0),
    BitLabel::bitIfOn("EP2", 1u << 1),
    BitLabel::bitIfOn("EP3", 1u << 2),
    BitLabel::bitIfOn("EP4", 1u << 3),
    BitLabel::bitIfOn("EP5", 1u << 4),
    BitLabel::bitIfOn("EP6", 1u << 5),
    BitLabel::bitIfOn("EP7", 1u << 6),
};

// 0x401F: Axis torque limit status (bit value 0: under limit /
// 1: over limit)
inline BitLabel kAxisTorqueLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("Ax1", 1u << 0),
    BitLabel::bitIfOn("Ax2", 1u << 1),
    BitLabel::bitIfOn("Ax3", 1u << 2),
    BitLabel::bitIfOn("Ax4", 1u << 3),
    BitLabel::bitIfOn("Ax5", 1u << 4),
    BitLabel::bitIfOn("Ax6", 1u << 5),
    BitLabel::bitIfOn("Ax7", 1u << 6),
};

// 0x4020: TCP force limit status (bit value 0: under limit / 1: over limit)
inline BitLabel kTCPForceLimitStatusBitLabels[] = {
    BitLabel::bitIfOn("TCP0", 1u << 0),
    BitLabel::bitIfOn("TCP1", 1u << 1),
    BitLabel::bitIfOn("TCP2", 1u << 2),
    BitLabel::bitIfOn("TCP3", 1u << 3),
    BitLabel::bitIfOn("TCP4", 1u << 4),
    BitLabel::bitIfOn("TCP5", 1u << 5),
    BitLabel::bitIfOn("TCP6", 1u << 6),
    BitLabel::bitIfOn("TCP7", 1u << 7),
    BitLabel::bitIfOn("TCP8", 1u << 8),
};

/// Returns the set of bit labels for a bitfield object in 0x4001-0x4023,
/// or an empty span for objects without defined labels.
inline std::span<const BitLabel> bitLabelsFor(uint16_t index) {
    switch (index) {
        case MonitoringSubStateIndex:          return std::span(kMonitoringSubStateBitLabels);
        case FaultAndViolationStatusIndex:     return std::span(kFaultAndViolationStatusBitLabels);
        case DriveStatusIndex:                 return std::span(kDriveStatusBitLabels);
        case DriveValidStatusIndex:            return std::span(kDriveValidStatusBitLabels);
        case DriveUserBitStatusIndex:          return std::span(kDriveUserBitStatusBitLabels);
        case InputDiscrepancyStatusIndex:      return std::span(kInputDiscrepancyStatusBitLabels);
        case EmergencyStopInputStateIndex:     return std::span(kEmergencyStopInputStateBitLabels);
        case NormalStopInputStateIndex:        return std::span(kNormalStopInputStateBitLabels);
        case ProtectiveStopInputStateIndex:    return std::span(kProtectiveStopInputStateBitLabels);
        case EnablingDeviceInputStateIndex:    return std::span(kEnablingDeviceInputStateBitLabels);
        case OperationModeInputStateIndex:     return std::span(kOperationModeInputStateBitLabels);
        case ResetInputStateIndex:             return std::span(kResetInputStateBitLabels);
        case CollaborativeInputStateIndex:     return std::span(kCollaborativeInputStateBitLabels);
        case HGCInputStateIndex:               return std::span(kHGCInputStateBitLabels);
        case MonitoredPositionInputStateIndex: return std::span(kMonitoredPositionInputStateBitLabels);
        case SSMInputStateIndex:               return std::span(kSSMInputStateBitLabels);
        case OutputDiscrepancyStatusIndex:     return std::span(kOutputDiscrepancyStatusBitLabels);
        case SafetyOutputStateIndex:           return std::span(kSafetyOutputStateBitLabels);
        case SafetyControlFunctionStateIndex:  return std::span(kSafetyControlFunctionStateBitLabels);
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
