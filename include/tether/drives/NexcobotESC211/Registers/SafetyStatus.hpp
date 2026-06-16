#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

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

} // namespace SafetyStatus
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
