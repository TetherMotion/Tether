#pragma once
#include <cstddef>
#include <cstdint>
#include "tether/slave/mailbox/IMailboxHandler.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"
#include "tether/profiles/cia404/CiA404Defs.hpp"
#include "tether/drives/AS715N/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace AS715N {
namespace U40 {

static constexpr uint16_t U40ObjectIndex = 0x2040; // Group U40 (Running Monitoring)

// U40.00 - Speed reference (read-only, I16, rpm)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedReference = {
    .index = U40ObjectIndex,
    .subindex = 0x01,
    .name = "Speed reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_RPM,
    .min_value = -9000,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.01 - Speed feedback (read-only, I16, rpm)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedback = {
    .index = U40ObjectIndex,
    .subindex = 0x02,
    .name = "Speed feedback",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_RPM,
    .min_value = -9000,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.02 - Torque reference (read-only, I16, 0.1% units)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueReference = {
    .index = U40ObjectIndex,
    .subindex = 0x03,
    .name = "Torque reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Percent, // value reported in 0.1% units
    .min_value = -4000,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.03 - Torque feedback (read-only, I16, 0.1% units)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueFeedback = {
    .index = U40ObjectIndex,
    .subindex = 0x04,
    .name = "Torque feedback",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Percent, // value reported in 0.1% units
    .min_value = -4000,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.04 - DI status (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DIStatus = {
    .index = U40ObjectIndex,
    .subindex = 0x05,
    .name = "DI status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.05 - DO status (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DOStatus = {
    .index = U40ObjectIndex,
    .subindex = 0x06,
    .name = "DO status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.06 - Bus voltage (read-only, U16, 0.1 V units)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BusVoltage = {
    .index = U40ObjectIndex,
    .subindex = 0x07,
    .name = "Bus voltage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Voltage_Volt, // reported in 0.1 V units
    .min_value = 0,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.07 - Average load ratio (read-only, U16, 0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AverageLoadRatio = {
    .index = U40ObjectIndex,
    .subindex = 0x08,
    .name = "Average load ratio",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.08 - Electrical angle (read-only, U16, 0.01°)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ElectricalAngle = {
    .index = U40ObjectIndex,
    .subindex = 0x09,
    .name = "Electrical angle",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None, // reported in 0.01° units
    .min_value = 0,
    .max_value = 36000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.09 - Mechanical angle (read-only, U16, 0.01°)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalAngle = {
    .index = U40ObjectIndex,
    .subindex = 0x0A,
    .name = "Mechanical angle",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None, // reported in 0.01° units
    .min_value = 0,
    .max_value = 36000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.0C - RMS value of phase current (read-only, I16, 0.1 A units)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PhaseCurrentRms = {
    .index = U40ObjectIndex,
    .subindex = 0x0D,
    .name = "RMS value of phase current",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Current_Ampere, // reported in 0.1 A units
    .min_value = -9000,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.10 - Position deviation counter (read-only, I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionDeviationCounter = {
    .index = U40ObjectIndex,
    .subindex = 0x11,
    .name = "Position deviation counter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.14 - Absolute position reference (read-only, I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionReference = {
    .index = U40ObjectIndex,
    .subindex = 0x15,
    .name = "Absolute position reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.16 - Absolute position feedback (reference unit) (read-only, I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionFeedbackRefUnit = {
    .index = U40ObjectIndex,
    .subindex = 0x17,
    .name = "Absolute position feedback (reference unit)",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.18 - Absolute position feedback (encoder unit) low/high (read-only, I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionFeedbackEncoderLow = {
    .index = U40ObjectIndex,
    .subindex = 0x19,
    .name = "Absolute position feedback (encoder unit) - low",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionFeedbackEncoderHigh = {
    .index = U40ObjectIndex,
    .subindex = 0x1B,
    .name = "Absolute position feedback (encoder unit) - high",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.24 - Absolute position feedback (encoder unit) (low 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionFeedbackEncoderLow32 = {
    .index = U40ObjectIndex,
    .subindex = 0x24,
    .name = "Absolute position feedback (encoder unit) - low 32",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.26 - Absolute position feedback (encoder unit) (high 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsolutePositionFeedbackEncoderHigh32 = {
    .index = U40ObjectIndex,
    .subindex = 0x26,
    .name = "Absolute position feedback (encoder unit) - high 32",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.28 - Position feedback in rotation mode (reference unit) (low 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionFeedbackRotationRefLow = {
    .index = U40ObjectIndex,
    .subindex = 0x28,
    .name = "Position feedback in rotation mode (reference unit) - low",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.2A - Position feedback in rotation mode (encoder unit) (low 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionFeedbackRotationEncLow = {
    .index = U40ObjectIndex,
    .subindex = 0x2A,
    .name = "Position feedback in rotation mode (encoder unit) - low",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.2C - Position feedback in rotation mode (encoder unit) (high 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionFeedbackRotationEncHigh = {
    .index = U40ObjectIndex,
    .subindex = 0x2C,
    .name = "Position feedback in rotation mode (encoder unit) - high",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.30 - Heatsink temperature (I16, 0.1°C)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HeatsinkTemperature = {
    .index = U40ObjectIndex,
    .subindex = 0x30,
    .name = "Heatsink temperature",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Temperature_C, // reported in 0.1°C units
    .min_value = -9000,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.34 - Offline inertia auto-tuning value (U16, %)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningValue = {
    .index = U40ObjectIndex,
    .subindex = 0x34,
    .name = "Offline inertia auto-tuning value",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 12000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.36 - Instantaneous value in phase U current (I32, 0.001 A)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InstantaneousPhaseUCurrent = {
    .index = U40ObjectIndex,
    .subindex = 0x36,
    .name = "Instantaneous value in phase U current",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Current_Ampere, // reported in 0.001 A units
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.38 - Instantaneous value in phase V current (I32, 0.001 A)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InstantaneousPhaseVCurrent = {
    .index = U40ObjectIndex,
    .subindex = 0x38,
    .name = "Instantaneous value in phase V current",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Current_Ampere,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.3A - Synchronization cycle measured value (U32, 10 ns)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SyncCycleMeasuredValue = {
    .index = U40ObjectIndex,
    .subindex = 0x3A,
    .name = "Synchronization cycle measured value",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None, // reported in 10 ns units
    .min_value = 0,
    .max_value = static_cast<int64_t>(0x7FFFFFFFu),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.3C - SYNC and IRQ phase value (I32, 10 ns)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SYNCAndIRQPhaseValue = {
    .index = U40ObjectIndex,
    .subindex = 0x3C,
    .name = "SYNC and IRQ phase value",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None, // reported in 10 ns units
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.3E - Drive accumulated heat (U16, 0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DriveAccumulatedHeat = {
    .index = U40ObjectIndex,
    .subindex = 0x3E,
    .name = "Drive accumulated heat",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.3F - Motor accumulated heat (U16, 0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorAccumulatedHeat = {
    .index = U40ObjectIndex,
    .subindex = 0x3F,
    .name = "Motor accumulated heat",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.1D - Encoder single turn information (I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderSingleTurnData = {
    .index = U40ObjectIndex,
    .subindex = 0x1D,
    .name = "Encoder single turn information",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.1F - Encoder multi turn information (U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderMultiTurnPosition = {
    .index = U40ObjectIndex,
    .subindex = 0x1F,
    .name = "Encoder multi turn information",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.20 - Encoder initial Angle (U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderInitialAngle = {
    .index = U40ObjectIndex,
    .subindex = 0x20,
    .name = "Encoder initial Angle",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.21 - Encoder multi turn information low 32 bits (I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderMultiTurnDataLow = {
    .index = U40ObjectIndex,
    .subindex = 0x21,
    .name = "Encoder multi turn information low 32 bits",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U40.23 - Encoder multi turn information High 32 bits (I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderMultiTurnDataHigh = {
    .index = U40ObjectIndex,
    .subindex = 0x23,
    .name = "Encoder multi turn information High 32 bits",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &SpeedReference,
    &SpeedFeedback,
    &TorqueReference,
    &TorqueFeedback,
    &DIStatus,
    &DOStatus,
    &BusVoltage,
    &AverageLoadRatio,
    &ElectricalAngle,
    &MechanicalAngle,
    &PhaseCurrentRms,
    &PositionDeviationCounter,
    &AbsolutePositionReference,
    &AbsolutePositionFeedbackRefUnit,
    &AbsolutePositionFeedbackEncoderLow,
    &AbsolutePositionFeedbackEncoderHigh,
    &AbsolutePositionFeedbackEncoderLow32,
    &AbsolutePositionFeedbackEncoderHigh32,
    &PositionFeedbackRotationRefLow,
    &PositionFeedbackRotationEncLow,
    &PositionFeedbackRotationEncHigh,
    &HeatsinkTemperature,
    &OfflineInertiaAutoTuningValue,
    &InstantaneousPhaseUCurrent,
    &InstantaneousPhaseVCurrent,
    &SyncCycleMeasuredValue,
    &SYNCAndIRQPhaseValue,
    &DriveAccumulatedHeat,
    &MotorAccumulatedHeat,
    &EncoderSingleTurnData,
    &EncoderMultiTurnPosition,
    &EncoderInitialAngle,
    &EncoderMultiTurnDataLow,
    &EncoderMultiTurnDataHigh,
};

} // namespace U40
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
