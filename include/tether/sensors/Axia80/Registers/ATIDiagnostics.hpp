/**
 * @file ATIDiagnostics.hpp
 * @brief Axia80 ATI diagnostic and monitoring objects (0x2060–0x2090)
 */

#pragma once

#include "tether/sensors/Axia80/Registers/Common.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {
namespace ATIDiagnostics {

// ============================================================================
// 0x2060: Monitor Condition 1
// ============================================================================

static constexpr uint16_t MonitorCondition1Index = 0x2060;

constexpr RegisterEntry MonitorCondition1_Threshold = {
    .index = MonitorCondition1Index,
    .subindex = 0x01,
    .name = "Threshold value",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Monitor threshold in counts",
};

constexpr RegisterEntry MonitorCondition1_Axis = {
    .index = MonitorCondition1Index,
    .subindex = 0x02,
    .name = "Axis",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis to monitor: 0=Fx, 1=Fy, 2=Fz, 3=Tx, 4=Ty, 5=Tz",
};

// 0x2061: Monitor Condition 2 (same structure)
static constexpr uint16_t MonitorCondition2Index = 0x2061;

constexpr RegisterEntry MonitorCondition2_Threshold = {
    .index = MonitorCondition2Index,
    .subindex = 0x01,
    .name = "Threshold value",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Monitor threshold in counts",
};

constexpr RegisterEntry MonitorCondition2_Axis = {
    .index = MonitorCondition2Index,
    .subindex = 0x02,
    .name = "Axis",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis to monitor: 0=Fx, 1=Fy, 2=Fz, 3=Tx, 4=Ty, 5=Tz",
};

// ============================================================================
// 0x2080: Diagnostic Readings
// ============================================================================

static constexpr uint16_t DiagnosticReadingsIndex = 0x2080;

constexpr RegisterEntry Diagnostic_SupplyVoltage = {
    .index = DiagnosticReadingsIndex,
    .subindex = 0x01,
    .name = "Supply voltage",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Measured external supply voltage x10 (decivolts). Valid range: 12–30V.",
};

constexpr RegisterEntry Diagnostic_GageTemperature = {
    .index = DiagnosticReadingsIndex,
    .subindex = 0x02,
    .name = "Gage temperature",
    .data_type = ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Temperature_C,
    .options_enum = nullptr,
    .min_value = -32768,
    .max_value = 32767,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Silicon/structural temperature x10 (decidegrees C). Valid range: -5 to +70C.",
};

constexpr RegisterEntry Diagnostic_StatusMessage = {
    .index = DiagnosticReadingsIndex,
    .subindex = 0x03,
    .name = "Status message",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Highest priority active error text string (40 chars max)",
};

// ============================================================================
// 0x2090: Version Information
// ============================================================================

static constexpr uint16_t VersionInfoIndex = 0x2090;

constexpr RegisterEntry Version_Major = {
    .index = VersionInfoIndex,
    .subindex = 0x01,
    .name = "Major",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Major firmware version number",
};

constexpr RegisterEntry Version_Minor = {
    .index = VersionInfoIndex,
    .subindex = 0x02,
    .name = "Minor",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Minor firmware version number",
};

constexpr RegisterEntry Version_Revision = {
    .index = VersionInfoIndex,
    .subindex = 0x03,
    .name = "Revision",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Firmware patch/revision level",
};

constexpr RegisterEntry Version_Bootloader = {
    .index = VersionInfoIndex,
    .subindex = 0x04,
    .name = "Bootloader version",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Embedded low-level bootloader firmware version",
};

constexpr RegisterEntry Version_SensorHwVer = {
    .index = VersionInfoIndex,
    .subindex = 0x05,
    .name = "SensorHwVer",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Hardware generation: 2=Gen-2, <2=Gen-1",
};

constexpr RegisterEntry Version_SensorInstrument = {
    .index = VersionInfoIndex,
    .subindex = 0x06,
    .name = "SensorInstrument",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Frontend analog instrumentation circuit tracking revision",
};

inline const RegisterList kRegisterList = {
    &MonitorCondition1_Threshold,
    &MonitorCondition1_Axis,
    &MonitorCondition2_Threshold,
    &MonitorCondition2_Axis,
    &Diagnostic_SupplyVoltage,
    &Diagnostic_GageTemperature,
    &Diagnostic_StatusMessage,
    &Version_Major,
    &Version_Minor,
    &Version_Revision,
    &Version_Bootloader,
    &Version_SensorHwVer,
    &Version_SensorInstrument,
};

} // namespace ATIDiagnostics
} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT
