#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace SyncManager {

static constexpr uint16_t RxPDOAssignmentIndex = 0x1C12;
static constexpr uint16_t TxPDOAssignmentIndex = 0x1C13;
static constexpr uint16_t SMOutputParamIndex   = 0x1C32;
static constexpr uint16_t SMInputParamIndex    = 0x1C33;

// ---------------------------------------------------------------------------
// 0x1C12: RxPDO Assignment
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOAssignmentCount = {
    .index = RxPDOAssignmentIndex,
    .subindex = 0x00,
    .name = "RxPDO assign count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of assigned RxPDOs",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOAssignment1 = {
    .index = RxPDOAssignmentIndex,
    .subindex = 0x01,
    .name = "RxPDO assign entry 1",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Assigned PDO index for RxPDO",
};

// ---------------------------------------------------------------------------
// 0x1C13: TxPDO Assignment
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOAssignmentCount = {
    .index = TxPDOAssignmentIndex,
    .subindex = 0x00,
    .name = "TxPDO assign count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of assigned TxPDOs",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOAssignment1 = {
    .index = TxPDOAssignmentIndex,
    .subindex = 0x01,
    .name = "TxPDO assign entry 1",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Assigned PDO index for TxPDO",
};

// ---------------------------------------------------------------------------
// 0x1C32: SM Output Parameter
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputParamCount = {
    .index = SMOutputParamIndex,
    .subindex = 0x00,
    .name = "SM output parameter count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 14,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for SM output parameter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputSyncType = {
    .index = SMOutputParamIndex,
    .subindex = 0x01,
    .name = "Synchronization Type",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output synchronization type",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputCycleTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x02,
    .name = "Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputSyncTypesSupported = {
    .index = SMOutputParamIndex,
    .subindex = 0x04,
    .name = "Synchronization Types supported",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Supported output synchronization types",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputMinCycleTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x05,
    .name = "Minimum Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Minimum output cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputCalcCopyTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x06,
    .name = "Calc and Copy Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output calculation and copy time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputGetCycleTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x08,
    .name = "Get Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Get output cycle time",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputDelayTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x09,
    .name = "Delay Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output delay time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputSync0CycleTime = {
    .index = SMOutputParamIndex,
    .subindex = 0x0A,
    .name = "Sync0 Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Sync0 cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputSMEventMissed = {
    .index = SMOutputParamIndex,
    .subindex = 0x0B,
    .name = "SM-Event Missed",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "SM event missed counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputCycleTimeTooSmall = {
    .index = SMOutputParamIndex,
    .subindex = 0x0C,
    .name = "Cycle Time Too Small",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Cycle time too small counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputShiftTimeTooShort = {
    .index = SMOutputParamIndex,
    .subindex = 0x0D,
    .name = "Shift Time Too Short Counter",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Shift time too short counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMOutputSyncError = {
    .index = SMOutputParamIndex,
    .subindex = 0x20,
    .name = "Sync Error",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output sync error flag",
};

// ---------------------------------------------------------------------------
// 0x1C33: SM Input Parameter
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputParamCount = {
    .index = SMInputParamIndex,
    .subindex = 0x00,
    .name = "SM input parameter count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 14,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for SM input parameter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputSyncType = {
    .index = SMInputParamIndex,
    .subindex = 0x01,
    .name = "Synchronization Type",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Input synchronization type",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputCycleTime = {
    .index = SMInputParamIndex,
    .subindex = 0x02,
    .name = "Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Input cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputSyncTypesSupported = {
    .index = SMInputParamIndex,
    .subindex = 0x04,
    .name = "Synchronization Types supported",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Supported input synchronization types",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputMinCycleTime = {
    .index = SMInputParamIndex,
    .subindex = 0x05,
    .name = "Minimum Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Minimum input cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputCalcCopyTime = {
    .index = SMInputParamIndex,
    .subindex = 0x06,
    .name = "Calc and Copy Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Input calculation and copy time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputGetCycleTime = {
    .index = SMInputParamIndex,
    .subindex = 0x08,
    .name = "Get Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Get input cycle time",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputDelayTime = {
    .index = SMInputParamIndex,
    .subindex = 0x09,
    .name = "Delay Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Input delay time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputSync0CycleTime = {
    .index = SMInputParamIndex,
    .subindex = 0x0A,
    .name = "Sync0 Cycle Time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Sync0 cycle time in ns",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputSMEventMissed = {
    .index = SMInputParamIndex,
    .subindex = 0x0B,
    .name = "SM-Event Missed",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "SM event missed counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputCycleTimeTooSmall = {
    .index = SMInputParamIndex,
    .subindex = 0x0C,
    .name = "Cycle Time Too Small",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Cycle time too small counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputShiftTimeTooShort = {
    .index = SMInputParamIndex,
    .subindex = 0x0D,
    .name = "Shift Time Too Short Counter",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Shift time too short counter",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SMInputSyncError = {
    .index = SMInputParamIndex,
    .subindex = 0x20,
    .name = "Sync Error",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Input sync error flag",
};

inline const RegisterList kRegisterList = {
    &RxPDOAssignmentCount,
    &RxPDOAssignment1,
    &TxPDOAssignmentCount,
    &TxPDOAssignment1,
    &SMOutputParamCount,
    &SMOutputSyncType,
    &SMOutputCycleTime,
    &SMOutputSyncTypesSupported,
    &SMOutputMinCycleTime,
    &SMOutputCalcCopyTime,
    &SMOutputGetCycleTime,
    &SMOutputDelayTime,
    &SMOutputSync0CycleTime,
    &SMOutputSMEventMissed,
    &SMOutputCycleTimeTooSmall,
    &SMOutputShiftTimeTooShort,
    &SMOutputSyncError,
    &SMInputParamCount,
    &SMInputSyncType,
    &SMInputCycleTime,
    &SMInputSyncTypesSupported,
    &SMInputMinCycleTime,
    &SMInputCalcCopyTime,
    &SMInputGetCycleTime,
    &SMInputDelayTime,
    &SMInputSync0CycleTime,
    &SMInputSMEventMissed,
    &SMInputCycleTimeTooSmall,
    &SMInputShiftTimeTooShort,
    &SMInputSyncError,
};

} // namespace SyncManager
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
