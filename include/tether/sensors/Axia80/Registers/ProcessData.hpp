/**
 * @file ProcessData.hpp
 * @brief Axia80 real-time process data objects (0x6000–0x6030)
 */

#pragma once

#include "tether/sensors/Axia80/Registers/Common.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {
namespace ProcessData {

// ============================================================================
// 0x6000: Reading Data (F/T resolved values)
// ============================================================================

static constexpr uint16_t ReadingDataIndex = 0x6000;

constexpr RegisterEntry ReadingData_Fx = {
    .index = ReadingDataIndex,
    .subindex = 0x01,
    .name = "Fx",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Force X in sensor counts. Engineering units = counts / (0x2021:37)",
};

constexpr RegisterEntry ReadingData_Fy = {
    .index = ReadingDataIndex,
    .subindex = 0x02,
    .name = "Fy",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Force Y in sensor counts. Engineering units = counts / (0x2021:37)",
};

constexpr RegisterEntry ReadingData_Fz = {
    .index = ReadingDataIndex,
    .subindex = 0x03,
    .name = "Fz",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Force Z in sensor counts. Engineering units = counts / (0x2021:37)",
};

constexpr RegisterEntry ReadingData_Tx = {
    .index = ReadingDataIndex,
    .subindex = 0x04,
    .name = "Tx",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Torque X in sensor counts. Engineering units = counts / (0x2021:38)",
};

constexpr RegisterEntry ReadingData_Ty = {
    .index = ReadingDataIndex,
    .subindex = 0x05,
    .name = "Ty",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Torque Y in sensor counts. Engineering units = counts / (0x2021:38)",
};

constexpr RegisterEntry ReadingData_Tz = {
    .index = ReadingDataIndex,
    .subindex = 0x06,
    .name = "Tz",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Torque Z in sensor counts. Engineering units = counts / (0x2021:38)",
};

// ============================================================================
// 0x6010: Status Code
// ============================================================================

static constexpr uint16_t StatusCodeIndex = 0x6010;

constexpr RegisterEntry StatusCode = {
    .index = StatusCodeIndex,
    .subindex = 0x00,
    .name = "Status code",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "32-bit status bitmap. See STATUS_* bit definitions.",
};

// ============================================================================
// 0x6020: Sample Counter
// ============================================================================

static constexpr uint16_t SampleCounterIndex = 0x6020;

constexpr RegisterEntry SampleCounter = {
    .index = SampleCounterIndex,
    .subindex = 0x00,
    .name = "Sample counter",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Increments by 1 per full gage computation set. Rolls over from 2^32-1 to 0.",
};

// ============================================================================
// 0x6030: Raw Unbiased Gage Data
// ============================================================================

static constexpr uint16_t RawGageDataIndex = 0x6030;

constexpr RegisterEntry RawGageData_Gage0 = {
    .index = RawGageDataIndex,
    .subindex = 0x01,
    .name = "Gage 0",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 0",
};

constexpr RegisterEntry RawGageData_Gage1 = {
    .index = RawGageDataIndex,
    .subindex = 0x02,
    .name = "Gage 1",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 1",
};

constexpr RegisterEntry RawGageData_Gage2 = {
    .index = RawGageDataIndex,
    .subindex = 0x03,
    .name = "Gage 2",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 2",
};

constexpr RegisterEntry RawGageData_Gage3 = {
    .index = RawGageDataIndex,
    .subindex = 0x04,
    .name = "Gage 3",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 3",
};

constexpr RegisterEntry RawGageData_Gage4 = {
    .index = RawGageDataIndex,
    .subindex = 0x05,
    .name = "Gage 4",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 4",
};

constexpr RegisterEntry RawGageData_Gage5 = {
    .index = RawGageDataIndex,
    .subindex = 0x06,
    .name = "Gage 5",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 5",
};

constexpr RegisterEntry RawGageData_Gage6 = {
    .index = RawGageDataIndex,
    .subindex = 0x07,
    .name = "Gage 6",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0x80000000,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Unbiased raw strain gage element channel 6",
};

// ============================================================================
// 0x6001: GPIO Inputs
// ============================================================================

static constexpr uint16_t GPIOInputsIndex = 0x6001;

constexpr RegisterEntry GPIOInputs = {
    .index = GPIOInputsIndex,
    .subindex = 0x00,
    .name = "GPIO inputs",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "General-purpose digital input states",
};

inline const RegisterList kRegisterList = {
    &ReadingData_Fx,
    &ReadingData_Fy,
    &ReadingData_Fz,
    &ReadingData_Tx,
    &ReadingData_Ty,
    &ReadingData_Tz,
    &StatusCode,
    &SampleCounter,
    &RawGageData_Gage0,
    &RawGageData_Gage1,
    &RawGageData_Gage2,
    &RawGageData_Gage3,
    &RawGageData_Gage4,
    &RawGageData_Gage5,
    &RawGageData_Gage6,
    &GPIOInputs,
};

} // namespace ProcessData
} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT
