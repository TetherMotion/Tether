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
namespace C01 {

static constexpr uint16_t C01ObjectIndex = 0x2001; // Group C01 (Basic Gain Parameters)

// Option enums for C01
enum class SpeedFeedbackFilterOptions : uint16_t {
    InternalSetting = 0,
    LowPassFilter = 1,
    OverlappingAverageFilter = 2,
    SpeedObserver = 3,
    NoFilter = 4,
};

// Register entries for C01
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionLoop1Gain = {
    .index = C01ObjectIndex,
    .subindex = 0x01,
    .name = "1st position loop gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 400,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedLoop1Gain = {
    .index = C01ObjectIndex,
    .subindex = 0x02,
    .name = "1st speed loop gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 250,
    .min_value = 1,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedLoop1IntegralTime = {
    .index = C01ObjectIndex,
    .subindex = 0x03,
    .name = "1st speed loop integral time parameter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3184,
    .min_value = 1,
    .max_value = 51200,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueRefFilterCutoffFrequency1 = {
    .index = C01ObjectIndex,
    .subindex = 0x04,
    .name = "1st torque reference filter cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 200,
    .unit = Unit_Frequency_Hertz,
    .min_value = 5,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionLoop2Gain = {
    .index = C01ObjectIndex,
    .subindex = 0x08,
    .name = "2nd position loop gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 560,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedLoop2Gain = {
    .index = C01ObjectIndex,
    .subindex = 0x09,
    .name = "2nd speed loop gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 350,
    .min_value = 1,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedLoop2IntegralTime = {
    .index = C01ObjectIndex,
    .subindex = 0x0A,
    .name = "2nd speed loop integral time parameter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 2274,
    .min_value = 1,
    .max_value = 51200,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueRefFilterCutoffFrequency2 = {
    .index = C01ObjectIndex,
    .subindex = 0x0B,
    .name = "2nd torque reference filter cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 280,
    .unit = Unit_Frequency_Hertz,
    .min_value = 5,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedbackFilter = {
    .index = C01ObjectIndex,
    .subindex = 0x10,
    .name = "Speed feedback filter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<SpeedFeedbackFilterOptions>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedbackLPFCutoffFreq = {
    .index = C01ObjectIndex,
    .subindex = 0x11,
    .name = "Cutoff frequency of speed feedback low-pass filter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// Additional speed-feedback related options (C01.12, C01.13)
enum class SpeedFeedbackOverlapFilterOptions : uint16_t {
    NoFilter = 0,
    Times2 = 1,
    Times4 = 2,
    Times8 = 3,
    Times16 = 4,
    Times32 = 5,
    Times64 = 6,
};

enum class SpeedFeedforwardSourceOptions : uint16_t {
    NoFeedforward = 0,
    InternalReference = 1,
    ModelTracking = 2,
    Communication = 5,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedbackOverlapFilterTimeConstant = {
    .index = C01ObjectIndex,
    .subindex = 0x12,
    .name = "Speed feedback overlapping average filter time constant",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<SpeedFeedbackOverlapFilterOptions>{},
    .min_value = 0,
    .max_value = 6,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedforwardSource = {
    .index = C01ObjectIndex,
    .subindex = 0x13,
    .name = "Speed feedforward source",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<SpeedFeedforwardSourceOptions>{},
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedforwardPercentage = {
    .index = C01ObjectIndex,
    .subindex = 0x14,
    .name = "Speed feedforward percentage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedFeedforwardFilterCutoffFreq = {
    .index = C01ObjectIndex,
    .subindex = 0x15,
    .name = "Speed feedforward filter cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 318,
    .unit = Unit_Frequency_Hertz,
    .min_value = 5,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

enum class TorqueFeedforwardSourceOptions : uint16_t {
    NoFeedforward = 0,
    InternalReference = 1,
    ModelTracking = 2,
    Communication = 5,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueFeedforwardSource = {
    .index = C01ObjectIndex,
    .subindex = 0x16,
    .name = "Torque feedforward source",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<TorqueFeedforwardSourceOptions>{},
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueFeedforwardPercentage = {
    .index = C01ObjectIndex,
    .subindex = 0x17,
    .name = "Torque feedforward percentage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueFeedforwardFilterCutoffFreq = {
    .index = C01ObjectIndex,
    .subindex = 0x18,
    .name = "Torque feedforward filter cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 318,
    .unit = Unit_Frequency_Hertz,
    .min_value = 5,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PDFFControlCoefficient = {
    .index = C01ObjectIndex,
    .subindex = 0x1B,
    .name = "PDFF control coefficient",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DampingFactorControlCoefficient = {
    .index = C01ObjectIndex,
    .subindex = 0x1C,
    .name = "Damping factor control coefficient",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefOverlapFilterTimeConstantA = {
    .index = C01ObjectIndex,
    .subindex = 0x20,
    .name = "Position reference overlapping average filter time constant A",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1280,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefOverlapFilterTimeConstantB = {
    .index = C01ObjectIndex,
    .subindex = 0x21,
    .name = "Position reference overlapping average filter time constant B",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1280,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefLPFTimeConstantA = {
    .index = C01ObjectIndex,
    .subindex = 0x22,
    .name = "Position reference low-pass filter time constant A",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefLPFTimeConstantB = {
    .index = C01ObjectIndex,
    .subindex = 0x23,
    .name = "Position reference low-pass filter time constant B",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefNotchFilterFrequency1 = {
    .index = C01ObjectIndex,
    .subindex = 0x24,
    .name = "1st notch filter frequency of position reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRefNotchFilterWidth1 = {
    .index = C01ObjectIndex,
    .subindex = 0x25,
    .name = "1st notch filter width of position reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AdaptiveNotchTestTimes = {
    .index = C01ObjectIndex,
    .subindex = 0x31,
    .name = "Adaptive notch test times",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// Gain switchover options (C01.38)
enum class GainSwitchoverModeOptions : uint16_t {
    FixedToFirstGainSet = 0,
    DISwitchover = 1,
    DIPPISwitchover = 2,
    TorqueReference = 3,
    SpeedReference = 4,
    SpeedFeedback = 5,
    SpeedReferenceChangeRate = 6,
    PositionDeviation = 7,
    PositionReference = 8,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry GainSwitchoverMode = {
    .index = C01ObjectIndex,
    .subindex = 0x38,
    .name = "Gain switchover mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<GainSwitchoverModeOptions>{},
    .min_value = 0,
    .max_value = 8,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry GainSwitchoverTime = {
    .index = C01ObjectIndex,
    .subindex = 0x39,
    .name = "Gain switchover time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 50,
    .min_value = 10,
    .max_value = 10000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry GainSwitchoverThreshold = {
    .index = C01ObjectIndex,
    .subindex = 0x3A,
    .name = "Gain switchover threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry GainSwitchoverLoopWidth = {
    .index = C01ObjectIndex,
    .subindex = 0x3B,
    .name = "Gain switchover loop width",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfFirstNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x40,
    .name = "Frequency of the 1st notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry WidthLevelOfFirstNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x41,
    .name = "Width level of the 1st notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DepthLevelOfFirstNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x42,
    .name = "Depth level of the 1st notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfSecondNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x43,
    .name = "Frequency of the 2nd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry WidthLevelOfSecondNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x44,
    .name = "Width level of the 2nd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DepthLevelOfSecondNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x45,
    .name = "Depth level of the 2nd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfThirdNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x46,
    .name = "Frequency of the 3rd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry WidthLevelOfThirdNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x47,
    .name = "Width level of the 3rd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DepthLevelOfThirdNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x48,
    .name = "Depth level of the 3rd notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfFourthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x49,
    .name = "Frequency of the 4th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry WidthLevelOfFourthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x4A,
    .name = "Width level of the 4th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DepthLevelOfFourthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x4B,
    .name = "Depth level of the 4th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfFifthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x4C,
    .name = "Frequency of the 5th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry WidthLevelOfFifthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x4D,
    .name = "Width level of the 5th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DepthLevelOfFifthNotch = {
    .index = C01ObjectIndex,
    .subindex = 0x4E,
    .name = "Depth level of the 5th notch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &PositionLoop1Gain,
    &SpeedLoop1Gain,
    &SpeedLoop1IntegralTime,
    &TorqueRefFilterCutoffFrequency1,
    &PositionLoop2Gain,
    &SpeedLoop2Gain,
    &SpeedLoop2IntegralTime,
    &TorqueRefFilterCutoffFrequency2,
    &SpeedFeedbackFilter,
    &SpeedFeedbackLPFCutoffFreq,
    &SpeedFeedbackOverlapFilterTimeConstant,
    &SpeedFeedforwardSource,
    &SpeedFeedforwardPercentage,
    &SpeedFeedforwardFilterCutoffFreq,
    &TorqueFeedforwardSource,
    &TorqueFeedforwardPercentage,
    &TorqueFeedforwardFilterCutoffFreq,
    &PDFFControlCoefficient,
    &DampingFactorControlCoefficient,
    &PositionRefOverlapFilterTimeConstantA,
    &PositionRefOverlapFilterTimeConstantB,
    &PositionRefLPFTimeConstantA,
    &PositionRefLPFTimeConstantB,
    &PositionRefNotchFilterFrequency1,
    &PositionRefNotchFilterWidth1,
    &AdaptiveNotchTestTimes,
    &GainSwitchoverMode,
    &GainSwitchoverTime,
    &GainSwitchoverThreshold,
    &GainSwitchoverLoopWidth,
    &FrequencyOfFirstNotch,
    &WidthLevelOfFirstNotch,
    &DepthLevelOfFirstNotch,
    &FrequencyOfSecondNotch,
    &WidthLevelOfSecondNotch,
    &DepthLevelOfSecondNotch,
    &FrequencyOfThirdNotch,
    &WidthLevelOfThirdNotch,
    &DepthLevelOfThirdNotch,
    &FrequencyOfFourthNotch,
    &WidthLevelOfFourthNotch,
    &DepthLevelOfFourthNotch,
    &FrequencyOfFifthNotch,
    &WidthLevelOfFifthNotch,
    &DepthLevelOfFifthNotch,
};

} // namespace C01
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
