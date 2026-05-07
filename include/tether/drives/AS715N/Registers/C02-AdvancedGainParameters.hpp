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
namespace C02 {

static constexpr uint16_t C02ObjectIndex = 0x2002; // Group C02 (Advanced Gain Parameters)

// Option enums for C02
enum class ModelTrackingControlOptions : uint16_t {
    Disabled = 0,
    SingleMassModelTracking = 1,
};

// Register entries for C02
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModelTrackingControl = {
    .index = C02ObjectIndex,
    .subindex = 0x00,
    .name = "Model tracking control",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<ModelTrackingControlOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModelTrackingControlGain = {
    .index = C02ObjectIndex,
    .subindex = 0x01,
    .name = "Model tracking control gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 500,
    .min_value = 10,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModelTrackingInertiaCorrection = {
    .index = C02ObjectIndex,
    .subindex = 0x02,
    .name = "Model tracking inertia correction coefficient",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedObserverGain = {
    .index = C02ObjectIndex,
    .subindex = 0x30,
    .name = "Speed observer gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 40000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedObserverInertiaCorrection = {
    .index = C02ObjectIndex,
    .subindex = 0x31,
    .name = "Speed observer inertia correction",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedObserverSpeedFeedbackCutoff = {
    .index = C02ObjectIndex,
    .subindex = 0x32,
    .name = "Speed observer speed feedback cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz,
    .min_value = 0,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x38,
    .name = "Frequency for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InertiaCorrectionForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x39,
    .name = "Inertia correction for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LowPassFilterCorrectionForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x3A,
    .name = "Low pass filter correction for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 32000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HighPassFilterCorrectionForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x3B,
    .name = "Correction of high-pass filter 1 for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .min_value = -9999,
    .max_value = 9999,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfHighPassFilter2ForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x3C,
    .name = "Frequency of high-pass filter 2 for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz,
    .min_value = 0,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation1ForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x3D,
    .name = "Ratio of compensation 1 for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation2ForVibrationSuppression1 = {
    .index = C02ObjectIndex,
    .subindex = 0x3E,
    .name = "Ratio of compensation 2 for vibration suppression 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyForVibrationSuppression2 = {
    .index = C02ObjectIndex,
    .subindex = 0x40,
    .name = "Frequency for vibration suppression 2",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation1ForVibrationSuppression2 = {
    .index = C02ObjectIndex,
    .subindex = 0x45,
    .name = "Ratio of compensation 1 for vibration suppression 2",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation2ForVibrationSuppression2 = {
    .index = C02ObjectIndex,
    .subindex = 0x46,
    .name = "Ratio of compensation 2 for vibration suppression 2",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x48,
    .name = "Frequency for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InertiaCorrectionForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x49,
    .name = "Inertia correction for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 10,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LowPassFilterCorrectionForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x4A,
    .name = "Low-pass filter correction for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .min_value = -9999,
    .max_value = 9999,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HighPassFilterCorrectionForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x4B,
    .name = "Correction of high-pass filter 1 for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .min_value = -9999,
    .max_value = 9999,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrequencyOfHighPassFilter2ForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x4C,
    .name = "Frequency of high-pass filter 2 for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 20000,
    .unit = Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 50000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation1ForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x4D,
    .name = "Ratio of compensation 1 for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RatioOfCompensation2ForVibrationSuppression3 = {
    .index = C02ObjectIndex,
    .subindex = 0x4E,
    .name = "Ratio of compensation 2 for vibration suppression 3",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 20000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DisturbanceObserverGain = {
    .index = C02ObjectIndex,
    .subindex = 0x60,
    .name = "Disturbance observer gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 40000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DisturbanceObserverInertiaCorrection = {
    .index = C02ObjectIndex,
    .subindex = 0x61,
    .name = "Disturbance observer inertia correction coefficient",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent,
    .min_value = 1,
    .max_value = 10000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DisturbanceObserverLowPassCutoff = {
    .index = C02ObjectIndex,
    .subindex = 0x62,
    .name = "Disturbance observer low-pass cutoff frequency",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .min_value = 0,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DisturbanceObserverCompensationTorque = {
    .index = C02ObjectIndex,
    .subindex = 0x63,
    .name = "Disturbance observer compensation torque percentage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrictionCompensationSwitch = {
    .index = C02ObjectIndex,
    .subindex = 0x68,
    .name = "Friction compensation switch and relevant setting",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrictionCompensationSpeedThreshold = {
    .index = C02ObjectIndex,
    .subindex = 0x69,
    .name = "Friction compensation speed threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 20,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 5000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry StaticFrictionCompensation = {
    .index = C02ObjectIndex,
    .subindex = 0x6A,
    .name = "Static friction compensation",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ForwardFrictionCompensationOfCoulombFriction = {
    .index = C02ObjectIndex,
    .subindex = 0x6B,
    .name = "Forward friction compensation of coulomb friction",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ReverseFrictionCompensationOfCoulombFriction = {
    .index = C02ObjectIndex,
    .subindex = 0x6C,
    .name = "Reverse friction compensation of coulomb friction",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = -2000,
    .max_value = 0,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ViscousFrictionTorqueForRatedSpeed = {
    .index = C02ObjectIndex,
    .subindex = 0x6D,
    .name = "Viscous friction torque for rated speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrictionCompensationFilterTime = {
    .index = C02ObjectIndex,
    .subindex = 0x6E,
    .name = "Friction compensation filter time",
    .data_type = ::EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FrictionCompensationThresholdForZeroSpeed = {
    .index = C02ObjectIndex,
    .subindex = 0x6F,
    .name = "Friction compensation threshold for zero speed",
    .data_type = ::EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &ModelTrackingControl,
    &ModelTrackingControlGain,
    &ModelTrackingInertiaCorrection,
    &SpeedObserverGain,
    &SpeedObserverInertiaCorrection,
    &SpeedObserverSpeedFeedbackCutoff,
    &FrequencyForVibrationSuppression1,
    &InertiaCorrectionForVibrationSuppression1,
    &LowPassFilterCorrectionForVibrationSuppression1,
    &HighPassFilterCorrectionForVibrationSuppression1,
    &FrequencyOfHighPassFilter2ForVibrationSuppression1,
    &RatioOfCompensation1ForVibrationSuppression1,
    &RatioOfCompensation2ForVibrationSuppression1,
    &FrequencyForVibrationSuppression2,
    &RatioOfCompensation1ForVibrationSuppression2,
    &RatioOfCompensation2ForVibrationSuppression2,
    &FrequencyForVibrationSuppression3,
    &InertiaCorrectionForVibrationSuppression3,
    &LowPassFilterCorrectionForVibrationSuppression3,
    &HighPassFilterCorrectionForVibrationSuppression3,
    &FrequencyOfHighPassFilter2ForVibrationSuppression3,
    &RatioOfCompensation1ForVibrationSuppression3,
    &RatioOfCompensation2ForVibrationSuppression3,
    &DisturbanceObserverGain,
    &DisturbanceObserverInertiaCorrection,
    &DisturbanceObserverLowPassCutoff,
    &DisturbanceObserverCompensationTorque,
    &FrictionCompensationSwitch,
    &FrictionCompensationSpeedThreshold,
    &StaticFrictionCompensation,
    &ForwardFrictionCompensationOfCoulombFriction,
    &ReverseFrictionCompensationOfCoulombFriction,
    &ViscousFrictionTorqueForRatedSpeed,
};

} // namespace C02
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
