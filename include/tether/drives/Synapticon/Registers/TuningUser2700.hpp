/**
 * @file TuningUser2700.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2701-0x2705
 *
 * Tuning command/status, user MOSI/MISO, and setup wizard completed flag.
 *
 * Object dictionary entries extracted from SOMANET_CiA_402_v5.1.9.xml ESI.
 */

#pragma once

#include <cstdint>
#include "tether/drives/Synapticon/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace Synapticon {

using ODDataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

// ============================================================================
// 0x2701 Tuning command — UDINT, rw
// ============================================================================
namespace Obj2701 {

static constexpr uint16_t ObjectIndex = 0x2701;

constexpr RegisterEntry TuningCommand = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Tuning command",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &TuningCommand,
};

} // namespace Obj2701

// ============================================================================
// 0x2702 Tuning status — UDINT, rw
// ============================================================================
namespace Obj2702 {

static constexpr uint16_t ObjectIndex = 0x2702;

constexpr RegisterEntry TuningStatus = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Tuning status",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &TuningStatus,
};

} // namespace Obj2702

// ============================================================================
// 0x2703 User MOSI — UDINT, rw
// ============================================================================
namespace Obj2703 {

static constexpr uint16_t ObjectIndex = 0x2703;

constexpr RegisterEntry UserMOSI = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "User MOSI",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &UserMOSI,
};

} // namespace Obj2703

// ============================================================================
// 0x2704 User MISO — UDINT, rw
// ============================================================================
namespace Obj2704 {

static constexpr uint16_t ObjectIndex = 0x2704;

constexpr RegisterEntry UserMISO = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "User MISO",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &UserMISO,
};

} // namespace Obj2704

// ============================================================================
// 0x2705 Setup wizard completed — USINT, rw
// ============================================================================
namespace Obj2705 {

static constexpr uint16_t ObjectIndex = 0x2705;

constexpr RegisterEntry SetupWizardCompleted = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Setup wizard completed",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SetupWizardCompleted,
};

} // namespace Obj2705

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
