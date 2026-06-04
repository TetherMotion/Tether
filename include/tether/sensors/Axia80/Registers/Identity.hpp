/**
 * @file Identity.hpp
 * @brief Axia80 standard EtherCAT communication / identity objects (0x1000–0x1018)
 */

#pragma once

#include "tether/sensors/Axia80/Registers/Common.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {
namespace Identity {

// Object index for the identity group
static constexpr uint16_t IdentityObjectIndex = 0x1018;
static constexpr uint16_t DeviceTypeIndex = 0x1000;
static constexpr uint16_t DeviceNameIndex = 0x1008;
static constexpr uint16_t ErrorRegisterIndex = 0x1001;
static constexpr uint16_t ErrorSettingsIndex = 0x10F1;

// 0x1000: Device Type
constexpr RegisterEntry DeviceType = {
    .index = DeviceTypeIndex,
    .subindex = 0x00,
    .name = "Device type",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00000192,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Default value 0x192 (402 decimal). Identifies the device category.",
};

// 0x1001: Error Register (cross-referenced to 0x6010)
constexpr RegisterEntry ErrorRegister = {
    .index = ErrorRegisterIndex,
    .subindex = 0x00,
    .name = "Error register",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0x00,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Cross-reference to Object 0x6010 Status Code Map",
};

// 0x1008: Device Name
constexpr RegisterEntry DeviceName = {
    .index = DeviceNameIndex,
    .subindex = 0x00,
    .name = "Device name",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Default: ATI Axia F/T Sensor. Can be modified for branding.",
};

// 0x1018: Identity Object subindices
constexpr RegisterEntry IdentityVendorId = {
    .index = IdentityObjectIndex,
    .subindex = 0x01,
    .name = "Vendor ID",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00000732,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "ATI vendor ID assigned by ETG (1842 decimal)",
};

constexpr RegisterEntry IdentityProductCode = {
    .index = IdentityObjectIndex,
    .subindex = 0x02,
    .name = "Product code",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x26483053,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Assigned unique series marker representing 9105-ECAT-Axiax-x",
};

constexpr RegisterEntry IdentityRevisionNumber = {
    .index = IdentityObjectIndex,
    .subindex = 0x03,
    .name = "Revision number",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Variable tracking revision level",
};

constexpr RegisterEntry IdentitySerialNumber = {
    .index = IdentityObjectIndex,
    .subindex = 0x04,
    .name = "Serial number",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Hardware production tracking number",
};

// 0x10F1: Error Settings
constexpr RegisterEntry ErrorSettings = {
    .index = ErrorSettingsIndex,
    .subindex = 0x00,
    .name = "Error settings",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "EtherCAT error behaviour configuration",
};

inline const RegisterList kRegisterList = {
    &DeviceType,
    &ErrorRegister,
    &DeviceName,
    &IdentityVendorId,
    &IdentityProductCode,
    &IdentityRevisionNumber,
    &IdentitySerialNumber,
    &ErrorSettings,
};

} // namespace Identity
} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT
