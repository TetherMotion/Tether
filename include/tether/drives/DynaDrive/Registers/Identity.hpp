#pragma once

#include <cstdint>
#include "tether/drives/DynaDrive/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {

static constexpr uint16_t IdentityObjectIndex = 0x1018;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VendorID = {
    .index = IdentityObjectIndex,
    .subindex = 0x01,
    .name = "Vendor ID",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00414E59,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ProductCode = {
    .index = IdentityObjectIndex,
    .subindex = 0x02,
    .name = "Product Code",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x17010001,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &VendorID,
    &ProductCode,
};

} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
