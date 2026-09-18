#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Configuration data types managed by the ESC211 safety subsystem.
/// FNI = Fieldbus Node Interface, RSP = Safety-Related Parameters,
/// SDD = Safety-Related Device Description.
enum class ConfigDataType : uint8_t {
    FNI,
    RSP,
    SDD,
};

/// Metadata for each config data type: SDO indices, section limits, and the
/// control command codes used to activate / save / load that type.
struct ConfigDataTypeInfo {
    ConfigDataType type;
    const char* name;
    uint16_t inputIndex;     // SDO index for writing (temp/input to device)
    uint16_t outputIndex;    // SDO index for reading (active/output from device)
    uint16_t crcIndex;       // SDO index for CRC register
    uint16_t maxSections;    // Maximum number of 256-byte sections
    EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode activateCmd;
    EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode saveFlashCmd;
    EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode loadFlashCmd;
};

namespace BulkData = EtherCAT::Drives::Registers::NexcobotESC211::BulkData;
namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;

/// Table of all config data types (FNI, RSP, SDD).
inline constexpr ConfigDataTypeInfo kConfigDataTypes[] = {
    {ConfigDataType::FNI, "fni",
     BulkData::TempFNIDataIndex, BulkData::ActiveFNIDataIndex, BulkData::ActiveFNIDataCRCIndex,
     8, UserSystem::ControlCommandCode::UpdateFNIFromObject,
     UserSystem::ControlCommandCode::SaveFNIToFlash,
     UserSystem::ControlCommandCode::LoadFNIFromFlash},
    {ConfigDataType::RSP, "rsp",
     BulkData::RSPDataInputIndex, BulkData::RSPDataOutputIndex, BulkData::RSPDataCRCIndex,
     48, UserSystem::ControlCommandCode::DownloadRSPAndSDDByObject,
     UserSystem::ControlCommandCode::SaveRSPAndSDDToFlash,
     UserSystem::ControlCommandCode::LoadRSPAndSDDFromFlash},
    {ConfigDataType::SDD, "sdd",
     BulkData::SDDDataInputIndex, BulkData::SDDDataOutputIndex, BulkData::SDDDataCRCIndex,
     25, UserSystem::ControlCommandCode::DownloadRSPAndSDDByObject,
     UserSystem::ControlCommandCode::SaveRSPAndSDDToFlash,
     UserSystem::ControlCommandCode::LoadRSPAndSDDFromFlash},
};

/// Look up the info struct for a config data type.
inline const ConfigDataTypeInfo& configDataTypeInfo(ConfigDataType type) {
    for (const auto& dt : kConfigDataTypes) {
        if (dt.type == type) return dt;
    }
    // FNI is the first entry — should never reach here.
    return kConfigDataTypes[0];
}

/// Look up the info struct by name string ("fni", "rsp", "sdd").
/// Returns nullptr if the name is not recognized.
inline const ConfigDataTypeInfo* configDataTypeByName(const char* name) {
    for (const auto& dt : kConfigDataTypes) {
        if (std::string_view(dt.name) == name) return &dt;
    }
    return nullptr;
}

/// All config data types as a span (for iteration).
inline std::span<const ConfigDataTypeInfo> allConfigDataTypes() {
    return kConfigDataTypes;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
