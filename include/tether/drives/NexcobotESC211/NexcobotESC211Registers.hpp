#pragma once

#include <cstdint>
#include <vector>

#include "tether/drives/NexcobotESC211/Registers/Common.hpp"
#include "tether/drives/NexcobotESC211/Registers/Identity.hpp"
#include "tether/drives/NexcobotESC211/Registers/PDOMapping.hpp"

namespace EtherCAT {
namespace Drives {
namespace NexcobotESC211Registers {

using RegisterList = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterList;
using RegisterListOfLists = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterListOfLists;
using RegisterListPtr = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterListPtr;

inline const RegisterListOfLists kAllRegisterLists = {
    &Registers::NexcobotESC211::Identity::kRegisterList,
    &Registers::NexcobotESC211::PDOMapping::kRegisterList,
};

} // namespace NexcobotESC211Registers

struct NexcobotESC211 {
    static constexpr uint32_t kVendorId    = 0x00000DCBu;
    static constexpr uint32_t kProductCode = 0x45534331u;

    // Object dictionary index aliases (generic CiA 301 objects are in CiA301:: namespace)
    static constexpr uint16_t kDeviceTypeIndex          = 0x1000;
    static constexpr uint16_t kErrorRegisterIndex       = 0x1001;
    static constexpr uint16_t kDeviceNameIndex          = 0x1008;
    static constexpr uint16_t kHardwareVersionIndex    = 0x1009;
    static constexpr uint16_t kSoftwareVersionIndex    = 0x100A;
    static constexpr uint16_t kIdentityObjectIndex     = 0x1018;
    static constexpr uint16_t kErrorSettingsIndex       = 0x10F1;
    static constexpr uint16_t kTimestampObjectIndex     = 0x10F8;

    // RxPDO mapping indices
    static constexpr uint16_t kRxPDOMapFSOEIndex        = 0x1600;
    static constexpr uint16_t kRxPDOMapIndex            = 0x1601;
    static constexpr uint16_t kRxPDOMapFSoE0Index       = 0x1610;
    static constexpr uint16_t kRxPDOMapFSoE7Index       = 0x1617;

    // TxPDO mapping indices
    static constexpr uint16_t kTxPDOMapIndex            = 0x1A01;
    static constexpr uint16_t kTxPDORSAPInfoIndex       = 0x1A02;
    static constexpr uint16_t kTxPDOMapFSoE0Index       = 0x1A10;
    static constexpr uint16_t kTxPDOMapFSoE7Index       = 0x1A17;

    // Access to all register lists
    static inline const auto& kAllRegisterLists = NexcobotESC211Registers::kAllRegisterLists;
};

using NexcobotESC211Device = NexcobotESC211;

} // namespace Drives
} // namespace EtherCAT
