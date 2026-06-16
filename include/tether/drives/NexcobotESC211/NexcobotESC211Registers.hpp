#pragma once

#include <cstdint>
#include <vector>

#include "tether/drives/NexcobotESC211/Registers/Common.hpp"
#include "tether/drives/NexcobotESC211/Registers/Identity.hpp"
#include "tether/drives/NexcobotESC211/Registers/PDOMapping.hpp"
#include "tether/drives/NexcobotESC211/Registers/SyncManager.hpp"
#include "tether/drives/NexcobotESC211/Registers/SafetyStatus.hpp"
#include "tether/drives/NexcobotESC211/Registers/RSAPMonitoring.hpp"
#include "tether/drives/NexcobotESC211/Registers/SafetyIO.hpp"

namespace EtherCAT {
namespace Drives {
namespace NexcobotESC211Registers {

using RegisterList = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterList;
using RegisterListOfLists = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterListOfLists;
using RegisterListPtr = ::EtherCAT::Drives::Registers::NexcobotESC211::RegisterListPtr;

inline const RegisterListOfLists kAllRegisterLists = {
    &Registers::NexcobotESC211::Identity::kRegisterList,
    &Registers::NexcobotESC211::PDOMapping::kRegisterList,
    &Registers::NexcobotESC211::SyncManager::kRegisterList,
    &Registers::NexcobotESC211::SafetyStatus::kRegisterList,
    &Registers::NexcobotESC211::RSAPMonitoring::kRegisterList,
    &Registers::NexcobotESC211::SafetyIO::kRegisterList,
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

    // Sync Manager / PDO assignment
    static constexpr uint16_t kRxPDOAssignmentIndex    = 0x1C12;
    static constexpr uint16_t kTxPDOAssignmentIndex    = 0x1C13;
    static constexpr uint16_t kSMOutputParamIndex       = 0x1C32;
    static constexpr uint16_t kSMInputParamIndex        = 0x1C33;

    // Safety status (0x4001-0x4016)
    static constexpr uint16_t kRSAPStatusIndex                  = 0x4001;
    static constexpr uint16_t kRSAPInformation1Index            = 0x4002;
    static constexpr uint16_t kRSAPFaultAndDiscrepancyIndex     = 0x4003;
    static constexpr uint16_t kSafetyInputDiscrepancyIndex        = 0x4004;
    static constexpr uint16_t kEmergencyStopStateIndex          = 0x4005;
    static constexpr uint16_t kProtectiveStopStateIndex         = 0x4006;
    static constexpr uint16_t kCollaborativeInputStateIndex     = 0x4007;
    static constexpr uint16_t kSafetyInputSummaryIndex           = 0x4008;
    static constexpr uint16_t kOutputDiscrepancyMonitorIndex    = 0x4009;
    static constexpr uint16_t kOutputStateMonitorIndex           = 0x400A;
    static constexpr uint16_t kSafetyFunctionDiscrepancyIndex     = 0x400B;
    static constexpr uint16_t kSafetyFunctionSummaryIndex        = 0x400C;
    static constexpr uint16_t kEndpointManualReducedSpeedIndex  = 0x400D;
    static constexpr uint16_t kSafetyTCPManualReducedSpeedIndex   = 0x400E;
    static constexpr uint16_t kSafetyTCPSpeedStateIndex         = 0x400F;
    static constexpr uint16_t kSafetyTCPForceStateIndex          = 0x4010;
    static constexpr uint16_t kCartesianPositionStateIndex      = 0x4011;
    static constexpr uint16_t kAxisPositionStateIndex             = 0x4012;
    static constexpr uint16_t kAxisSpeedStateIndex               = 0x4013;
    static constexpr uint16_t kAxisForceStateIndex               = 0x4014;
    static constexpr uint16_t kRSAPStateMirrorIndex              = 0x4015;
    static constexpr uint16_t kErrorCodeMirrorIndex              = 0x4016;

    // RSAP monitoring (0x4100-0x4108)
    static constexpr uint16_t kRSAPTCP1MonitoringVelocityIndex = 0x4100;
    static constexpr uint16_t kRSAPCalculateTCP1Index          = 0x4101;
    static constexpr uint16_t kRSAPCalculateTCPForceIndex    = 0x4108;

    // Safety I/O (0x4200-0x4202)
    static constexpr uint16_t kSafetyInputAIndex  = 0x4200;
    static constexpr uint16_t kSafetyInputBIndex  = 0x4201;
    static constexpr uint16_t kSafetyOutputIndex    = 0x4202;

    // Access to all register lists
    static inline const auto& kAllRegisterLists = NexcobotESC211Registers::kAllRegisterLists;
};

using NexcobotESC211Device = NexcobotESC211;

} // namespace Drives
} // namespace EtherCAT
