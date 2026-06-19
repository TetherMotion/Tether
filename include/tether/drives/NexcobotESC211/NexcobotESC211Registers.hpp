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
#include "tether/drives/NexcobotESC211/Registers/ModularDevice.hpp"
#include "tether/drives/NexcobotESC211/Registers/UserSystem.hpp"
#include "tether/drives/NexcobotESC211/Registers/BulkData.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOERx.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOERxChannels.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOETx.hpp"

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
    &Registers::NexcobotESC211::ModularDevice::kRegisterList,
    &Registers::NexcobotESC211::UserSystem::kRegisterList,
    &Registers::NexcobotESC211::BulkData::kRegisterList,
    &Registers::NexcobotESC211::FSOERx::kRegisterList,
    &Registers::NexcobotESC211::FSOERxChannels::kRegisterList,
    &Registers::NexcobotESC211::FSOETx::kRegisterList,
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
    static constexpr uint16_t kTxPDOMapFSOEIndex        = 0x1A00;
    static constexpr uint16_t kTxPDOMapIndex            = 0x1A01;
    static constexpr uint16_t kTxPDORSAPInfoIndex       = 0x1A02;
    static constexpr uint16_t kTxPDORSAPDebugIndex      = 0x1A03;
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

    // Modular device profile (0xF000-0xF010)
    static constexpr uint16_t kModularDeviceProfileIndex = 0xF000;
    static constexpr uint16_t kModuleProfileListIndex    = 0xF010;

    // User / system (0xF100-0xF112)
    static constexpr uint16_t kUserControlIndex           = 0xF100;
    static constexpr uint16_t kSystemCurrentStateIndex    = 0xF101;
    static constexpr uint16_t kSystemErrorCodeIndex      = 0xF102;
    static constexpr uint16_t kSystemErrorMessageIndex    = 0xF103;
    static constexpr uint16_t kLastErrorCodeIndex        = 0xF104;
    static constexpr uint16_t kUserPasswordInputIndex    = 0xF105;
    static constexpr uint16_t kUserPasswordOutputIndex   = 0xF106;
    static constexpr uint16_t kESCDebugMsgIndex          = 0xF110;
    static constexpr uint16_t kSystemCurrentStateMPUBIndex = 0xF111;
    static constexpr uint16_t kSystemErrorCodeMPUBIndex  = 0xF112;

    // Bulk data (0xF200-0xF222)
    static constexpr uint16_t kTempFNIDataIndex     = 0xF200;
    static constexpr uint16_t kActiveFNIDataIndex   = 0xF201;
    static constexpr uint16_t kActiveFNIDataCRCIndex = 0xF202;
    static constexpr uint16_t kRSPDataInputIndex    = 0xF210;
    static constexpr uint16_t kRSPDataOutputIndex   = 0xF211;
    static constexpr uint16_t kRSPDataCRCIndex      = 0xF212;
    static constexpr uint16_t kSDDDataInputIndex    = 0xF220;
    static constexpr uint16_t kSDDDataOutputIndex   = 0xF221;
    static constexpr uint16_t kSDDDataCRCIndex      = 0xF222;

    // FSOE Rx (0x6000-0x6052)
    static constexpr uint16_t kFSOESafetyPDURxIndex = 0x6000;
    static constexpr uint16_t kInputCounterIndex    = 0x6010;
    static constexpr uint16_t kSAFEDIIndex          = 0x6020;
    static constexpr uint16_t kPowerStatusIndex    = 0x6030;
    static constexpr uint16_t kDOMonitorIndex       = 0x6040;
    static constexpr uint16_t kDOValueActualIndex   = 0x6050;
    static constexpr uint16_t kDIValueIndex         = 0x6051;
    static constexpr uint16_t kDOCommandIndex       = 0x6052;

    // FSOE Rx channels (0x6100-0x6171)
    static constexpr uint16_t kFSOEFrameFSoE0Index  = 0x6100;
    static constexpr uint16_t kFSOESafeDataFSoE0Index = 0x6101;

    // FSOE Tx (0x7000-0x7020)
    static constexpr uint16_t kFSOESafetyPDUTxIndex = 0x7000;
    static constexpr uint16_t kOutputCounterIndex   = 0x7010;
    static constexpr uint16_t kSAFE_DOIndex         = 0x7020;

    // FSOE Tx channels (0x7100-0x7171)
    static constexpr uint16_t kFSOEFrameTxFSoE0Index  = 0x7100;
    static constexpr uint16_t kFSOESafeDataTxFSoE0Index = 0x7101;

    // Access to all register lists
    static inline const auto& kAllRegisterLists = NexcobotESC211Registers::kAllRegisterLists;
};

using NexcobotESC211Device = NexcobotESC211;

} // namespace Drives
} // namespace EtherCAT
