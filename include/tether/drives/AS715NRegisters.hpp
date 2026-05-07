#pragma once

#include <cstdint>
#include <vector>

// Register groups extracted from vendor PDF (generated)
#include "tether/drives/AS715N/Registers/C00-Parameters.hpp"
#include "tether/drives/AS715N/Registers/C01-BasicGainParameters.hpp"
#include "tether/drives/AS715N/Registers/C02-AdvancedGainParameters.hpp"
#include "tether/drives/AS715N/Registers/C03-InstructionParameters.hpp"
#include "tether/drives/AS715N/Registers/C04-IOParameters.hpp"
#include "tether/drives/AS715N/Registers/C05-StopMode.hpp"
#include "tether/drives/AS715N/Registers/C06-ProtectionParameters.hpp"
#include "tether/drives/AS715N/Registers/C07-AutoTuningParameters.hpp"
#include "tether/drives/AS715N/Registers/C0A-CommunicationParameters.hpp"
#include "tether/drives/AS715N/Registers/C10-RotationHomingParameters.hpp"
#include "tether/drives/AS715N/Registers/C13-EtherCATParameters.hpp"
#include "tether/drives/AS715N/Registers/F30-ControlInProgress.hpp"
#include "tether/drives/AS715N/Registers/F31-ControlInProgress.hpp"
#include "tether/drives/AS715N/Registers/R20-MotorParameters.hpp"
#include "tether/drives/AS715N/Registers/R22-Motor-Gain-Parameters.hpp"
#include "tether/drives/AS715N/Registers/U40-RunningMonitoringParameters.hpp"
#include "tether/drives/AS715N/Registers/U41-StatusMonitoringParameters.hpp"
#include "tether/drives/AS715N/Registers/U42-VersionParameters.hpp"
// #include "tether/drives/AS715N/Registers/C99-Example.hpp" // Likely example, skipping or including if relevant

namespace EtherCAT {
namespace Drives {
namespace AS715NRegisters {

using RegisterList = ::EtherCAT::Drives::Registers::RegisterList;
using RegisterListOfLists = ::EtherCAT::Drives::Registers::RegisterListOfLists;

using RegisterListPtr = ::EtherCAT::Drives::Registers::RegisterListPtr;

inline const RegisterListOfLists kAllRegisterLists = {
    &Registers::AS715N::C00::kRegisterList,
    &Registers::AS715N::C01::kRegisterList,
    &Registers::AS715N::C02::kRegisterList,
    &Registers::AS715N::C03::kRegisterList,
    &Registers::AS715N::C04::kRegisterList,
    &Registers::AS715N::C05::kRegisterList,
    &Registers::AS715N::C06::kRegisterList,
    &Registers::AS715N::C07::kRegisterList,
    &Registers::AS715N::C0A::kRegisterList,
    &Registers::AS715N::C10::kRegisterList,
    &Registers::AS715N::C13::kRegisterList,
    &Registers::AS715N::F30::kRegisterList,
    &Registers::AS715N::F31::kRegisterList,
    &Registers::AS715N::R20::kRegisterList,
    &Registers::AS715N::R22::kRegisterList,
    &Registers::AS715N::U40::kRegisterList,
    &Registers::AS715N::U41::kRegisterList,
    &Registers::AS715N::U42::kRegisterList,
};

} // namespace AS715NRegisters

struct AS715N {
    static constexpr uint32_t kVendorId = 0x00400000;     // ANCTL
    static constexpr uint32_t kProductCode = 0x00000715;  // AS715N

    // Error registers
    static constexpr uint16_t kManufacturerFaultIndex = 0x203F;  // UInt32: internal(high16) + external(low16)
    static constexpr uint16_t kCiA402ErrorIndex = 0x603F;        // UInt16

    // Running monitoring (U40)
    static constexpr uint16_t kRunningMonitoringIndex = 0x2040;
    static constexpr uint8_t kU40_PhaseCurrentRms_SubIndex = 0x0C;
    static constexpr uint8_t kU40_PositionDeviation_SubIndex = 0x10;
    static constexpr uint8_t kU40_HeatsinkTemperature_SubIndex = 0x30;

    // Forced DO (referenced in PDO mapping)
    static constexpr uint16_t kForcedPhysicalDOIndex = 0x60FE;
    static constexpr uint8_t kForcedPhysicalDO_SubIndex = 0x01;

    // Control-in-progress group (F31) and common subindexes used by examples/code
    static constexpr uint16_t kControlInProgressIndex = 0x2031; // F31 group
    static constexpr uint8_t  kFaultResetSubIndex      = 0x01;   // Fault reset subindex

    // Access to all registers
    static inline const auto& kAllRegisterLists = AS715NRegisters::kAllRegisterLists;
};


// Backwards-compatible type alias used throughout the codebase
using AS715NDevice = AS715N;

struct AS715NManufacturerFault203F {
    uint16_t internal_code = 0;
    uint16_t external_code = 0;

    static AS715NManufacturerFault203F fromU32(uint32_t raw) {
        AS715NManufacturerFault203F out;
        out.internal_code = static_cast<uint16_t>((raw >> 16) & 0xFFFFu);
        out.external_code = static_cast<uint16_t>(raw & 0xFFFFu);
        return out;
    }
};

} // namespace Drives
} // namespace EtherCAT
