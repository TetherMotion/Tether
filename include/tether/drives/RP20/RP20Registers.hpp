#pragma once

#include "tether/drives/RP20/Registers/Common.hpp"
#include "tether/drives/RP20/Registers/DigitalInput.hpp"
#include "tether/drives/RP20/Registers/DigitalOutput.hpp"
#include "tether/drives/RP20/Registers/AnalogInput.hpp"
#include "tether/drives/RP20/Registers/AnalogOutput.hpp"
#include "tether/drives/RP20/Registers/RTDInput.hpp"
#include "tether/drives/RP20/Registers/ThermocoupleInput.hpp"
#include "tether/drives/RP20/Registers/RelayOutput.hpp"
#include "tether/drives/RP20/Registers/ModuleInfo.hpp"

namespace EtherCAT {
namespace Drives {
namespace RP20Registers {

using RegisterList = ::EtherCAT::Drives::Registers::RP20::RegisterList;
using RegisterListOfLists = ::EtherCAT::Drives::Registers::RP20::RegisterListOfLists;
using RegisterListPtr = ::EtherCAT::Drives::Registers::RP20::RegisterListPtr;

inline const RegisterListOfLists kAllRegisterLists = {
    &::EtherCAT::Drives::Registers::RP20::DI::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::DO::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::AI::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::AO::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::RD::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::TC::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::DR::kRegisterList,
    &::EtherCAT::Drives::Registers::RP20::ModuleInfo::kRegisterList,
};

} // namespace RP20Registers
} // namespace Drives
} // namespace EtherCAT
