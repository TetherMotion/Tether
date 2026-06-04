#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {

using RegisterEntry = ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry;
using RegisterEntryPtr = const RegisterEntry*;
using RegisterList = std::vector<RegisterEntryPtr>;
using RegisterListPtr = const RegisterList*;

using OptionsType = ::EtherCAT::ObjectDictionary::OptionsType;
using Unit = ::EtherCAT::ObjectDictionary::Unit;
using ModificationMode = ::EtherCAT::ObjectDictionary::ModificationMode;
using EffectiveTime = ::EtherCAT::ObjectDictionary::EffectiveTime;

static constexpr Unit Unit_None           = ::EtherCAT::ObjectDictionary::Unit_None;
static constexpr Unit Unit_Dimensionless  = ::EtherCAT::ObjectDictionary::Unit_Dimensionless;
static constexpr Unit Unit_Length_Meter   = ::EtherCAT::ObjectDictionary::Unit_Length_Meter;
static constexpr Unit Unit_Current_Ampere = ::EtherCAT::ObjectDictionary::Unit_Current_Ampere;
static constexpr Unit Unit_Voltage_Volt   = ::EtherCAT::ObjectDictionary::Unit_Voltage_Volt;
static constexpr Unit Unit_Frequency_Hertz= ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz;
static constexpr Unit Unit_Power_Watt     = ::EtherCAT::ObjectDictionary::Unit_Power_Watt;
static constexpr Unit Unit_Resistance_Ohm = ::EtherCAT::ObjectDictionary::Unit_Resistance_Ohm;
static constexpr Unit Unit_Temperature_C  = ::EtherCAT::ObjectDictionary::Unit_Temperature_C;
static constexpr Unit Unit_Percent        = ::EtherCAT::ObjectDictionary::Unit_Percent;
static constexpr Unit Unit_RPM            = ::EtherCAT::ObjectDictionary::Unit_RPM;
static constexpr Unit Unit_Inc            = ::EtherCAT::ObjectDictionary::Unit_Inc;

} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
