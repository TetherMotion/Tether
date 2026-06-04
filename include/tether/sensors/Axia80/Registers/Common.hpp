/**
 * @file Common.hpp
 * @brief Axia80 sensor register-group plumbing and type aliases
 *
 * Mirrors the AS715N Registers/Common.hpp pattern for sensor register
 * definitions using the canonical ObjectDictionaryEntry structure.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {

using RegisterEntry = ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry;
using RegisterEntryPtr = const RegisterEntry*;
using RegisterList = std::vector<RegisterEntryPtr>;
using RegisterListPtr = const RegisterList*;
using RegisterListOfLists = std::vector<RegisterListPtr>;

using OptionsType = ::EtherCAT::ObjectDictionary::OptionsType;
using Unit = ::EtherCAT::ObjectDictionary::Unit;
using ModificationMode = ::EtherCAT::ObjectDictionary::ModificationMode;
using EffectiveTime = ::EtherCAT::ObjectDictionary::EffectiveTime;
using ObjectDictionaryDataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

// Re-export unit constants for convenience
static constexpr Unit Unit_None           = ::EtherCAT::ObjectDictionary::Unit_None;
static constexpr Unit Unit_Dimensionless  = ::EtherCAT::ObjectDictionary::Unit_Dimensionless;
static constexpr Unit Unit_Length_Meter   = ::EtherCAT::ObjectDictionary::Unit_Length_Meter;
static constexpr Unit Unit_Voltage_Volt   = ::EtherCAT::ObjectDictionary::Unit_Voltage_Volt;
static constexpr Unit Unit_Temperature_C  = ::EtherCAT::ObjectDictionary::Unit_Temperature_C;
static constexpr Unit Unit_Force_N        = ::EtherCAT::ObjectDictionary::Unit_None;
static constexpr Unit Unit_Torque_Nm      = ::EtherCAT::ObjectDictionary::Unit_None;
static constexpr Unit Unit_Percent        = ::EtherCAT::ObjectDictionary::Unit_Percent;

} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT
