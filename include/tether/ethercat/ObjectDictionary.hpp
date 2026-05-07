// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <cstddef>
#include <typeinfo>
#include <type_traits>

#if __cplusplus < 202002L
namespace std { template<class T> struct type_identity { using type = T; }; }
#endif

/**
 * @file ObjectDictionary.hpp
 * @brief Central place for CANopen/EtherCAT object-dictionary helpers and types
 *
 * This header defines the canonical type used to describe object-dictionary
 * data types (originally `ODDataType`).  The enum is exposed as
 * `EtherCAT::ObjectDictionary::ObjectDictionaryDataType` and a backward
 * compatibility alias is provided in `EtherCAT::slave` when the old header
 * is included.
 */

namespace EtherCAT {
namespace ObjectDictionary {

/**
 * @brief Object dictionary data types (CANopen)
 *
 * These values match the CiA 301 data type table and were previously named
 * `ODDataType` in `IMailboxHandler.hpp`.
 */
enum class ObjectDictionaryDataType : uint16_t {
    Boolean       = 0x0001,
    Integer8      = 0x0002,
    Integer16     = 0x0003,
    Integer32     = 0x0004,
    Unsigned8     = 0x0005,
    Unsigned16    = 0x0006,
    Unsigned32    = 0x0007,
    Real32        = 0x0008,
    VisibleString = 0x0009,
    OctetString   = 0x000A,
    UnicodeString = 0x000B,
    TimeOfDay     = 0x000C,
    TimeDifference= 0x000D,
    Domain        = 0x000F,
    Integer24     = 0x0010,
    Real64        = 0x0011,
    Integer40     = 0x0012,
    Integer48     = 0x0013,
    Integer56     = 0x0014,
    Integer64     = 0x0015,
    Unsigned24    = 0x0016,
    Unsigned40    = 0x0018,
    Unsigned48    = 0x0019,
    Unsigned56    = 0x001A,
    Unsigned64    = 0x001B,
};

// Types that describe a single object-dictionary entry (previously in
// `tether/drives/AS715N/Registers/Common.hpp`). These belong next to the
// canonical object-dictionary data-type enum so they are usable across the
// codebase.


enum class ModificationMode : uint8_t {
    Unknown = 0,
    ReadOnly,
    AtStop,
    DuringOperation,
};

enum class EffectiveTime : uint8_t {
    Unknown = 0,
    Immediately,
    UponRepowerOn,
};

// Human-friendly unit identifiers for register definitions. Keep as an
// unscoped `enum` so existing numeric initializers (and consumer code) don't
// need changes.
enum Unit : uint32_t {
    Unit_None = 0,
    Unit_Dimensionless    = 0x00,
    Unit_Length_Meter     = 0x01,
    Unit_Current_Ampere   = 0x04,
    Unit_Voltage_Volt     = 0x06,
    Unit_Frequency_Hertz  = 0x10,
    Unit_Power_Watt       = 0x11,
    Unit_Resistance_Ohm   = 0x12,
    Unit_Temperature_C    = 0x2D,
    Unit_Percent          = 0x1000,
    Unit_RPM              = 0x1001,
    Unit_Inc              = 0x1002,
};

// Lightweight wrapper that stores an optional `type_info` for an enum that
// represents choices for a register (used for SoE/CoE option lists).
struct OptionsType {
    const std::type_info* info;
    constexpr OptionsType(std::nullptr_t = nullptr) noexcept : info(nullptr) {}
    template<typename T>
    constexpr OptionsType(std::type_identity<T>) noexcept : info(&typeid(T)) {}
    constexpr bool has_value() const noexcept { return info != nullptr; }
    constexpr const char* name() const noexcept { return info ? info->name() : nullptr; }
};

// Canonical object-dictionary entry description used by device register
// definitions and other subsystems.
struct ObjectDictionaryEntry {
    uint16_t index;
    uint8_t subindex;
    const char* name;
    ObjectDictionaryDataType data_type;
    uint32_t default_value;
    Unit unit = Unit_None;
    OptionsType options_enum = nullptr;
    int64_t min_value;
    int64_t max_value;
    ModificationMode modification_mode;
    EffectiveTime effective_time;
    const char* comment = nullptr;
};

} // namespace ObjectDictionary
} // namespace EtherCAT
