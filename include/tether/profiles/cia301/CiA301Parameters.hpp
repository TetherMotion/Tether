#pragma once

#include <cstdint>
#include "tether/ethercat/ObjectDictionary.hpp"

/**
 * @file CiA301Parameters.hpp
 * @brief Generic CiA 301 object dictionary entries.
 *
 * Register definitions for the standard CiA 301 / ETG.1000 objects that
 * appear on every EtherCAT slave (identity, Sync-Manager PDO assignment,
 * configured/detected module ident lists), in the same style as the
 * device-specific Registers tables and CiA402::Parameters60xx.
 *
 * Record objects expose a `Count` (sub 0) plus a single `Entry`
 * definition; access member subindices with the subindex-override
 * overloads of CoEManager::readEntry / Slave::sdoReadEntry etc.
 */

namespace CiA301 {
namespace Objects {

using ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry;
using ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;
using ::EtherCAT::ObjectDictionary::ModificationMode;
using ::EtherCAT::ObjectDictionary::EffectiveTime;
using ::EtherCAT::ObjectDictionary::Unit_None;

// ============================================================================
// 0x1018 Identity — record: Vendor ID, Product Code, Revision, Serial
// ============================================================================
namespace Obj1018 {

constexpr ObjectDictionaryEntry VendorId = {
    .index = 0x1018,
    .subindex = 0x01,
    .name = "Vendor ID",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ObjectDictionaryEntry ProductCode = {
    .index = 0x1018,
    .subindex = 0x02,
    .name = "Product code",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ObjectDictionaryEntry RevisionNumber = {
    .index = 0x1018,
    .subindex = 0x03,
    .name = "Revision number",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ObjectDictionaryEntry SerialNumber = {
    .index = 0x1018,
    .subindex = 0x04,
    .name = "Serial number",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1018

// ============================================================================
// 0x1001 Error register — U8 bitmask of pending error categories
// ============================================================================
namespace Obj1001 {

constexpr ObjectDictionaryEntry ErrorRegister = {
    .index = 0x1001,
    .subindex = 0x00,
    .name = "Error register",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1001

// ============================================================================
// 0x1003 Pre-defined error field — record: sub 0 = count, sub 1..N = U32
// ============================================================================
namespace Obj1003 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0x1003,
    .subindex = 0x00,
    .name = "Error field count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

/// Subindex template — read member i via readEntry(Error, i).
constexpr ObjectDictionaryEntry Error = {
    .index = 0x1003,
    .subindex = 0x01,
    .name = "Error field entry",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1003

// ============================================================================
// 0x1C12 / 0x1C13 Sync Manager PDO assignment — record: sub 0 = count,
// sub 1..N = assigned PDO indices (U16)
// ============================================================================
namespace Obj1C12 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0x1C12,
    .subindex = 0x00,
    .name = "SM2 PDO assignment count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

/// Subindex template — read member i via readEntry(Assignment, i).
constexpr ObjectDictionaryEntry Assignment = {
    .index = 0x1C12,
    .subindex = 0x01,
    .name = "SM2 PDO assignment",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1C12

namespace Obj1C13 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0x1C13,
    .subindex = 0x00,
    .name = "SM3 PDO assignment count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ObjectDictionaryEntry Assignment = {
    .index = 0x1C13,
    .subindex = 0x01,
    .name = "SM3 PDO assignment",
    .data_type = ObjectDictionaryDataType::Unsigned16,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1C13

// ============================================================================
// 0x1B00 TxPDO mapping — record: sub 0 = count, sub 1..N = mapping
// values (U32, packed index:subindex:bitlength)
// ============================================================================
namespace Obj1B00 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0x1B00,
    .subindex = 0x00,
    .name = "TxPDO 0x1B00 mapping count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

/// Subindex template — read member i via readEntry(Mapping, i).
constexpr ObjectDictionaryEntry Mapping = {
    .index = 0x1B00,
    .subindex = 0x01,
    .name = "TxPDO 0x1B00 mapping",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace Obj1B00

// ============================================================================
// 0xF030 / 0xF050 Module ident lists — record: sub 0 = count,
// sub 1..N = module ident numbers (U32)
// ============================================================================
namespace ObjF030 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0xF030,
    .subindex = 0x00,
    .name = "Configured module ident list count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

/// Subindex template — access member i via sdoReadEntry(Entry, i).
constexpr ObjectDictionaryEntry Entry = {
    .index = 0xF030,
    .subindex = 0x01,
    .name = "Configured module ident",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace ObjF030

namespace ObjF050 {

constexpr ObjectDictionaryEntry Count = {
    .index = 0xF050,
    .subindex = 0x00,
    .name = "Detected module ident list count",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr ObjectDictionaryEntry Entry = {
    .index = 0xF050,
    .subindex = 0x01,
    .name = "Detected module ident",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

} // namespace ObjF050

} // namespace Objects
} // namespace CiA301
