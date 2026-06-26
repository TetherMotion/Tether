// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {

/// Infer the byte size of an object-dictionary entry from its data type.
/// Returns 0 if the size cannot be inferred (e.g. OctetString, Domain).
inline uint8_t inferByteSize(ObjectDictionary::ObjectDictionaryDataType dt) noexcept {
    using D = ObjectDictionary::ObjectDictionaryDataType;
    switch (dt) {
        case D::Boolean:
        case D::Integer8:
        case D::Unsigned8:
            return 1;
        case D::Integer16:
        case D::Unsigned16:
            return 2;
        case D::Integer32:
        case D::Unsigned32:
        case D::Real32:
        case D::Integer24:
        case D::Unsigned24:
            return 4;
        case D::Integer64:
        case D::Unsigned64:
        case D::Real64:
            return 8;
        case D::Integer40:
        case D::Unsigned40:
            return 5;
        case D::Integer48:
        case D::Unsigned48:
            return 6;
        case D::Integer56:
        case D::Unsigned56:
            return 7;
        default:
            return 0;
    }
}

/// Encode a PDO mapping entry value as a uint32_t:
///   bits 31..16 = object index
///   bits 15..8  = subindex
///   bits 7..0   = bit length
inline uint32_t encodePDOMappingValue(const ObjectDictionary::ObjectDictionaryEntry* entry,
                                       uint8_t byte_size) noexcept {
    return (static_cast<uint32_t>(entry->index) << 16) |
           (static_cast<uint32_t>(entry->subindex) << 8) |
           static_cast<uint32_t>(byte_size * 8);
}

/// A single entry in a custom PDO mapping.
///
/// Constructed from a pointer to an ObjectDictionaryEntry (i.e. a register
/// definition).  The byte size is inferred from the entry's data_type unless
/// \p byte_size is explicitly provided (needed for OctetString etc.).
struct CustomPDOMappingEntry {
    const ObjectDictionary::ObjectDictionaryEntry* entry;
    uint8_t byte_size;  ///< 0 = infer from entry->data_type

    /// Implicit from a register pointer — infers size from data_type.
    CustomPDOMappingEntry(const ObjectDictionary::ObjectDictionaryEntry* e)
        : entry(e), byte_size(inferByteSize(e->data_type)) {}

    /// Explicit byte size override (for OctetString, Domain, etc.).
    CustomPDOMappingEntry(const ObjectDictionary::ObjectDictionaryEntry* e, uint8_t sz)
        : entry(e), byte_size(sz) {}

    /// Resolved byte size (inferred if byte_size was 0).
    uint8_t resolvedSize() const noexcept {
        return byte_size != 0 ? byte_size : inferByteSize(entry->data_type);
    }
};

/// Layout of a single field within a custom PDO after offsets are computed.
struct CustomPDOFieldLayout {
    const ObjectDictionary::ObjectDictionaryEntry* entry;
    uint16_t offset;  ///< Byte offset within the PDO buffer
    uint8_t size;      ///< Resolved byte size
};

} // namespace EtherCAT
