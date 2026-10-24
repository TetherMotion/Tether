/**
 * @file OdSdoParam.hpp
 * @brief Shared helpers for exposers that surface object-dictionary entries
 *        as IO parameters reachable via SDO.
 *
 * Used by AS715NExposer (index-addressed `*.sdo.<INDEX>_<SUB>` naming) and
 * NamedSdoExposer (slugified `<prefix>.sdo.<name>` aliases).  Keeping the
 * OD-type mapping, metadata and SDO read/write plumbing here guarantees both
 * exposers present identical types, metadata and writability for the same
 * object.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/io/Registry.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace tether { namespace io { namespace exposers { namespace detail {

/// Map an object-dictionary data type to an IO value type.
/// Returns false for types with no meaningful scalar representation.
inline bool odToValueType(
    ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType t,
    ValueType& out) {
    using T = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;
    switch (t) {
        case T::Boolean:       out = ValueType::Bool;   return true;
        case T::Integer8:      out = ValueType::I8;     return true;
        case T::Integer16:     out = ValueType::I16;    return true;
        case T::Integer32:     out = ValueType::I32;    return true;
        case T::Integer64:     out = ValueType::I64;    return true;
        case T::Unsigned8:     out = ValueType::U8;     return true;
        case T::Unsigned16:    out = ValueType::U16;    return true;
        case T::Unsigned32:    out = ValueType::U32;    return true;
        case T::Unsigned64:    out = ValueType::U64;    return true;
        case T::Real32:        out = ValueType::F32;    return true;
        case T::Real64:        out = ValueType::F64;    return true;
        case T::VisibleString:
        case T::UnicodeString: out = ValueType::String; return true;
        default:               out = ValueType::Binary; return true;
    }
}

/// Fixed wire size for scalar OD types; 0 for variable-length / odd types.
inline uint8_t odFixedSize(
    ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType t) {
    using T = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;
    switch (t) {
        case T::Boolean:
        case T::Integer8:
        case T::Unsigned8:    return 1;
        case T::Integer16:
        case T::Unsigned16:   return 2;
        case T::Integer24:
        case T::Unsigned24:   return 3;
        case T::Integer32:
        case T::Unsigned32:
        case T::Real32:       return 4;
        case T::Integer40:
        case T::Unsigned40:   return 5;
        case T::Integer48:
        case T::Unsigned48:   return 6;
        case T::Integer56:
        case T::Unsigned56:   return 7;
        case T::Integer64:
        case T::Unsigned64:
        case T::Real64:
        case T::TimeOfDay:
        case T::TimeDifference: return 8;
        default:              return 0;
    }
}

inline const char* modificationModeName(
    ::EtherCAT::ObjectDictionary::ModificationMode m) {
    using M = ::EtherCAT::ObjectDictionary::ModificationMode;
    switch (m) {
        case M::ReadOnly:        return "readonly";
        case M::AtStop:          return "at_stop";
        case M::DuringOperation: return "during_operation";
        default:                 return "unknown";
    }
}

inline const char* effectiveTimeName(
    ::EtherCAT::ObjectDictionary::EffectiveTime e) {
    using E = ::EtherCAT::ObjectDictionary::EffectiveTime;
    switch (e) {
        case E::Immediately:    return "immediately";
        case E::UponRepowerOn:  return "upon_repower_on";
        default:                return "unknown";
    }
}

/// SDO transaction timeout for IO-triggered reads/writes (session thread).
inline constexpr uint32_t kSdoTimeoutMs = 500;

inline std::string odHexU16(uint16_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "0x%04X", v);
    return b;
}
inline std::string odHexU8(uint8_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "0x%02X", v);
    return b;
}

/**
 * @brief Turn a human-readable object-dictionary name into a
 *        protocol-friendly slug.
 *
 * ASCII letters are lowercased, digits are kept, every run of other
 * characters folds into a single '_', leading/trailing underscores are
 * trimmed: "1st speed loop gain" -> "1st_speed_loop_gain",
 * "Identity: vendor ID" -> "identity_vendor_id".  ASCII-only by design so
 * results are locale-independent.  May return an empty string.
 */
inline std::string slugifyName(std::string_view name) {
    std::string out;
    out.reserve(name.size());
    for (const char ch : name) {
        const unsigned char c = static_cast<unsigned char>(ch);
        const bool digit = c >= '0' && c <= '9';
        const bool upper = c >= 'A' && c <= 'Z';
        const bool lower = c >= 'a' && c <= 'z';
        if (digit || lower) {
            out += static_cast<char>(c);
        } else if (upper) {
            out += static_cast<char>(c - 'A' + 'a');
        } else if (!out.empty() && out.back() != '_') {
            out += '_';
        }
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out;
}

/**
 * @brief Build a ParamEntry that reads/writes one SDO-addressable
 *        object-dictionary entry through a queue-serialized CoEManager.
 *
 * The returned entry carries the full OD metadata set (index, subindex,
 * od_type, default, min, max, unit, modification, effective, options_enum)
 * and EntryFlags::NoStream — an SDO mailbox round-trip per stream row would
 * saturate the mailbox, so consumers must poll via GetParam.
 *
 * Fixed-size objects get typed readFn/writeFn callbacks; variable-length
 * objects (strings, octet strings, domains) get varReadFn/varWriteFn with a
 * 256-byte cap.  Read-only objects (modification_mode == ReadOnly) get no
 * write callback.  The callbacks reference `coe` — it must outlive the
 * registered entry.
 *
 * @param coe    Queue-serialized CoE manager for the owning slave.
 * @param e      Object-dictionary entry to expose.
 * @param id     Fully-computed parameter ID (module base | local id).
 * @param name   Parameter name (decided by the caller's naming scheme).
 * @param group  Logical group string (e.g. "as715n.drive0.sdo").
 */
inline ParamEntry makeSdoParam(
    ::EtherCAT::CoE::CoEManager& coe,
    const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry& e,
    uint64_t id, std::string name, std::string group) {

    ParamEntry p;
    p.id          = id;
    p.name        = std::move(name);
    p.description = e.name ? e.name : "";
    if (e.comment && *e.comment) {
        p.description += " — ";
        p.description += e.comment;
    }
    p.group      = std::move(group);
    p.extraFlags = EntryFlags::NoStream;
    p.metadata   = {
        {"index",        odHexU16(e.index)},
        {"subindex",     odHexU8(e.subindex)},
        {"od_type",      odHexU16(static_cast<uint16_t>(e.data_type))},
        {"default",      std::to_string(e.default_value)},
        {"min",          std::to_string(e.min_value)},
        {"max",          std::to_string(e.max_value)},
        {"unit",         std::to_string(static_cast<uint32_t>(e.unit))},
        {"modification", modificationModeName(e.modification_mode)},
        {"effective",    effectiveTimeName(e.effective_time)},
    };
    if (e.options_enum.has_value() && e.options_enum.name())
        p.metadata["options_enum"] = e.options_enum.name();

    const uint16_t index = e.index;
    const uint8_t  sub   = e.subindex;
    const bool writable =
        e.modification_mode !=
        ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly;
    const uint8_t fixedSize = odFixedSize(e.data_type);

    if (fixedSize > 0) {
        // Fixed-size scalar (including odd-width ints as raw bytes).
        odToValueType(e.data_type, p.valueType);
        p.readFn = [&coe, index, sub, fixedSize](void* d) {
            uint8_t buf[8] = {};
            coe.readSync(index, sub, buf, fixedSize,
                         kSdoTimeoutMs, nullptr);
            std::memcpy(d, buf, fixedSize);
        };
        if (writable) {
            p.writeFn = [&coe, index, sub, fixedSize](const void* src) {
                coe.writeSync(index, sub, src, fixedSize);
            };
        }
    } else {
        // Variable-length (strings, octet strings, domains).
        p.valueType    = ValueType::Binary;
        p.maxValueSize = 256;
        p.varReadFn = [&coe, index, sub](void* d, size_t maxLen) -> size_t {
            size_t actual = 0;
            coe.readSync(index, sub, d, maxLen, kSdoTimeoutMs, &actual);
            return actual;
        };
        if (writable) {
            p.varWriteFn = [&coe, index, sub](const void* src, size_t len) {
                coe.writeSync(index, sub, src, len);
            };
        }
    }
    return p;
}

}}}} // namespace tether::io::exposers::detail
