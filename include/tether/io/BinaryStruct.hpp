/**
 * @file BinaryStruct.hpp
 * @brief Definitions for composite binary struct parameters/signals.
 *
 * A "binary struct" is a parameter or signal whose value is a fixed-layout
 * concatenation of named, typed fields.  The StructDescriptor describes the
 * layout so clients can decode the raw bytes without out-of-band schema info.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace tether { namespace io {

/// Describes a single field inside a binary struct value.
struct StructField {
    std::string name;       ///< Human-readable field name
    ValueType   type;       ///< Field data type
    uint16_t    offset;     ///< Byte offset from the start of the struct
    uint16_t    size;       ///< Byte size (derived from type for fixed types)
    std::string unit;       ///< Optional unit (e.g. "mm", "rad/s")
};

/// Describes the full layout of a binary struct value.
struct StructDescriptor {
    uint64_t                entryId;    ///< Parameter or signal ID this struct belongs to
    std::string             name;       ///< Struct type name (e.g. "CiA402StatusPDO")
    uint32_t                totalSize;  ///< Total byte size of the struct
    std::vector<StructField> fields;

    /// Encode this descriptor into a DescribeStructResp message body (after the message type byte).
    void encode(BufWriter& w) const {
        w.putU64(entryId);
        w.putStr16(name.c_str(), name.size());
        w.putU32(totalSize);
        w.putU32(static_cast<uint32_t>(fields.size()));
        for (const auto& f : fields) {
            w.putStr16(f.name.c_str(), f.name.size());
            w.putU8(static_cast<uint8_t>(f.type));
            w.putU16(f.offset);
            w.putU16(f.size);
            w.putStr16(f.unit.c_str(), f.unit.size());
        }
    }

    /// Decode a DescribeStructResp body. Returns true on success.
    static bool decode(BufReader& r, StructDescriptor& out) {
        out.entryId = r.getU64();
        uint16_t nlen = r.getU16();
        auto* nb = r.getBytes(nlen);
        if (!r.ok()) return false;
        out.name.assign(reinterpret_cast<const char*>(nb), nlen);
        out.totalSize = r.getU32();
        uint32_t fc = r.getU32();
        if (!r.ok()) return false;
        out.fields.resize(fc);
        for (uint32_t i = 0; i < fc; ++i) {
            uint16_t fnl = r.getU16();
            auto* fnb = r.getBytes(fnl);
            if (!r.ok()) return false;
            out.fields[i].name.assign(reinterpret_cast<const char*>(fnb), fnl);
            out.fields[i].type = static_cast<ValueType>(r.getU8());
            out.fields[i].offset = r.getU16();
            out.fields[i].size = r.getU16();
            uint16_t ul = r.getU16();
            auto* ub = r.getBytes(ul);
            if (!r.ok()) return false;
            out.fields[i].unit.assign(reinterpret_cast<const char*>(ub), ul);
        }
        return r.ok();
    }
};

}} // namespace tether::io
