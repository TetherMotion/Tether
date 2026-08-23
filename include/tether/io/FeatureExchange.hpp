/**
 * @file FeatureExchange.hpp
 * @brief Feature negotiation between server and client.
 *
 * During connection setup, client and server exchange a list of supported
 * features represented as name/type/value triples.  Each side can then
 * query which features the other supports.
 *
 * ## Standard feature names (documented)
 *
 * | Name                      | Type   | Description |
 * |---------------------------|--------|-------------|
 * | `protocol_version`        | U32    | Protocol version number |
 * | `max_stream_entries`      | U32    | Max entries in a single stream config |
 * | `supports_datalogging`    | Bool   | Server supports datalogging |
 * | `supports_thresholds`     | Bool   | Server supports threshold filtering |
 * | `supports_binary_structs` | Bool   | Server supports binary struct descriptions |
 * | `supports_snapshots`      | Bool   | Server supports snapshot requests |
 * | `supports_set_parameter`  | Bool   | Server supports parameter writes |
 * | `server_name`             | String | Human-readable server name |
 * | `server_version`          | String | Server software version |
 * | `connection_type`         | String | e.g. "ethercat", "simulation", "sensor" |
 * | `max_chunk_size`          | U32    | Maximum rows per StreamData message |
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace tether { namespace io {

/// A single feature descriptor: name + typed value.
struct Feature {
    std::string name;
    ValueType   type;
    std::vector<uint8_t> value;

    // --- Convenience constructors ---
    static Feature makeBool(const std::string& name, bool v) {
        Feature f;
        f.name = name;
        f.type = ValueType::Bool;
        f.value = { static_cast<uint8_t>(v ? 1 : 0) };
        return f;
    }
    static Feature makeU32(const std::string& name, uint32_t v) {
        Feature f;
        f.name = name;
        f.type = ValueType::U32;
        f.value.resize(4);
        for (unsigned i = 0; i < 4; ++i) f.value[i] = static_cast<uint8_t>(v >> (8 * i));
        return f;
    }
    static Feature makeString(const std::string& name, const std::string& v) {
        Feature f;
        f.name = name;
        f.type = ValueType::String;
        f.value.assign(v.begin(), v.end());
        return f;
    }

    // --- Value accessors ---
    bool getBool() const {
        return !value.empty() && value[0] != 0;
    }
    uint32_t getU32() const {
        if (value.size() < 4) return 0;
        return static_cast<uint32_t>(value[0]) |
               (static_cast<uint32_t>(value[1]) << 8) |
               (static_cast<uint32_t>(value[2]) << 16) |
               (static_cast<uint32_t>(value[3]) << 24);
    }
    std::string getString() const {
        return std::string(value.begin(), value.end());
    }
};

/// A collection of features exchanged between client and server.
struct FeatureSet {
    std::vector<Feature> features;

    /// Find a feature by name. Returns nullptr if not found.
    const Feature* find(const std::string& name) const {
        for (const auto& f : features) {
            if (f.name == name) return &f;
        }
        return nullptr;
    }

    /// Check if a boolean feature is present and true.
    bool supports(const std::string& name) const {
        const Feature* f = find(name);
        return f && f->getBool();
    }

    /// Encode the feature set into a buffer.
    void encode(BufWriter& w) const {
        w.putU32(static_cast<uint32_t>(features.size()));
        for (const auto& f : features) {
            w.putStr16(f.name.c_str(), f.name.size());
            w.putU8(static_cast<uint8_t>(f.type));
            w.putU32(static_cast<uint32_t>(f.value.size()));
            w.putBytes(f.value.data(), f.value.size());
        }
    }

    /// Decode a feature set from a buffer. Returns true on success.
    static bool decode(BufReader& r, FeatureSet& out) {
        uint32_t count = r.getU32();
        if (!r.ok() || count > 1024) return false;
        out.features.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            uint16_t nl = r.getU16();
            auto* nb = r.getBytes(nl);
            if (!r.ok()) return false;
            out.features[i].name.assign(reinterpret_cast<const char*>(nb), nl);
            out.features[i].type = static_cast<ValueType>(r.getU8());
            uint32_t vl = r.getU32();
            if (vl > r.remaining()) return false;
            auto* vb = r.getBytes(vl);
            if (!r.ok()) return false;
            out.features[i].value.assign(vb, vb + vl);
        }
        return r.ok();
    }
};

}} // namespace tether::io
