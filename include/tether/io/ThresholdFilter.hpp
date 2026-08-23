/**
 * @file ThresholdFilter.hpp
 * @brief Threshold-based change detection and skip filtering for streams.
 *
 * Allows configuring per-entry thresholds that determine when a value change
 * is significant enough to include in a stream.  Supports absolute, relative,
 * and custom (named) threshold logic.  Multiple threshold configurations can
 * coexist for whitelist/blacklist selection of entries.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace tether { namespace io {

/// A single name=string / type / value primitive for custom threshold config.
struct ConfigPrimitive {
    std::string name;
    ValueType   type;
    std::vector<uint8_t> value;     ///< Raw bytes (valueTypeSize bytes for fixed types)

    /// Helper to set a float value.
    void setFloat(float v) {
        type = ValueType::F32;
        value.resize(4);
        std::memcpy(value.data(), &v, 4);
    }

    /// Helper to get a float value.
    float getFloat() const {
        if (type != ValueType::F32 || value.size() < 4) return 0.0f;
        float v;
        std::memcpy(&v, value.data(), 4);
        return v;
    }

    /// Helper to set a string value.
    void setString(const std::string& s) {
        type = ValueType::String;
        value.assign(s.begin(), s.end());
    }

    /// Helper to get a string value.
    std::string getString() const {
        if (type != ValueType::String) return {};
        return std::string(value.begin(), value.end());
    }
};

/// Configuration for a single threshold rule.
struct ThresholdRule {
    uint64_t        entryId;        ///< 0 = default rule for all entries
    ThresholdType   type;
    double          threshold;      ///< For Absolute/Relative types
    std::string     customName;     ///< For Custom type: logic name
    std::vector<ConfigPrimitive> customConfig;  ///< For Custom type

    /// Encode this rule into a buffer.
    void encode(BufWriter& w) const {
        w.putU64(entryId);
        w.putU8(static_cast<uint8_t>(type));
        w.putF64(threshold);
        w.putStr16(customName.c_str(), customName.size());
        w.putU32(static_cast<uint32_t>(customConfig.size()));
        for (const auto& p : customConfig) {
            w.putStr16(p.name.c_str(), p.name.size());
            w.putU8(static_cast<uint8_t>(p.type));
            w.putU32(static_cast<uint32_t>(p.value.size()));
            w.putBytes(p.value.data(), p.value.size());
        }
    }

    /// Decode a rule from a buffer. Returns true on success.
    static bool decode(BufReader& r, ThresholdRule& out) {
        out.entryId = r.getU64();
        out.type = static_cast<ThresholdType>(r.getU8());
        out.threshold = r.getF64();
        uint16_t nl = r.getU16();
        auto* nb = r.getBytes(nl);
        if (!r.ok()) return false;
        out.customName.assign(reinterpret_cast<const char*>(nb), nl);
        uint32_t cc = r.getU32();
        if (!r.ok() || cc > MAX_COLLECTION_COUNT) return false;
        out.customConfig.resize(cc);
        for (uint32_t i = 0; i < cc; ++i) {
            uint16_t pnl = r.getU16();
            auto* pnb = r.getBytes(pnl);
            if (!r.ok()) return false;
            out.customConfig[i].name.assign(reinterpret_cast<const char*>(pnb), pnl);
            out.customConfig[i].type = static_cast<ValueType>(r.getU8());
            uint32_t vl = r.getU32();
            if (!r.ok() || vl > MAX_VARIABLE_VALUE_SIZE || vl > r.remaining()) return false;
            auto* vb = r.getBytes(vl);
            if (!r.ok()) return false;
            out.customConfig[i].value.assign(vb, vb + vl);
        }
        return r.ok();
    }
};

/// A named set of threshold rules for filtering stream data.
struct ThresholdConfig {
    std::string name;               ///< Config name (e.g. "fast_mode", "precision")
    std::vector<ThresholdRule> rules;
    bool isWhitelist = true;        ///< true = only entries matching rules are included
                                    ///< false = entries matching rules are excluded

    /// Encode the full config.
    void encode(BufWriter& w) const {
        w.putStr16(name.c_str(), name.size());
        w.putU8(isWhitelist ? 1 : 0);
        w.putU32(static_cast<uint32_t>(rules.size()));
        for (const auto& r : rules) {
            r.encode(w);
        }
    }

    /// Decode a config. Returns true on success.
    static bool decode(BufReader& r, ThresholdConfig& out) {
        uint16_t nl = r.getU16();
        auto* nb = r.getBytes(nl);
        if (!r.ok()) return false;
        out.name.assign(reinterpret_cast<const char*>(nb), nl);
        out.isWhitelist = (r.getU8() != 0);
        uint32_t rc = r.getU32();
        if (!r.ok() || rc > MAX_COLLECTION_COUNT) return false;
        out.rules.resize(rc);
        for (uint32_t i = 0; i < rc; ++i) {
            if (!ThresholdRule::decode(r, out.rules[i])) return false;
        }
        return r.ok();
    }
};

/// Custom threshold evaluation callback type.
/// Receives: entry ID, old value ptr, new value ptr, value size, config primitives.
/// Returns true if the change should be reported.
using CustomThresholdFn = std::function<bool(
    uint64_t entryId, const void* oldVal, const void* newVal,
    size_t valSize, const std::vector<ConfigPrimitive>& config)>;

/**
 * @class ThresholdFilter
 * @brief Evaluates whether value changes exceed configured thresholds.
 *
 * Maintains per-entry state (last-sent value) and applies the configured
 * threshold rules to determine if a new sample should be included in the
 * stream output.
 */
class ThresholdFilter {
public:
    /// Set the active threshold configuration.
    void setConfig(const ThresholdConfig& config) { config_ = config; }

    /// Get the active threshold configuration.
    const ThresholdConfig& config() const { return config_; }

    /// Register a custom threshold evaluator by name.
    void registerCustom(const std::string& name, CustomThresholdFn fn) {
        customEvaluators_[name] = std::move(fn);
    }

    /// Check if a value change passes the threshold for the given entry.
    /// @param entryId  The parameter or signal ID.
    /// @param oldVal   Pointer to the previous value bytes.
    /// @param newVal   Pointer to the new value bytes.
    /// @param valSize  Byte size of the value.
    /// @return true if the new value should be transmitted.
    bool passes(uint64_t entryId, const void* oldVal, const void* newVal, size_t valSize) const;

    /// Reset all last-sent value state.
    void reset();

private:
    const ThresholdRule* findRule(uint64_t entryId) const;

    ThresholdConfig config_;
    std::map<std::string, CustomThresholdFn> customEvaluators_;
};

}} // namespace tether::io
