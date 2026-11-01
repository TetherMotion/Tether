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

/**A single name=string / type / value primitive for custom threshold config. */
struct ConfigPrimitive {
    std::string name;
    ValueType   type;
    std::vector<uint8_t> value; /**< Raw bytes (valueTypeSize bytes for fixed types) */

    /**
     * @brief Store a 32-bit float into this primitive.
     *
     * Sets `type` to ValueType::F32 and packs @p v little-endian into
     * `value` (exactly 4 bytes).
     *
     * @param v  Float value to store.
     */
    void setFloat(float v) {
        type = ValueType::F32;
        value.resize(4);
        std::memcpy(value.data(), &v, 4);
    }

    /**
     * @brief Read this primitive back as a float.
     *
     * @return The stored float when `type` is ValueType::F32 and at least
     *         4 bytes are present; 0.0f for any other type or truncated
     *         payload.
     */
    float getFloat() const {
        if (type != ValueType::F32 || value.size() < 4) return 0.0f;
        float v;
        std::memcpy(&v, value.data(), 4);
        return v;
    }

    /**
     * @brief Store a UTF-8 string into this primitive.
     *
     * Sets `type` to ValueType::String and copies @p s byte-for-byte into
     * `value` (no trailing NUL is stored).
     *
     * @param s  String content to store.
     */
    void setString(const std::string& s) {
        type = ValueType::String;
        value.assign(s.begin(), s.end());
    }

    /**
     * @brief Read this primitive back as a string.
     *
     * @return The stored bytes as a std::string when `type` is
     *         ValueType::String; an empty string for any other type —
     *         indistinguishable from a stored empty string.
     */
    std::string getString() const {
        if (type != ValueType::String) return {};
        return std::string(value.begin(), value.end());
    }
};

/**Configuration for a single threshold rule. */
struct ThresholdRule {
    uint64_t        entryId; /**< 0 = default rule for all entries */
    ThresholdType   type;
    double          threshold; /**< For Absolute/Relative types */
    std::string     customName; /**< For Custom type: logic name */
    std::vector<ConfigPrimitive> customConfig; /**< For Custom type */

    /**
     * @brief Encode this rule into a buffer.
     *
     * Wire layout: u64 entryId, u8 type, f64 threshold, str16 customName,
     * u32 customConfig count, then per primitive: str16 name, u8 type,
     * u32 value size, raw value bytes. decode() reads the same layout.
     *
     * @param w  Writer to append the encoded bytes to.
     */
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

    /**
     * @brief Decode a rule from a buffer.
     *
     * Reads the layout produced by encode(). Length and count fields are
     * validated against the remaining buffer and the protocol limits
     * (MAX_COLLECTION_COUNT, MAX_VARIABLE_VALUE_SIZE) — a malformed or
     * truncated buffer leaves the reader in its failed state.
     *
     * @param r    Reader positioned at an encoded ThresholdRule.
     * @param out  Rule to overwrite with the decoded contents.
     * @return true on success; false if the buffer is truncated,
     *         inconsistent, or exceeds a protocol bound — in which case
     *         @p out may be partially written.
     */
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

/**A named set of threshold rules for filtering stream data. */
struct ThresholdConfig {
    std::string name; /**< Config name (e.g. "fast_mode", "precision") */
    std::vector<ThresholdRule> rules;
    bool isWhitelist = true; /**< true = only entries matching rules are included */
                                    /**< false = entries matching rules are excluded */

    /**
     * @brief Encode the full config.
     *
     * Wire layout: str16 name, u8 whitelist flag, u32 rule count, then
     * each ThresholdRule in encode() order.
     *
     * @param w  Writer to append the encoded bytes to.
     */
    void encode(BufWriter& w) const {
        w.putStr16(name.c_str(), name.size());
        w.putU8(isWhitelist ? 1 : 0);
        w.putU32(static_cast<uint32_t>(rules.size()));
        for (const auto& r : rules) {
            r.encode(w);
        }
    }

    /**
     * @brief Decode a config. Returns true on success.
     *
     * Reads the layout produced by encode(); the rule count is bounded by
     * MAX_COLLECTION_COUNT and each rule is validated by
     * ThresholdRule::decode().
     *
     * @param r    Reader positioned at an encoded ThresholdConfig.
     * @param out  Config to overwrite with the decoded contents.
     * @return true on success; false on truncated/malformed input — in
     *         which case @p out may be partially written.
     */
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

/**
 * Custom threshold evaluation callback type.
 * Receives: entry ID, old value ptr, new value ptr, value size, config primitives.
 * Returns true if the change should be reported.
 */
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
    /**
     * @brief Set the active threshold configuration.
     *
     * Replaces the rule set used by passes() for subsequent calls. Does
     * not clear per-entry last-sent state — a rule change is compared
     * against the values last reported under the old configuration; call
     * reset() if the new config should start from scratch.
     *
     * @param config  ThresholdConfig to copy (name, rules, whitelist
     *                flag).
     */
    void setConfig(const ThresholdConfig& config) { config_ = config; }

    /**
     * @brief Get the active threshold configuration.
     *
     * @return Const reference to the installed config; valid until the
     *         next setConfig() call.
     */
    const ThresholdConfig& config() const { return config_; }

    /**
     * @brief Register a custom threshold evaluator by name.
     *
     * The name is matched against `ThresholdRule::customName` for rules
     * of type Custom — the callback then decides whether a value change
     * is significant using the rule's customConfig primitives. A custom
     * rule whose name has no registered evaluator passes every change
     * through (permissive default), so unknown evaluators degrade to
     * "no filtering" rather than blocking the stream. Re-registering a
     * name replaces the previous evaluator.
     *
     * @param name  Evaluator name referenced by custom rules.
     * @param fn    Callback invoked for matching custom rules.
     */
    void registerCustom(const std::string& name, CustomThresholdFn fn) {
        customEvaluators_[name] = std::move(fn);
    }

    /**
     * Check if a value change passes the threshold for the given entry.
     * @param entryId  The parameter or signal ID.
     * @param oldVal   Pointer to the previous value bytes.
     * @param newVal   Pointer to the new value bytes.
     * @param valSize  Byte size of the value.
     * @return true if the new value should be transmitted.
     */
    bool passes(uint64_t entryId, const void* oldVal, const void* newVal, size_t valSize) const;

    /**Reset all last-sent value state. */
    void reset();

private:
    const ThresholdRule* findRule(uint64_t entryId) const;

    ThresholdConfig config_;
    std::map<std::string, CustomThresholdFn> customEvaluators_;
};

}} // namespace tether::io
