/**
 * @file WriteCache.hpp
 * @brief Client-side write buffer: accumulate desired values, then
 *        flush() writes only what changed, coalesced into FC16/FC15
 *        runs (plan §10.6).
 *
 * Semantics:
 *  - set()/setCoil() stage a desired value (dirty when != known).
 *  - seed()/seedCoil() inform the cache of confirmed device state
 *    (e.g. after a read) without marking dirty.
 *  - flush() groups dirty addresses into write runs; gap positions are
 *    filled with their *known* value — an unknown gap address splits
 *    the run (never guess a value to write).
 */
#pragma once

#include <map>
#include <optional>
#include <vector>

#include "tether/modbus/client/ModbusClient.hpp"

namespace tether::modbus {

class WriteCache {
public:
    explicit WriteCache(ModbusClient& client, int maxGap = 0)
        : cli_(client), maxGap_(maxGap) {}

    // ---- staging ------------------------------------------------------------
    void set(uint16_t addr, uint16_t v)  { want_[addr] = v; }
    void setCoil(uint16_t addr, bool v)  { wantCoil_[addr] = v; }
    /// Mark confirmed device state (post-read) — clears dirty flag.
    void seed(uint16_t addr, uint16_t v) {
        known_[addr] = v;
        if (auto it = want_.find(addr);
            it != want_.end() && it->second == v) want_.erase(it);
    }
    void seedCoil(uint16_t addr, bool v) {
        knownCoil_[addr] = v;
        if (auto it = wantCoil_.find(addr);
            it != wantCoil_.end() && it->second == v)
            wantCoil_.erase(it);
    }
    void clear() { want_.clear(); wantCoil_.clear(); }

    /// Staged (or known) value for addr.
    std::optional<uint16_t> cached(uint16_t addr) const {
        if (auto it = want_.find(addr); it != want_.end())
            return it->second;
        if (auto it = known_.find(addr); it != known_.end())
            return it->second;
        return std::nullopt;
    }
    /// Addresses whose staged value differs from known device state.
    size_t dirtyRegs() const {
        size_t n = 0;
        for (auto& [a, v] : want_)
            if (knownValue(a) != v) ++n;
        return n;
    }
    size_t dirtyCoils() const {
        size_t n = 0;
        for (auto& [a, v] : wantCoil_)
            if (knownCoil(a) != v) ++n;
        return n;
    }

    // ---- flush --------------------------------------------------------------
    /// Write all dirty addresses; stops on first error.
    Expected<void> flush(const TransactionOptions& o = {}) {
        for (const auto& run : dirtyRuns()) {
            const uint16_t addr = run.first;
            if (run.second.size() == 1) {
                if (auto e = cli_.writeReg(addr, run.second[0], o); !e)
                    return e;
            } else if (auto e = cli_.writeRegs(addr, run.second, o); !e)
                return e;
            for (size_t i = 0; i < run.second.size(); ++i) {
                known_[uint16_t(addr + i)] = run.second[i];
                want_.erase(uint16_t(addr + i));
            }
        }
        for (const auto& run : dirtyCoilRuns()) {
            const uint16_t addr = run.first;
            if (run.second.size() == 1) {
                if (auto e = cli_.writeCoil(addr, run.second[0], o); !e)
                    return e;
            } else if (auto e = cli_.writeCoils(addr, run.second, o); !e)
                return e;
            for (size_t i = 0; i < run.second.size(); ++i) {
                knownCoil_[uint16_t(addr + i)] = run.second[i];
                wantCoil_.erase(uint16_t(addr + i));
            }
        }
        return {};
    }

    /// Dirty holding-register runs, each {start, values} (visible for
    /// tests/diagnostics). Gap holes are filled from known_ state.
    std::vector<std::pair<uint16_t, std::vector<uint16_t>>>
    dirtyRuns() const {
        std::vector<uint16_t> dirty;
        for (auto& [a, v] : want_)
            if (knownValue(a) != v) dirty.push_back(a);
        return group(dirty, [&](uint16_t a) {
            auto w = want_.find(a);
            if (w != want_.end()) return w->second;
            return known_.at(a);   // gap: must be known (checked below)
        }, [&](uint16_t a) { return known_.contains(a); });
    }
    std::vector<std::pair<uint16_t, std::vector<bool>>>
    dirtyCoilRuns() const {
        std::vector<uint16_t> dirty;
        for (auto& [a, v] : wantCoil_)
            if (knownCoil(a) != v) dirty.push_back(a);
        return group(dirty, [&](uint16_t a) {
            auto w = wantCoil_.find(a);
            if (w != wantCoil_.end()) return w->second;
            return knownCoil_.at(a);
        }, [&](uint16_t a) { return knownCoil_.contains(a); });
    }

private:
    std::optional<uint16_t> knownValue(uint16_t a) const {
        auto it = known_.find(a);
        return it == known_.end() ? std::nullopt
                                  : std::optional(it->second);
    }
    std::optional<bool> knownCoil(uint16_t a) const {
        auto it = knownCoil_.find(a);
        return it == knownCoil_.end() ? std::nullopt
                                      : std::optional(it->second);
    }

    /// Group sorted dirty addrs into runs: adjacent if gap ≤ maxGap_
    /// AND every hole addr has a known fill value.
    template<typename Get, typename Known>
    auto group(const std::vector<uint16_t>& dirty, Get&& get,
               Known&& known) const
        -> std::vector<std::pair<uint16_t,
               std::vector<std::invoke_result_t<Get, uint16_t>>>> {
        using V = std::invoke_result_t<Get, uint16_t>;
        std::vector<std::pair<uint16_t, std::vector<V>>> out;
        for (uint16_t a : dirty) {
            if (!out.empty() &&
                uint32_t(a) <= uint32_t(out.back().first) +
                    out.back().second.size() + maxGap_) {
                // fill holes; abort merge if a hole has no known value
                const uint32_t end =
                    uint32_t(out.back().first) + out.back().second.size();
                bool fillable = true;
                std::vector<V> fill;
                for (uint32_t h = end; h < a; ++h) {
                    if (!known(uint16_t(h))) { fillable = false; break; }
                    fill.push_back(get(uint16_t(h)));
                }
                if (fillable) {
                    for (auto v : fill) out.back().second.push_back(v);
                    out.back().second.push_back(get(a));
                    continue;
                }
            }
            out.push_back({a, {get(a)}});
        }
        return out;
    }

    ModbusClient& cli_;
    int           maxGap_;
    std::map<uint16_t, uint16_t> want_, known_;
    std::map<uint16_t, bool>     wantCoil_, knownCoil_;
};

} // namespace tether::modbus
