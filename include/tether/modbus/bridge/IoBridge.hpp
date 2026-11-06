/**
 * @file IoBridge.hpp
 * @brief Exposes Modbus device-profile registers as tether::io
 *        params/signals (IParameterExposer impl) — plan §14.
 *
 * Mapping: writable register → ParamEntry; read-only → SignalEntry.
 * Bit-typed registers map to ValueType::Bool (1 byte), everything else
 * to ValueType::F64 (8 bytes). A live read-through hits the device; a
 * background value cache (fed by a Poller via publish()) makes reads
 * cheap when configured.
 */
#pragma once

#if !defined(ESP_PLATFORM) && defined(TETHER_MODBUS_IO_BRIDGE)

#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "tether/io/ParameterExposer.hpp"
#include "tether/io/Registry.hpp"
#include "tether/modbus/client/Poller.hpp"
#include "tether/modbus/profile/Profile.hpp"

namespace tether::modbus {

class IoBridge final : public tether::io::IParameterExposer {
public:
    IoBridge(BoundDevice& device, std::string busName,
             Poller* poller = nullptr)
        : dev_(device), bus_(std::move(busName)), poller_(poller) {}

    const char* moduleName() const override { return "modbus"; }

    void expose(tether::io::Registry& registry,
                uint64_t idBase) override {
        uint32_t local = 0;
        for (const auto& decl : dev_.profile().registers) {
            const auto name = bus_ + "." + decl.name;
            const bool bit = decl.spec.type == ValueType::Bit;
            const auto vt = bit ? tether::io::ValueType::Bool
                                : tether::io::ValueType::F64;
            const auto group = "modbus." + bus_ +
                (decl.group.empty() ? "" : "." + decl.group);

            if (decl.writable) {
                tether::io::ParamEntry e;
                e.id = tether::io::makeId(idBase, local++);
                e.name = name;
                e.description = decl.description;
                e.group = group;
                e.valueType = vt;
                e.readFn = [this, nm = decl.name, bit](void* dst) {
                    store(dst, readCachedOrLive(nm), bit);
                };
                e.writeFn = [this, nm = decl.name, bit](const void* src) {
                    (void)dev_.write(nm, load(src, bit));
                };
                fillMeta(e, decl);
                registry.addParam(std::move(e));
            } else {
                tether::io::SignalEntry e;
                e.id = tether::io::makeId(idBase, local++);
                e.name = name;
                e.description = decl.description;
                e.group = group;
                e.valueType = vt;
                e.readFn = [this, nm = decl.name, bit](void* dst) {
                    store(dst, readCachedOrLive(nm), bit);
                };
                fillMeta(e, decl);
                registry.addSignal(std::move(e));
            }
        }
    }

    /// Poller-fed value cache update (call from PollItem::onResult).
    void publish(const std::string& name, double v) {
        std::lock_guard g(cacheM_);
        cache_[name] = v;
        fresh_[name] = true;
    }
    /// Last cached value for a register name.
    std::optional<double> cached(const std::string& name) const {
        std::lock_guard g(cacheM_);
        auto it = cache_.find(name);
        return it == cache_.end() ? std::nullopt
                                  : std::optional<double>(it->second);
    }
    /// Whether publish() has delivered at least one value.
    bool fresh(const std::string& name) const {
        std::lock_guard g(cacheM_);
        auto it = fresh_.find(name);
        return it != fresh_.end() && it->second;
    }

private:
    double readCachedOrLive(const std::string& name) {
        if (poller_) {
            if (auto v = cached(name)) return *v;
        }
        auto r = dev_.read(name);
        return r ? *r : std::numeric_limits<double>::quiet_NaN();
    }

    // io wire layout: Bool = 1 byte, F64 = 8-byte double.
    static double load(const void* src, bool bit) {
        if (bit) {
            uint8_t v = 0;
            std::memcpy(&v, src, 1);
            return v ? 1.0 : 0.0;
        }
        double v;
        std::memcpy(&v, src, 8);
        return v;
    }
    static void store(void* dst, double v, bool bit) {
        if (bit) {
            const uint8_t b = (v != 0.0 && v == v) ? 1 : 0;  // NaN→0
            std::memcpy(dst, &b, 1);
        } else {
            std::memcpy(dst, &v, 8);
        }
    }

#if TETHER_IO_METADATA_ENABLED
    static void fillMeta(auto& e, const RegisterDecl& d) {
        if (!d.unit.empty()) e.metadata["unit"] = d.unit;
        e.metadata["modbus.table"] = std::to_string(int(d.addr.table));
        e.metadata["modbus.addr"]  = std::to_string(d.addr.offset);
        if (d.writable) {
            e.metadata["min"] = std::to_string(d.min);
            e.metadata["max"] = std::to_string(d.max);
        }
        if (d.pollMs > 0)
            e.metadata["modbus.poll_ms"] = std::to_string(d.pollMs);
    }
#else
    static void fillMeta(auto&, const RegisterDecl&) {}
#endif

    BoundDevice& dev_;
    std::string  bus_;
    Poller*      poller_;
    std::map<std::string, double> cache_;
    std::map<std::string, bool>   fresh_;
    mutable std::mutex            cacheM_;
};

} // namespace tether::modbus

#endif // TETHER_MODBUS_IO_BRIDGE
