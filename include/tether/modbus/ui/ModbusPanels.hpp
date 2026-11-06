/**
 * @file ModbusPanels.hpp
 * @brief Toolkit-agnostic Tether::UI model adapters for Modbus
 *        (plan §16): ModbusEntityProvider (one entity per device),
 *        profileRegisters() (NamedRegister list from a DeviceProfile),
 *        ModbusRegisterReader (IRegisterReader over BoundDevice).
 *
 * Header-only — only depends on <tether/ui/Panels.hpp> value types,
 * never on ncurses/terminal_ui.
 */
#pragma once

#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "tether/modbus/profile/Profile.hpp"
#include "tether/ui/Panels.hpp"

namespace tether::modbus {

/// One entity row per bound Modbus device.
class ModbusEntityProvider final
    : public Tether::UI::ISlaveEntityProvider {
public:
    void add(uint16_t index, BoundDevice& dev, std::string name,
             std::string endpoint) {
        std::lock_guard g(m_);
        entries_.push_back({index, &dev, std::move(name),
                            std::move(endpoint), false, {}});
    }
    /// Health feed: call from poller/executor on success/failure.
    void report(uint16_t index, bool ok, std::string detail = {}) {
        std::lock_guard g(m_);
        for (auto& e : entries_)
            if (e.index == index) {
                e.err = !ok;
                if (!detail.empty()) e.detail = std::move(detail);
            }
    }

    std::vector<Tether::UI::SlaveEntity> entities() override {
        std::lock_guard g(m_);
        std::vector<Tether::UI::SlaveEntity> out;
        out.reserve(entries_.size());
        for (const auto& e : entries_) {
            out.push_back(Tether::UI::SlaveEntity{
                .index = e.index,
                .name = e.name,
                .state = e.err ? "ERR" : "ONLINE",
                .status_code = e.err ? uint16_t(1) : uint16_t(0),
                .error = e.err,
                .suspended = false,
                .detail = e.detail.empty() ? e.endpoint : e.detail,
            });
        }
        return out;
    }

private:
    struct Entry {
        uint16_t index;
        BoundDevice* dev;
        std::string name, endpoint;
        bool err;
        std::string detail;
    };
    std::vector<Entry> entries_;
    std::mutex m_;
};

/// NamedRegister list from a device profile (index = modbus offset,
/// subindex = table id, name = profile register name, bits = size).
inline std::vector<Tether::UI::NamedRegister>
profileRegisters(const DeviceProfile& p) {
    std::vector<Tether::UI::NamedRegister> out;
    out.reserve(p.registers.size());
    for (const auto& d : p.registers) {
        out.push_back(Tether::UI::NamedRegister{
            .index = d.addr.offset,
            .subindex = uint8_t(d.addr.table),
            .name = d.name,
            .bits = uint8_t(d.spec.type == ValueType::Bit
                                ? 1
                                : 8 * valueSize(d.spec.type)),
        });
    }
    return out;
}

/// IRegisterReader over a set of BoundDevices; entity_index selects
/// the device position in `devs` (same order as provider entries).
class ModbusRegisterReader final : public Tether::UI::IRegisterReader {
public:
    explicit ModbusRegisterReader(std::vector<BoundDevice*> devs)
        : devs_(std::move(devs)) {}

    Tether::UI::RegisterResult read(
        uint16_t entity_index,
        const Tether::UI::NamedRegister& reg) override {
        if (entity_index >= devs_.size() || !devs_[entity_index])
            return {false, {}, "no device"};
        auto v = devs_[entity_index]->read(reg.name);
        if (!v) return {false, {}, v.error().format()};
        const auto* decl = devs_[entity_index]->profile().find(reg.name);
        std::string val = fmt(*v);
        if (decl && !decl->unit.empty()) val += " " + decl->unit;
        return {true, val, {}};
    }

    /// Write path for editable registers (raw engineering value).
    Tether::UI::RegisterResult write(
        uint16_t entity_index,
        const Tether::UI::NamedRegister& reg, double v) {
        if (entity_index >= devs_.size() || !devs_[entity_index])
            return {false, {}, "no device"};
        auto r = devs_[entity_index]->write(reg.name, v);
        if (!r) return {false, {}, r.error().format()};
        return {true, fmt(v), {}};
    }

private:
    static std::string fmt(double v) {
        char buf[64];
        if (v == double(int64_t(v)) && std::abs(v) < 1e15)
            std::snprintf(buf, sizeof buf, "%lld", (long long)v);
        else
            std::snprintf(buf, sizeof buf, "%.4g", v);
        return buf;
    }
    std::vector<BoundDevice*> devs_;
};

} // namespace tether::modbus
