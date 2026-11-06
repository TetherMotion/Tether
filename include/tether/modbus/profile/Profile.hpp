/**
 * @file Profile.hpp
 * @brief Device profiles: named registers w/ types, quirks, write policy.
 *        (plan §12). Profiles are data + behavior, not subclassed code.
 */
#pragma once

#include <map>
#include <optional>
#include <vector>

#include "tether/modbus/typed/TypedRef.hpp"

namespace tether::modbus {

struct RegisterDecl {
    std::string  name;
    Address      addr;
    ScalarSpec   spec;
    int          pollMs = 0;          ///< suggested poll interval (0 = none)
    bool         writable = false;
    double       min = 0, max = 0;    ///< write clamp range
    std::string  unit;                ///< "°C", "rpm", ...
    std::string  group;               ///< UI grouping
    std::string  description;
};

/// Behavioral quirks a device may need (plan §12.4).
struct ProfileQuirks {
    bool     zeroBasedAddressing = true;   ///< false → subtract 1 from addrs
    bool     swapWordsIn32       = false;  ///< legacy word-swap f32/u32
    bool     needsT35Strict      = false;  ///< require extra inter-frame delay
    bool     noFc16              = false;  ///< only FC6 writes supported
    uint16_t maxReadQty          = 0;      ///< 0 = protocol default
    int      interFrameDelayUs   = 0;
};

struct DeviceProfile {
    std::string                id;          ///< "vendor/model" or device id
    std::string                vendor;
    std::string                model;
    std::vector<RegisterDecl>  registers;
    ProfileQuirks              quirks;
    uint8_t                    defaultUnit = 1;

    const RegisterDecl* find(const std::string& name) const {
        for (const auto& r : registers)
            if (r.name == name) return &r;
        return nullptr;
    }
};

/**
 * @brief A device bound to a client + profile: typed access by name.
 */
class BoundDevice {
public:
    BoundDevice(ModbusClient& client, const DeviceProfile& prof)
        : client_(client), prof_(prof) {
        client_.setUnit(prof.defaultUnit);
    }

    TypedRef ref(const std::string& name) const {
        const auto* d = prof_.find(name);
        if (!d) throw std::out_of_range("no register: " + name);
        return TypedRef(client_, applyQuirks(d->addr), d->spec);
    }
    Expected<double> read(const std::string& name,
                          const InvokeOptions& o = {}) const;
    Expected<void> write(const std::string& name, double v,
                         const InvokeOptions& o = {}) const;

    /// Suggested PollItems for all registers with pollMs > 0.
    std::vector<std::pair<std::string, RegisterDecl>> pollables() const;

    const DeviceProfile& profile() const { return prof_; }

private:
    /// Apply addressing quirks to a declared address.
    Address applyQuirks(Address a) const {
        if (!prof_.quirks.zeroBasedAddressing && a.offset > 0) --a.offset;
        return a;
    }

    ModbusClient&       client_;
    const DeviceProfile& prof_;
};

/// Registry: id → profile (+ a few built-ins).
class ProfileRegistry {
public:
    void add(DeviceProfile p) { profs_[p.id] = std::move(p); }
    const DeviceProfile* find(std::string_view id) const {
        auto it = profs_.find(std::string(id));
        return it == profs_.end() ? nullptr : &it->second;
    }
    std::vector<std::string> ids() const {
        std::vector<std::string> v;
        for (auto& [k, _] : profs_) v.push_back(k);
        return v;
    }
    /// Register the bundled catalog (generic VFD, energy meter, PLC IO).
    void registerBuiltins();
private:
    std::map<std::string, DeviceProfile> profs_;
};

} // namespace tether::modbus
