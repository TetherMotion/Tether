/**
 * @file ModbusIoExposer.hpp
 * @brief tether::io exposers for the Modbus device drivers + generic
 *        register access functions (plan §14.4):
 *
 *  - ModbusBusExposer      generic functions on a ModbusClient:
 *      "<p>.read"  {table:u8, addr:u16, qty:u16} → Binary
 *        bit tables return packed LSB-first bits; register tables
 *        return big-endian u16 words.
 *      "<p>.write" {table:u8, addr:u16, data:Binary} → Bool
 *        same payload encoding; qty derived from data length.
 *
 *  - ModbusVfdExposer      signals/params for a ModbusVfd driver.
 *  - ModbusRemoteIoExposer per-channel signals/params for ModbusRemoteIo.
 *
 * Header-only, compiled only under TETHER_MODBUS_IO_BRIDGE.
 */
#pragma once

#if defined(TETHER_MODBUS_IO_BRIDGE) && !defined(ESP_PLATFORM)

#include <cstring>
#include <string>
#include <vector>

#include "tether/io/Function.hpp"
#include "tether/io/ParameterExposer.hpp"
#include "tether/io/Registry.hpp"
#include "tether/modbus/drivers/Drivers.hpp"

namespace tether::modbus {

namespace ioex {

inline uint64_t argU(const std::vector<tether::io::FunctionArgument>& a,
                     size_t i, uint64_t def = 0) {
    if (i >= a.size() || a[i].value.empty()) return def;
    uint64_t v = 0;
    std::memcpy(&v, a[i].value.data(),
                std::min<size_t>(a[i].value.size(), sizeof v));
    return v;
}
inline tether::io::FunctionCallResult boolRes(bool ok) {
    tether::io::FunctionCallResult r;
    r.success = true;
    r.returnValue = {uint8_t(ok ? 1 : 0)};
    return r;
}
inline tether::io::FunctionCallResult failRes(const std::string& m) {
    tether::io::FunctionCallResult r;
    r.errorMessage = m;
    return r;
}
inline std::optional<TableId> tableFromU8(uint8_t v) {
    switch (TableId(v)) {
        case TableId::Coil:            return TableId::Coil;
        case TableId::DiscreteInput:   return TableId::DiscreteInput;
        case TableId::InputRegister:   return TableId::InputRegister;
        case TableId::HoldingRegister: return TableId::HoldingRegister;
    }
    return std::nullopt;
}
/// Pack bools LSB-first (Modbus bit-payload order).
inline std::vector<uint8_t> packBits(const std::vector<bool>& b) {
    std::vector<uint8_t> out((b.size() + 7) / 8, 0);
    for (size_t i = 0; i < b.size(); ++i)
        if (b[i]) out[i / 8] |= uint8_t(1u << (i % 8));
    return out;
}
inline std::vector<bool> unpackBits(std::span<const uint8_t> p, size_t qty) {
    std::vector<bool> out(std::min(qty, p.size() * 8));
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = (p[i / 8] >> (i % 8)) & 1;
    return out;
}
inline std::vector<uint8_t> wordsToBe(std::span<const uint16_t> w) {
    std::vector<uint8_t> out;
    out.reserve(w.size() * 2);
    for (uint16_t v : w) {
        out.push_back(uint8_t(v >> 8));
        out.push_back(uint8_t(v));
    }
    return out;
}

inline void addFn(tether::io::Registry& reg, uint64_t base, uint32_t lid,
                  std::string name, std::string desc, std::string group,
                  std::vector<tether::io::FunctionParameter> params,
                  tether::io::ValueType ret,
                  tether::io::FunctionCallback cb) {
    tether::io::FunctionEntry fn;
    fn.id = tether::io::makeId(base, lid);
    fn.name = std::move(name);
    fn.description = std::move(desc);
    fn.group = std::move(group);
    fn.parameters = std::move(params);
    fn.returnValue.present = true;
    fn.returnValue.name = "result";
    fn.returnValue.type = ret;
    fn.callback = std::move(cb);
    reg.addFunction(std::move(fn));
}
inline tether::io::FunctionParameter param(
    const char* name, const char* desc, tether::io::ValueType t) {
    tether::io::FunctionParameter p;
    p.name = name;
    p.description = desc;
    p.type = t;
    return p;
}
} // namespace ioex

// ---------------------------------------------------------------------------
// Generic register functions
// ---------------------------------------------------------------------------
class ModbusBusExposer : public tether::io::IParameterExposer {
public:
    ModbusBusExposer(ModbusClient& cli, std::string prefix)
        : cli_(cli), prefix_(std::move(prefix)) {}

    const char* moduleName() const override { return "modbus"; }

    void expose(tether::io::Registry& reg, uint64_t idBase) override {
        using VT = tether::io::ValueType;
        const std::string g = "modbus." + prefix_;

        ioex::addFn(reg, idBase, 0x01, prefix_ + ".read",
            "Read table cells. table: 0=coil 1=di 3=in 4=hr. "
            "Returns packed bits (bit tables) or BE u16 words.",
            g,
            {ioex::param("table", "table id (0/1/3/4)", VT::U8),
             ioex::param("addr",  "start address",       VT::U16),
             ioex::param("qty",   "cell count",          VT::U16)},
            VT::Binary,
            [this](const std::vector<tether::io::FunctionArgument>& a) {
                auto t = ioex::tableFromU8(uint8_t(ioex::argU(a, 0)));
                const auto addr = uint16_t(ioex::argU(a, 1));
                const auto qty  = uint16_t(ioex::argU(a, 2));
                if (!t) return ioex::failRes("bad table");
                if (qty == 0 || qty > 2000)
                    return ioex::failRes("bad qty");
                tether::io::FunctionCallResult r;
                if (isBitTable(*t)) {
                    auto v = *t == TableId::Coil
                        ? cli_.readCoils(addr, qty)
                        : cli_.readDiscreteInputs(addr, qty);
                    if (!v) return ioex::failRes(v.error().format());
                    r.returnValue = ioex::packBits(*v);
                } else {
                    auto v = *t == TableId::InputRegister
                        ? cli_.readInput(addr, qty)
                        : cli_.readHolding(addr, qty);
                    if (!v) return ioex::failRes(v.error().format());
                    r.returnValue = ioex::wordsToBe(*v);
                }
                r.success = true;
                return r;
            });

        ioex::addFn(reg, idBase, 0x02, prefix_ + ".write",
            "Write table cells (writable tables only). data = packed "
            "bits for coils, BE u16 words for holding registers.",
            g,
            {ioex::param("table", "table id (0=coil 4=hr)", VT::U8),
             ioex::param("addr",  "start address",           VT::U16),
             ioex::param("data",  "packed bits / BE words",  VT::Binary)},
            VT::Bool,
            [this](const std::vector<tether::io::FunctionArgument>& a) {
                auto t = ioex::tableFromU8(uint8_t(ioex::argU(a, 0)));
                const auto addr = uint16_t(ioex::argU(a, 1));
                if (!t) return ioex::failRes("bad table");
                if (!isWritable(*t)) return ioex::failRes("read-only table");
                if (a.size() < 3 || a[2].value.empty())
                    return ioex::failRes("missing data");
                const auto& d = a[2].value;
                Expected<void> r = err<void>(ModbusError::codec("?"));
                if (*t == TableId::Coil) {
                    auto bits = ioex::unpackBits(d, d.size() * 8);
                    r = cli_.writeCoils(addr, bits);
                } else {
                    if (d.size() % 2) return ioex::failRes("odd byte count");
                    std::vector<uint16_t> w(d.size() / 2);
                    for (size_t i = 0; i < w.size(); ++i)
                        w[i] = uint16_t(d[2 * i] << 8 | d[2 * i + 1]);
                    r = w.size() == 1 ? cli_.writeReg(addr, w[0])
                                      : cli_.writeRegs(addr, w);
                }
                if (!r) return ioex::failRes(r.error().format());
                return ioex::boolRes(true);
            });
    }

private:
    ModbusClient& cli_;
    std::string   prefix_;
};

// ---------------------------------------------------------------------------
// ModbusVfd exposer (device-driver style, like CiA402Exposer)
// ---------------------------------------------------------------------------
class ModbusVfdExposer : public tether::io::IParameterExposer {
public:
    ModbusVfdExposer(ModbusVfd& vfd, std::string prefix)
        : vfd_(vfd), prefix_(std::move(prefix)) {}

    const char* moduleName() const override { return "modbus_vfd"; }

    void expose(tether::io::Registry& reg, uint64_t idBase) override {
        using VT = tether::io::ValueType;
        const std::string g = "modbus." + prefix_;
        auto sig = [&](uint32_t lid, const std::string& nm,
                       const std::string& desc, VT vt,
                       std::function<Expected<double>()> read) {
            tether::io::SignalEntry e;
            e.id = tether::io::makeId(idBase, lid);
            e.name = prefix_ + "." + nm;
            e.description = desc;
            e.group = g;
            e.valueType = vt;
            e.readFn = [read = std::move(read), vt](void* d) {
                auto v = read();
                if (!v) { std::memset(d, 0, vt == VT::F64 ? 8 : 2); return; }
                if (vt == VT::F64) { double x = *v; std::memcpy(d, &x, 8); }
                else if (vt == VT::U16) { uint16_t x = uint16_t(*v);
                                        std::memcpy(d, &x, 2); }
                else { uint8_t x = uint8_t(*v != 0); std::memcpy(d, &x, 1); }
            };
            reg.addSignal(std::move(e));
        };

        sig(0x01, "freq_act", "Output frequency (Hz)", VT::F64,
            [&] { return vfd_.outputFrequencyHz(); });
        sig(0x02, "current", "Output current (A)", VT::F64,
            [&] { return vfd_.outputCurrent(); });
        sig(0x03, "fault", "Fault code", VT::U16,
            [&] { return vfd_.faultCode(); });
        sig(0x04, "faulted", "Fault active", VT::Bool,
            [&] -> Expected<double> {
                auto c = vfd_.faultCode();
                return c ? Expected<double>{*c != 0} : c;
            });
        sig(0x05, "running", "Run state", VT::Bool,
            [&] { return vfd_.running(); });

        tether::io::ParamEntry p;
        p.id = tether::io::makeId(idBase, 0x81);
        p.name = prefix_ + ".freq_set";
        p.description = "Frequency setpoint (Hz)";
        p.group = g;
        p.valueType = VT::F64;
        p.readFn = [this](void* d) {
            auto v = vfd_.device().read(vfd_.names().freqSet);
            double x = v ? *v : 0.0;
            std::memcpy(d, &x, 8);
        };
        p.writeFn = [this](const void* s) {
            double x;
            std::memcpy(&x, s, 8);
            (void)vfd_.setFrequencyHz(x);
        };
        reg.addParam(std::move(p));

        tether::io::ParamEntry r;
        r.id = tether::io::makeId(idBase, 0x82);
        r.name = prefix_ + ".run";
        r.description = "Run command (1=run, 0=stop)";
        r.group = g;
        r.valueType = VT::Bool;
        r.readFn = [this](void* d) {
            auto v = vfd_.running();
            uint8_t x = v && *v != 0 ? 1 : 0;
            std::memcpy(d, &x, 1);
        };
        r.writeFn = [this](const void* s) {
            uint8_t x;
            std::memcpy(&x, s, 1);
            (void)vfd_.run(x != 0);
        };
        reg.addParam(std::move(r));

        ioex::addFn(reg, idBase, 0x83, prefix_ + ".reset_fault",
            "Pulse the fault-reset register", g, {}, VT::Bool,
            [this](const std::vector<tether::io::FunctionArgument>&) {
                auto r2 = vfd_.resetFault();
                if (!r2) return ioex::failRes(r2.error().format());
                return ioex::boolRes(true);
            });
    }

private:
    ModbusVfd&  vfd_;
    std::string prefix_;
};

// ---------------------------------------------------------------------------
// ModbusRemoteIo exposer — per-channel signals/params
// ---------------------------------------------------------------------------
class ModbusRemoteIoExposer : public tether::io::IParameterExposer {
public:
    ModbusRemoteIoExposer(ModbusRemoteIo& io, std::string prefix)
        : io_(io), prefix_(std::move(prefix)) {}

    const char* moduleName() const override { return "modbus_io"; }

    void expose(tether::io::Registry& reg, uint64_t idBase) override {
        using VT = tether::io::ValueType;
        const std::string g = "modbus." + prefix_;
        const auto& c = io_.cfg();
        uint32_t lid = 0x10;

        for (uint16_t i = 0; i < c.nDi; ++i, ++lid) {
            tether::io::SignalEntry e;
            e.id = tether::io::makeId(idBase, lid);
            e.name = prefix_ + ".di" + std::to_string(i);
            e.description = "Digital input " + std::to_string(i);
            e.group = g;
            e.valueType = VT::Bool;
            e.readFn = [this, i](void* d) {
                auto v = io_.digitalInput(i);
                uint8_t x = v && *v ? 1 : 0;
                std::memcpy(d, &x, 1);
            };
            reg.addSignal(std::move(e));
        }
        lid = 0x100;
        for (uint16_t i = 0; i < c.nAi; ++i, ++lid) {
            tether::io::SignalEntry e;
            e.id = tether::io::makeId(idBase, lid);
            e.name = prefix_ + ".ai" + std::to_string(i);
            e.description = "Analog input " + std::to_string(i);
            e.group = g;
            e.valueType = VT::U16;
            e.readFn = [this, i](void* d) {
                auto v = io_.analogInput(i);
                uint16_t x = v ? *v : 0;
                std::memcpy(d, &x, 2);
            };
            reg.addSignal(std::move(e));
        }
        lid = 0x200;
        for (uint16_t i = 0; i < c.nDo; ++i, ++lid) {
            tether::io::ParamEntry e;
            e.id = tether::io::makeId(idBase, lid);
            e.name = prefix_ + ".do" + std::to_string(i);
            e.description = "Digital output " + std::to_string(i);
            e.group = g;
            e.valueType = VT::Bool;
            e.readFn = [this, i](void* d) {
                auto v = io_.digitalOutput(i);
                uint8_t x = v && *v ? 1 : 0;
                std::memcpy(d, &x, 1);
            };
            e.writeFn = [this, i](const void* s) {
                uint8_t x;
                std::memcpy(&x, s, 1);
                (void)io_.setDigitalOutput(i, x != 0);
            };
            reg.addParam(std::move(e));
        }
        lid = 0x300;
        for (uint16_t i = 0; i < c.nAo; ++i, ++lid) {
            tether::io::ParamEntry e;
            e.id = tether::io::makeId(idBase, lid);
            e.name = prefix_ + ".ao" + std::to_string(i);
            e.description = "Analog output " + std::to_string(i);
            e.group = g;
            e.valueType = VT::U16;
            e.readFn = [this, i](void* d) {
                auto v = io_.analogOutput(i);
                uint16_t x = v ? *v : 0;
                std::memcpy(d, &x, 2);
            };
            e.writeFn = [this, i](const void* s) {
                uint16_t x;
                std::memcpy(&x, s, 2);
                (void)io_.setAnalogOutput(i, x);
            };
            reg.addParam(std::move(e));
        }
    }

private:
    ModbusRemoteIo& io_;
    std::string     prefix_;
};

} // namespace tether::modbus

#endif // TETHER_MODBUS_IO_BRIDGE && !ESP_PLATFORM
