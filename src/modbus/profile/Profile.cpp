#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/drivers/JUMO/Imago500Map.hpp"

namespace tether::modbus {

Expected<double> BoundDevice::read(const std::string& name,
                                   const TransactionOptions& o) const {
    const auto* d = prof_.find(name);
    if (!d)
        return err<double>(ModbusError::config("unknown register: " + name));
    return TypedRef(client_, applyQuirks(d->addr), d->spec).read(o);
}

Expected<void> BoundDevice::write(const std::string& name, double v,
                                  const TransactionOptions& o) const {
    const auto* d = prof_.find(name);
    if (!d)
        return err<void>(ModbusError::config("unknown register: " + name));
    if (!d->writable)
        return err<void>(ModbusError::config("read-only: " + name));
    // min==max==0 = unclamped (writing a coil to 1 must not trip this).
    if (!(d->min == 0.0 && d->max == 0.0) && (v < d->min || v > d->max))
        return err<void>(ModbusError::config("out of range: " + name));
    return TypedRef(client_, applyQuirks(d->addr), d->spec).write(v, o);
}

std::vector<std::pair<std::string, RegisterDecl>>
BoundDevice::pollables() const {
    std::vector<std::pair<std::string, RegisterDecl>> out;
    for (const auto& r : prof_.registers)
        if (r.pollMs > 0) out.emplace_back(r.name, r);
    return out;
}

void ProfileRegistry::registerBuiltins() {
    {   // Generic VFD (frequency inverter), common layout
        DeviceProfile p;
        p.id = "generic/vfd"; p.vendor = "generic"; p.model = "vfd";
        p.defaultUnit = 1;
        p.registers = {
            {"status",   {TableId::InputRegister, 0},  {ValueType::U16}, 100},
            {"freq_act", {TableId::InputRegister, 1},
                         {ValueType::U16, WordOrder::BigEndian, 0.01}, 100,
                         false, 0, 0, "Hz"},
            {"current",  {TableId::InputRegister, 2},
                         {ValueType::U16, WordOrder::BigEndian, 0.1}, 200,
                         false, 0, 0, "A"},
            {"fault",    {TableId::InputRegister, 3}, {ValueType::U16}},
            {"cmd",      {TableId::HoldingRegister, 0}, {ValueType::U16},
                         0, true, 0, 65535},
            {"freq_set", {TableId::HoldingRegister, 1},
                         {ValueType::U16, WordOrder::BigEndian, 0.01}, 0,
                         true, 0, 400.0, "Hz"},
            {"fault_reset", {TableId::HoldingRegister, 2},
                         {ValueType::U16}, 0, true},
            {"run",      {TableId::Coil, 0}, {ValueType::Bit}, 0, true},
        };
        add(std::move(p));
    }
    {   // Generic energy meter (SDM120-style)
        DeviceProfile p;
        p.id = "generic/energy-meter"; p.vendor = "generic";
        p.model = "energy-meter";
        p.registers = {
            {"voltage",  {TableId::InputRegister, 0},
                         {ValueType::F32, WordOrder::BigEndian}, 500,
                         false, 0, 0, "V"},
            {"current",  {TableId::InputRegister, 2},
                         {ValueType::F32, WordOrder::BigEndian}, 500,
                         false, 0, 0, "A"},
            {"power",    {TableId::InputRegister, 12},
                         {ValueType::F32, WordOrder::BigEndian}, 500,
                         false, 0, 0, "W"},
            {"energy",   {TableId::InputRegister, 72},
                         {ValueType::F32, WordOrder::BigEndian}, 1000,
                         false, 0, 0, "kWh"},
        };
        p.quirks.swapWordsIn32 = true;   // many meters are word-swapped
        add(std::move(p));
    }
    {   // Generic remote IO block (16DI + 16DO + 4AI)
        DeviceProfile p;
        p.id = "generic/io-block"; p.vendor = "generic"; p.model = "io";
        p.registers = {
            {"di",  {TableId::DiscreteInput, 0}, {ValueType::U16}, 50},
            {"do",  {TableId::Coil, 0},          {ValueType::U16}, 0, true},
            {"ai0", {TableId::InputRegister, 0},
                    {ValueType::U16, WordOrder::BigEndian, 0.001}, 200,
                    false, 0, 0, "V"},
            {"ao0", {TableId::HoldingRegister, 0},
                    {ValueType::U16, WordOrder::BigEndian, 0.001}, 0,
                    true, 0, 10.0, "V"},
        };
        add(std::move(p));
    }
    // JUMO Imago 500 — complete grouped map lives in the driver header.
    add(jumo::imago500::imago500Profile());
}

} // namespace tether::modbus
