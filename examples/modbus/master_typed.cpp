// modbus_master_typed — typed register access over a simulated bus.
//   Demonstrates: ScalarSpec (u16/i16/f32, word orders, scale+offset),
//   Address notation parsing, TypedRef read/write, BlockPlan merging,
//   and device profiles (BoundDevice) — all without hardware.
#include <print>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;

static void dumpAddress(std::string_view text) {
    auto a = parseAddress(text);
    if (!a) { std::print("  {:<10} -> {}\n", text, a.error().format());
              return; }
    std::print("  {:<10} -> table={} off={}{}\n", text, int(a->table),
               a->offset,
               a->hasBit() ? std::format(" bit={}", a->bit)
                           : std::string{});
}

int main() {
    // ---- simulated device -------------------------------------------------
    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 64);
    (void)dev.store().declare(TableId::InputRegister, 0, 64);
    (void)dev.store().declare(TableId::Coil, 0, 32);
    dev.preload(0, {1234, 0xFF9C, 0x41BC, 0x0000});   // u16, i16, f32 23.5

    ModbusClient cli(bus.invoker(), 1);

    // ---- address notations ------------------------------------------------
    std::print("== address parsing\n");
    for (auto s : {"40001", "40001.3", "3x0005", "hr:10", "ir:4",
                   "coil:9", "0x1F", "17"})
        dumpAddress(s);

    // ---- TypedRef: scaled/engineering-unit access --------------------------
    std::print("== TypedRef scalar decode\n");
    TypedRef u16r(cli, Address{TableId::HoldingRegister, 0},
                  {ValueType::U16});
    TypedRef i16r(cli, Address{TableId::HoldingRegister, 1},
                  {ValueType::I16});
    // hr2/3 hold f32 23.456 in big-endian (0x41BB4839-ish layout)
    TypedRef f32r(cli, Address{TableId::HoldingRegister, 2},
                  {ValueType::F32, WordOrder::BigEndian});
    // 0.1 °C scaled temperature: raw 231 -> 23.1
    TypedRef temp(cli, Address{TableId::HoldingRegister, 4},
                  {ValueType::I16, WordOrder::BigEndian, 0.1, 0.0});

    std::print("  u16={} i16={} f32={:.3f}\n",
               u16r.read().value_or(-1), i16r.read().value_or(-1),
               f32r.read().value_or(-1));
    (void)temp.write(24.5);
    std::print("  wrote 24.5 -> raw {} (expect 245)\n",
               cli.readHoldingOne(4).value_or(0));

    // ---- write path: engineering -> raw ------------------------------------
    std::print("== typed writes\n");
    TypedRef volt(cli, Address{TableId::HoldingRegister, 8},
                  {ValueType::F32, WordOrder::WordSwap});
    (void)volt.write(230.4);
    auto regs = cli.readHolding(8, 2);
    if (regs)
        std::print("  230.4 word-swapped -> [{:04x} {:04x}]\n",
                   (*regs)[0], (*regs)[1]);

    // ---- BlockPlan: merge scattered reads into minimal requests -----------
    std::print("== BlockPlan\n");
    BlockPlan plan(/*maxGap*/4, /*maxQty*/125);
    plan.add(TableId::HoldingRegister, 0, 2);
    plan.add(TableId::HoldingRegister, 4, 1);   // merges w/ 0..1 (gap 2)
    plan.add(TableId::HoldingRegister, 20, 4);  // too far -> own block
    plan.add(TableId::InputRegister, 0, 8);     // different table
    for (auto& b : plan.blocks())
        std::print("  table={} off={} qty={}\n", int(b.table), b.offset,
                   b.qty);

    // ---- device profile: named typed access --------------------------------
    std::print("== BoundDevice profile\n");
    ProfileRegistry reg;
    reg.registerBuiltins();
    auto* prof = reg.find("generic/vfd");
    if (prof) {
        BoundDevice vfd(cli, *prof);
        (void)vfd.write("freq_set", 48.25);        // -> raw 4825
        auto f = vfd.read("freq_set");
        std::print("  freq_set: {:.2f} Hz (raw {})\n",
                   f.value_or(-1),
                   cli.readHoldingOne(1).value_or(0));
        (void)vfd.write("run", 1.0);               // coil 0 on
        std::print("  run coil -> {}\n",
                   cli.readCoils(0, 1).value_or(std::vector<bool>{})[0]);
    }
    return 0;
}
