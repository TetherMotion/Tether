// modbus_jumo_imago500 — JUMO Imago 500 (703590) multi-channel process
//   and program controller over Modbus/RTU. Runs against SimBus — no
//   hardware needed; swap `bus.invoker()` for an RtuChannel on a serial
//   port for the real device.
//
//   Covered:
//    - JumoImago500 typed driver: status bits, per-channel PV block,
//      EEPROM vs RAM setpoints, controller params, command words,
//      program start / manual mode / segment transmission, clock,
//      identification.
//    - imago500Profile(): every datasheet register exposed by name
//      through BoundDevice (group-tagged for UI/IoBridge).
#include <cstring>
#include <print>
#include <set>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/drivers/JUMO/Imago500.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using namespace tether::modbus::jumo::imago500;

/// Little helper: JUMO FLOAT = IEEE-754, low word first on the wire.
static void putFloat(DataStore& st, TableId t, uint16_t a, float f) {
    uint32_t u; std::memcpy(&u, &f, 4);
    (void)st.setReg(t, a,     uint16_t(u & 0xFFFF));
    (void)st.setReg(t, a + 1, uint16_t(u >> 16));
}
static void putLong(DataStore& st, uint16_t a, int32_t v) {   // high word 1st
    (void)st.setReg(TableId::InputRegister, a,
                    uint16_t(uint32_t(v) >> 16));
    (void)st.setReg(TableId::InputRegister, a + 1,
                    uint16_t(uint32_t(v)));
}

int main() {
    SimBus bus;
    auto& jumo = bus.add(7);
    (void)jumo.store().declare(TableId::InputRegister,  0, 0x1000);
    (void)jumo.store().declare(TableId::HoldingRegister,0, 0x1000);
    auto& st = jumo.store();

    // ---- seed a live-looking device ----------------------------------------
    st.setReg(TableId::InputRegister, addr::genStatus,
              (1u << 8) | (1u << 2));            // automatic + delay
    st.setReg(TableId::InputRegister, addr::ctlStatus,
              (1u << 0) | (1u << 13));           // C4 manual, C1 self-opt
    st.setReg(TableId::InputRegister, addr::logicOutputs, 1u << 1); // out7
    st.setReg(TableId::InputRegister, addr::analogInBase, 0);       // fillers
    putFloat(st, TableId::InputRegister, addr::analogInBase,   211.4f);
    putFloat(st, TableId::InputRegister, addr::chBlockBase + 0x02,
             187.25f);                           // C1 filtered PV
    putFloat(st, TableId::InputRegister, addr::chBlockBase + 0x06,
             200.0f);                            // C1 setpoint
    putFloat(st, TableId::InputRegister, addr::chBlockBase + 0x08,
             62.5f);                             // C1 output %
    st.setReg(TableId::InputRegister, addr::chBlockBase + 0x12, 1); // heat on
    putLong(st, addr::progRunTime, 3725);        // 1h 2m 5s
    st.setReg(TableId::InputRegister, addr::programNumber, 4);
    st.setReg(TableId::InputRegister, addr::identOutBoard,     0x11);
    st.setReg(TableId::InputRegister, addr::identInterface,  0x02);

    ModbusClient cli(bus.invoker());
    JumoImago500 dev(cli, 7);

    std::print("== JUMO Imago 500 @ unit 7\n"); std::fflush(stdout);

    // ---- status -------------------------------------------------------------
    if (auto s = dev.generatorStatus(); s)
        std::print("status: automatic={} manual={} delay={} end={}\n",
                   s->automatic, s->manual, s->delay, s->programEnd);
    std::print("ctl status: c1 selfopt={} c4 manual={}\n",
               dev.ctlSelfOptActive(1).value_or(false),
               dev.ctlManualActive(4).value_or(false));
    std::print("logic out7={} out1={}\n",
               dev.logicOutput(7).value_or(false),
               dev.logicOutput(1).value_or(false));
    std::print("analog in1 = {:.2f}\n",
               dev.analogInput(1).value_or(-1));

    // ---- per-channel process values ------------------------------------------
    if (auto pv = dev.channelPv(1); pv)
        std::print("c1: pv={:.2f} sp={:.1f} out={:.1f}% heat={}\n",
                   pv->pvFiltered, pv->setpoint, pv->output,
                   pv->heatingOn);
    std::print("program {} running {} s ({} left)\n",
               dev.programNumber().value_or(-1),
               dev.programRunTimeS().value_or(-1),
               dev.programRemainingS().value_or(-1));

    // ---- setpoints: EEPROM vs RAM ---------------------------------------------
    (void)dev.setSetpoint(1, 1, 210.0);       // EEPROM W1 — infrequent!
    (void)dev.setSetpointRam(1, 212.5);       // RAM — frequent writes safe
    std::print("c1 w1={:.1f} (EEPROM) ram_sp={:.1f} (volatile)\n",
               dev.setpoint(1, 1).value_or(-1),
               dev.setpointRam(1).value_or(-1));

    // ---- controller params ------------------------------------------------------
    (void)dev.setParam(1, 1, JumoImago500::Param::TN1, 240.0);
    std::print("c1.p1.tn1 = {:.0f} s\n",
               dev.param(1, 1, JumoImago500::Param::TN1).value_or(-1));

    // ---- commands ----------------------------------------------------------
    (void)dev.ctlAutomatic(1);
    (void)dev.selectSetpoint(1, 1);
    (void)dev.selectParamSet(1, 1);
    std::print("c1 -> automatic, W1, param set 1\n");

    // ---- program start -----------------------------------------------------
    JumoImago500::ProgramStart ps;
    ps.programNo = 4; ps.secs = -1;      // instant start
    (void)dev.startProgram(ps);
    std::print("program 4 start issued (buffer=0x{:04x})\n",
               cli.readHolding(addr::progBuffer, 1)
                   .value_or(std::vector<uint16_t>{0})[0]);

    // ---- clock + ident ---------------------------------------------------------
    (void)dev.setClock(24, 6, 15, 9, 41, 0);
    if (auto id = dev.deviceIdent(); id)
        std::print("ident: out_board1=0x{:02x} interface=0x{:02x}\n",
                   id->outBoard[0], id->interface);

    // ---- name-based access through the profile ----------------------------------
    std::print("== profile exposure\n");
    const auto prof = imago500Profile(7);
    BoundDevice bd(cli, prof);
    std::print("  {} registers in {} profile\n", prof.registers.size(),
               prof.id);
    std::print("  bd[\"c1.pv_filtered\"] = {:.2f}\n",
               bd.read("c1.pv_filtered").value_or(-1));
    (void)bd.write("c2.ram_setpoint", 180.0);
    std::print("  bd[\"c2.ram_setpoint\"] <- 180 -> {:.1f}\n",
               bd.read("c2.ram_setpoint").value_or(-1));
    std::print("  write to R/O c1.pv_filtered -> {}\n",
               bd.write("c1.pv_filtered", 1).has_value()
                   ? "accepted!" : "rejected (read-only)");
    // groups available for UI/IO exposition
    std::set<std::string_view> groups;
    for (const auto& r : prof.registers) groups.insert(r.group);
    std::print("  groups:");
    for (auto g : groups) std::print(" {}", g);
    std::print("\n");
    return 0;
}
