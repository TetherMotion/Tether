// modbus_vfd_driver — device drivers on top of profiles and raw tables.
//   ModbusVfd wraps a BoundDevice profile ("generic/vfd" semantics);
//   ModbusRemoteIo drives a remote I/O module with no profile at all.
//   Both run over SimBus — no hardware needed.
#include <print>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/drivers/Drivers.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;

int main() {
    SimBus bus;

    // ---- simulated VFD (unit 1): profile-shaped store -----------------------
    auto& vfd = bus.add(1);
    (void)vfd.store().declare(TableId::InputRegister, 0, 8);
    (void)vfd.store().declare(TableId::HoldingRegister, 0, 8);
    (void)vfd.store().declare(TableId::Coil, 0, 8);
    // telemetry regs in the sim store (raw: freq x100, current x10)
    (void)vfd.store().setReg(TableId::InputRegister, 0, 0x0103); // status
    (void)vfd.store().setReg(TableId::InputRegister, 1, 5000);   // 50.00 Hz
    (void)vfd.store().setReg(TableId::InputRegister, 2, 42);     // 4.2 A

    // ---- simulated remote-I/O (unit 2): flat tables --------------------------
    auto& io = bus.add(2);
    (void)io.store().declare(TableId::DiscreteInput, 0, 16);
    (void)io.store().declare(TableId::Coil, 0, 16);
    (void)io.store().declare(TableId::InputRegister, 0, 8);
    (void)io.store().declare(TableId::HoldingRegister, 0, 8);
    (void)io.store().setBit(TableId::DiscreteInput, 3, true);  // di3 on
    (void)io.store().setReg(TableId::InputRegister, 0, 1234);  // ai0

    ModbusClient cli(bus.invoker());

    // ---- VFD via profile + driver -------------------------------------------
    std::print("== ModbusVfd (unit 1, generic/vfd profile)\n");
    ProfileRegistry reg; reg.registerBuiltins();
    const auto* prof = reg.find("generic/vfd");
    if (!prof) { std::print(stderr, "profile missing\n"); return 1; }
    BoundDevice bound(cli, *prof);
    ModbusVfd drive(bound);

    (void)drive.setFrequencyHz(48.25);
    (void)drive.run(true);
    std::print("  setpoint {:.2f} Hz, output {:.2f} Hz, current {:.1f} A, "
               "running {}\n",
               bound.read("freq_set").value_or(-1),
               drive.outputFrequencyHz().value_or(-1),
               drive.outputCurrent().value_or(-1),
               drive.running().value_or(-1));
    std::print("  fault={} status=0x{:04x}\n",
               drive.faultCode().value_or(-1),
               cli.readInput(0, 1).value_or(std::vector<uint16_t>{0})[0]);

    // ---- Remote I/O via raw tables ------------------------------------------
    std::print("== ModbusRemoteIo (unit 2, no profile)\n");
    cli.setUnit(2);
    ModbusRemoteIo rio(cli, {.nDi = 16, .nDo = 16,
                             .nAi = 4, .nAo = 4});
    std::print("  di3={} ai0={}\n",
               rio.digitalInput(3).value_or(false),
               rio.analogInput(0).value_or(0));
    (void)rio.setDigitalOutput(5, true);
    (void)rio.setAnalogOutput(0, 4095);
    std::print("  do5 -> {} ao0 -> {}\n",
               rio.digitalOutput(5).value_or(false),
               rio.analogOutput(0).value_or(0));

    // Snapshot reads = single block transactions
    auto ins = rio.inputs();
    if (ins) {
        std::print("  di snapshot:");
        for (size_t i = 0; i < ins->size(); ++i)
            if ((*ins)[i]) std::print(" di{}", i);
        std::print("\n");
    }
    return 0;
}
