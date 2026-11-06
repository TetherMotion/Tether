// modbus_sim_demo — SimBus + robust strategies, no hardware needed.
#include <print>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/robust/RobustInvoker.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;

int main() {
    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 100);
    dev.preload(0, {100, 200, 300, 400});
    dev.traits().dropResponse = 0.05;   // 5% flakiness
    dev.traits().corrupt = 0.02;

    ModbusClient raw(bus.invoker(), 1);
    std::print("== direct reads (5%% drops)\n");
    for (int i = 0; i < 8; ++i) {
        auto r = raw.readHolding(0, 4);
        if (r) std::print("  ok: {} {} {} {}\n",
                          (*r)[0], (*r)[1], (*r)[2], (*r)[3]);
        else   std::print("  err: {}\n", r.error().format());
    }

#if defined(TETHER_MODBUS_ROBUST_THRICE_VOTE)
    std::print("== thrice-vote reads\n");
    RobustInvoker robust(bus.invoker(),
                         AssuranceOpts{.strategy =
                             Strategy::ThriceMajorityVote});
    ModbusClient rc(robust, 1);
    for (int i = 0; i < 5; ++i) {
        auto r = rc.readHolding(0, 4);
        std::print("  {}\n", r ? "voted ok" : r.error().format());
    }
#else
    std::print("(thrice vote not compiled)\n");
#endif

#if defined(TETHER_MODBUS_ROBUST_WRITE_CONFIRM)
    std::print("== write+confirm\n");
    RobustInvoker wc(bus.invoker(),
                     AssuranceOpts{.strategy = Strategy::WriteConfirm});
    auto wb = build::writeSingleReg(9, 0x77);
    auto rb = build::readRange(FunctionCode::ReadHoldingRegisters, 9, 1);
    auto res = wc.assuredWrite(1, FunctionCode::WriteSingleRegister,
                               wb->body(), FunctionCode::ReadHoldingRegisters,
                               rb->body(),
                               std::vector<uint8_t>{0x02, 0x00, 0x77});
    std::print("  {}\n", res ? "confirmed" : res.error().format());
#endif
    return 0;
}
