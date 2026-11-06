// modbus_master_poller — periodic polling + write coalescing over a
// simulated bus. Demonstrates Poller (intervals, priorities,
// change-only + deadband + debounce) and WriteCache (dirty tracking,
// run coalescing, seed-on-read).
#include <chrono>
#include <print>
#include <thread>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/client/Poller.hpp"
#include "tether/modbus/client/WriteCache.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using namespace std::chrono_literals;

int main() {
    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 64);
    (void)dev.store().declare(TableId::InputRegister, 0, 64);
    dev.preload(0, {0, 0, 0, 0});

    // hr0: slow sine-ish drift (simulated process variable)
    std::atomic<bool> run{true};
    std::thread pv([&] {
        int i = 0;
        while (run.load()) {
            (void)dev.store().setReg(TableId::HoldingRegister, 0,
                                     uint16_t(i++ % 200));
            std::this_thread::sleep_for(20ms);
        }
    });

    ModbusClient cli(bus.invoker(), 1);

    // ---- Poller ------------------------------------------------------------
    Poller poller(cli);
    poller.add(PollItem{
        .fc = FunctionCode::ReadHoldingRegisters, .addr = 0, .qty = 1,
        .intervalMs = 50,
        .onResult = [](const PollItem& it,
                       const std::vector<uint16_t>& v,
                       const ModbusError* e) {
            if (e) std::print("  {} err {}\n", it.name, e->format());
            else   std::print("  {} changed -> {}\n", it.name, v[0]);
        },
        .changedOnly = true, .epsilon = 5,       // fire on |Δ|>5
        .name = "pv"});
    poller.add(PollItem{
        .fc = FunctionCode::ReadHoldingRegisters, .addr = 8, .qty = 2,
        .intervalMs = 200, .priority = 10,
        .onResult = [](const PollItem& it,
                       const std::vector<uint16_t>& v,
                       const ModbusError*) {
            std::print("  {} = {} {}\n", it.name, v[0], v[1]);
        },
        .name = "pair"});

    (void)poller.start();
    std::print("polling 1.5 s (pv: changedOnly+epsilon5 @50ms, "
               "pair @200ms)\n");
    std::this_thread::sleep_for(1500ms);
    poller.stop();
    std::print("stats: polls={} errors={} changes={}\n",
               poller.stats.polls.load(), poller.stats.errors.load(),
               poller.stats.changes.load());

    // ---- WriteCache ----------------------------------------------------------
    std::print("\n== WriteCache\n");
    WriteCache wc(cli);
    wc.seed(20, 0);      // inform cache of known device state
    wc.set(20, 10);      // dirty: 0 -> 10
    wc.set(21, 11);      // dirty, adjacent -> coalesces into one FC16
    wc.set(23, 33);      // dirty, gap at 22 -> same run (gap filled)
    wc.set(40, 1);       // separate run
    wc.setCoil(2, true);
    std::print("dirty regs={} coils={} runs:\n", wc.dirtyRegs(),
               wc.dirtyCoils());
    for (auto& [a, v] : wc.dirtyRuns())
        std::print("  hr {}..{} = {} vals\n", a, a + v.size() - 1,
                   v.size());
    if (auto e = wc.flush(); !e)
        std::print("flush err: {}\n", e.error().format());
    auto r20 = cli.readHolding(20, 4);
    if (r20)
        std::print("after flush hr20..23 = {} {} {} {}\n",
                   (*r20)[0], (*r20)[1], (*r20)[2], (*r20)[3]);
    std::print("dirty after flush: {}\n", wc.dirtyRegs());

    run.store(false); pv.join();
    return 0;
}
