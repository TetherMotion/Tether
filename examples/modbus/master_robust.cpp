// modbus_master_robust — read/write assurance strategies on a flaky bus.
//   RobustInvoker decorates any IModbusInvoker; the strategy is chosen
//   per endpoint, so different registers can get different assurance.
//   All strategies run on SimBus — no hardware needed.
#include <print>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/robust/RobustInvoker.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;

static void show(std::string_view tag,
                 const Expected<std::vector<uint16_t>>& r) {
    if (r) {
        std::print("  {:<30} ok [", tag);
        for (auto v : *r) std::print(" {}", v);
        std::print(" ]\n");
    } else {
        std::print("  {:<30} err {}\n", tag, r.error().format());
    }
}

int main() {
    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 64);
    dev.preload(0, {100, 200, 300, 400});
    dev.traits().corrupt = 0.25;      // 1-in-4 responses mangled

    auto tryRead = [&](std::string_view name, Strategy s,
                       AssuranceOpts extra = {}) {
        extra.strategy = s;
        RobustInvoker inv(bus.invoker(), extra);
        ModbusClient c(inv, 1);
        show(name, c.readHolding(0, 4));
    };

    std::print("== read strategies (25% PDU corruption)\n");
    tryRead("direct", Strategy::Direct);
    tryRead("twice, error on mismatch",
            Strategy::TwiceErrorOnMismatch);
    tryRead("twice, reread on mismatch",
            Strategy::TwiceRereadOnMismatch);
    tryRead("thrice majority vote", Strategy::ThriceMajorityVote);
    tryRead("n-quorum 4/3", Strategy::NQuorum, {.n = 4, .quorum = 3});
    tryRead("median of 5", Strategy::Median, {.n = 5});
    tryRead("until stable", Strategy::UntilStable, {.n = 6});
    tryRead("sticky (stale on fail)", Strategy::Sticky,
            {.allowStale = true});

    // ---- assured writes ------------------------------------------------------
    std::print("== write strategies\n");
    {
        RobustInvoker inv(bus.invoker(),
                          AssuranceOpts{.strategy = Strategy::WriteConfirm});
        auto wb = build::writeSingleReg(9, 0x77);
        auto rb = build::readRange(FunctionCode::ReadHoldingRegisters, 9, 1);
        auto r = inv.assuredWrite(1, FunctionCode::WriteSingleRegister,
                                  wb->body(),
                                  FunctionCode::ReadHoldingRegisters,
                                  rb->body(),
                                  std::vector<uint8_t>{0x02, 0x00, 0x77});
        std::print("  {:<30} {}\n", "write+confirm hr9=0x77",
                   r ? "confirmed" : r.error().format());
    }
    {
        // Watchdog: confirm after a settle delay (device applies slowly)
        RobustInvoker inv(bus.invoker(), AssuranceOpts{
            .strategy = Strategy::WriteWatchdog, .confirmDelayMs = 20});
        auto wb = build::writeSingleReg(10, 0x55);
        auto rb = build::readRange(FunctionCode::ReadHoldingRegisters,
                                   10, 1);
        auto r = inv.assuredWrite(1, FunctionCode::WriteSingleRegister,
                                  wb->body(),
                                  FunctionCode::ReadHoldingRegisters,
                                  rb->body(),
                                  std::vector<uint8_t>{0x02, 0x00, 0x55});
        std::print("  {:<30} {}\n", "write+watchdog(20ms)",
                   r ? "confirmed" : r.error().format());
    }

    // ---- assurance statistics --------------------------------------------------
    auto& s = RobustInvoker::stats();
    std::print("\nstats: ops={} samples={} mismatches={} votesWon={} "
               "votesLost={} staleServed={} confirms={} confirmFail={}\n",
               s.ops.load(), s.samples.load(), s.mismatches.load(),
               s.votesWon.load(), s.votesLost.load(), s.staleServed.load(),
               s.confirms.load(), s.confirmFailures.load());
    return 0;
}
