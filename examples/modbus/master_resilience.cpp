// modbus_master_resilience — error-handling strategies on a flaky RTU bus.
//   ErrorPolicy is applied by the bus executor (the "connection"); a
//   SimLink produces real RTU frames with injectable faults, so this is
//   the full retry engine — no hardware needed.
//
//   Demonstrates:
//     - default policy: retry every failure cause, max 8 retries
//     - TransactionOptions layering: connection defaults -> client
//       defaults -> per-call overrides (field-wise, sentinel -1 = inherit)
//     - retries=0 to disable, per-cause toggles, exponential backoff,
//       fail-fast, circuit breaker
#include <chrono>
#include <print>
#include <thread>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ErrorPolicy.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using namespace std::chrono_literals;

static void show(std::string_view tag,
                 const Expected<std::vector<uint16_t>>& r) {
    if (r) std::print("  {:<34} ok [{} {}]\n", tag, (*r)[0], (*r)[1]);
    else   std::print("  {:<34} err {}\n", tag, r.error().format());
}

int main() {
    // ---- bus: one slave, link-level fault injection -------------------------
    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 16);
    dev.preload(0, {42, 43});

    auto link = std::make_unique<SimLink>(bus, /*rtu*/true);
    auto& faults = link->traits();            // SimLinkTraits
    auto ch = std::make_unique<RtuChannel>(
        std::move(link), RtuTiming::forParams(1000000, 10));
    if (auto e = ch->open(); !e) {
        std::print(stderr, "open: {}\n", e.error().format());
        return 1;
    }

    BusExecutor ex(*ch);
    ModbusClient cli(ex, 1);

    // Connection defaults: short timeouts so the demo runs fast.
    ex.setDefaultOptions(TransactionOptions{
        .timeoutMs = 30, .retryDelayMs = 2});

    // ---- 1) default policy: retry-any-cause x8 -------------------------------
    std::print("== default policy (Retry x8, all causes)\n");
    faults.corruptByte = 0.5;                 // ~50% CRC corruption
    faults.dropResponse = 0.2;                // ~20% silence
    show("50% corrupt + 20% drop", cli.readHolding(0, 2));

    // ---- 2) connection-level retry callback (inherited) ----------------------
    TransactionOptions d = ex.defaultOptions();
    d.onRetry = [](int a, const ModbusError& e) {
        std::print("      retry#{} ({})\n", a, e.format());
    };
    ex.setDefaultOptions(d);
    show("same faults, retries visible", cli.readHolding(0, 2));

    // ---- 3) per-call override: retries=0 disables, errorPolicy replaces ------
    std::print("== per-call overrides (fields merge; unset -> inherit)\n");
    show("retries=0 -> single attempt",
         cli.readHolding(0, 2, TransactionOptions{.retries = 0}));
    show("timeout only overridden",
         cli.readHolding(0, 2, TransactionOptions{.timeoutMs = 10}));
    TransactionOptions ff;
    ff.errorPolicy = ErrorPolicy{.strategy = ErrorStrategy::FailFast};
    show("FailFast (policy replaced)", cli.readHolding(0, 2, ff));

    // ---- 4) per-cause toggles: retry corruption but not silence --------------
    std::print("== per-cause toggles\n");
    ErrorPolicy pc;                            // defaults: retry all
    pc.retryOnTimeout = false;                 // silence -> fail, no retry
    pc.retryOnCorrupt = true;
    ex.setErrorPolicy(pc);
    faults.corruptByte = 0.0; faults.dropResponse = 1.0;   // pure silence
    show("silence, retryOnTimeout=false", cli.readHolding(0, 2));

    // ---- 5) exponential backoff ----------------------------------------------
    std::print("== RetryBackoff (5ms -> 10 -> 20 -> 40)\n");
    faults.dropResponse = 0.0; faults.corruptByte = 0.7;
    ex.setErrorPolicy(ErrorPolicy{
        .strategy = ErrorStrategy::RetryBackoff, .maxRetries = 4,
        .backoffBaseMs = 5, .backoffMaxMs = 40});
    auto t0 = std::chrono::steady_clock::now();
    show("backoff, 70% corrupt", cli.readHolding(0, 2));
    std::print("      elapsed {} ms (delays between attempts)\n",
               std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0).count());

    // ---- 6) circuit breaker -----------------------------------------------------
    std::print("== CircuitBreaker (opens after 3 failed calls)\n");
    ex.setErrorPolicy(ErrorPolicy{
        .strategy = ErrorStrategy::CircuitBreaker, .maxRetries = 0,
        .circuitThreshold = 3, .circuitResetMs = 300});
    faults.corruptByte = 0.0; faults.dropResponse = 1.0;   // dead bus
    for (int i = 0; i < 5; ++i)
        show(std::format("call {}", i), cli.readHolding(0, 2));
    std::print("      (calls 3,4 fail instantly — circuit open)\n");
    std::this_thread::sleep_for(350ms);         // reset window
    faults.dropResponse = 0.0;                  // bus back: half-open ok
    show("after reset window", cli.readHolding(0, 2));
    return 0;
}
