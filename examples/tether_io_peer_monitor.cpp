/**
 * @file tether_io_peer_monitor.cpp
 * @brief MonitoredRemoteFunctionInvocation: serialized async peer calls
 *        with tick-stamped freshness — the "remote stage" primitive.
 *
 * Wraps an invoker (here Server::callPeerFunction) so a fast loop can
 * treat a peer function like a pipeline stage without blocking:
 *
 *   - one call in flight; call() during a call returns false
 *   - results arrive stamped with the issuing tick; fresh(tick)
 *     classifies them against a max-age window
 *   - issued/completed/timeouts/errors are counted separately
 *
 * A slow peer (~3 ms) is called from a simulated 1 kHz loop: results
 * land several ticks after issue, so maxAgeTicks decides applied-vs-
 * fallback. Shrinking the window to 0 shows the degenerate case — an
 * async result can never be same-tick-fresh.
 *
 *   ./tether_io_peer_monitor [--port 42787]
 */

#include "tether/io/MonitoredRemoteFunctionInvocation.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    uint16_t port = 42787;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) { server.stop(); return 1; }
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);

    // Client-hosted peer: 2*x after ~3 ms of work.
    FunctionEntry dbl;
    dbl.id = 500;
    dbl.name = "dbl";
    dbl.description = "returns 2*x after 3 ms";
    dbl.group = "peer";
    dbl.parameters = {{"x", "input", ValueType::F64}};
    dbl.returnValue.present = true;
    dbl.returnValue.name = "y";
    dbl.returnValue.type = ValueType::F64;
    dbl.callback = [](const std::vector<FunctionArgument>& args) {
        FunctionCallResult r;
        double x = 0;
        for (const auto& a : args)
            if (a.position == 0 && a.value.size() == 8)
                std::memcpy(&x, a.value.data(), 8);
        std::this_thread::sleep_for(3ms);
        const double y = 2.0 * x;
        r.success = true;
        r.returnValue.resize(8);
        std::memcpy(r.returnValue.data(), &y, 8);
        return r;
    };
    if (!client.registerFunctions({dbl})) { server.stop(); return 1; }

    std::atomic<bool> pumpOn{true};
    std::thread pump([&] {
        while (pumpOn) {
            (void)client.receiveMessage(50);
            std::this_thread::sleep_for(1ms);
        }
    });

    // Monitor: invoke via callPeerFunction, decode F64 payload.
    MonitoredRemoteFunctionInvocation<double> mon(
        [&](std::vector<uint8_t> arg, uint64_t timeoutUs, auto done) {
            FunctionArgument a;
            a.position = 0;
            a.type = ValueType::F64;
            a.value = std::move(arg);
            return server.callPeerFunction(
                "dbl", {a}, timeoutUs,
                [done](const Session::InvokeResult& r) {
                    MonitoredRemoteFunctionInvocation<double>::Outcome o;
                    o.timedOut = r.timedOut;
                    o.success = r.success;
                    o.hasReturnValue = r.hasReturnValue;
                    o.returnValue = r.returnValue;
                    done(o);
                });
        },
        [](const uint8_t* d, size_t n, double& out) {
            if (n != 8) return false;
            std::memcpy(&out, d, 8);
            return true;
        },
        /*maxAgeTicks=*/20, /*timeoutUs=*/100'000);

    // Simulated 1 kHz loop: arg is the tick as F64; remote result 2*tick
    // replaces the local value (tick) only while fresh.
    int remote = 0, local = 0, skipped = 0;
    for (uint64_t tick = 0; tick < 60; ++tick) {
        std::vector<uint8_t> arg(8);
        const double x = static_cast<double>(tick);
        std::memcpy(arg.data(), &x, 8);
        if (!mon.call(tick, std::move(arg))) ++skipped;
        if (mon.fresh(tick)) {
            const double y = mon.result().payload;
            (void)y;
            ++remote;
        } else {
            ++local;
        }
        std::this_thread::sleep_for(1ms);
    }
    // Drain the last in-flight call.
    for (int i = 0; i < 200 && mon.inFlight(); ++i)
        std::this_thread::sleep_for(5ms);

    std::printf("loop(60 ticks): remote-applied=%d local-fallback=%d "
                "call-skipped(in-flight)=%d\n",
                remote, local, skipped);
    std::printf("counters: issued=%llu completed=%llu timeouts=%llu "
                "errors=%llu\n",
                static_cast<unsigned long long>(mon.issued()),
                static_cast<unsigned long long>(mon.completed()),
                static_cast<unsigned long long>(mon.timeouts()),
                static_cast<unsigned long long>(mon.errors()));

    // Degenerate window: async results can never be same-tick-fresh.
    mon.setMaxAgeTicks(0);
    const bool everFresh = mon.fresh(mon.producerTick() + 1);
    std::printf("maxAgeTicks=0: fresh(one tick after issue)=%d — async "
                "results are never same-tick fresh\n", everFresh);

    pumpOn = false;
    pump.join();
    server.stop();
    return (remote > 0 && local > 0 && mon.completed() > 0 && !everFresh)
               ? 0 : 1;
}
