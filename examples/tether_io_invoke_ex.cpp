/**
 * @file tether_io_invoke_ex.cpp
 * @brief InvokeEx: correlated calls with a relative deadline budget.
 *
 * InvokeEx differs from CallFunction in two places:
 *
 *   - The request carries requestId + deadlineUs. The deadline reaches
 *     the callee through FunctionInvokeContext (contextualCallback) —
 *     a cooperative hint, since the session does not preempt running
 *     functions.
 *   - On the *invoker* side the deadline is enforced: the session sweeps
 *     pending calls and fires the callback with timedOut=true at expiry;
 *     a response that crosses the wire late is dropped. This is the
 *     callPeerFunction() path — e.g. a 1 kHz loop can bound a plug-in
 *     call without stalling.
 *
 * Demo: a server function echoes the deadline it saw; a slow
 * client-hosted peer is then called once with a 50 ms budget (times out)
 * and once with none (completes in ~150 ms).
 *
 *   ./tether_io_invoke_ex [--port 42785]
 */

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
    uint16_t port = 42785;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    {
        FunctionEntry fn;
        fn.id = 1;
        fn.name = "deadline_echo";
        fn.description = "Returns the deadlineUs the caller sent";
        fn.group = "demo";
        fn.returnValue.present = true;
        fn.returnValue.name = "deadline_us";
        fn.returnValue.type = ValueType::U64;
        fn.contextualCallback =
            [](const std::vector<FunctionArgument>&,
               const FunctionInvokeContext& ctx) {
                FunctionCallResult r;
                r.success = true;
                r.returnValue.resize(8);
                std::memcpy(r.returnValue.data(), &ctx.deadlineUs, 8);
                return r;
            };
        if (!registry.addFunction(fn)) return 1;
    }

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

    auto deadlineOf = [](const auto& res) -> uint64_t {
        uint64_t v = UINT64_MAX;
        if (res && res->success && res->returnValue.size() == 8)
            std::memcpy(&v, res->returnValue.data(), 8);
        return v;
    };
    std::printf("[client] callFunction -> deadline seen by callee: %llu\n",
                static_cast<unsigned long long>(
                    deadlineOf(client.callFunction(1))));
    std::printf("[client] invokeEx(75 ms) -> deadline seen: %llu\n",
                static_cast<unsigned long long>(
                    deadlineOf(client.invokeEx(1, {}, 75'000))));

    // Client-hosted slow function: 150 ms of work, returns a constant.
    FunctionEntry slow;
    slow.id = 500;
    slow.name = "slow";
    slow.description = "Sleeps 150 ms, returns 42";
    slow.group = "peer";
    slow.returnValue.present = true;
    slow.returnValue.name = "y";
    slow.returnValue.type = ValueType::U64;
    slow.callback = [](const std::vector<FunctionArgument>&) {
        std::this_thread::sleep_for(150ms);
        FunctionCallResult r;
        r.success = true;
        r.returnValue.resize(8);
        const uint64_t y = 42;
        std::memcpy(r.returnValue.data(), &y, 8);
        return r;
    };
    if (!client.registerFunctions({slow})) { server.stop(); return 1; }

    // The client's InvokeEx dispatch runs inside receiveMessage() waits.
    std::atomic<bool> pumpOn{true};
    std::thread pump([&] {
        while (pumpOn) {
            (void)client.receiveMessage(50);
            std::this_thread::sleep_for(1ms);
        }
    });

    int rc = 0;
    auto callSlow = [&](uint64_t deadlineUs) {
        std::atomic<bool> done{false};
        Session::InvokeResult res;
        if (!server.callPeerFunction("slow", {}, deadlineUs,
                                     [&](const Session::InvokeResult& r) {
                                         res = r;
                                         done = true;
                                     })) {
            std::fprintf(stderr, "callPeerFunction: 'slow' not found\n");
            rc = 1;
            return;
        }
        for (int i = 0; i < 300 && !done; ++i)
            std::this_thread::sleep_for(5ms);
        if (!done) { std::fprintf(stderr, "no result\n"); rc = 1; return; }
        std::printf("[server] deadline=%lluus -> success=%d timedOut=%d "
                    "rtt=%lluus%s\n",
                    static_cast<unsigned long long>(deadlineUs), res.success,
                    res.timedOut, static_cast<unsigned long long>(res.rttUs),
                    res.timedOut ? "" : " (late reply dropped on timeout)");
        // Expect: 50 ms budget -> timedOut; 0 -> success.
        if (deadlineUs != 0 && !res.timedOut) rc = 1;
        if (deadlineUs == 0 && !res.success) rc = 1;
    };

    callSlow(50'000);   // budget < work -> timedOut at ~50 ms
    callSlow(0);        // unbounded -> success at ~150 ms

    pumpOn = false;
    pump.join();
    server.stop();
    return rc;
}
