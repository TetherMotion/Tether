/**
 * @file tether_io_peer_function.cpp
 * @brief Client-hosted processing function (peer RPC) over Tether IO.
 *
 * A Tether IO client can publish functions to the server via
 * RegisterFunctionsReq; the server then invokes them back over the same
 * connection (InvokeExReq) — the "remote processing" pattern used for
 * hosted torque filters and estimators. Invocation dispatch runs inside
 * TetherIOClient::receiveMessage(), so a client without a receive pump
 * never answers calls.
 *
 * This example is self-contained: it brings up a minimal in-process
 * Server on a TCP port (SLIP framing, matching deployed setups), then
 * connects a client that hosts `scale(x: F64) -> F64`. The server calls
 * it via callPeerFunction() and prints the round trip.
 *
 *   ./tether_io_peer_function [--port 42777]
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

namespace {

/// The function this example hosts on the *client*: scale(x) -> 2x.
FunctionEntry makeScale(uint64_t id) {
    FunctionEntry fn;
    fn.id = id;
    fn.name = "scale";
    fn.description = "Client-hosted F64 processing: returns 2*x";
    fn.group = "peer";
    fn.parameters = {{"x", "Input", ValueType::F64}};
    fn.returnValue.present = true;
    fn.returnValue.name = "y";
    fn.returnValue.type = ValueType::F64;
    fn.callback = [](const std::vector<FunctionArgument>& args) {
        FunctionCallResult result;
        double x = 0;
        for (const auto& arg : args) {
            if (arg.position == 0 && arg.value.size() == 8)
                std::memcpy(&x, arg.value.data(), 8);
        }
        const double y = 2.0 * x;
        result.success = true;
        result.returnValue.resize(8);
        std::memcpy(result.returnValue.data(), &y, 8);
        std::printf("[client] scale(%g) -> %g\n", x, y);
        return result;
    };
    return fn;
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42777;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
    }

    // Minimal server: empty registry, SLIP framing on loopback TCP.
    Registry registry;
    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"), sc);
    if (!server.start()) {
        std::fprintf(stderr, "server start failed (port %u in use?)\n", port);
        return 1;
    }

    // Client: TCP connect, SLIP-frame it, run the V6 handshake in the ctor.
    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) {
        std::fprintf(stderr, "connect 127.0.0.1:%u failed\n", port);
        return 1;
    }
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);

    // Publish the client-hosted function catalog, then pump receiveMessage()
    // so incoming InvokeExReq messages dispatch to the callback above.
    auto count = client.registerFunctions({makeScale(500)});
    if (!count) {
        std::fprintf(stderr, "registerFunctions: %s\n",
                     count.error().message.c_str());
        return 1;
    }
    std::printf("[client] registered %u function(s); server sees %zu peer fn(s)\n",
                *count, server.peerFunctions().size());

    std::atomic<bool> pumpRunning{true};
    std::thread pump([&] {
        while (pumpRunning) {
            (void)client.receiveMessage(50);
            // receiveMessage() holds the client mutex during its wait —
            // yield so concurrent request() calls are not starved.
            std::this_thread::sleep_for(1ms);
        }
    });

    // Server -> client invocation. Arguments are keyed TLVs; the callback
    // carries the correlated result (return bytes are the callee's encoding).
    std::atomic<bool> done{false};
    FunctionArgument arg;
    arg.position = 0;
    arg.type = ValueType::F64;
    arg.value.resize(8);
    const double x = 3.5;
    std::memcpy(arg.value.data(), &x, 8);
    const bool queued = server.callPeerFunction(
        "scale", {arg}, /*deadlineUs=*/2'000'000,  // relative budget
        [&](const Session::InvokeResult& res) {
            if (res.success && res.hasReturnValue) {
                double y = 0;
                std::memcpy(&y, res.returnValue.data(), 8);
                std::printf("[server] scale(3.5) answered %g in %llu us\n", y,
                            static_cast<unsigned long long>(res.rttUs));
            } else {
                std::printf("[server] call failed: success=%d timedOut=%d\n",
                            res.success, res.timedOut);
            }
            done = true;
        });
    if (!queued) {
        std::fprintf(stderr, "callPeerFunction: peer 'scale' not found\n");
        pumpRunning = false;
        pump.join();
        return 1;
    }

    for (int i = 0; i < 200 && !done; ++i) std::this_thread::sleep_for(10ms);
    pumpRunning = false;
    pump.join();
    server.stop();
    return done ? 0 : 1;
}
