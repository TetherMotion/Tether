/**
 * @file tether_io_peer_failover.cpp
 * @brief Duplicate providers: health-scored dispatch and failover.
 *
 * When several connected clients host the same function name,
 * Server::callPeerFunction picks the session with the fewest
 * consecutive invoke timeouts:
 *
 *   - a responsive peer (0 timeouts) always wins over a stalling one
 *   - a peer that stops answering accumulates timeouts and loses the
 *     next call to its healthier duplicate
 *   - a disconnected peer is skipped outright
 *
 * Demo: two clients host `work`, identified by their return value
 * (A->1, B->2). A starts hanging its handler; one call dies against
 * A's deadline, then B takes over. Disconnecting B falls back to A —
 * which still gets calls (a *running* peer is never excluded, only
 * scored down) and keeps timing out.
 *
 *   ./tether_io_peer_failover [--port 42788]
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
#include <memory>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

namespace {

FunctionEntry makeWork(double tag, std::atomic<int>* hangMs) {
    FunctionEntry fn;
    fn.id = 500;
    fn.name = "work";
    fn.group = "peer";
    fn.returnValue.present = true;
    fn.returnValue.name = "who";
    fn.returnValue.type = ValueType::F64;
    fn.callback = [tag, hangMs](const std::vector<FunctionArgument>&) {
        const int hang = hangMs->load();
        if (hang > 0) std::this_thread::sleep_for(
            std::chrono::milliseconds(hang));
        FunctionCallResult r;
        r.success = true;
        r.returnValue.resize(8);
        std::memcpy(r.returnValue.data(), &tag, 8);
        return r;
    };
    return fn;
}

struct Peer {
    std::unique_ptr<TetherIOClient> client;
    std::shared_ptr<std::atomic<int>> hangMs{
        std::make_shared<std::atomic<int>>(0)};
    std::atomic<bool> pumpOn{true};
    std::thread pump;
};

std::unique_ptr<Peer> connectPeer(uint16_t port, double tag) {
    auto p = std::make_unique<Peer>();
    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) return {};
    p->client = std::make_unique<TetherIOClient>(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);
    if (!p->client->registerFunctions({makeWork(tag, p->hangMs.get())})) {
        return {};
    }
    TetherIOClient* c = p->client.get();
    p->pump = std::thread([c, &on = p->pumpOn] {
        while (on) {
            (void)c->receiveMessage(50);
            std::this_thread::sleep_for(1ms);
        }
    });
    return p;
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42788;
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

    auto a = connectPeer(port, 1.0);
    auto b = connectPeer(port, 2.0);
    if (!a || !b) { server.stop(); return 1; }
    std::printf("providers: %zu\n", server.peerFunctions().size());

    int rc = 0;
    auto call = [&](const char* phase) {
        std::atomic<bool> done{false};
        double who = -1;
        bool timedOut = true;
        if (!server.callPeerFunction(
                "work", {}, /*deadlineUs=*/120'000,
                [&](const Session::InvokeResult& r) {
                    timedOut = r.timedOut || !r.success;
                    if (r.hasReturnValue && r.returnValue.size() == 8)
                        std::memcpy(&who, r.returnValue.data(), 8);
                    done = true;
                })) {
            std::printf("%-22s -> no provider\n", phase);
            rc = 1;
            return;
        }
        for (int i = 0; i < 200 && !done; ++i)
            std::this_thread::sleep_for(5ms);
        std::printf("%-22s -> %s\n", phase,
                    timedOut ? "TIMEOUT"
                             : (who == 1.0 ? "served by A"
                                : who == 2.0 ? "served by B" : "??"));
    };

    call("healthy");
    *a->hangMs = 500;              // A stalls (>> 120 ms deadline)
    call("A stalls: burns 120ms");  // dispatched to A -> times out
    call("failover");             // A has 1 timeout -> B (0) wins
    call("still B");
    b->pumpOn = false;
    if (b->pump.joinable()) b->pump.join();
    b->client.reset();            // disconnect B entirely
    std::this_thread::sleep_for(100ms);
    call("B gone, A still hung"); // A is the only provider -> timeout

    a->pumpOn = false;
    if (a->pump.joinable()) a->pump.join();
    server.stop();
    return rc;
}
