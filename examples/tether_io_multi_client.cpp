/**
 * @file tether_io_multi_client.cpp
 * @brief Concurrent sessions on one server: per-session peer catalogs
 *        and the server-side liveness view.
 *
 * Every accepted connection gets its own Session with a session-scoped
 * peer-function catalog. The server sees all of them at once:
 * activeSessionCount() counts live sessions, peerFunctions() lists
 * every callable peer entry, and callPeerFunction() routes an
 * invocation to whichever session hosts the name. A session's peer
 * catalog dies with its connection.
 *
 *   ./tether_io_multi_client
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

using namespace tether::io;

namespace {
FunctionEntry makeEcho(uint64_t id, const char* name, double tag) {
    FunctionEntry f;
    f.id = id;
    f.name = name;
    f.returnValue.present = true;
    f.returnValue.type = ValueType::F64;
    f.callback = [tag](const std::vector<FunctionArgument>&) {
        FunctionCallResult r;
        r.success = true;
        r.returnValue.resize(8);
        std::memcpy(r.returnValue.data(), &tag, 8);
        return r;
    };
    return f;
}

// Synchronous wrapper for the async callPeerFunction callback.
Session::InvokeResult callPeer(Server& server, const char* name) {
    Session::InvokeResult result;
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    const bool queued = server.callPeerFunction(
        name, {}, 2'000'000, [&](const Session::InvokeResult& r) {
            {
                std::lock_guard lk(mtx);
                result = r;
                done = true;
            }
            cv.notify_one();
        });
    if (!queued) return result;
    std::unique_lock lk(mtx);
    cv.wait_for(lk, std::chrono::seconds(3), [&] { return done; });
    return result;
}
} // namespace

int main() {
    Registry registry;

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(42788, "127.0.0.1"),
                  sc);
    if (!server.start()) return 1;

    // Two clients, each hosting a distinct peer function. A client
    // answers invokes only while receiveMessage() is being pumped.
    auto clientA = std::make_unique<TetherIOClient>(
        std::make_unique<SlipFramedTransport>(
            tcpConnect("127.0.0.1", 42788, 2000)),
        2000);
    auto clientB = std::make_unique<TetherIOClient>(
        std::make_unique<SlipFramedTransport>(
            tcpConnect("127.0.0.1", 42788, 2000)),
        2000);
    if (!clientA->registerFunctions({makeEcho(500, "a.ping", 1.0)}) ||
        !clientB->registerFunctions({makeEcho(501, "b.ping", 2.0)})) {
        std::fprintf(stderr, "registerFunctions failed\n");
        return 1;
    }
    std::atomic<bool> pumpAon{true}, pumpBon{true};
    auto pump = [](TetherIOClient* c, std::atomic<bool>& on) {
        while (on) (void)c->receiveMessage(50);
    };
    std::thread pumpA(pump, clientA.get(), std::ref(pumpAon));
    std::thread pumpB(pump, clientB.get(), std::ref(pumpBon));

    // Registration propagates asynchronously — wait for both to appear.
    for (int i = 0; i < 100 && server.peerFunctions().size() < 2; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::printf("[server] sessions=%zu peers=%zu\n",
                server.activeSessionCount(), server.peerFunctions().size());
    for (const auto& [name, key] : server.peerFunctions())
        std::printf("         %s (id=%llu)\n", name.c_str(),
                    static_cast<unsigned long long>(key));

    // The server routes each call to the owning session.
    const auto ra = callPeer(server, "a.ping");
    const auto rb = callPeer(server, "b.ping");
    if (!ra.success || !rb.success || ra.returnValue.size() != 8 ||
        rb.returnValue.size() != 8)
        return 1;
    double ta, tb;
    std::memcpy(&ta, ra.returnValue.data(), 8);
    std::memcpy(&tb, rb.returnValue.data(), 8);
    std::printf("[server] a.ping -> tag=%.1f (served by client A)\n", ta);
    std::printf("[server] b.ping -> tag=%.1f (served by client B)\n", tb);

    // B's session-scoped catalog dies with its connection.
    clientB->close();
    pumpBon = false;
    pumpB.join();
    for (int i = 0; i < 200 && server.peerFunctions().size() > 1; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::printf("[server] after B disconnect: sessions=%zu peers=%zu\n",
                server.activeSessionCount(), server.peerFunctions().size());
    for (const auto& [name, key] : server.peerFunctions())
        std::printf("         %s\n", name.c_str());

    // The gone provider is unreachable.
    const auto rg = callPeer(server, "b.ping");

    const bool ok = server.peerFunctions().size() == 1 &&
                    server.peerFunctions()[0].first == "a.ping" &&
                    !rg.success;
    pumpAon = false;
    clientA->close();
    pumpA.join();
    server.stop();
    return ok ? 0 : 1;
}
