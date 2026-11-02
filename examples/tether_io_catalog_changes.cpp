/**
 * @file tether_io_catalog_changes.cpp
 * @brief Dynamic catalogs: runtime add + CatalogChanged push.
 *
 * Registry::addParam/addSignal/addFunction work while sessions are
 * live: each mutation bumps registry.revision() and fires every
 * session's change listener, which pushes CatalogChanged (0x2C) to the
 * client. The client sees it inside receiveMessage()/request() waits
 * via setCatalogChangedCallback — its signal to re-run the list APIs;
 * the new entry is immediately callable.
 *
 *   ./tether_io_catalog_changes [--port 42796]
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
    uint16_t port = 42796;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    {
        ParamEntry p;
        p.id = 1;
        p.name = "static.param";
        p.group = "demo";
        p.valueType = ValueType::U32;
        p.readFn = [](void* d) { const uint32_t v = 7; std::memcpy(d, &v, 4); };
        if (!registry.addParam(p)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    int rc = 0;
    {
        auto transport = tcpConnect("127.0.0.1", port, 2000);
        if (!transport) rc = 1;
        if (rc == 0) {
            TetherIOClient client(
                std::make_unique<SlipFramedTransport>(std::move(transport)),
                2000);

            std::atomic<uint32_t> changes{0};
            client.setCatalogChangedCallback(
                [&] { ++changes; });

            const auto before = client.listSignals();
            std::printf("[client] signals before: %zu (registry rev %u)\n",
                        before ? before->size() : 0, registry.revision());

            // Mutate the catalog while the session is live.
            {
                SignalEntry s;
                s.id = 42;
                s.name = "dyn.temp";
                s.description = "Added at runtime";
                s.group = "demo";
                s.valueType = ValueType::F64;
                s.readFn = [](void* d) {
                    const double v = 21.5;
                    std::memcpy(d, &v, 8);
                };
                if (!registry.addSignal(s)) { rc = 1; }
            }
            std::printf("[server] added dyn.temp (registry rev %u)\n",
                        registry.revision());

            // Pump until the push arrives.
            for (int i = 0; i < 40 && changes == 0; ++i)
                (void)client.receiveMessage(25);

            const auto after = client.listSignals();
            std::printf("[client] catalogChanged=%u; signals after: %zu\n",
                        changes.load(), after ? after->size() : 0);
            if (changes == 0 || !after || after->size() != 1) rc = 1;

            // The new entry is immediately live.
            if (rc == 0) {
                const auto v = client.getSignal(42);
                double d = 0;
                if (v && v->size() == 8)
                    std::memcpy(&d, v->data(), 8);
                std::printf("[client] dyn.temp = %.2f\n", d);
                if (d != 21.5) rc = 1;
            }
        }
    }
    server.stop();
    return rc;
}
