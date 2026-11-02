/**
 * @file tether_io_metadata.cpp
 * @brief Entry metadata: units, ranges and notes carried by the catalog.
 *
 * ParamEntry/SignalEntry/FunctionEntry all take a string->string
 * `metadata` map — free-form annotations that travel with the catalog
 * but do not appear in list calls. getMetadata(id) fetches them on
 * demand: units for display, ranges for UI bounds, semantic tags a
 * client can branch on.
 *
 *   ./tether_io_metadata [--port 42794]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <cstring>
#include <cstdio>
#include <string>

using namespace tether::io;

int main(int argc, char** argv) {
    uint16_t port = 42794;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    {
        ParamEntry p;
        p.id = 1;
        p.name = "pid.kp";
        p.description = "Proportional gain";
        p.group = "pid.axis0";
        p.valueType = ValueType::F64;
        p.readFn = [](void* d) { const double v = 1.5; std::memcpy(d, &v, 8); };
        p.writeFn = [](const void*) {};
        p.metadata["unit"] = "Nm/rad";
        p.metadata["range"] = "0..50";
        p.metadata["tunable"] = "online";
        if (!registry.addParam(p)) return 1;
    }
    {
        SignalEntry s;
        s.id = 2;
        s.name = "drive.current";
        s.group = "drive";
        s.valueType = ValueType::F64;
        s.readFn = [](void* d) { const double v = 3.2; std::memcpy(d, &v, 8); };
        s.metadata["unit"] = "A";
        s.metadata["phase"] = "q";
        if (!registry.addSignal(s)) return 1;
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
            for (uint64_t id : {1ULL, 2ULL, 99ULL}) {
                const auto md = client.getMetadata(id);
                if (!md) {
                    std::printf("[client] id=%llu -> %s\n",
                                static_cast<unsigned long long>(id),
                                md.error().message.c_str());
                    continue;
                }
                std::printf("[client] id=%llu ->", static_cast<unsigned long long>(id));
                for (const auto& m : *md)
                    std::printf(" %s=%s", m.key.c_str(), m.value.c_str());
                std::printf("\n");
            }
        }
    }
    server.stop();
    return rc;
}
