/**
 * @file tether_io_snapshots.cpp
 * @brief SnapshotParams / SnapshotSignals: one-timestamp multi-reads.
 *
 * getParam/getSignal read entries one round-trip at a time — values from
 * different requests carry unrelated timestamps. The snapshot calls read
 * a whole id list in one request and stamp every value with the same
 * server timestamp: a consistent cut across the catalog. An empty id
 * list snapshots everything.
 *
 *   ./tether_io_snapshots [--port 42791]
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
    uint16_t port = 42791;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    std::atomic<double> pos{0.0}, vel{0.0}, torque{0.0};
    Registry registry;
    auto addSignal = [&](uint64_t id, const char* name,
                         std::atomic<double>& src) {
        SignalEntry s;
        s.id = id;
        s.name = name;
        s.group = "drive";
        s.valueType = ValueType::F64;
        s.readFn = [&src](void* d) {
            const double v = src.load();
            std::memcpy(d, &v, 8);
        };
        return registry.addSignal(s);
    };
    if (!addSignal(1, "drive.pos", pos) ||
        !addSignal(2, "drive.vel", vel) ||
        !addSignal(3, "drive.torque", torque))
        return 1;
    {
        ParamEntry p;
        p.id = 10;
        p.name = "drive.enabled";
        p.group = "drive";
        p.valueType = ValueType::U8;
        p.readFn = [](void* d) { *static_cast<uint8_t*>(d) = 1; };
        p.writeFn = [](const void*) {};
        if (!registry.addParam(p)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    // Move the "axis" so individual reads would catch different samples.
    std::atomic<bool> run{true};
    std::thread producer([&] {
        double t = 0;
        while (run) {
            t += 0.001;
            pos.store(t);
            vel.store(1.0);
            torque.store(t * 0.5);
            std::this_thread::sleep_for(1ms);
        }
    });

    int rc = 0;
    {
        auto transport = tcpConnect("127.0.0.1", port, 2000);
        if (!transport) rc = 1;
        if (rc == 0) {
            TetherIOClient client(
                std::make_unique<SlipFramedTransport>(std::move(transport)),
                2000);
            std::this_thread::sleep_for(100ms);

            // Empty list = the whole signal catalog, one timestamp.
            const auto snap = client.snapshotSignals();
            if (!snap) { std::fprintf(stderr, "snapshotSignals failed\n"); rc = 1; }
            if (rc == 0) {
                const auto& [ts, values] = *snap;
                std::printf("[client] snapshot ts=%llu, %zu values\n",
                            static_cast<unsigned long long>(ts),
                            values.size());
                for (const auto& v : values) {
                    double d = 0;
                    if (v.value.size() == 8)
                        std::memcpy(&d, v.value.data(), 8);
                    std::printf("  id=%llu slot=%u %+.4f\n",
                                static_cast<unsigned long long>(v.id),
                                v.schemaSlot, d);
                }
                if (values.size() != 3) rc = 1;
            }

            const auto psnap = client.snapshotParams();
            if (!psnap) { std::fprintf(stderr, "snapshotParams failed\n"); rc = 1; }
            if (rc == 0) {
                const auto& [ts, values] = *psnap;
                std::printf("[client] param snapshot ts=%llu, %zu values\n",
                            static_cast<unsigned long long>(ts),
                            values.size());
                if (values.size() != 1) rc = 1;
            }
        }
    }

    run = false;
    producer.join();
    server.stop();
    return rc;
}
