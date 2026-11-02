/**
 * @file tether_io_threshold.cpp
 * @brief ThresholdFilter: dead-band stream compression via the wire.
 *
 * configureThreshold() installs a per-session ThresholdConfig that the
 * stream collector applies to every field: a change that fails its rule
 * is emitted as the *last-sent* value, so slow-moving entries stop
 * consuming bandwidth. Rules:
 *
 *   - entryId selects the entry; entryId 0 is the default rule
 *   - Absolute: pass when |new-old| > threshold
 *   - Relative: pass when |new-old|/|old| > threshold
 *   - Custom: named evaluator (server-side registration; not reachable
 *     through Server — unknown evaluators pass everything)
 *
 * isWhitelist=true freezes entries with no matching rule outright;
 * blacklist lets them through. encodedRules on the wire is
 * u32 count + ThresholdRule records.
 *
 *   ./tether_io_threshold [--port 42793]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/ThresholdFilter.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    uint16_t port = 42793;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    // slow drifts in milli-steps; fast jumps by 1 each tick.
    std::atomic<double> slow{0.0}, fast{0.0};
    Registry registry;
    auto addSig = [&](uint64_t id, const char* name,
                      std::atomic<double>& src) {
        SignalEntry s;
        s.id = id;
        s.name = name;
        s.group = "demo";
        s.valueType = ValueType::F64;
        s.readFn = [&src](void* d) {
            const double v = src.load();
            std::memcpy(d, &v, 8);
        };
        return registry.addSignal(s);
    };
    if (!addSig(1, "demo.slow", slow) || !addSig(2, "demo.fast", fast))
        return 1;

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    std::atomic<bool> run{true};
    std::thread producer([&] {
        uint64_t i = 0;
        while (run) {
            slow.store(i * 0.001);  // +0.001 per tick — below the 0.1 band
            fast.store(static_cast<double>(i) * 1.0);
            ++i;
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

            std::atomic<uint32_t> slowChanges{0}, fastChanges{0};
            std::atomic<double> lastSlow{0}, lastFast{0};
            bool first = true;
            client.setStreamDataCallback([&](const ClientStreamRow& row) {
                if (row.values.size() < 2) return;
                double s, f;
                std::memcpy(&s, row.values[0].data(), 8);
                std::memcpy(&f, row.values[1].data(), 8);
                if (first) {
                    first = false;
                } else {
                    if (s != lastSlow.load()) ++slowChanges;
                    if (f != lastFast.load()) ++fastChanges;
                }
                lastSlow.store(s);
                lastFast.store(f);
            });

            // Whitelist: absolute-dead-band 0.1 on entry 1 only;
            // entry 2 has no rule -> frozen at last value.
            std::vector<uint8_t> rulesBytes;
            {
                ThresholdRule r;
                r.entryId = 1;
                r.type = ThresholdType::Absolute;
                r.threshold = 0.1;
                std::vector<uint8_t> buf(64);
                BufWriter w(buf.data(), buf.size());
                w.putU32(1);
                r.encode(w);
                rulesBytes.assign(buf.begin(), buf.begin() + w.pos);
            }
            if (!client.configureThreshold("deadband", true, rulesBytes)) {
                std::fprintf(stderr, "configureThreshold failed\n");
                rc = 1;
            }

            if (rc == 0) {
                const auto cfg = client.configureStream({1, 2}, 5,
                                                        /*chunkSize=*/16);
                if (!cfg || !client.startStream()) rc = 1;
            }
            if (rc == 0) {
                const auto deadline =
                    std::chrono::steady_clock::now() + 400ms;
                while (std::chrono::steady_clock::now() < deadline)
                    (void)client.receiveMessage(20);
                (void)client.stopStream();
                std::printf("[client] in ~400ms: slow changed %u times, "
                            "fast changed %u times\n",
                            slowChanges.load(), fastChanges.load());
                // slow drifts 0.001/tick vs 0.1 band -> a handful of
                // crossings; fast has no rule under whitelist -> 0.
                if (fastChanges != 0 || slowChanges > 20) rc = 1;
            }
        }
    }

    run = false;
    producer.join();
    server.stop();
    return rc;
}
