/**
 * @file tether_io_ring_stream.cpp
 * @brief SpscRingStreamSource: bulk-drained producer rows over a
 *        configured stream — no sampling loss.
 *
 * The stock streaming path is pull-based: the session calls each
 * entry's readFn once per interval, so a 1 kHz producer streamed at a
 * 50 ms interval is decimated to 20 rows/s. A ring source registered in
 * ServerConfig::ringSources instead buffers producer rows in an SPSC
 * ring that the session drains in bulk — every produced row is emitted.
 *
 * Wire shape of a ring row (Session projects fields into the client's
 * configured entries):
 *
 *     [ u64 producer timestamp ][ field0 ][ field1 ] ...
 *
 * Each field maps to one fixed-size scalar registry entry via
 * setSchema({entryIds}, {fieldSizes}). A configured stream binds the
 * ring only if every configured entry is covered — otherwise the
 * session silently falls back to interval polling. Variable-length
 * entries (String/Binary/Struct/Array) can never ring-bind.
 *
 *   ./tether_io_ring_stream [--port 42784]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/RingStreamSource.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

namespace {

// Row image: leading producer timestamp, then one fixed-size field per
// schema entry, in setSchema() order.
struct Row {
    uint64_t ts;
    uint64_t seq;
    double value;
};

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42784;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    // Latest values for the polled getSignal path — the ring covers
    // streaming while readFn still answers polls.
    std::atomic<uint64_t> curSeq{0};
    std::atomic<double> curValue{0.0};

    constexpr uint64_t kSeqId = 1, kValId = 2;
    Registry registry;
    {
        SignalEntry s;
        s.id = kSeqId;
        s.name = "demo.seq";
        s.description = "Producer sequence";
        s.group = "demo";
        s.valueType = ValueType::U64;
        s.readFn = [&curSeq](void* d) {
            const uint64_t v = curSeq.load();
            std::memcpy(d, &v, 8);
        };
        if (!registry.addSignal(s)) return 1;

        s = {};
        s.id = kValId;
        s.name = "demo.value";
        s.description = "sin(seq/1000)";
        s.group = "demo";
        s.valueType = ValueType::F64;
        s.readFn = [&curValue](void* d) {
            const double v = curValue.load();
            std::memcpy(d, &v, 8);
        };
        if (!registry.addSignal(s)) return 1;
    }

    SpscRingStreamSource<Row, 512> ring;
    ring.setSchema({kSeqId, kValId}, {8, 8});

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.ringSources = {&ring};
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    std::atomic<bool> run{true};
    std::thread producer([&] {
        uint64_t seq = 0;
        while (run) {
            const double v = std::sin(static_cast<double>(seq) * 0.001);
            curSeq.store(seq);
            curValue.store(v);
            ring.produce([&](Row& r) {
                r.ts = steadyClockUs();
                r.seq = seq;
                r.value = v;
            });
            ++seq;
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

            // Polled path still works on the scalar readFns.
            if (const auto v = client.getSignal(kValId)) {
                double d;
                std::memcpy(&d, v->data(), 8);
                std::printf("[client] polled demo.value = %+.4f\n", d);
            }

            std::atomic<uint64_t> rows{0};
            std::atomic<uint64_t> firstSeq{UINT64_MAX};
            std::atomic<uint64_t> lastSeq{0};
            client.setStreamDataCallback([&](const ClientStreamRow& row) {
                ++rows;
                if (row.values.size() >= 2 && row.values[0].size() == 8) {
                    uint64_t s;
                    std::memcpy(&s, row.values[0].data(), 8);
                    uint64_t expected = firstSeq.load();
                    while (expected == UINT64_MAX &&
                           !firstSeq.compare_exchange_weak(expected, s)) {}
                    lastSeq.store(s);
                }
            });

            // 50 ms interval: a polled stream would emit ~10 rows in
            // 500 ms; the ring drains every produced row (~500).
            // chunkSize=128 lets one StreamData carry up to 128 rows.
            const auto cfg = client.configureStream(
                {kSeqId, kValId}, 50, /*chunkSize=*/128);
            if (!cfg || !client.startStream()) {
                std::fprintf(stderr, "stream configure/start failed\n");
                rc = 1;
            }
            if (rc == 0) {
                const auto deadline =
                    std::chrono::steady_clock::now() + 500ms;
                while (std::chrono::steady_clock::now() < deadline)
                    (void)client.receiveMessage(20);
                (void)client.stopStream();

                const uint64_t got = rows.load();
                const uint64_t lo = firstSeq.load();
                const uint64_t hi = lastSeq.load();
                std::printf("[client] %llu rows in ~500 ms, seq %llu..%llu, "
                            "ring dropped %llu\n",
                            static_cast<unsigned long long>(got),
                            static_cast<unsigned long long>(lo),
                            static_cast<unsigned long long>(hi),
                            static_cast<unsigned long long>(ring.dropped()));
                if (got < 100 || hi - lo + 1 != got) rc = 1;  // gap-free
            }
        }
    }

    run = false;
    producer.join();
    server.stop();
    return rc;
}
