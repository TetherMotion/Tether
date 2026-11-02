/**
 * @file tether_io_polled_signal.cpp
 * @brief LatestSignal: producer-emitted rows read back via getSignal.
 *
 * A Tether IO signal has two reader views: a streamed history (ring)
 * and a latest-value snapshot. LatestSignal owns both; this example
 * exercises the snapshot half — a producer thread calls produce() at
 * 100 Hz and a client polls getSignal, always seeing the newest row.
 *
 * LatestSignal wires into SignalEntry::varReadFn, which only serves
 * variable-length entries — a fixed U64 would be read via readFn and
 * never see the signal. The demo signal is a Packed-struct schema whose
 * wire encoding IS the row bytes verbatim: produce() emits a valid
 * schema value with zero marshalling.
 *
 *   ./tether_io_polled_signal [--port 42783]
 */

#include "tether/io/LatestSignal.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaValue.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <algorithm>
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

SchemaKey demoKey(uint8_t tag) { SchemaKey k{}; k[0] = 0xDE; k[1] = tag; return k; }

/// Row layout the producer emits: seq at 0, value at 8.
struct Row {
    uint64_t seq;
    double value;
};

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42783;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    // Schemas: two scalar leaf types + the Packed struct row.
    // Packed encoding = fields at fixed offsets, no tags — the C++ Row
    // layout below is already a valid wire value.
    SchemaGraph graph;
    SchemaNode u64n;
    u64n.key = demoKey(1); u64n.kind = SchemaKind::Scalar;
    u64n.scalarType = ValueType::U64; u64n.name = "demo.U64";
    SchemaNode f64n;
    f64n.key = demoKey(2); f64n.kind = SchemaKind::Scalar;
    f64n.scalarType = ValueType::F64; f64n.name = "demo.F64";
    SchemaNode row;
    row.key = demoKey(3); row.kind = SchemaKind::Struct;
    row.structEncoding = StructEncoding::Packed;
    row.name = "demo.SampleRow";
    row.fields = {
        {1, 0, {demoKey(1), {}}, "seq", ""},
        {2, 0, {demoKey(2), {}}, "value", ""},
    };
    graph.nodes = {std::move(u64n), std::move(f64n), std::move(row)};
    if (!resolveSchemaDigests(graph)) return 1;
    SchemaCatalog catalog;
    if (!catalog.install(graph, buildSchemaManifest(graph))) return 1;

    // The signal: 16-byte rows, ring of 64, latest snapshot pre-sized.
    LatestSignal<16, 64> demoSig(sizeof(Row));
    demoSig.setSchema({/*entryIds=*/1}, {/*fieldSizes=*/sizeof(Row)});

    Registry registry;
    {
        SignalEntry s;
        s.id = 1;
        s.name = "demo.sample";
        s.description = "100 Hz {seq, value} rows — polled snapshot";
        s.group = "demo";
        s.valueType = ValueType::Struct;
        s.maxValueSize = sizeof(Row);
        s.schema = {demoKey(3),
                    computeSchemaDigest(*graph.find(demoKey(3)))};
        s.schemaSlot = *catalog.slotForKey(demoKey(3));
        s.varReadFn = demoSig.varReadFn();  // LatestSignal must outlive it
        if (!registry.addSignal(s)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    sc.ringSources = {&demoSig.source()};  // stream view, used next example
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    std::atomic<bool> run{true};
    std::thread producer([&] {
        uint64_t seq = 0;
        while (run) {
            Row r{seq, std::sin(static_cast<double>(seq) * 0.01)};
            demoSig.produce(steadyClockUs(),
                            {reinterpret_cast<const uint8_t*>(&r),
                             sizeof(r)});
            ++seq;
            std::this_thread::sleep_for(10ms);
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

            const auto signals = client.listSignals();
            if (!signals || signals->empty()) rc = 1;
            const uint64_t sigId = rc == 0 ? (*signals)[0].id : 0;

            for (int i = 0; i < 5 && rc == 0; ++i) {
                const auto raw = client.getSignal(sigId);
                if (!raw) {
                    std::fprintf(stderr, "getSignal: %s\n",
                                 raw.error().message.c_str());
                    rc = 1;
                    break;
                }
                // The wire bytes are the packed row verbatim — and also
                // decode as a schema value via the negotiated slot.
                const auto v = client.decodeValue(
                    (*signals)[0].schemaSlot, raw->data(), raw->size());
                Row r;
                std::memcpy(&r, raw->data(),
                            std::min(raw->size(), sizeof(r)));
                const char* decoded = "";
                char dbuf[64];
                if (v) {
                    if (const SchemaValue* fv = fieldValue(*v, "value"))
                        if (const auto d = asF64(*fv)) {
                            std::snprintf(dbuf, sizeof dbuf,
                                          " decoded=%+.4f", *d);
                            decoded = dbuf;
                        }
                }
                std::printf("[client] poll %d: seq=%llu value=%+.4f%s\n", i,
                            static_cast<unsigned long long>(r.seq), r.value,
                            decoded);
                std::this_thread::sleep_for(50ms);
            }
            if (rc == 0)
                std::printf("[client] note: seq advances between polls — "
                            "getSignal returns the latest row, not history\n");
        }
    }

    run = false;
    producer.join();
    server.stop();
    return rc;
}
