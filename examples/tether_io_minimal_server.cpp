/**
 * @file tether_io_minimal_server.cpp
 * @brief The smallest useful Tether IO v6 server: one param, one signal.
 *
 * Everything a catalog server needs, nothing else:
 *   1. A SchemaGraph + SchemaCatalog — entries reference schemas by slot.
 *   2. A Registry with a writable param (`demo.gain`, F64) and a
 *      read-only signal (`demo.ticks`, U64 fed by a producer thread).
 *   3. A Server on TCP/SLIP that accepts V6 sessions.
 *
 *   ./tether_io_minimal_server [--port 42780] [--selfcheck]
 *
 * Default mode serves until interrupted — point a client at it:
 *   tether_io_client -u tcp://127.0.0.1:42780 list params
 * --selfcheck runs a quick in-process client round-trip and exits.
 */

#include "tether/io/ProducerHelpers.hpp"  // steadyClockUs
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
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

// Demo schema namespace: keys 0xDE/xx never collide with machine-profile
// or application schemas.
SchemaKey demoKey(uint8_t tag) {
    SchemaKey k{};
    k[0] = 0xDE;
    k[1] = tag;
    return k;
}

/// One scalar node per wire type the catalog uses.
struct DemoSchemas {
    SchemaGraph graph;
    SchemaRef f64, u64;
};

DemoSchemas buildDemoSchemas() {
    DemoSchemas d;
    auto scalar = [&](SchemaKey key, ValueType type, const char* name,
                      SchemaRef& out) {
        SchemaNode n;
        n.key = key;
        n.kind = SchemaKind::Scalar;
        n.scalarType = type;
        n.name = name;
        d.graph.nodes.push_back(std::move(n));
        out = {key, {}};  // digest filled by resolveSchemaDigests
    };
    scalar(demoKey(1), ValueType::F64, "demo.F64", d.f64);
    scalar(demoKey(2), ValueType::U64, "demo.U64", d.u64);
    if (!resolveSchemaDigests(d.graph))
        throw std::runtime_error("schema digest resolution failed");
    // Refs returned above carry empty digests — re-read from the graph.
    d.f64.digest = computeSchemaDigest(*d.graph.find(d.f64.key));
    d.u64.digest = computeSchemaDigest(*d.graph.find(d.u64.key));
    return d;
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42780;
    bool selfcheck = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
        else if (a == "--selfcheck")
            selfcheck = true;
    }

    const DemoSchemas schemas = buildDemoSchemas();
    SchemaCatalog catalog;
    if (!catalog.install(schemas.graph, buildSchemaManifest(schemas.graph))) {
        std::fprintf(stderr, "schema install failed\n");
        return 1;
    }
    // Entries carry the *installed* slot, not the manifest build index:
    // install() sorts slots by schema key.
    const auto slotF64 = catalog.slotForKey(demoKey(1));
    const auto slotU64 = catalog.slotForKey(demoKey(2));
    if (!slotF64 || !slotU64) return 1;

    std::atomic<double> gain{1.0};
    std::atomic<uint64_t> ticks{0};

    Registry registry;
    {
        ParamEntry p;
        p.id = 1;
        p.name = "demo.gain";
        p.description = "Writable gain (RW)";
        p.group = "demo";
        p.valueType = ValueType::F64;
        p.schema = schemas.f64;
        p.schemaSlot = *slotF64;
        p.readFn  = [&](void* dst) { const double v = gain.load();
                                     std::memcpy(dst, &v, 8); };
        p.writeFn = [&](const void* s) { double v;
                                         std::memcpy(&v, s, 8);
                                         gain.store(v); };
        if (!registry.addParam(p)) return 1;

        SignalEntry s;
        s.id = 2;
        s.name = "demo.ticks";
        s.description = "1 kHz counter (RO)";
        s.group = "demo";
        s.valueType = ValueType::U64;
        s.schema = schemas.u64;
        s.schemaSlot = *slotU64;
        s.readFn = [&](void* dst) { const uint64_t v = ticks.load();
                                    std::memcpy(dst, &v, 8); };
        if (!registry.addSignal(s)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    sc.schemaCatalog = &catalog;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) {
        std::fprintf(stderr, "server start failed (port %u in use?)\n", port);
        return 1;
    }

    std::atomic<bool> run{true};
    std::thread producer([&] {
        while (run) { ++ticks; std::this_thread::sleep_for(1ms); }
    });
    std::printf("[server] listening on 127.0.0.1:%u — params=%u signals=%u\n",
                port, registry.paramCount(), registry.signalCount());

    if (!selfcheck) {
        // Serve until interrupted.
        while (true) std::this_thread::sleep_for(1s);
    }

    // In-process client round-trip: read the param, write it, read the
    // signal — proves the catalog answers over the wire.
    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) return 1;
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);

    const auto params = client.listParams();
    const auto signals = client.listSignals();
    if (!params || !signals) return 1;
    const uint64_t gainId = (*params)[0].id, ticksId = (*signals)[0].id;

    if (const auto v = client.getParam(gainId)) {
        double g; std::memcpy(&g, v->data(), 8);
        std::printf("[client] demo.gain = %g\n", g);
    }
    const double newGain = 2.5;
    if (!client.setParam(gainId, &newGain, 8)) return 1;
    if (const auto v = client.getParam(gainId)) {
        double g; std::memcpy(&g, v->data(), 8);
        std::printf("[client] set -> demo.gain = %g\n", g);
    }
    if (const auto v = client.getSignal(ticksId)) {
        uint64_t t; std::memcpy(&t, v->data(), 8);
        std::printf("[client] demo.ticks = %llu\n",
                    static_cast<unsigned long long>(t));
    }
    run = false;
    producer.join();
    server.stop();
    return 0;
}
