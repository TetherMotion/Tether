/**
 * @file tether_io_client_basics.cpp
 * @brief TetherIOClient walkthrough: handshake, catalogs, params, signals,
 *        functions.
 *
 * The client-side counterpart to tether_io_minimal_server — this tour
 * covers every synchronous catalog API:
 *   connect + V6 handshake (ctor)  → ping()
 *   listParams()/listSignals()/listFunctions() → getParam()/setParam()
 *   getSignal()                    → callFunction()
 *
 * Self-contained: an embedded server hosts one of each entry, the client
 * runs the tour against it.
 *
 *   ./tether_io_client_basics [--port 42781]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <cstring>
#include <cstdio>
#include <string>

using namespace tether::io;

namespace {

SchemaKey demoKey(uint8_t tag) { SchemaKey k{}; k[0] = 0xDE; k[1] = tag; return k; }

// Catalog with one of each shape the tour needs.
bool buildCatalog(Registry& registry, SchemaCatalog& catalog,
                  SchemaGraph& graph, double& paramStore) {
    SchemaNode n;
    n.key = demoKey(1);
    n.kind = SchemaKind::Scalar;
    n.scalarType = ValueType::F64;
    n.name = "demo.F64";
    graph.nodes.push_back(n);
    n.key = demoKey(2);
    n.name = "demo.U64";
    n.scalarType = ValueType::U64;
    graph.nodes.push_back(n);
    if (!resolveSchemaDigests(graph)) return false;
    if (!catalog.install(graph, buildSchemaManifest(graph))) return false;
    const auto slotF64 = catalog.slotForKey(demoKey(1));
    const auto slotU64 = catalog.slotForKey(demoKey(2));
    if (!slotF64 || !slotU64) return false;
    const SchemaRef f64ref{demoKey(1), computeSchemaDigest(graph.nodes[0])};
    const SchemaRef u64ref{demoKey(2), computeSchemaDigest(graph.nodes[1])};

    ParamEntry p;
    p.id = 1; p.name = "demo.gain"; p.group = "demo";
    p.description = "writable F64";
    p.valueType = ValueType::F64;
    p.schema = f64ref; p.schemaSlot = *slotF64;
    p.readFn  = [&](void* dst) { std::memcpy(dst, &paramStore, 8); };
    p.writeFn = [&](const void* s) { std::memcpy(&paramStore, s, 8); };
    if (!registry.addParam(p)) return false;

    SignalEntry s;
    s.id = 2; s.name = "demo.answer"; s.group = "demo";
    s.description = "constant U64";
    s.valueType = ValueType::U64;
    s.schema = u64ref; s.schemaSlot = *slotU64;
    s.readFn = [](void* dst) { const uint64_t v = 42;
                               std::memcpy(dst, &v, 8); };
    if (!registry.addSignal(s)) return false;

    FunctionEntry f;
    f.id = 3; f.name = "demo.add_one"; f.group = "demo";
    f.description = "y = x + 1";
    f.parameters = {{"x", "input", ValueType::F64}};
    f.returnValue.present = true;
    f.returnValue.name = "y";
    f.returnValue.type = ValueType::F64;
    f.callback = [](const std::vector<FunctionArgument>& args) {
        FunctionCallResult r;
        double x = 0;
        for (const auto& a : args)
            if (a.position == 0 && a.value.size() == 8)
                std::memcpy(&x, a.value.data(), 8);
        const double y = x + 1.0;
        r.success = true;
        r.returnValue.resize(8);
        std::memcpy(r.returnValue.data(), &y, 8);
        return r;
    };
    return registry.addFunction(f);
}

const ClientCatalogEntry* findByName(
    const std::vector<ClientCatalogEntry>& entries, std::string_view name) {
    for (const auto& e : entries) if (e.name == name) return &e;
    return nullptr;
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42781;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    SchemaCatalog catalog;
    SchemaGraph graph;
    double gain = 1.0;
    if (!buildCatalog(registry, catalog, graph, gain)) return 1;

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) return 1;
    // The ctor runs the V6 handshake: schema manifest + epoch are
    // negotiated before the first catalog call.
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);
    std::printf("[client] handshake ok — schema epoch %llu, manifest %zu entries\n",
                (unsigned long long)client.schemaEpoch(),
                client.schemaManifest().size());

    if (const auto pong = client.ping(0x12345678))
        std::printf("[client] ping -> 0x%08x\n", *pong);

    const auto params = client.listParams();
    const auto signals = client.listSignals();
    const auto functions = client.listFunctions();
    if (!params || !signals || !functions) return 1;
    for (const auto& e : *params)
        std::printf("  param   %s (id %llu)\n", e.name.c_str(),
                    (unsigned long long)e.id);
    for (const auto& e : *signals)
        std::printf("  signal  %s (id %llu)\n", e.name.c_str(),
                    (unsigned long long)e.id);
    for (const auto& e : *functions)
        std::printf("  func    %s (id %llu)\n", e.name.c_str(),
                    (unsigned long long)e.id);

    const ClientCatalogEntry* gain_e = findByName(*params, "demo.gain");
    const ClientCatalogEntry* answer_e = findByName(*signals, "demo.answer");
    if (!gain_e || !answer_e) return 1;

    if (const auto v = client.getParam(gain_e->id)) {
        double g; std::memcpy(&g, v->data(), 8);
        std::printf("[client] demo.gain = %g\n", g);
    }
    const double want = 7.5;
    if (!client.setParam(gain_e->id, &want, 8)) return 1;
    std::printf("[client] setParam(demo.gain, 7.5) -> store now %g\n", gain);

    if (const auto v = client.getSignal(answer_e->id)) {
        uint64_t a; std::memcpy(&a, v->data(), 8);
        std::printf("[client] demo.answer = %llu\n",
                    (unsigned long long)a);
    }

    // callFunction carries FunctionArg TLVs (position + typed bytes).
    const uint64_t addOne = (*functions)[0].id;
    TetherIOClient::FunctionArg x;
    x.position = 0;
    x.type = ValueType::F64;
    x.value.resize(8);
    const double xin = 41.0;
    std::memcpy(x.value.data(), &xin, 8);
    if (const auto r = client.callFunction(addOne, {x})) {
        double y = 0;
        if (r->success && r->returnValue.size() == 8)
            std::memcpy(&y, r->returnValue.data(), 8);
        std::printf("[client] demo.add_one(41) -> %g\n", y);
    }

    server.stop();
    return 0;
}
