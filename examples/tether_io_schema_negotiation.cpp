/**
 * @file tether_io_schema_negotiation.cpp
 * @brief V6 schema negotiation: epoch, manifest slots, decode-by-slot.
 *
 * During the V6 bootstrap the server sends ServerHello{epoch, manifest}
 * — the manifest is an ordered list of {key, digest} schema refs; the
 * index is the wire slot. The client fetches the node definitions it
 * needs (SchemaRequest/SchemaDefinition) and commits the epoch.
 * TetherIOClient performs this in its constructor; afterwards:
 *
 *   - catalog entries carry schemaSlot + schemaEpoch
 *   - resolveSchemaSlot(slot) maps a slot to the negotiated node
 *   - decodeValue(slot, bytes) decodes a value the program never
 *     compiled knowledge of — the negotiated graph is the dictionary
 *
 *   ./tether_io_schema_negotiation [--port 42786]
 */

#include "tether/io/LatestSignal.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

namespace {
SchemaKey k(uint8_t tag) { SchemaKey x{}; x[0] = 0xDE; x[1] = tag; return x; }
struct Row { uint64_t seq; double value; };
} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42786;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    SchemaGraph graph;
    {
        SchemaNode u64n;
        u64n.key = k(1); u64n.kind = SchemaKind::Scalar;
        u64n.scalarType = ValueType::U64; u64n.name = "demo.U64";
        SchemaNode f64n;
        f64n.key = k(2); f64n.kind = SchemaKind::Scalar;
        f64n.scalarType = ValueType::F64; f64n.name = "demo.F64";
        SchemaNode row;
        row.key = k(3); row.kind = SchemaKind::Struct;
        row.structEncoding = StructEncoding::Packed;
        row.name = "demo.SampleRow";
        row.fields = {{1, 0, {k(1), {}}, "seq", ""},
                      {2, 0, {k(2), {}}, "value", ""}};
        graph.nodes = {std::move(u64n), std::move(f64n), std::move(row)};
    }
    if (!resolveSchemaDigests(graph)) return 1;
    SchemaCatalog catalog;
    if (!catalog.install(graph, buildSchemaManifest(graph))) return 1;
    const auto rowSlot = catalog.slotForKey(k(3));
    if (!rowSlot) return 1;

    LatestSignal<16, 8> sig(sizeof(Row));
    Registry registry;
    {
        SignalEntry s;
        s.id = 1;
        s.name = "demo.sample";
        s.description = "packed {seq, value} row";
        s.group = "demo";
        s.valueType = ValueType::Struct;
        s.maxValueSize = sizeof(Row);
        s.schema = {k(3), computeSchemaDigest(*graph.find(k(3)))};
        s.schemaSlot = *rowSlot;
        s.varReadFn = sig.varReadFn();
        if (!registry.addSignal(s)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    std::atomic<bool> run{true};
    std::thread producer([&] {
        uint64_t seq = 0;
        while (run) {
            Row r{seq, seq * 0.5};
            sig.produce(steadyClockUs(),
                        {reinterpret_cast<const uint8_t*>(&r), sizeof(r)});
            ++seq;
            std::this_thread::sleep_for(50ms);
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

            // What the bootstrap negotiated.
            std::printf("[client] schemaEpoch=%llu manifest=%zu slots "
                        "features=0x%x\n",
                        static_cast<unsigned long long>(
                            client.schemaEpoch()),
                        client.schemaManifest().size(),
                        client.negotiatedFeatures());
            for (size_t i = 0; i < client.schemaManifest().size(); ++i) {
                const auto* n = client.resolveSchemaSlot(
                    static_cast<uint32_t>(i));
                std::printf("  slot %zu -> %s\n", i,
                            n ? n->name.c_str() : "(unresolved)");
            }

            // Catalog entries reference manifest slots.
            const auto signals = client.listSignals();
            if (!signals || signals->empty()) rc = 1;
            if (rc == 0) {
                const auto& e = (*signals)[0];
                std::printf("[client] '%s' schemaSlot=%u epoch=%llu\n",
                            e.name.c_str(), e.schemaSlot,
                            static_cast<unsigned long long>(e.schemaEpoch));

                // Decode the value against the negotiated graph — no
                // compile-time Row knowledge on this path.
                const auto raw = client.getSignal(e.id);
                const auto val = raw ? client.decodeValue(
                    e.schemaSlot, raw->data(), raw->size())
                                     : std::expected<SchemaValue,
                                                     ClientError>(
                                           std::unexpected(
                                               ClientError{ErrorCode::Timeout,
                                                           "no value"}));
                if (!val) { std::fprintf(stderr, "decodeValue failed\n"); rc = 1; }
                if (rc == 0) {
                    const auto& st =
                        std::get<SchemaValue::Struct>(val->payload);
                    for (const auto& f : st.fields) {
                        const auto* s = std::get_if<SchemaValue::Scalar>(
                            &f.value->payload);
                        std::printf("  decoded %s bits=0x%llx\n",
                                    f.schemaField->name.c_str(),
                                    static_cast<unsigned long long>(
                                        s ? s->bits : 0));
                    }
                }
            }
        }
    }

    run = false;
    producer.join();
    server.stop();
    return rc;
}
