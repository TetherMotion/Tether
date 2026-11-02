/**
 * @file tether_io_input_stream.cpp
 * @brief Input streams: client->server bulk value channel.
 *
 * The reverse direction of signal streaming: the client opens a typed
 * stream against a negotiated schema slot, then pushes batches of
 * schema-encoded values. Server-side:
 *
 *   - ServerConfig::inputStreamCreateFn vets the stream (schema ref,
 *     sizes) — absent means FeatureNotSupported, false means rejected
 *   - every value is validated against the schema before delivery
 *   - ServerConfig::inputStreamDataFn receives each batch on the
 *     session thread (still wire-encoded — decode with the catalog)
 *
 *   ./tether_io_input_stream [--port 42789]
 */

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
#include <cstdio>
#include <string>
#include <vector>

using namespace tether::io;

namespace {
SchemaKey k(uint8_t tag) { SchemaKey x{}; x[0] = 0xDE; x[1] = tag; return x; }
} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42789;
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
        row.name = "demo.CmdRow";
        row.fields = {{1, 0, {k(1), {}}, "seq", ""},
                      {2, 0, {k(2), {}}, "setpoint", ""}};
        graph.nodes = {std::move(u64n), std::move(f64n), std::move(row)};
    }
    if (!resolveSchemaDigests(graph)) return 1;
    SchemaCatalog catalog;
    if (!catalog.install(graph, buildSchemaManifest(graph))) return 1;
    const auto rowSlot = catalog.slotForKey(k(3));
    if (!rowSlot) return 1;

    Registry registry;

    std::atomic<uint32_t> received{0};
    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    // Accept only the command-row schema; anything else is rejected.
    sc.inputStreamCreateFn =
        [rowSlot](uint32_t streamId, const SchemaRef& ref, SchemaEpoch,
                  SchemaSlot slot, uint32_t, uint32_t) {
            std::printf("[server] create stream id=%u slot=%u\n", streamId,
                        slot);
            return slot == *rowSlot && ref.key == k(3);
        };
    sc.inputStreamDataFn =
        [&received, &catalog](
            uint32_t streamId,
            const std::vector<std::vector<uint8_t>>& values) {
            for (const auto& v : values) {
                const auto dv = decodeSchemaValue(*catalog.graph(),
                                                *catalog.graph()->find(k(3)),
                                                v.data(), v.size());
                if (dv) {
                    const auto& st =
                        std::get<SchemaValue::Struct>(dv->payload);
                    const auto* sq = std::get_if<SchemaValue::Scalar>(
                        &st.fields[0].value->payload);
                    const auto* sp = std::get_if<SchemaValue::Scalar>(
                        &st.fields[1].value->payload);
                    double f;
                    const uint64_t bits = sp ? sp->bits : 0;
                    std::memcpy(&f, &bits, 8);
                    std::printf("[server] stream %u: seq=%llu setpoint=%.3f\n",
                                streamId,
                                static_cast<unsigned long long>(
                                    sq ? sq->bits : 0),
                                f);
                }
            }
            received += static_cast<uint32_t>(values.size());
        };
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

            // Slot 0 is demo.U64 — the create hook rejects it.
            const auto bad = client.createInputStream(0, 8, 4);
            std::printf("[client] wrong-slot stream: %s\n",
                        bad && bad->success ? "accepted (BAD)" : "rejected");

            const auto created =
                client.createInputStream(*rowSlot, 64, 8);
            if (!created || !created->success) {
                std::fprintf(stderr, "createInputStream failed: %s\n",
                             created ? "success=0"
                                     : created.error().message.c_str());
                rc = 1;
            }
            if (rc == 0) {
                const SchemaNode& rowNode = *graph.find(k(3));
                auto encode = [&](uint64_t seq, double sp) {
                    return encodeNative(
                        graph, rowNode,
                        nv::fields(nv::field("seq", nv::u64(seq)),
                                   nv::field("setpoint", nv::f64(sp))));
                };
                std::vector<std::vector<uint8_t>> batch;
                for (uint64_t i = 0; i < 3; ++i) {
                    auto v = encode(i, i * 0.25);
                    if (!v) { rc = 1; break; }
                    batch.push_back(std::move(*v));
                }
                if (rc == 0 &&
                    (!client.inputStreamData(created->streamId, batch) ||
                     !client.closeInputStream(created->streamId)))
                    rc = 1;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100));
            }
        }
    }

    std::printf("[client] server received %u rows\n", received.load());
    server.stop();
    return rc || received != 3 ? 1 : 0;
}
