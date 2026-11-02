/**
 * @file tether_io_machine_profile.cpp
 * @brief The CiA 402 machine profile layer, distilled: descriptor,
 *        coherent snapshots, per-drive state, and the retained event
 *        journal — all schema-encoded.
 *
 * SimulatedCiA402Fleet installs the production profile surface:
 *   - signal machine.descriptor        (machine identity + axes)
 *   - signal machine.snapshot          (all drives, coherent)
 *   - signal drive.<id>.snapshot       (per-axis CiA 402 state)
 *   - signal machine.events.cursor     (latest journal cursor)
 *   - function machine.events.read     (paged event reads)
 *
 * Values are schema-structured: the client decodes via decodeValue()
 * against the negotiated manifest, so field names — not byte offsets —
 * carry the meaning.
 *
 *   ./tether_io_machine_profile
 */

#include "tether/io/CiA402MachineProfile.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Schema.hpp"
#include "tether/io/SchemaValue.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace tether;
using namespace tether::io;

namespace {
const SchemaValue* findField(const SchemaValue& v, const char* name) {
    const auto* st = std::get_if<SchemaValue::Struct>(&v.payload);
    if (!st) return nullptr;
    for (const auto& f : st->fields)
        if (f.schemaField->name == name) return f.value.get();
    return nullptr;
}

double scalarToF64(const SchemaValue* v) {
    if (!v) return 0;
    const auto* s = std::get_if<SchemaValue::Scalar>(&v->payload);
    if (!s) return 0;
    if (v->node && v->node->scalarType == ValueType::F64) {
        double d;
        const uint64_t b = s->bits;
        std::memcpy(&d, &b, 8);
        return d;
    }
    return static_cast<double>(s->bits);
}
} // namespace

int main() {
    // Profile schemas → catalog → registry entries — three calls.
    // The catalog references the graph; it must outlive the server.
    const SchemaGraph graph = cia402::machineProfileSchemaGraph();
    SchemaCatalog catalog;
    if (!cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog)) {
        std::fprintf(stderr, "schema install failed\n");
        return 1;
    }
    std::printf("[setup] machine profile: %zu schema slots\n",
                catalog.size());

    Registry registry;
    cia402::SimulatedCiA402Fleet fleet;
    if (!fleet.registerSignals(registry, catalog)) {
        std::fprintf(stderr, "fleet registration failed\n");
        return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(42790, "127.0.0.1"),
                  sc);
    if (!server.start()) return 1;

    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(
            tcpConnect("127.0.0.1", 42790, 2000)),
        2000);

    // --- Descriptor: axis topology ------------------------------------------
    const auto signals = client.listSignals();
    if (!signals) return 1;
    std::printf("[client] %zu signals:\n", signals->size());
    for (const auto& s : *signals)
        std::printf("         %-32s slot=%u\n", s.name.c_str(),
                    s.schemaSlot);

    // --- A fault on axis X generates a journal event ------------------------
    auto snap = fleet.driveSnapshot("sim-axis-x").value_or(
        cia402::DriveSnapshotV1{});
    snap.ds402State = 7;       // CiA 402 "fault"
    snap.faultCode = 0x2220;
    fleet.updateDriveSnapshot("sim-axis-x", snap);

    const auto signalId = [&](const char* name) -> uint64_t {
        for (const auto& s : *signals)
            if (s.name == name) return s.id;
        return 0;
    };

    // --- Event journal: cursor + paged read ---------------------------------
    const auto cursorSig = client.getSignal(signalId("machine.events.cursor"));
    if (!cursorSig || cursorSig->size() != 8) return 1;
    uint64_t cursor;
    std::memcpy(&cursor, cursorSig->data(), 8);
    std::printf("[client] events cursor = %llu\n",
                static_cast<unsigned long long>(cursor));

    // machine.events.read(after_cursor=0, limit=10) → EventPage struct
    const auto funcs = client.listFunctions();
    if (!funcs || funcs->empty()) return 1;
    const auto& readFn = (*funcs)[0];
    TetherIOClient::FunctionArg cur, lim;
    cur.position = 0;
    cur.value.resize(8);           // after_cursor = 0 → all events
    lim.position = 1;
    lim.value = {10, 0, 0, 0};     // limit = 10
    const auto page = client.callFunction(readFn.id, {cur, lim});
    if (!page || !page->success) {
        std::fprintf(stderr, "events.read failed\n");
        return 1;
    }
    const auto decoded = client.decodeValue(readFn.returnSchemaSlot,
                                            page->returnValue.data(),
                                            page->returnValue.size());
    if (!decoded) return 1;
    size_t nEvents = 0;
    if (const auto* events = findField(*decoded, "events")) {
        if (const auto* arr =
                std::get_if<SchemaValue::Array>(&events->payload))
            nEvents = arr->elements.size();
    }
    const auto* oldest = findField(*decoded, "oldest_cursor");
    const auto* next = findField(*decoded, "next_cursor");
    std::printf("[client] events.read -> %zu events, oldest_cursor=%.0f "
                "next_cursor=%.0f\n", nEvents, scalarToF64(oldest),
                scalarToF64(next));

    // --- Drive snapshot decode ----------------------------------------------
    const uint64_t driveId = signalId("drive.sim-axis-x.snapshot");
    const auto driveEntry = std::find_if(
        signals->begin(), signals->end(),
        [driveId](const ClientCatalogEntry& e) { return e.id == driveId; });
    if (driveId == 0 || driveEntry == signals->end()) return 1;
    const auto raw = client.getSignal(driveId);
    if (!raw) return 1;
    const auto drv = client.decodeValue(driveEntry->schemaSlot,
                                        raw->data(), raw->size());
    if (drv) {
        const auto* state = findField(*drv, "ds402_state");
        const auto* fault = findField(*drv, "fault_code");
        std::printf("[client] drive.sim-axis-x: ds402_state=%.0f "
                    "fault_code=0x%04llx\n",
                    scalarToF64(state),
                    static_cast<unsigned long long>(scalarToF64(fault)));
    }

    server.stop();
    return (nEvents >= 2) ? 0 : 1;
}
