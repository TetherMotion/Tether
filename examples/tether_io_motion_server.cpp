/**
 * @file tether_io_motion_server.cpp
 * @brief Tether IO server exposing simulated CiA 402 drives.
 *
 * Serves one SimulatedCiA402Drive axis per configured slave index. Each
 * standard DS402 object (controlword, statusword, modes, actuals, targets)
 * is published as a cyclic IO parameter named `mc.<slave>.<index>` so a
 * client (e.g. a PLC runtime) can read/write the object dictionary over
 * the Tether IO protocol without SDO commissioning rate limits.
 *
 *   tether_io_motion_server [--port 4000] [--bind 127.0.0.1] [--axes 4]
 *
 * Connect with:
 *   tether_io_client -u tcp://127.0.0.1:4000 list params
 *   tether_io_client -u tcp://127.0.0.1:4000 get mc.0.24640
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Server.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SimulatedCiA402Drive.hpp"

#include <argparse/argparse.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace tether::io;
using namespace tether::io::machine::sim402;

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop = true; }

uint64_t steadyNowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

/* Client-driven sim clock: the mc.time param writes this; the drive
   advances to it deterministically (PLC --sim replay). --clock realtime
   uses the wall clock instead. */
std::atomic<uint64_t> g_simUs{0};
uint64_t simNowUs() { return g_simUs.load(std::memory_order_relaxed); }
std::atomic<bool> g_realtime{false};
uint64_t driveNowUs() { return g_realtime.load() ? steadyNowUs() : simNowUs(); }

void motionLog(const char* level, const char* msg, ...) {
    va_list ap;
    va_start(ap, msg);
    std::fprintf(stderr, "[motion:%s] ", level);
    std::vfprintf(stderr, msg, ap);
    std::fprintf(stderr, "\n");
    va_end(ap);
}

// ---------------------------------------------------------------------------
// Schema slots for the scalar value types the DS402 objects use. One scalar
// node per type, keyed in a private namespace.
// ---------------------------------------------------------------------------
struct MotionSchemas {
    SchemaGraph graph;
    std::vector<SchemaManifestEntry> manifest;
    SchemaRef i8, i16, i32, u16, u32, u64;
    uint32_t slotI8 = 0, slotI16 = 0, slotI32 = 0, slotU16 = 0, slotU32 = 0,
             slotU64 = 0;
};

MotionSchemas buildMotionSchemas() {
    MotionSchemas d;
    auto addScalar = [&](uint8_t tag, ValueType t, const char* name,
                         SchemaRef& ref, uint32_t& slot) {
        SchemaNode n;
        n.key = {};
        n.key[0] = 0xC4;                 // CiA402/motion namespace
        n.key[1] = tag;
        n.kind = SchemaKind::Scalar;
        n.scalarType = t;
        n.name = name;
        ref = {n.key, computeSchemaDigest(n)};
        slot = static_cast<uint32_t>(d.manifest.size());
        d.graph.nodes.push_back(std::move(n));
        d.manifest.push_back({ref, 1});
    };
    addScalar(1, ValueType::I8,  "mc.I8",  d.i8,  d.slotI8);
    addScalar(2, ValueType::I16, "mc.I16", d.i16, d.slotI16);
    addScalar(3, ValueType::I32, "mc.I32", d.i32, d.slotI32);
    addScalar(4, ValueType::U16, "mc.U16", d.u16, d.slotU16);
    addScalar(5, ValueType::U32, "mc.U32", d.u32, d.slotU32);
    addScalar(6, ValueType::U64, "mc.U64", d.u64, d.slotU64);
    return d;
}

// ---------------------------------------------------------------------------
// DS402 object bindings. index = CiA402 object dictionary index, subindex is
// always 0 for the scalar objects exposed here. `size` is the on-wire byte
// count and must match the SimulatedCiA402Drive encoding.
// ---------------------------------------------------------------------------
struct ObjDef {
    uint16_t    index;
    ValueType   type;
    uint8_t     size;
    const char* name;
    bool        writable;
};

const ObjDef kObjects[] = {
    {0x603F, ValueType::U16, 2, "error",            false},
    {0x6040, ValueType::U16, 2, "controlword",       true },
    {0x6041, ValueType::U16, 2, "statusword",        false},
    {0x6060, ValueType::I8,  1, "mode",              true },
    {0x6061, ValueType::I8,  1, "mode_display",      false},
    {0x6064, ValueType::I32, 4, "position_actual",   true },
    {0x6065, ValueType::U32, 4, "follow_window",     true },
    {0x606C, ValueType::I32, 4, "velocity_actual",   false},
    {0x6071, ValueType::I16, 2, "torque_target",     true },
    {0x6077, ValueType::I16, 2, "torque_actual",     false},
    {0x607A, ValueType::I32, 4, "target_position",   true },
    {0x607C, ValueType::I32, 4, "home_offset",       true },
    {0x60FF, ValueType::I32, 4, "target_velocity",   true },
};

SchemaRef refFor(const MotionSchemas& d, ValueType t) {
    switch (t) {
        case ValueType::I8:  return d.i8;
        case ValueType::I16: return d.i16;
        case ValueType::I32: return d.i32;
        case ValueType::U16: return d.u16;
        case ValueType::U32: return d.u32;
        case ValueType::U64: return d.u64;
        default:             return {};
    }
}
uint32_t slotFor(const MotionSchemas& d, ValueType t) {
    switch (t) {
        case ValueType::I8:  return d.slotI8;
        case ValueType::I16: return d.slotI16;
        case ValueType::I32: return d.slotI32;
        case ValueType::U16: return d.slotU16;
        case ValueType::U32: return d.slotU32;
        case ValueType::U64: return d.slotU64;
        default:             return 0;
    }
}

/**Register every DS402 object of `slave` as an mc.<slave>.<index> param.*/
bool registerAxisObjects(Registry& reg, const MotionSchemas& d,
                         SimulatedCiA402Drive& drive, uint16_t slave,
                         uint64_t& nextId) {
    for (const ObjDef& o : kObjects) {
        ParamEntry p;
        p.id = nextId++;
        p.name = std::string("mc.") + std::to_string(slave) + "." +
                 [&]{ char b[8]; std::snprintf(b, sizeof b, "%04X", o.index);
                      return std::string(b); }();
        p.description = std::string("DS402 ") + o.name +
                        (o.writable ? " (RW)" : " (RO)");
        p.group = "mc." + std::to_string(slave);
        p.valueType = o.type;
        p.schema = refFor(d, o.type);
        p.schemaSlot = slotFor(d, o.type);
        const uint16_t idx = o.index;
        const uint8_t  sz  = o.size;
        p.readFn = [&drive, slave, idx, sz](void* dst) {
            auto r = drive.read(slave, idx, 0, sz);
            std::memset(dst, 0, sz);
            if (r.ok && !r.data.empty())
                std::memcpy(dst, r.data.data(),
                            std::min<size_t>(r.data.size(), sz));
        };
        if (o.writable) {
            p.writeFn = [&drive, slave, idx, sz](const void* src) {
                const uint8_t* b = static_cast<const uint8_t*>(src);
                drive.write(slave, idx, 0,
                            std::vector<uint8_t>(b, b + sz));
            };
        }
        if (!reg.addParam(p)) return false;
    }
    return true;
}

/**Global mc.time param: write sets the deterministic sim clock (us);
   read returns the last value. Drives advance to it on next access.*/
bool registerTimeParam(Registry& reg, const MotionSchemas& d,
                       uint64_t& nextId) {
    ParamEntry p;
    p.id = nextId++;
    p.name = "mc.time";
    p.description = "Simulated drive clock, microseconds (RW)";
    p.group = "mc";
    p.valueType = ValueType::U64;
    p.schema = d.u64;
    p.schemaSlot = d.slotU64;
    p.readFn = [](void* dst) {
        const uint64_t t = simNowUs();
        std::memcpy(dst, &t, 8);
    };
    p.writeFn = [](const void* src) {
        uint64_t t;
        std::memcpy(&t, src, 8);
        g_simUs.store(t, std::memory_order_relaxed);
    };
    return reg.addParam(p);
}

} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser prog("tether_io_motion_server");
    prog.add_argument("--port").default_value(std::string("4000"));
    prog.add_argument("--bind").default_value(std::string("127.0.0.1"));
    prog.add_argument("--axes").default_value(std::string("4"))
        .help("number of simulated CiA402 axes (slave indices 0..N-1)");
    prog.add_argument("--clock").default_value(std::string("sim"))
        .help("sim = client-driven via mc.time (deterministic); "
              "realtime = wall clock");
    try { prog.parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
    const uint16_t port = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--port")));
    const std::string bind = prog.get<std::string>("--bind");
    const int axes = std::max(1, std::stoi(prog.get<std::string>("--axes")));
    g_realtime.store(prog.get<std::string>("--clock") == "realtime");

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    // --- catalog + registry -------------------------------------------------
    MotionSchemas schemas = buildMotionSchemas();
    SchemaCatalog catalog;
    if (!catalog.install(schemas.graph, schemas.manifest)) {
        std::fprintf(stderr, "schema install failed\n");
        return 1;
    }

    SimulatedCiA402Drive drive;
    drive.setClock(driveNowUs);

    Registry registry;
    uint64_t nextId = 1;
    if (!registerTimeParam(registry, schemas, nextId)) {
        std::fprintf(stderr, "mc.time param registration failed\n");
        return 1;
    }
    for (int a = 0; a < axes; ++a) {
        const uint16_t slave = static_cast<uint16_t>(a);
        drive.configure(slave);   // create with defaults
        if (!registerAxisObjects(registry, schemas, drive, slave, nextId)) {
            std::fprintf(stderr, "param registration failed for axis %d\n", a);
            return 1;
        }
    }

    // --- server --------------------------------------------------------------
    ServerConfig cfg;
    cfg.framing = Framing::Slip;
    cfg.timestampFn = steadyNowUs;
    cfg.logFn = motionLog;
    cfg.schemaCatalog = &catalog;
    cfg.maxClients = 4;

    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, bind), cfg);
    if (!server.start()) {
        std::fprintf(stderr, "listen %s:%u failed\n", bind.c_str(), port);
        return 1;
    }
    std::printf("tether_io_motion_server: SLIP/TCP on %s:%u — %d CiA402 "
                "axes, %u params, %zu schema slots\n",
                bind.c_str(), port, axes, registry.paramCount(),
                catalog.size());

    // Advance the simulated drives at ~1 kHz so dynamics integrate
    // continuously between client polls.
    std::thread ticker([&] {
        while (!g_stop) {
            drive.advanceTo(driveNowUs());
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.stop();
    g_stop = true;
    ticker.join();
    return 0;
}
