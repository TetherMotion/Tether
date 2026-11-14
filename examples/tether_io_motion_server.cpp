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
#include "tether/gcode/PlanningSegmentBuilder.hpp"
#include "tether/motion_planner/MotionPlan.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/kinematics/ForwardKinematics.hpp"

#include <argparse/argparse.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#if TETHER_HAVE_OPCUA
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#endif

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
    SchemaRef i8, i16, i32, u8, u16, u32, u64, f64;
    uint32_t slotI8 = 0, slotI16 = 0, slotI32 = 0, slotU8 = 0, slotU16 = 0,
             slotU32 = 0, slotU64 = 0, slotF64 = 0;
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
    addScalar(7, ValueType::U8,  "mc.U8",  d.u8,  d.slotU8);
    addScalar(8, ValueType::F64, "mc.F64", d.f64, d.slotF64);
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
    {0x2001, ValueType::I32, 4, "probe_abort",       true },
    {0x2002, ValueType::I32, 4, "pos_offset",        true },
    {0x2003, ValueType::I32, 4, "brake_cmd",         true },
    {0x2004, ValueType::I32, 4, "park_cmd",          true },
    {0x2005, ValueType::I32, 4, "coe_cmd",           true },
    {0x2006, ValueType::I32, 4, "saw_char0",         true },
    {0x2007, ValueType::I32, 4, "saw_char1",         true },
    {0x2008, ValueType::I32, 4, "saw_char2",         true },
    {0x2009, ValueType::I32, 4, "saw_char3",         true },
    {0x200A, ValueType::I32, 4, "pos_min",           true },
    {0x200B, ValueType::I32, 4, "pos_max",           true },
    {0x200C, ValueType::I32, 4, "axis_flags",        true },
    {0x200D, ValueType::I32, 4, "err_inject",        true },
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

/* NCI channel (defined below): tick is driven by mc.time writes in
   sim mode and by the 1 ms wall ticker in realtime mode. */
namespace nci { void nciTick(uint64_t nowUs); }
namespace fifo { void fifoTick(uint64_t nowUs); }

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
        /* The NCI channel and FiFo channels run on this clock: each
           client time write advances plans/FIFOs deterministically. */
        nci::nciTick(t);
        fifo::fifoTick(t);
    };
    return reg.addParam(p);
}

/* ec.dc.* params: simulated EtherCAT DC (distributed-clock) sync
   state for the TetherPLC FB_EcExtSync* surface. A real master would
   source these from DCManager/EtherCATDC; this server synthesizes
   deterministic values on the sim clock so the PLC side can be
   exercised end to end. */
std::atomic<uint8_t> g_dcSyncEnabled{1};

bool registerEcDcParams(Registry& reg, const MotionSchemas& d,
                        uint64_t& nextId) {
    auto add = [&](const char* name, const char* desc,
                   ValueType vt, const SchemaRef& schema, uint32_t slot,
                   ReadFn rd, WriteFn wr) {
        ParamEntry p;
        p.id = nextId++;
        p.name = name;
        p.description = desc;
        p.group = "ec.dc";
        p.valueType = vt;
        p.schema = schema;
        p.schemaSlot = slot;
        p.readFn = rd;
        p.writeFn = wr;
        return reg.addParam(p);
    };
    const auto rdU8 = [](void* dst) {
        const uint8_t v = 1;   /* link up whenever the server runs */
        std::memcpy(dst, &v, 1);
    };
    if (!add("ec.dc.connected", "Simulated EtherCAT master link (1=up)",
             ValueType::U8, d.u8, d.slotU8, rdU8, nullptr))
        return false;
    const auto rdEn = [](void* dst) {
        const uint8_t v = g_dcSyncEnabled.load(std::memory_order_relaxed);
        std::memcpy(dst, &v, 1);
    };
    const auto wrEn = [](const void* src) {
        uint8_t v = 0;
        std::memcpy(&v, src, 1);
        g_dcSyncEnabled.store(v ? 1 : 0, std::memory_order_relaxed);
    };
    if (!add("ec.dc.sync_enabled",
             "Simulated DC sync enable (RW, 1=enabled)",
             ValueType::U8, d.u8, d.slotU8, rdEn, wrEn))
        return false;
    const auto rdDiff = [](void* dst) {
        /* deterministic +/-1000 ns wobble on the sim clock */
        const double diff =
            double(int64_t((simNowUs() / 1000) % 2001)) - 1000.0;
        std::memcpy(dst, &diff, 8);
    };
    if (!add("ec.dc.time_diff_ns",
             "Simulated internal-vs-external clock diff, ns (F64)",
             ValueType::F64, d.f64, d.slotF64, rdDiff, nullptr))
        return false;
    const auto rdRef = [](void* dst) {
        const uint64_t t = simNowUs() * 1000;   /* ns */
        std::memcpy(dst, &t, 8);
    };
    if (!add("ec.dc.ref_time_ns",
             "Simulated reference (external) clock, ns since epoch-ish",
             ValueType::U64, d.u64, d.slotU64, rdRef, nullptr))
        return false;
    return true;
}

/* ec.coediag.* params: simulated CoE 0x10F3 Diagnosis History for the
   TetherPLC FB_CoE_DiagHistory_Read surface.  A real master would
   source these from per-slave SDO reads of object 0x10F3; this server
   synthesizes deterministic entries.

   Params:
    - ec.coediag.slave   U16 RW — slave port address selector
    - ec.coediag.history Binary RO — object dump for the selected slave:
        u8 max_messages, u8 newest, u8 newest_acked,
        u8 new_msgs_available, u16 flags, u16 reserved,
        then `newest` records of 22 bytes each (ETG.1020 DiagMessage,
        little-endian): u32 diag_code, u16 flags (bits0-1 severity
        1=info/2=warn/3=err, bit15 emergency), u16 text_id,
        u64 timestamp_ns, u8 params[6].  Message numbers are 1-based
        in wire order (oldest first).
    - ec.coediag.ack     U8 RW — write sets newest_acked for the
      selected slave (clamped to 0..16). */
std::atomic<uint16_t> g_diagSlave{0};
std::array<std::atomic<uint8_t>, 16> g_diagAcked{};

uint8_t diagEntryCount(uint16_t slave) {
    if (slave == 0) return 3;
    if (slave == 1) return 2;
    if (slave == 2) return 1;
    return 0;
}

void putU16(uint8_t* p, uint16_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
}
void putU32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}
void putU64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (8 * i));
}

void diagRecord(uint8_t* r, uint16_t slave, uint8_t idx, uint8_t count) {
    /* deterministic per-slave entries; newest message last */
    const uint8_t pos = count - 1 - idx;   /* 0 = newest */
    uint32_t code = 0x00001000u + slave * 0x100u + idx;
    uint16_t sev = 1, flags = 0, tid = 0x1100u + idx;
    uint16_t mod = 0, ch = 0;
    if (pos == 0) { sev = 3; flags = 0x8000; code = 0xE0008105u;
                    tid = 0x8105; mod = 1; ch = 2; }
    else if (pos == 1 && count > 1) { sev = 2; tid = 0x1200u + idx; }
    flags |= sev;
    putU32(r + 0, code);
    putU16(r + 4, flags);
    putU16(r + 6, tid);
    /* boot-relative ns: (idx+1) s per message, 1 h offset per slave —
       keeps ms values small and exactly verifiable */
    putU64(r + 8, uint64_t(idx + 1) * 1000000000ull +
              uint64_t(slave) * 3600000000000ull);
    std::memset(r + 16, 0, 6);
    putU16(r + 16, mod);
    putU16(r + 18, ch);
}

bool registerEcCoeDiagParams(Registry& reg, const MotionSchemas& d,
                             uint64_t& nextId) {
    /* default: all but the newest message acknowledged */
    for (uint16_t s = 0; s < 16; ++s) {
        const uint8_t n = diagEntryCount(s);
        g_diagAcked[s].store(n > 0 ? n - 1 : 0,
                             std::memory_order_relaxed);
    }
    auto add = [&](const char* name, const char* desc,
                   ValueType vt, const SchemaRef& schema, uint32_t slot,
                   ReadFn rd, WriteFn wr) {
        ParamEntry p;
        p.id = nextId++;
        p.name = name;
        p.description = desc;
        p.group = "ec.coediag";
        p.valueType = vt;
        p.schema = schema;
        p.schemaSlot = slot;
        p.readFn = rd;
        p.writeFn = wr;
        return reg.addParam(p);
    };

    const auto rdSlave = [](void* dst) {
        const uint16_t v = g_diagSlave.load(std::memory_order_relaxed);
        std::memcpy(dst, &v, 2);
    };
    const auto wrSlave = [](const void* src) {
        uint16_t v = 0;
        std::memcpy(&v, src, 2);
        g_diagSlave.store(v, std::memory_order_relaxed);
    };
    if (!add("ec.coediag.slave",
             "CoE diag-history slave selector (port address, RW)",
             ValueType::U16, d.u16, d.slotU16, rdSlave, wrSlave))
        return false;

    ParamEntry hist;
    hist.id = nextId++;
    hist.name = "ec.coediag.history";
    hist.description =
        "CoE 0x10F3 diag history dump for ec.coediag.slave "
        "(header + 22-byte ETG.1020 records)";
    hist.group = "ec.coediag";
    hist.valueType = ValueType::Binary;
    hist.maxValueSize = 512;
    hist.varReadFn = [](void* dst, size_t maxLen) -> size_t {
        const uint16_t slave = g_diagSlave.load(std::memory_order_relaxed);
        const uint8_t n = diagEntryCount(slave);
        const uint8_t acked =
            g_diagAcked[slave & 15].load(std::memory_order_relaxed);
        uint8_t* p = static_cast<uint8_t*>(dst);
        if (maxLen < 8 + size_t(n) * 22) return 0;
        p[0] = 16;                 /* max_messages */
        p[1] = n;                  /* newest */
        p[2] = acked > n ? n : acked;  /* newest_acked */
        p[3] = n > p[2] ? 1 : 0;    /* new_msgs_available */
        putU16(p + 4, 0);          /* flags */
        putU16(p + 6, 0);
        for (uint8_t i = 0; i < n; ++i)
            diagRecord(p + 8 + size_t(i) * 22, slave, i, n);
        return 8 + size_t(n) * 22;
    };
    if (!reg.addParam(hist)) return false;

    const auto rdAck = [](void* dst) {
        const uint16_t s = g_diagSlave.load(std::memory_order_relaxed);
        const uint8_t v = g_diagAcked[s & 15].load(std::memory_order_relaxed);
        std::memcpy(dst, &v, 1);
    };
    const auto wrAck = [](const void* src) {
        uint8_t v = 0;
        std::memcpy(&v, src, 1);
        const uint16_t s = g_diagSlave.load(std::memory_order_relaxed);
        const uint8_t n = diagEntryCount(s);
        g_diagAcked[s & 15].store(v > n ? n : v,
                                  std::memory_order_relaxed);
    };
    return add("ec.coediag.ack",
               "Newest-acked diag message for selected slave (U8 RW)",
               ValueType::U8, d.u8, d.slotU8, rdAck, wrAck);
}

/* ec.soe.* params: simulated Sercos-over-EtherCAT (SoE) mailbox for
   the TetherPLC FB_SoE* surface (Tc2_Drive general SoE + AX5000
   blocks).  A real master would bridge these to the drive's SoE
   mailbox services (S-0-xxxx); this server synthesizes deterministic
   per-slave state.

   Params:
    - ec.soe.slave  U16 RW — drive (port address) selector
    - ec.soe.state  Binary RO — 128-byte read snapshot, LE:
        u16 status        S-0-0135 drive status word
                           (bit0 ready, bit1 enabled, bit2 fault,
                           bit3 warning)
        u16 cmd_state     command state (0 idle, 1 running,
                           2 done, 3 error)
        u32 password      S-0-0267 readback
        u16 diag_num      diagnosis number
        u16 classx        class-X diag flags
        u16 diag_count    valid entries in diag_list
        u16 diag_list[8]  diagnosis number list
        f32 amp_temp      amplifier temperature
        f32 motor_temp    motor temperature
        f32 dc_volt       DC bus voltage
        f32 dc_curr       DC bus current
        f32 ax_mains      AX5000 P-0-0200 mains voltage
        u16 ax_ctrl       AX5000 motor ctrl word readback
        u16 ax_park       AX5000 park state (0/1)
        u16 ax_fw_rev     AX5000 firmware revision
        u16 ax_fw_build   AX5000 firmware build
        u16 cmd_ctrl      last command-control write readback
        char diagmsg[64]  S-0-0095 Sercos string (NUL-terminated)
        u32 reserved
    - ec.soe.cmd    Binary RW — 16-byte command mailbox, LE:
        u16 op            1=password(u32@4) 2=cmd_ctrl(u16@4)
                          3=cmd_exec(u16 command id @4; id bit15 set
                          -> error) 4=reset 5=ax_ctrl(u16@4)
                          6=ax_park(u8@4)
        u16 reserved
        u32 payload
        u32 aux
      cmd_exec records the sim time; cmd_state reads 1 (running)
      for the first 100 ms then 2 (done), or 3 (error) plus a fault
      bit in status when the command id has bit15 set; op 4=reset
      clears the fault bit. */
std::atomic<uint16_t> g_soeSlave{0};
struct SoeSlaveState {
    std::atomic<uint32_t> password{0};
    std::atomic<uint16_t> cmdCtrl{0};
    std::atomic<uint16_t> axCtrl{0};
    std::atomic<uint16_t> axPark{0};
    std::atomic<uint64_t> execUs{0};
    std::atomic<uint16_t> execId{0};
    std::atomic<uint8_t> execPend{0};
    std::atomic<uint8_t> fault{0};
};
std::array<SoeSlaveState, 16> g_soe{};

void putF32(uint8_t* p, float v) { std::memcpy(p, &v, 4); }

void soeDiag(uint16_t slave, char* msg, size_t cap, uint16_t& num,
             uint16_t& classx, uint16_t* list, uint16_t& count,
             float& at, float& mt, float& dv, float& dc) {
    count = 0; num = 0; classx = 0;
    if (cap) msg[0] = 0;
    at = mt = dv = dc = 0.0f;
    if (slave == 0) {
        std::snprintf(msg, cap, "OVERLOAD");
        num = 0x8105; classx = 0x0003;
        list[0] = 0x8105; list[1] = 0x1200; list[2] = 0x2201;
        count = 3;
        at = 61.5f; mt = 58.25f; dv = 565.5f; dc = 12.75f;
    } else if (slave == 1) {
        std::snprintf(msg, cap, "OVERTEMP");
        num = 0x8201; classx = 0x0001;
        list[0] = 0x8201; count = 1;
        at = 72.0f; mt = 80.5f; dv = 540.0f; dc = 3.5f;
    } else if (slave == 2) {
        std::snprintf(msg, cap, "NMT_STATE_CHANGE");
        num = 0x0001; classx = 0x0000;
        list[0] = 0x0001; count = 1;
        at = 45.0f; mt = 44.0f; dv = 560.0f; dc = 0.5f;
    }
}

uint16_t soeStatus(uint16_t slave) {
    uint16_t s = 0x0003;   /* ready + enabled */
    if (g_soe[slave & 15].fault.load(std::memory_order_relaxed))
        s |= 0x0004;
    return s;
}

uint16_t soeCmdState(uint16_t slave) {
    const auto& st = g_soe[slave & 15];
    if (!st.execPend.load(std::memory_order_relaxed)) return 0;
    const uint16_t id = st.execId.load(std::memory_order_relaxed);
    if (id & 0x8000u) return 3;   /* error latched until reset */
    const uint64_t exec = st.execUs.load(std::memory_order_relaxed);
    const uint64_t now = driveNowUs();
    if (now >= exec && now - exec < 100000) return 1;  /* running 100 ms */
    return 2;                                          /* done */
}

void soeCmdWrite(uint16_t slave, const uint8_t* p, size_t len) {
    if (len < 8) return;
    const uint16_t op = uint16_t(p[0]) | (uint16_t(p[1]) << 8);
    const uint32_t v32 = uint32_t(p[4]) | (uint32_t(p[5]) << 8) |
                         (uint32_t(p[6]) << 16) | (uint32_t(p[7]) << 24);
    auto& st = g_soe[slave & 15];
    switch (op) {
    case 1: st.password.store(v32, std::memory_order_relaxed); break;
    case 2:
        st.cmdCtrl.store(uint16_t(v32 & 0xFFFF),
                         std::memory_order_relaxed);
        break;
    case 3: {
        const uint16_t id = uint16_t(v32 & 0xFFFF);
        st.execUs.store(driveNowUs(), std::memory_order_relaxed);
        st.execId.store(id, std::memory_order_relaxed);
        st.execPend.store(1, std::memory_order_relaxed);
        if (id & 0x8000u)
            st.fault.store(1, std::memory_order_relaxed);
        break;
    }
    case 4:
        st.fault.store(0, std::memory_order_relaxed);
        st.execPend.store(0, std::memory_order_relaxed);
        break;
    case 5: st.axCtrl.store(uint16_t(v32 & 0xFFFF),
                            std::memory_order_relaxed); break;
    case 6: st.axPark.store(uint8_t(v32 & 0xFF),
                            std::memory_order_relaxed); break;
    default: break;
    }
}

bool registerEcSoeParams(Registry& reg, const MotionSchemas& d,
                         uint64_t& nextId) {
    auto add = [&](const char* name, const char* desc,
                   ValueType vt, const SchemaRef& schema, uint32_t slot,
                   ReadFn rd, WriteFn wr) {
        ParamEntry p;
        p.id = nextId++;
        p.name = name;
        p.description = desc;
        p.group = "ec.soe";
        p.valueType = vt;
        p.schema = schema;
        p.schemaSlot = slot;
        p.readFn = rd;
        p.writeFn = wr;
        return reg.addParam(p);
    };

    const auto rdSlave = [](void* dst) {
        const uint16_t v = g_soeSlave.load(std::memory_order_relaxed);
        std::memcpy(dst, &v, 2);
    };
    const auto wrSlave = [](const void* src) {
        uint16_t v = 0;
        std::memcpy(&v, src, 2);
        g_soeSlave.store(v, std::memory_order_relaxed);
    };
    if (!add("ec.soe.slave",
             "SoE drive selector (port address, RW)",
             ValueType::U16, d.u16, d.slotU16, rdSlave, wrSlave))
        return false;

    ParamEntry state;
    state.id = nextId++;
    state.name = "ec.soe.state";
    state.description =
        "SoE drive state snapshot for ec.soe.slave "
        "(128-byte LE blob: status/cmd/diag/temps/AX5000 fields)";
    state.group = "ec.soe";
    state.valueType = ValueType::Binary;
    state.maxValueSize = 128;
    state.varReadFn = [](void* dst, size_t maxLen) -> size_t {
        if (maxLen < 128) return 0;
        const uint16_t slave = g_soeSlave.load(std::memory_order_relaxed);
        const uint16_t s = slave & 15;
        uint8_t* p = static_cast<uint8_t*>(dst);
        std::memset(p, 0, 128);
        const uint16_t status = soeStatus(s);
        const uint16_t cmdState = soeCmdState(s);
        p[0] = uint8_t(status); p[1] = uint8_t(status >> 8);
        p[2] = uint8_t(cmdState); p[3] = uint8_t(cmdState >> 8);
        const uint32_t pw =
            g_soe[s].password.load(std::memory_order_relaxed);
        for (int i = 0; i < 4; ++i) p[4 + i] = uint8_t(pw >> (8 * i));
        char msg[64];
        uint16_t num = 0, classx = 0, count = 0;
        uint16_t list[8] = {};
        float at = 0, mt = 0, dv = 0, dc = 0;
        soeDiag(s, msg, sizeof(msg), num, classx, list, count,
                at, mt, dv, dc);
        p[8] = uint8_t(num); p[9] = uint8_t(num >> 8);
        p[10] = uint8_t(classx); p[11] = uint8_t(classx >> 8);
        p[12] = uint8_t(count); p[13] = uint8_t(count >> 8);
        for (uint16_t i = 0; i < count && i < 8; ++i) {
            putU16(p + 14 + i * 2, list[i]);
        }
        putF32(p + 30, at);
        putF32(p + 34, mt);
        putF32(p + 38, dv);
        putF32(p + 42, dc);
        /* AX5000 mains: 230 V + 0.5 V steps on the sim clock */
        const float mains =
            230.0f + float((driveNowUs() / 100000) % 10) * 0.5f;
        putF32(p + 46, mains);
        const uint16_t axc =
            g_soe[s].axCtrl.load(std::memory_order_relaxed);
        putU16(p + 50, axc);
        const uint16_t axp =
            g_soe[s].axPark.load(std::memory_order_relaxed);
        putU16(p + 52, axp);
        putU16(p + 54, 1);    /* fw rev */
        putU16(p + 56, 5);    /* fw build */
        const uint16_t cc =
            g_soe[s].cmdCtrl.load(std::memory_order_relaxed);
        putU16(p + 58, cc);
        std::snprintf(reinterpret_cast<char*>(p + 60), 64, "%s", msg);
        return 128;
    };
    if (!reg.addParam(state)) return false;

    ParamEntry cmd;
    cmd.id = nextId++;
    cmd.name = "ec.soe.cmd";
    cmd.description =
        "SoE command mailbox for ec.soe.slave "
        "(16-byte LE blob: op@0, payload@4, aux@8)";
    cmd.group = "ec.soe";
    cmd.valueType = ValueType::Binary;
    cmd.maxValueSize = 16;
    cmd.varWriteFn = [](const void* src, size_t len) {
        const uint16_t slave = g_soeSlave.load(std::memory_order_relaxed);
        soeCmdWrite(slave, static_cast<const uint8_t*>(src), len);
    };
    if (!reg.addParam(cmd)) return false;
    return true;
}

// ---------------------------------------------------------------------------
// NCI channel — G-code program executed by Tether's interpreter and
// motion planner, streamed as CSP setpoints to the first three axes.
//
// Pipeline: program text -> GCode::PlanningSegmentBuilder (the
// RS274/NGC interpreter emits geometry + feed per block) ->
// MotionPlanner::MotionPlanBuilder (corner blending + velocity profile)
// -> per-tick plan.evaluateAt(planT) -> scaled int32 counts written to
// each axis's 0x607A target. planT advances on the same clock as the
// drives: mc.time writes under --clock sim (deterministic), the 1 ms
// wall ticker under --clock realtime.
//
// nc.* params:
//   nc.prog       String  W  program text -> compile (state=READY/ERROR)
//   nc.prog_path  String  W  server-side file path -> compile
//   nc.cmd        U8      RW channel command (1 run 2 step 3 pause
//                            4 resume 5 stop 6 reset 7 verify)
//   nc.state      U8      R  0 idle 1 ready 2 running 3 paused 4 finished
//                            5 error 6 stopped (InterpreterState values)
//   nc.line       U32     R  source line of the executing block
//   nc.total      U32     R  total parsed blocks
//   nc.errcode    U32     R  channel error (0 ok, 1 bad cmd, 2 bad state,
//                            3 compile, 4 plan build, 5 unsupported seg)
//   nc.progress   F64     R  plan time elapsed, seconds
//   nc.duration   F64     R  total plan duration, seconds
//   nc.scale      F64     RW counts per user unit (default 1.0)
//   nc.feedovr    F64     RW feed override percent (default 100)
// ---------------------------------------------------------------------------

namespace nci {

enum State : uint8_t {
    Idle = 0, Ready = 1, Running = 2, Paused = 3,
    Finished = 4, Error = 5, Stopped = 6
};

enum Cmd : uint8_t {
    CmdRun = 1, CmdStep = 2, CmdPause = 3, CmdResume = 4,
    CmdStop = 5, CmdReset = 6, CmdVerify = 7
};

enum ChanErr : uint32_t {
    ErrNone = 0, ErrBadCmd = 1, ErrBadState = 2, ErrCompile = 3,
    ErrPlan = 4, ErrUnsupported = 5
};

struct Channel {
    mutable std::mutex m;
    State state = Idle;
    std::vector<GCode::PlanningSegment> segs;
    std::vector<GCode::BlockMetadata> blocks;
    /* planner-segment index -> index into segs (zero-length planning
       segments are dropped before the plan is built) */
    std::vector<int> planSegToSeg;
    std::unique_ptr<MotionPlanner::MotionPlan<3, double>> plan;
    double planT = 0.0;
    double duration = 0.0;
    double scale = 1.0;
    double feedOvr = 100.0;
    uint64_t lastUs = 0;
    uint32_t errcode = 0;
    uint32_t curLine = 0;
    uint8_t lastCmd = 0;
    int curSeg = 0;
    int stepTarget = -1;   /* single-step pause target, or -1 */
    SimulatedCiA402Drive* drive = nullptr;
    int axes = 0;   // configured axis count; the channel drives 0..2

    /* FIFO block queue: each nc.fifo write appends one block text;
       when the active plan finishes the next entry is compiled as a
       continuation anchored at the last commanded position. */
    std::deque<std::string> fifoQ;
    uint32_t fifoSeq = 0;        /* entries consumed since program start */
    /* M-function handshake: when the executing block carries an
       M-code outside the program-control set, the channel pauses with
       pendingM set until nc.confirm clears it. */
    uint32_t pendingM = 0;
    int mScan = 0;               /* blocks[] index scanned to */
    /* blocks[] index of the executing segment comes straight from
       PlanningSegment.blockIndex (the builder maps it via source
       line). */
    /* Injected prefix lines (R-param assigns + G92 zero-shift) — kept
       out of nc.total/nc.line so the PLC sees program line numbers. */
    int prefixLines = 0;
    std::unordered_map<int32_t, double> rparams;
    double zsx = 0.0, zsy = 0.0, zsz = 0.0;
    uint32_t ridx = 0;           /* selected R-param index */
    std::array<double, 3> lastPos{0.0, 0.0, 0.0};

    /* Extract the first M<number> from a block text, or -1. */
    static int mcodeOf(const std::string& s) {
        for (size_t i = 0; i < s.size(); ++i) {
            if (std::toupper(static_cast<unsigned char>(s[i])) != 'M') continue;
            size_t j = i + 1;
            int v = 0;
            while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j])))
                v = v * 10 + (s[j++] - '0');
            if (j > i + 1) return v;
        }
        return -1;
    }
    /* M-codes the interpreter owns (program control): not handshake. */
    static bool mcodeHandshake(int m) {
        return m > 1 && m != 2 && m != 30;
    }

    /* Compile program text into planning segments. READY on success.
       prefix is injected BEFORE the program (R-params + G92) and its
       line count recorded so nc.line stays in program coordinates. */
    bool compile(const std::string& text, const std::string& prefix = {}) {
        const std::string full = prefix.empty() ? text : prefix + text;
        auto r = GCode::PlanningSegmentBuilder::fromText(full);
        std::lock_guard lk(m);
        if (!r.error.ok() && !r.error.isEnd()) {
            errcode = static_cast<uint32_t>(r.error.code);
            segs.clear(); blocks.clear(); plan.reset();
            planT = duration = 0.0; curLine = 0;
            state = Error;
            return false;
        }
        prefixLines = 0;
        if (!prefix.empty()) {
            std::istringstream ps(prefix);
            std::string pl;
            while (std::getline(ps, pl)) {
                bool content = false;
                for (char c : pl)
                    if (!std::isspace(static_cast<unsigned char>(c))) { content = true; break; }
                if (content) ++prefixLines;
            }
        }
        /* Drop the injected blocks so blocks[] holds only program
           blocks and line numbers stay in program coordinates. */
        if (prefixLines > 0 && prefixLines <= static_cast<int>(r.blocks.size()))
            r.blocks.erase(r.blocks.begin(),
                           r.blocks.begin() + prefixLines);
        segs = std::move(r.segments);
        blocks = std::move(r.blocks);
        for (size_t i = 0; i < blocks.size(); ++i)
            blocks[i].blockIndex = static_cast<int>(i);
        errcode = ErrNone; plan.reset();
        planT = duration = 0.0; curLine = 0; curSeg = 0;
        mScan = 0; pendingM = 0; stepTarget = -1;
        state = Ready;
        return true;
    }

    /* Enable the channel axes (slaves 0..2) in CSP + OperationEnabled. */
    void enableAxes() {
        for (uint16_t s = 0; s < 3 && s < axes; ++s) {
            drive->write(s, 0x6060, 0, {0x08});
            drive->write(s, 0x6040, 0, {0x0F, 0x00});
        }
    }

    /* PlanningSegments -> planner segments -> MotionPlan. */
    bool buildPlan() {
        namespace mp = MotionPlanner;
        mp::MotionSegmentList list;
        planSegToSeg.clear();
        for (size_t si = 0; si < segs.size(); ++si) {
            const auto& s = segs[si];
            const mp::Vec3 st{s.start.x(), s.start.y(), s.start.z()};
            const mp::Vec3 en{s.end.x(), s.end.y(), s.end.z()};
            const double feed = s.feedRate / 60.0;   // units/min -> units/s
            const bool moved =
                std::abs(en[0] - st[0]) > 1e-9 ||
                std::abs(en[1] - st[1]) > 1e-9 ||
                std::abs(en[2] - st[2]) > 1e-9;
            /* The path builder rejects zero-length segments; a move
               whose start equals its end contributes no path. */
            if (!moved && s.motionType != GCode::SegmentMotionType::Dwell)
                continue;
            switch (s.motionType) {
                case GCode::SegmentMotionType::Rapid:
                    list.append(mp::MotionSegment::rapid<3>(st, en));
                    break;
                case GCode::SegmentMotionType::Linear:
                    list.append(mp::MotionSegment::linear<3>(st, en, feed));
                    break;
                case GCode::SegmentMotionType::ArcCW:
                case GCode::SegmentMotionType::ArcCCW: {
                    const mp::Vec3 c{s.center.x(), s.center.y(), s.center.z()};
                    const auto pl = static_cast<mp::ArcPlane>(
                        static_cast<uint8_t>(s.plane));
                    list.append(s.motionType == GCode::SegmentMotionType::ArcCW
                        ? mp::MotionSegment::arcCW<3>(st, en, c, feed, pl)
                        : mp::MotionSegment::arcCCW<3>(st, en, c, feed, pl));
                    break;
                }
                case GCode::SegmentMotionType::Dwell: {
                    std::array<double, 9> pos{};
                    pos[0] = s.end.x(); pos[1] = s.end.y(); pos[2] = s.end.z();
                    list.append(mp::MotionSegment::dwell(s.segmentTime, pos));
                    break;
                }
                default:
                    errcode = ErrUnsupported;
                    state = Error;
                    return false;
            }
            planSegToSeg.push_back(static_cast<int>(si));
        }
        if (list.empty()) {
            /* No path segments — a program without motion completes
               instantly on RUN. */
            plan.reset();
            duration = 0.0;
            return true;
        }
        mp::KinematicLimits<3, double> lim;
        mp::MotionPlanBuilder<3, double> b(lim);
        plan = std::make_unique<mp::MotionPlan<3, double>>(
            b.build(list, lim.path.maxPathVelocity));
        duration = plan->totalDuration();
        if (duration <= 0.0) {
            errcode = ErrPlan;
            state = Error;
            plan.reset();
            return false;
        }
        return true;
    }

    /* Write the plan position at planT as scaled 0x607A targets. */
    void emitTargets() {
        if (!drive || axes < 3 || !plan) return;
        const auto st = plan->evaluateAt(planT);
        for (uint16_t ax = 0; ax < 3; ++ax) {
            const int32_t counts = static_cast<int32_t>(
                std::lround(st.position[ax] * scale));
            const uint8_t le[4] = {
                uint8_t(counts), uint8_t(counts >> 8),
                uint8_t(counts >> 16), uint8_t(counts >> 24)};
            drive->write(ax, 0x607A, 0,
                         std::vector<uint8_t>(le, le + 4));
        }
        lastPos = {st.position[0], st.position[1], st.position[2]};
        curSeg = static_cast<int>(st.segmentIndex);
        if (st.segmentIndex < planSegToSeg.size()) {
            const int si = planSegToSeg[st.segmentIndex];
            const int32_t bi = segs[si].blockIndex;
            if (bi >= 0 && bi < static_cast<int32_t>(blocks.size()))
                curLine = static_cast<uint32_t>(blocks[bi].lineNumber);
            scanMFuncs(bi);
        }
    }

    /* Scan blocks up to and including the executing one; a block
       carrying a handshake M-code pauses the channel with pendingM
       set until nc.confirm. */
    void scanMFuncs(int32_t upto) {
        if (pendingM) return;
        const int n = static_cast<int>(blocks.size());
        if (upto >= n) upto = n - 1;
        for (; mScan <= upto && mScan < n; ++mScan) {
            const int m = mcodeOf(blocks[mScan].gcodeText);
            if (m >= 0 && mcodeHandshake(m)) {
                pendingM = static_cast<uint32_t>(m);
                state = Paused;
                ++mScan;
                return;
            }
        }
    }

    void command(uint8_t cmd) {
        std::lock_guard lk(m);
        lastCmd = cmd;
        switch (cmd) {
            case CmdRun:
                /* FIFO-only program: queued blocks with nothing
                   loaded run straight from the queue. */
                if (state == Idle && !fifoQ.empty()) {
                    if (axes < 3) { errcode = ErrBadState; return; }
                    if (!advanceFifo()) { errcode = ErrPlan; return; }
                    enableAxes();
                    emitTargets();
                    state = pendingM ? Paused : Running;
                    errcode = ErrNone;
                    return;
                }
                if (state != Ready && state != Stopped && state != Finished) {
                    errcode = ErrBadState;
                    return;
                }
                if (axes < 3) { errcode = ErrBadState; return; }
                if (!plan && !segs.empty() && !buildPlan()) return;
                planT = 0.0;
                if (!plan) {
                    /* No motion segments — program done immediately. */
                    state = Finished;
                    errcode = ErrNone;
                    return;
                }
                enableAxes();
                emitTargets();
                stepTarget = -1;
                /* A handshake M-code on the first block holds. */
                state = pendingM ? Paused : Running;
                errcode = ErrNone;
                return;
            case CmdStep:
                /* Single-block step: run until the active planner
                   segment advances past the current one, then pause.
                   Starts a program from Ready/Stopped/Finished or
                   advances one block from Paused. */
                if (state != Ready && state != Stopped &&
                    state != Finished && state != Paused) {
                    errcode = ErrBadState;
                    return;
                }
                if (axes < 3) { errcode = ErrBadState; return; }
                if (state != Paused) {
                    if (!plan && !segs.empty() && !buildPlan()) return;
                    planT = 0.0;
                    if (!plan) {
                        state = Finished;
                        errcode = ErrNone;
                        return;
                    }
                    enableAxes();
                }
                emitTargets();
                stepTarget = pendingM ? -1 : curSeg + 1;
                state = pendingM ? Paused : Running;
                errcode = ErrNone;
                return;
            case CmdPause:
                if (state == Running) { state = Paused; errcode = ErrNone; }
                else errcode = ErrBadState;
                return;
            case CmdResume:
                if (state == Paused) { state = Running; errcode = ErrNone; }
                else errcode = ErrBadState;
                return;
            case CmdStop:
                /* bStop is permissive: any active or completed state
                   drops to Stopped; only Idle/Error with nothing
                   loaded is a bad state. */
                if (state == Idle || state == Error) {
                    errcode = ErrBadState;
                } else {
                    state = Stopped; planT = 0.0; errcode = ErrNone;
                }
                return;
            case CmdReset:
                segs.clear(); blocks.clear(); plan.reset();
                fifoQ.clear(); fifoSeq = 0;
                mScan = 0; pendingM = 0; stepTarget = -1;
                planT = duration = 0.0; curLine = 0; errcode = ErrNone;
                state = Idle;
                return;
            case CmdVerify:
                errcode = (state == Ready) ? ErrNone : ErrBadState;
                return;
            default:
                errcode = ErrBadCmd;
                return;
        }
    }

    /* Advance plan time on the drive clock and stream setpoints. */
    void tick(uint64_t nowUs) {
        std::lock_guard lk(m);
        double dt = 0.0;
        if (lastUs && nowUs > lastUs)
            dt = static_cast<double>(nowUs - lastUs) / 1e6;
        lastUs = nowUs;
        if (state != Running || !plan) return;
        planT += dt * feedOvr / 100.0;
        if (planT >= duration) {
            planT = duration;
            emitTargets();
            stepTarget = -1;
            /* FIFO: chain the next queued block as a continuation
               anchored at the last commanded position. */
            if (advanceFifo()) return;
            state = Finished;
            return;
        }
        emitTargets();
        if (pendingM) { state = Paused; return; }
        if (stepTarget >= 0 && curSeg >= stepTarget) {
            stepTarget = -1;
            state = Paused;
        }
    }

    /* Compile the next FIFO entry as a continuation plan anchored at
       lastPos; returns true when a new plan is now Running. */
    bool advanceFifo() {
        if (fifoQ.empty()) {
            /* Tail blocks (M-codes after the last motion block) still
               need scanning before the program can finish. */
            scanMFuncs(static_cast<int>(blocks.size()));
            return false;
        }
        char anchor[96];
        std::snprintf(anchor, sizeof(anchor),
                      "G21 G90\nG1 X%.6f Y%.6f Z%.6f\n",
                      lastPos[0], lastPos[1], lastPos[2]);
        const std::string entry = fifoQ.front();
        fifoQ.pop_front();
        ++fifoSeq;
        /* compile() takes the lock; it resets segs/blocks/plan. */
        const std::string prefix = anchor;
        {
            auto r = GCode::PlanningSegmentBuilder::fromText(prefix + entry);
            if (!r.error.ok() && !r.error.isEnd()) {
                errcode = static_cast<uint32_t>(r.error.code);
                segs.clear(); blocks.clear(); plan.reset();
                state = Error;
                return false;
            }
            /* Drop the two injected blocks (mode line + anchor). */
            if (r.blocks.size() > 2)
                r.blocks.erase(r.blocks.begin(), r.blocks.begin() + 2);
            segs = std::move(r.segments);
            blocks = std::move(r.blocks);
            for (size_t i = 0; i < blocks.size(); ++i)
                blocks[i].blockIndex = static_cast<int>(i);
            plan.reset(); planT = 0.0; curSeg = 0; mScan = 0;
            stepTarget = -1;
        }
        if (!buildPlan()) return false;
        state = Running;
        return true;
    }

    /* Queue a FIFO block. Trial-parses first so a bad entry reports
       the compile error immediately instead of poisoning the run. */
    bool appendFifo(const std::string& entry) {
        auto r = GCode::PlanningSegmentBuilder::fromText("G21 G90\n" + entry);
        if (!r.error.ok() && !r.error.isEnd()) {
            std::lock_guard lk(m);
            errcode = static_cast<uint32_t>(r.error.code);
            return false;
        }
        std::lock_guard lk(m);
        fifoQ.push_back(entry);
        errcode = ErrNone;
        return true;
    }

    /* Confirm a pending M-function; resumes a held channel. */
    void confirm() {
        std::lock_guard lk(m);
        if (!pendingM) { errcode = ErrBadState; return; }
        pendingM = 0;
        if (state == Paused) state = Running;
        errcode = ErrNone;
    }

    /* Compile prefix carrying the R-parameter registers and the
       zero-shift: the interpreter genuinely evaluates both, so the
       injected lines count against blocks[]/line numbers. */
    std::string progPrefix() const {
        std::ostringstream p;
        for (const auto& [n, v] : rparams)
            p << "#" << n << " = " << v << "\n";
        if (zsx != 0.0 || zsy != 0.0 || zsz != 0.0) {
            char b[96];
            std::snprintf(b, sizeof(b), "G92 X%.6f Y%.6f Z%.6f\n",
                          zsx, zsy, zsz);
            p << b;
        }
        return p.str();
    }
};

Channel g_nci;

void nciTick(uint64_t nowUs) { g_nci.tick(nowUs); }

void regU8(Registry& reg, const MotionSchemas& d, uint64_t& nextId,
           const char* name, const char* desc, bool rw,
           std::function<void(void*)> rd, std::function<void(const void*)> wr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "nc";
    p.valueType = ValueType::U8;
    p.schema = d.u8;
    p.schemaSlot = d.slotU8;
    p.readFn = std::move(rd);
    if (rw) p.writeFn = std::move(wr);
    reg.addParam(p);
}

void regU32(Registry& reg, const MotionSchemas& d, uint64_t& nextId,
            const char* name, const char* desc,
            std::function<void(void*)> rd,
            std::function<void(const void*)> wr = nullptr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "nc";
    p.valueType = ValueType::U32;
    p.schema = d.u32;
    p.schemaSlot = d.slotU32;
    p.readFn = std::move(rd);
    if (wr) p.writeFn = std::move(wr);
    reg.addParam(p);
}

void regF64(Registry& reg, const MotionSchemas& d, uint64_t& nextId,
            const char* name, const char* desc, bool rw,
            std::function<void(void*)> rd,
            std::function<void(const void*)> wr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "nc";
    p.valueType = ValueType::F64;
    p.schema = d.f64;
    p.schemaSlot = d.slotF64;
    p.readFn = std::move(rd);
    if (rw) p.writeFn = std::move(wr);
    reg.addParam(p);
}

void regStr(Registry& reg, uint64_t& nextId, const char* name,
            const char* desc, std::function<size_t(void*, size_t)> rd,
            std::function<void(const void*, size_t)> wr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "nc";
    p.valueType = ValueType::String;
    p.maxValueSize = 65535;
    p.varReadFn = std::move(rd);
    if (wr) p.varWriteFn = std::move(wr);
    reg.addParam(p);
}

/* Register all nc.* channel params. */
bool registerNciParams(Registry& reg, const MotionSchemas& d,
                       uint64_t& nextId) {
    auto u8rd = [](auto fn) {
        return [fn](void* dst) {
            std::lock_guard lk(g_nci.m);
            const uint8_t v = static_cast<uint8_t>(fn());
            std::memcpy(dst, &v, 1);
        };
    };
    auto u32rd = [](auto fn) {
        return [fn](void* dst) {
            std::lock_guard lk(g_nci.m);
            const uint32_t v = fn();
            std::memcpy(dst, &v, 4);
        };
    };
    auto f64rd = [](auto fn) {
        return [fn](void* dst) {
            std::lock_guard lk(g_nci.m);
            const double v = fn();
            std::memcpy(dst, &v, 8);
        };
    };

    regStr(reg, nextId, "nc.prog",
           "G-code program text; write compiles it (state READY/ERROR)",
           [](void* dst, size_t maxLen) -> size_t { (void)dst; (void)maxLen; return 0; },
           [](const void* src, size_t len) {
               g_nci.compile(std::string(static_cast<const char*>(src), len),
                             g_nci.progPrefix());
           });

    regStr(reg, nextId, "nc.prog_path",
           "server-side G-code file path; write loads and compiles it",
           [](void* dst, size_t maxLen) -> size_t { (void)dst; (void)maxLen; return 0; },
           [](const void* src, size_t len) {
               const std::string path(static_cast<const char*>(src), len);
               std::ifstream f(path);
               if (!f) {
                   std::lock_guard lk(g_nci.m);
                   g_nci.errcode = nci::ErrCompile;
                   g_nci.state = nci::Error;
                   return;
               }
               std::ostringstream ss; ss << f.rdbuf();
               g_nci.compile(ss.str(), g_nci.progPrefix());
           });

    regU8(reg, d, nextId, "nc.cmd",
          "channel command: 1 run 2 step 3 pause 4 resume 5 stop "
          "6 reset 7 verify",
          true,
          [](void* dst) {
              std::lock_guard lk(g_nci.m);
              std::memcpy(dst, &g_nci.lastCmd, 1);
          },
          [](const void* src) {
              uint8_t v;
              std::memcpy(&v, src, 1);
              g_nci.command(v);
          });

    regU8(reg, d, nextId, "nc.state",
          "channel state: 0 idle 1 ready 2 running 3 paused 4 finished "
          "5 error 6 stopped", false,
          u8rd([] { return g_nci.state; }), nullptr);
    regU32(reg, d, nextId, "nc.line", "executing G-code line number",
           u32rd([] { return g_nci.curLine; }));
    regU32(reg, d, nextId, "nc.total", "parsed block count",
           u32rd([] { return static_cast<uint32_t>(g_nci.blocks.size()); }));
    regU32(reg, d, nextId, "nc.errcode", "channel error code",
           u32rd([] { return g_nci.errcode; }));
    regF64(reg, d, nextId, "nc.progress", "plan time elapsed (s)", false,
           f64rd([] { return g_nci.planT; }), nullptr);
    regF64(reg, d, nextId, "nc.duration", "total plan duration (s)", false,
           f64rd([] { return g_nci.duration; }), nullptr);
    regF64(reg, d, nextId, "nc.scale", "counts per user unit", true,
           f64rd([] { return g_nci.scale; }),
           [](const void* src) {
               double v;
               std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m);
               if (v > 0.0) g_nci.scale = v;
           });
    regF64(reg, d, nextId, "nc.feedovr", "feed override percent", true,
           f64rd([] { return g_nci.feedOvr; }),
           [](const void* src) {
               double v;
               std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m);
               g_nci.feedOvr = std::max(0.0, std::min(500.0, v));
           });
    regStr(reg, nextId, "nc.fifo",
           "FIFO block append: one G-code block queued for chained "
           "execution after the active plan",
           [](void* dst, size_t maxLen) -> size_t { (void)dst; (void)maxLen; return 0; },
           [](const void* src, size_t len) {
               g_nci.appendFifo(std::string(static_cast<const char*>(src), len));
           });
    regU32(reg, d, nextId, "nc.fifolen", "queued FIFO block count",
           u32rd([] { return static_cast<uint32_t>(g_nci.fifoQ.size()); }));
    regU32(reg, d, nextId, "nc.fifoseq", "FIFO entries consumed this run",
           u32rd([] { return g_nci.fifoSeq; }));
    regU32(reg, d, nextId, "nc.mfunc", "pending M-code (0 = none)",
           u32rd([] { return g_nci.pendingM; }));
    regU8(reg, d, nextId, "nc.confirm",
          "write 1: confirm pending M-function and resume", true,
          [](void* dst) { std::memset(dst, 0, 1); },
          [](const void*) { g_nci.confirm(); });
    regU32(reg, d, nextId, "nc.ridx",
           "selected R-parameter index (0-5999)",
           u32rd([] { return g_nci.ridx; }),
           [](const void* src) {
               uint32_t v; std::memcpy(&v, src, 4);
               std::lock_guard lk(g_nci.m);
               g_nci.ridx = v;
           });
    regF64(reg, d, nextId, "nc.rval",
           "R-parameter value at nc.ridx (applied on next compile)", true,
           f64rd([] {
               auto it = g_nci.rparams.find(g_nci.ridx);
               return it != g_nci.rparams.end() ? it->second : 0.0;
           }),
           [](const void* src) {
               double v;
               std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m);
               g_nci.rparams[g_nci.ridx] = v;
           });
    regF64(reg, d, nextId, "nc.zsx", "zero-shift X (G92, next compile)",
           true, f64rd([] { return g_nci.zsx; }),
           [](const void* src) { double v; std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m); g_nci.zsx = v; });
    regF64(reg, d, nextId, "nc.zsy", "zero-shift Y (G92, next compile)",
           true, f64rd([] { return g_nci.zsy; }),
           [](const void* src) { double v; std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m); g_nci.zsy = v; });
    regF64(reg, d, nextId, "nc.zsz", "zero-shift Z (G92, next compile)",
           true, f64rd([] { return g_nci.zsz; }),
           [](const void* src) { double v; std::memcpy(&v, src, 8);
               std::lock_guard lk(g_nci.m); g_nci.zsz = v; });
    regF64(reg, d, nextId, "nc.lastx", "last commanded X (user units)",
           false, f64rd([] { return g_nci.lastPos[0]; }), nullptr);
    regF64(reg, d, nextId, "nc.lasty", "last commanded Y (user units)",
           false, f64rd([] { return g_nci.lastPos[1]; }), nullptr);
    regF64(reg, d, nextId, "nc.lastz", "last commanded Z (user units)",
           false, f64rd([] { return g_nci.lastPos[2]; }), nullptr);
    return true;
}

} // namespace nci

/* ------------------------------------------------------------------ */
/* FiFoAxes channel (fifo.* params): synchronous numeric setpoint
   streaming. Each channel owns a FIFO of rows; every row holds one
   LREAL position per integrated axis (group position = column).
   While running, each drive-clock tick pops one row and writes it
   as the axis's 0x607A CSP target — the same mechanism the NC
   channel uses, but driven by explicit PLC writes instead of a
   G-code plan. fi.fo state: 0 idle 1 running 2 finished 3 error. */
namespace fifo {

constexpr uint8_t kMaxChan = 4;
constexpr uint8_t kMaxAxes = 8;
/* 1000 rows * 8 cols * 8 B + 4 B header = 64004 B, within the
   uint16 param maxValueSize limit (65535). */
constexpr uint32_t kMaxRows = 1000;

enum State : uint8_t { Idle = 0, Running = 1, Finished = 2, Error = 3 };
enum Cmd : uint8_t { CmdStart = 1, CmdStop = 2, CmdReset = 3 };

struct Channel {
    mutable std::mutex m;
    State state = Idle;
    int32_t axes[kMaxAxes];       /* slave index per group position, -1 */
    uint32_t naxes = 0;
    std::deque<std::array<double, kMaxAxes>> rows;
    uint32_t seq = 0;             /* rows consumed since start */
    double overridePct = 100.0;
    uint64_t lastUs = 0;          /* last row emission time */
    SimulatedCiA402Drive* drive = nullptr;
    int axisCount = 0;

    void clearAxes() {
        for (auto& a : axes) a = -1;
        naxes = 0;
    }
    Channel() { clearAxes(); }

    /* integrate axis `slave` at group position `gpos` (column). */
    bool integrate(uint32_t slave, uint32_t gpos) {
        if (gpos >= kMaxAxes || slave >= static_cast<uint32_t>(axisCount))
            return false;
        std::lock_guard lk(m);
        axes[gpos] = static_cast<int32_t>(slave);
        uint32_t n = 0;
        for (auto& a : axes) if (a >= 0) ++n;
        naxes = n;
        return true;
    }

    void disintegrate() {
        std::lock_guard lk(m);
        clearAxes();
        rows.clear();
        state = Idle; seq = 0; lastUs = 0;
    }

    /* Append nrows rows; payload = u32 nrows + nrows*kMaxAxes f64 LE
       (fixed 8-column stride; unused columns ignored). `replace`
       clears the queue first (FiFoOverwrite). */
    bool writeRows(const void* src, size_t len, bool replace) {
        if (len < 4) return false;
        uint32_t nrows;
        std::memcpy(&nrows, src, 4);
        const uint8_t* p = static_cast<const uint8_t*>(src) + 4;
        const size_t need = 4 + static_cast<size_t>(nrows) * kMaxAxes * 8;
        if (naxes == 0 || len < need || nrows > kMaxRows) return false;
        std::lock_guard lk(m);
        if (replace) rows.clear();
        if (rows.size() + nrows > kMaxRows) return false;
        for (uint32_t r = 0; r < nrows; ++r) {
            std::array<double, kMaxAxes> row{};
            for (uint32_t c = 0; c < kMaxAxes; ++c) {
                double v;
                std::memcpy(&v, p, 8); p += 8;
                row[c] = v;
            }
            rows.push_back(row);
        }
        if (state == Finished) state = Idle;
        return true;
    }

    void command(uint8_t cmd) {
        std::lock_guard lk(m);
        switch (cmd) {
            case CmdStart:
                if (state == Running || naxes == 0) { state = Error; return; }
                if (rows.empty()) { state = Finished; return; }
                /* CSP enable on every member axis. */
                for (auto& a : axes) {
                    if (a < 0 || !drive) continue;
                    drive->write(static_cast<uint16_t>(a), 0x6060, 0, {0x08});
                    drive->write(static_cast<uint16_t>(a), 0x6040, 0,
                                 {0x0F, 0x00});
                }
                state = Running; seq = 0; lastUs = 0;
                return;
            case CmdStop:
                state = Idle; lastUs = 0; return;
            case CmdReset:
                rows.clear(); seq = 0; state = Idle; lastUs = 0;
                return;
            default: state = Error; return;
        }
    }

    /* One FIFO row per 1 ms of drive-clock time -> 0x607A targets.
       Deterministic under --clock sim: mc.time writes advance
       lastUs and emit floor(dt/1ms) rows (capped by queue depth).
       The override percent is recorded for the PLC status surface;
       setpoint rows are absolute positions so it does not rescale
       them (documented deviation from TwinCAT velocity override). */
    void tick(uint64_t nowUs) {
        std::lock_guard lk(m);
        if (state != Running || !drive) return;
        if (lastUs == 0) { lastUs = nowUs; return; }
        if (nowUs <= lastUs) return;
        const uint64_t dt = nowUs - lastUs;
        uint64_t steps = dt / 1000;
        if (steps == 0) return;
        if (steps > rows.size()) steps = rows.size();
        lastUs += steps * 1000;
        for (uint64_t s = 0; s < steps; ++s) {
            const auto row = rows.front();
            rows.pop_front();
            ++seq;
            for (uint32_t c = 0; c < kMaxAxes; ++c) {
                const int32_t slave = axes[c];
                if (slave < 0) continue;
                const int32_t counts =
                    static_cast<int32_t>(std::lround(row[c]));
                const uint8_t le[4] = {
                    uint8_t(counts), uint8_t(counts >> 8),
                    uint8_t(counts >> 16), uint8_t(counts >> 24)};
                drive->write(static_cast<uint16_t>(slave), 0x607A, 0,
                             std::vector<uint8_t>(le, le + 4));
            }
        }
        if (rows.empty()) state = Finished;
    }
};

Channel g_fifo[kMaxChan];
std::atomic<uint8_t> g_fifoSel{0};   /* fifo.chan selector */

void fifoTick(uint64_t nowUs) {
    for (auto& c : g_fifo) c.tick(nowUs);
}

Channel& curChan() { return g_fifo[g_fifoSel.load() % kMaxChan]; }

void regFifoParam(Registry& reg, const MotionSchemas& d, uint64_t& nextId,
                  const char* name, const char* desc, ValueType vt,
                  const SchemaRef& schema, uint32_t slot,
                  ReadFn rd, WriteFn wr = nullptr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "fifo";
    p.valueType = vt;
    p.schema = schema;
    p.schemaSlot = slot;
    p.readFn = std::move(rd);
    if (wr) p.writeFn = std::move(wr);
    reg.addParam(p);
}

bool registerFifoParams(Registry& reg, const MotionSchemas& d,
                        uint64_t& nextId) {
    auto u8rd = [](auto fn) {
        return [fn](void* dst) {
            const uint8_t v = static_cast<uint8_t>(fn());
            std::memcpy(dst, &v, 1);
        };
    };
    auto u32rd = [](auto fn) {
        return [fn](void* dst) {
            const uint32_t v = fn();
            std::memcpy(dst, &v, 4);
        };
    };
    auto f64rd = [](auto fn) {
        return [fn](void* dst) {
            const double v = fn();
            std::memcpy(dst, &v, 8);
        };
    };

    regFifoParam(reg, d, nextId, "fifo.chan",
                 "FiFo channel selector 0-3", ValueType::U8, d.u8,
                 d.slotU8,
                 u8rd([] {
                     return static_cast<uint8_t>(g_fifoSel.load());
                 }),
                 [](const void* src) {
                     uint8_t v; std::memcpy(&v, src, 1);
                     g_fifoSel.store(v % kMaxChan);
                 });
    regFifoParam(reg, d, nextId, "fifo.cmd",
                 "channel command: 1 start 2 stop 3 reset", ValueType::U8,
                 d.u8, d.slotU8,
                 u8rd([] { return uint8_t{0}; }),
                 [](const void* src) {
                     uint8_t v; std::memcpy(&v, src, 1);
                     curChan().command(v);
                 });
    regFifoParam(reg, d, nextId, "fifo.state",
                 "channel state: 0 idle 1 running 2 finished 3 error",
                 ValueType::U8, d.u8, d.slotU8,
                 u8rd([] {
                     std::lock_guard lk(curChan().m);
                     return static_cast<uint8_t>(curChan().state);
                 }));
    regFifoParam(reg, d, nextId, "fifo.naxes",
                 "integrated axis count", ValueType::U8, d.u8, d.slotU8,
                 u8rd([] {
                     std::lock_guard lk(curChan().m);
                     return static_cast<uint8_t>(curChan().naxes);
                 }));
    regFifoParam(reg, d, nextId, "fifo.maxrows",
                 "FIFO row capacity", ValueType::U32, d.u32, d.slotU32,
                 u32rd([] { return kMaxRows; }));
    regFifoParam(reg, d, nextId, "fifo.queued",
                 "queued row count", ValueType::U32, d.u32, d.slotU32,
                 u32rd([] {
                     std::lock_guard lk(curChan().m);
                     return static_cast<uint32_t>(curChan().rows.size());
                 }));
    regFifoParam(reg, d, nextId, "fifo.seq",
                 "rows consumed since start", ValueType::U32, d.u32,
                 d.slotU32,
                 u32rd([] {
                     std::lock_guard lk(curChan().m);
                     return curChan().seq;
                 }));
    regFifoParam(reg, d, nextId, "fifo.override",
                 "channel override percent (recorded; setpoint rows "
                 "are absolute so the factor does not rescale them)",
                 ValueType::F64, d.f64, d.slotF64,
                 f64rd([] {
                     std::lock_guard lk(curChan().m);
                     return curChan().overridePct;
                 }),
                 [](const void* src) {
                     double v; std::memcpy(&v, src, 8);
                     std::lock_guard lk(curChan().m);
                     curChan().overridePct =
                         std::max(0.0, std::min(500.0, v));
                 });
    /* fifo.integrate: U32 = slave | (groupPos << 16). */
    regFifoParam(reg, d, nextId, "fifo.integrate",
                 "integrate axis: slave | (group position << 16)",
                 ValueType::U32, d.u32, d.slotU32,
                 u32rd([] { return 0; }),
                 [](const void* src) {
                     uint32_t v; std::memcpy(&v, src, 4);
                     curChan().integrate(v & 0xFFFF, v >> 16);
                 });
    /* fifo.disintegrate: any write removes all axes + queued rows. */
    regFifoParam(reg, d, nextId, "fifo.disintegrate",
                 "write any value: remove all axes and queued rows",
                 ValueType::U8, d.u8, d.slotU8,
                 u8rd([] { return 0; }),
                 [](const void*) { curChan().disintegrate(); });
    /* fifo.write / fifo.overwrite: binary blob u32 nrows +
       nrows*8 f64 LE rows (fixed 8-column stride). */
    auto rdQueued = [](void* dst, size_t) -> size_t {
        std::lock_guard lk(curChan().m);
        const uint32_t n = static_cast<uint32_t>(curChan().rows.size());
        std::memcpy(dst, &n, 4);
        return 4;
    };
    ParamEntry w;
    w.id = nextId++;
    w.name = "fifo.write";
    w.description = "append rows: u32 nrows + nrows*8 f64 LE; read = "
                    "queued row count";
    w.group = "fifo";
    w.valueType = ValueType::Binary;
    w.maxValueSize = 65535;
    w.varReadFn = rdQueued;
    w.varWriteFn = [](const void* src, size_t len) {
        curChan().writeRows(src, len, false);
    };
    reg.addParam(w);
    ParamEntry ow = w;
    ow.id = nextId++;
    ow.name = "fifo.overwrite";
    ow.description = "replace queue with rows (same blob as fifo.write)";
    ow.varWriteFn = [](const void* src, size_t len) {
        curChan().writeRows(src, len, true);
    };
    reg.addParam(ow);
    return true;
}

} // namespace fifo

/* Kinematic transformation objects (kin.* params): each object binds a
   configured ForwardKinematicsBase chain (type + preset/config blob)
   to optional ACS (joint) and MCS (Cartesian) axis lists. kin.cmd
   performs synchronous forward/backward transforms: forward maps
   kin.in joints to a Pose6D in kin.out; backward runs numeric
   position IK (kinematics::numericPositionIK) seeded from kin.in[3..]
   toward the kin.in[0..2] target, with the joint solution in kin.q.
   kin.locked gates type/preset/cfg writes. kin.rot holds a 3-axis
   rotation preset; cmd 4/5 save/restore it (extended rotation range). */
namespace kin {

using namespace tether::kinematics;

constexpr uint8_t kMaxObj = 8;
constexpr uint32_t kMaxCfg = 32;    /* f64 config values */
constexpr uint32_t kMaxDim = 8;     /* joints / pose doubles */

enum Type : uint8_t {
    TCartesian = 0, TPlanar2D = 1, TArticulated3 = 2, TSCARA = 3,
    TSerial6 = 4, TSerial7 = 5, TGantry5 = 6, TDelta = 7, TStewart = 8,
};
enum State : uint8_t { StIdle = 0, StReady = 1, StError = 2 };
enum Cmd : uint8_t { CmdFwd = 1, CmdInv = 2, CmdReset = 3,
                     CmdRotSave = 4, CmdRotRestore = 5 };
enum Preset6 : uint8_t { PreUR5 = 0, PreKUKA = 1 };

struct Obj {
    std::unique_ptr<ForwardKinematicsBase> model;
    int32_t type = -1;               /* -1 = unconfigured */
    uint8_t preset = 0;
    std::vector<double> cfg;
    uint8_t locked = 0;
    std::array<double, kMaxDim> in{};
    std::array<double, kMaxDim> out{};
    std::array<double, kMaxDim> q{};
    uint32_t acs[kMaxDim]{};
    int32_t nacs = 0;
    uint32_t mcs[kMaxDim]{};
    int32_t nmcs = 0;
    State state = StIdle;
    uint16_t err = 0;
    std::array<double, 3> rot{};
    std::array<double, 3> rotsaved{};
    uint8_t chn = 0;                 /* NC channel id (kinematic group) */
};

Obj g_obj[kMaxObj];
std::atomic<uint8_t> g_objSel{0};
std::mutex g_objMtx;

Obj& curObj() { return g_obj[g_objSel.load() % kMaxObj]; }

static float cfgAt(const Obj& o, size_t i, float dflt) {
    return i < o.cfg.size() ? float(o.cfg[i]) : dflt;
}

/* (Re)build the FK model from type/preset/cfg. */
bool rebuild(Obj& o) {
    o.model.reset();
    if (o.type < 0) { o.state = StIdle; return false; }
    switch (o.type) {
        case TCartesian:   o.model = std::make_unique<CartesianRobot>(); break;
        case TPlanar2D:    o.model = std::make_unique<Planar2DOF>(
                                cfgAt(o,0,0.5f), cfgAt(o,1,0.5f)); break;
        case TArticulated3:o.model = std::make_unique<Articulated3DOF>(
                                cfgAt(o,0,0.3f), cfgAt(o,1,0.4f),
                                cfgAt(o,2,0.3f)); break;
        case TSCARA:       o.model = std::make_unique<SCARA>(
                                cfgAt(o,0,0.35f), cfgAt(o,1,0.35f),
                                cfgAt(o,2,0.4f), cfgAt(o,3,0.2f)); break;
        case TSerial6: {
            auto m = std::make_unique<Serial6DOF>();
            if (o.preset == PreKUKA) m->setKukaKR6Parameters();
            else m->setUR5Parameters();
            if (o.cfg.size() >= 24) {
                DHParameters dh[6];
                for (int i = 0; i < 6; ++i)
                    dh[i] = {float(o.cfg[i*4]),   float(o.cfg[i*4+1]),
                             float(o.cfg[i*4+2]), float(o.cfg[i*4+3])};
                m->setDHParameters(dh);
            }
            o.model = std::move(m); break;
        }
        case TSerial7: {
            auto m = std::make_unique<Serial7DOF>(); /* Panda default */
            if (o.cfg.size() >= 28) {
                DHParameters dh[7];
                for (int i = 0; i < 7; ++i)
                    dh[i] = {float(o.cfg[i*4]),   float(o.cfg[i*4+1]),
                             float(o.cfg[i*4+2]), float(o.cfg[i*4+3])};
                m->setDHParameters(dh);
            }
            o.model = std::move(m); break;
        }
        case TGantry5:    o.model = std::make_unique<Gantry5DOF>(); break;
        case TDelta:      o.model = std::make_unique<DeltaRobot>(
                                cfgAt(o,0,0.15f), cfgAt(o,1,0.05f),
                                cfgAt(o,2,0.2f),  cfgAt(o,3,0.4f)); break;
        case TStewart:    o.model = std::make_unique<StewartPlatform>(
                                cfgAt(o,0,0.3f), cfgAt(o,1,0.15f),
                                cfgAt(o,2,10.0*DEG_TO_RAD),
                                cfgAt(o,3,10.0*DEG_TO_RAD)); break;
        default: return false;
    }
    o.state = StReady; o.err = 0;
    return true;
}

void command(uint8_t cmd) {
    Obj& o = curObj();
    std::lock_guard lk(g_objMtx);
    switch (cmd) {
        case CmdFwd: {
            if (!o.model) { o.state = StError; o.err = 1; return; }
            const size_t n = o.model->getDOF();
            float q[8]{};
            for (size_t i = 0; i < n && i < kMaxDim; ++i)
                q[i] = float(o.in[i]);
            const Pose6D p = o.model->forwardKinematics(q);
            const EulerAngles e = p.euler();
            o.out = {double(p.position.x), double(p.position.y),
                     double(p.position.z), double(e.roll),
                     double(e.pitch), double(e.yaw), 0, 0};
            for (size_t i = 0; i < kMaxDim; ++i)
                o.q[i] = i < n ? double(q[i]) : 0.0;
            o.state = StReady; o.err = 0;
            return;
        }
        case CmdInv: {
            if (!o.model) { o.state = StError; o.err = 1; return; }
            const size_t n = o.model->getDOF();
            if (n == 0 || n > kMaxDim) { o.state = StError; o.err = 2; return; }
            /* kin.in[0..2] = target position; kin.in[3..3+n) = seed
               joints (zero when absent). Position-only IK; orientation
               of the result is whatever the chain produces. */
            float q[8]{};
            for (size_t i = 0; i < n; ++i)
                q[i] = i + 3 < kMaxDim ? float(o.in[i + 3]) : 0.0f;
            const Position3D target(float(o.in[0]), float(o.in[1]),
                                    float(o.in[2]));
            const bool ok = numericPositionIK(*o.model, target, q);
            const Pose6D p = o.model->forwardKinematics(q);
            const EulerAngles e = p.euler();
            o.out = {double(p.position.x), double(p.position.y),
                     double(p.position.z), double(e.roll),
                     double(e.pitch), double(e.yaw),
                     ok ? 1.0 : 0.0, 0};
            for (size_t i = 0; i < kMaxDim; ++i)
                o.q[i] = i < n ? double(q[i]) : 0.0;
            o.state = ok ? StReady : StError;
            o.err = ok ? 0 : 3;
            return;
        }
        case CmdReset:
            o.model.reset(); o.type = -1; o.state = StIdle; o.err = 0;
            o.nacs = 0; o.nmcs = 0; o.in = {}; o.out = {}; o.q = {};
            o.rot = {}; o.rotsaved = {};
            return;
        case CmdRotSave:   o.rotsaved = o.rot; return;
        case CmdRotRestore: o.rot = o.rotsaved; return;
        default: o.state = StError; o.err = 4; return;
    }
}

void regKinParam(Registry& reg, const MotionSchemas& d, uint64_t& nextId,
                 const char* name, const char* desc, ValueType vt,
                 const SchemaRef& schema, uint32_t slot,
                 ReadFn rd, WriteFn wr = nullptr) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "kin";
    p.valueType = vt;
    p.schema = schema;
    p.schemaSlot = slot;
    p.readFn = std::move(rd);
    if (wr) p.writeFn = std::move(wr);
    reg.addParam(p);
}

/* u32-count-prefixed f64/u32 list blob helpers. */
bool readF64List(const void* src, size_t len, double* dst, uint32_t cap,
                 uint32_t& n) {
    if (len < 4) return false;
    uint32_t cnt; std::memcpy(&cnt, src, 4);
    if (cnt > cap || len < 4 + size_t(cnt) * 8) return false;
    const uint8_t* p = static_cast<const uint8_t*>(src) + 4;
    for (uint32_t i = 0; i < cnt; ++i) {
        std::memcpy(&dst[i], p, 8); p += 8;
    }
    n = cnt;
    return true;
}
bool readU32List(const void* src, size_t len, uint32_t* dst, uint32_t cap,
                 uint32_t& n) {
    if (len < 4) return false;
    uint32_t cnt; std::memcpy(&cnt, src, 4);
    if (cnt > cap || len < 4 + size_t(cnt) * 4) return false;
    const uint8_t* p = static_cast<const uint8_t*>(src) + 4;
    for (uint32_t i = 0; i < cnt; ++i) {
        std::memcpy(&dst[i], p, 4); p += 4;
    }
    n = cnt;
    return true;
}

bool registerKinParams(Registry& reg, const MotionSchemas& d,
                       uint64_t& nextId) {
    auto u8rd = [](auto fn) {
        return [fn](void* dst) {
            const uint8_t v = static_cast<uint8_t>(fn());
            std::memcpy(dst, &v, 1);
        };
    };
    auto u16rd = [](auto fn) {
        return [fn](void* dst) {
            const uint16_t v = fn();
            std::memcpy(dst, &v, 2);
        };
    };

    regKinParam(reg, d, nextId, "kin.obj",
                "kinematic object selector 0-7", ValueType::U8, d.u8,
                d.slotU8,
                u8rd([] { return g_objSel.load(); }),
                [](const void* src) {
                    uint8_t v; std::memcpy(&v, src, 1);
                    g_objSel.store(v % kMaxObj);
                });
    regKinParam(reg, d, nextId, "kin.type",
                "chain type: 0 cartesian 1 planar2d 2 articulated3 "
                "3 scara 4 serial6 5 serial7 6 gantry5 7 delta "
                "8 stewart (-1 unconfigured)", ValueType::I32, d.i32,
                d.slotI32,
                [/* read */](void* dst) {
                    int32_t v;
                    { std::lock_guard lk(g_objMtx); v = curObj().type; }
                    std::memcpy(dst, &v, 4);
                },
                [](const void* src) {
                    int32_t v; std::memcpy(&v, src, 4);
                    Obj& o = curObj();
                    std::lock_guard lk(g_objMtx);
                    if (o.locked || v < -1 || v > TStewart) return;
                    o.type = v;
                    rebuild(o);
                });
    regKinParam(reg, d, nextId, "kin.preset",
                "chain preset: serial6 0=UR5 1=KUKA; serial7 Panda",
                ValueType::U8, d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx); return curObj().preset;
                }),
                [](const void* src) {
                    uint8_t v; std::memcpy(&v, src, 1);
                    Obj& o = curObj();
                    std::lock_guard lk(g_objMtx);
                    if (o.locked) return;
                    o.preset = v;
                    if (o.model) rebuild(o);
                });
    /* kin.cfg: binary u32 n + n f64 (type-specific geometry). */
    {
        ParamEntry p;
        p.id = nextId++;
        p.name = "kin.cfg";
        p.description = "chain config: u32 n + n f64 geometry values";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = 4 + kMaxCfg * 8;
        p.varReadFn = [](void* dst, size_t) -> size_t {
            std::lock_guard lk(g_objMtx);
            const Obj& o = curObj();
            uint8_t* p = static_cast<uint8_t*>(dst);
            const uint32_t n = uint32_t(o.cfg.size());
            std::memcpy(p, &n, 4); p += 4;
            for (double v : o.cfg) { std::memcpy(p, &v, 8); p += 8; }
            return 4 + n * 8;
        };
        p.varWriteFn = [](const void* src, size_t len) {
            Obj& o = curObj();
            std::lock_guard lk(g_objMtx);
            if (o.locked) return;
            double tmp[kMaxCfg]; uint32_t n = 0;
            if (!readF64List(src, len, tmp, kMaxCfg, n)) return;
            o.cfg.assign(tmp, tmp + n);
            if (o.model) rebuild(o);
        };
        reg.addParam(p);
    }
    regKinParam(reg, d, nextId, "kin.dof",
                "configured chain DOF (0 when unconfigured)",
                ValueType::U8, d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx);
                    const Obj& o = curObj();
                    return o.model ? uint8_t(o.model->getDOF()) : 0;
                }));
    regKinParam(reg, d, nextId, "kin.locked",
                "1 = type/preset/cfg writes rejected (param lock)",
                ValueType::U8, d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx); return curObj().locked;
                }),
                [](const void* src) {
                    uint8_t v; std::memcpy(&v, src, 1);
                    std::lock_guard lk(g_objMtx);
                    curObj().locked = v ? 1 : 0;
                });
    /* kin.in: binary u32 n + n f64. Forward: joint values. Backward:
       target x,y,z in [0..2], optional seed joints in [3..3+dof). */
    {
        ParamEntry p;
        p.id = nextId++;
        p.name = "kin.in";
        p.description = "transform input: u32 n + n f64 (joints or "
                        "target pose + seed)";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = 4 + kMaxDim * 8;
        p.varWriteFn = [](const void* src, size_t len) {
            Obj& o = curObj();
            double tmp[kMaxDim]; uint32_t n = 0;
            if (!readF64List(src, len, tmp, kMaxDim, n)) return;
            std::lock_guard lk(g_objMtx);
            for (uint32_t i = 0; i < kMaxDim; ++i)
                o.in[i] = i < n ? tmp[i] : 0.0;
        };
        reg.addParam(p);
    }
    /* kin.out / kin.q: fixed 8-f64 read blobs. */
    {
        ParamEntry p;
        p.id = nextId++;
        p.name = "kin.out";
        p.description = "last pose: x,y,z,roll,pitch,yaw,ikOk,0 (f64[8])";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = kMaxDim * 8;
        p.varReadFn = [](void* dst, size_t) -> size_t {
            std::lock_guard lk(g_objMtx);
            const auto& a = curObj().out;
            std::memcpy(dst, a.data(), a.size() * 8);
            return a.size() * 8;
        };
        reg.addParam(p);
    }
    {
        ParamEntry p;
        p.id = nextId++;
        p.name = "kin.q";
        p.description = "joint values: last input (fwd) or IK "
                        "solution (inv), f64[8]";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = kMaxDim * 8;
        p.varReadFn = [](void* dst, size_t) -> size_t {
            std::lock_guard lk(g_objMtx);
            const auto& a = curObj().q;
            std::memcpy(dst, a.data(), a.size() * 8);
            return a.size() * 8;
        };
        reg.addParam(p);
    }
    regKinParam(reg, d, nextId, "kin.cmd",
                "command write: 1 forward 2 backward/IK 3 reset "
                "4 save rotation 5 restore rotation",
                ValueType::U8, d.u8, d.slotU8,
                u8rd([] { return uint8_t{0}; }),
                [](const void* src) {
                    uint8_t v; std::memcpy(&v, src, 1);
                    command(v);
                });
    regKinParam(reg, d, nextId, "kin.state",
                "object state: 0 idle 1 ready 2 error", ValueType::U8,
                d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx);
                    return static_cast<uint8_t>(curObj().state);
                }));
    regKinParam(reg, d, nextId, "kin.err",
                "last error code (0 none, 1 no model, 2 bad DOF, "
                "3 IK no convergence, 4 bad cmd)", ValueType::U16,
                d.u16, d.slotU16,
                u16rd([] {
                    std::lock_guard lk(g_objMtx); return curObj().err;
                }));
    /* kin.acs / kin.mcs: binary u32 n + n u32 slave indices. */
    for (int which = 0; which < 2; ++which) {
        ParamEntry p;
        p.id = nextId++;
        p.name = which ? "kin.mcs" : "kin.acs";
        p.description = which
            ? "MCS (Cartesian) axis slave list: u32 n + n u32"
            : "ACS (joint) axis slave list: u32 n + n u32";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = 4 + kMaxDim * 4;
        p.varReadFn = [which](void* dst, size_t) -> size_t {
            std::lock_guard lk(g_objMtx);
            const Obj& o = curObj();
            const int32_t n = which ? o.nmcs : o.nacs;
            const uint32_t* a = which ? o.mcs : o.acs;
            uint8_t* p = static_cast<uint8_t*>(dst);
            const uint32_t nn = uint32_t(n);
            std::memcpy(p, &nn, 4); p += 4;
            for (int32_t i = 0; i < n; ++i) {
                std::memcpy(p, &a[i], 4); p += 4;
            }
            return 4 + nn * 4;
        };
        p.varWriteFn = [which](const void* src, size_t len) {
            Obj& o = curObj();
            uint32_t tmp[kMaxDim]; uint32_t n = 0;
            if (!readU32List(src, len, tmp, kMaxDim, n)) return;
            std::lock_guard lk(g_objMtx);
            if (which) {
                o.nmcs = int32_t(n);
                for (uint32_t i = 0; i < kMaxDim; ++i)
                    o.mcs[i] = i < n ? tmp[i] : 0;
            } else {
                o.nacs = int32_t(n);
                for (uint32_t i = 0; i < kMaxDim; ++i)
                    o.acs[i] = i < n ? tmp[i] : 0;
            }
        };
        reg.addParam(p);
    }
    regKinParam(reg, d, nextId, "kin.nacs",
                "configured ACS (joint) axis count", ValueType::U8,
                d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx);
                    return uint8_t(std::max(0, curObj().nacs));
                }));
    regKinParam(reg, d, nextId, "kin.nmcs",
                "configured MCS (Cartesian) axis count", ValueType::U8,
                d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx);
                    return uint8_t(std::max(0, curObj().nmcs));
                }));
    /* kin.rot: fixed 3-f64 rotation preset (rpy, rad). */
    {
        ParamEntry p;
        p.id = nextId++;
        p.name = "kin.rot";
        p.description = "rotation preset/state rpy [rad], f64[3]";
        p.group = "kin";
        p.valueType = ValueType::Binary;
        p.maxValueSize = 24;
        p.varReadFn = [](void* dst, size_t) -> size_t {
            std::lock_guard lk(g_objMtx);
            const auto& a = curObj().rot;
            std::memcpy(dst, a.data(), a.size() * 8);
            return a.size() * 8;
        };
        p.varWriteFn = [](const void* src, size_t len) {
            Obj& o = curObj();
            if (len < 24) return;
            std::lock_guard lk(g_objMtx);
            std::memcpy(o.rot.data(), src, 24);
        };
        reg.addParam(p);
    }
    regKinParam(reg, d, nextId, "kin.chn",
                "NC channel id of the kinematic group", ValueType::U8,
                d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx); return curObj().chn;
                }),
                [](const void* src) {
                    uint8_t v; std::memcpy(&v, src, 1);
                    std::lock_guard lk(g_objMtx); curObj().chn = v;
                });
    regKinParam(reg, d, nextId, "kin.chnstate",
                "kinematic channel operation state (0 idle 1 ready "
                "2 error)", ValueType::U8, d.u8, d.slotU8,
                u8rd([] {
                    std::lock_guard lk(g_objMtx);
                    const Obj& o = curObj();
                    return o.model ? uint8_t(o.state) : uint8_t(0);
                }));
    return true;
}

} // namespace kin


#if TETHER_HAVE_OPCUA
/* Minimal OPC UA endpoint via open62541: every Registry param becomes
   a ns=2;s=<name> Variable node with a DataSource read/write callback.
   Optional (--opcua <port>) and opt-in at build time
   (-DTETHER_ENABLE_OPCUA=ON with libopen62541 present). */
namespace opcua {

const UA_DataType* uaType(ValueType t) {
    switch (t) {
        case ValueType::Bool: return &UA_TYPES[UA_TYPES_BOOLEAN];
        case ValueType::I8:   return &UA_TYPES[UA_TYPES_SBYTE];
        case ValueType::U8:   return &UA_TYPES[UA_TYPES_BYTE];
        case ValueType::I16:  return &UA_TYPES[UA_TYPES_INT16];
        case ValueType::U16:  return &UA_TYPES[UA_TYPES_UINT16];
        case ValueType::I32:  return &UA_TYPES[UA_TYPES_INT32];
        case ValueType::U32:  return &UA_TYPES[UA_TYPES_UINT32];
        case ValueType::I64:  return &UA_TYPES[UA_TYPES_INT64];
        case ValueType::U64:  return &UA_TYPES[UA_TYPES_UINT64];
        case ValueType::F32:  return &UA_TYPES[UA_TYPES_FLOAT];
        case ValueType::F64:  return &UA_TYPES[UA_TYPES_DOUBLE];
        default:              return &UA_TYPES[UA_TYPES_DOUBLE];
    }
}
size_t scalarSize(ValueType t) {
    switch (t) {
        case ValueType::Bool: case ValueType::I8: case ValueType::U8: return 1;
        case ValueType::I16: case ValueType::U16: return 2;
        case ValueType::I32: case ValueType::U32: case ValueType::F32: return 4;
        default: return 8;
    }
}

UA_StatusCode readCb(UA_Server*, const UA_NodeId*, void*,
                     const UA_NodeId*, void* source,
                     UA_Boolean includeSourceTimestamp,
                     const UA_NumericRange*, UA_DataValue* v) {
    (void)includeSourceTimestamp;
    const ParamEntry* e = static_cast<const ParamEntry*>(source);
    uint8_t buf[8192];
    if (e->isVariableLength() && e->varReadFn) {
        const size_t n = e->varReadFn(buf, sizeof(buf));
        UA_String us; us.length = n; us.data = buf;
        UA_Variant_setScalarCopy(&v->value, &us,
                                 &UA_TYPES[UA_TYPES_STRING]);
    } else if (e->readFn) {
        e->readFn(buf);
        UA_Variant_setScalarCopy(&v->value, buf, uaType(e->valueType));
    } else {
        return UA_STATUSCODE_BADINTERNALERROR;
    }
    v->hasValue = true;
    v->hasSourceTimestamp = true;
    v->sourceTimestamp = UA_DateTime_now();
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode writeCb(UA_Server*, const UA_NodeId*, void*,
                      const UA_NodeId*, void* source,
                      const UA_NumericRange*, const UA_DataValue* v) {
    const ParamEntry* e = static_cast<const ParamEntry*>(source);
    if (!v->hasValue || !v->value.data) return UA_STATUSCODE_BADWRITENOTSUPPORTED;
    if (e->isVariableLength() && e->varWriteFn) {
        const UA_String* s = static_cast<const UA_String*>(v->value.data);
        e->varWriteFn(s->data, s->length);
        return UA_STATUSCODE_GOOD;
    }
    if (e->writeFn) { e->writeFn(v->value.data); return UA_STATUSCODE_GOOD; }
    return UA_STATUSCODE_BADWRITENOTSUPPORTED;
}

void serve(uint16_t port, Registry& reg) {
    UA_Server* srv = UA_Server_new();
    UA_ServerConfig_setMinimal(UA_Server_getConfig(srv), port, nullptr);
    const uint16_t ns =
        UA_Server_addNamespace(srv, "http://tether.io/params");
    motionLog("info", "opc ua: namespace index %u", ns);
    /* fold the Registry into UA Variables under ObjectsFolder */
    size_t added = 0, failed = 0;
    for (const auto& v : reg.paramPage(0, 65535)) {
        const ParamEntry* e = v.asParam();
        if (!e) continue;
        const std::string name(v.name());
        UA_VariableAttributes a = UA_VariableAttributes_default;
        a.displayName = UA_LOCALIZEDTEXT((char*)"en-US",
                                         const_cast<char*>(name.c_str()));
        a.description = a.displayName;
        a.accessLevel = static_cast<UA_Byte>(UA_ACCESSLEVELMASK_READ |
            (e->writeFn || e->varWriteFn ? UA_ACCESSLEVELMASK_WRITE : 0));
        UA_DataSource ds;
        ds.read = readCb;
        ds.write = (e->writeFn || e->varWriteFn) ? writeCb : nullptr;
        const UA_StatusCode st = UA_Server_addDataSourceVariableNode(
            srv, UA_NODEID_STRING_ALLOC(ns, name.c_str()),
            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(ns, const_cast<char*>(name.c_str())),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            a, ds, const_cast<ParamEntry*>(e), nullptr);
        if (st == UA_STATUSCODE_GOOD) ++added; else ++failed;
    }
    motionLog("info", "opc ua: %zu param nodes added (%zu failed, ns=%u)",
              added, failed, ns);
    volatile UA_Boolean running = true;
    std::thread watcher([&running] {
        while (!g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        running = false;
    });
    motionLog("info", "opc ua (open62541) on opc.tcp://127.0.0.1:%u", port);
    UA_Server_run(srv, &running);
    watcher.join();
    UA_Server_delete(srv);
}

} // namespace opcua
#endif /* TETHER_HAVE_OPCUA */

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
    prog.add_argument("--opcua").default_value(std::string(""))
        .help("if set to a port, also serve a minimal OPC UA binary "
              "endpoint exposing all params as ns=2;s=<name> nodes");
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

    // NCI channel: G-code -> MotionPlan -> CSP setpoints on slaves 0..2.
    nci::g_nci.drive = &drive;
    nci::g_nci.axes = axes;
    if (!nci::registerNciParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "nci param registration failed\n");
        return 1;
    }
    for (auto& c : fifo::g_fifo) {
        c.drive = &drive;
        c.axisCount = axes;
    }
    if (!fifo::registerFifoParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "fifo param registration failed\n");
        return 1;
    }
    if (!kin::registerKinParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "kin param registration failed\n");
        return 1;
    }
    if (!registerEcDcParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "ec.dc param registration failed\n");
        return 1;
    }
    if (!registerEcCoeDiagParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "ec.coediag param registration failed\n");
        return 1;
    }
    if (!registerEcSoeParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "ec.soe param registration failed\n");
        return 1;
    }

    // --- server --------------------------------------------------------------
    ServerConfig cfg;
    cfg.framing = Framing::Slip;
    cfg.timestampFn = steadyNowUs;
    cfg.logFn = motionLog;
    cfg.schemaCatalog = &catalog;
    cfg.maxClients = 16;

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
    // continuously between client polls. In realtime mode the NCI
    // channel also advances here; in sim mode it advances on mc.time
    // writes instead (dt is 0 between writes, so the call is a no-op).
    std::thread ticker([&] {
        while (!g_stop) {
            const uint64_t now = driveNowUs();
            drive.advanceTo(now);
            nci::nciTick(now);
            fifo::fifoTick(now);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // Optional OPC UA binary endpoint exposing the same Registry.
    std::thread uaThr;
    const std::string uaPort = prog.get<std::string>("--opcua");
    if (!uaPort.empty()) {
        const uint16_t p = static_cast<uint16_t>(std::stoul(uaPort));
#if TETHER_HAVE_OPCUA
        uaThr = std::thread([p, &registry] { opcua::serve(p, registry); });
#else
        (void)p;
        motionLog("warn", "--opcua given but built without "
                 "TETHER_ENABLE_OPCUA (libopen62541)");
#endif
    }

    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.stop();
    g_stop = true;
    if (uaThr.joinable()) uaThr.join();
    ticker.join();
    return 0;
}
