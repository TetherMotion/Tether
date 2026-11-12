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

#include <argparse/argparse.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

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
        /* The NCI channel runs on this clock: each client time write
           advances the plan deterministically. */
        nci::nciTick(t);
    };
    return reg.addParam(p);
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

    /* Compile program text into planning segments. READY on success. */
    bool compile(const std::string& text) {
        auto r = GCode::PlanningSegmentBuilder::fromText(text);
        std::lock_guard lk(m);
        if (!r.error.ok() && !r.error.isEnd()) {
            errcode = static_cast<uint32_t>(r.error.code);
            segs.clear(); blocks.clear(); plan.reset();
            planT = duration = 0.0; curLine = 0;
            state = Error;
            return false;
        }
        segs = std::move(r.segments);
        blocks = std::move(r.blocks);
        errcode = ErrNone; plan.reset();
        planT = duration = 0.0; curLine = 0;
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
        curSeg = static_cast<int>(st.segmentIndex);
        if (st.segmentIndex < planSegToSeg.size()) {
            const int si = planSegToSeg[st.segmentIndex];
            const int32_t bi = segs[si].blockIndex;
            if (bi >= 0 && bi < static_cast<int32_t>(blocks.size()))
                curLine = static_cast<uint32_t>(blocks[bi].lineNumber);
        }
    }

    void command(uint8_t cmd) {
        std::lock_guard lk(m);
        lastCmd = cmd;
        switch (cmd) {
            case CmdRun:
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
                state = Running;
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
                stepTarget = curSeg + 1;
                state = Running;
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
            state = Finished;
            return;
        }
        emitTargets();
        if (stepTarget >= 0 && curSeg >= stepTarget) {
            stepTarget = -1;
            state = Paused;
        }
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
            std::function<void(void*)> rd) {
    ParamEntry p;
    p.id = nextId++;
    p.name = name;
    p.description = desc;
    p.group = "nc";
    p.valueType = ValueType::U32;
    p.schema = d.u32;
    p.schemaSlot = d.slotU32;
    p.readFn = std::move(rd);
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
               g_nci.compile(std::string(static_cast<const char*>(src), len));
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
               g_nci.compile(ss.str());
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
    return true;
}

} // namespace nci

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

    // NCI channel: G-code -> MotionPlan -> CSP setpoints on slaves 0..2.
    nci::g_nci.drive = &drive;
    nci::g_nci.axes = axes;
    if (!nci::registerNciParams(registry, schemas, nextId)) {
        std::fprintf(stderr, "nci param registration failed\n");
        return 1;
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
    // continuously between client polls. In realtime mode the NCI
    // channel also advances here; in sim mode it advances on mc.time
    // writes instead (dt is 0 between writes, so the call is a no-op).
    std::thread ticker([&] {
        while (!g_stop) {
            const uint64_t now = driveNowUs();
            drive.advanceTo(now);
            nci::nciTick(now);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.stop();
    g_stop = true;
    ticker.join();
    return 0;
}
