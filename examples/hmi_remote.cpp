// hmi_remote.cpp — remote native HMI client over Tether IO / TCP.
//
// Connects to a Tether IO server (e.g. tplc-io serving the plc.*
// catalog published by tether-plc), derives an HMI screen —
// automatically from catalog naming conventions (--auto, the same
// rules as TetherWebUI/src/hmi/autoscreen.ts) or from an authored
// screen JSON (--screen) — builds the shared HmiScene documents with
// the C++ widget builders (examples/common/HmiWidgets.hpp, mirror of
// src/hmi/widgets.ts) and rasterizes them with the shared Vulkan
// backend (examples/common/HmiVulkan.cpp).
//
// Web and native views are identical by construction: same scene IR,
// same builders, same glyb glyph engine.
//
// Usage:
//   hmi_remote --host 127.0.0.1 --port 43770 [options]
//     --auto              generate screen from plc.* catalog (default)
//     --screen FILE       load authored screen JSON instead
//     --frames N          render N frames (default 1; 0 = until Ctrl-C)
//     --period MS         poll period between frames (default 500)
//     --scale N           output scale (default 2)
//     --font PATH         font file (default DejaVu Sans Mono search)
//     --out PREFIX        output prefix → PREFIX_0000.ppm …
//     --list              print discovered catalog + screen, no render
//
// SPDX-License-Identifier: Apache-2.0 OR MPL-2.0

#include "common/HmiScene.hpp"
#include "common/HmiWidgets.hpp"
#include "common/HmiAuto.hpp"
#include "common/HmiVulkan.hpp"

#include <tether/io/TetherIOClient.hpp>
#include <tether/io/TcpTransport.hpp>
#include <tether/io/SlipTransport.hpp>
#include <tether/io/SchemaValue.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace tether::hmi;
using tether::io::TetherIOClient;
using tether::io::ClientCatalogEntry;
using tether::io::ClientError;
using tether::io::ValueType;

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop = true; }

// ==========================================================================
// catalog → values
// ==========================================================================

struct IoValue {
    const ClientCatalogEntry* entry = nullptr;
    bool isParam = false;
};

static uint64_t rdLE(const uint8_t* p, int n) {
    uint64_t v = 0;
    for (int i = n - 1; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}
static int64_t rdLEs(const uint8_t* p, int n) {
    uint64_t u = rdLE(p, n);
    int bits = n * 8;
    if (bits < 64 && (u >> (bits - 1)) & 1)
        return int64_t(u | (~0ull << bits));
    return int64_t(u);
}
static double flatToDouble(ValueType t, const uint8_t* p, size_t n) {
    switch (t) {
    case ValueType::F64: { double d; std::memcpy(&d, p, 8); return d; }
    case ValueType::F32: { float f; std::memcpy(&f, p, 4); return f; }
    case ValueType::Bool: return n && p[0] ? 1.0 : 0.0;
    case ValueType::U8:  return n ? p[0] : 0;
    case ValueType::U16: return n >= 2 ? rdLE(p, 2) : 0;
    case ValueType::U32: return n >= 4 ? rdLE(p, 4) : 0;
    case ValueType::U64: return n >= 8 ? rdLE(p, 8) : 0;
    case ValueType::I8:  return n ? int64_t(int8_t(p[0])) : 0;
    case ValueType::I16: return n >= 2 ? rdLEs(p, 2) : 0;
    case ValueType::I32: return n >= 4 ? rdLEs(p, 4) : 0;
    case ValueType::I64: return n >= 8 ? rdLEs(p, 8) : 0;
    default: return 0;
    }
}

/** Extract a double from a schema-decoded value (scalar leaves only). */
static std::optional<double> schemaToDouble(
    const tether::io::SchemaValue& v) {
    if (const auto* s = std::get_if<tether::io::SchemaValue::Scalar>(
            &v.payload)) {
        ValueType t = v.node ? v.node->scalarType : ValueType::None;
        uint64_t bits = s->bits;
        switch (t) {
        case ValueType::F64: { double d; std::memcpy(&d, &bits, 8); return d; }
        case ValueType::F32: { uint32_t u = uint32_t(bits); float f;
                                std::memcpy(&f, &u, 4); return f; }
        case ValueType::Bool: return bits != 0 ? 1.0 : 0.0;
        case ValueType::I8: case ValueType::I16: case ValueType::I32:
        case ValueType::I64:
        case ValueType::U8: case ValueType::U16: case ValueType::U32:
        case ValueType::U64: case ValueType::Enum:
        case ValueType::UVarint: case ValueType::IVarint:
            return double(int64_t(bits));
        default: return std::nullopt;
        }
    }
    return std::nullopt;
}

struct IoCatalog {
    TetherIOClient* client;
    std::map<std::string, IoValue> byName;

    bool read(const std::string& name, double& out) {
        auto it = byName.find(name);
        if (it == byName.end()) return false;
        const IoValue& iv = it->second;
        auto res = iv.isParam
            ? client->getParam(iv.entry->id)
            : client->getSignal(iv.entry->id);
        if (!res) return false;
        const auto& bytes = *res;
#if TETHER_IO_SCHEMA_ENABLED
        if (iv.entry->schemaTyped() && !bytes.empty()) {
            auto v = client->decodeValue(iv.entry->schemaSlot,
                                         bytes.data(), bytes.size());
            if (v) {
                if (auto d = schemaToDouble(*v)) { out = *d; return true; }
            }
            return false;
        }
#endif
        if (iv.entry->flat() || iv.entry->typeTag ==
            tether::io::EntryTypeTag::Flat)
            out = flatToDouble(iv.entry->type, bytes.data(), bytes.size());
        else
            out = flatToDouble(ValueType::F64, bytes.data(), bytes.size());
        return true;
    }
};

/** Offset every coordinate of a scene op (translate into screen cell). */
static void translateOp(SceneOp& op, float dx, float dy) {
    op.x += dx; op.y += dy;
    op.w += dx; op.h += dy;   // line: x2,y2 live in w/h
    for (auto& p : op.pts) { p.x += dx; p.y += dy; }
}

// ==========================================================================
// widget spec → scene (uses live values)
// ==========================================================================

static std::string optOf(const WidgetSpec& w, const char* key,
                         const char* dflt = "") {
    for (const auto& [k, v] : w.options)
        if (k == key) return v;
    return dflt;
}
static float optNum(const WidgetSpec& w, const char* key, float dflt) {
    std::string s = optOf(w, key);
    return s.empty() ? dflt : float(std::atof(s.c_str()));
}
static int optInt(const WidgetSpec& w, const char* key, int dflt) {
    std::string s = optOf(w, key);
    return s.empty() ? dflt : std::atoi(s.c_str());
}

static bool hasOpt(const WidgetSpec& w, const char* key) {
    for (const auto& [k, v] : w.options)
        if (k == key && !v.empty()) return true;
    return false;
}
static std::optional<float> optF(const WidgetSpec& w, const char* key) {
    return hasOpt(w, key)
        ? std::optional<float>(optNum(w, key, 0.f)) : std::nullopt;
}

static double sig(IoCatalog& cat, const std::string& s, bool& ok,
                  double dflt = 0.0) {
    double v;
    ok = cat.read(s, v);
    return ok ? v : dflt;
}

/** Build the widget's scene; ops are translated by (ox, oy). */
static bool buildWidgetScene(IoCatalog& cat, const WidgetSpec& w,
                             float ox, float oy, float phase,
                             HmiScene& out) {
    std::string type = w.type;
    if (type == "alarms") {
        // alarm journal is not part of the IO value catalog — render
        // the empty-state board so the cell still shows the frame.
        out = alarmBoardScene(AlarmBoardOpts{
            w.w, w.h, optOf(w, "title", "ALARMS"), {}, optInt(w, "maxRows", 8)});
    } else if (type == "lamp") {
        LampOpts o; o.w = w.w; o.h = w.h; o.phase = phase;
        o.runLabel = optOf(w, "runLabel", "RUN");
        o.alarmLabel = optOf(w, "alarmLabel", "ALM");
        o.faultLabel = optOf(w, "faultLabel", "FLT");
        auto lamp = [&](size_t i) -> int {
            if (i >= w.signals.size()) return 0;
            bool ok; double v = sig(cat, w.signals[i], ok);
            if (!ok) return 0;
            return v == 2.0 ? 2 : (v != 0.0 ? 1 : 0);
        };
        o.run = lamp(0); o.alarm = lamp(1); o.fault = lamp(2);
        out = lampStackScene(o);
    } else if (type == "bar") {
        BarOpts o; o.w = w.w; o.h = w.h;
        o.label = optOf(w, "label"); o.unit = optOf(w, "unit");
        o.min = optNum(w, "min", 0); o.max = optNum(w, "max", 100);
        o.warn = optF(w, "warn");
        o.crit = optF(w, "crit");
        o.vertical = optOf(w, "orientation") == "vertical";
        o.decimals = optInt(w, "decimals", 1);
        if (!w.signals.empty()) {
            bool ok; double v = sig(cat, w.signals[0], ok);
            if (ok) o.value = float(v);
        }
        out = bargraphScene(o);
    } else if (type == "gauge") {
        GaugeOpts o; o.w = w.w; o.h = w.h;
        o.label = optOf(w, "label"); o.unit = optOf(w, "unit");
        o.min = optNum(w, "min", 0); o.max = optNum(w, "max", 100);
        o.warn = optF(w, "warn");
        o.crit = optF(w, "crit");
        o.decimals = optInt(w, "decimals", 1);
        if (!w.signals.empty()) {
            bool ok; double v = sig(cat, w.signals[0], ok);
            if (ok) o.value = float(v);
        }
        out = gaugeScene(o);
    } else {
        // digital + profile members + fallback: one digital per signal
        // (profile widgets render their bound members as a cluster of
        // readouts — the dedicated profile panel is a web-tile feature;
        // scene parity for it lands with the profile scene builder).
        if (type == "profile") {
            // emit a row of digital readouts for each bound member
            HmiScene merged; merged.w = w.w; merged.h = w.h;
            sceneBg(merged);
            float cw = w.w / std::max<size_t>(1, w.signals.size());
            for (size_t i = 0; i < w.signals.size(); ++i) {
                bool ok; double v = sig(cat, w.signals[i], ok);
                DigitalOpts d; d.w = cw; d.h = w.h;
                d.label = w.signals[i].substr(
                    w.signals[i].find_last_of('.') == std::string::npos
                        ? 0 : w.signals[i].find_last_of('.') + 1);
                d.decimals = 1;
                if (ok) d.value = float(v);
                HmiScene one = digitalScene(d);
                for (size_t oi = 0; oi < one.ops.size(); ++oi) {
                    auto& op = one.ops[oi];
                    // strip the per-widget bg rect (index 0) — merged
                    // scene already has one
                    if (oi == 0) continue;
                    translateOp(op, cw * float(i), 0.f);
                    merged.ops.push_back(op);
                }
            }
            out = std::move(merged);
            return true;
        }
        DigitalOpts o; o.w = w.w; o.h = w.h;
        o.label = optOf(w, "label"); o.unit = optOf(w, "unit");
        o.min = optF(w, "min");
        o.max = optF(w, "max");
        o.warn = optF(w, "warn");
        o.crit = optF(w, "crit");
        o.decimals = optInt(w, "decimals", 1);
        o.window = optOf(w, "window") == "1" || optOf(w, "window") == "true";
        if (!w.signals.empty()) {
            bool ok; double v = sig(cat, w.signals[0], ok);
            if (ok) o.value = float(v);
        }
        out = digitalScene(o);
    }

    // translate all ops into screen position
    for (auto& op : out.ops) translateOp(op, ox, oy);
    return true;
}

// ==========================================================================
// main
// ==========================================================================

static void usage(const char* a0) {
    std::cerr <<
        "usage: " << a0 << " --host HOST --port PORT [options]\n"
        "  --auto | --screen FILE   screen source (default --auto)\n"
        "  --frames N               frames to render (0 = forever)\n"
        "  --period MS              poll period (default 500)\n"
        "  --scale N                pixel scale (default 2)\n"
        "  --font PATH              font file\n"
        "  --out PREFIX             output prefix (default hmi_remote)\n"
        "  --list                   list catalog + screen, no render\n";
}

int main(int argc, char** argv) {
    std::string host = "127.0.0.1", screenPath, fontPath, outPrefix = "hmi_remote";
    uint16_t port = 43770;
    int frames = 1, scale = 2, periodMs = 500;
    bool autoScreenMode = true, listOnly = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* k) -> const char* {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(2); }
            return argv[++i];
        };
        if (a == "--host") host = need("--host");
        else if (a == "--port") port = uint16_t(std::atoi(need("--port")));
        else if (a == "--screen") { screenPath = need("--screen"); autoScreenMode = false; }
        else if (a == "--auto") autoScreenMode = true;
        else if (a == "--frames") frames = std::atoi(need("--frames"));
        else if (a == "--period") periodMs = std::atoi(need("--period"));
        else if (a == "--scale") scale = std::atoi(need("--scale"));
        else if (a == "--font") fontPath = need("--font");
        else if (a == "--out") outPrefix = need("--out");
        else if (a == "--list") listOnly = true;
        else { usage(argv[0]); return 2; }
    }
    std::signal(SIGINT, onSig);
    std::signal(SIGTERM, onSig);

    auto tcp = tether::io::tcpConnect(host.c_str(), port, 5000);
    if (!tcp) {
        std::cerr << "connect " << host << ":" << port << " failed\n";
        return 1;
    }
    /* TCP is a byte stream — wrap in SLIP framing (server default) */
    TetherIOClient client(
        std::make_unique<tether::io::SlipFramedTransport>(std::move(tcp)));
    if (!client.isConnected()) {
        std::cerr << "io handshake failed\n";
        return 1;
    }
    std::cerr << "connected " << host << ":" << port
              << " schemaEpoch=" << client.schemaEpoch() << "\n";

    // ---- catalog -------------------------------------------------------
    std::vector<CatalogSignal> catalogSignals;
    IoCatalog cat{&client, {}};
    // entry storage must outlive cat.byName pointers — static vectors
    static thread_local std::vector<ClientCatalogEntry> params, signals;
    params.clear(); signals.clear();
    for (uint32_t off = 0;;) {
        auto r = client.listParams(off, 500);
        if (!r) { std::cerr << "listParams: " << r.error().message << "\n"; break; }
        if (r->empty()) break;
        for (auto& e : *r) params.push_back(e);
        off += uint32_t(r->size());
        if (r->size() < 500) break;
    }
    for (uint32_t off = 0;;) {
        auto r = client.listSignals(off, 500);
        if (!r) { std::cerr << "listSignals: " << r.error().message << "\n"; break; }
        if (r->empty()) break;
        for (auto& e : *r) signals.push_back(e);
        off += uint32_t(r->size());
        if (r->size() < 500) break;
    }
    /* re-point IoValue at the stable vectors */
    cat.byName.clear();
    for (const auto& e : params)
        if (!e.name.empty()) {
            catalogSignals.push_back({e.name, true, e.description});
            cat.byName[e.name] = IoValue{&e, true};
        }
    for (const auto& e : signals)
        if (!e.name.empty()) {
            catalogSignals.push_back({e.name, false, e.description});
            cat.byName[e.name] = IoValue{&e, false};
        }

    // ---- screen ---------------------------------------------------------
    HmiScreen screen;
    if (autoScreenMode) {
        screen = autoScreen(catalogSignals);
    } else {
        std::ifstream in(screenPath, std::ios::binary);
        if (!in) { std::cerr << "cannot open " << screenPath << "\n"; return 1; }
        std::stringstream ss; ss << in.rdbuf();
        screen = parseScreenJson(ss.str());
    }
    if (screen.widgets.empty()) {
        std::cerr << "empty screen — no plc.* catalog entries matched\n";
        return 1;
    }

    if (listOnly) {
        std::cout << screenToJson(screen) << "\n";
        return 0;
    }

    // ---- layout ---------------------------------------------------------
    ScreenLayout layout;
    layout.cols = 4;
    /* widen cells for profile widgets */
    float totalW = 0, totalH = 0;
    {
        float x, y;
        for (size_t i = 0; i < screen.widgets.size(); ++i) {
            const WidgetSpec& w = screen.widgets[i];
            float cw = w.type == "profile" ? 320.f : layout.cellW;
            float ch = w.type == "profile" ? 190.f : layout.cellH;
            int col = int(i % size_t(layout.cols));
            int row = int(i / size_t(layout.cols));
            (void)x; (void)y;
            totalW = std::max(totalW,
                layout.originX + (col + 1) * cw);
            totalH = std::max(totalH,
                layout.originY + (row + 1) * ch);
        }
    }

    HmiRenderer renderer;
    if (!renderer.init(fontPath)) {
        std::cerr << "renderer init failed\n";
        return 1;
    }

    int frame = 0;
    while (!g_stop && (frames == 0 || frame < frames)) {
        // build the full-screen scene
        HmiScene scene;
        scene.w = totalW + layout.originX;
        scene.h = totalH + layout.originY;
        scene.hasBg = true;
        scene.br = kBg.r; scene.bg = kBg.g; scene.bb = kBg.b; scene.ba = kBg.a;
        scene.ops.push_back(rectOp(0, 0, scene.w, scene.h, kBg));

        float phase = float(frame % 10) / 10.f;
        for (size_t i = 0; i < screen.widgets.size(); ++i) {
            const WidgetSpec& w = screen.widgets[i];
            float cw = w.type == "profile" ? 320.f : layout.cellW;
            float ch = w.type == "profile" ? 190.f : layout.cellH;
            int col = int(i % size_t(layout.cols));
            int row = int(i / size_t(layout.cols));
            float ox = layout.originX + col * cw;
            float oy = layout.originY + row * ch;
            /* wrap each widget in a subtle cell frame for readability */
            scene.ops.push_back(rectOp(ox - 2.f, oy - 2.f,
                                       cw + 4.f, ch + 4.f,
                                       std::nullopt, kPanelEdge, 1.f));
            HmiScene one;
            if (buildWidgetScene(cat, w, ox, oy, phase, one)) {
                for (auto& op : one.ops) scene.ops.push_back(std::move(op));
            }
        }

        char path[512];
        std::snprintf(path, sizeof path, "%s_%04d.ppm",
                      outPrefix.c_str(), frame);
        if (!renderer.renderPpm(scene, scale, path)) {
            std::cerr << "render failed at frame " << frame << "\n";
            return 1;
        }
        std::cerr << "frame " << frame << " → " << path
                  << " (" << scene.w << "x" << scene.h << ", "
                  << scene.ops.size() << " ops, "
                  << screen.widgets.size() << " widgets)\n";
        ++frame;
        if (frames != 0 && frame >= frames) break;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(periodMs));
    }
    return 0;
}
