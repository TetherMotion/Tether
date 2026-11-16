// HmiWidgets.hpp — C++ port of TetherWebUI/src/hmi/widgets.ts.
//
// Each builder produces the identical HmiScene document the TS
// builders emit for the same inputs, so the native Vulkan backend
// renders views identical to the web SVG/WebGPU backend.
// Keep in lockstep with widgets.ts — parity is verified by
// comparing sceneToJson() output against the TS builders on shared
// fixtures.
#pragma once

#include "HmiScene.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

namespace tether { namespace hmi {

// palette — keep in sync with HMI_COLORS in widgets.ts
enum class Level { Ok, Warn, Crit };

struct Rgba { float r, g, b, a = 1.f; };

inline Rgba levelColor(Level l) {
    switch (l) {
    case Level::Crit: return {1.00f, 0.373f, 0.337f};   // #ff5f56
    case Level::Warn: return {1.00f, 0.702f, 0.278f};   // #ffb347
    default:          return {0.373f, 0.816f, 0.478f};  // #5fd07a
    }
}

constexpr Rgba kBg       {0.078f, 0.094f, 0.114f};   // #14181d
constexpr Rgba kPanel    {0.114f, 0.141f, 0.173f};   // #1d242c
constexpr Rgba kPanelEdge{0.200f, 0.251f, 0.302f};   // #33404d
constexpr Rgba kText     {0.843f, 0.871f, 0.910f};   // #d7dee8
constexpr Rgba kDim      {0.545f, 0.596f, 0.647f};   // #8b98a5
constexpr Rgba kTrack    {0.169f, 0.208f, 0.251f};   // #2b3540
constexpr Rgba kNeedle   {0.910f, 0.933f, 0.961f};   // #e8eef5
constexpr Rgba kWindowBg {0.043f, 0.055f, 0.070f};   // #0b0e12

/** value vs optional thresholds → level (>= comparisons, TS levelOf) */
inline Level levelOf(float value, std::optional<float> warn,
                     std::optional<float> crit) {
    if (crit && value >= *crit) return Level::Crit;
    if (warn && value >= *warn) return Level::Warn;
    return Level::Ok;
}

// ---- op builders (mirror the file-local helpers in widgets.ts) ------

inline SceneOp txt(float x, float y, const std::string& s,
                   Rgba c = kText, float size = 11.f, int anchor = 0) {
    SceneOp o; o.kind = SceneOp::Text;
    o.x = x; o.y = y; o.s = s; o.size = size; o.anchor = anchor;
    o.cr = c.r; o.cg = c.g; o.cb = c.b; o.ca = c.a;
    return o;
}

inline SceneOp rectOp(float x, float y, float w, float h,
                      std::optional<Rgba> fill = std::nullopt,
                      std::optional<Rgba> stroke = std::nullopt,
                      float sw = 1.f) {
    SceneOp o; o.kind = SceneOp::Rect;
    o.x = x; o.y = y; o.w = w; o.h = h; o.sw = sw;
    if (fill) { o.hasFill = true; o.fr = fill->r; o.fg = fill->g;
                o.fb = fill->b; o.fa = fill->a; }
    if (stroke) { o.hasStroke = true; o.cr = stroke->r; o.cg = stroke->g;
                  o.cb = stroke->b; o.ca = stroke->a; }
    return o;
}

inline SceneOp pathOp(std::vector<Pt> pts, Rgba c, float w = 1.f,
                      bool close = false, std::optional<Rgba> fill = std::nullopt) {
    SceneOp o; o.kind = SceneOp::Path;
    o.pts = std::move(pts); o.sw = w; o.close = close;
    o.cr = c.r; o.cg = c.g; o.cb = c.b; o.ca = c.a;
    if (fill) { o.hasFill = true; o.fr = fill->r; o.fg = fill->g;
                o.fb = fill->b; o.fa = fill->a; }
    return o;
}

inline SceneOp disc(float cx, float cy, float r, Rgba fill) {
    return pathOp(circlePoints(cx, cy, r, 48), Rgba{0,0,0,0}, 0.f, true, fill);
}

inline SceneOp ring(float cx, float cy, float r, Rgba c, float w) {
    return pathOp(circlePoints(cx, cy, r, 48), c, w, true);
}

/** JS Number.prototype.toExponential(0) format: "1e+4", "2e+4" —
 *  C printf gives "1e+04"; normalize the exponent to match. */
inline std::string jsExp0(float v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0e", double(v));
    std::string s = buf;
    auto e = s.find('e');
    if (e != std::string::npos) {
        std::string mant = s.substr(0, e);
        std::string ex = s.substr(e + 1);           // "+04" / "-04"
        char sign = ex.empty() ? '+' : ex[0];
        std::string digits = ex.substr(1);
        while (digits.size() > 1 && digits[0] == '0') digits.erase(0, 1);
        s = mant + "e" + sign + digits;
    }
    return s;
}

inline std::string fmtNum(float v, int dec) {
    if (!std::isfinite(v)) return "———";
    float a = std::fabs(v);
    if (a >= 10000.f) return jsExp0(v);
    char buf[32];
    if (a >= 100.f) std::snprintf(buf, sizeof buf, "%.0f", double(v));
    else std::snprintf(buf, sizeof buf, "%.*f", dec, double(v));
    return buf;
}

inline void sceneBg(HmiScene& s) {
    s.hasBg = true; s.br = kBg.r; s.bg = kBg.g; s.bb = kBg.b; s.ba = kBg.a;
    s.ops.push_back(rectOp(0, 0, s.w, s.h, kBg));
}

// ======================================================================
// gauge — circular process gauge (mirror gaugeScene in widgets.ts)
// ======================================================================

struct GaugeOpts {
    float w, h;
    std::string label, unit;
    float min = 0, max = 100;
    std::optional<float> value;
    std::optional<float> warn, crit;
    int decimals = 1;
};

inline HmiScene gaugeScene(const GaugeOpts& o) {
    HmiScene s; s.w = o.w; s.h = o.h; sceneBg(s);
    constexpr float kA0 = 135.f, kA1 = 405.f;
    constexpr int kTicks = 10;
    auto& ops = s.ops;

    float cx = o.w / 2.f, cy = o.h * 0.62f;
    float rOut = std::min(o.w * 0.44f, o.h * 0.52f);
    float rIn = rOut * 0.78f;
    float rNeedle = rIn * 0.92f;

    auto fracOf = [&](float v) {
        return o.max > o.min
            ? std::min(1.f, std::max(0.f, (v - o.min) / (o.max - o.min)))
            : 0.f;
    };

    auto band = [&](float f0, float f1, Rgba color, float width) {
        ops.push_back(pathOp(
            arcPoints(cx, cy, rOut - width / 2.f,
                      arcAngleAt(kA0, kA1, f0), arcAngleAt(kA0, kA1, f1), 2.f),
            color, width));
    };
    float warnF = o.warn ? fracOf(*o.warn) : 1.f;
    float critF = o.crit ? fracOf(*o.crit) : 1.f;
    band(0.f, std::min(warnF, critF), levelColor(Level::Ok), rOut - rIn);
    if (warnF < 1.f)
        band(std::min(warnF, critF), critF, levelColor(Level::Warn), rOut - rIn);
    if (critF < 1.f && critF > warnF)
        band(critF, 1.f, levelColor(Level::Crit), rOut - rIn);
    else if (critF <= warnF && critF < 1.f)
        band(critF, warnF, levelColor(Level::Crit), rOut - rIn);

    for (int i = 0; i <= kTicks; ++i) {
        float f = float(i) / kTicks;
        float a = arcAngleAt(kA0, kA1, f) * 3.14159265358979f / 180.f;
        bool major = i % 5 == 0;
        float r1 = rOut + (major ? 4.f : 2.f);
        float r2 = rOut + (major ? 9.f : 6.f);
        SceneOp l; l.kind = SceneOp::Line; l.sw = major ? 1.4f : 1.f;
        l.x = cx + std::cos(a) * r1; l.y = cy + std::sin(a) * r1;
        l.w = cx + std::cos(a) * r2; l.h = cy + std::sin(a) * r2;
        l.cr = kDim.r; l.cg = kDim.g; l.cb = kDim.b; l.ca = kDim.a;
        ops.push_back(l);
    }

    if (o.value) {
        float f = fracOf(*o.value);
        if (f > 0.001f) {
            Level lvl = levelOf(*o.value, o.warn, o.crit);
            ops.push_back(pathOp(
                arcPoints(cx, cy, (rOut + rIn) / 2.f,
                          arcAngleAt(kA0, kA1, 0.f),
                          arcAngleAt(kA0, kA1, f), 2.f),
                levelColor(lvl), rOut - rIn - 2.f));
        }
        float a = arcAngleAt(kA0, kA1, fracOf(*o.value))
                  * 3.14159265358979f / 180.f;
        Level lvl = levelOf(*o.value, o.warn, o.crit);
        Rgba nc = lvl == Level::Ok ? kNeedle : levelColor(lvl);
        SceneOp l; l.kind = SceneOp::Line;
        l.sw = std::max(1.6f, rOut * 0.035f);
        l.x = cx - std::cos(a) * rNeedle * 0.18f;
        l.y = cy - std::sin(a) * rNeedle * 0.18f;
        l.w = cx + std::cos(a) * rNeedle;
        l.h = cy + std::sin(a) * rNeedle;
        l.cr = nc.r; l.cg = nc.g; l.cb = nc.b; l.ca = nc.a;
        ops.push_back(l);
    }
    ops.push_back(disc(cx, cy, rOut * 0.09f, kPanelEdge));
    ops.push_back(disc(cx, cy, rOut * 0.05f, kText));

    int dec = o.decimals;
    std::string valStr = o.value ? fmtNum(*o.value, dec) : "———";
    Level lvl = o.value ? levelOf(*o.value, o.warn, o.crit) : Level::Ok;
    float vsize = std::max(12.f, rOut * 0.3f);
    ops.push_back(txt(cx, cy - rOut * 0.34f, valStr, levelColor(lvl),
                      vsize, 1));
    if (!o.unit.empty())
        ops.push_back(txt(cx, cy - rOut * 0.34f + vsize * 0.9f, o.unit,
                          kDim, std::max(8.f, rOut * 0.13f), 1));
    if (!o.label.empty())
        ops.push_back(txt(cx, o.h - 6.f, o.label, kDim,
                          std::max(8.f, rOut * 0.11f), 1));
    ops.push_back(txt(6.f, o.h - 6.f, fmtNum(o.min, dec), kDim, 8.f));
    ops.push_back(txt(o.w - 6.f, o.h - 6.f, fmtNum(o.max, dec), kDim, 8.f, 2));
    return s;
}

// ======================================================================
// bargraph — horizontal/vertical process bar (mirror bargraphScene)
// ======================================================================

struct BarOpts {
    float w, h;
    std::string label, unit;
    float min = 0, max = 100;
    std::optional<float> value;
    std::optional<float> warn, crit;
    bool vertical = false;
    int decimals = 1;
};

inline HmiScene bargraphScene(const BarOpts& o) {
    HmiScene s; s.w = o.w; s.h = o.h; sceneBg(s);
    auto& ops = s.ops;
    int dec = o.decimals;
    auto fracOf = [&](float v) {
        return o.max > o.min
            ? std::min(1.f, std::max(0.f, (v - o.min) / (o.max - o.min)))
            : 0.f;
    };

    float labelW = 34.f;
    float labH = o.label.empty() ? 0.f : 14.f;
    float bx = o.vertical ? 8.f : 4.f;
    float by = labH + (o.vertical ? 4.f : 8.f);
    float bw = o.vertical ? o.w - 16.f : o.w - labelW - 10.f;
    float bh = o.vertical ? o.h - labH - 22.f : o.h - labH - 16.f;

    ops.push_back(rectOp(bx, by, bw, bh, kTrack, kPanelEdge, 1.f));

    float warnF = o.warn ? fracOf(*o.warn) : 1.f;
    float critF = o.crit ? fracOf(*o.crit) : 1.f;
    auto marker = [&](float f, Rgba color) {
        if (f <= 0.f || f >= 1.f) return;
        SceneOp l; l.kind = SceneOp::Line; l.sw = 1.f;
        if (o.vertical) {
            l.x = bx + bw * f; l.y = by;
            l.w = bx + bw * f; l.h = by + bh;
        } else {
            l.x = bx; l.y = by + bh * (1.f - f);
            l.w = bx + bw; l.h = by + bh * (1.f - f);
        }
        l.cr = color.r; l.cg = color.g; l.cb = color.b; l.ca = color.a;
        ops.push_back(l);
    };
    marker(warnF, levelColor(Level::Warn));
    marker(critF, levelColor(Level::Crit));

    if (o.value) {
        float f = fracOf(*o.value);
        Rgba col = levelColor(levelOf(*o.value, o.warn, o.crit));
        if (o.vertical)
            ops.push_back(rectOp(bx, by + bh * (1.f - f), bw, bh * f, col));
        else
            ops.push_back(rectOp(bx, by, bw * f, bh, col));
    }

    int ticks = 5;
    for (int i = 0; i <= ticks; ++i) {
        float f = float(i) / ticks;
        if (o.vertical) {
            float y = by + bh * (1.f - f);
            SceneOp l; l.kind = SceneOp::Line; l.sw = 1.f;
            l.x = bx - 3.f; l.y = y; l.w = bx; l.h = y;
            l.cr = kDim.r; l.cg = kDim.g; l.cb = kDim.b; l.ca = kDim.a;
            ops.push_back(l);
            ops.push_back(txt(bx - 5.f, y + 3.f,
                              fmtNum(o.min + (o.max - o.min) * f, 0),
                              kDim, 7.f, 2));
        } else {
            float x = bx + bw * f;
            SceneOp l; l.kind = SceneOp::Line; l.sw = 1.f;
            l.x = x; l.y = by + bh; l.w = x; l.h = by + bh + 3.f;
            l.cr = kDim.r; l.cg = kDim.g; l.cb = kDim.b; l.ca = kDim.a;
            ops.push_back(l);
            ops.push_back(txt(x, by + bh + 10.f,
                              fmtNum(o.min + (o.max - o.min) * f, 0),
                              kDim, 7.f, 1));
        }
    }

    std::string valStr = o.value ? fmtNum(*o.value, dec) : "———";
    Rgba col = levelColor(o.value ? levelOf(*o.value, o.warn, o.crit)
                                   : Level::Ok);
    std::string unitStr = o.unit.empty() ? "" : " " + o.unit;
    if (o.vertical)
        ops.push_back(txt(o.w / 2.f, o.h - 4.f, valStr + unitStr, col, 13.f, 1));
    else
        ops.push_back(txt(bx + bw + 4.f, by + bh / 2.f + 4.f,
                          valStr + unitStr, col, 12.f));
    if (!o.label.empty())
        ops.push_back(txt(4.f, 11.f, o.label, kDim, 9.f));
    return s;
}

// ======================================================================
// digital — large numeric readout (mirror digitalScene)
// ======================================================================

struct DigitalOpts {
    float w, h;
    std::string label, unit;
    std::optional<float> min, max;
    std::optional<float> value;
    std::optional<float> warn, crit;
    int decimals = 1;
    bool window = false;
};

inline HmiScene digitalScene(const DigitalOpts& o) {
    HmiScene s; s.w = o.w; s.h = o.h; sceneBg(s);
    auto& ops = s.ops;
    int dec = o.decimals;
    std::string valStr = o.value ? fmtNum(*o.value, dec) : "———";
    Level lvl = o.value ? levelOf(*o.value, o.warn, o.crit) : Level::Ok;
    Rgba col = levelColor(lvl);

    if (o.window)
        ops.push_back(rectOp(2.f, 2.f, o.w - 4.f, o.h - 4.f,
                             kWindowBg, kPanelEdge, 1.f));

    int chars = int(valStr.size()) + (o.unit.empty() ? 0 : int(o.unit.size()) + 1);
    float availW = o.w - 12.f;
    float size = std::max(10.f, std::min(o.h * 0.52f,
        availW / std::max(1.f, chars * 0.62f)));

    float valY = o.h * 0.58f;
    ops.push_back(txt(o.w / 2.f, valY, valStr, col, size, 1));
    if (!o.unit.empty()) {
        float unitSize = std::max(8.f, size * 0.42f);
        float valW = float(valStr.size()) * size * 0.602031f;
        ops.push_back(txt(o.w / 2.f + valW / 2.f + 3.f, valY - size * 0.28f,
                          o.unit, kDim, unitSize));
    }
    if (!o.label.empty())
        ops.push_back(txt(o.w / 2.f, o.h - 5.f, o.label, kDim, 9.f, 1));

    if (lvl != Level::Ok)
        ops.push_back(rectOp(1.f, 1.f, o.w - 2.f, o.h - 2.f,
                             std::nullopt, col, 2.f));
    return s;
}

// ======================================================================
// lamp stack — RUN/ALARM/FAULT (mirror lampStackScene)
// ======================================================================

struct LampOpts {
    float w, h;
    /** -1 off, 0 off, 1 on, 2 blink */
    int run = 0, alarm = 0, fault = 0;
    std::string runLabel = "RUN", alarmLabel = "ALM", faultLabel = "FLT";
    float phase = 0.f;   // 0..1 blink phase
};

inline HmiScene lampStackScene(const LampOpts& o) {
    HmiScene s; s.w = o.w; s.h = o.h; sceneBg(s);
    auto& ops = s.ops;
    auto lit = [&](int st) {
        if (st == 1) return true;
        if (st == 2) return o.phase < 0.5f;
        return false;
    };
    struct L { int state; Rgba color; std::string label; };
    const L lamps[3] = {
        {o.run,   levelColor(Level::Ok),   o.runLabel},
        {o.alarm, levelColor(Level::Warn), o.alarmLabel},
        {o.fault, levelColor(Level::Crit), o.faultLabel},
    };
    float r = std::min(o.w * 0.16f, o.h * 0.14f);
    float gap = o.h * 0.2f;
    float cx = o.w / 2.f;
    float firstY = o.h * 0.5f - gap;
    for (int i = 0; i < 3; ++i) {
        const L& l = lamps[i];
        float cy = firstY + i * gap;
        bool on = lit(l.state);
        ops.push_back(disc(cx, cy, r * 1.25f, kPanel));
        ops.push_back(ring(cx, cy, r * 1.25f, kPanelEdge, 1.f));
        Rgba lens = on ? l.color : Rgba{0.137f, 0.165f, 0.200f}; // #232a33
        ops.push_back(disc(cx, cy, r, lens));
        if (on) {
            Rgba hi = l.color.r == 1.f && l.color.g < 0.5f ? Rgba{1.f,0.690f,0.675f}
                     : l.color.g > 0.7f && l.color.r == 1.f ? Rgba{1.f,0.851f,0.627f}
                     : Rgba{0.722f,0.941f,0.784f};
            ops.push_back(disc(cx - r * 0.25f, cy - r * 0.25f, r * 0.45f, hi));
        }
        ops.push_back(txt(cx + r * 1.5f, cy + 3.f, l.label,
                          on ? l.color : kDim, 9.f));
    }
    return s;
}

// ======================================================================
// alarm board — active alarm summary (mirror alarmBoardScene)
// ======================================================================

struct AlarmRow {
    std::string text;
    Level level = Level::Ok;
    bool acked = false;
};

struct AlarmBoardOpts {
    float w, h;
    std::string title = "ALARMS";
    std::vector<AlarmRow> rows;
    int maxRows = 8;
};

inline HmiScene alarmBoardScene(const AlarmBoardOpts& o) {
    HmiScene s; s.w = o.w; s.h = o.h; sceneBg(s);
    auto& ops = s.ops;
    float headerH = 16.f;
    float rowH = std::max(12.f,
        (o.h - headerH - 4.f) / std::max(1, o.maxRows));
    int maxRows = int(std::min<size_t>(o.rows.size(),
        std::floor((o.h - headerH - 4.f) / rowH)));

    ops.push_back(rectOp(0, 0, o.w, headerH, kPanel, kPanelEdge, 1.f));
    int nActive = 0;
    for (const auto& r : o.rows) if (!r.acked) ++nActive;
    ops.push_back(txt(4.f, 11.f,
        (o.title.empty() ? "ALARMS" : o.title) +
            " — " + std::to_string(nActive) + " active",
        nActive > 0 ? levelColor(Level::Warn) : kDim, 10.f));

    Rgba rowBg[2] = {Rgba{0.078f,0.106f,0.137f},   // #141b23
                     Rgba{0.094f,0.125f,0.157f}};  // #182028
    for (int i = 0; i < maxRows; ++i) {
        const AlarmRow& r = o.rows[size_t(i)];
        float y = headerH + i * rowH;
        ops.push_back(rectOp(0, y, o.w, rowH - 1.f, rowBg[i % 2]));
        ops.push_back(rectOp(2.f, y + 2.f, 6.f, rowH - 5.f,
                             levelColor(r.level)));
        std::string label = r.acked ? "✓ " + r.text : r.text;
        ops.push_back(txt(12.f, y + rowH - 4.f, label,
                          r.acked ? kDim : kText, 9.f));
    }
    if (o.rows.empty())
        ops.push_back(txt(o.w / 2.f, o.h / 2.f, "no active alarms",
                          levelColor(Level::Ok), 11.f, 1));
    return s;
}

}} // namespace tether::hmi
