// HmiScene.hpp — C++ model of the TetherWebUI HMI scene IR.
//
// Mirrors TetherWebUI/src/hmi/scene.ts exactly: a flat op list
// (rect / line / path / text) in a top-left-origin, y-down space.
// Both the web SVG backend and the native Vulkan backend consume the
// same document, so views are identical by construction.
//
// Text convention (shared with hmi_font_metrics.cpp → font-atlas.ts):
// op (x, y) is the baseline origin; `size` is em size in scene units;
// anchor start/middle/end shifts by measured advance.
//
// Color format: '#rgb', '#rrggbb' or '#rrggbbaa' (alpha optional).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace tether { namespace hmi {

// ==========================================================================
// Minimal JSON (parse + emit)
// ==========================================================================

struct Json {
    using Obj = std::map<std::string, Json>;
    using Arr = std::vector<Json>;
    std::variant<std::nullptr_t, bool, double, std::string, Arr, Obj> v;

    bool isObj() const { return std::holds_alternative<Obj>(v); }
    bool isArr() const { return std::holds_alternative<Arr>(v); }
    const Obj& obj() const { return std::get<Obj>(v); }
    const Arr& arr() const { return std::get<Arr>(v); }
    double num(double d = 0) const {
        if (const auto* p = std::get_if<double>(&v)) return *p;
        return d;
    }
    bool boolean() const {
        if (const auto* p = std::get_if<bool>(&v)) return *p;
        return false;
    }
    std::string str() const {
        if (const auto* p = std::get_if<std::string>(&v)) return *p;
        return {};
    }
    const Json* find(const char* k) const {
        if (!isObj()) return nullptr;
        auto it = obj().find(k);
        return it == obj().end() ? nullptr : &it->second;
    }
};

struct JsonParser {
    const char* p;
    explicit JsonParser(const std::string& s) : p(s.c_str()) {}
    void ws() { while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p; }
    bool eat(char c) { ws(); if (*p == c) { ++p; return true; } return false; }
    void expect(char c) { if (!eat(c)) throw std::runtime_error("json: expected char"); }
    Json parse() { ws(); return value(); }

    Json value() {
        ws();
        switch (*p) {
        case '{': return object();
        case '[': return array();
        case '"': return Json{str()};
        case 't': p += 4; return Json{true};
        case 'f': p += 5; return Json{false};
        case 'n': p += 4; return Json{nullptr};
        default: return Json{number()};
        }
    }
    Json object() {
        Json::Obj o;
        expect('{'); ws();
        if (eat('}')) return Json{o};
        while (true) {
            ws(); std::string k = str(); expect(':');
            o.emplace(std::move(k), value());
            if (eat('}')) break;
            expect(',');
        }
        return Json{o};
    }
    Json array() {
        Json::Arr a;
        expect('['); ws();
        if (eat(']')) return Json{a};
        while (true) {
            a.push_back(value());
            if (eat(']')) break;
            expect(',');
        }
        return Json{a};
    }
    std::string str() {
        expect('"');
        std::string out;
        while (*p && *p != '"') {
            if (*p == '\\') {
                ++p;
                switch (*p) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'u': {
                    unsigned cp = 0;
                    for (int i = 0; i < 4 && p[1]; ++i) {
                        ++p; char c = *p; cp <<= 4;
                        cp |= c <= '9' ? c - '0' : (c | 32) - 'a' + 10;
                    }
                    if (cp < 0x80) out += char(cp);
                    else if (cp < 0x800) {
                        out += char(0xC0 | (cp >> 6));
                        out += char(0x80 | (cp & 63));
                    } else {
                        out += char(0xE0 | (cp >> 12));
                        out += char(0x80 | ((cp >> 6) & 63));
                        out += char(0x80 | (cp & 63));
                    }
                    break;
                }
                default: out += *p;
                }
                ++p;
            } else {
                out += *p++;
            }
        }
        expect('"');
        return out;
    }
    double number() {
        char* end = nullptr;
        double d = std::strtod(p, &end);
        if (end == p) throw std::runtime_error("json: bad number");
        p = end;
        return d;
    }
};

inline void jsonEscape(std::string& out, const std::string& s) {
    for (char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\t': out += "\\t"; break;
        case '\r': out += "\\r"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", c);
                out += buf;
            } else out += c;
        }
    }
}

// ==========================================================================
// Scene model — mirrors src/hmi/scene.ts
// ==========================================================================

struct Pt { float x = 0, y = 0; };

struct SceneOp {
    enum Kind { Rect, Line, Path, Text } kind = Rect;
    float x = 0, y = 0, w = 0, h = 0;          // rect; line reuses x1y1x2y2
    float sw = 1;                              // stroke width
    std::vector<Pt> pts;                       // path
    bool close = false;
    bool hasFill = false;                      // rect/path fill present
    bool hasStroke = false;                    // rect stroke present
    float cr = 0, cg = 0, cb = 0, ca = 1;      // primary color (stroke/text)
    float fr = 0, fg = 0, fb = 0, fa = 1;      // fill color
    std::string s;                             // text
    float size = 10;
    int anchor = 0;                            // 0=start 1=middle 2=end
};

struct HmiScene {
    float w = 0, h = 0;
    bool hasBg = false;
    float br = 0, bg = 0, bb = 0, ba = 1;
    std::vector<SceneOp> ops;
};

/** '#rgb'/'#rrggbb'/'#rrggbbaa' → floats; false on invalid. */
inline bool parseColor(const std::string& c,
                       float& r, float& g, float& b, float& a) {
    if (c.empty() || c[0] != '#') return false;
    auto nib = [](char ch) -> int {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
        if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
        return -1;
    };
    auto byte = [&](size_t i) { return nib(c[i]) * 16 + nib(c[i + 1]); };
    if (c.size() == 4) {           // #rgb
        r = nib(c[1]) * 17 / 255.f; g = nib(c[2]) * 17 / 255.f;
        b = nib(c[3]) * 17 / 255.f; a = 1.f;
        return true;
    }
    if (c.size() != 7 && c.size() != 9) return false;
    r = byte(1) / 255.f; g = byte(3) / 255.f; b = byte(5) / 255.f;
    a = c.size() == 9 ? byte(7) / 255.f : 1.f;
    return true;
}

inline std::string colorHex(float r, float g, float b, float a = 1.f) {
    char buf[16];
    if (a >= 0.999f)
        std::snprintf(buf, sizeof buf, "#%02x%02x%02x",
                      int(r * 255 + 0.5f), int(g * 255 + 0.5f),
                      int(b * 255 + 0.5f));
    else
        std::snprintf(buf, sizeof buf, "#%02x%02x%02x%02x",
                      int(r * 255 + 0.5f), int(g * 255 + 0.5f),
                      int(b * 255 + 0.5f), int(a * 255 + 0.5f));
    return buf;
}

inline bool opColor(const Json& j, const char* key,
                    float& r, float& g, float& b, float& a) {
    const Json* c = j.find(key);
    if (!c) return false;
    std::string s = c->str();
    if (!parseColor(s, r, g, b, a)) return false;
    return true;
}

inline HmiScene parseScene(const std::string& text) {
    Json root = JsonParser(text).parse();
    HmiScene sc;
    if (const Json* w = root.find("w")) sc.w = float(w->num());
    if (const Json* h = root.find("h")) sc.h = float(h->num());
    if (const Json* bg = root.find("bg")) sc.hasBg = opColor(root, "bg",
        sc.br, sc.bg, sc.bb, sc.ba);
    const Json* ops = root.find("ops");
    if (!ops || !ops->isArr()) return sc;
    for (const Json& jo : ops->arr()) {
        const Json* jk = jo.find("k");
        if (!jk) continue;
        std::string k = jk->str();
        SceneOp op;
        auto num = [&](const char* key, float d = 0.f) {
            const Json* j = jo.find(key);
            return j ? float(j->num()) : d;
        };
        if (k == "rect") {
            op.kind = SceneOp::Rect;
            op.x = num("x"); op.y = num("y");
            op.w = num("w"); op.h = num("h");
            op.hasFill = opColor(jo, "fill", op.fr, op.fg, op.fb, op.fa);
            op.hasStroke = opColor(jo, "stroke", op.cr, op.cg, op.cb, op.ca);
            op.sw = num("sw", 1.f);
        } else if (k == "line") {
            op.kind = SceneOp::Line;
            op.x = num("x1"); op.y = num("y1");
            op.w = num("x2"); op.h = num("y2");
            op.sw = num("w", 1.f);
            opColor(jo, "color", op.cr, op.cg, op.cb, op.ca);
        } else if (k == "path") {
            op.kind = SceneOp::Path;
            if (const Json* pts = jo.find("pts"); pts && pts->isArr())
                for (const Json& pj : pts->arr())
                    if (pj.isArr() && pj.arr().size() >= 2)
                        op.pts.push_back({float(pj.arr()[0].num()),
                                          float(pj.arr()[1].num())});
            if (const Json* c = jo.find("close")) op.close = c->boolean();
            op.sw = num("w", 1.f);
            opColor(jo, "color", op.cr, op.cg, op.cb, op.ca);
            op.hasFill = opColor(jo, "fill", op.fr, op.fg, op.fb, op.fa);
        } else if (k == "text") {
            op.kind = SceneOp::Text;
            op.x = num("x"); op.y = num("y");
            if (const Json* s = jo.find("s")) op.s = s->str();
            op.size = num("size", 10.f);
            if (const Json* a = jo.find("anchor")) {
                std::string an = a->str();
                op.anchor = an == "middle" ? 1 : an == "end" ? 2 : 0;
            }
            opColor(jo, "color", op.cr, op.cg, op.cb, op.ca);
        } else {
            continue;
        }
        sc.ops.push_back(std::move(op));
    }
    return sc;
}

// ---- emit (used by the C++ widget builders — same document the
//      TS builders produce) -----------------------------------------

inline void emitNum(std::string& o, double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.2f", v);
    /* trim trailing zeros for cleaner JSON — values stay exact f32 */
    std::string s = buf;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s == "-0") s = "0";
    o += s;
}

inline void emitOpJson(std::string& o, const SceneOp& op) {
    auto col = [&](const char* key, float r, float g, float b, float a,
                   bool present) {
        if (!present) return;
        o += ",\""; o += key; o += "\":\"";
        o += colorHex(r, g, b, a);
        o += "\"";
    };
    switch (op.kind) {
    case SceneOp::Rect:
        o += "{\"k\":\"rect\",\"x\":"; emitNum(o, op.x);
        o += ",\"y\":"; emitNum(o, op.y);
        o += ",\"w\":"; emitNum(o, op.w);
        o += ",\"h\":"; emitNum(o, op.h);
        col("fill", op.fr, op.fg, op.fb, op.fa, op.hasFill);
        col("stroke", op.cr, op.cg, op.cb, op.ca, op.hasStroke);
        if (op.hasStroke) { o += ",\"sw\":"; emitNum(o, op.sw); }
        break;
    case SceneOp::Line:
        o += "{\"k\":\"line\",\"x1\":"; emitNum(o, op.x);
        o += ",\"y1\":"; emitNum(o, op.y);
        o += ",\"x2\":"; emitNum(o, op.w);
        o += ",\"y2\":"; emitNum(o, op.h);
        col("color", op.cr, op.cg, op.cb, op.ca, true);
        o += ",\"w\":"; emitNum(o, op.sw);
        break;
    case SceneOp::Path:
        o += "{\"k\":\"path\",\"pts\":[";
        for (size_t i = 0; i < op.pts.size(); ++i) {
            if (i) o += ',';
            o += '['; emitNum(o, op.pts[i].x);
            o += ','; emitNum(o, op.pts[i].y); o += ']';
        }
        o += "],\"color\":\"";
        o += colorHex(op.cr, op.cg, op.cb, op.ca); o += "\"";
        o += ",\"w\":"; emitNum(o, op.sw);
        if (op.close) o += ",\"close\":true";
        col("fill", op.fr, op.fg, op.fb, op.fa, op.hasFill);
        break;
    case SceneOp::Text:
        o += "{\"k\":\"text\",\"x\":"; emitNum(o, op.x);
        o += ",\"y\":"; emitNum(o, op.y);
        o += ",\"s\":\""; jsonEscape(o, op.s); o += "\"";
        col("color", op.cr, op.cg, op.cb, op.ca, true);
        o += ",\"size\":"; emitNum(o, op.size);
        if (op.anchor) {
            o += ",\"anchor\":\"";
            o += op.anchor == 1 ? "middle" : "end";
            o += "\"";
        }
        break;
    }
    o += '}';
}

inline std::string sceneToJson(const HmiScene& sc) {
    std::string o = "{\"w\":";
    emitNum(o, sc.w);
    o += ",\"h\":"; emitNum(o, sc.h);
    if (sc.hasBg) {
        o += ",\"bg\":\"";
        o += colorHex(sc.br, sc.bg, sc.bb, sc.ba);
        o += "\"";
    }
    o += ",\"ops\":[";
    for (size_t i = 0; i < sc.ops.size(); ++i) {
        if (i) o += ',';
        emitOpJson(o, sc.ops[i]);
    }
    o += "]}";
    return o;
}

// ---- shared geometry helpers (mirror src/hmi/scene.ts) --------------

/** Arc/polyline points; angles in degrees, 0=+x, positive sweeps
 *  toward +y (screen down). Identical segmentation both backends. */
inline std::vector<Pt> arcPoints(float cx, float cy, float r,
                                 float a0, float a1, float stepDeg = 4.f) {
    float sweep = a1 - a0;
    int n = std::max(2, int(std::ceil(std::fabs(sweep) / stepDeg)));
    std::vector<Pt> pts;
    pts.reserve(size_t(n) + 1);
    for (int i = 0; i <= n; ++i) {
        float a = (a0 + sweep * i / n) * 3.14159265358979f / 180.f;
        pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return pts;
}

inline std::vector<Pt> circlePoints(float cx, float cy, float r,
                                    int n = 48) {
    std::vector<Pt> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; ++i) {
        float a = float(i) / n * 6.28318530717959f;
        pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return pts;
}

inline float arcAngleAt(float a0, float a1, float frac) {
    float f = frac < 0.f ? 0.f : frac > 1.f ? 1.f : frac;
    return a0 + (a1 - a0) * f;
}

}} // namespace tether::hmi
