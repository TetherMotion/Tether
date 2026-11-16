// HmiAuto.cpp — catalog → HMI screen rules.
// Mirror of TetherWebUI/src/hmi/autoscreen.ts — keep in lockstep.
#include "HmiAuto.hpp"
#include <algorithm>
#include <cctype>
#include <map>
#include <set>

namespace tether { namespace hmi {
namespace {

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower((unsigned char)c));
    return s;
}

std::string baseName(const std::string& path) {
    auto pos = path.find_last_of('.');
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

bool isPlc(const std::string& name) {
    return name.rfind("plc.", 0) == 0;
}

/** Split "plc.<prog>.<rest...>" → prog + member path. */
bool splitPlc(const std::string& name, std::string& prog,
               std::string& rest) {
    if (!isPlc(name)) return false;
    auto p1 = name.find('.', 4);
    if (p1 == std::string::npos) return false;
    prog = name.substr(4, p1 - 4);
    rest = name.substr(p1 + 1);
    return !prog.empty() && !rest.empty();
}

/** Instance prefix = everything before the last dot of rest
 *  ("loop1.PV" → "loop1"); empty when rest has no dot. */
std::string instOf(const std::string& rest) {
    auto pos = rest.find_last_of('.');
    return pos == std::string::npos ? std::string() : rest.substr(0, pos);
}

bool hasMember(const std::map<std::string, const CatalogSignal*>& members,
               const char* m) {
    return members.count(m) != 0;
}

WidgetSpec digitalWidget(const std::string& sig, const char* label) {
    WidgetSpec w;
    w.type = "digital"; w.w = 160; w.h = 64;
    w.options = {{"label", label}};
    w.signals = {sig};
    return w;
}

} // namespace

HmiScreen autoScreen(const std::vector<CatalogSignal>& entries) {
    HmiScreen screen;

    // group plc.* leaves: prog → instance → member → signal
    struct Group { std::map<std::string, const CatalogSignal*> members; };
    std::map<std::string, std::map<std::string, Group>> byProg;
    std::vector<const CatalogSignal*> scalars;   // prog.var (no instance)
    for (const auto& e : entries) {
        if (!isPlc(e.name)) continue;
        std::string prog, rest;
        if (!splitPlc(e.name, prog, rest)) continue;
        std::string inst = instOf(rest);
        std::string member = inst.empty() ? rest : baseName(rest);
        if (inst.empty()) {
            scalars.push_back(&e);
        } else {
            byProg[prog][inst].members[member] = &e;
        }
    }

    // --- instance groups -------------------------------------------------
    static const std::set<std::string> kProfileMembers = {
        "Done", "Busy", "Err", "SegIdx", "SegTime", "SegTgt",
        "RateNow", "RemTime", "SP", "EV", "RUN", "HOLD", "RESET",
        "NUM_SEG", "PV",
    };
    static const std::set<std::string> kLoopMembers = {
        "PV", "SP", "OUT", "U", "EV", "Q",
    };
    static const std::set<std::string> kBoolNames = {
        "run", "enable", "auto", "start", "stop", "fault", "alarm",
        "done", "busy", "err", "error", "ok", "ready", "hold",
        "reset", "active", "inpos", "homed", "estop", "perm",
    };

    for (auto& [prog, insts] : byProg) {
        for (auto& [inst, g] : insts) {
            const std::string prefix = "plc." + prog + "." + inst;
            auto memberName = [&](const char* m) -> std::string {
                auto it = g.members.find(m);
                return it == g.members.end() ? std::string()
                                             : it->second->name;
            };

            // profile panel: FB_AtcProfile-shaped instance
            int profHits = 0;
            for (const char* m : {"Done", "SegIdx", "SegTgt", "RateNow",
                                  "RemTime", "Busy", "Err"})
                if (hasMember(g.members, m)) ++profHits;
            if (profHits >= 3) {
                WidgetSpec w;
                w.type = "profile"; w.w = 300; w.h = 170;
                w.options = {{"label", inst}};
                for (const char* m : {"Done", "Busy", "Err", "SegIdx",
                                      "SegTime", "SegTgt", "RateNow",
                                      "RemTime", "SP", "EV", "RUN",
                                      "HOLD", "RESET", "NUM_SEG", "PV"}) {
                    std::string n = memberName(m);
                    if (!n.empty()) w.signals.push_back(n);
                }
                screen.widgets.push_back(std::move(w));
                continue;
            }

            // loop cluster: PV+SP present → digital PV + digital SP
            if (hasMember(g.members, "PV") && hasMember(g.members, "SP")) {
                screen.widgets.push_back(
                    digitalWidget(memberName("PV"), (inst + " PV").c_str()));
                screen.widgets.push_back(
                    digitalWidget(memberName("SP"), (inst + " SP").c_str()));
                std::string out = memberName("OUT");
                if (out.empty()) out = memberName("U");
                if (out.empty()) out = memberName("EV");
                if (!out.empty()) {
                    WidgetSpec w;
                    w.type = "bar"; w.w = 220; w.h = 56;
                    w.options = {{"label", (inst + " OUT").c_str()}};
                    w.signals = {out};
                    screen.widgets.push_back(std::move(w));
                }
                continue;
            }

            // boolean-ish members → lamp stack (up to 3)
            std::vector<std::string> bools;
            for (auto& [m, sig] : g.members) {
                std::string lm = lower(m);
                if (kBoolNames.count(lm)) bools.push_back(sig->name);
                if (bools.size() == 3) break;
            }
            if (!bools.empty() && bools.size() == g.members.size()) {
                WidgetSpec w;
                w.type = "lamp"; w.w = 72; w.h = 170;
                w.options = {{"label", inst}};
                w.signals = bools;
                screen.widgets.push_back(std::move(w));
                continue;
            }

            // remaining instance members → digital readouts (cap 4)
            int n = 0;
            for (auto& [m, sig] : g.members) {
                if (n >= 4) break;
                if (kProfileMembers.count(m) || kLoopMembers.count(m))
                    continue;
                screen.widgets.push_back(
                    digitalWidget(sig->name, (inst + " " + m).c_str()));
                ++n;
            }
        }
    }

    // --- program-level scalars ------------------------------------------
    static const std::set<std::string> kScalarSkip = {
        "retain", "image", "status", "version", "name",
    };
    int n = 0;
    for (const CatalogSignal* e : scalars) {
        if (n >= 12) break;
        std::string rest = e->name.substr(e->name.find('.', 4) + 1);
        /* strip prog prefix: plc.<prog>.<var> → var may itself be dotted
           for multi-POU layouts; use the leaf + parent hint as label */
        std::string leaf = baseName(rest);
        std::string ll = lower(leaf);
        bool skip = false;
        for (const auto& k : kScalarSkip)
            if (ll.find(k) != std::string::npos) { skip = true; break; }
        if (skip) continue;
        // boolean-ish leaf → collect into one lamp per program
        WidgetSpec w;
        w.type = "digital"; w.w = 160; w.h = 64;
        w.options = {{"label", leaf}};
        w.signals = {e->name};
        screen.widgets.push_back(std::move(w));
        ++n;
    }

    // always append the alarm board — the journal exists on every server
    {
        WidgetSpec w;
        w.type = "alarms"; w.w = 240; w.h = 140;
        w.options = {{"title", "ALARMS"}};
        screen.widgets.push_back(std::move(w));
    }
    return screen;
}

// ---- screen JSON ------------------------------------------------------

HmiScreen parseScreenJson(const std::string& text) {
    HmiScreen screen;
    Json root = JsonParser(text).parse();
    const Json* ws = root.find("widgets");
    if (!ws || !ws->isArr()) return screen;
    for (const Json& jw : ws->arr()) {
        WidgetSpec w;
        if (const Json* t = jw.find("type")) w.type = t->str();
        auto num = [&](const char* k, float d) {
            const Json* j = jw.find(k); return j ? float(j->num()) : d;
        };
        w.w = num("w", 160.f); w.h = num("h", 64.f);
        if (const Json* opts = jw.find("options"); opts && opts->isObj())
            for (auto& [k, v] : opts->obj())
                w.options.emplace_back(k, v.str());
        if (const Json* sigs = jw.find("signals"); sigs && sigs->isArr())
            for (const Json& s : sigs->arr()) w.signals.push_back(s.str());
        if (!w.type.empty()) screen.widgets.push_back(std::move(w));
    }
    return screen;
}

std::string screenToJson(const HmiScreen& screen) {
    std::string o = "{\"widgets\":[";
    for (size_t i = 0; i < screen.widgets.size(); ++i) {
        const WidgetSpec& w = screen.widgets[i];
        if (i) o += ',';
        o += "{\"type\":\""; o += w.type; o += "\"";
        o += ",\"w\":"; { char b[16]; std::snprintf(b,16,"%.0f",double(w.w)); o += b; }
        o += ",\"h\":"; { char b[16]; std::snprintf(b,16,"%.0f",double(w.h)); o += b; }
        if (!w.options.empty()) {
            o += ",\"options\":{";
            for (size_t j = 0; j < w.options.size(); ++j) {
                if (j) o += ',';
                o += '"'; jsonEscape(o, w.options[j].first);
                o += "\":\""; jsonEscape(o, w.options[j].second); o += '"';
            }
            o += '}';
        }
        if (!w.signals.empty()) {
            o += ",\"signals\":[";
            for (size_t j = 0; j < w.signals.size(); ++j) {
                if (j) o += ',';
                o += '"'; jsonEscape(o, w.signals[j]); o += '"';
            }
            o += ']';
        }
        o += '}';
    }
    o += "]}";
    return o;
}

}} // namespace tether::hmi
