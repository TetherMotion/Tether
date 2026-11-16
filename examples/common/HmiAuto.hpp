// HmiAuto.hpp — automatic HMI screen generation from the IO catalog.
//
// "Automagically tying into the PLC code": given the catalog entries a
// Tether IO server exposes (plc.<prog>.<var>[.<member>] published by
// tplc-io / the PLC bridge), derive a screen of widgets with no manual
// binding. The same rules are implemented in
// TetherWebUI/src/hmi/autoscreen.ts so web and native clients generate
// identical screens from the same catalog.
//
// Screen document (JSON, shared contract):
//   { "widgets": [ { "type": "digital|lamp|profile|alarms|gauge|bar",
//                    "w": 160, "h": 64,
//                    "options": { "label": "...", "unit": "...", ... },
//                    "signals": ["plc.Main.loop.PV", ...] } ] }
//
// Layout: widgets flow left→right, top→bottom on a fixed grid
// (cellW x cellH); `screenLayout()` computes positions deterministically.
#pragma once

#include "HmiScene.hpp"
#include <optional>
#include <string>
#include <vector>

namespace tether { namespace hmi {

/** One catalog entry as the auto-binder sees it (name + writability). */
struct CatalogSignal {
    std::string name;        // e.g. "plc.Main.loop1.PV"
    bool writable = false;   // param vs read-only signal
    std::string description;
};

struct WidgetSpec {
    std::string type;                       // digital|lamp|profile|alarms|gauge|bar
    float w = 160, h = 64;                  // design-space size
    std::vector<std::pair<std::string, std::string>> options; // label/unit/min/…
    std::vector<std::string> signals;       // catalog names, order = binding order
};

struct HmiScreen {
    std::vector<WidgetSpec> widgets;
};

// ---- catalog → screen ------------------------------------------------

/**
 * Derive widgets from catalog entries. Rules (v1):
 *  - plc.<prog>.<inst>.{Done,Busy,Err,SegIdx,SegTime,SegTgt,RateNow,
 *    RemTime} present → "profile" widget bound to those members.
 *  - plc.<prog>.<inst>.{PV,SP[,OUT|U|EV]} present → "loop" widget
 *    cluster (PV digital + SP digital + OUT bar placeholder).
 *  - Boolean-ish leaf names (run|enable|auto|start|stop|fault|alarm|
 *    done|busy|err|ok|ready|hold|reset) grouped per instance → "lamp".
 *  - Remaining scalar plc.* leaves → "digital" readouts.
 *  - Non-plc.* entries are ignored (HMI binds PLC namespace only).
 */
HmiScreen autoScreen(const std::vector<CatalogSignal>& entries);

// ---- screen JSON -----------------------------------------------------

HmiScreen parseScreenJson(const std::string& json);
std::string screenToJson(const HmiScreen& screen);

// ---- layout ----------------------------------------------------------

/** Deterministic grid layout: cell size + origin, returns top-left
 *  position per widget index (same result on every backend). */
struct ScreenLayout {
    float cellW = 170.f, cellH = 76.f;
    float originX = 8.f, originY = 8.f;
    int cols = 4;
};

/** (x, y) top-left for widget `index` under `layout`. */
inline void widgetPos(const ScreenLayout& l, size_t index,
                      float& x, float& y) {
    x = l.originX + float(index % size_t(l.cols)) * l.cellW;
    y = l.originY + float(index / size_t(l.cols)) * l.cellH;
}

/** Total pixel size needed to contain `n` widgets. */
inline void screenSize(const ScreenLayout& l, size_t n,
                       float& w, float& h) {
    int rows = int((n + size_t(l.cols) - 1) / size_t(l.cols));
    w = l.originX * 2.f + float(l.cols) * l.cellW;
    h = l.originY * 2.f + float(std::max(1, rows)) * l.cellH;
}

}} // namespace tether::hmi
