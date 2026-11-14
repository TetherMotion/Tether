/**
 * @file oven_gradient_tui.cpp
 * @brief Interactive terminal UI for the multi-zone oven gradient
 *        controller (OvenGradientController on the MultiZoneOven
 *        benchmark plant).
 *
 * Simulates three thermally coupled heater zones in real time and runs
 * the gradient supervisory layer: per-zone OvenController loops,
 * master-profile + offset/time-shift zone references, adjacent-zone
 * delta constraint, coupling feedforward, and a measured-delta alarm.
 *
 * Layout: one row per zone (measured T, governed reference, actuator
 * u, temperature sparkline), a gradient panel with inter-zone deltas
 * against the configured limit, and a status line.
 *
 * Keys:
 *   space  run / pause simulation
 *   r      reset plant + controllers
 *   1      profile: reflow ramp+soak with spatial gradient
 *   2      profile: setpoint gradient hold (zones at fixed offsets)
 *   3      profile: aggressive ramp (exceeds delta limit -> alarm)
 *   q      quit
 *
 * Usage: ./oven_gradient_tui
 */

// Tether headers first — ncurses macros (OK/ERR/...) collide with
// Tether identifiers and must never be visible during their parse.
#include "tether/control/thermal/OvenGradientController.hpp"
#include "tether/control/thermal/ThermalCurve.hpp"
#include "tether/simulation/systems/thermal/MultiZoneOven.hpp"
#include "tether/terminal_ui/Session.hpp"
#include "tether/ui/Sparkline.hpp"

#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace tether::control::thermal;
using Tether::UI::SparklineHistory;
using Tether::UI::renderSparkline;
using Tether::TUI::Session;

constexpr int kZones = 3;
constexpr double kDt = 0.2;          // sim seconds per control cycle
constexpr int kSubstepsPerFrame = 8; // ~16 sim-s per 50 ms frame -> 80x
constexpr double kPmax = 3000.0;     // W per zone at u=1
constexpr size_t kHistCap = 240;

struct Sim {
    std::unique_ptr<Simulation::MultiZoneOven> plant =
        std::make_unique<Simulation::MultiZoneOven>();
    OvenGradientController gc;
    SparklineHistory<kHistCap> histT[kZones];
    SparklineHistory<kHistCap> histR[kZones];
    double simTime{0.0};
    bool running{true};
    int preset{1};
    int curvePreset{0};   // ThermalCurvePreset index
    TemperatureProfile master;

    Sim() {
        // gamma = 1/(R*C): R12=R23=0.5 K/W, C=3000 J/K -> 6.67e-4 1/s
        const double g = 1.0 / (0.5 * 3000.0);
        const double b0 = 1.0 / (1.0 * 3000.0);
        const double b1 = 1.0 / (1.5 * 3000.0);
        const double b2 = 1.0 / (1.0 * 3000.0);
        for (int i = 0; i < kZones; ++i) {
            GradientZoneConfig z;
            z.controller.model.alpha = 0.9;
            z.controller.model.beta = (i == 1) ? b1 : b0;
            z.controller.model.delay = 1.0;
            z.controller.model.ambient = 25.0;
            z.controller.adaptParams = false;
            z.controller.ditherAmp = 0.0;
            z.couplingLeft = (i > 0) ? g : 0.0;
            z.couplingRight = (i + 1 < kZones) ? g : 0.0;
            gc.addZone(z);
        }
        gc.setGradientAlarmLimit(20.0);
        applyPreset(1);
    }

    TemperatureProfile curveBase() const {
        return presetProfile(
            static_cast<ThermalCurvePreset>(curvePreset));
    }

    void cycleCurve() {
        curvePreset = (curvePreset + 1) %
            int(thermalCurvePresetCount);
        if (preset == 1 || preset == 3) applyPreset(preset);
    }

    void applyPreset(int p) {
        preset = p;
        gc.reset();
        plant = std::make_unique<Simulation::MultiZoneOven>();
        simTime = 0.0;
        for (auto& h : histT) h.clear();
        for (auto& h : histR) h.clear();
        if (p == 1) {
            // Reflow-style: master curve + spatial gradient via
            // per-zone offsets + conveyor time shifts.
            master = curveBase();
            GradientSpec spec;
            spec.base = master;
            spec.tempOffsetC = {0.0, 9.0, 18.0};
            spec.timeShiftS = {0.0, 25.0, 50.0};
            spec.maxZoneDeltaC = 15.0;
            gc.setGradientProfile(spec);
        } else if (p == 2) {
            // Fixed gradient hold: zone setpoints 120/132/144.
            master = TemperatureProfile{};
            gc.setZoneSetpoints({120.0, 132.0, 144.0}, 15.0);
        } else {
            // Aggressive: same master curve stretched hotter via
            // bigger offsets, tight zones -> delta alarm.
            master = curveBase();
            GradientSpec spec;
            spec.base = master;
            spec.tempOffsetC = {0.0, 20.0, 40.0};
            spec.timeShiftS = {0.0, 10.0, 20.0};
            spec.maxZoneDeltaC = 60.0;   // loose: let deltas grow
            gc.setGradientProfile(spec);
        }
    }

    void step() {
        // MultiZoneOven dynamics, Euler at kDt (thermal: stable).
        Simulation::StateVector s{T[0], T[1], T[2]};
        std::vector<double> u;
        for (int k = 0; k < kSubstepsPerFrame; ++k) {
            u = gc.compute(T, kDt);
            Simulation::StateVector in{u[0] * kPmax, u[1] * kPmax,
                                       u[2] * kPmax};
            auto ds = plant->dynamics(simTime, s, in);
            for (int i = 0; i < kZones; ++i) {
                T[i] += ds[i] * kDt;
                s[i] = T[i];
            }
            simTime += kDt;
        }
        for (int i = 0; i < kZones; ++i) {
            histT[i].push(T[i]);
            histR[i].push(gc.zoneStatus()[i].controller.reference);
        }
    }

    double T[kZones]{25.0, 25.0, 25.0};
};

void drawZone(int y, const char* name, const Sim& sim, int zone) {
    const auto& st = sim.gc.zoneStatus()[zone];
    const double t = st.controller.measuredFiltered;
    const double r = st.controller.reference;
    const double u = st.controller.output;
    const double err = t - r;
    WINDOW* w = stdscr;

    wattrset(w, COLOR_PAIR(err > 8.0 ? 4 : (err < -8.0 ? 5 : 1)));
    mvwprintw(w, y, 1, "%-4s %7.1fC  ref %7.1fC  err %+6.1f  u %.2f",
              name, t, r, err, u);
    wattrset(w, COLOR_PAIR(0));

    // u bar
    int bw = 30;
    int filled = int(std::clamp(u, 0.0, 1.0) * bw);
    mvwprintw(w, y, 62, "[");
    for (int i = 0; i < bw; ++i)
        mvwaddch(w, y, 63 + i, i < filled ? '#' : '-');
    mvwaddch(w, y, 63 + bw, ']');

    // sparkline of measured T
    std::string spark = renderSparkline(sim.histT[zone].samples(),
                                        60, 0.0, 220.0);
    mvwprintw(w, y + 1, 1, "%s", spark.c_str());
}

// ASCII temperature-vs-time plot of the master curve. Past portion
// (profile clock <= tNow) drawn solid, future dotted.
void drawCurve(int y, int x, int h, int w, const Sim& sim) {
    if (sim.master.empty() || w < 10 || h < 3) return;
    const double tMax = std::max(sim.master.duration(), 1.0);
    double tHi = 0.0;
    for (double tt = 0.0; tt <= tMax; tt += tMax / 40.0)
        tHi = std::max(tHi, sim.master.value(tt));
    tHi = std::max(tHi, 50.0);
    const double tNow = std::min(sim.simTime, tMax);
    for (int c = 0; c < w; ++c) {
        const double t = tMax * c / (w - 1);
        const double T = sim.master.value(t);
        const int row = h - 1 - int(T / tHi * (h - 1) + 0.5);
        mvwaddch(stdscr, y + std::clamp(row, 0, h - 1), x + c,
                 t <= tNow ? '#' : '.');
    }
    // current-time marker column
    const int mc = int(tNow / tMax * (w - 1) + 0.5);
    for (int r = 0; r < h; ++r)
        mvwaddch(stdscr, y + r, x + mc, '|');
}

} // namespace

int main() {
    if (!Session::available()) {
        std::fprintf(stderr, "oven_gradient_tui: no usable terminal\n");
        return 1;
    }
    Session ses;
    curs_set(0);   // Session already ran start_color + init_pair

    Sim sim;
    const int refreshMs = 50;

    while (true) {
        if (sim.running) sim.step();

        const int rows = LINES, cols = COLS;
        // ---- header ----
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(0, 0, " Oven Gradient Controller — MultiZoneOven (3 zones)");
        attroff(COLOR_PAIR(2) | A_BOLD);
        mvprintw(0, cols - 22, "t=%7.0fs  preset %d",
                 sim.simTime, sim.preset);
        if (sim.running) {
            attron(COLOR_PAIR(1));
            mvprintw(1, 0, " RUNNING");
            attroff(COLOR_PAIR(1));
        } else {
            attron(COLOR_PAIR(3));
            mvprintw(1, 0, " PAUSED ");
            attroff(COLOR_PAIR(3));
        }
        if (sim.gc.gradientAlarm()) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvprintw(1, 12, " GRADIENT ALARM (delta > 20C)");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }

        // ---- zone rows ----
        const char* names[kZones] = {"Z1", "Z2", "Z3"};
        for (int i = 0; i < kZones; ++i)
            drawZone(3 + i * 3, names[i], sim, i);

        // ---- gradient panel ----
        const int gy = 3 + kZones * 3 + 1;
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(gy, 0, " Zone gradient (measured deltas vs 20C limit):");
        attroff(COLOR_PAIR(2) | A_BOLD);
        for (int i = 1; i < kZones; ++i) {
            const double d = sim.T[i] - sim.T[i - 1];
            const bool bad = std::abs(d) > 20.0;
            attron(COLOR_PAIR(bad ? 4 : 1));
            mvprintw(gy + i, 2, "T%d-T%d = %+7.1fC", i + 1, i, d);
            attroff(COLOR_PAIR(bad ? 4 : 1));
        }
        mvprintw(gy + kZones, 2,
                 "max |dT| = %.1fC   max |dRef| = %.1fC",
                 sim.gc.maxMeasuredDelta(), sim.gc.maxSetpointDelta());

        // ---- master curve plot + spec report ----
        const int cy = gy + kZones + 2;
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(cy, 0, " Master curve: %s",
                 thermalCurvePresetName(
                     static_cast<ThermalCurvePreset>(sim.curvePreset)));
        attroff(COLOR_PAIR(2) | A_BOLD);
        drawCurve(cy + 1, 0, 4, cols - 2, sim);
        if (!sim.master.empty()) {
            auto rep = analyzeProfile(
                sim.master,
                specForPreset(static_cast<ThermalCurvePreset>(
                    sim.curvePreset)));
            // metrics ride on the title line; verdict colors it
            attron(COLOR_PAIR(rep.pass() ? 1 : 4));
            mvprintw(cy, 40,
                     "peak %.0f TAL %.0fs soak %.0fs r %.1f/%.1f %s",
                     rep.peakC, rep.talS, rep.soakS,
                     rep.heatRampMax, rep.coolRampMax,
                     rep.pass() ? "PASS" : "FAIL");
            attroff(COLOR_PAIR(rep.pass() ? 1 : 4));
            if (!rep.pass()) {
                attron(COLOR_PAIR(4));
                mvprintw(2, 0, " ! %s",
                         rep.violations.front().c_str());
                attroff(COLOR_PAIR(4));
            }
        }

        // ---- footer ----
        attron(COLOR_PAIR(3));
        mvprintw(rows - 1, 0,
                 "space run/pause  r reset  1/2/3 preset  c curve  q quit");
        attroff(COLOR_PAIR(3));

        refresh();

        int key = ses.pollKeyDecoded(refreshMs);
        if (key == 'q' || key == 27) break;
        if (key == ' ') sim.running = !sim.running;
        if (key == 'r') sim.applyPreset(sim.preset);
        if (key == '1') sim.applyPreset(1);
        if (key == '2') sim.applyPreset(2);
        if (key == '3') sim.applyPreset(3);
        if (key == 'c') sim.cycleCurve();
    }
    return 0;
}
