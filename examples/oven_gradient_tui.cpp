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

    void applyPreset(int p) {
        preset = p;
        gc.reset();
        plant = std::make_unique<Simulation::MultiZoneOven>();
        simTime = 0.0;
        for (auto& h : histT) h.clear();
        for (auto& h : histR) h.clear();
        if (p == 1) {
            // Reflow-style: master ramp+soak, spatial gradient via
            // per-zone offsets + conveyor time shifts.
            TemperatureProfile base(25.0);
            base.addRampSoak(145.0, 0.35, 120.0);
            base.addRampSoak(60.0, -0.25, 200.0);   // cool-down zone
            GradientSpec spec;
            spec.base = base;
            spec.tempOffsetC = {0.0, 9.0, 18.0};
            spec.timeShiftS = {0.0, 25.0, 50.0};
            spec.maxZoneDeltaC = 15.0;
            gc.setGradientProfile(spec);
        } else if (p == 2) {
            // Fixed gradient hold: zone setpoints 120/132/144.
            gc.setZoneSetpoints({120.0, 132.0, 144.0}, 15.0);
        } else {
            // Aggressive: steep ramp, tight zones -> delta alarm.
            TemperatureProfile base(25.0);
            base.addRampSoak(200.0, 0.8, 100.0);
            GradientSpec spec;
            spec.base = base;
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

        // ---- footer ----
        attron(COLOR_PAIR(3));
        mvprintw(rows - 1, 0,
                 "space run/pause  r reset  1 reflow  2 hold  3 aggressive  q quit");
        attroff(COLOR_PAIR(3));

        refresh();

        int key = ses.pollKeyDecoded(refreshMs);
        if (key == 'q' || key == 27) break;
        if (key == ' ') sim.running = !sim.running;
        if (key == 'r') sim.applyPreset(sim.preset);
        if (key == '1') sim.applyPreset(1);
        if (key == '2') sim.applyPreset(2);
        if (key == '3') sim.applyPreset(3);
    }
    return 0;
}
