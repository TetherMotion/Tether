/**
 * @file beckhoff_analog_output_sweep.cpp
 * @brief Beckhoff analog-output terminal sweep demo (EL4xxx family)
 *
 * Finds every known analog-output terminal (EL400x–EL4374) in the
 * EtherCAT chain via MultiAnalogOutputTerminal, brings all of them to OP,
 * and sweeps one channel at a time with a triangle ramp across the raw
 * output range (-32767 .. +32767); after a full cycle it advances to the
 * next channel of the flat channel space and wraps after the last.
 *
 * Two display modes:
 *   - interactive TUI (default on a terminal): navigable device tree  - 
 *     level 1 = coupler(s) (EK1100 ...), level 2 = terminals  -  with the
 *     selected node's live outputs in the right pane.  Arrows navigate,
 *     Enter skips to the next channel, space pauses the ramp, q quits.
 *   - --stream: plain stdout  -  auto-sweeps and prints one state line per
 *     channel change (plus periodic progress).  Selected automatically
 *     when the terminal can't do a TUI (non-TTY, TERM=dumb, curses-less
 *     build).
 *
 * CAUTION: this demo drives real outputs  -  a ±10 V terminal outputs
 * ±10 V.  Only run with safe loads attached.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_analog_output_sweep                # TUI, auto-detected NIC
 *   ./beckhoff_analog_output_sweep -i enp3s0      # specify interface
 *   ./beckhoff_analog_output_sweep --stream       # line mode for pipes
 *   ./beckhoff_analog_output_sweep -t 30          # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/MultiAnalogOutputTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include "tether/terminal_ui/Session.hpp"
#include "tether/terminal_ui/TreeScreen.hpp"
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

// ncurses last: it #defines OK/ERR/timeout/... which collide with
// identifiers in the Tether headers (e.g. HALTypes' enum class Error::OK).
#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
#endif

static const char* TAG = "beckhoff_analog_output_sweep";

namespace Beckhoff = EtherCAT::Beckhoff;
namespace Platform = Tether::Platform;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Sweep state shared by both display modes.
struct Sweep {
    size_t   channel = 0;          ///< flat channel being driven
    int32_t  value   = -32767;     ///< current output value
    int      dir     = +400;       ///< ramp step (signed)
    bool     paused  = false;

    /// Advance the ramp one step; wraps to the next channel after a full
    /// up-down cycle.  Returns true when the active channel changed.
    bool tick(Beckhoff::MultiAnalogOutputTerminal<>& outs) {
        if (paused || outs.channelCount() == 0) return false;
        value += dir;
        if (value > 32767)  { value = 32767;  dir = -dir; }
        if (value < -32767) { value = -32767; dir = -dir; return next(outs); }
        outs.setValue(channel, value);
        return false;
    }

    /// Move to the next flat channel; the old channel returns to 0.
    bool next(Beckhoff::MultiAnalogOutputTerminal<>& outs) {
        outs.setValue(channel, 0);
        channel = (channel + 1) % outs.channelCount();
        value   = -32767;
        dir     = +400;
        return true;
    }
};

/// Compact per-module value rendering: "s3:+01234,-00042 ...".
static std::string moduleStates(
    const Beckhoff::MultiAnalogOutputTerminal<>& outs) {
    std::string out;
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        if (m) out += " ";
        out += "s" + std::to_string(outs.slaveIndex(m)) + ":";
        const auto& mod = outs.module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (k) out += ",";
            out += std::format("{:+d}", mod.value(k));
        }
    }
    return out;
}

/// Which module+channel a flat chain channel belongs to.
static void locateChannel(
    const Beckhoff::MultiAnalogOutputTerminal<>& outs,
    size_t channel, size_t& mod, size_t& ch) {
    mod = 0;
    for (; mod + 1 < outs.moduleCount(); ++mod) {
        if (outs.channelOffset(mod + 1) > channel) break;
    }
    ch = channel - outs.channelOffset(mod);
}

#ifdef TETHER_HAS_TERMINAL_UI
// ---------------------------------------------------------------------------
// Interactive TUI  -  device tree left, selected node's live outputs right
// ---------------------------------------------------------------------------

namespace TUI = Tether::TUI;

static void runTui(Beckhoff::MultiAnalogOutputTerminal<>& outs,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    Sweep sweep;

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < outs.moduleCount(); ++m)
            if (outs.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "enter: next channel  space: pause";
    hooks.onTick = [&]() { sweep.tick(outs); };
    hooks.onKey = [&](int key) {
        if (key == '\n' || key == '\r' || key == KEY_ENTER) {
            sweep.next(outs);
            return true;
        }
        if (key == ' ') {
            sweep.paused = !sweep.paused;
            return true;
        }
        return false;
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        // Managed output terminal → live channels, swept one highlighted.
        for (size_t m = 0; m < outs.moduleCount(); ++m) {
            if (outs.slaveIndex(m) != static_cast<uint16_t>(node.tag)) continue;
            const auto& mod = outs.module(m);
            mvwprintw(win, row++, 1, "%s  —  slave %d", mod.deviceName(),
                      node.tag);
            ++row;
            mvwprintw(win, row++, 1, "ch   value");
            const size_t base = outs.channelOffset(m);
            for (size_t k = 0; k < mod.channelCount() && row < h - 3; ++k) {
                const int32_t v  = mod.value(k);
                const bool  act  = !sweep.paused && (base + k == sweep.channel);
                mvwprintw(win, row, 1, "%zu", k + 1);
                wattron(win, act ? (COLOR_PAIR(TUI::PalSelected) | A_BOLD)
                                 : (COLOR_PAIR(TUI::PalValue) | A_BOLD));
                mvwprintw(win, row, 4, "%+d", v);
                wattroff(win, act ? (COLOR_PAIR(TUI::PalSelected) | A_BOLD)
                                  : (COLOR_PAIR(TUI::PalValue) | A_BOLD));
                // Simple bar: position within [-32767, 32767] -> 0..20 cols.
                const int pos = static_cast<int>(
                    (static_cast<long>(v) + 32767) * 20 / 65534);
                mvwaddch(win, row, 12 + pos, '|');
                ++row;
            }
            ++row;
            size_t am = 0, ac = 0;
            locateChannel(outs, sweep.channel, am, ac);
            mvwprintw(win, row++, 1,
                      "sweep: ch %zu = slave %u CH%zu value %+d%s",
                      sweep.channel, outs.slaveIndex(am), ac + 1,
                      sweep.value, sweep.paused ? " (paused)" : "");
            return;
        }

        // Coupler or unmanaged node → identity info.
        const auto* s = slaveByIndex(slaves, node.tag);
        if (s) {
            mvwprintw(win, row++, 1, "%s",
                      s->device_name ? s->device_name->c_str() : "?");
            mvwprintw(win, row++, 1, "slave   : %u", s->index);
            mvwprintw(win, row++, 1, "vendor  : 0x%08X",
                      s->vendor_id ? *s->vendor_id : 0);
            mvwprintw(win, row++, 1, "product : 0x%08X",
                      s->product_code ? *s->product_code : 0);
        }
        if (!node.children.empty()) {
            ++row;
            mvwprintw(win, row++, 1, "%zu terminal(s) below",
                      node.children.size());
        } else if (row < h) {
            ++row;
            wattron(win, A_DIM);
            mvwprintw(win, row++, 1,
                      "(not an analog output  —  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff analog outputs  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ---------------------------------------------------------------------------
// Stream mode  -  auto-sweep, one line per channel change (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiAnalogOutputTerminal<>& outs,
                      double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    Sweep sweep;
    std::cout << std::format("t={:7.3f} ch {:>3} {:+d}  {}\n",
                             stamp(), sweep.channel, sweep.value,
                             moduleStates(outs));
    std::cout.flush();

    auto next_step = std::chrono::steady_clock::now();
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_step) {
            next_step = now + std::chrono::milliseconds(5);  // ~1.3 s/ramp
            if (sweep.tick(outs)) {
                std::cout << std::format("t={:7.3f} ch {:>3} {:+d}  {}\n",
                                         stamp(), sweep.channel, sweep.value,
                                         moduleStates(outs));
                std::cout.flush();
            }
        }
        Tether::Platform::Clock::instance().delayMilliseconds(2);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_analog_output_sweep", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help("Print output state lines instead of the TUI")
        .flag();

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    if (program.get<bool>("--list-interfaces")) {
        Tether::Examples::listPhysicalInterfaces(TAG);
        return 0;
    }

    std::string iface =
        Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::EncapsulationConfig encapsulation;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encapsulation, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    // --stream wins when both are given; otherwise interactive is the
    // default and falls back to stream when the terminal can't do a TUI.
    bool interactive = !program.get<bool>("--stream");
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal  —  falling back to --stream");
        }
        interactive = false;
    }
#else
    if (interactive) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "built without ncurses  —  using --stream mode");
        }
        interactive = false;
    }
#endif

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    // ---- Host Ethernet + master bring-up ----
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
    Tether::Examples::applyDebugFlags(debug_flags, master, TAG);

    if (!Tether::Examples::setupEncapsulation(session, master, encapsulation, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    // ---- Discover the chain and pick out every analog-output terminal ----
    // A full discovery gives us the slave names for logging and lets the
    // driver reuse the SII data instead of re-reading each terminal's EEPROM.
    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    TETHER_LOGI(TAG, "=== Discovered {} slave(s) ===", slaves.size());
    for (const auto& s : slaves) {
        TETHER_LOGI(TAG, "Slave {}: {} (vendor=0x{:08X} product=0x{:08X})",
                    s.index,
                    s.device_name ? s.device_name->c_str() : "?",
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);
    }

    Beckhoff::MultiAnalogOutputTerminal<> outs(master);
    auto found = outs.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No analog-output terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} analog output terminal(s), {} channel(s) total",
                outs.moduleCount(), outs.channelCount());
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        TETHER_LOGI(TAG, "  module {}: slave {} = {} ({} ch)",
                    m, outs.slaveIndex(m), outs.module(m).deviceName(),
                    outs.module(m).channelCount());
    }

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = outs.start(); !r) {
        TETHER_LOGE(TAG, "Analog output bring-up failed on module {}: {}",
                    outs.lastErrorModule(),
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- Display loop ----
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(outs, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(outs, duration_sec);
    }

    // ---- Shutdown: all outputs to 0, then stop ----
    outs.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. All outputs zeroed.");
    return 0;
}
