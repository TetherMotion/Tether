/**
 * @file beckhoff_output_toggle.cpp
 * @brief Beckhoff digital-output terminal stepper demo (EL200x family)
 *
 * Finds every EL200x terminal in the EtherCAT chain via the
 * MultiOutputTerminal driver, brings all of them to OP, and lights exactly
 * one output at a time across the combined channel space (the module at
 * the lowest bus position occupies bits [0, w0), the next one
 * [w0, w0+w1), ... where wN is that terminal's channel count  -  2 for
 * EL2002, 4 for EL2004, 8 for EL2008).  Advancing steps the lit output;
 * after the last channel of the last terminal it wraps to the first.
 *
 * Detected by this example (the "EL200x" series, x = channel count):
 *   EL2002 (2ch, 24V/0.5A), EL2004 (4ch), EL2008 (8ch).
 *   Verified on hardware: EL2004.  EL2002/EL2008 share the identical ESI
 *   shape  -  supported, not verified yet.
 *
 * The same OutputTerminal driver also supports every other terminal in
 * Devices::kOutputTerminals  -  all verified against the ESI to share the
 * EL2004's shape (single "Outputs" SM + FMMU, N x 1-bit RxPDOs, no
 * mailbox), none verified on hardware yet.  Swap kEl200x for
 * Devices::kOutputTerminals in detect() to accept them all.
 *
 * Two display modes:
 *   - interactive TUI (default on a terminal): navigable device tree  - 
 *     level 1 = coupler(s) (EK1100 ...), level 2 = terminals  -  with the
 *     selected node's live outputs in the right pane.  Arrows navigate,
 *     Enter steps the lit output, q quits.
 *   - --stream: plain stdout  -  auto-steps the lit output every 500 ms and
 *     prints one state line per step.  Selected automatically when the
 *     terminal can't do a TUI (non-TTY, TERM=dumb, curses-less build).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_output_toggle                # TUI on auto-detected interface
 *   ./beckhoff_output_toggle -i enp3s0      # specify interface
 *   ./beckhoff_output_toggle --stream       # line mode for pipes/scripts
 *   ./beckhoff_output_toggle -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/MultiOutputTerminal.hpp"
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

static const char* TAG = "beckhoff_output_toggle";

namespace Beckhoff = EtherCAT::Beckhoff;

/// The "EL200x" detection set  -  x = channel count.
constexpr Beckhoff::DeviceIdentity kEl200x[] = {
    Beckhoff::Devices::EL2002,
    Beckhoff::Devices::EL2004,
    Beckhoff::Devices::EL2008,
};

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Compact per-module bit rendering: "s3:0100 s4:0000".
static std::string moduleStates(const Beckhoff::MultiOutputTerminal<>& outs) {
    std::string out;
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        out += " s" + std::to_string(outs.slaveIndex(m)) + ":";
        const auto& mod = outs.module(m);
        for (size_t k = 0; k < mod.bitCount(); ++k) {
            out += mod.bit(k) ? '1' : '0';
        }
    }
    return out;
}

/// Which module+channel a flat chain bit belongs to.
static void locateBit(const Beckhoff::MultiOutputTerminal<>& outs,
                      size_t bit, size_t& mod, size_t& ch) {
    mod = 0;
    for (; mod + 1 < outs.moduleCount(); ++mod) {
        if (outs.bitOffset(mod + 1) > bit) break;
    }
    ch = bit - outs.bitOffset(mod);
}

#ifdef TETHER_HAS_TERMINAL_UI
// ---------------------------------------------------------------------------
// Interactive TUI  -  device tree left, selected node's live outputs right
// ---------------------------------------------------------------------------

namespace TUI = Tether::TUI;

static void runTui(Beckhoff::MultiOutputTerminal<>& outs,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    size_t active = 0;
    outs.setOnly(active);

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < outs.moduleCount(); ++m)
            if (outs.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "enter: next output";
    hooks.onKey = [&](int key) {
        if (key != '\n' && key != '\r' && key != KEY_ENTER) return false;
        active = (active + 1) % outs.channelCount();
        outs.setOnly(active);
        return true;
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int row = 1;

        // Managed output terminal → live channels, lit one highlighted.
        for (size_t m = 0; m < outs.moduleCount(); ++m) {
            if (outs.slaveIndex(m) != static_cast<uint16_t>(node.tag)) continue;
            const auto& mod = outs.module(m);
            mvwprintw(win, row++, 1, "%s  —  slave %d", mod.deviceName(),
                      node.tag);
            ++row;
            mvwprintw(win, row++, 1, "channel :");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                mvwprintw(win, row - 1, 12 + k * 4, "%zu", k + 1);
            }
            mvwprintw(win, row++, 1, "output  :");
            const size_t base = outs.bitOffset(m);
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                const bool on = mod.bit(k);
                const bool activeBit = on && (base + k == active);
                const int attrs = activeBit
                    ? (COLOR_PAIR(TUI::PalSelected) | A_BOLD)
                    : (on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
                wattron(win, attrs);
                mvwprintw(win, row - 1, 12 + k * 4, "%s", on ? "*" : ".");
                wattroff(win, attrs);
            }
            ++row;
            size_t am = 0, ac = 0;
            locateBit(outs, active, am, ac);
            mvwprintw(win, row++, 1,
                      "active: bit %zu = slave %u CH%zu (enter: next)",
                      active, outs.slaveIndex(am), ac + 1);
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
        } else {
            ++row;
            wattron(win, A_DIM);
            mvwprintw(win, row++, 1,
                      "(not an EL200x  —  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff output terminals  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ---------------------------------------------------------------------------
// Stream mode  -  auto-step the lit output, one line per step (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiOutputTerminal<>& outs,
                    double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    size_t active = 0;
    outs.setOnly(active);
    std::cout << std::format("t={:7.3f} bit {:>3}{}\n",
                             stamp(), active, moduleStates(outs));
    std::cout.flush();

    auto next_step = std::chrono::steady_clock::now();
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_step) {
            next_step = now + std::chrono::milliseconds(500);
            active = (active + 1) % outs.channelCount();
            outs.setOnly(active);
            std::cout << std::format("t={:7.3f} bit {:>3}{}\n",
                                     stamp(), active, moduleStates(outs));
            std::cout.flush();
        }
        Tether::Platform::Clock::instance().delayMilliseconds(20);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_output_toggle", "1.0",
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

    // ---- Discover the chain and pick out every EL200x ----
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

    Beckhoff::MultiOutputTerminal<> outs(master);
    auto found = outs.detect(std::span<const Beckhoff::DeviceIdentity>(kEl200x),
                           slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No EL200x terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} EL200x terminal(s), {} output bits total",
                outs.moduleCount(), outs.channelCount());
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        TETHER_LOGI(TAG, "  module {}: slave {} = {} ({} ch)",
                    m, outs.slaveIndex(m), outs.module(m).deviceName(),
                    outs.module(m).bitCount());
    }

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = outs.start(); !r) {
        TETHER_LOGE(TAG, "EL200x bring-up failed on module {}: {}",
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

    // ---- Shutdown: all outputs off, then stop ----
    outs.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. All outputs off.");
    return 0;
}
