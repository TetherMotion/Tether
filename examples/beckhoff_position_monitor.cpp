/**
 * @file beckhoff_position_monitor.cpp
 * @brief Beckhoff position/encoder terminal monitor (EL5xxx family)
 *
 * Finds every known position-input terminal (EL500x SSI, EL5021 SinCos,
 * EL503x/EL5042 EnDat/BiSS, EL5072 displacement, EL51xx incremental) in
 * the EtherCAT chain via MultiPositionInputTerminal, brings all of them
 * to OP, and displays the live counter/latch values of every channel.
 *
 * Two display modes:
 *   - interactive TUI (default on a terminal): a navigable device tree  - 
 *     level 1 = coupler(s) (EK1100 ...), level 2 = the terminals below
 *     each coupler  -  with the selected node's live channels in the right
 *     pane.  Arrows navigate, left/right fold, q quits.
 *   - --stream: plain stdout  -  one line per poll with all positions.
 *     Selected automatically when ncurses/the terminal can't do a TUI
 *     (non-TTY output, TERM=dumb, or a curses-less build), or when the
 *     user passes --stream.  --interactive forces an interactive attempt.
 *
 * Values are raw field-bus integers (encoder counts); scaling to
 * revolutions/length units is application territory.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_position_monitor                # TUI, auto-detected NIC
 *   ./beckhoff_position_monitor -i enp3s0      # specify interface
 *   ./beckhoff_position_monitor --stream       # line mode for pipes
 *   ./beckhoff_position_monitor -t 30          # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/MultiPositionInputTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"
#include "logging/Logger.hpp"

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

static const char* TAG = "beckhoff_position_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;
namespace Platform = Tether::Platform;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Compact per-module value rendering: "s1:+1234/-7 s2:...".
static std::string moduleStates(
    const Beckhoff::MultiPositionInputTerminal<>& encs) {
    std::string out;
    for (size_t m = 0; m < encs.moduleCount(); ++m) {
        if (m) out += " ";
        out += "s" + std::to_string(encs.slaveIndex(m)) + ":";
        const auto& mod = encs.module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (k) out += ",";
            out += std::format("{:+d}", mod.value(k, 0));
        }
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
// ---------------------------------------------------------------------------
// Interactive TUI  -  device tree left, selected node's live channels right
// ---------------------------------------------------------------------------

namespace TUI = Tether::TUI;

static void runTui(Beckhoff::MultiPositionInputTerminal<>& encs,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < encs.moduleCount(); ++m)
            if (encs.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "";
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        // Managed position terminal → live channel values.
        for (size_t m = 0; m < encs.moduleCount(); ++m) {
            if (encs.slaveIndex(m) != static_cast<uint16_t>(node.tag))
                continue;
            const auto& mod = encs.module(m);
            mvwprintw(win, row++, 1, "%s  —  slave %d", mod.deviceName(),
                      node.tag);
            ++row;
            mvwprintw(win, row++, 1, "ch   position           status");
            for (size_t k = 0; k < mod.channelCount() && row < h - 1; ++k) {
                mvwprintw(win, row, 1, "%zu", k + 1);
                wattron(win, COLOR_PAIR(TUI::PalValue) | A_BOLD);
                mvwprintw(win, row, 4, "%+lld",
                          static_cast<long long>(mod.value(k, 0)));
                wattroff(win, COLOR_PAIR(TUI::PalValue) | A_BOLD);
                // Additional value fields (latch, ...) in dim.
                wattron(win, A_DIM);
                for (size_t i = 1;
                     i < mod.valueCount(k) && i < 4; ++i) {
                    mvwprintw(win, row, 18 + 13 * int(i - 1), "%+lld",
                              static_cast<long long>(mod.value(k, i)));
                }
                wattroff(win, A_DIM);
                if (mod.hasStatus(k)) {
                    const uint16_t st = mod.status(k);
                    wattron(win, st ? COLOR_PAIR(TUI::PalError) : A_DIM);
                    mvwprintw(win, row, 56, "0x%04X", st);
                    wattroff(win, st ? COLOR_PAIR(TUI::PalError) : A_DIM);
                }
                ++row;
            }
            if (encs.moduleCount() == 0) break;
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
                      "(not a position input  —  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff position inputs  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ---------------------------------------------------------------------------
// Stream mode  -  one line per poll (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiPositionInputTerminal<>& encs,
                      double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();

    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        std::cout << std::format("t={:7.3f}  {}\n", stamp(),
                                 moduleStates(encs));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(200);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_position_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help("Print channel values as plain lines instead of the TUI")
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
        Tether::Examples::resolveInterface(
            program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::EncapConfig encap;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encap, TAG)) {
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

    if (!Tether::Examples::setupEncapAndRxCallback(session, master, encap, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, encap, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    // ---- Discover the chain and pick out every position terminal ----
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

    Beckhoff::MultiPositionInputTerminal<> encs(master);
    auto found = encs.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No position-input terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} position terminal(s), {} channel(s) total",
                encs.moduleCount(), encs.channelCount());

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = encs.start(); !r) {
        TETHER_LOGE(TAG, "Position input bring-up failed on module {}: {}",
                    encs.lastErrorModule(),
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- Display loop ----
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(encs, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(encs, duration_sec);
    }

    // ---- Shutdown ----
    encs.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
