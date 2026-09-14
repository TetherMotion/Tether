/**
 * @file beckhoff_input_monitor.cpp
 * @brief Beckhoff digital-input terminal monitor (EL1014 family)
 *
 * Finds every EL1014 in the EtherCAT chain via the MultiEL1014 driver,
 * brings all of them to OP, and displays the live state of every input
 * channel.  Module 0 occupies bits 0-3, module 1 bits 4-7, and so on.
 *
 * Two display modes:
 *   - interactive TUI (default on a terminal): a navigable device tree  - 
 *     level 1 = coupler(s) (EK1100 ...), level 2 = the terminals below
 *     each coupler  -  with the selected node's live inputs in the right
 *     pane.  Arrows navigate, left/right fold, q quits.
 *   - --stream: plain stdout  -  one line whenever any input changes.
 *     Selected automatically when ncurses/the terminal can't do a TUI
 *     (non-TTY output, TERM=dumb, or a curses-less build), or when the
 *     user passes --stream.  --interactive forces an interactive attempt.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_input_monitor                # TUI on auto-detected interface
 *   ./beckhoff_input_monitor -i enp3s0      # specify interface
 *   ./beckhoff_input_monitor --stream       # line mode for pipes/scripts
 *   ./beckhoff_input_monitor -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/MultiEL1014.hpp"
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

static const char* TAG = "beckhoff_input_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;
namespace Platform = Tether::Platform;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Compact per-module bit rendering: "s1:1001 s2:0000".
static std::string moduleStates(const Beckhoff::MultiEL1014<>& ins) {
    std::string out;
    for (size_t m = 0; m < ins.moduleCount(); ++m) {
        out += " s" + std::to_string(ins.slaveIndex(m)) + ":";
        const auto& mod = ins.module(m);
        for (size_t k = 0; k < mod.bitCount(); ++k) {
            out += mod.bit(k) ? '1' : '0';
        }
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
// ---------------------------------------------------------------------------
// Interactive TUI  -  device tree left, selected node's live inputs right
// ---------------------------------------------------------------------------

namespace TUI = Tether::TUI;

static void runTui(Beckhoff::MultiEL1014<>& ins,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    // Per-channel transition counters (updated in onTick).
    std::vector<uint32_t> transitions(ins.channelCount(), 0);
    auto prev = ins.bits();

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < ins.moduleCount(); ++m)
            if (ins.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "";
    hooks.onTick = [&]() {
        const auto now = ins.bits();
        for (size_t b = 0; b < ins.channelCount(); ++b) {
            if (now[b] != prev[b]) ++transitions[b];
        }
        prev = now;
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        // Managed input terminal → live channels + transition counts.
        for (size_t m = 0; m < ins.moduleCount(); ++m) {
            if (ins.slaveIndex(m) != static_cast<uint16_t>(node.tag)) continue;
            const auto& mod = ins.module(m);
            mvwprintw(win, row++, 1, "%s  —  slave %d", mod.deviceName(),
                      node.tag);
            ++row;
            mvwprintw(win, row++, 1, "channel :");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                mvwprintw(win, row - 1, 12 + k * 4, "%zu", k + 1);
            }
            mvwprintw(win, row++, 1, "input   :");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                const bool on = mod.bit(k);
                wattron(win, on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
                mvwprintw(win, row - 1, 12 + k * 4, "%s", on ? "*" : ".");
                wattroff(win, on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
            }
            mvwprintw(win, row++, 1, "transit.:");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                mvwprintw(win, row - 1, 12 + k * 4, "%u",
                          transitions[ins.bitOffset(m) + k]);
            }
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
            mvwprintw(win, row++, 1, "(not an EL1014  —  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff input terminals  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ---------------------------------------------------------------------------
// Stream mode  -  one line per input change (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiEL1014<>& ins, double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto prev = ins.bits();

    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    std::cout << std::format("t={:7.3f}{}\n", stamp(), moduleStates(ins));
    std::cout.flush();

    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        const auto now = ins.bits();
        if (now != prev) {
            prev = now;
            std::cout << std::format("t={:7.3f}{}\n",
                                     stamp(), moduleStates(ins));
            std::cout.flush();
        }
        Tether::Platform::Clock::instance().delayMilliseconds(20);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_input_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help("Print input changes as plain lines instead of the TUI")
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

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"), vlan, TAG)) {
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

    if (!Tether::Examples::setupVlanAndRxCallback(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    // ---- Discover the chain and pick out every EL1014 ----
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

    Beckhoff::MultiEL1014<> ins(master);
    auto found = ins.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No EL1014 found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} EL1014 terminal(s), {} input bits total",
                ins.moduleCount(), ins.channelCount());

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = ins.start(); !r) {
        TETHER_LOGE(TAG, "EL1014 bring-up failed on module {}: {}",
                    ins.lastErrorModule(),
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- Display loop ----
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(ins, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(ins, duration_sec);
    }

    // ---- Shutdown ----
    ins.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
