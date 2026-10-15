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
#include "tether/ethercat/HostSetup.hpp"
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"

// ncurses last: it #defines OK/ERR/timeout/... which collide with
// identifiers in the Tether headers (e.g. HALTypes' enum class Error::OK).
// Widget headers that render via curses calls go after <ncurses.h>.
#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
#include "tether/terminal_ui/StyledText.hpp"
#include "tether/terminal_ui/Lamp.hpp"
#include "tether/terminal_ui/RowWriter.hpp"
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
        TUI::RowWriter rw(win, 1, 1);

        // Managed input terminal → live channels + transition counts.
        for (size_t m = 0; m < ins.moduleCount(); ++m) {
            if (ins.slaveIndex(m) != static_cast<uint16_t>(node.tag)) continue;
            const auto& mod = ins.module(m);
            rw.text("%s  —  slave %d", mod.deviceName(), node.tag);
            rw.blank();
            rw.put(1, 0, "channel :");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                rw.put(12 + k * 4, 0, "%zu", k + 1);
            }
            rw.advance();
            rw.put(1, 0, "input   :");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                rw.putLamp(12 + k * 4,
                           mod.bit(k) ? TUI::LampState::On
                                      : TUI::LampState::Off,
                           TUI::kLampBits);
            }
            rw.advance();
            rw.put(1, 0, "transit.:");
            for (size_t k = 0; k < mod.bitCount(); ++k) {
                rw.put(12 + k * 4, 0, "%u",
                       transitions[ins.bitOffset(m) + k]);
            }
            rw.advance();
            return;
        }

        // Coupler or unmanaged node → identity info.
        const auto* s = slaveByIndex(slaves, node.tag);
        if (s) {
            rw.text("%s", s->device_name ? s->device_name->c_str() : "?");
            rw.text("slave   : %u", s->index);
            rw.text("vendor  : 0x%08X", s->vendor_id ? *s->vendor_id : 0);
            rw.text("product : 0x%08X",
                    s->product_code ? *s->product_code : 0);
        }
        if (!node.children.empty()) {
            rw.blank();
            rw.text("%zu terminal(s) below", node.children.size());
        } else if (rw.fits()) {
            rw.blank();
            rw.styled(A_DIM, "(not an EL1014  —  not managed by this demo)");
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
    auto prev = ins.bits();
    Tether::Utils::printTimestamped(0.0, moduleStates(ins));

    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 20, [&](double t) {
            const auto now = ins.bits();
            if (now != prev) {
                prev = now;
                Tether::Utils::printTimestamped(t, moduleStates(ins));
            }
        });
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_input_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print input changes as plain lines instead of the TUI");

    if (!Tether::Examples::parseArgs(program, argc, argv)) return 1;

    if (program.get<bool>("--list-interfaces")) {
        EtherCAT::HAL::listPhysicalInterfaces(TAG);
        return 0;
    }

    std::string iface =
        EtherCAT::HAL::resolveInterface(program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (EtherCAT::debug::printFlagsHelp(debug_str)) return 0;
    auto debug_flags = EtherCAT::debug::parseFlags(debug_str);

    EtherCAT::EncapsulationConfig encapsulation;
    if (!EtherCAT::parseEncapsulationSpec(program.get<std::string>("--encapsulation"), encapsulation, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    // --stream wins when both are given; otherwise interactive is the
    // default and falls back to stream when the terminal can't do a TUI.
    const bool interactive =
        Tether::Examples::wantsInteractive(program, TAG);

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    // ---- Host Ethernet + master bring-up ----
    EtherCAT::HostEtherNetSession session;
    if (!EtherCAT::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
    EtherCAT::debug::applyFlags(debug_flags, master, TAG);

    if (!EtherCAT::setupEncapsulation(session, master, encapsulation, TAG)) {
        EtherCAT::shutdownHostEthernet(session);
        return 5;
    }
    EtherCAT::startHostPollThread(session, TAG);
    if (!EtherCAT::startHostMaster(session, master, TAG)) {
        EtherCAT::shutdownHostEthernet(session);
        return 5;
    }

    // ---- Discover the chain and pick out every EL1014 ----
    // A full discovery gives us the slave names for logging and lets the
    // driver reuse the SII data instead of re-reading each terminal's EEPROM.
    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
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
        EtherCAT::shutdownHostEthernet(session);
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
        EtherCAT::shutdownHostEthernet(session);
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
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
