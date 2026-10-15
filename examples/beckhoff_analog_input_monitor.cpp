/**
 * @file beckhoff_analog_input_monitor.cpp
 * @brief Beckhoff analog-input terminal monitor (EL3xxx family)
 *
 * Finds every known analog-input terminal (EL30xx–EL37xx) in the EtherCAT
 * chain via MultiAnalogInputTerminal, brings all of them to OP, and
 * displays the live value (+ raw status word) of every channel.  Module 0
 * occupies the first channels of the flat index space, module 1 the next
 * ones, and so on.
 *
 * Two display modes:
 *   - interactive TUI (default on a terminal): a navigable device tree  - 
 *     level 1 = coupler(s) (EK1100 ...), level 2 = the terminals below
 *     each coupler  -  with the selected node's live channels in the right
 *     pane.  Arrows navigate, left/right fold, q quits.
 *   - --stream: plain stdout  -  one line per poll with all raw values.
 *     Selected automatically when ncurses/the terminal can't do a TUI
 *     (non-TTY output, TERM=dumb, or a curses-less build), or when the
 *     user passes --stream.  --interactive forces an interactive attempt.
 *
 * Values are raw field-bus integers  -  for a ±10 V terminal 32767 ≈ +10 V;
 * RTD/TC terminals report scaled temperature (typically 0.1 °C/digit).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_analog_input_monitor                # TUI, auto-detected NIC
 *   ./beckhoff_analog_input_monitor -i enp3s0      # specify interface
 *   ./beckhoff_analog_input_monitor --stream       # line mode for pipes
 *   ./beckhoff_analog_input_monitor -t 30          # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/MultiAnalogInputTerminal.hpp"
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

static const char* TAG = "beckhoff_analog_input_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;
namespace Platform = Tether::Platform;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Compact per-module value rendering: "s1:+00123/-1024 s2:...".
static std::string moduleStates(
    const Beckhoff::MultiAnalogInputTerminal<>& ins) {
    std::string out;
    for (size_t m = 0; m < ins.moduleCount(); ++m) {
        if (m) out += " ";
        out += "s" + std::to_string(ins.slaveIndex(m)) + ":";
        const auto& mod = ins.module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (k) out += ",";
            out += std::format("{:+d}", mod.value(k));
        }
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
// ---------------------------------------------------------------------------
// Interactive TUI  -  device tree left, selected node's live channels right
// ---------------------------------------------------------------------------

namespace TUI = Tether::TUI;

static void runTui(Beckhoff::MultiAnalogInputTerminal<>& ins,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < ins.moduleCount(); ++m)
            if (ins.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "";
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);

        // Managed input terminal → live channel values + status words.
        for (size_t m = 0; m < ins.moduleCount(); ++m) {
            if (ins.slaveIndex(m) != static_cast<uint16_t>(node.tag)) continue;
            const auto& mod = ins.module(m);
            rw.text("%s  —  slave %d", mod.deviceName(), node.tag);
            rw.blank();
            rw.styled(A_BOLD, "ch   value        status");
            for (size_t k = 0; k < mod.channelCount() && rw.remaining() > 1;
                 ++k) {
                const int32_t v = mod.value(k);
                rw.put(4, TUI::pairAttrs(TUI::PalValue) | A_BOLD, "%+d", v);
                rw.put(1, 0, "%zu", k + 1);
                if (mod.hasStatus(k)) {
                    const uint16_t st = mod.status(k);
                    rw.put(16, st ? TUI::pairAttrs(TUI::PalError) : A_DIM,
                           "0x%04X", st);
                } else {
                    rw.put(16, A_DIM, "-");
                }
                rw.advance();
            }
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
            rw.styled(A_DIM,
                      "(not an analog input  —  not managed by this demo)");
        }
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff analog inputs  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ---------------------------------------------------------------------------
// Stream mode  -  one line per poll (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiAnalogInputTerminal<>& ins,
                      double duration_sec) {
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 200, [&](double t) {
            Tether::Utils::printTimestamped(t, moduleStates(ins));
        });
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_analog_input_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print channel values as plain lines instead of the TUI");

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

    // ---- Discover the chain and pick out every analog-input terminal ----
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

    Beckhoff::MultiAnalogInputTerminal<> ins(master);
    auto found = ins.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No analog-input terminal found in the chain");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} analog input terminal(s), {} channel(s) total",
                ins.moduleCount(), ins.channelCount());

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = ins.start(); !r) {
        TETHER_LOGE(TAG, "Analog input bring-up failed on module {}: {}",
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
