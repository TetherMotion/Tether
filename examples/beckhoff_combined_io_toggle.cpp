/**
 * @file beckhoff_combined_io_toggle.cpp
 * @brief Beckhoff combined I/O terminal demo  -  mixed input/output devices
 *
 * Finds every combined packed-bit device in the chain via the
 * MultiCombinedIoTerminal driver  -  mixed digital I/O (EL1852/EL1859,
 * EP23xx), output terminals with diagnostics (EL2032/34/44/68, EL2212,
 * EL2819), relays with counters (ELM2642/2742), LED drivers
 * (EL2574/2595/2596), power-distribution terminals (EL922x, EP9214/
 * EP9224), and couplers with integrated I/O (EK1814/1818/1828)  -  brings
 * them to OP in one shared logical address space (a single LRW datagram
 * per cycle for the whole chain), then walks one lit output across the
 * flat output channel space while showing live inputs.
 *
 * Detected set: Devices::kCombinedIoTerminals (all ESI-verified for
 * shape, none verified on hardware yet).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_combined_io_toggle                # TUI
 *   ./beckhoff_combined_io_toggle -i enp3s0      # specify interface
 *   ./beckhoff_combined_io_toggle --stream       # line mode
 *   ./beckhoff_combined_io_toggle -t 30          # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/MultiCombinedIoTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

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

static const char* TAG = "beckhoff_combined_io_toggle";

namespace Beckhoff = EtherCAT::Beckhoff;
using Chain = Beckhoff::MultiCombinedIoTerminal<>;

static std::atomic<bool> g_cancel{false};

/// Compact per-module rendering: "s3 in:01001100 out:0001".
static std::string moduleStates(const Chain& io) {
    std::string out;
    for (size_t m = 0; m < io.moduleCount(); ++m) {
        const auto& mod = io.module(m);
        out += " s" + std::to_string(io.slaveIndex(m)) + " in:";
        for (size_t k = 0; k < mod.inputChannelCount(); ++k)
            out += mod.input(k) ? '1' : '0';
        if (mod.outputChannelCount()) {
            out += " out:";
            for (size_t k = 0; k < mod.outputChannelCount(); ++k)
                out += mod.output(k) ? '1' : '0';
        }
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Chain& io,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    size_t active = 0;
    auto relight = [&] {
        io.allOutputsOff();
        if (io.outputChannelCount()) io.setOutput(active, true);
    };
    relight();

    auto managed = [&](uint16_t idx) {
        for (size_t m = 0; m < io.moduleCount(); ++m)
            if (io.slaveIndex(m) == idx) return true;
        return false;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "enter: next output";
    hooks.onKey = [&](int key) {
        if (key != '\n' && key != '\r' && key != KEY_ENTER) return false;
        if (!io.outputChannelCount()) return true;
        active = (active + 1) % io.outputChannelCount();
        relight();
        return true;
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);

        for (size_t m = 0; m < io.moduleCount(); ++m) {
            if (io.slaveIndex(m) != static_cast<uint16_t>(node.tag))
                continue;
            const auto& mod = io.module(m);
            rw.text("%s  —  slave %d", mod.deviceName(), node.tag);
            rw.blank();
            rw.put(1, 0, "inputs  :");
            for (size_t k = 0; k < mod.inputChannelCount(); ++k) {
                rw.putLamp(12 + k * 2,
                           mod.input(k) ? TUI::LampState::On
                                        : TUI::LampState::Off,
                           TUI::kLampBits);
            }
            rw.advance();
            rw.put(1, 0, "outputs :");
            const size_t base = io.outputOffset(m);
            for (size_t k = 0; k < mod.outputChannelCount(); ++k) {
                const bool on = mod.output(k);
                const bool act = on && (base + k == active);
                rw.put(12 + k * 2,
                       act ? TUI::pairAttrs(TUI::PalSelected) | A_BOLD
                           : (on ? TUI::pairAttrs(TUI::PalValue) | A_BOLD
                                 : A_DIM),
                       "%s", on ? "*" : ".");
            }
            rw.advance();
            return;
        }

        const auto* s = slaveByIndex(slaves, node.tag);
        if (s) {
            rw.text("%s", s->device_name ? s->device_name->c_str() : "?");
            rw.text("slave   : %u", s->index);
            rw.text("vendor  : 0x%08X", s->vendor_id ? *s->vendor_id : 0);
            rw.text("product : 0x%08X",
                    s->product_code ? *s->product_code : 0);
        }
        rw.blank();
        rw.styled(A_DIM,
                  "(not a combined I/O device  —  not managed by this demo)");
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff combined I/O  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Chain& io, double duration_sec) {
    size_t active = 0;
    const size_t n_out = io.outputChannelCount();
    if (n_out) io.setOutput(active, true);
    Tether::Utils::printTimestamped(
        0.0, std::format("out {:>3}{}", active, moduleStates(io)));

    auto next_step = std::chrono::steady_clock::now();
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 20, [&](double t) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_step) {
                next_step = now + std::chrono::milliseconds(500);
                if (n_out) {
                    active = (active + 1) % n_out;
                    io.allOutputsOff();
                    io.setOutput(active, true);
                }
                Tether::Utils::printTimestamped(
                    t, std::format("out {:>3}{}", active,
                                   moduleStates(io)));
            }
        });
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_combined_io_toggle", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print state lines instead of the TUI");

    if (!Tether::Examples::parseArgs(program, argc, argv)) return 1;

    if (program.get<bool>("--list-interfaces")) {
        EtherCAT::HAL::listPhysicalInterfaces(TAG);
        return 0;
    }

    std::string iface = EtherCAT::HAL::resolveInterface(
        program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (EtherCAT::debug::printFlagsHelp(debug_str)) return 0;
    auto debug_flags = EtherCAT::debug::parseFlags(debug_str);

    EtherCAT::EncapsulationConfig encapsulation;
    if (!EtherCAT::parseEncapsulationSpec(program.get<std::string>("--encapsulation"), encapsulation, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    const bool interactive =
        Tether::Examples::wantsInteractive(program, TAG);

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    EtherCAT::HostEtherNetSession session;
    if (!EtherCAT::initHostEthernet(session, iface, TAG)) return 2;

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

    Chain io(master);
    auto found = io.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No combined I/O terminal found in the chain");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} combined I/O device(s), {} in / {} out bits",
                io.moduleCount(), io.inputChannelCount(),
                io.outputChannelCount());
    for (size_t m = 0; m < io.moduleCount(); ++m) {
        TETHER_LOGI(TAG, "  module {}: slave {} = {} ({} in / {} out)",
                    m, io.slaveIndex(m), io.module(m).deviceName(),
                    io.module(m).inputChannelCount(),
                    io.module(m).outputChannelCount());
    }

    if (auto r = io.start(); !r) {
        TETHER_LOGE(TAG, "bring-up failed on module {}: {}",
                    io.lastErrorModule(),
                    Beckhoff::errorToString(r.error()));
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 7;
    }

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(io, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(io, duration_sec);
    }

    io.allOutputsOff();
    io.stop();
    master.stop();
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. All outputs off.");
    return 0;
}
