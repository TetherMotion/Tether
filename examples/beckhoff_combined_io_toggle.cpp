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
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
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
        int row = 1;

        for (size_t m = 0; m < io.moduleCount(); ++m) {
            if (io.slaveIndex(m) != static_cast<uint16_t>(node.tag))
                continue;
            const auto& mod = io.module(m);
            mvwprintw(win, row++, 1, "%s  —  slave %d", mod.deviceName(),
                      node.tag);
            ++row;
            mvwprintw(win, row++, 1, "inputs  :");
            for (size_t k = 0; k < mod.inputChannelCount(); ++k) {
                const bool on = mod.input(k);
                wattron(win, on ? COLOR_PAIR(TUI::PalValue) | A_BOLD
                                : A_DIM);
                mvwprintw(win, row - 1, 12 + k * 2, "%s", on ? "*" : ".");
                wattroff(win, COLOR_PAIR(TUI::PalValue) | A_BOLD | A_DIM);
            }
            mvwprintw(win, row++, 1, "outputs :");
            const size_t base = io.outputOffset(m);
            for (size_t k = 0; k < mod.outputChannelCount(); ++k) {
                const bool on = mod.output(k);
                const bool act = on && (base + k == active);
                const int attrs = act
                    ? (COLOR_PAIR(TUI::PalSelected) | A_BOLD)
                    : (on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
                wattron(win, attrs);
                mvwprintw(win, row - 1, 12 + k * 2, "%s", on ? "*" : ".");
                wattroff(win, attrs);
            }
            return;
        }

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
        ++row;
        wattron(win, A_DIM);
        mvwprintw(win, row++, 1,
                  "(not a combined I/O device  —  not managed by this demo)");
        wattroff(win, A_DIM);
    };

    TUI::TreeScreen screen(
        std::string("Beckhoff combined I/O  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Chain& io, double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    size_t active = 0;
    const size_t n_out = io.outputChannelCount();
    if (n_out) io.setOutput(active, true);
    std::cout << std::format("t={:7.3f} out {:>3}{}\n",
                             stamp(), active, moduleStates(io));
    std::cout.flush();

    auto next_step = std::chrono::steady_clock::now();
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_step) {
            next_step = now + std::chrono::milliseconds(500);
            if (n_out) {
                active = (active + 1) % n_out;
                io.allOutputsOff();
                io.setOutput(active, true);
            }
            std::cout << std::format("t={:7.3f} out {:>3}{}\n",
                                     stamp(), active, moduleStates(io));
            std::cout.flush();
        }
        Tether::Platform::Clock::instance().delayMilliseconds(20);
    }
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_combined_io_toggle", "1.0",
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
        .help("Print state lines instead of the TUI")
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

    std::string iface = Tether::Examples::resolveInterface(
        program.get<std::string>("--interface"), TAG);
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

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) return 2;

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

    Chain io(master);
    auto found = io.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No combined I/O terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
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
        Tether::Examples::shutdownHostEthernet(session);
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
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. All outputs off.");
    return 0;
}
