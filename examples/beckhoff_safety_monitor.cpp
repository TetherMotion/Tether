/**
 * @file beckhoff_safety_monitor.cpp
 * @brief Beckhoff TwinSAFE terminal monitor (EL19xx/EL29xx via FSoE)
 *
 * Finds the first known TwinSAFE terminal in the EtherCAT chain, brings
 * it to OP, pumps the FSoE master connection once per poll cycle, and
 * displays the safe input states / connection status.
 *
 * The FSoE slave address and connection parameters come from the
 * TwinSAFE project downloaded to the terminal  -  pass them via
 * --safety-addr (and --conn-id when it differs).
 *
 * Two display modes:
 *   - interactive TUI (default): device tree left, safe I/O right.
 *     o toggles safe output bit 0 on EL29xx outputs, r resets the
 *     connection, q quits.
 *   - --stream: one line per poll with connection state + safe inputs.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_safety_monitor --safety-addr 0x0001
 *   ./beckhoff_safety_monitor -i enp3s0 --safety-addr 1 --stream
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/SafetyTerminal.hpp"
// Complete type needed for the optional<unique_ptr<...>> move.
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/ethercat/Master.hpp"
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
// Widget headers that render via curses calls go after <ncurses.h>.
#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
#include "tether/terminal_ui/StyledText.hpp"
#include "tether/terminal_ui/Lamp.hpp"
#include "tether/terminal_ui/RowWriter.hpp"
#endif

static const char* TAG = "beckhoff_safety_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};
static std::atomic<bool> g_out0{false};
static std::atomic<bool> g_reset{false};

static std::string statusLine(const Beckhoff::SafetyTerminal& s,
                              bool fsoe_out) {
    std::string inputs;
    for (uint8_t b = 0; b < s.channelCount() && b < 16; ++b) {
        inputs += s.safeInputBit(b) ? '1' : '0';
        if (b % 4 == 3) inputs += ' ';
    }
    return std::format("fsoe={} state={} err=0x{:04X} out0={} in=[{}]",
                       fsoe_out ? "ok" : "--", s.fsoeState(),
                       s.fsoeErrorCode(), g_out0.load(), inputs);
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Beckhoff::SafetyTerminal& s,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;
    const auto t0 = std::chrono::steady_clock::now();

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "o out-bit0 · r conn-reset";
    hooks.onKey = [&](int key) {
        if (key == 'o' || key == 'O') g_out0.store(!g_out0.load());
        else if (key == 'r' || key == 'R') g_reset.store(true);
        else return false;
        return true;
    };
    hooks.onTick = [&] {
        if (g_reset.exchange(false)) s.resetConnection();
        s.setSafeOutputBit(0, g_out0.load());
        const auto ms = std::chrono::duration_cast<
            std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
        s.exchange(static_cast<uint64_t>(ms));
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);

        if (static_cast<uint16_t>(node.tag) == s.slaveIndex()) {
            rw.text("%s  —  slave %u (TwinSAFE)",
                    s.deviceName(), s.slaveIndex());
            rw.blank();
            const bool op = s.isOperational();
            const bool fs = s.isFailSafe();
            rw.styled(
                TUI::pairAttrs(op ? TUI::PalValue : TUI::PalError) | A_BOLD,
                "FSoE: %s%s  (state %u, err 0x%04X)",
                op ? "DATA" : "handshake",
                fs ? "  FAIL-SAFE" : "",
                s.fsoeState(), s.fsoeErrorCode());
            rw.blank();
            rw.text("safe inputs:");
            for (uint8_t b = 0; b < s.channelCount() && b < 16; ++b) {
                rw.putLamp(3 + b + b / 4,
                           s.safeInputBit(b) ? TUI::LampState::On
                                             : TUI::LampState::Off,
                           TUI::kLampDigits);
            }
            rw.advance(2);
            rw.text("safe output bit 0: %s", g_out0.load() ? "ON" : "off");
            return;
        }

        const auto* sd = slaveByIndex(slaves, node.tag);
        if (sd) {
            rw.text("%s", sd->device_name ? sd->device_name->c_str() : "?");
            rw.text("slave   : %u", sd->index);
            rw.text("vendor  : 0x%08X", sd->vendor_id ? *sd->vendor_id : 0);
            rw.text("product : 0x%08X",
                    sd->product_code ? *sd->product_code : 0);
        }
        if (!node.children.empty()) {
            rw.blank();
            rw.text("%zu terminal(s) below", node.children.size());
        } else if (rw.fits()) {
            rw.blank();
            rw.styled(A_DIM,
                      "(not a safety terminal  —  not managed by this demo)");
        }
    };

    auto managed = [&](uint16_t idx) { return idx == s.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff TwinSAFE monitor  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::SafetyTerminal& s, double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto ms = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
    };
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && ms() / 1000.0 >= duration_sec) break;
        const bool ok = s.exchange(static_cast<uint64_t>(ms()));
        std::cout << std::format("t={:7.3f}  {}\n", ms() / 1000.0,
                                 statusLine(s, ok));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(15);
    }
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_safety_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help("Print status as plain lines instead of the TUI")
        .flag();
    program.add_argument("--safety-addr")
        .help("FSoE slave (safety) address from the TwinSAFE project")
        .required()
        .scan<'i', int>();
    program.add_argument("--conn-id")
        .help("FSoE connection ID (default: derived from safety addr)")
        .default_value(0)
        .scan<'i', int>();

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

    Tether::Examples::EncapsulationConfig encapsulation;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encapsulation, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    Beckhoff::SafetyTerminal::Config scfg;
    scfg.safety_addr =
        static_cast<uint16_t>(program.get<int>("--safety-addr"));
    scfg.connection_id =
        static_cast<uint16_t>(program.get<int>("--conn-id"));

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

    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    // Try every known TwinSAFE identity.
    std::optional<Beckhoff::SafetyTerminal> s_opt;
    for (const auto& id : Beckhoff::Devices::kSafetyTerminals) {
        auto r = Beckhoff::SafetyTerminal::findFirst(master, id, slaves);
        if (r) { s_opt.emplace(std::move(*r)); break; }
    }
    if (!s_opt) {
        TETHER_LOGE(TAG, "No TwinSAFE terminal (EL19xx/EL29xx) "
                    "found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    auto& safe = *s_opt;

    if (auto r = safe.start(scfg); !r) {
        TETHER_LOGE(TAG, "Safety terminal bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }
    TETHER_LOGI(TAG, "{} on slave {}: {} safe channel(s), "
                "FSoE addr 0x{:04X}",
                safe.deviceName(), safe.slaveIndex(),
                safe.channelCount(), scfg.safety_addr);

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(safe, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(safe, duration_sec);
    }

    safe.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
