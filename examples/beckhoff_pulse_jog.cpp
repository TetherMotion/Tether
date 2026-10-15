/**
 * @file beckhoff_pulse_jog.cpp
 * @brief Beckhoff pulse-train terminal jog demo (EL2521/EL2522 family)
 *
 * Finds the first PTO terminal in the EtherCAT chain, brings it to OP,
 * and jogs the pulse output back and forth by setting the frequency
 * setpoint and direction bits  -  the classic step/dir interface for
 * external stepper/servo drives.
 *
 * Two display modes:
 *   - interactive TUI (default): device tree left, live status right.
 *     Space toggles direction, up/down adjusts frequency, g toggles
 *     the go-counter output, q quits.
 *   - --stream: one line per poll with frequency + counter values.
 *
 * WARNING: the pulse output physically moves whatever is wired to it.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_pulse_jog                  # TUI, auto-detected NIC
 *   ./beckhoff_pulse_jog -i enp3s0        # specify interface
 *   ./beckhoff_pulse_jog --stream         # line mode for pipes
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/PulseTrainTerminal.hpp"
#include "tether/ethercat/Master.hpp"
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

static const char* TAG = "beckhoff_pulse_jog";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};
static std::atomic<int>  g_direction{1};
static std::atomic<int>  g_freq{5000};      // pulses/s
static std::atomic<bool> g_go{false};

/// Push the shared setpoints into every PTO channel.
static void tickPto(Beckhoff::PulseTrainTerminal& p) {
    for (size_t ch = 0; ch < p.channelCount(); ++ch) {
        if (!p.hasFrequency(ch)) continue;
        p.setFrequency(ch, static_cast<uint16_t>(g_freq.load()));
        if (g_go.load()) {
            p.setGoCounter(ch, true);
            if (g_direction.load() >= 0) {
                p.setForward(ch, true);
                p.setReverse(ch, false);
            } else {
                p.setForward(ch, false);
                p.setReverse(ch, true);
            }
        } else {
            p.setGoCounter(ch, false);
            p.setForward(ch, false);
            p.setReverse(ch, false);
        }
    }
}

static std::string statusLine(const Beckhoff::PulseTrainTerminal& p) {
    std::string out;
    for (size_t ch = 0; ch < p.channelCount(); ++ch) {
        if (ch) out += " ";
        out += std::format("ch{}:f={} cnt={} ramp={} err={}",
            ch + 1,
            p.hasFrequency(ch) ? p.frequency(ch) : 0,
            p.hasEnc(ch) ? p.counterValue(ch) : 0,
            p.rampActive(ch), p.error(ch));
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Beckhoff::PulseTrainTerminal& p,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "space dir · up/down freq · g go";
    hooks.onKey = [&](int key) {
        if (key == ' ') g_direction.store(-g_direction.load());
        else if (key == KEY_UP)
            g_freq.store(std::min(g_freq.load() + 1000, 500000));
        else if (key == KEY_DOWN)
            g_freq.store(std::max(g_freq.load() - 1000, 0));
        else if (key == 'g' || key == 'G') g_go.store(!g_go.load());
        else return false;
        return true;
    };
    hooks.onTick = [&] { tickPto(p); };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);

        if (static_cast<uint16_t>(node.tag) == p.slaveIndex()) {
            rw.text("%s  —  slave %u, %zu ch",
                    p.deviceName(), p.slaveIndex(), p.channelCount());
            rw.text("go=%s  freq=%d  dir=%+d",
                    g_go.load() ? "ON" : "off",
                    g_freq.load(), g_direction.load());
            rw.blank();
            for (size_t ch = 0; ch < p.channelCount() && rw.remaining() > 1;
                 ++ch) {
                rw.text("ch%zu", ch + 1);
                rw.margin(3);
                if (p.hasEnc(ch)) {
                    rw.text("counter : %d", p.counterValue(ch));
                    rw.text("latch   : %d", p.latchValue(ch));
                }
                auto flag = [&](const char* label, bool v) {
                    rw.styled(v ? TUI::pairAttrs(TUI::PalError) : A_DIM,
                              "%-9s %s", label, v ? "true" : "false");
                };
                flag("ramp",     p.rampActive(ch));
                flag("error",    p.error(ch));
                flag("underfl.", p.counterUnderflow(ch));
                flag("overfl.",  p.counterOverflow(ch));
                rw.margin(1).blank();
            }
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
        if (!node.children.empty()) {
            rw.blank();
            rw.text("%zu terminal(s) below", node.children.size());
        } else if (rw.fits()) {
            rw.blank();
            rw.styled(A_DIM,
                      "(not a pulse-train terminal  —  not managed here)");
        }
    };

    auto managed = [&](uint16_t idx) { return idx == p.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff pulse-train jog  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::PulseTrainTerminal& p,
                    double duration_sec) {
    int dir = 1;
    int phase = 0;
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 200, [&](double t) {
            if (++phase % 15 == 0) dir = -dir;
            g_direction.store(dir);
            g_go.store(true);
            tickPto(p);
            Tether::Utils::printTimestamped(t, statusLine(p));
        });
    g_go.store(false);
    tickPto(p);
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_pulse_jog", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print status as plain lines instead of the TUI");
    program.add_argument("--freq")
        .help("Pulse frequency setpoint (pulses/s)")
        .default_value(5000)
        .scan<'i', int>();

    if (!Tether::Examples::parseArgs(program, argc, argv)) return 1;

    if (program.get<bool>("--list-interfaces")) {
        EtherCAT::HAL::listPhysicalInterfaces(TAG);
        return 0;
    }
    std::string iface =
        EtherCAT::HAL::resolveInterface(
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
    g_freq.store(program.get<int>("--freq"));

    const bool interactive =
        Tether::Examples::wantsInteractive(program, TAG);

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

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

    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 4;
    }

    // Try every known pulse-train identity.
    std::optional<Beckhoff::PulseTrainTerminal> p_opt;
    for (const auto& id : Beckhoff::Devices::kPulseTrainTerminals) {
        auto r = Beckhoff::PulseTrainTerminal::findFirst(
            master, id, slaves);
        if (r) { p_opt.emplace(std::move(*r)); break; }
    }
    if (!p_opt) {
        TETHER_LOGE(TAG, "No pulse-train terminal (EL2521/EL2522) "
                    "found in the chain");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    auto& p = *p_opt;

    if (auto r = p.start(); !r) {
        TETHER_LOGE(TAG, "Pulse-train bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 7;
    }
    TETHER_LOGI(TAG, "{} on slave {}: {} channel(s)",
                p.deviceName(), p.slaveIndex(), p.channelCount());

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(p, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(p, duration_sec);
    }

    g_go.store(false);
    tickPto(p);
    p.stop();
    master.stop();
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
