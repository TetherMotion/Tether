/**
 * @file beckhoff_pulse_jog.cpp
 * @brief Beckhoff pulse-train terminal jog demo (EL2521/EL2522 family)
 *
 * Finds the first PTO terminal in the EtherCAT chain, brings it to OP,
 * and jogs the pulse output back and forth by setting the frequency
 * setpoint and direction bits — the classic step/dir interface for
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
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
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
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        if (static_cast<uint16_t>(node.tag) == p.slaveIndex()) {
            mvwprintw(win, row++, 1, "%s — slave %u, %zu ch",
                      p.deviceName(), p.slaveIndex(), p.channelCount());
            mvwprintw(win, row++, 1, "go=%s  freq=%d  dir=%+d",
                      g_go.load() ? "ON" : "off",
                      g_freq.load(), g_direction.load());
            ++row;
            for (size_t ch = 0; ch < p.channelCount() && row < h - 1;
                 ++ch) {
                mvwprintw(win, row++, 1, "ch%zu", ch + 1);
                if (p.hasEnc(ch)) {
                    mvwprintw(win, row++, 3, "counter : %d",
                              p.counterValue(ch));
                    mvwprintw(win, row++, 3, "latch   : %d",
                              p.latchValue(ch));
                }
                auto flag = [&](const char* label, bool v) {
                    wattron(win, v ? COLOR_PAIR(TUI::PalError) : A_DIM);
                    mvwprintw(win, row++, 3, "%-9s %s", label,
                              v ? "true" : "false");
                    wattroff(win, COLOR_PAIR(TUI::PalError) | A_DIM);
                };
                flag("ramp",     p.rampActive(ch));
                flag("error",    p.error(ch));
                flag("underfl.", p.counterUnderflow(ch));
                flag("overfl.",  p.counterOverflow(ch));
                ++row;
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
        if (!node.children.empty()) {
            ++row;
            mvwprintw(win, row++, 1, "%zu terminal(s) below",
                      node.children.size());
        } else if (row < h) {
            ++row;
            wattron(win, A_DIM);
            mvwprintw(win, row++, 1,
                      "(not a pulse-train terminal — not managed here)");
            wattroff(win, A_DIM);
        }
    };

    auto managed = [&](uint16_t idx) { return idx == p.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff pulse-train jog — ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::PulseTrainTerminal& p,
                    double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };
    int dir = 1;
    int phase = 0;
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        if (++phase % 15 == 0) dir = -dir;
        g_direction.store(dir);
        g_go.store(true);
        tickPto(p);
        std::cout << std::format("t={:7.3f}  {}\n", stamp(),
                                 statusLine(p));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(200);
    }
    g_go.store(false);
    tickPto(p);
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_pulse_jog", "1.0",
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
        .help("Print status as plain lines instead of the TUI")
        .flag();
    program.add_argument("--freq")
        .help("Pulse frequency setpoint (pulses/s)")
        .default_value(5000)
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

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"), vlan, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");
    g_freq.store(program.get<int>("--freq"));

    bool interactive = !program.get<bool>("--stream");
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal — falling back to --stream");
        }
        interactive = false;
    }
#else
    if (interactive) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "built without ncurses — using --stream mode");
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
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    auto& p = *p_opt;

    if (auto r = p.start(); !r) {
        TETHER_LOGE(TAG, "Pulse-train bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
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
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
