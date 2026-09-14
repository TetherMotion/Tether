/**
 * @file beckhoff_stepper_jog.cpp
 * @brief Beckhoff stepper terminal jog demo (EL7031/EL7037/EL7041 family)
 *
 * Finds the first known POS-interface stepper terminal in the EtherCAT
 * chain, brings it to OP, enables the output stage, and jogs the motor
 * back and forth by ramping the STM velocity setpoint.
 *
 * Two display modes:
 *   - interactive TUI (default): device tree left, live status right.
 *     Space toggles direction, up/down adjusts the speed setpoint,
 *     e toggles the enable bit, q quits.
 *   - --stream: one line per poll with status + position.
 *
 * WARNING: this drives a real motor output stage.  Run it only on a
 * bench setup with a safely mounted (or disconnected) motor.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_stepper_jog                  # TUI, auto-detected NIC
 *   ./beckhoff_stepper_jog -i enp3s0        # specify interface
 *   ./beckhoff_stepper_jog --stream         # line mode for pipes
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/StepperTerminal.hpp"
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

static const char* TAG = "beckhoff_stepper_jog";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};
static std::atomic<int>  g_direction{1};
static std::atomic<int>  g_speed{400};      // velocity setpoint (raw)
static std::atomic<bool> g_enabled{false};

static std::string statusLine(const Beckhoff::StepperTerminal& st) {
    return std::format(
        "en={} rdy={} rdyEn={} warn={} err={} pos={} latched={} "
        "stm=0x{:04X} dir={} vel={}",
        g_enabled.load(), st.ready(), st.readyToEnable(),
        st.warning(), st.error(),
        static_cast<int>(st.counterValue()),
        st.setCounterDone(),
        st.stmStatusWord(), g_direction.load(), g_speed.load());
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Beckhoff::StepperTerminal& st,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "space dir · up/down speed · e enable";
    hooks.onKey = [&](int key) {
        if (key == ' ') g_direction.store(-g_direction.load());
        else if (key == KEY_UP)   g_speed.store(std::min(g_speed.load() + 100, 30000));
        else if (key == KEY_DOWN) g_speed.store(std::max(g_speed.load() - 100, 0));
        else if (key == 'e' || key == 'E') g_enabled.store(!g_enabled.load());
        else return false;
        return true;
    };
    hooks.onTick = [&] {
        st.setEnable(g_enabled.load());
        st.setVelocity(static_cast<int16_t>(
            g_direction.load() * g_speed.load()));
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        if (static_cast<uint16_t>(node.tag) == st.slaveIndex()) {
            mvwprintw(win, row++, 1, "%s  -  slave %u",
                      st.deviceName(), st.slaveIndex());
            ++row;
            mvwprintw(win, row++, 1, "enable   : %s",
                      g_enabled.load() ? "ON" : "off");
            mvwprintw(win, row++, 1, "velocity : %d (dir %+d)",
                      g_speed.load(), g_direction.load());
            mvwprintw(win, row++, 1, "position : %d",
                      static_cast<int>(st.counterValue()));
            mvwprintw(win, row++, 1, "latch    : %d",
                      static_cast<int>(st.latchValue()));
            ++row;
            mvwprintw(win, row++, 1, "STM status : 0x%04X",
                      st.stmStatusWord());
            mvwprintw(win, row++, 1, "ENC status : 0x%04X",
                      st.encStatusWord());
            ++row;
            auto flag = [&](const char* label, bool v) {
                wattron(win, v ? COLOR_PAIR(TUI::PalError) : A_DIM);
                mvwprintw(win, row++, 1, "%-12s %s", label,
                          v ? "true" : "false");
                wattroff(win, v ? COLOR_PAIR(TUI::PalError) : A_DIM);
            };
            flag("ready",        st.ready());
            flag("ready-enable", st.readyToEnable());
            flag("warning",      st.warning());
            flag("error",        st.error());
            flag("moving +",     st.movingPositive());
            flag("moving -",     st.movingNegative());
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
                      "(not a stepper  -  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    auto managed = [&](uint16_t idx) { return idx == st.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff stepper jog  -  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::StepperTerminal& st, double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };
    // In stream mode the demo just jogs back and forth automatically.
    int dir = 1;
    int phase = 0;
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        if (++phase % 15 == 0) dir = -dir;   // flip every ~3 s
        st.setEnable(true);
        st.setVelocity(static_cast<int16_t>(dir * g_speed.load()));
        std::cout << std::format("t={:7.3f}  {}\n", stamp(), statusLine(st));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(200);
    }
    st.setVelocity(0);
    st.setEnable(false);
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_stepper_jog", "1.0",
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
    program.add_argument("--speed")
        .help("Jog speed setpoint (raw velocity units)")
        .default_value(400)
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
    g_speed.store(program.get<int>("--speed"));

    bool interactive = !program.get<bool>("--stream");
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal  -  falling back to --stream");
        }
        interactive = false;
    }
#else
    if (interactive) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "built without ncurses  -  using --stream mode");
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

    // Try every known POS-interface stepper identity.
    std::optional<Beckhoff::StepperTerminal> st_opt;
    for (const auto& id : Beckhoff::Devices::kStepperTerminals) {
        auto r = Beckhoff::StepperTerminal::findFirst(master, id, slaves);
        if (r) { st_opt.emplace(std::move(*r)); break; }
    }
    if (!st_opt) {
        TETHER_LOGE(TAG, "No POS-interface stepper terminal "
                    "(EL703x/EL704x) found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    auto& st = *st_opt;

    if (auto r = st.start(); !r) {
        TETHER_LOGE(TAG, "Stepper bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        // TUI drives enable/velocity through the shared atomics each tick.
        runTui(st, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(st, duration_sec);
    }

    st.setVelocity(0);
    st.setEnable(false);
    st.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
