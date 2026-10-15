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
        TUI::RowWriter rw(win, 1, 1);

        if (static_cast<uint16_t>(node.tag) == st.slaveIndex()) {
            rw.text("%s  —  slave %u", st.deviceName(), st.slaveIndex());
            rw.blank();
            rw.text("enable   : %s", g_enabled.load() ? "ON" : "off");
            rw.text("velocity : %d (dir %+d)",
                    g_speed.load(), g_direction.load());
            rw.text("position : %d", static_cast<int>(st.counterValue()));
            rw.text("latch    : %d", static_cast<int>(st.latchValue()));
            rw.blank();
            rw.text("STM status : 0x%04X", st.stmStatusWord());
            rw.text("ENC status : 0x%04X", st.encStatusWord());
            rw.blank();
            auto flag = [&](const char* label, bool v) {
                rw.styled(v ? TUI::pairAttrs(TUI::PalError) : A_DIM,
                          "%-12s %s", label, v ? "true" : "false");
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
            rw.styled(A_DIM, "(not a stepper  —  not managed by this demo)");
        }
    };

    auto managed = [&](uint16_t idx) { return idx == st.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff stepper jog  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::StepperTerminal& st, double duration_sec) {
    // In stream mode the demo just jogs back and forth automatically.
    int dir = 1;
    int phase = 0;
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 200, [&](double t) {
            if (++phase % 15 == 0) dir = -dir;   // flip every ~3 s
            st.setEnable(true);
            st.setVelocity(static_cast<int16_t>(dir * g_speed.load()));
            Tether::Utils::printTimestamped(t, statusLine(st));
        });
    st.setVelocity(0);
    st.setEnable(false);
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_stepper_jog", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print status as plain lines instead of the TUI");
    program.add_argument("--speed")
        .help("Jog speed setpoint (raw velocity units)")
        .default_value(400)
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
    g_speed.store(program.get<int>("--speed"));

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
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    auto& st = *st_opt;

    if (auto r = st.start(); !r) {
        TETHER_LOGE(TAG, "Stepper bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
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
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
