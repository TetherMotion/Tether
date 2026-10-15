/**
 * @file beckhoff_drive_move.cpp
 * @brief Beckhoff compact drive jog demo (EL7062/EL7411/EL72xx/ELM72xx)
 *
 * Finds the first DRV-interface compact drive in the EtherCAT chain,
 * brings it to OP, walks the CiA402 state machine to OperationEnabled,
 * and jogs the axis back and forth in profile-velocity mode.
 *
 * Two display modes:
 *   - interactive TUI (default): device tree left, per-axis state right.
 *     Space toggles direction, up/down adjusts the speed setpoint,
 *     e toggles the enable sequence, r issues a fault reset, q quits.
 *   - --stream: one line per poll with statusword, state and feedback.
 *
 * WARNING: this drives a real servo/stepper output stage.  Run it only
 * on a bench setup with a safely mounted (or disconnected) motor.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_drive_move                  # TUI, auto-detected NIC
 *   ./beckhoff_drive_move -i enp3s0        # specify interface
 *   ./beckhoff_drive_move --stream         # line mode for pipes
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/CompactDriveTerminal.hpp"
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

static const char* TAG = "beckhoff_drive_move";

namespace Beckhoff = EtherCAT::Beckhoff;
using Beckhoff::DriveState;

static std::atomic<bool> g_cancel{false};
static std::atomic<int>  g_direction{1};
static std::atomic<int>  g_speed{2000};
static std::atomic<bool> g_enabled{false};
static std::atomic<bool> g_fault_reset{false};

static const char* stateName(DriveState s) {
    switch (s) {
        case DriveState::NotReadyToSwitchOn: return "NotReadyToSwitchOn";
        case DriveState::SwitchOnDisabled:   return "SwitchOnDisabled";
        case DriveState::ReadyToSwitchOn:    return "ReadyToSwitchOn";
        case DriveState::SwitchedOn:         return "SwitchedOn";
        case DriveState::OperationEnabled:   return "OperationEnabled";
        case DriveState::QuickStopActive:    return "QuickStopActive";
        case DriveState::FaultReactionActive:return "FaultReactionActive";
        case DriveState::Fault:              return "Fault";
        default:                             return "Unknown";
    }
}

/// Step the CiA402 FSA one command per cycle toward OperationEnabled.
/// The slave must observe each controlword before the next is issued.
static void tickAxis(Beckhoff::CompactDriveTerminal& d, size_t ax) {
    const auto st = d.driveState(ax);

    if (g_fault_reset.exchange(false) && st == DriveState::Fault) {
        d.requestFaultReset(ax);
        return;
    }
    if (st == DriveState::Fault) {
        d.clearFaultReset(ax);
        return;
    }

    if (!g_enabled.load()) {
        d.requestShutdown(ax);            // drive back to ReadyToSwitchOn
        return;
    }

    // Modes of operation: profile velocity (3) when the field exists.
    if (d.hasModesOfOperation(ax) &&
        d.modesOfOperationDisplay(ax) != 3 &&
        st == DriveState::OperationEnabled) {
        d.setModesOfOperation(ax, 3);
    }

    switch (st) {
        case DriveState::SwitchOnDisabled:
        case DriveState::ReadyToSwitchOn:
            d.requestSwitchOn(ax);
            break;
        case DriveState::SwitchedOn:
        case DriveState::QuickStopActive:
            d.requestEnableOperation(ax);
            break;
        case DriveState::OperationEnabled:
            d.setTargetVelocity(ax, static_cast<int32_t>(
                g_direction.load() * g_speed.load()));
            break;
        default:
            d.requestShutdown(ax);
            break;
    }
}

static std::string statusLine(const Beckhoff::CompactDriveTerminal& d) {
    std::string out;
    for (size_t a = 0; a < d.axisCount(); ++a) {
        if (a) out += "  ";
        out += std::format("ax{}: sw=0x{:04X} {} pos={} vel={} fterr={}",
            a, d.statusword(a), stateName(d.driveState(a)),
            d.hasActualPosition(a) ? d.actualPosition(a) : 0,
            d.hasActualVelocity(a) ? d.actualVelocity(a) : 0,
            d.hasFollowingError(a) ? d.followingError(a) : 0);
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Beckhoff::CompactDriveTerminal& d,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "space dir · up/down speed · e enable · r reset";
    hooks.onKey = [&](int key) {
        if (key == ' ') g_direction.store(-g_direction.load());
        else if (key == KEY_UP)
            g_speed.store(std::min(g_speed.load() + 500, 200000));
        else if (key == KEY_DOWN)
            g_speed.store(std::max(g_speed.load() - 500, 0));
        else if (key == 'e' || key == 'E')
            g_enabled.store(!g_enabled.load());
        else if (key == 'r' || key == 'R')
            g_fault_reset.store(true);
        else return false;
        return true;
    };
    hooks.onTick = [&] {
        for (size_t a = 0; a < d.axisCount(); ++a) tickAxis(d, a);
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);

        if (static_cast<uint16_t>(node.tag) == d.slaveIndex()) {
            rw.text("%s  —  slave %u, %zu axis",
                    d.deviceName(), d.slaveIndex(), d.axisCount());
            rw.blank();
            rw.text("enable=%s  speed=%d  dir=%+d",
                    g_enabled.load() ? "ON" : "off",
                    g_speed.load(), g_direction.load());
            rw.blank();
            for (size_t a = 0; a < d.axisCount() && rw.remaining() > 8; ++a) {
                rw.text("axis %zu  sw=0x%04X", a, d.statusword(a));
                rw.margin(3);
                const auto st = d.driveState(a);
                rw.styled(
                    (st == DriveState::OperationEnabled)
                        ? TUI::pairAttrs(TUI::PalValue) | A_BOLD
                        : (st == DriveState::Fault
                               ? TUI::pairAttrs(TUI::PalError) | A_BOLD
                               : A_NORMAL),
                    "%s", stateName(st));
                if (d.hasActualPosition(a))
                    rw.text("position   : %d", d.actualPosition(a));
                if (d.hasActualVelocity(a))
                    rw.text("velocity   : %d", d.actualVelocity(a));
                if (d.hasActualTorque(a))
                    rw.text("torque     : %d", d.actualTorque(a));
                if (d.hasFollowingError(a))
                    rw.text("following  : %d", d.followingError(a));
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
                      "(not a compact drive  —  not managed by this demo)");
        }
    };

    auto managed = [&](uint16_t idx) { return idx == d.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff compact drive  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::CompactDriveTerminal& d,
                    double duration_sec) {
    g_enabled.store(true);
    int dir = 1;
    int phase = 0;
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 200, [&](double t) {
            if (++phase % 15 == 0) dir = -dir;
            g_direction.store(dir);
            for (size_t a = 0; a < d.axisCount(); ++a) tickAxis(d, a);
            Tether::Utils::printTimestamped(t, statusLine(d));
        });
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_drive_move", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print status as plain lines instead of the TUI");
    program.add_argument("--speed")
        .help("Jog speed setpoint (raw velocity units)")
        .default_value(2000)
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

    // Try every known DRV-interface compact-drive identity.
    std::optional<Beckhoff::CompactDriveTerminal> d_opt;
    for (const auto& id : Beckhoff::Devices::kCompactDriveTerminals) {
        auto r = Beckhoff::CompactDriveTerminal::findFirst(
            master, id, slaves);
        if (r) { d_opt.emplace(std::move(*r)); break; }
    }
    if (!d_opt) {
        TETHER_LOGE(TAG, "No DRV-interface compact drive "
                    "(EL7062/EL7411/EL72xx/ELM72xx) found in the chain");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    auto& d = *d_opt;

    if (auto r = d.start(); !r) {
        TETHER_LOGE(TAG, "Drive bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 7;
    }
    TETHER_LOGI(TAG, "{} on slave {}: {} axis",
                d.deviceName(), d.slaveIndex(), d.axisCount());

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(d, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(d, duration_sec);
    }

    // Coast down before leaving.
    g_enabled.store(false);
    for (size_t a = 0; a < d.axisCount(); ++a) {
        if (d.hasTargetVelocity(a)) d.setTargetVelocity(a, 0);
        d.requestShutdown(a);
    }
    d.stop();
    master.stop();
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
