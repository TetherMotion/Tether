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
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
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
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        if (static_cast<uint16_t>(node.tag) == d.slaveIndex()) {
            mvwprintw(win, row++, 1, "%s  -  slave %u, %zu axis",
                      d.deviceName(), d.slaveIndex(), d.axisCount());
            ++row;
            mvwprintw(win, row++, 1, "enable=%s  speed=%d  dir=%+d",
                      g_enabled.load() ? "ON" : "off",
                      g_speed.load(), g_direction.load());
            ++row;
            for (size_t a = 0; a < d.axisCount() && row < h - 8; ++a) {
                mvwprintw(win, row++, 1, "axis %zu  sw=0x%04X", a,
                          d.statusword(a));
                const auto st = d.driveState(a);
                wattron(win, (st == DriveState::OperationEnabled)
                                 ? COLOR_PAIR(TUI::PalValue) | A_BOLD
                                 : (st == DriveState::Fault
                                        ? COLOR_PAIR(TUI::PalError) | A_BOLD
                                        : A_NORMAL));
                mvwprintw(win, row++, 3, "%s", stateName(st));
                wattroff(win, COLOR_PAIR(TUI::PalValue) |
                              COLOR_PAIR(TUI::PalError) | A_BOLD);
                if (d.hasActualPosition(a))
                    mvwprintw(win, row++, 3, "position   : %d",
                              d.actualPosition(a));
                if (d.hasActualVelocity(a))
                    mvwprintw(win, row++, 3, "velocity   : %d",
                              d.actualVelocity(a));
                if (d.hasActualTorque(a))
                    mvwprintw(win, row++, 3, "torque     : %d",
                              d.actualTorque(a));
                if (d.hasFollowingError(a))
                    mvwprintw(win, row++, 3, "following  : %d",
                              d.followingError(a));
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
                      "(not a compact drive  -  not managed by this demo)");
            wattroff(win, A_DIM);
        }
    };

    auto managed = [&](uint16_t idx) { return idx == d.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff compact drive  -  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::CompactDriveTerminal& d,
                    double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };
    g_enabled.store(true);
    int dir = 1;
    int phase = 0;
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        if (++phase % 15 == 0) dir = -dir;
        g_direction.store(dir);
        for (size_t a = 0; a < d.axisCount(); ++a) tickAxis(d, a);
        std::cout << std::format("t={:7.3f}  {}\n", stamp(), statusLine(d));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(200);
    }
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_drive_move", "1.0",
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
        .default_value(2000)
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
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    auto& d = *d_opt;

    if (auto r = d.start(); !r) {
        TETHER_LOGE(TAG, "Drive bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
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
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
