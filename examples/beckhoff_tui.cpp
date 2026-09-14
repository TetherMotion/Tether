/**
 * @file beckhoff_tui.cpp
 * @brief Beckhoff Device Explorer — a unified terminal-UI catalog
 *
 * Scans the EtherCAT chain, classifies every Beckhoff terminal by its
 * product code against the implemented driver families, and presents a
 * navigable coupler/terminal device tree.  The right pane shows
 * identity, family, channel/axis/sample count, and the exact example
 * command to run for that device type.
 *
 * This is intentionally a catalog/launcher rather than a live control
 * panel: each actuator/sensor family has its own dedicated example that
 * brings the right driver to OP; `beckhoff_tui` helps find which one to
 * run and for which slave index.  Press `r` in the TUI to re-discover
 * the bus, `q` or ESC to quit.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_tui
 *   ./beckhoff_tui -i enp3s0
 *   ./beckhoff_tui -t 0          # run forever until q/ESC/SIGINT
 */

#include <atomic>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/AnalogInputTerminal.hpp"
#include "tether/Beckhoff/AnalogOutputTerminal.hpp"
#include "tether/Beckhoff/CombinedIoTerminal.hpp"
#include "tether/Beckhoff/CommTerminal.hpp"
#include "tether/Beckhoff/CompactDriveTerminal.hpp"
#include "tether/Beckhoff/DcMotorTerminal.hpp"
#include "tether/Beckhoff/InputTerminal.hpp"
#include "tether/Beckhoff/OversamplingTerminal.hpp"
#include "tether/Beckhoff/OutputTerminal.hpp"
#include "tether/Beckhoff/PositionInputTerminal.hpp"
#include "tether/Beckhoff/PowerMeterTerminal.hpp"
#include "tether/Beckhoff/PulseTrainTerminal.hpp"
#include "tether/Beckhoff/PwmTerminal.hpp"
#include "tether/Beckhoff/SafetyTerminal.hpp"
#include "tether/Beckhoff/StepperTerminal.hpp"
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

static const char* TAG = "beckhoff_tui";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};

// ============================================================================
// Device classification — product code → family + recommended example
// ============================================================================

struct TypeInfo {
    const char* family;   ///< human family name, e.g. "EL1xxx digital input"
    const char* example;  ///< example binary to run, e.g. "beckhoff_input_monitor"
    const char* args;     ///< extra args, e.g. "--axis 0"
};

namespace {

using ProductMap = std::map<uint32_t, TypeInfo>;

void add(ProductMap& m, uint32_t code, const TypeInfo& info) {
    // Later insertions win (more specific families override generic input/output).
    m[code] = info;
}

void addArray(ProductMap& m,
              std::span<const Beckhoff::DeviceIdentity> ids,
              const TypeInfo& info) {
    for (const auto& id : ids) add(m, id.product_code, info);
}

ProductMap buildProductMap() {
    ProductMap m;

    // Combined I/O first — it overlaps some timestamped input-only entries.
    addArray(m, Beckhoff::Devices::kCombinedIoTerminals,
             {"combined I/O", "beckhoff_combined_io_toggle", ""});

    // Digital I/O.
    addArray(m, Beckhoff::Devices::kInputTerminals,
             {"digital input", "beckhoff_input_monitor", ""});
    addArray(m, Beckhoff::Devices::kOutputTerminals,
             {"digital output", "beckhoff_output_toggle", ""});

    // Analog.
    addArray(m, Beckhoff::Devices::kAnalogInputTerminals,
             {"analog input", "beckhoff_analog_input_monitor", ""});
    addArray(m, Beckhoff::Devices::kAnalogOutputTerminals,
             {"analog output", "beckhoff_analog_output_sweep", ""});

    // Position / encoder.
    addArray(m, Beckhoff::Devices::kPositionInputTerminals,
             {"position/encoder", "beckhoff_position_monitor", ""});

    // Stepper & compact drive.
    addArray(m, Beckhoff::Devices::kStepperTerminals,
             {"POS stepper", "beckhoff_stepper_jog", ""});
    addArray(m, Beckhoff::Devices::kCompactDriveTerminals,
             {"DRV/CiA402 drive", "beckhoff_drive_move", ""});

    // PWM and pulse-train.
    addArray(m, Beckhoff::Devices::kPwmTerminals,
             {"PWM", "beckhoff_pwm_sweep", ""});
    addArray(m, Beckhoff::Devices::kPulseTrainTerminals,
             {"pulse-train output", "beckhoff_pulse_jog", ""});

    // Safety.
    addArray(m, Beckhoff::Devices::kSafetyTerminals,
             {"TwinSAFE FSoE", "beckhoff_safety_monitor", ""});

    // DC motor.
    addArray(m, Beckhoff::Devices::kDcMotorTerminals,
             {"DC motor", "beckhoff_dcmotor_jog", ""});

    // Power measurement.
    addArray(m, Beckhoff::Devices::kPowerMeterTerminals,
             {"power measurement", "beckhoff_power_meter", ""});

    // Oversampling.
    addArray(m, Beckhoff::Devices::kOversamplingTerminals,
             {"oversampling", "beckhoff_oversampling_monitor", ""});

    // Communication.
    addArray(m, Beckhoff::Devices::kCommTerminals,
             {"serial/IO-Link", "beckhoff_serial_bridge", ""});

    return m;
}

} // anonymous namespace

// ============================================================================
// TUI
// ============================================================================

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static const ProductMap kTypes = buildProductMap();

static const TypeInfo* classify(uint32_t product_code) {
    auto it = kTypes.find(product_code);
    return it != kTypes.end() ? &it->second : nullptr;
}

static std::string exampleCommand(const std::string& iface,
                                  uint16_t slave_index,
                                  const TypeInfo& t) {
    std::string cmd = std::string("./") + t.example
                    + " -i " + iface;
    if (t.args[0] != '\0') cmd += std::string(" ") + t.args;
    return cmd;
}

static void runTui(EtherCAT::Master& master,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;

    auto managed = [&](uint16_t idx) {
        const auto* s = slaveByIndex(slaves, idx);
        return s && s->product_code && classify(*s->product_code);
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "r: re-discover";
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int row = 1;

        const auto* s = slaveByIndex(slaves, node.tag);
        if (!s) {
            mvwprintw(win, row++, 1, "(unknown slave %d)", node.tag);
            return;
        }

        const char* sii_name =
            s->device_name ? s->device_name->c_str() : "?";
        const uint32_t product =
            s->product_code ? *s->product_code : 0;

        const auto* t = classify(product);

        mvwprintw(win, row++, 1, "%s", sii_name);
        mvwprintw(win, row++, 1, "slave   : %u", s->index);
        mvwprintw(win, row++, 1, "vendor  : 0x%08X",
                  s->vendor_id ? *s->vendor_id : 0);
        mvwprintw(win, row++, 1, "product : 0x%08X", product);

        if (t) {
            ++row;
            wattron(win, A_BOLD);
            mvwprintw(win, row++, 1, "family  : %s", t->family);
            wattroff(win, A_BOLD);
            mvwprintw(win, row++, 1, "example :");
            wattron(win, COLOR_PAIR(TUI::PalValue));
            mvwprintw(win, row - 1, 12, "%s",
                      exampleCommand(iface, s->index, *t).c_str());
            wattroff(win, COLOR_PAIR(TUI::PalValue));
        } else {
            ++row;
            wattron(win, A_DIM);
            mvwprintw(win, row++, 1,
                      "(product not in any Beckhoff driver registry)");
            wattroff(win, A_DIM);
        }

        if (!node.children.empty()) {
            ++row;
            mvwprintw(win, row++, 1, "%zu terminal(s) below",
                      node.children.size());
        }
    };

    auto discover = [&]() {
        auto fresh = master.discovery().discover(
            EtherCAT::DiscoveryOption::All);
        return buildDeviceTree(fresh, managed);
    };

    // Initial tree.
    TUI::TreeScreen screen(
        std::string("Beckhoff Device Explorer — ") + iface,
        discover(), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_tui", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
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

    bool interactive = true;
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal — TUI unavailable");
        }
        interactive = false;
    }
#else
    if (program.get<bool>("--interactive")) {
        TETHER_LOGW(TAG, "built without ncurses — TUI unavailable");
    }
    interactive = false;
#endif

    if (!interactive) {
        TETHER_LOGE(TAG, "beckhoff_tui requires a terminal with ncurses");
        return 2;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) return 3;

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

#ifdef TETHER_HAS_TERMINAL_UI
    runTui(master, slaves, duration_sec, iface);
#else
    (void)duration_sec;
#endif

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
