/**
 * @file beckhoff_pwm_sweep.cpp
 * @brief Beckhoff PWM terminal sweep demo (EL2502/EL2535/EL2564 family)
 *
 * Finds the first known PWM output terminal in the EtherCAT chain,
 * brings it to OP, enables every channel, and sweeps the duty cycle
 * up and down as a triangle wave  -  staggered per channel.
 *
 * Two display modes:
 *   - interactive TUI (default): device tree left, duty bars right.
 *     Space pauses, up/down adjusts the sweep period, q quits.
 *   - --stream: one line per poll with per-channel duty values.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_pwm_sweep                  # TUI, auto-detected NIC
 *   ./beckhoff_pwm_sweep -i enp3s0        # specify interface
 *   ./beckhoff_pwm_sweep --stream         # line mode for pipes
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include <unistd.h>

#include "tether/Beckhoff/PwmTerminal.hpp"
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

static const char* TAG = "beckhoff_pwm_sweep";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};
static std::atomic<bool> g_paused{false};
static std::atomic<int>  g_period_ms{4000};

/// Triangle wave duty for channel `ch` at time `ms` (0..kDutyFullScale).
static uint16_t sweepDuty(size_t ch, int64_t ms) {
    const int period = g_period_ms.load();
    if (period <= 0) return 0;
    const int64_t ph = (ms + static_cast<int64_t>(ch) * period / 8) % period;
    const int64_t half = period / 2;
    const int64_t tri = ph < half ? ph : period - ph;   // 0..half
    return static_cast<uint16_t>(
        tri * Beckhoff::PwmTerminal::kDutyFullScale / half);
}

static std::string statusLine(const Beckhoff::PwmTerminal& pwm) {
    std::string out;
    for (size_t k = 0; k < pwm.channelCount(); ++k) {
        if (k) out += " ";
        out += std::format("ch{}:{:5.1f}%", k + 1,
            100.0 * pwm.duty(k) / Beckhoff::PwmTerminal::kDutyFullScale);
        if (pwm.hasStatus(k)) {
            if (pwm.warning(k)) out += "!";
            if (pwm.error(k))   out += "ERR";
        }
    }
    return out;
}

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

static void runTui(Beckhoff::PwmTerminal& pwm,
                   std::span<const EtherCAT::DiscoveredSlave> slaves,
                   double duration_sec, const std::string& iface) {
    using namespace Tether::Examples;
    const auto t0 = std::chrono::steady_clock::now();

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "space pause · up/down period";
    hooks.onKey = [&](int key) {
        if (key == ' ') g_paused.store(!g_paused.load());
        else if (key == KEY_UP)
            g_period_ms.store(std::min(g_period_ms.load() + 500, 60000));
        else if (key == KEY_DOWN)
            g_period_ms.store(std::max(g_period_ms.load() - 500, 500));
        else return false;
        return true;
    };
    hooks.onTick = [&] {
        if (g_paused.load()) return;
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        for (size_t k = 0; k < pwm.channelCount(); ++k) {
            pwm.setDuty(k, sweepDuty(k, ms));
        }
    };
    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        TUI::RowWriter rw(win, 1, 1);
        const int cols = getmaxx(win);

        if (static_cast<uint16_t>(node.tag) == pwm.slaveIndex()) {
            rw.text("%s  —  slave %u, %zu ch",
                    pwm.deviceName(), pwm.slaveIndex(),
                    pwm.channelCount());
            rw.text("sweep period: %d ms%s",
                    g_period_ms.load(),
                    g_paused.load() ? "  (paused)" : "");
            rw.blank();
            const int bar_w = std::max(10, cols - 22);
            for (size_t k = 0; k < pwm.channelCount() && rw.remaining() > 1;
                 ++k) {
                const int fill =
                    pwm.duty(k) * bar_w / Beckhoff::PwmTerminal::kDutyFullScale;
                rw.put(1, 0, "ch%zu ", k + 1);
                rw.put(5, TUI::pairAttrs(TUI::PalValue) | A_BOLD, "%s",
                       std::string(fill, '#').c_str());
                rw.put(5 + fill, A_DIM, "%s",
                       std::string(bar_w - fill, '-').c_str());
                rw.put(5 + bar_w, 0, " %5.1f%%",
                       100.0 * pwm.duty(k) /
                           Beckhoff::PwmTerminal::kDutyFullScale);
                rw.advance();
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
                      "(not a PWM terminal  —  not managed by this demo)");
        }
    };

    auto managed = [&](uint16_t idx) { return idx == pwm.slaveIndex(); };
    TUI::TreeScreen screen(
        std::string("Beckhoff PWM sweep  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

static void runStream(Beckhoff::PwmTerminal& pwm, double duration_sec) {
    Tether::Platform::runCancellableLoop(
        g_cancel, duration_sec, 50, [&](double t) {
            const auto ms = static_cast<int64_t>(t * 1000.0);
            for (size_t k = 0; k < pwm.channelCount(); ++k) {
                pwm.setDuty(k, sweepDuty(k, ms));
            }
            Tether::Utils::printTimestamped(t, statusLine(pwm));
        });
    pwm.allOff();
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_pwm_sweep", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addStreamArgs(
        program, "Print duty values as plain lines instead of the TUI");

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

    // Try every known PWM identity.
    std::optional<Beckhoff::PwmTerminal> pwm_opt;
    for (const auto& id : Beckhoff::Devices::kPwmTerminals) {
        auto r = Beckhoff::PwmTerminal::findFirst(master, id, slaves);
        if (r) { pwm_opt.emplace(std::move(*r)); break; }
    }
    if (!pwm_opt) {
        TETHER_LOGE(TAG, "No PWM terminal (EL2502/EL2535/EL2564) "
                    "found in the chain");
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 6;
    }
    auto& pwm = *pwm_opt;

    if (auto r = pwm.start(); !r) {
        TETHER_LOGE(TAG, "PWM bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        EtherCAT::shutdownHostEthernet(session);
        return 7;
    }
    TETHER_LOGI(TAG, "{} on slave {}: {} channel(s)",
                pwm.deviceName(), pwm.slaveIndex(), pwm.channelCount());

#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive) {
        runTui(pwm, slaves, duration_sec, iface);
    } else
#endif
    {
        runStream(pwm, duration_sec);
    }

    pwm.allOff();
    pwm.stop();
    master.stop();
    EtherCAT::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
