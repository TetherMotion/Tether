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
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
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
        int h = 0, cols = 0;
        getmaxyx(win, h, cols);
        int row = 1;

        if (static_cast<uint16_t>(node.tag) == pwm.slaveIndex()) {
            mvwprintw(win, row++, 1, "%s  —  slave %u, %zu ch",
                      pwm.deviceName(), pwm.slaveIndex(),
                      pwm.channelCount());
            mvwprintw(win, row++, 1, "sweep period: %d ms%s",
                      g_period_ms.load(),
                      g_paused.load() ? "  (paused)" : "");
            ++row;
            const int bar_w = std::max(10, cols - 22);
            for (size_t k = 0; k < pwm.channelCount() && row < h - 1; ++k) {
                const int fill =
                    pwm.duty(k) * bar_w / Beckhoff::PwmTerminal::kDutyFullScale;
                mvwprintw(win, row, 1, "ch%zu ", k + 1);
                wattron(win, COLOR_PAIR(TUI::PalValue) | A_BOLD);
                for (int i = 0; i < fill; ++i) waddch(win, '#');
                wattroff(win, COLOR_PAIR(TUI::PalValue) | A_BOLD);
                wattron(win, A_DIM);
                for (int i = fill; i < bar_w; ++i) waddch(win, '-');
                wattroff(win, A_DIM);
                mvwprintw(win, row, 5 + bar_w, " %5.1f%%",
                          100.0 * pwm.duty(k) /
                              Beckhoff::PwmTerminal::kDutyFullScale);
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
                      "(not a PWM terminal  —  not managed by this demo)");
            wattroff(win, A_DIM);
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
    const auto t0 = std::chrono::steady_clock::now();
    auto ms = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
    };
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && ms() / 1000.0 >= duration_sec) break;
        for (size_t k = 0; k < pwm.channelCount(); ++k) {
            pwm.setDuty(k, sweepDuty(k, ms()));
        }
        std::cout << std::format("t={:7.3f}  {}\n", ms() / 1000.0,
                                 statusLine(pwm));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }
    pwm.allOff();
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_pwm_sweep", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help("Print duty values as plain lines instead of the TUI")
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
    std::string iface =
        Tether::Examples::resolveInterface(
            program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::EncapConfig encap;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encap, TAG)) {
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
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
    Tether::Examples::applyDebugFlags(debug_flags, master, TAG);

    if (!Tether::Examples::setupEncapAndRxCallback(session, master, encap, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, encap, TAG)) {
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
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    auto& pwm = *pwm_opt;

    if (auto r = pwm.start(); !r) {
        TETHER_LOGE(TAG, "PWM bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
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
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
