/**
 * @file el1014_monitor.cpp
 * @brief Beckhoff EL1014 (4ch digital input, 24V/10us) live monitor demo
 *
 * Finds every EL1014 in the EtherCAT chain via the MultiEL1014 driver,
 * brings all of them to OP, and displays the live state of every input
 * channel.  Module 0 occupies bits 0-3, module 1 bits 4-7, and so on.
 *
 * Two display modes:
 *   - ncurses panel (default on a terminal): per-module channel
 *     indicators, per-channel transition counters and a small log area.
 *     Press 'q' to quit.
 *   - --stream: plain stdout — one line whenever any input changes
 *     (plus the initial state).  Also used automatically when ncurses
 *     is unavailable or stdout is not a terminal.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./el1014_monitor                # TUI on auto-detected interface
 *   ./el1014_monitor -i enp3s0      # specify interface
 *   ./el1014_monitor --stream       # line mode for pipes/scripts
 *   ./el1014_monitor -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include <poll.h>
#include <unistd.h>

#include "tether/Beckhoff/MultiEL1014.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"
#include "logging/Logger.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

// ncurses last: it #defines OK/ERR/timeout/... which collide with
// identifiers in the Tether headers (e.g. HALTypes' enum class Error::OK).
#ifdef HAVE_NCURSES
#include <clocale>
#include <mutex>
#include <ncurses.h>
#endif

static const char* TAG = "el1014_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;
namespace Platform = Tether::Platform;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Compact per-module bit rendering: "s1:1001 s2:0000".
static std::string moduleStates(const Beckhoff::MultiEL1014<>& ins) {
    std::string out;
    for (size_t m = 0; m < ins.moduleCount(); ++m) {
        out += " s" + std::to_string(ins.slaveIndex(m)) + ":";
        const auto& mod = ins.module(m);
        for (size_t k = 0; k < mod.bitCount(); ++k) {
            out += mod.bit(k) ? '1' : '0';
        }
    }
    return out;
}

#ifdef HAVE_NCURSES
// ---------------------------------------------------------------------------
// ncurses TUI
// ---------------------------------------------------------------------------

/// Log lines captured while curses owns the screen.
static std::deque<std::string> g_log_lines;
static std::mutex              g_log_mutex;
constexpr size_t kMaxLogLines = 4;

static void captureLogToTui() {
    Platform::Logger::instance().setHandler(
        [](Platform::LogLevel level, const char* tag, const char* msg) {
            static const char* lv[] = {"", "E", "W", "I", "D", "V"};
            const char* l = lv[std::min<int>(
                static_cast<int>(level), 5)];
            std::lock_guard<std::mutex> lock(g_log_mutex);
            g_log_lines.emplace_back(
                std::string(l) + " " + tag + ": " + msg);
            while (g_log_lines.size() > kMaxLogLines) g_log_lines.pop_front();
        });
}

static void runTui(Beckhoff::MultiEL1014<>& ins, double duration_sec,
                   const std::string& iface) {
    setlocale(LC_ALL, "");
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    const bool colors = has_colors();
    if (colors) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN,  -1);   // channel ON
        init_pair(2, COLOR_CYAN,   -1);   // header
        init_pair(3, COLOR_YELLOW, -1);   // footer/keys
        init_pair(4, COLOR_RED,    -1);   // log lines
    }

    std::vector<uint32_t> transitions(ins.channelCount(), 0);
    auto prev = ins.bits();
    const auto t0 = std::chrono::steady_clock::now();

    while (!g_cancel.load()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (duration_sec > 0.0 && elapsed >= duration_sec) break;

        const auto now = ins.bits();
        for (size_t b = 0; b < ins.channelCount(); ++b) {
            if (now[b] != prev[b]) ++transitions[b];
        }
        prev = now;

        erase();
        int row = 0;

        attron(A_BOLD | (colors ? COLOR_PAIR(2) : 0));
        mvprintw(row++, 0,
                 "EL1014 Digital Input Monitor - %zu module(s), %zu channel(s) on %s",
                 ins.moduleCount(), ins.channelCount(), iface.c_str());
        attroff(A_BOLD | (colors ? COLOR_PAIR(2) : 0));
        ++row;

        mvprintw(row++, 0, " slave  device   channels                 trans");
        mvchgat(row - 1, 0, -1, A_UNDERLINE, 0, nullptr);

        for (size_t m = 0; m < ins.moduleCount(); ++m) {
            const auto& mod = ins.module(m);
            const size_t  w = mod.bitCount();
            mvprintw(row, 0, " s%-6u %-8s  ",
                     static_cast<unsigned>(ins.slaveIndex(m)),
                     mod.deviceName());
            int col = 22;
            uint32_t mod_trans = 0;
            for (size_t k = 0; k < w; ++k) {
                const bool on = mod.bit(k);
                mod_trans += transitions[ins.bitOffset(m) + k];
                if (colors) {
                    attron(on ? (COLOR_PAIR(1) | A_BOLD) : A_DIM);
                }
                mvprintw(row, col, "%zu", k + 1);
                mvprintw(row, col + 1, on ? "*" : ".");
                if (colors) {
                    attroff(on ? (COLOR_PAIR(1) | A_BOLD) : A_DIM);
                }
                col += 4;
            }
            mvprintw(row, 42, "%8u", mod_trans);
            ++row;
        }
        ++row;

        // Flat bit field, grouped in nibbles.
        mvprintw(row++, 0, " flat : ");
        int col = 8;
        for (size_t b = 0; b < ins.channelCount(); ++b) {
            mvaddch(row - 1, col++, now[b] ? '1' : '0');
            if ((b + 1) % 4 == 0) ++col;
        }

        mvprintw(row++, 0, " t=%.1fs", elapsed);

        // Recent log lines (captured so they don't corrupt the screen).
        {
            std::lock_guard<std::mutex> lock(g_log_mutex);
            for (const auto& line : g_log_lines) {
                if (row < LINES - 2) {
                    if (colors) attron(COLOR_PAIR(4));
                    mvprintw(row++, 0, " %.*s", COLS - 2, line.c_str());
                    if (colors) attroff(COLOR_PAIR(4));
                }
            }
        }

        attron(colors ? COLOR_PAIR(3) : 0);
        mvprintw(LINES - 1, 0, " q: quit");
        attroff(colors ? COLOR_PAIR(3) : 0);

        refresh();

        const int ch = getch();
        if (ch == 'q' || ch == 'Q' || ch == 27) break;

        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }

    endwin();
    Platform::Logger::instance().setHandler(nullptr);   // restore console
}
#endif // HAVE_NCURSES

// ---------------------------------------------------------------------------
// Stream mode — one line per input change (pipe-friendly)
// ---------------------------------------------------------------------------

static void runStream(Beckhoff::MultiEL1014<>& ins, double duration_sec) {
    const auto t0 = std::chrono::steady_clock::now();
    auto prev = ins.bits();

    auto stamp = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    std::cout << std::format("t={:7.3f}{}\n", stamp(), moduleStates(ins));
    std::cout.flush();

    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && stamp() >= duration_sec) break;
        const auto now = ins.bits();
        if (now != prev) {
            prev = now;
            std::cout << std::format("t={:7.3f}{}\n",
                                     stamp(), moduleStates(ins));
            std::cout.flush();
        }
        Tether::Platform::Clock::instance().delayMilliseconds(20);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("el1014_monitor", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--stream")
        .help("Print input changes as plain lines instead of the TUI")
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
        Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
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
    bool stream_mode = program.get<bool>("--stream");

#ifndef HAVE_NCURSES
    if (!stream_mode) {
        TETHER_LOGW(TAG, "built without ncurses — using --stream mode");
        stream_mode = true;
    }
#else
    if (!stream_mode && !isatty(STDOUT_FILENO)) {
        stream_mode = true;   // piped output — curses would emit escape codes
    }
#endif

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    // ---- Host Ethernet + master bring-up ----
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

    // ---- Discover the chain and pick out every EL1014 ----
    // A full discovery gives us the slave names for logging and lets the
    // driver reuse the SII data instead of re-reading each terminal's EEPROM.
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

    Beckhoff::MultiEL1014<> ins(master);
    auto found = ins.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No EL1014 found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} EL1014 terminal(s), {} input bits total",
                ins.moduleCount(), ins.channelCount());

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = ins.start(); !r) {
        TETHER_LOGE(TAG, "EL1014 bring-up failed on module {}: {}",
                    ins.lastErrorModule(),
                    Beckhoff::EL1014::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- Display loop ----
#ifdef HAVE_NCURSES
    if (!stream_mode) {
        captureLogToTui();
        runTui(ins, duration_sec, iface);
    } else
#endif
    {
        runStream(ins, duration_sec);
    }

    // ---- Shutdown ----
    ins.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}
