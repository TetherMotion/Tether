/**
 * @file beckhoff_output_toggle.cpp
 * @brief Beckhoff EL200x digital output channel stepper demo
 *
 * Finds every EL200x terminal in the EtherCAT chain via the MultiOutputTerminal
 * driver, brings all of them to OP, and lights exactly one output at a
 * time across the combined channel space (the module at the lowest bus
 * position occupies bits [0, w0), the next one [w0, w0+w1), ... where wN
 * is that terminal's channel count — 2 for EL2002, 4 for EL2004, 8 for
 * EL2008).  Pressing Enter advances the lit output; after the last channel
 * of the last terminal it wraps back to the very first one.
 *
 * Detected by this example (the "EL200x" series, x = channel count):
 *   EL2002 (2ch, 24V/0.5A), EL2004 (4ch), EL2008 (8ch).
 *   Verified on hardware: EL2004.  EL2002/EL2008 share the identical ESI
 *   shape — supported, not verified yet.
 *
 * The same OutputTerminal driver also supports every other terminal in
 * Devices::kOutputTerminals — all verified against the ESI to share the
 * EL2004's shape (single "Outputs" SM + FMMU, N x 1-bit RxPDOs, no
 * mailbox), none verified on hardware yet.  Swap kEl200x for
 * Devices::kOutputTerminals in detect() to accept them all.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_output_toggle                # auto-detect interface
 *   ./beckhoff_output_toggle -i enp3s0      # specify interface
 *   ./beckhoff_output_toggle -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <poll.h>
#include <unistd.h>

#include "tether/Beckhoff/MultiOutputTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "beckhoff_output_toggle";

namespace Beckhoff = EtherCAT::Beckhoff;

/// The "EL200x" detection set — x = channel count.
constexpr Beckhoff::DeviceIdentity kEl200x[] = {
    Beckhoff::Devices::EL2002,
    Beckhoff::Devices::EL2004,
    Beckhoff::Devices::EL2008,
};

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Print one line per module: "s3: 0100" means slave 3 has channel 3 lit.
static void printOutputState(const Beckhoff::MultiOutputTerminal<>& outs,
                             size_t active) {
    std::cout << "Outputs:";
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        const auto& mod = outs.module(m);
        std::cout << "  s" << outs.slaveIndex(m) << ":";
        for (size_t k = 0; k < mod.bitCount(); ++k) {
            std::cout << (mod.bit(k) ? '1' : '0');
        }
    }
    // Locate the module + channel the active bit belongs to.
    size_t mod = 0, off = 0;
    for (; mod + 1 < outs.moduleCount(); ++mod) {
        if (outs.bitOffset(mod + 1) > active) break;
    }
    off = active - outs.bitOffset(mod);
    std::cout << "   (bit " << active << " = slave " << outs.slaveIndex(mod)
              << " CH" << off + 1
              << " — press Enter for next)" << std::endl;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_output_toggle", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);

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

    // ---- Discover the chain and pick out every EL200x ----
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

    Beckhoff::MultiOutputTerminal<> outs(master);
    auto found = outs.detect(std::span<const Beckhoff::DeviceIdentity>(kEl200x),
                           slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No EL200x terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} EL200x terminal(s), {} output bits total",
                outs.moduleCount(), outs.channelCount());
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        TETHER_LOGI(TAG, "  module {}: slave {} = {} ({} ch)",
                    m, outs.slaveIndex(m), outs.module(m).deviceName(),
                    outs.module(m).bitCount());
    }

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = outs.start(); !r) {
        TETHER_LOGE(TAG, "EL200x bring-up failed on module {}: {}",
                    outs.lastErrorModule(),
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- UI loop: Enter steps the single lit output across all bits ----
    std::cout << "\n" << outs.moduleCount() << " EL200x terminal(s), "
              << outs.channelCount() << " channels — exactly one output is ON.\n";
    size_t active = 0;
    outs.setOnly(active);
    printOutputState(outs, active);

    const auto start_time = std::chrono::steady_clock::now();
    bool stdin_open = true;

    while (!g_cancel.load()) {
        if (duration_sec > 0.0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= duration_sec) break;
        }

        if (!stdin_open) {
            Tether::Platform::Clock::instance().delayMilliseconds(100);
            continue;
        }

        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        int pr = ::poll(&pfd, 1, 100);
        if (pr <= 0) continue;
        if (pfd.revents & POLLIN) {
            std::string line;
            if (!std::getline(std::cin, line)) {
                stdin_open = false;  // piped/closed stdin — keep running
                continue;
            }
            active = (active + 1) % outs.channelCount();
            outs.setOnly(active);
            printOutputState(outs, active);
        } else if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            stdin_open = false;
        }
    }

    // ---- Shutdown: all outputs off, then stop ----
    outs.stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. All outputs off.");
    return 0;
}
