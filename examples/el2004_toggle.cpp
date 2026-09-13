/**
 * @file el2004_toggle.cpp
 * @brief Beckhoff EL2004 (4ch digital output, 24V/0.5A) channel stepper demo
 *
 * Finds every EL2004 in the EtherCAT chain via the MultiEL2004 driver,
 * brings all of them to OP, and lights exactly one output at a time across
 * the combined channel space (module 0 = bits 0-3, module 1 = bits 4-7, ...).
 * Pressing Enter advances the lit output; after the last channel of the last
 * terminal it wraps back to the very first one.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./el2004_toggle                # auto-detect interface
 *   ./el2004_toggle -i enp3s0      # specify interface
 *   ./el2004_toggle -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <poll.h>
#include <unistd.h>

#include "tether/Beckhoff/MultiEL2004.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "el2004_toggle";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};
// SignalHandler sets this to true on SIGINT/SIGTERM.

/// Print one line per module: "s3: 0100" means slave 3 has channel 3 lit.
static void printOutputState(const Beckhoff::MultiEL2004<>& outs,
                             size_t active) {
    std::cout << "Outputs:";
    for (size_t m = 0; m < outs.moduleCount(); ++m) {
        const auto ch = outs.module(m).channels();
        std::cout << "  s" << outs.slaveIndex(m) << ":";
        for (size_t k = 0; k < Beckhoff::EL2004::kNumChannels; ++k) {
            std::cout << (ch[k] ? '1' : '0');
        }
    }
    const size_t mod = active / Beckhoff::EL2004::kNumChannels;
    std::cout << "   (bit " << active << " = slave " << outs.slaveIndex(mod)
              << " CH" << (active % Beckhoff::EL2004::kNumChannels) + 1
              << " — press Enter for next)" << std::endl;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("el2004_toggle", "1.0",
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

    // ---- Discover the chain and pick out every EL2004 ----
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

    Beckhoff::MultiEL2004<> outs(master);
    auto found = outs.detect(slaves);
    if (!found || *found == 0) {
        TETHER_LOGE(TAG, "No EL2004 found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} EL2004 terminal(s), {} output bits total",
                outs.moduleCount(), outs.channelCount());

    // ---- Configure all modules, start the RT loop, enter OP ----
    if (auto r = outs.start(); !r) {
        TETHER_LOGE(TAG, "EL2004 bring-up failed on module {}: {}",
                    outs.lastErrorModule(),
                    Beckhoff::EL2004::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // ---- UI loop: Enter steps the single lit output across all bits ----
    std::cout << "\n" << outs.moduleCount() << " EL2004 terminal(s), "
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
