/**
 * @file beckhoff_power_meter.cpp
 * @brief Beckhoff EL34xx power-measurement terminal monitor
 *
 * Finds the first EL34xx terminal (EL3403/3413/3423/3433/3443/3453/
 * 3475/3483, EKM1101), brings it to OP, and prints the per-phase
 * measurements once per second: voltage, current, active/apparent/
 * reactive power and power factor, plus the raw status word.
 *
 * For index-addressed devices (family C: EL3413/3433/3475) the RX index
 * selectors are left at 0 — the echoed per-selector value is shown when
 * mapped; per-phase accessors show whatever the device exposes directly.
 *
 * Detected set: Devices::kPowerMeterTerminals (all ESI-verified for
 * shape, none verified on hardware yet).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_power_meter                  # monitor at 1 Hz
 *   ./beckhoff_power_meter -i enp3s0        # specify interface
 *   ./beckhoff_power_meter -t 60            # run for 60 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "tether/Beckhoff/PowerMeterTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "beckhoff_power_meter";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};

static void printPhase(const Beckhoff::PowerMeterTerminal& pm,
                       size_t p) {
    auto fmt = [](std::optional<float> v, const char* unit) {
        return v ? std::format("{:9.2f}{}", *v, unit)
                 : std::string("        --");
    };
    std::cout << std::format(
        "  L{}: U={} I={} P={} S={} Q={} PF={} status=0x{:04X}{}\n",
        p + 1,
        fmt(pm.voltage(p), "V"), fmt(pm.current(p), "A"),
        fmt(pm.activePower(p), "W"), fmt(pm.apparentPower(p), "VA"),
        fmt(pm.reactivePower(p), "var"), fmt(pm.powerFactor(p), ""),
        pm.statusWord(p),
        pm.syncError(p) ? " SYNC-ERR" : "");
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_power_meter", "1.0",
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

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) return 2;

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
    for (const auto& s : slaves) {
        TETHER_LOGI(TAG, "Slave {}: {} (vendor=0x{:08X} product=0x{:08X})",
                    s.index,
                    s.device_name ? s.device_name->c_str() : "?",
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);
    }

    std::optional<Beckhoff::PowerMeterTerminal> pm;
    for (const auto& id : Beckhoff::Devices::kPowerMeterTerminals) {
        auto r = Beckhoff::PowerMeterTerminal::findFirst(master, id, slaves);
        if (r) { pm.emplace(std::move(*r)); break; }
    }
    if (!pm) {
        TETHER_LOGE(TAG, "No EL34xx power-measurement terminal found");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} at slave {}: {} phase(s), {} index selector(s)",
                pm->deviceName(), pm->slaveIndex(), pm->phaseCount(),
                pm->indexSelectors());

    if (auto r = pm->start(); !r) {
        TETHER_LOGE(TAG, "bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    const auto t0 = std::chrono::steady_clock::now();
    while (!g_cancel.load()) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (duration_sec > 0.0 && t >= duration_sec) break;

        std::cout << std::format("t={:8.3f}\n", t);
        for (size_t p = 0; p < pm->phaseCount(); ++p) {
            printPhase(*pm, p);
        }
        for (size_t s = 0; s < pm->indexSelectors(); ++s) {
            auto v = pm->indexedValue(s);
            auto ix = pm->indexedIndexEcho(s);
            std::cout << std::format(
                "  sel{}: idx_echo={} value={}\n", s,
                ix ? std::format("{}", *ix) : std::string("--"),
                v ? std::format("{:.2f}", *v) : std::string("--"));
        }
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(1000);
    }

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
