/**
 * @file beckhoff_dcmotor_jog.cpp
 * @brief Beckhoff EL73xx DC-motor (H-bridge) terminal jog demo
 *
 * Finds the first MOT-interface DC-motor terminal (EL7332/EL7342 +
 * EJ/EP/ER/EPP variants), enables each axis' power stage, and applies a
 * velocity setpoint that flips direction periodically.  EL7342-class
 * terminals also report their per-axis encoder counter/latch values.
 *
 * Detected set: Devices::kDcMotorTerminals (all ESI-verified for shape,
 * none verified on hardware yet).
 *
 * CAUTION: real motors will spin.  Use --velocity 0 for a dry-run that
 * only exercises enable/status.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_dcmotor_jog                        # jog at default speed
 *   ./beckhoff_dcmotor_jog -i enp3s0 -v 5000      # custom velocity
 *   ./beckhoff_dcmotor_jog --period 3             # flip every 3 s
 *   ./beckhoff_dcmotor_jog -v 0                   # enable only, no motion
 *   ./beckhoff_dcmotor_jog -t 30                  # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "tether/Beckhoff/DcMotorTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "beckhoff_dcmotor_jog";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};

static std::string axisStates(const Beckhoff::DcMotorTerminal& mot) {
    std::string out;
    for (size_t a = 0; a < mot.axisCount(); ++a) {
        out += std::format(
            " ax{}[{}{}{}{}{}]",
            a,
            mot.ready(a) ? "R" : "-",
            mot.movingPositive(a) ? "+" : "",
            mot.movingNegative(a) ? "-" : "",
            mot.warning(a) ? "W" : "",
            mot.error(a) ? "E" : "");
        if (mot.hasEncoder(a)) {
            out += std::format(" cnt={}", mot.counterValue(a));
        }
    }
    return out;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_dcmotor_jog", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--velocity")
        .help("Velocity setpoint (signed 16-bit raw); 0 = enable only")
        .default_value<int>(10000)
        .scan<'i', int>();
    program.add_argument("--period")
        .help("Direction-flip period in seconds")
        .default_value<double>(2.0)
        .scan<'g', double>();
    program.add_argument("--axis")
        .help("Jog only this axis (default: all)")
        .default_value<int>(-1)
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

    std::string iface = Tether::Examples::resolveInterface(
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
    const int    velocity     = program.get<int>("--velocity");
    const double period       = program.get<double>("--period");
    const int    only_axis    = program.get<int>("--axis");

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) return 2;

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
    for (const auto& s : slaves) {
        TETHER_LOGI(TAG, "Slave {}: {} (vendor=0x{:08X} product=0x{:08X})",
                    s.index,
                    s.device_name ? s.device_name->c_str() : "?",
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);
    }

    // First matching DC-motor terminal on the bus.
    std::optional<Beckhoff::DcMotorTerminal> mot;
    for (const auto& id : Beckhoff::Devices::kDcMotorTerminals) {
        auto r = Beckhoff::DcMotorTerminal::findFirst(master, id, slaves);
        if (r) { mot.emplace(std::move(*r)); break; }
    }
    if (!mot) {
        TETHER_LOGE(TAG, "No EL73xx DC-motor terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} at slave {}: {} axis(es)",
                mot->deviceName(), mot->slaveIndex(), mot->axisCount());

    if (auto r = mot->start(); !r) {
        TETHER_LOGE(TAG, "bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // Enable every axis (clear a pending fault first, then power on).
    for (size_t a = 0; a < mot->axisCount(); ++a) {
        mot->setReset(a, true);
    }
    Tether::Platform::Clock::instance().delayMilliseconds(50);
    for (size_t a = 0; a < mot->axisCount(); ++a) {
        mot->setReset(a, false);
        mot->setEnable(a, true);
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    int16_t vel = static_cast<int16_t>(velocity);
    TETHER_LOGI(TAG, "jogging at velocity {} (flip every {} s)",
                vel, period);
    while (!g_cancel.load()) {
        if (duration_sec > 0.0 && elapsed() >= duration_sec) break;
        const int16_t v =
            (period > 0.0 &&
             static_cast<long>(elapsed() / period) % 2 == 1) ? -vel : vel;
        for (size_t a = 0; a < mot->axisCount(); ++a) {
            if (only_axis < 0 || static_cast<size_t>(only_axis) == a) {
                mot->setVelocity(a, v);
            }
        }
        std::cout << std::format("t={:7.3f} vel={:>6}{}\n",
                                 elapsed(), v, axisStates(*mot));
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(200);
    }

    // Ramp down and disable before leaving OP.
    for (size_t a = 0; a < mot->axisCount(); ++a) {
        mot->setVelocity(a, 0);
        mot->setEnable(a, false);
    }
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
