/**
 * @file beckhoff_oversampling_monitor.cpp
 * @brief Beckhoff oversampling measurement terminal monitor
 *
 * Finds the first oversampling terminal (EL3773/EL3783 power
 * oversampling, ELM30xx–ELM37xx premium measurement), brings it to OP,
 * and once per second dumps the first/last/min/max of each channel's
 * per-cycle sample block plus the channel status word.
 *
 * Oversampling terminals transfer N samples per channel per EtherCAT
 * cycle (DC-synchronized).  The per-sample series for a full capture is
 * what an application would record  -  this demo summarizes one cycle.
 *
 * Detected set: Devices::kOversamplingTerminals (all ESI-verified for
 * shape, none verified on hardware yet).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_oversampling_monitor               # summary at 1 Hz
 *   ./beckhoff_oversampling_monitor -i enp3s0     # specify interface
 *   ./beckhoff_oversampling_monitor --full        # print every sample
 *   ./beckhoff_oversampling_monitor -t 30         # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "tether/Beckhoff/OversamplingTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "beckhoff_oversampling_monitor";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_oversampling_monitor",
                                     "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--full")
        .help("Print every sample instead of a summary")
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

    Tether::Examples::EncapConfig encap;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encap, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");
    const bool   full         = program.get<bool>("--full");

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

    std::optional<Beckhoff::OversamplingTerminal> os;
    for (const auto& id : Beckhoff::Devices::kOversamplingTerminals) {
        auto r = Beckhoff::OversamplingTerminal::findFirst(master, id,
                                                           slaves);
        if (r) { os.emplace(std::move(*r)); break; }
    }
    if (!os) {
        TETHER_LOGE(TAG, "No oversampling terminal found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} at slave {}: {} channel(s)",
                os->deviceName(), os->slaveIndex(), os->channels());
    for (size_t c = 0; c < os->channels(); ++c) {
        TETHER_LOGI(TAG, "  ch{}: {} samples/cycle x {} bit", c,
                    os->samplesPerCycle(c), os->sampleBits(c));
    }

    if (auto r = os->start(); !r) {
        TETHER_LOGE(TAG, "bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    std::vector<int32_t> buf;
    const auto t0 = std::chrono::steady_clock::now();
    while (!g_cancel.load()) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (duration_sec > 0.0 && t >= duration_sec) break;

        for (size_t c = 0; c < os->channels(); ++c) {
            const size_t n = os->samplesPerCycle(c);
            buf.resize(n);
            os->samples(c, buf);
            if (full) {
                std::string line;
                for (size_t i = 0; i < n; ++i)
                    line += std::format("{}{}", i ? "," : "",
                                        buf[i]);
                std::cout << std::format("t={:8.3f} ch{}: {}\n",
                                         t, c, line);
            } else {
                int32_t lo = 0, hi = 0;
                for (int32_t v : buf) {
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
                std::cout << std::format(
                    "t={:8.3f} ch{}: n={} first={} last={} "
                    "min={} max={} status=0x{:04X}\n",
                    t, c, n,
                    n ? buf.front() : 0, n ? buf.back() : 0,
                    lo, hi, os->statusWord(c));
            }
        }
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(1000);
    }

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
