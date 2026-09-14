/**
 * @file beckhoff_serial_bridge.cpp
 * @brief Beckhoff EL6xxx serial/communication terminal FIFO demo
 *
 * Finds the first byte-FIFO communication terminal (EL6001/EL6002
 * RS232, EL6021/EL6022 RS422/485, EL6080 memory, EL6090 display,
 * EL6224 IO-Link, plus EJ/EP/ER/EPP variants), brings it to OP, and
 * exercises the process-data FIFOs: every second it writes a counter
 * pattern into each channel's transmit image and dumps the receive
 * image.  Ctrl/status words and individual handshake bits are shown so
 * the raw handshake is visible.
 *
 * This is a raw-FIFO demonstration  -  it does NOT implement the EL6001
 * receive/transmit handshake (ctrl bit toggling) or any serial
 * protocol; on real hardware the terminal's UART must be configured via
 * CoE SDOs first (baud rate, data bits).
 *
 * Detected set: Devices::kCommTerminals (all ESI-verified for shape,
 * none verified on hardware yet).
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_serial_bridge                # FIFO dump at 1 Hz
 *   ./beckhoff_serial_bridge -i enp3s0      # specify interface
 *   ./beckhoff_serial_bridge --send 4869    # payload hex bytes
 *   ./beckhoff_serial_bridge -t 30          # run for 30 s
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "tether/Beckhoff/CommTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "beckhoff_serial_bridge";

namespace Beckhoff = EtherCAT::Beckhoff;

static std::atomic<bool> g_cancel{false};

static std::string hexDump(std::span<const uint8_t> d) {
    std::string out;
    for (uint8_t b : d) out += std::format("{:02X} ", b);
    return out;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_serial_bridge", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--send")
        .help("Hex bytes to write into each channel's TX FIFO image "
              "(e.g. 48656C6C6F); default: a rolling counter pattern")
        .default_value<std::string>("");

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
    const std::string send_hex = program.get<std::string>("--send");

    std::vector<uint8_t> payload;
    for (size_t i = 0; i + 1 < send_hex.size(); i += 2) {
        payload.push_back(static_cast<uint8_t>(
            std::stoul(send_hex.substr(i, 2), nullptr, 16)));
    }

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

    std::optional<Beckhoff::CommTerminal> comm;
    for (const auto& id : Beckhoff::Devices::kCommTerminals) {
        auto r = Beckhoff::CommTerminal::findFirst(master, id, slaves);
        if (r) { comm.emplace(std::move(*r)); break; }
    }
    if (!comm) {
        TETHER_LOGE(TAG, "No EL6xxx communication terminal found");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }
    TETHER_LOGI(TAG, "{} at slave {}: {} FIFO channel(s)",
                comm->deviceName(), comm->slaveIndex(), comm->channels());
    for (size_t c = 0; c < comm->channels(); ++c) {
        TETHER_LOGI(TAG, "  ch{}: tx {} B / rx {} B", c,
                    comm->txCapacity(c), comm->rxCapacity(c));
    }

    if (auto r = comm->start(); !r) {
        TETHER_LOGE(TAG, "bring-up failed: {}",
                    Beckhoff::errorToString(r.error()));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    const auto t0 = std::chrono::steady_clock::now();
    uint8_t seq = 0;
    while (!g_cancel.load()) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (duration_sec > 0.0 && t >= duration_sec) break;

        for (size_t c = 0; c < comm->channels(); ++c) {
            if (payload.empty()) {
                std::vector<uint8_t> pat(comm->txCapacity(c));
                for (size_t i = 0; i < pat.size(); ++i)
                    pat[i] = static_cast<uint8_t>(seq + i);
                comm->writeData(c, pat);
            } else {
                comm->writeData(c, payload);
            }
        }
        ++seq;

        for (size_t c = 0; c < comm->channels(); ++c) {
            std::cout << std::format(
                "t={:8.3f} ch{}: status=0x{:04X} rx[{}]: {}\n",
                t, c, comm->statusWord(c), comm->rxCapacity(c),
                hexDump(comm->dataIn(c)));
        }
        std::cout.flush();
        Tether::Platform::Clock::instance().delayMilliseconds(1000);
    }

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
