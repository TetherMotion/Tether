/**
 * @file detect_slaves.cpp
 * @brief Minimal EtherCAT slave detection example
 *
 * Scans the bus, reports the number of slaves found,
 * and prints a brief identity / SII summary for each.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./detect_slaves              # uses eth0
 *   ./detect_slaves -i enp3s0    # or: ./detect_slaves --interface enp3s0
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/sii/SIIParser.hpp"

#ifdef UNIT_TEST_HOST
#include <argparse/argparse.hpp>
#include "tether/hal/IEthernet.hpp"
#endif

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

static const char* TAG = "detect_slaves";

#ifndef UNIT_TEST_HOST
// ----- ESP-IDF / embedded entry point -----
extern "C" void detect_slaves_main(const EtherCAT::NetworkInterface* iface,
                                     const uint8_t src_mac[6]) {
    TETHER_LOGI(TAG, "detect_slaves (embedded)");

    EtherCAT::EtherCATMaster master;
    if (!iface || !src_mac) { TETHER_LOGE(TAG, "No NetworkInterface registered"); return; }
    master.start(*iface, src_mac);

    vTaskDelay(pdMS_TO_TICKS(500));

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    master.logDiscoveredSlavesSummary(TAG);
}

#else // UNIT_TEST_HOST — host/Linux build

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("detect_slaves");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    TETHER_LOGI(TAG, "detect_slaves (host) — interface: %s", iface.c_str());

    // ---- Open raw socket ----
    auto eth = EtherCAT::HAL::createDefaultEthernet();
    if (!eth) { TETHER_LOGE(TAG, "No Ethernet HAL available"); return 1; }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = iface.c_str();
    cfg.promiscuous   = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    {
        auto err = eth->init(cfg);
        if (err != EtherCAT::HAL::Error::OK) {
            if (err == EtherCAT::HAL::Error::InterfaceNotFound)
                TETHER_LOGE(TAG, "Interface '%s' not found — check `ip link`", iface.c_str());
            else if (err == EtherCAT::HAL::Error::PermissionDenied)
                TETHER_LOGE(TAG, "Permission denied — run as root or with CAP_NET_RAW");
            else
                TETHER_LOGE(TAG, "Failed to init '%s' (%s)", iface.c_str(),
                           EtherCAT::HAL::errorToString(err));
            return 2;
        }

        auto ls = eth->getLinkStatus();
        if (!ls.up) {
            TETHER_LOGE(TAG, "Link DOWN on '%s' — check cable", iface.c_str());
            return 6;
        }
    }

    // ---- MAC address ----
    EtherCAT::HAL::MacAddress mac;
    if (eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to read MAC address");
        return 3;
    }
    uint8_t src_mac[6];
    std::memcpy(src_mac, mac.bytes, 6);

    // ---- NetworkInterface wrapper ----
    auto ni_ptr = std::make_unique<EtherCAT::NetworkInterface>();
    ni_ptr->send = [eth_raw = eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth_raw->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };
    EtherCAT::Raw::set_network_interface(ni_ptr.get());
    EtherCAT::Raw::set_src_mac(src_mac);

    // ---- Create master ----
    EtherCAT::EtherCATMaster master;

    // Route RX frames directly to the master — no fragile findByNetworkInterface
    eth->setRxCallback([&master](const uint8_t* frame, size_t len,
                                  const EtherCAT::HAL::RxFrameInfo&, void*) {
        master.handleRxFrame(frame, len);
    }, nullptr);

    // ---- Poll thread ----
    std::atomic<bool> poll_running{true};
    std::thread poll_thread([&]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (poll_running.load()) eth->poll(1);
    });

    // ---- Start master + discover ----
    master.start(*EtherCAT::Raw::network_interface(), src_mac);

    // Give time for discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- Report ----
    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered %u slave(s) ===", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (slaves == 0) {
        TETHER_LOGW(TAG, "No slaves found — check wiring, power, and interface name");
    }

    // ---- Cleanup ----
    master.stop();
    poll_running.store(false);
    poll_thread.join();
    eth->shutdown();

    return (slaves > 0) ? 0 : 4;
}

#endif // UNIT_TEST_HOST
