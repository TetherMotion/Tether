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
#include <set>
#include <sstream>
#include <thread>
#include <magic_enum/magic_enum.hpp>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include <argparse/argparse.hpp>
#include "tether/hal/IEthernet.hpp"

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

static const char* TAG = "detect_slaves";

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("detect_slaves");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Known flags: sii-derivation, mailbox-configuration, al-state, tx-ethercat-packets, rx-ethercat-packets, rx-pdo, tx-pdo");
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID (e.g. 100), range (e.g. 100-200), or 'any' for catch-all undefined target");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID (e.g. 100)");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    std::string debug_str = program.get<std::string>("--debug");
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");

    // Known debug flags
    const std::set<std::string> known_debug_flags = {
        "sii-derivation",
        "mailbox-configuration",
        "al-state",
        "tx-ethercat-packets",
        "rx-ethercat-packets",
        "rx-pdo",
        "tx-pdo"
    };

    // Parse debug flags
    std::set<std::string> debug_flags;
    std::set<std::string> unknown_flags;
    if (!debug_str.empty()) {
        std::stringstream ss(debug_str);
        std::string flag;
        while (std::getline(ss, flag, ',')) {
            // Trim whitespace
            flag.erase(0, flag.find_first_not_of(" \t"));
            flag.erase(flag.find_last_not_of(" \t") + 1);
            if (!flag.empty()) {
                debug_flags.insert(flag);
                if (known_debug_flags.find(flag) == known_debug_flags.end()) {
                    unknown_flags.insert(flag);
                }
            }
        }
    }

    // Warn about unknown debug flags
    if (!unknown_flags.empty()) {
        TETHER_LOGW(TAG, "Unknown debug flags:");
        for (const auto& flag : unknown_flags) {
            TETHER_LOGW(TAG, "  - %s", flag.c_str());
        }
        TETHER_LOGW(TAG, "Known debug flags:");
        for (const auto& flag : known_debug_flags) {
            TETHER_LOGW(TAG, "  - %s", flag.c_str());
        }
    }

    // Enable statemachine debug if requested
    if (debug_flags.count("al-state")) {
        EtherCAT::enableStateMachineDebug(true);
        TETHER_LOGI(TAG, "EtherCAT state machine debug logging enabled");
    }

    // Enable TX packet debug if requested
    if (debug_flags.count("tx-ethercat-packets")) {
        EtherCAT::enableTxPacketDebug(true);
        TETHER_LOGI(TAG, "TX EtherCAT packet debug logging enabled");
    }

    // Enable RX packet debug if requested
    if (debug_flags.count("rx-ethercat-packets")) {
        EtherCAT::enableRxPacketDebug(true);
        TETHER_LOGI(TAG, "RX EtherCAT packet debug logging enabled");
    }

    // Enable RxPDO debug if requested
    if (debug_flags.count("rx-pdo")) {
        EtherCAT::enableRxPDODebug(true);
        TETHER_LOGI(TAG, "RxPDO debug logging enabled");
    }

    // Enable TxPDO debug if requested
    if (debug_flags.count("tx-pdo")) {
        EtherCAT::enableTxPDODebug(true);
        TETHER_LOGI(TAG, "TxPDO debug logging enabled");
    }

    // ---- Parse VLAN arguments ----
    bool vlan_mode = !rx_vlan_str.empty() || !tx_vlan_str.empty();
    std::optional<uint16_t> tx_vlan;
    bool rx_any = false;
    std::optional<EtherCAT::VLANRouter::VLANRange> rx_range;

    if (vlan_mode) {
        if (!tx_vlan_str.empty()) {
            try {
                int v = std::stoi(tx_vlan_str);
                if (v < 1 || v > 4095) {
                    std::cerr << "--tx-vlan must be in range 1–4095\n";
                    return 1;
                }
                tx_vlan = static_cast<uint16_t>(v);
            } catch (...) {
                std::cerr << "Invalid --tx-vlan value: " << tx_vlan_str << "\n";
                return 1;
            }
        }

        if (!rx_vlan_str.empty()) {
            if (rx_vlan_str == "any") {
                rx_any = true;
            } else {
                // Parse single VID or range "start-end"
                size_t dash = rx_vlan_str.find('-');
                try {
                    if (dash == std::string::npos) {
                        int v = std::stoi(rx_vlan_str);
                        if (v < 1 || v > 4095) {
                            std::cerr << "--rx-vlan must be in range 1–4095\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(v), static_cast<uint16_t>(v)};
                    } else {
                        int start = std::stoi(rx_vlan_str.substr(0, dash));
                        int end   = std::stoi(rx_vlan_str.substr(dash + 1));
                        if (start < 1 || end > 4095 || start > end) {
                            std::cerr << "--rx-vlan range must be 1–4095 with start <= end\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(start), static_cast<uint16_t>(end)};
                    }
                } catch (...) {
                    std::cerr << "Invalid --rx-vlan value: " << rx_vlan_str << "\n";
                    return 1;
                }
            }
        }
    }

    TETHER_LOGI(TAG, "detect_slaves (host) — interface: %s", iface.c_str());
    if (!debug_flags.empty()) {
        TETHER_LOGI(TAG, "Debug flags: %s", debug_str.c_str());
    }
    if (vlan_mode) {
        if (rx_any) {
            TETHER_LOGI(TAG, "VLAN mode: RX=any (undefined target), TX=%s",
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        } else if (rx_range) {
            TETHER_LOGI(TAG, "VLAN mode: RX=%u-%u, TX=%s",
                        rx_range->start, rx_range->end,
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        } else {
            TETHER_LOGI(TAG, "VLAN mode: RX=untagged, TX=%s",
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        }
    }

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
                           magic_enum::enum_name(err).data());
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
    EtherCAT::Master master;

    // ---- Optional VLAN router ----
    std::unique_ptr<EtherCAT::VLANRouter> router;
    if (vlan_mode) {
        router = std::make_unique<EtherCAT::VLANRouter>();
        router->setBackend(ni_ptr.get());
        if (rx_any) {
            router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                tx_vlan, true);
        } else if (rx_range) {
            router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                *rx_range, tx_vlan);
        } else {
            router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                std::nullopt, tx_vlan);
        }
    }

    // Route RX frames
    if (router) {
        eth->setRxCallback([&router](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            router->processRxFrame(frame, len);
        }, nullptr);
    } else {
        eth->setRxCallback([&master](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            master.handleRxFrame(frame, len);
        }, nullptr);
    }

    // ---- Poll thread ----
    std::atomic<bool> poll_running{true};
    std::thread poll_thread([&]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (poll_running.load()) eth->poll(1);
    });

    // ---- Start master + discover ----
    if (router) {
        EtherCAT::NetworkInterface* master_iface = rx_any
            ? router->undefinedNetworkInterface()
            : router->networkInterfaceFor(&master);
        if (!master_iface) {
            TETHER_LOGE(TAG, "Failed to obtain per-master NetworkInterface from VLAN router");
            return 5;
        }
        master.start(*master_iface, src_mac);
    } else {
        master.start(*EtherCAT::Raw::network_interface(), src_mac);
    }

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    // ---- Report ----
    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered %u slave(s) ===", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    // ---- Debug output ----
    if (debug_flags.count("sii-derivation") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== SII Mailbox Derivation Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::SII::debugSIIMailboxDerivation(master, i, TAG);
        }
    }

    if (debug_flags.count("mailbox-configuration") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== Mailbox Hardware Configuration Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::debugMailboxConfiguration(master, i, TAG);
        }
    }

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
