/**
 * @file slave_read_identity.cpp
 * @brief Read slave identity object (0x1018) via CoE/SDO
 *
 * Discovers an EtherCAT slave, configures the mailbox from SII,
 * transitions to PRE-OP, and reads all subindexes of the identity
 * object (0x1018), printing the results.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./slave_read_identity              # uses eth0, slave 0
 *   ./slave_read_identity -i enp3s0  # specify interface
 *   ./slave_read_identity -s 1         # specify slave index
 */

#include <atomic>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <memory>
#include <string>
#include <thread>
#include <magic_enum/magic_enum.hpp>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include <argparse/argparse.hpp>

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

static const char* TAG = "slave_read_identity";

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("slave_read_identity");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("-s", "--slave")
        .default_value(0)
        .help("Slave index to query (0-based)")
        .scan<'i', int>();
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID, range, or 'any'");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    int slave_idx = program.get<int>("--slave");
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");

    if (slave_idx < 0 || slave_idx > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
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
                    std::cerr << "--tx-vlan must be in range 1-4095\n";
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
                size_t dash = rx_vlan_str.find('-');
                try {
                    if (dash == std::string::npos) {
                        int v = std::stoi(rx_vlan_str);
                        if (v < 1 || v > 4095) {
                            std::cerr << "--rx-vlan must be in range 1-4095\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(v), static_cast<uint16_t>(v)};
                    } else {
                        int start = std::stoi(rx_vlan_str.substr(0, dash));
                        int end   = std::stoi(rx_vlan_str.substr(dash + 1));
                        if (start < 1 || end > 4095 || start > end) {
                            std::cerr << "--rx-vlan range must be 1-4095 with start <= end\n";
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

    TETHER_LOGI(TAG, "slave_read_identity — interface: %s, target slave: %d",
                iface.c_str(), slave_idx);

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
            poll_running = false;
            poll_thread.join();
            eth->shutdown();
            return 5;
        }
        master.start(*master_iface, src_mac);
    } else {
        master.start(*EtherCAT::Raw::network_interface(), src_mac);
    }

    if (!master.discoverSlaves()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);

    if (static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index %d out of range (only %u slave(s) found)",
                    slave_idx, slaves);
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 5;
    }

    auto& sl = master.slave(static_cast<uint16_t>(slave_idx));

    // ---- Configure mailbox from SII ----
    TETHER_LOGI(TAG, "Configuring mailbox via SII for slave %d...", slave_idx);
    auto mb_err = sl.configureMailbox();
    if (mb_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Mailbox configuration failed: %s",
                    EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    // ---- Transition to PRE-OP ----
    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: %s",
                    EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 8;
    }
    TETHER_LOGI(TAG, "Slave %d is in PRE-OP", slave_idx);

    // ---- Read Identity Object 0x1018 ----
    constexpr uint16_t kIdentityIndex = 0x1018;

    // First read subindex 0 to know how many entries exist
    uint8_t num_entries = 0;
    auto err = sl.sdoReadU8(kIdentityIndex, 0, num_entries);
    if (err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to read subindex 0 of 0x%04X", kIdentityIndex);
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 9;
    }

    std::cout << "\n=== Identity Object (0x" << std::hex << kIdentityIndex << std::dec << ") ===\n";
    std::cout << "Number of entries (subindex 0): " << static_cast<int>(num_entries) << "\n\n";

    bool identity_ok = true;

    for (uint8_t sub = 1; sub <= num_entries; ++sub) {
        uint32_t value = 0;
        err = sl.sdoReadU32(kIdentityIndex, sub, value);
        if (err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to read subindex %u of 0x%04X", sub, kIdentityIndex);
            identity_ok = false;
            std::cout << "Subindex " << static_cast<int>(sub) << ": [READ FAILED]\n";
            continue;
        }

        const char* label = nullptr;
        switch (sub) {
            case 1: label = "Vendor ID";       break;
            case 2: label = "Product Code";      break;
            case 3: label = "Revision Number";   break;
            case 4: label = "Serial Number";     break;
            default: label = "Reserved";         break;
        }

        std::cout << "Subindex " << static_cast<int>(sub) << " — " << label << ":\n";
        std::cout << "  Decimal: " << value << "\n";
        std::cout << "  Hex:     0x" << std::hex << std::setw(8) << std::setfill('0')
                  << value << std::dec << "\n\n";
    }
    std::cout.flush();

    // ---- Exit cleanly ----
    master.stop();
    poll_running = false;
    poll_thread.join();
    eth->shutdown();

    return identity_ok ? 0 : 10;
}
