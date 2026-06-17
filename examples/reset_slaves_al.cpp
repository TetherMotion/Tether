/**
 * @file reset_slaves_al.cpp
 * @brief Reset EtherCAT slave(s) to a target ESM state via AL Control
 *
 * Scans the bus, then for each selected slave performs a bounded AL reset
 * loop: write target state with error-ack clear, then write target state
 * with error-ack set, until the slave reports the target state with no error
 * bit, or the iteration limit is reached.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./reset_slaves_al                  # reset all slaves to INIT
 *   ./reset_slaves_al -s 0             # reset only slave 0
 *   ./reset_slaves_al -s 1 -s 2        # reset slaves 1 and 2
 *   ./reset_slaves_al -s 1,2,3         # reset slaves 1, 2 and 3
 *   ./reset_slaves_al --target-state 2 # reset to PRE_OP
 *   ./reset_slaves_al -n 20 --sleep 10 # 20 iterations, 10 ms sleep
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <magic_enum/magic_enum.hpp>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/platform/EspCompat.hpp"

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

static const char* TAG = "reset_slaves_al";

// ============================================================================
// Parse repeatable -s flags that accept single index or comma list
// ============================================================================

static std::vector<uint16_t> parseSlaveIndices(const std::vector<std::string>& args) {
    std::vector<uint16_t> indices;
    for (const auto& arg : args) {
        std::stringstream ss(arg);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (token.empty()) continue;
            try {
                int v = std::stoi(token);
                if (v < 0 || v > 65535) {
                    std::cerr << "Invalid slave index: " << token << " (must be 0-65535)\n";
                    std::exit(1);
                }
                indices.push_back(static_cast<uint16_t>(v));
            } catch (...) {
                std::cerr << "Invalid slave index: " << token << "\n";
                std::exit(1);
            }
        }
    }
    return indices;
}

// ============================================================================
// Host-side helpers
// ============================================================================

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("reset_slaves_al");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Known flags: sii-derivation, mailbox-configuration, al-state, tx-ethercat-packets, rx-ethercat-packets, rx-pdo, tx-pdo, sii-eeprom");
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID, range, or 'any'");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID");
    program.add_argument("-s", "--slave")
        .append()
        .help("Slave index to reset. Repeatable. Accepts comma list (e.g. -s 1,2,3). Default: all discovered slaves.");
    program.add_argument("-n", "--max-iterations")
        .default_value(std::string("50"))
        .help("Maximum reset iterations per slave (default 50)");
    program.add_argument("--sleep")
        .default_value(std::string("50"))
        .help("Sleep in ms between reset iterations (default 50)");
    program.add_argument("--target-state")
        .default_value(std::string("1"))
        .help("Target ESM state as decimal or hex (default 1 = INIT). Prefix with 0x for hex.");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface       = program.get<std::string>("--interface");
    std::string debug_str   = program.get<std::string>("--debug");
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");

    int max_iterations = 50;
    try {
        max_iterations = std::stoi(program.get<std::string>("--max-iterations"));
        if (max_iterations < 1 || max_iterations > 10000) {
            std::cerr << "--max-iterations must be 1-10000\n";
            return 1;
        }
    } catch (...) {
        std::cerr << "Invalid --max-iterations value\n";
        return 1;
    }

    int sleep_ms = 50;
    try {
        sleep_ms = std::stoi(program.get<std::string>("--sleep"));
        if (sleep_ms < 0 || sleep_ms > 60000) {
            std::cerr << "--sleep must be 0-60000\n";
            return 1;
        }
    } catch (...) {
        std::cerr << "Invalid --sleep value\n";
        return 1;
    }

    uint8_t target_state = 0x01;
    try {
        std::string ts_str = program.get<std::string>("--target-state");
        int ts_val = 0;
        if (ts_str.size() > 2 && (ts_str.substr(0, 2) == "0x" || ts_str.substr(0, 2) == "0X")) {
            ts_val = std::stoi(ts_str.substr(2), nullptr, 16);
        } else {
            ts_val = std::stoi(ts_str);
        }
        if (ts_val < 0 || ts_val > 15) {
            std::cerr << "--target-state must be 0-15\n";
            return 1;
        }
        target_state = static_cast<uint8_t>(ts_val);
    } catch (...) {
        std::cerr << "Invalid --target-state value\n";
        return 1;
    }

    std::vector<uint16_t> selected_slaves;
    if (program.is_used("--slave")) {
        selected_slaves = parseSlaveIndices(program.get<std::vector<std::string>>("--slave"));
    }

    // Known debug flags
    const std::set<std::string> known_debug_flags = {
        "sii-derivation",
        "mailbox-configuration",
        "al-state",
        "tx-ethercat-packets",
        "rx-ethercat-packets",
        "rx-pdo",
        "tx-pdo",
        "sii-eeprom",
        "coe-reads",
        "coe-writes",
        "coe-rx-packets",
        "coe-tx-packets"
    };

    // Parse debug flags
    std::set<std::string> debug_flags;
    std::set<std::string> unknown_flags;
    if (!debug_str.empty()) {
        std::stringstream ss(debug_str);
        std::string flag;
        while (std::getline(ss, flag, ',')) {
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

    if (debug_flags.count("al-state")) {
        EtherCAT::enableStateMachineDebug(true);
        TETHER_LOGI(TAG, "EtherCAT state machine debug logging enabled");
    }
    if (debug_flags.count("tx-ethercat-packets")) {
        EtherCAT::enableTxPacketDebug(true);
        TETHER_LOGI(TAG, "TX EtherCAT packet debug logging enabled");
    }
    if (debug_flags.count("rx-ethercat-packets")) {
        EtherCAT::enableRxPacketDebug(true);
        TETHER_LOGI(TAG, "RX EtherCAT packet debug logging enabled");
    }
    if (debug_flags.count("rx-pdo")) {
        EtherCAT::enableRxPDODebug(true);
        TETHER_LOGI(TAG, "RxPDO debug logging enabled");
    }
    if (debug_flags.count("tx-pdo")) {
        EtherCAT::enableTxPDODebug(true);
        TETHER_LOGI(TAG, "TxPDO debug logging enabled");
    }

    // Enable SII/EEPROM debug if requested
    if (debug_flags.count("sii-eeprom")) {
        EtherCAT::enableSIIEEPROMDebug(true);
        TETHER_LOGI(TAG, "SII/EEPROM debug logging enabled");
    }

    // Enable CoE read debug if requested
    if (debug_flags.count("coe-reads")) {
        EtherCAT::enableCoEReadsDebug(true);
        TETHER_LOGI(TAG, "CoE read debug logging enabled");
    }

    // Enable CoE write debug if requested
    if (debug_flags.count("coe-writes")) {
        EtherCAT::enableCoEWritesDebug(true);
        TETHER_LOGI(TAG, "CoE write debug logging enabled");
    }

    // Enable CoE RX packet debug if requested
    if (debug_flags.count("coe-rx-packets")) {
        EtherCAT::enableCoERxPacketsDebug(true);
        TETHER_LOGI(TAG, "CoE RX packet debug logging enabled");
    }

    // Enable CoE TX packet debug if requested
    if (debug_flags.count("coe-tx-packets")) {
        EtherCAT::enableCoETxPacketsDebug(true);
        TETHER_LOGI(TAG, "CoE TX packet debug logging enabled");
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

    TETHER_LOGI(TAG, "reset_slaves_al (host) — interface: %s", iface.c_str());
    TETHER_LOGI(TAG, "Parameters: target-state=0x%02X, max-iterations=%d, sleep=%d ms",
                target_state, max_iterations, sleep_ms);
    if (!selected_slaves.empty()) {
        std::string list;
        for (size_t i = 0; i < selected_slaves.size(); ++i) {
            if (i) list += ",";
            list += std::to_string(selected_slaves[i]);
        }
        TETHER_LOGI(TAG, "Selected slaves: %s", list.c_str());
    } else {
        TETHER_LOGI(TAG, "Selected slaves: all");
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
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered %u slave(s) ===", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (slaves == 0) {
        TETHER_LOGW(TAG, "No slaves found — check wiring, power, and interface name");
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    // Resolve target slave list
    std::vector<uint16_t> targets;
    if (selected_slaves.empty()) {
        for (uint16_t i = 0; i < slaves; ++i) targets.push_back(i);
    } else {
        for (uint16_t si : selected_slaves) {
            if (si >= slaves) {
                TETHER_LOGW(TAG, "Slave %u not discovered (only %u slave(s) on bus), skipping", si, slaves);
                continue;
            }
            targets.push_back(si);
        }
    }

    if (targets.empty()) {
        TETHER_LOGW(TAG, "No valid slaves selected for reset");
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    // ---- AL Reset loop ----
    EtherCAT::ALResetController ctrl(master);
    ctrl.setProgressCallback(
        [](uint16_t si, int iter, int max_iter, uint16_t al, uint16_t code, bool reached) {
            if (!reached) {
                const char* state_name = EtherCAT::al_status_get_state_name(al);
                const bool has_err = EtherCAT::al_status_has_error(al);
                TETHER_LOGI(TAG, "  Slave %u iter %d/%d: AL_STATUS=0x%04X (state=%s, error=%s)",
                            si, iter, max_iter, al,
                            state_name, has_err ? "true" : "false");
                if (code != 0) {
                    TETHER_LOGI(TAG, "    AL_STATUS_CODE=0x%04X (%s)",
                                code, EtherCAT::getALStatusCodeName(code));
                }
            }
        });

    uint16_t success_count = 0;
    uint16_t fail_count    = 0;

    for (uint16_t si : targets) {
        TETHER_LOGI(TAG, "Resetting slave %u to state 0x%02X (max %d iterations, %d ms sleep) ...",
                    si, target_state, max_iterations, sleep_ms);

        auto result = ctrl.resetSlave(si, target_state, max_iterations, sleep_ms);

        if (result.success) {
            TETHER_LOGI(TAG, "  Slave %u => target state 0x%02X OK (%s)",
                        si, target_state, result.message.c_str());
            ++success_count;
        } else {
            TETHER_LOGE(TAG, "  Slave %u => target state 0x%02X FAILED (%s)",
                        si, target_state, result.message.c_str());
            ++fail_count;
        }
    }

    // ---- Summary ----
    TETHER_LOGI(TAG, "=== Reset Summary: %u succeeded, %u failed ===", success_count, fail_count);

    // ---- Cleanup ----
    master.stop();
    poll_running.store(false);
    poll_thread.join();
    eth->shutdown();

    if (fail_count == 0) {
        return 0; // all ok
    } else if (success_count == 0) {
        return 7; // all failed
    } else {
        return 5; // some failed
    }
}
