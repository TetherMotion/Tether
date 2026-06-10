/**
 * @file axia80_stream.cpp
 * @brief ATI Axia80 Force/Torque Sensor Streaming Example
 *
 * Discovers an Axia80 sensor on the EtherCAT bus, configures PDOs,
 * reads calibration data via SDO, and streams 6-DOF force/torque data.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./axia80_stream              # uses eth0, streams engineering units
 *   ./axia80_stream -i enp3s0  # specify interface
 *   ./axia80_stream --raw      # stream raw sensor counts
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sensors/Axia80.hpp"

#ifdef UNIT_TEST_HOST
#include <argparse/argparse.hpp>
#endif

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

static const char* TAG = "axia80_stream";
static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running.store(false);
}

#ifndef UNIT_TEST_HOST
// ----- ESP-IDF / embedded entry point -----
extern "C" void axia80_stream_main(const EtherCAT::NetworkInterface* iface,
                                     const uint8_t src_mac[6]) {
    TETHER_LOGI(TAG, "axia80_stream (embedded) — not yet implemented");
}

#else // UNIT_TEST_HOST — host/Linux build

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("axia80_stream");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("--raw")
        .default_value<bool>(false)
        .implicit_value(true)
        .help("Stream raw sensor counts instead of engineering units");
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-t", "--time")
        .scan<'g', double>()
        .default_value(0.0)
        .help("Stream duration in seconds (0 = infinite until Ctrl-C)");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Known flags: sii-derivation, mailbox-configuration, ethercat-statemachine, tx-ethercat-packets, rx-ethercat-packets, rx-pdo, tx-pdo");
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
    bool raw_mode = program.get<bool>("--raw");
    int slave_idx = program.get<int>("--slave");
    double duration_sec = program.get<double>("--time");
    std::string debug_str = program.get<std::string>("--debug");
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");

    // Known debug flags
    const std::set<std::string> known_debug_flags = {
        "sii-derivation",
        "mailbox-configuration",
        "ethercat-statemachine",
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
    if (debug_flags.count("ethercat-statemachine")) {
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

    TETHER_LOGI(TAG, "axia80_stream (host) — interface: %s, raw: %s, slave: %d",
                iface.c_str(), raw_mode ? "yes" : "no", slave_idx);
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

    // ---- Install signal handler ----
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

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
                TETHER_LOGE(TAG, "Interface '%s' not found", iface.c_str());
            else if (err == EtherCAT::HAL::Error::PermissionDenied)
                TETHER_LOGE(TAG, "Permission denied — run as root or with CAP_NET_RAW");
            else
                TETHER_LOGE(TAG, "Failed to init '%s' (%s)", iface.c_str(),
                            EtherCAT::HAL::errorToString(err));
            return 2;
        }

        auto ls = eth->getLinkStatus();
        if (!ls.up) {
            TETHER_LOGE(TAG, "Link DOWN on '%s'", iface.c_str());
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

    // ---- Optional VLAN router ----
    std::unique_ptr<EtherCAT::VLANRouter> router;
    if (vlan_mode) {
        router = std::make_unique<EtherCAT::VLANRouter>();
        router->setBackend(ni_ptr.get());
        if (rx_any) {
            router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
                tx_vlan, true);
        } else if (rx_range) {
            router->addMaster(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
                *rx_range, tx_vlan);
        } else {
            router->addMaster(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
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
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling");
        }
        while (poll_running.load()) { eth->poll(1); }
    });

    // ---- Start master ----
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

    // ---- Wait for discovery ----
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    uint16_t slaves = master.getDiscoveredSlaveCount();
    if (slaves == 0) {
        TETHER_LOGE(TAG, "No slaves found — check wiring and power");
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index %d out of range (max %u)", slave_idx, slaves - 1);
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 5;
    }

    // ---- Initialise Axia80 sensor ----
    EtherCAT::Sensors::Axia80Sensor sensor(master, static_cast<uint16_t>(slave_idx));

    if (!sensor.isAxia80Device()) {
        TETHER_LOGW(TAG, "Slave %d does not appear to be an Axia80 (wrong VID/PID)", slave_idx);
    }

    if (!sensor.init(Tether::Platform::LogLevel::Info)) {
        TETHER_LOGE(TAG, "Failed to initialise Axia80 sensor");
        master.stop();
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    // ---- Read calibration data via SDO ----
    EtherCAT::Sensors::Axia80::CalibrationData cal;
    bool have_cal = sensor.readCalibrationData(cal);
    if (have_cal) {
        TETHER_LOGI(TAG, "Calibration loaded: serial=%s, cpf=%u, cpt=%u",
                    cal.ft_serial, cal.counts_per_force, cal.counts_per_torque);
    } else {
        TETHER_LOGW(TAG, "Could not read calibration data — using raw counts");
        raw_mode = true;
    }

    // ---- Read product info ----
    EtherCAT::Sensors::Axia80::ProductDescription desc;
    if (sensor.readProductDescription(desc)) {
        TETHER_LOGI(TAG, "Device: %s (SN: %u)", desc.product_name, desc.product_serial_number);
    }

    // ---- Configure sensor ----
    sensor.setConfiguration(
        EtherCAT::Sensors::Axia80::FilterType::FILTER_3,
        EtherCAT::Sensors::Axia80::CalibrationSlot::SLOT_0,
        EtherCAT::Sensors::Axia80::SampleRate::RATE_1953_HZ);
    // Send one cycle to apply config
    master.pdo().exchangeAll();

    // ---- Set bias (tare) ----
    TETHER_LOGI(TAG, "Setting bias (tare)...");
    sensor.setBias();
    master.pdo().exchangeAll();
    // Clear bias bit so it doesn't stay set
    if (auto* pdo = sensor.rxPDO()) {
        pdo->control1 &= ~EtherCAT::Sensors::Axia80::CTRL_BIAS_BIT;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---- Stream data ----
    TETHER_LOGI(TAG, "Streaming %s data... Press Ctrl-C to stop",
                raw_mode ? "raw" : "engineering-unit");

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "      Fx        Fy        Fz        Tx        Ty        Tz    Status   Counter\n";
    std::cout << "-------------------------------------------------------------------------------\n";

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(static_cast<int>(duration_sec));
    uint64_t cycle_count = 0;

    while (g_running.load()) {
        if (duration_sec > 0 && std::chrono::steady_clock::now() >= end_time) {
            break;
        }

        if (!master.pdo().exchangeAll()) {
            TETHER_LOGW(TAG, "PDO exchange failed");
            continue;
        }

        auto* tx = sensor.txPDO();
        if (!tx) continue;

        double fx = 0, fy = 0, fz = 0, tx_val = 0, ty = 0, tz = 0;

        if (raw_mode || !have_cal || cal.counts_per_force == 0 || cal.counts_per_torque == 0) {
            fx = static_cast<double>(tx->fx);
            fy = static_cast<double>(tx->fy);
            fz = static_cast<double>(tx->fz);
            tx_val = static_cast<double>(tx->tx);
            ty = static_cast<double>(tx->ty);
            tz = static_cast<double>(tx->tz);
        } else {
            int32_t raw[6] = { tx->fx, tx->fy, tx->fz, tx->tx, tx->ty, tx->tz };
            double out[6] = {};
            EtherCAT::Sensors::Axia80Sensor::convertWrench(
                raw, out, cal.counts_per_force, cal.counts_per_torque);
            fx = out[0]; fy = out[1]; fz = out[2];
            tx_val = out[3]; ty = out[4]; tz = out[5];
        }

        std::cout << std::setw(10) << fx
                  << std::setw(10) << fy
                  << std::setw(10) << fz
                  << std::setw(10) << tx_val
                  << std::setw(10) << ty
                  << std::setw(10) << tz
                  << "  0x" << std::hex << std::setw(8) << std::setfill('0') << tx->status
                  << std::dec << std::setfill(' ')
                  << std::setw(10) << tx->counter
                  << "\n";

        if (EtherCAT::Sensors::Axia80Sensor::hasError(tx->status)) {
            std::cout << "  [ERROR] status=0x" << std::hex << tx->status << std::dec << "\n";
        }

        ++cycle_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    TETHER_LOGI(TAG, "Streamed %lu cycles", cycle_count);

    // ---- Cleanup ----
    master.stop();
    poll_running.store(false);
    poll_thread.join();
    eth->shutdown();

    return 0;
}

#endif // UNIT_TEST_HOST
