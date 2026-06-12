/**
 * @file ethercat_dump_sii.cpp
 * @brief Example: dump parsed SII for a slave index (host + embedded)
 *
 * Usage (host build):
 *   ./ethercat_dump_sii -i eth0                      # dump slave 0
 *   ./ethercat_dump_sii -s 1 -i eth0                  # dump slave 1 (or use --interface)
 */

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include "tether/ethercat/EtherCATDiagnostics.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/platform/EspCompat.hpp"

// Forward-declare a small subset of Raw transport helpers used by the host example
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
    const uint8_t* get_src_mac();
}
}

#ifdef UNIT_TEST_HOST
#include <argparse/argparse.hpp>
#include "tether/hal/IEthernet.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#endif

static const char* TAG = "ethercat_dump_sii";

static int dumpSiiForSlave(EtherCAT::EtherCATMaster& master, uint16_t slave_idx) {
    uint16_t slaves = master.getDiscoveredSlaveCount();
    if (slave_idx >= slaves) {
        TETHER_LOGE(TAG, "Requested slave index %u >= discovered slaves (%u)", slave_idx, slaves);
        return 2;
    }

    return EtherCAT::Diagnostics::logParsedSlaveSII(master, slave_idx, TAG) ? 0 : 3;
}

#ifndef UNIT_TEST_HOST
extern "C" void ethercat_dump_sii_main(const EtherCAT::NetworkInterface* iface,
                                         const uint8_t src_mac[6]) {
    TETHER_LOGI(TAG, "ethercat_dump_sii (embedded)");

    EtherCAT::EtherCATMaster::Config cfg;
    EtherCAT::EtherCATMaster master(cfg);

    if (!iface || !src_mac) { TETHER_LOGE(TAG, "No network interface registered"); return; }
    master.start(*iface, src_mac);

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
        return;
    }

    (void)dumpSiiForSlave(master, 0);
}

#else // UNIT_TEST_HOST

int main(int argc, char** argv) {
    argparse::ArgumentParser program("ethercat_dump_sii");

    program.add_argument("-i", "--interface").default_value(std::string("eth0"))
        .help("Network interface name for host builds (e.g. eth0)");

    program.add_argument("-s","--slave-index").default_value(0).scan<'i', int>()
        .help("Slave index to dump (default: 0)");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    int slave_idx = program.get<int>("--slave-index");

    TETHER_LOGI(TAG, "ethercat_dump_sii (host)\nNetwork interface: %s", iface.c_str());

    auto eth = EtherCAT::HAL::createDefaultEthernet();
    if (!eth) { TETHER_LOGE(TAG, "No Ethernet HAL available"); return 1; }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = iface.c_str();
    cfg.promiscuous = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    {
        auto err = eth->init(cfg);
        if (err != EtherCAT::HAL::Error::OK) {
            if (err == EtherCAT::HAL::Error::InterfaceNotFound) {
                TETHER_LOGE(TAG, "Network interface '%s' not found — verify interface name (run: `ip link`)", iface.c_str());
            } else if (err == EtherCAT::HAL::Error::PermissionDenied) {
                TETHER_LOGE(TAG, "Permission denied while opening interface '%s' — raw sockets require root or CAP_NET_RAW", iface.c_str());
            } else {
                TETHER_LOGE(TAG, "Failed to init Ethernet interface '%s' (%s)", iface.c_str(), EtherCAT::HAL::errorToString(err));
            }
            return 2;
        }

        EtherCAT::HAL::LinkStatus ls = eth->getLinkStatus();
        if (!ls.up) {
            TETHER_LOGE(TAG, "Network interface '%s' link is DOWN — check cable/driver", iface.c_str());
            return 6;
        }
    }

    EtherCAT::HAL::MacAddress mac;
    if (eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to read MAC address");
        return 3;
    }

    uint8_t src_mac[6]; std::memcpy(src_mac, mac.bytes, 6);

    auto ni_ptr = std::make_unique<EtherCAT::NetworkInterface>();
    ni_ptr->send = [eth = eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };

    EtherCAT::Raw::set_network_interface(ni_ptr.get());
    EtherCAT::Raw::set_src_mac(src_mac);

    EtherCAT::EtherCATMaster::Config mcfg;
    mcfg.enable_mailbox_fallback = true;
    EtherCAT::EtherCATMaster master(mcfg);

    // Route RX frames directly to master — avoids the fragile findByNetworkInterface lookup
    eth->setRxCallback([&master](const uint8_t* frame, size_t len, const EtherCAT::HAL::RxFrameInfo& info, void*){
        (void)info; master.handleRxFrame(frame, len);
    }, nullptr);

    std::atomic<bool> poll_running{true};
    std::thread poll_thread([&](){
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (poll_running.load()) eth->poll(1);
    });

    master.start(*EtherCAT::Raw::network_interface(), src_mac);

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    if (slaves == 0) {
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 5;
    }

    int rc = dumpSiiForSlave(master, static_cast<uint16_t>(slave_idx));

    poll_running.store(false);
    poll_thread.join();
    eth->shutdown();

    return rc;
}

#endif // UNIT_TEST_HOST
