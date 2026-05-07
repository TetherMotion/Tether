#pragma once

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

#ifdef UNIT_TEST_HOST
#include "tether/hal/IEthernet.hpp"
#endif

namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

namespace Tether::Examples {

struct SingleDriveExampleConfig {
    EtherCAT::DS402Master::DriveConfiguration drive;
    EtherCAT::DC::DCConfig dc_config{EtherCAT::DC::DCConfig::defaults()};
    uint32_t discovery_timeout_ms{2000};
    uint32_t enable_timeout_ms{5000};
};

#ifdef UNIT_TEST_HOST
struct HostMasterSession {
    std::unique_ptr<EtherCAT::HAL::IEthernet> ethernet;
    std::unique_ptr<EtherCAT::NetworkInterface> network_interface;
    std::atomic<bool> poll_running{false};
    std::thread poll_thread;
    uint8_t src_mac[6]{};
};

inline bool startHostMasterSession(const std::string& interface_name,
                                   EtherCAT::DS402Master& master,
                                   HostMasterSession& session,
                                   const char* tag)
{
    session.ethernet = EtherCAT::HAL::createDefaultEthernet();
    if (!session.ethernet) {
        TETHER_LOGE(tag, "No Ethernet HAL available");
        return false;
    }

    EtherCAT::HAL::EthernetConfig config;
    config.interfaceName = interface_name.c_str();
    config.promiscuous = true;
    config.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    const auto init_result = session.ethernet->init(config);
    if (init_result != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(tag, "Failed to init '%s' (%s)", interface_name.c_str(),
                    EtherCAT::HAL::errorToString(init_result));
        session.ethernet.reset();
        return false;
    }

    const auto link_status = session.ethernet->getLinkStatus();
    if (!link_status.up) {
        TETHER_LOGE(tag, "Link DOWN on '%s'", interface_name.c_str());
        session.ethernet->shutdown();
        session.ethernet.reset();
        return false;
    }

    EtherCAT::HAL::MacAddress mac;
    if (session.ethernet->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(tag, "Failed to read MAC address");
        session.ethernet->shutdown();
        session.ethernet.reset();
        return false;
    }
    std::memcpy(session.src_mac, mac.bytes, sizeof(session.src_mac));

    session.network_interface = std::make_unique<EtherCAT::NetworkInterface>();
    session.network_interface->send = [eth = session.ethernet.get()](const uint8_t* data, size_t length) {
        return eth->transmit(data, length) == EtherCAT::HAL::Error::OK;
    };

    EtherCAT::Raw::set_network_interface(session.network_interface.get());
    EtherCAT::Raw::set_src_mac(session.src_mac);

    session.ethernet->setRxCallback(
        [&master](const uint8_t* frame, size_t len, const EtherCAT::HAL::RxFrameInfo&, void*) {
            master.ethercatMaster().handleRxFrame(frame, len);
        },
        nullptr);

    session.poll_running.store(true);
    session.poll_thread = std::thread([&session, tag]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(tag, "poll_thread: realtime scheduling unavailable");
        }
        while (session.poll_running.load()) {
            session.ethernet->poll(1);
        }
    });

    master.start(*EtherCAT::Raw::network_interface(), session.src_mac);
    return true;
}

inline void stopHostMasterSession(EtherCAT::DS402Master& master, HostMasterSession& session)
{
    master.stopDistributedClocks();
    master.stop();

    session.poll_running.store(false);
    if (session.poll_thread.joinable()) {
        session.poll_thread.join();
    }
    if (session.ethernet) {
        session.ethernet->shutdown();
    }
}
#endif

inline bool configureAndEnableSingleDrive(EtherCAT::DS402Master& master,
                                          const SingleDriveExampleConfig& config,
                                          const char* tag)
{
    const uint16_t minimum_drive_count = static_cast<uint16_t>(config.drive.slave_index + 1);
    if (!master.waitForDriveCount(minimum_drive_count, config.discovery_timeout_ms)) {
        TETHER_LOGE(tag, "Timed out waiting for %u drive(s)", minimum_drive_count);
        return false;
    }

    if (!master.initializeDistributedClocks(config.dc_config)) {
        TETHER_LOGE(tag, "Failed to initialize distributed clocks");
        return false;
    }

    if (!master.startDistributedClocks()) {
        TETHER_LOGE(tag, "Failed to start distributed clocks");
        return false;
    }

    if (!master.configureDrive(config.drive)) {
        TETHER_LOGE(tag, "Failed to configure slave %u", config.drive.slave_index);
        master.stopDistributedClocks();
        return false;
    }

    if (!master.enableDrive(config.drive.slave_index, config.enable_timeout_ms)) {
        TETHER_LOGE(tag, "Failed to enable slave %u", config.drive.slave_index);
        master.stopDistributedClocks();
        return false;
    }

    return true;
}

inline void shutdownSingleDrive(EtherCAT::DS402Master& master, uint16_t slave_index)
{
    (void)master.disableDrive(slave_index);
    master.stopDistributedClocks();
}

} // namespace Tether::Examples