#include "EtherCATHostSetup.hpp"

#include <cstring>
#include <iostream>

#include "tether/hal/NetworkInterfaceEnumerator.hpp"
#include "tether/platform/Platform.hpp"

namespace Tether::Examples {

HostEtherNetSession::~HostEtherNetSession() {
    if (pollRunning.load()) {
        shutdownHostEthernet(*this);
    }
}

bool initHostEthernet(HostEtherNetSession& session,
                      const std::string& interfaceName,
                      const char* tag) {
    session.eth = EtherCAT::HAL::createDefaultEthernet();
    if (!session.eth) {
        TETHER_LOGE(tag, "No Ethernet HAL available");
        return false;
    }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = interfaceName.c_str();
    cfg.promiscuous = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    auto err = session.eth->init(cfg);
    if (err != EtherCAT::HAL::Error::OK) {
        if (err == EtherCAT::HAL::Error::InterfaceNotFound) {
            TETHER_LOGE(tag, "Interface '{}' not found", interfaceName.c_str());
            // List physical Ethernet interfaces to guide the user
            auto physIfaces = EtherCAT::HAL::getPhysicalEthernetInterfaces();
            if (!physIfaces.empty()) {
                std::string names;
                for (const auto& iface : physIfaces) {
                    if (!names.empty()) names += ", ";
                    names += iface.name;
                }
                TETHER_LOGI(tag, "Available physical Ethernet interfaces: {}",
                            names.c_str());
            } else {
                TETHER_LOGI(tag, "No physical Ethernet interfaces found on this system");
            }
        } else if (err == EtherCAT::HAL::Error::PermissionDenied) {
            logPermissionDeniedError(tag);
        } else {
            TETHER_LOGE(tag, "Critical error: Prerequisites for operation not fulfilled ({})",
                        "unknown"); // magic_enum may not be available here
        }
        session.eth.reset();
        return false;
    }

    auto ls = session.eth->getLinkStatus();
    if (!ls.up) {
        TETHER_LOGE(tag, "Link DOWN on '{}' -- check cable", interfaceName.c_str());
        session.eth->shutdown();
        session.eth.reset();
        return false;
    }

    EtherCAT::HAL::MacAddress mac;
    if (session.eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(tag, "Failed to read MAC address");
        session.eth->shutdown();
        session.eth.reset();
        return false;
    }
    std::memcpy(session.srcMac, mac.bytes, 6);

    session.ni = std::make_unique<EtherCAT::NetworkInterface>();
    session.ni->send = [eth = session.eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };

    return true;
}

void startHostPollThread(HostEtherNetSession& session, const char* tag) {
    session.pollRunning.store(true);
    session.pollThread = std::thread([&session, tag]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(tag, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (session.pollRunning.load()) {
            session.eth->poll(1);
        }
    });
}

void shutdownHostEthernet(HostEtherNetSession& session) {
    session.pollRunning.store(false);
    if (session.pollThread.joinable()) {
        session.pollThread.join();
    }
    if (session.eth) {
        session.eth->shutdown();
    }
}

bool setupVlanAndRxCallback(HostEtherNetSession& session,
                            EtherCAT::Master& master,
                            const VlanConfig& vlan,
                            const char* /*tag*/) {
    if (vlan.enabled) {
        session.router = std::make_unique<EtherCAT::VLANRouter>();
        session.router->setBackend(session.ni.get());
        if (vlan.rxAny) {
            session.router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                vlan.txVlan, true);
        } else if (vlan.rxRange) {
            session.router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                *vlan.rxRange, vlan.txVlan);
        } else {
            session.router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                std::nullopt, vlan.txVlan);
        }

        session.eth->setRxCallback(
            [&router = session.router](const uint8_t* frame, size_t len,
                                       const EtherCAT::HAL::RxFrameInfo&, void*) {
                router->processRxFrame(frame, len);
            },
            nullptr);
    } else {
        session.eth->setRxCallback(
            [&master](const uint8_t* frame, size_t len,
                      const EtherCAT::HAL::RxFrameInfo&, void*) {
                master.handleRxFrame(frame, len);
            },
            nullptr);
    }
    return true;
}

bool startHostMaster(HostEtherNetSession& session,
                     EtherCAT::Master& master,
                     const VlanConfig& vlan,
                     const char* tag) {
    if (vlan.enabled && session.router) {
        EtherCAT::NetworkInterface* masterIface = vlan.rxAny
            ? session.router->undefinedNetworkInterface()
            : session.router->networkInterfaceFor(&master);
        if (!masterIface) {
            TETHER_LOGE(tag, "Failed to obtain per-master NetworkInterface from VLAN router");
            return false;
        }
        master.start(*masterIface, session.srcMac);
    } else {
        master.start(*session.ni, session.srcMac);
    }
    return true;
}

bool startHostMasterAndDiscover(HostEtherNetSession& session,
                                EtherCAT::Master& master,
                                const VlanConfig& vlan,
                                const char* tag) {
    if (!startHostMaster(session, master, vlan, tag)) {
        return false;
    }
    if (!master.discoverSlaves()) {
        TETHER_LOGW(tag, "No slaves discovered");
    }
    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(tag, "Discovered {} slave(s)", slaves);
    if (slaves == 0) {
        TETHER_LOGE(tag, "No slaves found -- check wiring, power, and interface name");
        master.stop();
        shutdownHostEthernet(session);
        return false;
    }
    master.logDiscoveredSlavesSummary(tag);
    return true;
}

} // namespace Tether::Examples
