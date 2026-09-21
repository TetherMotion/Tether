#include "EtherCATHostSetup.hpp"

#include <cstring>
#include <iostream>

#include "tether/hal/NetworkInterfaceEnumerator.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"

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
    // Direct-receive fast path: lets the cyclic executive drain the socket
    // itself inside its bounded response wait (ppoll on socket + eventfd).
    // Only wired on the non-VLAN interface — VLAN-routed masters must not
    // bypass the VLANRouter.
    session.ni->native_handle = session.eth->nativeHandle();
    session.ni->receive = [eth = session.eth.get()](uint8_t* buf, size_t cap,
                                                  size_t* out) -> bool {
        int r = eth->recvFrame(buf, cap, nullptr);
        if (r > 0) { *out = static_cast<size_t>(r); return true; }
        return false;
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

bool setupEncapAndRxCallback(HostEtherNetSession& session,
                             EtherCAT::Master& master,
                             const EncapConfig& encap,
                             const char* tag) {
    // Kernel-side ingress filter: drops non-EtherCAT traffic (and, in VLAN
    // mode, everything outside the configured VID) before userspace sees it.
    attachEncapBpfFilter(*session.eth, encap, tag);

    if (encap.udp) {
        EtherCAT::UdpEncapsulationConfig uc;
        uc.enabled          = true;
        uc.destination_port = encap.udpPort;
        master.setUdpEncapsulation(uc);
        if (!master.isUdpEncapsulationEnabled()) {
            TETHER_LOGE(tag, "--encapsulation udp requires a build with "
                             "TETHER_ENABLE_UDP_ENCAPSULATION=ON");
            return false;
        }
        // The HAL's software EtherType filter (0x88A4) would drop IPv4/UDP
        // frames — disable it; the attached cBPF filter does the real work
        // and the master parser rejects anything stray regardless.
        session.eth->setEthertypeFilter(0);
    }

    if (encap.vlanActive()) {
        // VLAN-routed masters must not bypass the VLANRouter: drop the
        // direct-receive fast path so the cyclic executive waits on the
        // eventfd/slot while the poll thread keeps demuxing by VLAN tag.
        session.ni->receive = nullptr;
        session.ni->native_handle = nullptr;
        session.router = std::make_unique<EtherCAT::VLANRouter>();
        session.router->setBackend(session.ni.get());
        if (encap.rxAny) {
            session.router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                encap.txVlan, true);
        } else if (encap.rxRange) {
            session.router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                *encap.rxRange, encap.txVlan);
        } else {
            session.router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*) {}),
                std::nullopt, encap.txVlan);
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
                     const EncapConfig& encap,
                     const char* tag) {
    if (encap.vlanActive() && session.router) {
        EtherCAT::NetworkInterface* masterIface = encap.rxAny
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
                                const EncapConfig& encap,
                                const char* tag) {
    if (!startHostMaster(session, master, encap, tag)) {
        return false;
    }
    auto slaves = master.discovery().discover();
    if (slaves.empty()) {
        TETHER_LOGW(tag, "No slaves discovered");
    }
    uint16_t slave_count = master.getDiscoveredSlaveCount();
    TETHER_LOGI(tag, "Discovered {} slave(s)", slave_count);
    if (slave_count == 0) {
        TETHER_LOGE(tag, "No slaves found -- check wiring, power, and interface name");
        master.stop();
        shutdownHostEthernet(session);
        return false;
    }
    for (const auto& s : slaves) {
        const char* name = s.device_name ? s.device_name->c_str() : "Unknown";
        TETHER_LOGI(tag, "Slave {}: Vendor=0x{:08X} Product=0x{:08X} {}",
                    s.index,
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0,
                    name);
    }
    return true;
}

} // namespace Tether::Examples
