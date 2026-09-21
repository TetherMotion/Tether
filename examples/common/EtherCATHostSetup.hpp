#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/hal/IEthernet.hpp"

#include "ExampleHelpers.hpp"

namespace Tether::Examples {

// ============================================================================
// Host Ethernet + EtherCAT Master session helpers
//
// These functions encapsulate the repetitive boilerplate found in every
// Linux host example:
//   1. createDefaultEthernet() + init() + link-check + getMacAddress()
//   2. NetworkInterface wrapper held in the session (no process-global
//      registration  -  the iface/src MAC are passed directly to Master::start)
//   3. Optional VLAN router + RX callback registration
//   4. Poll thread with best-effort realtime scheduling
//   5. Master.start() (via session NetworkInterface or VLAN router)
//   6. Graceful shutdown (stop master, stop poll, join, shutdown eth)
// ============================================================================

struct HostEtherNetSession {
    std::unique_ptr<EtherCAT::HAL::IEthernet> eth;
    std::unique_ptr<EtherCAT::NetworkInterface> ni;
    std::unique_ptr<EtherCAT::VLANRouter> router;
    /// Interface the master runs on — set by setupEncapsulation()
    /// (router's per-master view under VLAN, `ni.get()` otherwise).
    EtherCAT::NetworkInterface* masterInterface = nullptr;
    std::atomic<bool> pollRunning{false};
    std::thread pollThread;
    uint8_t srcMac[6]{};

    ~HostEtherNetSession();
};

/// Initialise the Ethernet HAL, verify link, read MAC, create NetworkInterface
/// and store it in the session.  Returns false on any error (already
/// logged via TETHER_LOGE).
bool initHostEthernet(HostEtherNetSession& session,
                      const std::string& interfaceName,
                      const char* tag);

/// Start a background thread that polls the Ethernet HAL.
void startHostPollThread(HostEtherNetSession& session, const char* tag);

/// Stop the poll thread and shut down the Ethernet HAL.
void shutdownHostEthernet(HostEtherNetSession& session);

/// One-call encapsulation setup: attaches the kernel cBPF ingress filter,
/// enables EtherCAT-over-UDP on the master when requested (fails if the
/// build lacks TETHER_ENABLE_UDP_ENCAPSULATION), wires up VLAN routing and
/// the RX callback, and stores the interface the master should start() on
/// in `session.masterInterface`.  See the lower-level setupEncapsulation()
/// in ExampleHelpers.hpp for the full behaviour.
bool setupEncapsulation(HostEtherNetSession& session,
                        EtherCAT::Master& master,
                        const EncapsulationConfig& encapsulation,
                        const char* tag);

/// Start the EtherCAT master on the interface selected by
/// setupEncapsulation() (per-master VLAN-router interface under VLAN,
/// otherwise the session's own NetworkInterface).
bool startHostMaster(HostEtherNetSession& session,
                     EtherCAT::Master& master,
                     const char* tag);

/// Convenience: perform full master startup (discover + summary logging).
/// Returns false if no slaves are found or the master fails to start.
bool startHostMasterAndDiscover(HostEtherNetSession& session,
                                EtherCAT::Master& master,
                                const char* tag);

} // namespace Tether::Examples
