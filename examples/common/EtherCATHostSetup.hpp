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
//      registration — the iface/src MAC are passed directly to Master::start)
//   3. Optional VLAN router + RX callback registration
//   4. Poll thread with best-effort realtime scheduling
//   5. Master.start() (via session NetworkInterface or VLAN router)
//   6. Graceful shutdown (stop master, stop poll, join, shutdown eth)
// ============================================================================

struct HostEtherNetSession {
    std::unique_ptr<EtherCAT::HAL::IEthernet> eth;
    std::unique_ptr<EtherCAT::NetworkInterface> ni;
    std::unique_ptr<EtherCAT::VLANRouter> router;
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

/// If @p vlan.enabled is true, create a VLANRouter, wire it to the master,
/// and install an RX callback that routes through the router.
/// If disabled, install a direct RX callback to the master.
bool setupVlanAndRxCallback(HostEtherNetSession& session,
                            EtherCAT::Master& master,
                            const VlanConfig& vlan,
                            const char* tag);

/// Start the EtherCAT master.  If VLAN is enabled the master is started via
/// the per-master NetworkInterface from the router, otherwise via the
/// session's own NetworkInterface.
bool startHostMaster(HostEtherNetSession& session,
                     EtherCAT::Master& master,
                     const VlanConfig& vlan,
                     const char* tag);

/// Convenience: perform full master startup (discover + summary logging).
/// Returns false if no slaves are found or the master fails to start.
bool startHostMasterAndDiscover(HostEtherNetSession& session,
                                EtherCAT::Master& master,
                                const VlanConfig& vlan,
                                const char* tag);

} // namespace Tether::Examples
