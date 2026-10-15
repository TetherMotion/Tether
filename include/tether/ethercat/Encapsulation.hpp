#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/EtherCATCommandTypes.hpp"
#include "tether/ethercat/VLANRouter.hpp"

namespace EtherCAT {

class Master;
struct NetworkInterface;
namespace HAL { class IEthernet; }

// ============================================================================
// Encapsulation helpers (802.1Q VLAN + EtherCAT-over-UDP)
// ============================================================================

/**
 * @brief Frame encapsulation selected by the `--encapsulation` spec grammar.
 *
 * Grammar (comma-separated tokens):
 *   raw | none                      — plain EtherCAT (default)
 *   vlan:<vid>                      — RX filter + TX tag on <vid> (1-4095)
 *   vlan:<lo>-<hi>                  — RX VID range; TX tag = <lo>
 *   vlan:any                        — RX catch-all for tagged frames
 *   vlantx:<vid|off>                — override/disable TX tag
 *   udp[:<port>]                    — EtherCAT-over-UDP (ETG.1000.3, port 34980)
 *
 * Examples: "vlan:1999", "udp", "vlan:100,udp", "vlan:100-200,vlantx:off".
 */
struct EncapsulationConfig {
    /// VLAN mode is active (router needed) when any of these are set.
    std::optional<uint16_t> txVlan;
    bool rxAny = false;
    std::optional<VLANRouter::VLANRange> rxRange;

    /// EtherCAT-over-UDP encapsulation (ETG.1000.3).
    bool     udp     = false;
    uint16_t udpPort = kEtherCATUdpPort;

    /// VLAN routing/tagging is in use.
    bool vlanActive() const {
        return rxAny || rxRange.has_value() || txVlan.has_value();
    }
    /// Any encapsulation (VLAN or UDP) is in use.
    bool enabled() const { return vlanActive() || udp; }
};

/// Parse an encapsulation spec string (grammar documented on
/// EncapsulationConfig) into @p out.
/// Returns false and prints to stderr on invalid input.
bool parseEncapsulationSpec(const std::string& spec,
                            EncapsulationConfig& out,
                            const char* tag);

/// Log the encapsulation configuration via TETHER_LOGI.
void logEncapsulationConfig(const EncapsulationConfig& config,
                            const char* tag);

/// Build the kernel socket-filter program matching @p config — untagged
/// EtherCAT (+any-VID tagged) for raw mode, VID-restricted for vlan mode,
/// plus EtherCAT-over-UDP when enabled.  Empty program = attach nothing.
/// @p delivery prunes the unreachable VLAN leg when probeVlanTagDelivery()
/// determined how this NIC presents tags (Auto = emit both legs).
std::vector<CBPFInsn> buildEncapsulationBpfProgram(
    const EncapsulationConfig& config,
    VlanDeliveryHint delivery = VlanDeliveryHint::Auto);

/// Attach the encapsulation filter to the Ethernet device's native socket
/// (SO_ATTACH_FILTER on Linux; soft-fails with a warning).
void attachEncapsulationBpfFilter(
    HAL::IEthernet& eth,
    const EncapsulationConfig& config,
    const char* tag,
    VlanDeliveryHint delivery = VlanDeliveryHint::Auto);

/**
 * @brief One-call encapsulation setup for a host-side EtherCAT interface.
 *
 * Applies @p encapsulation end to end:
 *   1. attaches a kernel cBPF ingress filter to `eth`'s native socket —
 *      rejects non-EtherCAT traffic (and, in VLAN mode, frames outside the
 *      configured VID) before userspace sees them;
 *   2. enables EtherCAT-over-UDP on `master` when requested — fails if the
 *      build lacks TETHER_ENABLE_UDP_ENCAPSULATION, and disables the HAL's
 *      software EtherType filter so IPv4 frames reach the master;
 *   3. when VLAN routing is active, creates a VLANRouter (stored into
 *      `routerStorage`), registers `master` on it, and installs an RX
 *      callback on `eth` that routes tagged frames by VID; otherwise
 *      installs a direct RX callback to `master`.
 *
 * @param eth            initialized Ethernet device
 * @param backend        NetworkInterface wrapping `eth` (its send lambda is
 *                       used for TX; the receive fast-path is cleared when
 *                       VLAN routing is active)
 * @param master         master to wire up (RX callback + UDP config)
 * @param routerStorage  receives the router when VLAN routing is active;
 *                       caller keeps it alive for the session duration
 * @return the NetworkInterface the master should start() on — the router's
 *         per-master view under VLAN, or `&backend` otherwise; nullptr on
 *         failure (e.g. UDP encapsulation not compiled in).
 */
NetworkInterface* setupEncapsulation(
    HAL::IEthernet& eth,
    NetworkInterface& backend,
    Master& master,
    std::unique_ptr<VLANRouter>& routerStorage,
    const EncapsulationConfig& encapsulation,
    const char* tag);

} // namespace EtherCAT
