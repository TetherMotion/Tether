/**
 * @file NetworkInterfaceEnumerator.hpp
 * @brief Generic network interface enumeration and classification
 *
 * Provides a platform-agnostic API for listing all network interfaces
 * on the system and classifying them by type (physical Ethernet, Wi-Fi,
 * bridge, veth, VLAN, tunnel, etc.).  This is intended for tools that
 * need to present the user with a choice of interfaces — for example,
 * selecting the physical NIC to use for EtherCAT.
 *
 * The primary entry point is enumerateNetworkInterfaces(), which returns
 * a std::vector<NetworkInterface>.  Convenience filters such as
 * getPhysicalEthernetInterfaces() narrow the list to the subset relevant
 * for a given use-case.
 *
 * Linux implementation uses rtnetlink (NETLINK_ROUTE / RTM_GETLINK).
 * Other platforms return an empty vector or a platform-specific subset.
 */

#pragma once

#include "hal/HALTypes.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Interface Classification
// ============================================================================

/// High-level link-layer type, derived from ARPHRD_* and IFLA_INFO_KIND.
enum class InterfaceType {
    Unknown,       ///< Could not be classified
    Loopback,      ///< Loopback interface (lo)
    Ethernet,      ///< Physical wired Ethernet adapter
    Wireless,      ///< Wi-Fi / wireless adapter
    Bridge,        ///< Software bridge (br0, docker0, ...)
    Veth,          ///< Virtual ethernet pair endpoint
    Vlan,          ///< 802.1Q VLAN interface
    Bond,          ///< Bonding / link aggregation
    Macvlan,       ///< MAC-based virtual interface
    Tunnel,        ///< L3 tunnel (WireGuard, TUN, GRE, ...)
    Other,         ///< Virtual interface of an unknown kind
};

/// Whether the interface is backed by physical hardware or is software-created.
enum class InterfaceCategory {
    Unknown,
    Physical,      ///< Backed by a real network adapter (NIC, Wi-Fi, USB-Ethernet)
    Virtual,       ///< Created in software (bridge, veth, VLAN, tunnel, ...)
};

// ============================================================================
// Interface Info
// ============================================================================

/**
 * @brief Describes a single network interface returned by the enumerator.
 *
 * All fields are populated on Linux via rtnetlink.  On other platforms
 * some fields may be left at their default values.
 */
struct NetworkInterface {
    std::string name;                ///< Interface name (e.g. "eth0", "enp3s0")
    int ifindex = 0;                 ///< Kernel interface index

    /// ARP hardware type (ARPHRD_*).  1 = Ethernet, 772 = Loopback,
    /// 65534 = ARPHRD_NONE (tunnels).
    uint16_t arpType = 0;

    /// Link kind from IFLA_INFO_KIND (e.g. "bridge", "veth", "vlan",
    /// "wireguard", "tun").  Empty for physical interfaces.
    std::string kind;

    /// Slave kind from IFLA_INFO_SLAVE_KIND (e.g. "bridge" when a veth
    /// is enslaved to a bridge).  Empty when not enslaved.
    std::string slaveKind;

    std::optional<MacAddress> mac;        ///< L2 address (if present)
    std::optional<MacAddress> broadcast;  ///< Broadcast L2 address (if present)
    uint32_t mtu = 0;                     ///< Maximum transfer unit

    /// Raw kernel flags bitmask (IFF_* values).
    uint32_t flags = 0;

    bool isUp = false;           ///< IFF_UP is set (admin enabled)
    bool isRunning = false;      ///< IFF_RUNNING is set (operational)
    bool hasCarrier = false;     ///< IFLA_CARRIER == 1
    bool isWireless = false;     ///< /sys/class/net/<name>/wireless exists
    bool isPromiscuous = false;  ///< IFF_PROMISC is set

    /// IfIndex of the parent/underlying device (IFLA_LINK).
    /// Present for VLANs, veths, macvlans, etc.
    std::optional<int> parentIfindex;

    /// Classified link-layer type.
    InterfaceType type = InterfaceType::Unknown;

    /// Physical vs. virtual classification.
    InterfaceCategory category = InterfaceCategory::Unknown;
};

// ============================================================================
// Enumeration API
// ============================================================================

/**
 * @brief Enumerate all network interfaces on the system.
 *
 * On Linux this sends an RTM_GETLINK dump via rtnetlink and parses
 * every RTM_NEWLINK reply.  No special privileges are required.
 *
 * @return Vector of all interfaces, in kernel order.
 */
std::vector<NetworkInterface> enumerateNetworkInterfaces();

/**
 * @brief Filter to physical wired Ethernet adapters only.
 *
 * Applies three criteria:
 *  1. arpType == ARPHRD_ETHER (1)
 *  2. No IFLA_LINKINFO (not a virtual interface)
 *  3. No /sys/class/net/<name>/wireless (not Wi-Fi)
 *
 * @return Subset of enumerateNetworkInterfaces() matching all criteria.
 */
std::vector<NetworkInterface> getPhysicalEthernetInterfaces();

/**
 * @brief Filter to interfaces matching a specific InterfaceType.
 *
 * @param type Desired interface type
 * @return Subset of enumerateNetworkInterfaces() with that type.
 */
std::vector<NetworkInterface> getInterfacesByType(InterfaceType type);

/**
 * @brief Filter to interfaces matching a specific InterfaceCategory.
 *
 * @param category Desired category (Physical or Virtual)
 * @return Subset of enumerateNetworkInterfaces() with that category.
 */
std::vector<NetworkInterface> getInterfacesByCategory(InterfaceCategory category);

/**
 * @brief Convert an InterfaceType to a human-readable string.
 */
const char* interfaceTypeToString(InterfaceType type);

/**
 * @brief Convert an InterfaceCategory to a human-readable string.
 */
const char* interfaceCategoryToString(InterfaceCategory category);

} // namespace HAL
} // namespace EtherCAT
