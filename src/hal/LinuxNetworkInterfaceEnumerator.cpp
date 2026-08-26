/**
 * @file LinuxNetworkInterfaceEnumerator.cpp
 * @brief Linux rtnetlink-based network interface enumeration
 *
 * Uses NETLINK_ROUTE / RTM_GETLINK to dump all network interfaces from
 * the kernel and classifies them by type (physical, virtual, wireless,
 * bridge, veth, VLAN, tunnel, etc.).
 */

#ifdef __linux__

#include "hal/NetworkInterfaceEnumerator.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Internal helpers
// ============================================================================

namespace {

/// Send an RTM_GETLINK dump request and collect all RTM_NEWLINK replies.
/// Returns the raw message buffer; each datagram may contain multiple
/// netlink messages back-to-back.
bool sendDumpRequest(int sock_fd) {
    struct {
        struct nlmsghdr nlh;
        struct ifinfomsg ifm;
    } req{};

    req.nlh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nlh.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
    req.nlh.nlmsg_type = RTM_GETLINK;
    req.ifm.ifi_family = AF_UNSPEC;

    struct sockaddr_nl dest{};
    dest.nl_family = AF_NETLINK;

    struct iovec iov = { &req, req.nlh.nlmsg_len };
    struct msghdr msg = { &dest, sizeof(dest), &iov, 1, nullptr, 0, 0 };

    return sendmsg(sock_fd, &msg, 0) >= 0;
}

/// Check whether /sys/class/net/<name>/wireless exists.
/// This is the reliable way to detect Wi-Fi adapters — rtnetlink does
/// not include a "is wireless" flag in dump replies.
bool isWirelessInterface(const std::string& name) {
    std::string path = "/sys/class/net/" + name + "/wireless";
    struct stat st{};
    return stat(path.c_str(), &st) == 0;
}

/// Classify an interface based on arpType, kind string, and wireless flag.
InterfaceType classifyType(uint16_t arpType,
                           const std::string& kind,
                           bool isWireless) {
    if (arpType == ARPHRD_LOOPBACK)
        return InterfaceType::Loopback;

    if (!kind.empty()) {
        if (kind == "bridge")  return InterfaceType::Bridge;
        if (kind == "veth")    return InterfaceType::Veth;
        if (kind == "vlan")    return InterfaceType::Vlan;
        if (kind == "bond")    return InterfaceType::Bond;
        if (kind == "macvlan" || kind == "macvtap")
            return InterfaceType::Macvlan;
        if (kind == "wireguard" || kind == "tun" || kind == "tap" ||
            kind == "gre" || kind == "ipip" || kind == "sit" ||
            kind == "vti" || kind == "xfrm" || kind == "ip6tnl")
            return InterfaceType::Tunnel;
        return InterfaceType::Other;
    }

    // No IFLA_LINKINFO → physical device.
    if (arpType == ARPHRD_ETHER) {
        return isWireless ? InterfaceType::Wireless : InterfaceType::Ethernet;
    }

    // ARPHRD_NONE (65534) without a known kind — likely a tunnel.
    if (arpType == ARPHRD_NONE)
        return InterfaceType::Tunnel;

    return InterfaceType::Unknown;
}

/// Determine whether an interface is physical or virtual.
InterfaceCategory classifyCategory(InterfaceType type) {
    switch (type) {
        case InterfaceType::Ethernet:
        case InterfaceType::Wireless:
            return InterfaceCategory::Physical;
        case InterfaceType::Loopback:
        case InterfaceType::Bridge:
        case InterfaceType::Veth:
        case InterfaceType::Vlan:
        case InterfaceType::Bond:
        case InterfaceType::Macvlan:
        case InterfaceType::Tunnel:
        case InterfaceType::Other:
            return InterfaceCategory::Virtual;
        default:
            return InterfaceCategory::Unknown;
    }
}

/// Parse a single RTM_NEWLINK message into a NetworkInterface.
NetworkInterface parseLinkMessage(struct nlmsghdr* nlh) {
    auto* ifi = reinterpret_cast<struct ifinfomsg*>(NLMSG_DATA(nlh));
    int attr_len = IFLA_PAYLOAD(nlh);
    struct rtattr* rta = IFLA_RTA(ifi);

    NetworkInterface iface;
    iface.ifindex = ifi->ifi_index;
    iface.arpType = static_cast<uint16_t>(ifi->ifi_type);
    iface.flags = ifi->ifi_flags;
    iface.isUp = (ifi->ifi_flags & IFF_UP) != 0;
    iface.isRunning = (ifi->ifi_flags & IFF_RUNNING) != 0;
    iface.isPromiscuous = (ifi->ifi_flags & IFF_PROMISC) != 0;

    for (; RTA_OK(rta, attr_len); rta = RTA_NEXT(rta, attr_len)) {
        switch (rta->rta_type) {
            case IFLA_IFNAME:
                iface.name = reinterpret_cast<char*>(RTA_DATA(rta));
                break;
            case IFLA_MTU:
                iface.mtu = *reinterpret_cast<uint32_t*>(RTA_DATA(rta));
                break;
            case IFLA_ADDRESS: {
                int len = RTA_PAYLOAD(rta);
                auto* mac = reinterpret_cast<unsigned char*>(RTA_DATA(rta));
                if (len == 6)
                    iface.mac = MacAddress(mac);
                break;
            }
            case IFLA_BROADCAST: {
                int len = RTA_PAYLOAD(rta);
                auto* mac = reinterpret_cast<unsigned char*>(RTA_DATA(rta));
                if (len == 6)
                    iface.broadcast = MacAddress(mac);
                break;
            }
            case IFLA_LINK:
                iface.parentIfindex =
                    static_cast<int>(*reinterpret_cast<uint32_t*>(RTA_DATA(rta)));
                break;
            case IFLA_CARRIER:
                iface.hasCarrier =
                    *reinterpret_cast<uint8_t*>(RTA_DATA(rta)) != 0;
                break;
            case IFLA_LINKINFO: {
                // Nested attribute: parse IFLA_INFO_KIND and
                // IFLA_INFO_SLAVE_KIND from the payload.
                int sub_len = RTA_PAYLOAD(rta);
                auto* sub = reinterpret_cast<struct rtattr*>(RTA_DATA(rta));
                for (; RTA_OK(sub, sub_len); sub = RTA_NEXT(sub, sub_len)) {
                    switch (sub->rta_type) {
                        case IFLA_INFO_KIND:
                            iface.kind =
                                reinterpret_cast<char*>(RTA_DATA(sub));
                            break;
                        case IFLA_INFO_SLAVE_KIND:
                            iface.slaveKind =
                                reinterpret_cast<char*>(RTA_DATA(sub));
                            break;
                    }
                }
                break;
            }
        }
    }

    // Check wireless via sysfs (requires the name to be known).
    if (!iface.name.empty())
        iface.isWireless = isWirelessInterface(iface.name);

    // Classify type and category.
    iface.type = classifyType(iface.arpType, iface.kind, iface.isWireless);
    iface.category = classifyCategory(iface.type);

    return iface;
}

} // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

std::vector<NetworkInterface> enumerateNetworkInterfaces() {
    std::vector<NetworkInterface> result;

    int sock_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (sock_fd < 0)
        return result;

    // Bind to our own PID (optional for dump, but good practice).
    struct sockaddr_nl sa{};
    sa.nl_family = AF_NETLINK;
    sa.nl_pid = getpid();
    sa.nl_groups = 0;
    if (bind(sock_fd, reinterpret_cast<struct sockaddr*>(&sa), sizeof(sa)) < 0) {
        close(sock_fd);
        return result;
    }

    if (!sendDumpRequest(sock_fd)) {
        close(sock_fd);
        return result;
    }

    // Read replies until NLMSG_DONE.  A large dump may span multiple
    // datagrams, so we loop on recv().
    char buf[16384];
    bool done = false;

    while (!done) {
        ssize_t len = recv(sock_fd, buf, sizeof(buf), 0);
        if (len < 0)
            break;

        auto* nlh = reinterpret_cast<struct nlmsghdr*>(buf);
        for (; NLMSG_OK(nlh, len); nlh = NLMSG_NEXT(nlh, len)) {
            if (nlh->nlmsg_type == NLMSG_DONE) {
                done = true;
                break;
            }
            if (nlh->nlmsg_type == NLMSG_ERROR) {
                done = true;
                break;
            }
            if (nlh->nlmsg_type == RTM_NEWLINK)
                result.push_back(parseLinkMessage(nlh));
        }
    }

    close(sock_fd);
    return result;
}

std::vector<NetworkInterface> getPhysicalEthernetInterfaces() {
    std::vector<NetworkInterface> result;
    for (const auto& iface : enumerateNetworkInterfaces()) {
        if (iface.type == InterfaceType::Ethernet)
            result.push_back(iface);
    }
    return result;
}

std::vector<NetworkInterface> getInterfacesByType(InterfaceType type) {
    std::vector<NetworkInterface> result;
    for (const auto& iface : enumerateNetworkInterfaces()) {
        if (iface.type == type)
            result.push_back(iface);
    }
    return result;
}

std::vector<NetworkInterface> getInterfacesByCategory(InterfaceCategory category) {
    std::vector<NetworkInterface> result;
    for (const auto& iface : enumerateNetworkInterfaces()) {
        if (iface.category == category)
            result.push_back(iface);
    }
    return result;
}

const char* interfaceTypeToString(InterfaceType type) {
    switch (type) {
        case InterfaceType::Loopback:  return "Loopback";
        case InterfaceType::Ethernet:  return "Ethernet";
        case InterfaceType::Wireless:  return "Wireless";
        case InterfaceType::Bridge:    return "Bridge";
        case InterfaceType::Veth:      return "Veth";
        case InterfaceType::Vlan:      return "VLAN";
        case InterfaceType::Bond:      return "Bond";
        case InterfaceType::Macvlan:   return "Macvlan";
        case InterfaceType::Tunnel:    return "Tunnel";
        case InterfaceType::Other:     return "Other";
        default:                       return "Unknown";
    }
}

const char* interfaceCategoryToString(InterfaceCategory category) {
    switch (category) {
        case InterfaceCategory::Physical: return "Physical";
        case InterfaceCategory::Virtual:  return "Virtual";
        default:                          return "Unknown";
    }
}

} // namespace HAL
} // namespace EtherCAT

#else // !__linux__

// ============================================================================
// Non-Linux stubs
// ============================================================================

#include "hal/NetworkInterfaceEnumerator.hpp"

namespace EtherCAT {
namespace HAL {

std::vector<NetworkInterface> enumerateNetworkInterfaces() {
    return {};
}

std::vector<NetworkInterface> getPhysicalEthernetInterfaces() {
    return {};
}

std::vector<NetworkInterface> getInterfacesByType(InterfaceType) {
    return {};
}

std::vector<NetworkInterface> getInterfacesByCategory(InterfaceCategory) {
    return {};
}

const char* interfaceTypeToString(InterfaceType type) {
    switch (type) {
        case InterfaceType::Loopback:  return "Loopback";
        case InterfaceType::Ethernet:  return "Ethernet";
        case InterfaceType::Wireless:  return "Wireless";
        case InterfaceType::Bridge:    return "Bridge";
        case InterfaceType::Veth:      return "Veth";
        case InterfaceType::Vlan:      return "VLAN";
        case InterfaceType::Bond:      return "Bond";
        case InterfaceType::Macvlan:   return "Macvlan";
        case InterfaceType::Tunnel:    return "Tunnel";
        case InterfaceType::Other:     return "Other";
        default:                       return "Unknown";
    }
}

const char* interfaceCategoryToString(InterfaceCategory category) {
    switch (category) {
        case InterfaceCategory::Physical: return "Physical";
        case InterfaceCategory::Virtual:  return "Virtual";
        default:                          return "Unknown";
    }
}

} // namespace HAL
} // namespace EtherCAT

#endif // __linux__
