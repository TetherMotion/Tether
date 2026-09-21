/**
 * @file LinuxEthernet.cpp
 * @brief Linux raw socket Ethernet HAL implementation
 *
 * This implementation uses AF_PACKET raw sockets for low-level Ethernet access.
 */

#ifdef __linux__

#include "hal/IEthernet.hpp"
#include "hal/HALTypes.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <atomic>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif

namespace EtherCAT {
namespace HAL {

/**
 * @brief Linux raw socket Ethernet implementation
 */
class LinuxRawSocketEthernet : public IEthernet {
public:
    LinuxRawSocketEthernet() = default;
    ~LinuxRawSocketEthernet() override { shutdown(); }

    Error init(const EthernetConfig& config) override {
        if (m_initialized) {
            return Error::AlreadyInitialized;
        }

        // Determine interface name
        const char* ifname = config.interfaceName;
        if (!ifname || ifname[0] == '\0') {
            ifname = "eth0";  // Default interface
        }
        strncpy(m_ifname, ifname, sizeof(m_ifname) - 1);
        m_ifname[sizeof(m_ifname) - 1] = '\0';

        // Create raw socket
        m_socket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
        if (m_socket < 0) {
            if (errno == EPERM || errno == EACCES) {
                return Error::PermissionDenied;
            }
            return Error::InternalError;
        }

        // Get interface index
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, m_ifname, IFNAMSIZ - 1);
        
        if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0) {
            close(m_socket);
            m_socket = -1;
            return Error::InterfaceNotFound;
        }
        m_ifindex = ifr.ifr_ifindex;

        // Get MAC address
        if (ioctl(m_socket, SIOCGIFHWADDR, &ifr) < 0) {
            close(m_socket);
            m_socket = -1;
            return Error::InternalError;
        }
        memcpy(m_mac.bytes, ifr.ifr_hwaddr.sa_data, 6);

        // Bind to interface
        struct sockaddr_ll sll;
        memset(&sll, 0, sizeof(sll));
        sll.sll_family = AF_PACKET;
        sll.sll_ifindex = m_ifindex;
        sll.sll_protocol = htons(ETH_P_ALL);

        if (bind(m_socket, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
            close(m_socket);
            m_socket = -1;
            return Error::InternalError;
        }

        // Store filter
        m_ethertypeFilter = config.ethertypeFilter;

        // Set socket to non-blocking
        int flags = fcntl(m_socket, F_GETFL, 0);
        fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);

#ifdef PACKET_IGNORE_OUTGOING
        // Don't deliver our own transmitted frames back to us — removes one
        // wasted wake-up + copy per cyclic TX.  Non-fatal if unsupported.
        {
            int one = 1;
            setsockopt(m_socket, SOL_PACKET, PACKET_IGNORE_OUTGOING,
                       &one, sizeof(one));
        }
#endif

#ifdef PACKET_AUXDATA
        // The kernel strips inbound 802.1Q tags into skb metadata before
        // packet sockets see the frame (rx-vlan-offload / the generic RX
        // untag in __netif_receive_skb_core).  Request PACKET_AUXDATA so
        // poll()/recvFrame() can surface the stripped VID/PCP through
        // RxFrameInfo so the VLAN router routes on the metadata directly.
        {
            int one = 1;
            setsockopt(m_socket, SOL_PACKET, PACKET_AUXDATA,
                       &one, sizeof(one));
        }
#endif

        // Mark initialized before performing operations that require initialized state
        m_initialized = true;

        // Set promiscuous mode if requested (setPromiscuous requires m_initialized)
        if (config.promiscuous) {
            Error err = setPromiscuous(true);
            if (err != Error::OK) {
                // Revert initialization and cleanup
                m_initialized = false;
                close(m_socket);
                m_socket = -1;
                return err;
            }
        }

        return Error::OK;
    }

    void shutdown() override {
        m_running = false;
        
        if (m_socket >= 0) {
            // Remove promiscuous mode if we set it
            if (m_promiscuous) {
                setPromiscuous(false);
            }
            close(m_socket);
            m_socket = -1;
        }
        
        m_initialized = false;
        m_rxCallback = nullptr;
        m_linkCallback = nullptr;
    }

    bool isInitialized() const override {
        return m_initialized;
    }

    Error getMacAddress(MacAddress& mac) const override {
        if (!m_initialized) return Error::NotInitialized;
        mac = m_mac;
        return Error::OK;
    }

    Error setMacAddress(const MacAddress& mac) override {
        if (!m_initialized) return Error::NotInitialized;

        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, m_ifname, IFNAMSIZ - 1);
        ifr.ifr_hwaddr.sa_family = ARPHRD_ETHER;
        memcpy(ifr.ifr_hwaddr.sa_data, mac.bytes, 6);

        if (ioctl(m_socket, SIOCSIFHWADDR, &ifr) < 0) {
            if (errno == EPERM) {
                return Error::PermissionDenied;
            }
            return Error::InternalError;
        }

        m_mac = mac;
        return Error::OK;
    }

    Error transmit(const uint8_t* frame, size_t length) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length > kMaxFrameSize) return Error::BufferTooSmall;

        struct sockaddr_ll sll;
        memset(&sll, 0, sizeof(sll));
        sll.sll_family = AF_PACKET;
        sll.sll_ifindex = m_ifindex;
        sll.sll_halen = ETH_ALEN;
        memcpy(sll.sll_addr, frame, 6);  // Destination MAC

        ssize_t sent = sendto(m_socket, frame, length, 0,
                              (struct sockaddr*)&sll, sizeof(sll));
        
        if (sent < 0) {
            m_stats.txErrors++;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                m_stats.txDropped++;
                return Error::WouldBlock;
            }
            return Error::TransmitFailed;
        }

        m_stats.txFrames++;
        m_stats.txBytes += length;
        return Error::OK;
    }

    Error transmitVlan(const uint8_t* frame, size_t length,
                       uint16_t vlanId, uint8_t priority) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length + kVlanTagSize > kMaxFrameSizeVlan) return Error::BufferTooSmall;

        // Build frame with VLAN tag
        uint8_t vlanFrame[kMaxFrameSizeVlan];
        
        // Copy MAC addresses (12 bytes)
        memcpy(vlanFrame, frame, 12);
        
        // Insert VLAN tag
        vlanFrame[12] = 0x81;  // TPID high byte
        vlanFrame[13] = 0x00;  // TPID low byte
        uint16_t tci = ((priority & 0x07) << 13) | (vlanId & 0x0FFF);
        vlanFrame[14] = (tci >> 8) & 0xFF;
        vlanFrame[15] = tci & 0xFF;
        
        // Copy rest of frame
        memcpy(vlanFrame + 16, frame + 12, length - 12);

        return transmit(vlanFrame, length + kVlanTagSize);
    }

    Error transmitGather(const BufferDesc* iov, size_t count) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!iov || count == 0) return Error::InvalidArgument;

        // Calculate total length
        size_t totalLen = 0;
        for (size_t i = 0; i < count; i++) {
            totalLen += iov[i].length;
        }

        if (totalLen < kMinFrameSize || totalLen > kMaxFrameSize) {
            return Error::InvalidArgument;
        }

        // Copy to contiguous buffer
        uint8_t frame[kMaxFrameSize];
        size_t offset = 0;
        for (size_t i = 0; i < count; i++) {
            memcpy(frame + offset, iov[i].data, iov[i].length);
            offset += iov[i].length;
        }

        return transmit(frame, totalLen);
    }

    void setRxCallback(RxCallback callback, void* userData) override {
        m_rxCallback = callback;
        m_rxUserData = userData;
    }

    int poll(Milliseconds timeoutMs) override {
        if (!m_initialized || m_socket < 0) return 0;

        struct pollfd pfd;
        pfd.fd = m_socket;
        pfd.events = POLLIN;
        pfd.revents = 0;

        int ret = ::poll(&pfd, 1, static_cast<int>(timeoutMs));
        if (ret <= 0) return 0;

        int count = 0;
        uint8_t buffer[kMaxFrameSizeVlan];

        // Read all available frames
        while (true) {
            struct sockaddr_ll sll;
            struct tpacket_auxdata aux;

            ssize_t len = recvPacketAux(buffer, sizeof(buffer), &sll, &aux);

            if (len <= 0) break;

            // Check packet direction - skip outgoing packets
            if (sll.sll_pkttype == PACKET_OUTGOING) {
                continue;
            }

            // Apply EtherType filter
            if (len >= 14) {
                bool hasVlan = false;
                uint16_t vlanId = 0;
                uint8_t vlanPriority = 0;
                uint16_t checkType = 0;
                fillVlanInfo(buffer, len, aux,
                             hasVlan, vlanId, vlanPriority, checkType);

                // Apply filter — checkType is always the inner EtherType,
                // whether the tag was inline or kernel-stripped.
                if (m_ethertypeFilter != 0 &&
                    checkType != m_ethertypeFilter) {
                    m_stats.rxFiltered++;
                    continue;
                }

                m_stats.rxFrames++;
                m_stats.rxBytes += len;

                if (m_rxCallback) {
                    RxFrameInfo info;
                    info.timestamp = getCurrentTimestamp();
                    info.vlanTagPresent = hasVlan;
                    info.vlanId = vlanId;
                    info.vlanPriority = vlanPriority;

                    m_rxCallback(buffer, len, info, m_rxUserData);
                }
            }

            count++;
        }

        return count;
    }

    int recvFrame(uint8_t* buffer, size_t capacity,
                  RxFrameInfo* info) override {
        if (!m_initialized || m_socket < 0) return -1;

        // Drain past filtered/outgoing frames so the caller sees either a
        // deliverable frame or an empty queue — never a "consumed but
        // dropped" result that would prematurely end a drain loop.
        for (int guard = 0; guard < 64; ++guard) {
            struct sockaddr_ll sll;
            struct tpacket_auxdata aux;
            ssize_t len = recvPacketAux(buffer, capacity, &sll, &aux);
            if (len <= 0) return 0;   // EAGAIN: queue empty

            if (sll.sll_pkttype == PACKET_OUTGOING) continue;

            if (len >= 14) {
                bool hasVlan = false;
                uint16_t vlanId = 0;
                uint8_t vlanPriority = 0;
                uint16_t checkType = 0;
                fillVlanInfo(buffer, len, aux,
                             hasVlan, vlanId, vlanPriority, checkType);
                if (m_ethertypeFilter != 0 && checkType != m_ethertypeFilter) {
                    m_stats.rxFiltered++;
                    continue;
                }
                if (info) {
                    info->timestamp = getCurrentTimestamp();
                    info->vlanTagPresent = hasVlan;
                    info->vlanId = vlanId;
                    info->vlanPriority = vlanPriority;
                }
            }

            m_stats.rxFrames++;
            m_stats.rxBytes += static_cast<uint64_t>(len);
            return static_cast<int>(len);
        }
        return 0;
    }

    void setEthertypeFilter(uint16_t ethertype) override {
        m_ethertypeFilter = ethertype;
    }

    Error setPromiscuous(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;

        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, m_ifname, IFNAMSIZ - 1);

        if (ioctl(m_socket, SIOCGIFFLAGS, &ifr) < 0) {
            return Error::InternalError;
        }

        if (enable) {
            ifr.ifr_flags |= IFF_PROMISC;
        } else {
            ifr.ifr_flags &= ~IFF_PROMISC;
        }

        if (ioctl(m_socket, SIOCSIFFLAGS, &ifr) < 0) {
            if (errno == EPERM) {
                return Error::PermissionDenied;
            }
            return Error::InternalError;
        }

        m_promiscuous = enable;
        return Error::OK;
    }

    Error addMulticastAddress(const MacAddress& mac) override {
        if (!m_initialized) return Error::NotInitialized;

        struct packet_mreq mreq;
        memset(&mreq, 0, sizeof(mreq));
        mreq.mr_ifindex = m_ifindex;
        mreq.mr_type = PACKET_MR_MULTICAST;
        mreq.mr_alen = 6;
        memcpy(mreq.mr_address, mac.bytes, 6);

        if (setsockopt(m_socket, SOL_PACKET, PACKET_ADD_MEMBERSHIP,
                       &mreq, sizeof(mreq)) < 0) {
            return Error::InternalError;
        }

        return Error::OK;
    }

    Error removeMulticastAddress(const MacAddress& mac) override {
        if (!m_initialized) return Error::NotInitialized;

        struct packet_mreq mreq;
        memset(&mreq, 0, sizeof(mreq));
        mreq.mr_ifindex = m_ifindex;
        mreq.mr_type = PACKET_MR_MULTICAST;
        mreq.mr_alen = 6;
        memcpy(mreq.mr_address, mac.bytes, 6);

        if (setsockopt(m_socket, SOL_PACKET, PACKET_DROP_MEMBERSHIP,
                       &mreq, sizeof(mreq)) < 0) {
            return Error::InternalError;
        }

        return Error::OK;
    }

    Error setAllMulticast(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;

        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, m_ifname, IFNAMSIZ - 1);

        if (ioctl(m_socket, SIOCGIFFLAGS, &ifr) < 0) {
            return Error::InternalError;
        }

        if (enable) {
            ifr.ifr_flags |= IFF_ALLMULTI;
        } else {
            ifr.ifr_flags &= ~IFF_ALLMULTI;
        }

        if (ioctl(m_socket, SIOCSIFFLAGS, &ifr) < 0) {
            return Error::InternalError;
        }

        return Error::OK;
    }

    LinkStatus getLinkStatus() const override {
        LinkStatus status;
        if (!m_initialized) return status;

        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, m_ifname, IFNAMSIZ - 1);

        if (ioctl(m_socket, SIOCGIFFLAGS, &ifr) == 0) {
            status.up = (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
        }

        // Get speed and duplex via ethtool ioctl (simplified)
        status.speedMbps = 1000;  // Assume gigabit
        status.fullDuplex = true;
        status.autoneg = true;

        return status;
    }

    void setLinkCallback(LinkCallback callback, void* userData) override {
        m_linkCallback = callback;
        m_linkUserData = userData;
    }

    Error waitForLinkUp(Milliseconds timeoutMs) override {
        Timestamp start = getCurrentTimestamp();
        
        while (true) {
            LinkStatus status = getLinkStatus();
            if (status.up) return Error::OK;

            Timestamp elapsed = getCurrentTimestamp() - start;
            if (elapsed / 1000 >= static_cast<uint64_t>(timeoutMs)) {
                return Error::Timeout;
            }

            usleep(10000);  // 10ms
        }
    }

    EthernetStats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = {};
    }

    void* nativeHandle() override {
        return reinterpret_cast<void*>(static_cast<intptr_t>(m_socket));
    }

    const char* getInterfaceName() const override {
        return m_ifname;
    }

private:
    /**
     * @brief recvmsg() wrapper that also collects PACKET_AUXDATA control
     *        data describing a kernel-stripped 802.1Q tag.
     *
     * The kernel untags VLAN frames on RX before delivering them to
     *        packet sockets: the data buffer then shows the inner
     *        EtherType at [12] and the tag lives only in skb metadata
     *        (tpacket_auxdata).  Rather than reinserting the tag (a
     *        per-frame memmove), the metadata is surfaced through
     *        RxFrameInfo so consumers such as VLANRouter can route on it
     *        directly.
     */
    ssize_t recvPacketAux(uint8_t* buf, size_t cap, sockaddr_ll* sll,
                          tpacket_auxdata* aux) {
        iovec iov{buf, cap};
        alignas(cmsghdr) char cbuf[CMSG_SPACE(sizeof(tpacket_auxdata))];
        msghdr msg{};
        msg.msg_name       = sll;
        msg.msg_namelen    = sizeof(*sll);
        msg.msg_iov        = &iov;
        msg.msg_iovlen     = 1;
        msg.msg_control    = cbuf;
        msg.msg_controllen = sizeof(cbuf);

        aux->tp_status   = 0;
        aux->tp_vlan_tci = 0;
        ssize_t len = ::recvmsg(m_socket, &msg, MSG_DONTWAIT);
        if (len <= 0) return len;

        for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level == SOL_PACKET &&
                c->cmsg_type == PACKET_AUXDATA) {
                memcpy(aux, CMSG_DATA(c), sizeof(*aux));
                break;
            }
        }
        return len;
    }

    /// Fill VLAN fields of a RxFrameInfo from inline bytes or, when the
    /// kernel stripped the tag, from PACKET_AUXDATA metadata.
    static void fillVlanInfo(const uint8_t* buf, ssize_t len,
                             const tpacket_auxdata& aux,
                             bool& hasVlan, uint16_t& vlanId,
                             uint8_t& vlanPriority, uint16_t& innerType) {
        const uint16_t ethertype =
            static_cast<uint16_t>((buf[12] << 8) | buf[13]);
        innerType = ethertype;
        if (ethertype == kEtherType8021Q && len >= 18) {
            hasVlan = true;
            vlanId = static_cast<uint16_t>(
                ((buf[14] & 0x0F) << 8) | buf[15]);
            vlanPriority = static_cast<uint8_t>((buf[14] >> 5) & 0x07);
            innerType = static_cast<uint16_t>((buf[16] << 8) | buf[17]);
            return;
        }
#ifdef TP_STATUS_VLAN_VALID
        if (aux.tp_status & TP_STATUS_VLAN_VALID) {
            // Kernel stripped the tag — auxdata carries the wire VID.
            hasVlan = true;
            vlanId = static_cast<uint16_t>(aux.tp_vlan_tci & 0x0FFF);
            vlanPriority =
                static_cast<uint8_t>((aux.tp_vlan_tci >> 13) & 0x07);
            // innerType is already the (inner) EtherType at [12].
        }
#endif
    }

    bool m_initialized = false;
    int m_socket = -1;
    int m_ifindex = 0;
    char m_ifname[IFNAMSIZ] = {0};
    MacAddress m_mac;
    bool m_promiscuous = false;
    uint16_t m_ethertypeFilter = 0;
    std::atomic<bool> m_running{false};

    RxCallback m_rxCallback = nullptr;
    void* m_rxUserData = nullptr;
    LinkCallback m_linkCallback = nullptr;
    void* m_linkUserData = nullptr;

    EthernetStats m_stats;

    static Timestamp getCurrentTimestamp() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<Timestamp>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    }
};

std::unique_ptr<IEthernet> createLinuxRawSocketEthernet() {
    return std::make_unique<LinuxRawSocketEthernet>();
}

// Default factory for Linux platform
std::unique_ptr<IEthernet> createDefaultEthernet() {
    return createLinuxRawSocketEthernet();
}

} // namespace HAL
} // namespace EtherCAT

#endif // __linux__
