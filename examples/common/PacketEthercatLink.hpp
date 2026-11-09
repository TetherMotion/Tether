// SPDX-License-Identifier: MIT
/**
 * @file PacketEthercatLink.hpp
 * @brief AF_PACKET carrier for raw EtherCAT frames on a kernel interface.
 *
 * Carries complete Ethernet frames (destination MAC + ethertype 0x88A4 +
 * EtherCAT payload) as L2 frames on a real network interface — typically
 * one end of a veth pair, which acts as a point-to-point "virtual
 * interface" between two processes:
 *
 *   ip link add veth-ec0 type veth peer name veth-ec1
 *   ip link set veth-ec0 up && ip link set veth-ec1 up
 *   beckhoff_slave_chain --iface veth-ec0 &
 *   ethercat_fleet_probe  --iface veth-ec1
 *
 * Unlike UdpEthercatLink this needs CAP_NET_RAW (AF_PACKET socket) and
 * the interface must exist, so run under runec or with equivalent
 * capabilities.  The socket binds to ethertype 0x88A4 only, so IP and
 * other traffic on the interface is ignored, and PACKET_IGNORE_OUTGOING
 * keeps our own transmissions out of the receive path.
 */
#pragma once

#include "tether/ethercat/NetworkAbstraction.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace tether::examples {

class PacketEthercatLink {
public:
    PacketEthercatLink() = default;
    ~PacketEthercatLink() { close(); }

    PacketEthercatLink(const PacketEthercatLink&) = delete;
    PacketEthercatLink& operator=(const PacketEthercatLink&) = delete;

    /**
     * Open the link on @p ifaceName (e.g. "veth-ec0").  The interface
     * must exist and be up; frames are exchanged with whatever is on
     * the other end.
     */
    bool open(const std::string& ifaceName) {
        close();
        const unsigned idx = ::if_nametoindex(ifaceName.c_str());
        if (idx == 0) return false;

        fd_ = ::socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC,
                       htons(kEtherCatType));
        if (fd_ < 0) return false;

        // Do not tap our own transmissions; without this every request
        // we send would also arrive as an inbound frame.  If the kernel
        // rejects the option we fall back to filtering sll_pkttype in
        // userspace and disable the recvmmsg batch path (which cannot
        // see the packet header).
        int one = 1;
        ignoreOutgoing_ =
            ::setsockopt(fd_, SOL_PACKET, PACKET_IGNORE_OUTGOING,
                         &one, sizeof(one)) == 0;

        sockaddr_ll bindAddr{};
        bindAddr.sll_family   = AF_PACKET;
        bindAddr.sll_protocol = htons(kEtherCatType);
        bindAddr.sll_ifindex  = static_cast<int>(idx);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&bindAddr),
                   sizeof(bindAddr)) != 0) {
            close();
            return false;
        }
        ifindex_ = idx;
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        ifindex_ = 0;
        ignoreOutgoing_ = false;
    }

    int fd() const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    /// Wrap as an EtherCAT::NetworkInterface (send/receive/native_handle).
    EtherCAT::NetworkInterface interface() {
        EtherCAT::NetworkInterface ni;
        ni.send = [this](const uint8_t* data, size_t len) -> bool {
            return sendFrame(data, len);
        };
        ni.receive = [this](uint8_t* buf, size_t cap, size_t* out) -> bool {
            return recvFrame(buf, cap, out);
        };
        // The recvmmsg batch path reads the fd directly and cannot filter
        // packet direction, so only expose it when the kernel already
        // drops outgoing taps for us.
        ni.native_handle = ignoreOutgoing_
            ? reinterpret_cast<void*>(static_cast<intptr_t>(fd_))
            : nullptr;
        return ni;
    }

    /// Receive one inbound Ethernet frame (non-blocking).  Skips
    /// outgoing/looped-back packets as a fallback for kernels without
    /// PACKET_IGNORE_OUTGOING.
    bool recvFrom(uint8_t* buf, size_t cap, size_t* out) {
        return recvFrame(buf, cap, out);
    }

    /// Transmit @p data as one Ethernet frame on the interface.  The
    /// destination MAC is taken from the frame header (bytes 0-5), so
    /// callers pass the complete L2 frame unchanged.
    bool sendFrame(const uint8_t* data, size_t len) {
        if (fd_ < 0 || len < 14) return false;
        sockaddr_ll to{};
        to.sll_family   = AF_PACKET;
        to.sll_ifindex  = static_cast<int>(ifindex_);
        to.sll_halen    = 6;
        std::memcpy(to.sll_addr, data, 6);
        const ssize_t n = ::sendto(fd_, data, len, MSG_DONTWAIT,
                                   reinterpret_cast<const sockaddr*>(&to),
                                   sizeof(to));
        return n == static_cast<ssize_t>(len);
    }

private:
    bool recvFrame(uint8_t* buf, size_t cap, size_t* out) {
        if (fd_ < 0) return false;
        // A few tries to skip self/loopback taps when the kernel-side
        // PACKET_IGNORE_OUTGOING filter is unavailable.
        for (int tries = 0; tries < 8; ++tries) {
            iovec   iov{};
            iov.iov_base = buf;
            iov.iov_len  = cap;
            sockaddr_ll sll{};
            msghdr msg{};
            msg.msg_name    = &sll;
            msg.msg_namelen = sizeof(sll);
            msg.msg_iov     = &iov;
            msg.msg_iovlen  = 1;
            const ssize_t n = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
            if (n <= 0) return false;
            if (msg.msg_flags & MSG_TRUNC) continue;
            if (!ignoreOutgoing_ &&
                (sll.sll_pkttype == PACKET_OUTGOING ||
                 sll.sll_pkttype == PACKET_LOOPBACK)) {
                continue;
            }
            *out = static_cast<size_t>(n);
            return true;
        }
        return false;
    }

    static constexpr uint16_t kEtherCatType = 0x88A4;

    int          fd_ = -1;
    unsigned     ifindex_ = 0;
    bool         ignoreOutgoing_ = false;
};

} // namespace tether::examples
