// SPDX-License-Identifier: MIT
/**
 * @file UdpEthercatLink.hpp
 * @brief Point-to-point UDP carrier for raw EtherCAT frames.
 *
 * Carries complete Ethernet frames (destination MAC + ethertype 0x88A4 +
 * EtherCAT payload) as UDP datagram payloads between two processes on
 * localhost (or any reachable host).  This is the privilege-free
 * "virtual network interface" used by the Beckhoff fleet example and
 * the EtherCAT master probe: no CAP_NET_RAW, no kernel interfaces.
 *
 * One datagram == one Ethernet frame in each direction.
 */
#pragma once

#include "tether/ethercat/NetworkAbstraction.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace tether::examples {

class UdpEthercatLink {
public:
    UdpEthercatLink() = default;
    ~UdpEthercatLink() { close(); }

    UdpEthercatLink(const UdpEthercatLink&) = delete;
    UdpEthercatLink& operator=(const UdpEthercatLink&) = delete;

    /**
     * Open the link.
     * @param peerHost   peer address (e.g. "127.0.0.1")
     * @param peerPort   peer UDP port
     * @param localPort  local UDP port to bind (0 = ephemeral).  The peer
     *                   learns this port from the source address of the
     *                   first datagram, so it may stay 0.
     * @param bindAddr   local bind address ("0.0.0.0" or "127.0.0.1")
     */
    bool open(const std::string& peerHost, uint16_t peerPort,
              uint16_t localPort = 0, const std::string& bindAddr = "0.0.0.0") {
        close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;

        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port   = htons(localPort);
        if (::inet_pton(AF_INET, bindAddr.c_str(), &local.sin_addr) != 1) {
            close();
            return false;
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
            close();
            return false;
        }

        peer_ = {};
        peer_.sin_family = AF_INET;
        peer_.sin_port   = htons(peerPort);
        if (::inet_pton(AF_INET, peerHost.c_str(), &peer_.sin_addr) != 1) {
            close();
            return false;
        }
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    int fd() const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    /// The UDP port this socket is bound to (for --port advertisement).
    uint16_t localPort() const {
        if (fd_ < 0) return 0;
        sockaddr_in a{};
        socklen_t n = sizeof(a);
        if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &n) != 0)
            return 0;
        return ntohs(a.sin_port);
    }

    /// Wrap as an EtherCAT::NetworkInterface (send/receive/native_handle).
    EtherCAT::NetworkInterface interface() {
        EtherCAT::NetworkInterface ni;
        ni.send = [this](const uint8_t* data, size_t len) -> bool {
            return sendFrame(data, len);
        };
        ni.receive = [this](uint8_t* buf, size_t cap, size_t* out) -> bool {
            return recvFrame(buf, cap, out);
        };
        ni.native_handle = reinterpret_cast<void*>(
            static_cast<intptr_t>(fd_));
        return ni;
    }

    /// Server side: receive one datagram (non-blocking).  Returns the
    /// sender address so the caller can reply to it.
    bool recvFrom(uint8_t* buf, size_t cap, size_t* out,
                  sockaddr_in& from) {
        if (fd_ < 0) return false;
        socklen_t flen = sizeof(from);
        ssize_t n = ::recvfrom(fd_, buf, cap, MSG_DONTWAIT,
                               reinterpret_cast<sockaddr*>(&from), &flen);
        if (n <= 0) return false;
        *out = static_cast<size_t>(n);
        return true;
    }

    /// Server side: send a frame back to @p to.
    bool sendTo(const uint8_t* data, size_t len, const sockaddr_in& to) {
        if (fd_ < 0) return false;
        ssize_t n = ::sendto(fd_, data, len, 0,
                             reinterpret_cast<const sockaddr*>(&to),
                             sizeof(to));
        return n == static_cast<ssize_t>(len);
    }

private:
    bool sendFrame(const uint8_t* data, size_t len) {
        if (fd_ < 0) return false;
        ssize_t n = ::sendto(fd_, data, len, 0,
                             reinterpret_cast<const sockaddr*>(&peer_),
                             sizeof(peer_));
        return n == static_cast<ssize_t>(len);
    }

    bool recvFrame(uint8_t* buf, size_t cap, size_t* out) {
        if (fd_ < 0) return false;
        uint8_t tmp[2048];
        sockaddr_in from{};
        socklen_t flen = sizeof(from);
        // UDP datagrams larger than the buffer are truncated by the
        // kernel; EtherCAT frames stay well under 2 KiB here.
        ssize_t n = ::recvfrom(fd_, tmp, sizeof(tmp), MSG_DONTWAIT,
                               reinterpret_cast<sockaddr*>(&from), &flen);
        if (n <= 0 || static_cast<size_t>(n) > cap) return false;
        // Only accept datagrams from the configured peer.
        if (from.sin_addr.s_addr != peer_.sin_addr.s_addr ||
            from.sin_port != peer_.sin_port) {
            return false;
        }
        std::memcpy(buf, tmp, static_cast<size_t>(n));
        *out = static_cast<size_t>(n);
        return true;
    }

    int          fd_ = -1;
    sockaddr_in  peer_{};
};

} // namespace tether::examples
