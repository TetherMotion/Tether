/**
 * @file UdpLink.hpp
 * @brief Modbus/UDP client link (POSIX datagram socket, MBAP framing).
 *        Behind TETHER_MODBUS_UDP.
 *
 * UDP is datagram-oriented: each write() is one datagram carrying a
 * complete MBAP ADU; each read() returns at most one datagram. The
 * socket is connect()ed so stray datagrams from other peers are
 * filtered by the kernel. Lost datagrams surface as read timeouts.
 */
#pragma once

#include "tether/modbus/link/ILink.hpp"

#if defined(TETHER_MODBUS_UDP) && !defined(ESP_PLATFORM)

#include <string>

namespace tether::modbus {

class UdpLink final : public ILink {
public:
    UdpLink(std::string host, uint16_t port, int connectMs = 3000,
            OpenOpts opts = {});
    explicit UdpLink(int fd, std::string peer);   ///< adopt a socket
    ~UdpLink() override;

    Expected<void>   open() override;
    void             close() override;
    bool             isOpen() const override;
    /// One datagram (must fit one ADU).
    Expected<void>   write(std::span<const uint8_t>, int timeoutMs) override;
    /// One datagram truncated to buf.size(); 0 = timeout.
    Expected<size_t> read(std::span<uint8_t> buf, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    Expected<size_t> drain() override;
    LinkInfo         info() const override;
    int              fd() const override { return fd_; }
    void setTap(std::function<void(bool, std::span<const uint8_t>)> t)
        override { tap_ = std::move(t); }

    const std::string& peer() const { return peer_; }

private:
    std::string host_, peer_;
    uint16_t    port_ = 0;
    int         connectMs_ = 3000;
    OpenOpts    opts_;
    int         fd_ = -1;
    std::function<void(bool, std::span<const uint8_t>)> tap_;
};

} // namespace tether::modbus

#endif // TETHER_MODBUS_UDP && !ESP_PLATFORM
