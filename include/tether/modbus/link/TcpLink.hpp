/**
 * @file TcpLink.hpp
 * @brief TCP client link + listener (POSIX sockets).
 */
#pragma once

#include "tether/modbus/link/ILink.hpp"

#if !defined(ESP_PLATFORM)

namespace tether::modbus {

class TcpLink final : public ILink {
public:
    TcpLink(std::string host, uint16_t port, int connectMs,
            OpenOpts opts = {});
    /// Adopt an already-connected fd (server-side accept path).
    explicit TcpLink(int fd, std::string peer);
    ~TcpLink() override;

    Expected<void>   open() override;
    void             close() override;
    bool             isOpen() const override;
    Expected<void>   write(std::span<const uint8_t>, int timeoutMs) override;
    Expected<size_t> read(std::span<uint8_t>, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    LinkInfo         info() const override;
    int              fd() const override;
    void setTap(std::function<void(bool, std::span<const uint8_t>)> t) override {
        tap_ = std::move(t);
    }
    const std::string& peer() const { return peer_; }

private:
    std::string host_;
    std::string peer_;
    uint16_t    port_ = 0;
    int         connectMs_ = 5000;
    OpenOpts    opts_;
    int         fd_ = -1;
    std::function<void(bool, std::span<const uint8_t>)> tap_;
};

class TcpListener final : public ILinkListener {
public:
    TcpListener(std::string bindHost, uint16_t port, int backlog = 8,
                bool reuseAddr = true);
    ~TcpListener() override;

    Expected<void> listen() override;
    Expected<std::unique_ptr<ILink>> accept(int timeoutMs) override;
    void           close() override;
    bool           isListening() const override;
    std::string    boundEndpoint() const override;

private:
    std::string bindHost_;
    uint16_t    port_ = 0;
    int         backlog_ = 8;
    bool        reuseAddr_ = true;
    int         fd_ = -1;
};

} // namespace tether::modbus

#endif // !ESP_PLATFORM
