#pragma once

#ifndef BOOST_ERROR_CODE_HEADER_ONLY
#define BOOST_ERROR_CODE_HEADER_ONLY
#endif

#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED
#endif

#include "tether/io/Transport.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tether::examples {

class WebSocketTransport final : public io::ITransport {
public:
    using WsStream = boost::beast::websocket::stream<boost::beast::tcp_stream>;

    explicit WebSocketTransport(std::unique_ptr<WsStream> stream);

    bool send(const uint8_t* data, size_t len) override;
    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    void close() override;
    bool isConnected() const override;

private:
    std::unique_ptr<WsStream> stream_;
    std::vector<uint8_t> pendingRx_;
    size_t pendingOffset_ = 0;
    std::atomic<bool> connected_{true};
};

class WebSocketTransportServer final : public io::ITransportServer {
public:
    explicit WebSocketTransportServer(uint16_t port, std::string expectedPath = "/");
    ~WebSocketTransportServer() override;

    bool start() override;
    void stop() override;
    std::unique_ptr<io::ITransport> accept() override;
    bool isListening() const override;

private:
    using tcp = boost::asio::ip::tcp;

    uint16_t port_;
    std::string expectedPath_;
    boost::asio::io_context ioContext_{1};
    tcp::acceptor acceptor_{ioContext_};
    std::atomic<bool> listening_{false};
};

} // namespace tether::examples