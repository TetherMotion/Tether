#include "WebSocketTransport.hpp"

#include <boost/asio/buffer.hpp>

namespace tether::examples {

using tcp = boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;

WebSocketTransport::WebSocketTransport(std::unique_ptr<WsStream> stream)
    : stream_(std::move(stream)) {}

bool WebSocketTransport::send(const uint8_t* data, size_t len) {
    if (!connected_.load(std::memory_order_relaxed) || !stream_) {
        return false;
    }

    beast::error_code ec;
    stream_->binary(true);
    stream_->write(net::buffer(data, len), ec);
    if (ec) {
        connected_.store(false, std::memory_order_relaxed);
        return false;
    }
    return true;
}

size_t WebSocketTransport::receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (!connected_.load(std::memory_order_relaxed) || !stream_) {
        return 0;
    }

    if (pendingOffset_ < pendingRx_.size()) {
        const size_t available = pendingRx_.size() - pendingOffset_;
        const size_t toCopy = std::min(maxLen, available);
        std::memcpy(buf, pendingRx_.data() + pendingOffset_, toCopy);
        pendingOffset_ += toCopy;
        if (pendingOffset_ >= pendingRx_.size()) {
            pendingRx_.clear();
            pendingOffset_ = 0;
        }
        return toCopy;
    }

    beast::flat_buffer frame;
    beast::error_code ec;
    if (timeoutMs > 0) {
        beast::get_lowest_layer(*stream_).expires_after(std::chrono::milliseconds(timeoutMs));
    } else {
        beast::get_lowest_layer(*stream_).expires_never();
    }

    stream_->read(frame, ec);
    if (ec == beast::error::timeout || ec == net::error::timed_out || ec == net::error::would_block) {
        return 0;
    }
    if (ec == websocket::error::closed || ec) {
        connected_.store(false, std::memory_order_relaxed);
        return 0;
    }

    pendingRx_.resize(frame.size());
    net::buffer_copy(net::buffer(pendingRx_), frame.data());
    pendingOffset_ = 0;

    const size_t toCopy = std::min(maxLen, pendingRx_.size());
    std::memcpy(buf, pendingRx_.data(), toCopy);
    pendingOffset_ = toCopy;
    if (pendingOffset_ >= pendingRx_.size()) {
        pendingRx_.clear();
        pendingOffset_ = 0;
    }
    return toCopy;
}

void WebSocketTransport::close() {
    if (!stream_) {
        connected_.store(false, std::memory_order_relaxed);
        return;
    }

    beast::error_code ec;
    stream_->close(websocket::close_code::normal, ec);
    beast::get_lowest_layer(*stream_).socket().shutdown(tcp::socket::shutdown_both, ec);
    beast::get_lowest_layer(*stream_).socket().close(ec);
    connected_.store(false, std::memory_order_relaxed);
}

bool WebSocketTransport::isConnected() const {
    return connected_.load(std::memory_order_relaxed);
}

WebSocketTransportServer::WebSocketTransportServer(uint16_t port, std::string expectedPath)
    : port_(port)
    , expectedPath_(std::move(expectedPath)) {}

WebSocketTransportServer::~WebSocketTransportServer() {
    stop();
}

bool WebSocketTransportServer::start() {
    if (listening_.load(std::memory_order_relaxed)) {
        return false;
    }

    beast::error_code ec;
    const tcp::endpoint endpoint{tcp::v4(), port_};

    acceptor_.open(endpoint.protocol(), ec);
    if (ec) {
        return false;
    }
    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
        return false;
    }
    acceptor_.bind(endpoint, ec);
    if (ec) {
        return false;
    }
    acceptor_.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
        return false;
    }

    listening_.store(true, std::memory_order_relaxed);
    return true;
}

void WebSocketTransportServer::stop() {
    listening_.store(false, std::memory_order_relaxed);
    beast::error_code ec;
    acceptor_.cancel(ec);
    acceptor_.close(ec);
}

std::unique_ptr<io::ITransport> WebSocketTransportServer::accept() {
    while (listening_.load(std::memory_order_relaxed)) {
        beast::error_code ec;
        tcp::socket socket(ioContext_);
        acceptor_.accept(socket, ec);
        if (ec) {
            if (!listening_.load(std::memory_order_relaxed)) {
                return nullptr;
            }
            continue;
        }

        beast::flat_buffer requestBuffer;
        http::request<http::string_body> request;
        http::read(socket, requestBuffer, request, ec);
        if (ec) {
            socket.close();
            continue;
        }

        if (!websocket::is_upgrade(request)) {
            http::response<http::string_body> response{http::status::upgrade_required, request.version()};
            response.set(http::field::server, "tether-native-backend");
            response.set(http::field::content_type, "text/plain");
            response.keep_alive(false);
            response.body() = "WebSocket upgrade required\n";
            response.prepare_payload();
            http::write(socket, response, ec);
            socket.shutdown(tcp::socket::shutdown_both, ec);
            socket.close(ec);
            continue;
        }

        if (!expectedPath_.empty() && expectedPath_ != "/" && request.target() != expectedPath_) {
            http::response<http::string_body> response{http::status::not_found, request.version()};
            response.set(http::field::server, "tether-native-backend");
            response.set(http::field::content_type, "text/plain");
            response.keep_alive(false);
            response.body() = "Unknown WebSocket path\n";
            response.prepare_payload();
            http::write(socket, response, ec);
            socket.shutdown(tcp::socket::shutdown_both, ec);
            socket.close(ec);
            continue;
        }

        auto ws = std::make_unique<WebSocketTransport::WsStream>(beast::tcp_stream(std::move(socket)));
        ws->set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws->binary(true);
        ws->accept(request, ec);
        if (ec) {
            beast::get_lowest_layer(*ws).socket().close();
            continue;
        }

        return std::make_unique<WebSocketTransport>(std::move(ws));
    }

    return nullptr;
}

bool WebSocketTransportServer::isListening() const {
    return listening_.load(std::memory_order_relaxed);
}

} // namespace tether::examples