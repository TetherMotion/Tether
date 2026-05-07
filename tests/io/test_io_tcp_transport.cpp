/**
 * @file test_io_tcp_transport.cpp
 * @brief Host-side tests for TcpTransport and TcpTransportServer.
 */
#include <gtest/gtest.h>
#include "tether/io/TcpTransport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <future>
#include <thread>

using namespace tether::io;

namespace {

class ScopedFd {
public:
    explicit ScopedFd(int fd = -1) : fd_(fd) {}
    ~ScopedFd() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    ScopedFd(const ScopedFd&) = delete;
    ScopedFd& operator=(const ScopedFd&) = delete;

    ScopedFd(ScopedFd&& other) noexcept : fd_(other.fd_) {
        other.fd_ = -1;
    }

    ScopedFd& operator=(ScopedFd&& other) noexcept {
        if (this != &other) {
            if (fd_ >= 0) {
                ::close(fd_);
            }
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    int get() const { return fd_; }
    int release() {
        int fd = fd_;
        fd_ = -1;
        return fd;
    }

private:
    int fd_;
};

uint16_t reserveLoopbackPort() {
    ScopedFd fd(::socket(AF_INET, SOCK_STREAM, 0));
    EXPECT_GE(fd.get(), 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;

    EXPECT_EQ(::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);

    socklen_t len = sizeof(addr);
    EXPECT_EQ(::getsockname(fd.get(), reinterpret_cast<sockaddr*>(&addr), &len), 0);
    return ntohs(addr.sin_port);
}

ScopedFd connectLoopback(uint16_t port) {
    ScopedFd fd(::socket(AF_INET, SOCK_STREAM, 0));
    EXPECT_GE(fd.get(), 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);

    EXPECT_EQ(::connect(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
    return fd;
}

} // namespace

TEST(TcpTransport, SendAndReceive) {
    std::array<int, 2> fds{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds.data()), 0);

    ScopedFd peer(fds[1]);
    TcpTransport transport(fds[0]);

    const uint8_t outbound[] = {1, 2, 3};
    ASSERT_TRUE(transport.send(outbound, sizeof(outbound)));

    uint8_t peerBuf[8] = {};
    ASSERT_EQ(::recv(peer.get(), peerBuf, sizeof(peerBuf), 0), 3);
    EXPECT_EQ(std::memcmp(peerBuf, outbound, sizeof(outbound)), 0);

    const uint8_t inbound[] = {9, 8, 7, 6};
    ASSERT_EQ(::send(peer.get(), inbound, sizeof(inbound), 0), 4);

    uint8_t rx[8] = {};
    EXPECT_EQ(transport.receive(rx, sizeof(rx), 100), 4u);
    EXPECT_EQ(std::memcmp(rx, inbound, sizeof(inbound)), 0);
    EXPECT_TRUE(transport.isConnected());
}

TEST(TcpTransport, ReceiveTimeoutReturnsZero) {
    std::array<int, 2> fds{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds.data()), 0);

    ScopedFd peer(fds[1]);
    TcpTransport transport(fds[0]);

    uint8_t buf[4] = {};
    EXPECT_EQ(transport.receive(buf, sizeof(buf), 20), 0u);
    EXPECT_TRUE(transport.isConnected());
}

TEST(TcpTransport, PeerCloseMarksDisconnected) {
    std::array<int, 2> fds{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds.data()), 0);

    ScopedFd peer(fds[1]);
    TcpTransport transport(fds[0]);

    ::close(peer.release());

    uint8_t buf[4] = {};
    EXPECT_EQ(transport.receive(buf, sizeof(buf), 100), 0u);
    EXPECT_FALSE(transport.isConnected());
    EXPECT_FALSE(transport.send(buf, 1));
}

TEST(TcpTransport, CloseDisconnects) {
    std::array<int, 2> fds{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds.data()), 0);

    ScopedFd peer(fds[1]);
    TcpTransport transport(fds[0]);

    transport.close();
    EXPECT_FALSE(transport.isConnected());

    uint8_t buf[4] = {};
    EXPECT_EQ(transport.receive(buf, sizeof(buf), 10), 0u);
    EXPECT_FALSE(transport.send(buf, 1));
}

TEST(TcpTransportServer, AcceptConnectionAndExchangeData) {
    uint16_t port = reserveLoopbackPort();
    TcpTransportServer server(port);
    ASSERT_TRUE(server.start());
    ASSERT_TRUE(server.isListening());

    auto acceptFuture = std::async(std::launch::async, [&server]() {
        return server.accept();
    });

    ScopedFd clientFd = connectLoopback(port);
    auto accepted = acceptFuture.get();
    ASSERT_NE(accepted, nullptr);

    const uint8_t clientMsg[] = {5, 4, 3};
    ASSERT_EQ(::send(clientFd.get(), clientMsg, sizeof(clientMsg), 0), 3);

    uint8_t serverBuf[8] = {};
    EXPECT_EQ(accepted->receive(serverBuf, sizeof(serverBuf), 1000), 3u);
    EXPECT_EQ(std::memcmp(serverBuf, clientMsg, sizeof(clientMsg)), 0);

    const uint8_t serverMsg[] = {7, 8};
    ASSERT_TRUE(accepted->send(serverMsg, sizeof(serverMsg)));

    uint8_t clientBuf[8] = {};
    ASSERT_EQ(::recv(clientFd.get(), clientBuf, sizeof(clientBuf), 0), 2);
    EXPECT_EQ(std::memcmp(clientBuf, serverMsg, sizeof(serverMsg)), 0);

    server.stop();
    EXPECT_FALSE(server.isListening());
}

TEST(TcpTransportServer, StartTwiceFails) {
    uint16_t port = reserveLoopbackPort();
    TcpTransportServer server(port);
    ASSERT_TRUE(server.start());
    EXPECT_FALSE(server.start());
    server.stop();
}

TEST(TcpTransportServer, BindFailureReturnsFalse) {
    uint16_t port = reserveLoopbackPort();
    TcpTransportServer first(port);
    TcpTransportServer second(port);

    ASSERT_TRUE(first.start());
    EXPECT_FALSE(second.start());
    first.stop();
}

TEST(TcpTransportServer, AcceptWithoutStartReturnsNull) {
    TcpTransportServer server(0);
    EXPECT_EQ(server.accept(), nullptr);
}

TEST(TcpTransportServer, StopWithoutStartIsSafe) {
    TcpTransportServer server(0);
    server.stop();
    EXPECT_FALSE(server.isListening());
}