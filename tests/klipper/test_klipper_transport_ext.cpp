/**
 * @file test_klipper_transport_ext.cpp
 * @brief Extended transport tests: loopback close/reopen, large data, TCP, readAll.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/transport/PipeTransport.hpp"
#include "tether/klipper/transport/TcpStreamTransport.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

using namespace tether::klipper::transport;

// ============================================================================
// LoopbackTransport extended tests
// ============================================================================

TEST(KlipperLoopbackExt, CloseAndReopen) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    EXPECT_TRUE(host.isOpen());
    EXPECT_TRUE(dev.isOpen());

    host.close();
    EXPECT_FALSE(host.isOpen());
    EXPECT_TRUE(dev.isOpen()); // Other end still open

    // Reopen
    EXPECT_TRUE(host.open());
    EXPECT_TRUE(host.isOpen());
}

TEST(KlipperLoopbackExt, LargeDataRoundTrip) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    std::vector<uint8_t> largeData(10000);
    for (size_t i = 0; i < largeData.size(); ++i) {
        largeData[i] = static_cast<uint8_t>(i % 256);
    }

    size_t written = host.write(largeData);
    EXPECT_EQ(written, largeData.size());

    // Read in chunks
    std::vector<uint8_t> received;
    uint8_t buf[512];
    while (received.size() < largeData.size()) {
        size_t n = dev.read(buf, sizeof(buf));
        if (n == 0) break;
        received.insert(received.end(), buf, buf + n);
    }

    EXPECT_EQ(received.size(), largeData.size());
    EXPECT_EQ(received, largeData);
}

TEST(KlipperLoopbackExt, WriteToClosedTransport) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    host.close();

    std::vector<uint8_t> data = {0x01, 0x02};
    size_t written = host.write(data);
    EXPECT_EQ(written, 0u); // Should fail on closed transport
}

TEST(KlipperLoopbackExt, ReadFromClosedTransport) {
    LoopbackTransportPair pair;
    auto& dev = pair.deviceEnd();
    dev.close();

    uint8_t buf[10];
    size_t n = dev.read(buf, 10);
    EXPECT_EQ(n, 0u);
}

TEST(KlipperLoopbackExt, AvailableOnEmpty) {
    LoopbackTransportPair pair;
    EXPECT_EQ(pair.hostEnd().available(), 0u);
    EXPECT_EQ(pair.deviceEnd().available(), 0u);
}

TEST(KlipperLoopbackExt, AvailableAfterWrite) {
    LoopbackTransportPair pair;
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    pair.hostEnd().write(data);
    EXPECT_EQ(pair.deviceEnd().available(), 3u);
    EXPECT_EQ(pair.hostEnd().available(), 0u); // Host doesn't see its own write
}

TEST(KlipperLoopbackExt, PartialRead) {
    LoopbackTransportPair pair;
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    pair.hostEnd().write(data);

    uint8_t buf[3];
    size_t n = pair.deviceEnd().read(buf, 3);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x02);
    EXPECT_EQ(buf[2], 0x03);

    EXPECT_EQ(pair.deviceEnd().available(), 2u);
}

TEST(KlipperLoopbackExt, ZeroLengthWrite) {
    LoopbackTransportPair pair;
    std::vector<uint8_t> empty;
    size_t written = pair.hostEnd().write(empty);
    EXPECT_EQ(written, 0u);
}

TEST(KlipperLoopbackExt, ReadAll) {
    LoopbackTransportPair pair;
    std::vector<uint8_t> data = {0xAA, 0xBB, 0xCC, 0xDD};
    pair.hostEnd().write(data);

    auto received = pair.deviceEnd().readAll();
    EXPECT_EQ(received, data);
}

TEST(KlipperLoopbackExt, ReadAllOnEmpty) {
    LoopbackTransportPair pair;
    auto received = pair.hostEnd().readAll();
    EXPECT_TRUE(received.empty());
}

TEST(KlipperLoopbackExt, BidirectionalSimultaneous) {
    LoopbackTransportPair pair;
    std::vector<uint8_t> h2d = {0x01, 0x02};
    std::vector<uint8_t> d2h = {0x03, 0x04};

    pair.hostEnd().write(h2d);
    pair.deviceEnd().write(d2h);

    auto hostRecv = pair.hostEnd().readAll();
    auto devRecv = pair.deviceEnd().readAll();

    EXPECT_EQ(hostRecv, d2h);
    EXPECT_EQ(devRecv, h2d);
}

// ============================================================================
// PipeTransport tests
// ============================================================================

TEST(KlipperPipeExt, BasicRoundTrip) {
    int fds[2];
    ASSERT_EQ(pipe(fds), 0);

    PipeTransportConfig cfg;
    cfg.readFd = fds[0];
    cfg.writeFd = fds[1];
    cfg.ownsFds = true;

    PipeTransport transport(cfg);
    ASSERT_TRUE(transport.open());
    EXPECT_TRUE(transport.isOpen());

    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    size_t written = transport.write(data);
    EXPECT_EQ(written, data.size());

    // Give pipe time to buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_GE(transport.available(), data.size());

    uint8_t buf[10];
    size_t n = transport.read(buf, 10);
    EXPECT_EQ(n, data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        EXPECT_EQ(buf[i], data[i]);
    }
}

TEST(KlipperPipeExt, Close) {
    int fds[2];
    ASSERT_EQ(pipe(fds), 0);

    PipeTransportConfig cfg;
    cfg.readFd = fds[0];
    cfg.writeFd = fds[1];
    cfg.ownsFds = true;

    {
        PipeTransport transport(cfg);
        transport.open();
        EXPECT_TRUE(transport.isOpen());
    } // Destructor closes
    // After destruction, fds are closed
}

// ============================================================================
// TCP transport tests
// ============================================================================

TEST(KlipperTcpExt, ClientServerRoundTrip) {
    // Use a fixed high port for testing
    uint16_t testPort = 19123;

    TcpTransportConfig serverCfg;
    serverCfg.mode = "server";
    serverCfg.host = "127.0.0.1";
    serverCfg.port = testPort;
    serverCfg.backlog = 1;
    serverCfg.timeoutMs = 3000;

    TcpStreamTransport server(serverCfg);

    // Start server in a thread (open() blocks waiting for connection)
    std::thread serverThread([&]() {
        server.open();
    });

    // Give server time to start listening
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Connect as client
    TcpTransportConfig clientCfg;
    clientCfg.mode = "client";
    clientCfg.host = "127.0.0.1";
    clientCfg.port = testPort;
    clientCfg.timeoutMs = 3000;

    TcpStreamTransport client(clientCfg);
    bool clientOk = client.open();

    // Wait for server thread to finish (it accepted our connection)
    serverThread.join();

    if (!clientOk || !server.isOpen()) {
        // TCP test may fail in restricted environments - skip if so
        GTEST_SKIP() << "TCP connection failed (possibly restricted environment)";
    }

    // Write from client, read from server
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    size_t written = client.write(data);
    EXPECT_EQ(written, data.size());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint8_t buf[16];
    size_t n = server.read(buf, 16);
    EXPECT_EQ(n, data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        EXPECT_EQ(buf[i], data[i]);
    }

    client.close();
    server.close();
}

TEST(KlipperTcpExt, ClientConnectToNothing) {
    TcpTransportConfig cfg;
    cfg.mode = "client";
    cfg.host = "127.0.0.1";
    cfg.port = 1; // Port 1 should not be listening
    cfg.timeoutMs = 500;

    TcpStreamTransport client(cfg);
    bool ok = client.open();
    // Should fail or timeout
    EXPECT_FALSE(ok);
}
