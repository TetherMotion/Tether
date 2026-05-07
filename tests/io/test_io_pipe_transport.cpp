/**
 * @file test_io_pipe_transport.cpp
 * @brief Unit tests for PipeTransport, PipeBuffer, PipeTransportServer.
 */
#include <gtest/gtest.h>
#include "PipeTransport.hpp"
#include <thread>
#include <chrono>

using namespace tether::io;
using namespace tether::io::testing;

// ===========================================================================
// PipeBuffer
// ===========================================================================

TEST(PipeBuffer, PushAndPop) {
    PipeBuffer buf;
    uint8_t data[] = {1, 2, 3, 4, 5};
    buf.push(data, 5);

    uint8_t out[16];
    size_t n = buf.pop(out, sizeof(out), 0);
    EXPECT_EQ(n, 5u);
    EXPECT_EQ(out[0], 1);
    EXPECT_EQ(out[4], 5);
}

TEST(PipeBuffer, PopPartial) {
    PipeBuffer buf;
    uint8_t data[] = {10, 20, 30, 40};
    buf.push(data, 4);

    uint8_t out[2];
    size_t n = buf.pop(out, 2, 0);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(out[0], 10);
    EXPECT_EQ(out[1], 20);

    // Remaining data
    n = buf.pop(out, 2, 0);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(out[0], 30);
    EXPECT_EQ(out[1], 40);
}

TEST(PipeBuffer, PopEmptyReturnsZero) {
    PipeBuffer buf;
    uint8_t out[8];
    size_t n = buf.pop(out, sizeof(out), 0);
    EXPECT_EQ(n, 0u);
}

TEST(PipeBuffer, PopBlocksAndWakes) {
    PipeBuffer buf;
    uint8_t data[] = {42};

    std::thread writer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        buf.push(data, 1);
    });

    uint8_t out[4];
    size_t n = buf.pop(out, sizeof(out), 500);
    EXPECT_EQ(n, 1u);
    EXPECT_EQ(out[0], 42);

    writer.join();
}

TEST(PipeBuffer, PopTimeoutExpires) {
    PipeBuffer buf;
    uint8_t out[4];
    auto start = std::chrono::steady_clock::now();
    size_t n = buf.pop(out, sizeof(out), 50);
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_EQ(n, 0u);
    EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 40);
}

TEST(PipeBuffer, CloseWakesPop) {
    PipeBuffer buf;
    std::thread closer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        buf.close();
    });

    uint8_t out[4];
    size_t n = buf.pop(out, sizeof(out), 5000);
    EXPECT_EQ(n, 0u);
    EXPECT_TRUE(buf.isClosed());
    closer.join();
}

// ===========================================================================
// PipeTransport pair
// ===========================================================================

TEST(PipeTransport, CreatePairAndCommunicate) {
    auto [a, b] = PipeTransport::create();

    uint8_t msg[] = {1, 2, 3};
    EXPECT_TRUE(a->send(msg, 3));

    uint8_t out[16];
    size_t n = b->receive(out, sizeof(out), 100);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(out[0], 1);
    EXPECT_EQ(out[2], 3);

    // Reverse direction
    uint8_t reply[] = {10, 20};
    EXPECT_TRUE(b->send(reply, 2));

    n = a->receive(out, sizeof(out), 100);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(out[0], 10);
    EXPECT_EQ(out[1], 20);
}

TEST(PipeTransport, IsConnected) {
    auto [a, b] = PipeTransport::create();
    EXPECT_TRUE(a->isConnected());
    EXPECT_TRUE(b->isConnected());

    a->close();
    EXPECT_FALSE(a->isConnected());

    // b should also detect closure via its buffers
    uint8_t out[4];
    b->receive(out, sizeof(out), 10);
}

TEST(PipeTransport, SendAfterClose) {
    auto [a, b] = PipeTransport::create();
    a->close();
    uint8_t data[] = {1};
    EXPECT_FALSE(a->send(data, 1));
}

TEST(PipeTransport, ReceiveAfterClose) {
    auto [a, b] = PipeTransport::create();
    a->close();
    uint8_t out[4];
    size_t n = a->receive(out, sizeof(out), 0);
    EXPECT_EQ(n, 0u);
}

// ===========================================================================
// PipeTransportServer
// ===========================================================================

TEST(PipeTransportServer, StartStopListening) {
    PipeTransportServer server;
    EXPECT_FALSE(server.isListening());
    EXPECT_TRUE(server.start());
    EXPECT_TRUE(server.isListening());
    server.stop();
    EXPECT_FALSE(server.isListening());
}

TEST(PipeTransportServer, AcceptYieldsConnection) {
    PipeTransportServer server;
    server.start();

    auto client = server.addPendingConnection();
    ASSERT_NE(client, nullptr);

    auto serverSide = server.accept();
    ASSERT_NE(serverSide, nullptr);

    // Verify communication works
    uint8_t msg[] = {99};
    EXPECT_TRUE(client->send(msg, 1));
    uint8_t out[4];
    size_t n = serverSide->receive(out, sizeof(out), 100);
    EXPECT_EQ(n, 1u);
    EXPECT_EQ(out[0], 99);

    server.stop();
}

TEST(PipeTransportServer, AcceptReturnsNullWhenEmpty) {
    PipeTransportServer server;
    server.start();
    auto t = server.accept();
    EXPECT_EQ(t, nullptr);
    server.stop();
}
