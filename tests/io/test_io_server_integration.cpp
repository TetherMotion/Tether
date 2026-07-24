/**
 * @file test_io_server_integration.cpp
 * @brief Integration tests for Server using PipeTransportServer.
 */
#include <gtest/gtest.h>
#include "tether/io/Server.hpp"
#include "PipeTransport.hpp"
#include "SLIPStream/Buffer.hpp"
#include <thread>
#include <chrono>
#include <cstring>

using namespace tether::io;
using namespace tether::io::testing;

// ===========================================================================
// Helper
// ===========================================================================

static void noopLogFn(const char* /*tag*/, const char* /*fmt*/, ...) {}

static int g_logCallCount = 0;
static void countingLogFn(const char* /*tag*/, const char* /*fmt*/, ...) {
    ++g_logCallCount;
}

static void slipSend(ITransport* tp, const uint8_t* msg, size_t len) {
    size_t encLen = SLIPStream::encoded_length(msg, len);
    std::vector<uint8_t> enc(encLen);
    SLIPStream::encode_packet(msg, len, enc.data(), enc.size());
    tp->send(enc.data(), enc.size());
}

static std::vector<uint8_t> slipReceive(ITransport* tp, uint32_t timeoutMs = 500) {
    std::vector<uint8_t> accum;
    uint8_t buf[4096];
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        uint32_t remain = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        if (remain == 0) remain = 1;
        size_t n = tp->receive(buf, sizeof(buf), std::min(remain, 50u));
        if (n > 0) {
            accum.insert(accum.end(), buf, buf + n);
            if (!accum.empty() && accum.back() == 0xC0) {
                size_t decLen = SLIPStream::decoded_length(accum.data(), accum.size());
                if (decLen != SLIPStream::DECODE_ERROR && decLen > 0) {
                    std::vector<uint8_t> decoded(decLen);
                    size_t wrote = SLIPStream::decode_packet(
                        accum.data(), accum.size(), decoded.data(), decoded.size());
                    if (wrote != SLIPStream::DECODE_ERROR) {
                        decoded.resize(wrote);
                        return decoded;
                    }
                }
            }
        }
    }
    return {};
}

/// Send a GetParam request and expect a response — proves the session is alive.
static bool verifySessionAlive(ITransport* client, uint64_t paramId = 1,
                               uint32_t timeoutMs = 2000) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(paramId);
    slipSend(client, msg, w.pos);
    auto resp = slipReceive(client, timeoutMs);
    if (resp.empty()) return false;
    return resp[0] == static_cast<uint8_t>(MessageType::GetParamResp);
}

static bool waitForSessionAlive(ITransport* client, uint64_t timeoutMs = 3000,
                                uint64_t paramId = 1) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (verifySessionAlive(client, paramId)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

/// Wait until the server's active session count reaches the expected value.
/// Replaces fixed 600ms sleeps with a poll that exits as soon as cleanup
/// completes (typically <50ms).
static void waitSessionCount(Server& server, size_t expected,
                             uint64_t timeoutMs = 2000) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (server.activeSessionCount() == expected) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/// Wait until a client can no longer communicate (i.e. the server has
/// rejected or disconnected it).  Replaces fixed 600ms sleeps in tests
/// that verify a client is rejected by maxClients enforcement.
static void waitClientDead(ITransport* client, uint64_t timeoutMs = 2000) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        // Use a short 100ms probe so we retry quickly instead of blocking 2s
        if (!verifySessionAlive(client, 1, 100)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ===========================================================================
// Fixture
// ===========================================================================

class ServerIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        uint32_t* valPtr = &paramVal_;
        ParamEntry p;
        p.id = 1;
        p.name = "test_param";
        p.description = "Test";
        p.group = "test";
        p.valueType = ValueType::U32;
        p.readFn = [valPtr](void* d) { std::memcpy(d, valPtr, 4); };
        p.writeFn = [valPtr](const void* s) { std::memcpy(valPtr, s, 4); };
        registry_.addParam(std::move(p));
    }

    Registry registry_;
    uint32_t paramVal_ = 100;
    uint64_t fakeTs_ = 1000;
};

// ===========================================================================
// Tests
// ===========================================================================

TEST_F(ServerIntegrationTest, StartAndStop) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    EXPECT_TRUE(server.start());
    EXPECT_TRUE(server.isRunning());
    EXPECT_EQ(server.activeSessionCount(), 0u);

    server.stop();
    EXPECT_FALSE(server.isRunning());
}

TEST_F(ServerIntegrationTest, StartWithoutTimestampFails) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    ServerConfig config;
    config.timestampFn = nullptr;

    Server server(registry_, std::move(pipeServer), config);
    EXPECT_FALSE(server.start());
}

TEST_F(ServerIntegrationTest, DoubleStartFails) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    ServerConfig config;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    EXPECT_TRUE(server.start());
    EXPECT_FALSE(server.start());  // already running
    server.stop();
}

TEST_F(ServerIntegrationTest, SingleClientSession) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    // Connect a client and verify the session is alive via actual communication
    auto client = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client.get()));
    EXPECT_EQ(server.activeSessionCount(), 1u);

    // Also verify GetParam response contents
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(1);
    slipSend(client.get(), msg, w.pos);

    auto resp = slipReceive(client.get());
    ASSERT_GE(resp.size(), 10u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetParamResp));
    EXPECT_EQ(r.getU64(), 1u);

    client->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, MultipleClients) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 4;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client1 = ps->addPendingConnection();
    auto client2 = ps->addPendingConnection();

    // Both clients can query independently — proves both sessions are alive
    ASSERT_TRUE(waitForSessionAlive(client1.get()));
    ASSERT_TRUE(waitForSessionAlive(client2.get()));
    EXPECT_EQ(server.activeSessionCount(), 2u);

    client1->close();
    client2->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, MaxClientsEnforced) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 1;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client1 = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client1.get()));

    // Second connection should be rejected (server closes its end)
    auto client2 = ps->addPendingConnection();
    waitClientDead(client2.get());

    // client2 should be disconnected or unable to communicate
    EXPECT_FALSE(verifySessionAlive(client2.get(), 1, 200));

    client1->close();
    client2->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, CleanupFinishedSessions) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 4;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    // Connect first client and verify it works
    auto client1 = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client1.get()));

    // Disconnect client1
    client1->close();
    waitSessionCount(server, 0);

    // Connect another client — cleanup should have reclaimed the slot
    auto client2 = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client2.get()));

    client2->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, ServerWithLogging) {
    auto pipeServer = std::make_unique<PipeTransportServer>();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };
    config.logFn = noopLogFn;

    Server server(registry_, std::move(pipeServer), config);
    server.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    server.stop();
}

TEST_F(ServerIntegrationTest, StopWhileClientIsActive) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client.get()));
    EXPECT_EQ(server.activeSessionCount(), 1u);

    server.stop();

    EXPECT_FALSE(server.isRunning());
    EXPECT_EQ(server.activeSessionCount(), 0u);
}

TEST_F(ServerIntegrationTest, MaxClientsLoggingPath) {
    g_logCallCount = 0;

    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 1;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };
    config.logFn = countingLogFn;

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client1 = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client1.get()));

    auto client2 = ps->addPendingConnection();
    waitClientDead(client2.get());
    EXPECT_FALSE(verifySessionAlive(client2.get(), 1, 200));
    EXPECT_GT(g_logCallCount, 0);

    client1->close();
    client2->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, ServerWithFeatures) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };
    config.serverFeatures.features.push_back(
        Feature::makeString("server_name", "TestServer"));

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client = ps->addPendingConnection();

    // Verify the session is alive before sending feature exchange
    ASSERT_TRUE(waitForSessionAlive(client.get()));

    // Feature exchange
    FeatureSet clientFeatures;
    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    clientFeatures.encode(w);
    slipSend(client.get(), msg, w.pos);

    auto resp = slipReceive(client.get());
    ASSERT_FALSE(resp.empty());
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::FeatureExchangeResp));

    FeatureSet respFeatures;
    EXPECT_TRUE(FeatureSet::decode(r, respFeatures));
    EXPECT_NE(respFeatures.find("server_name"), nullptr);

    client->close();
    waitSessionCount(server, 0);
    server.stop();
}

TEST_F(ServerIntegrationTest, DatalogRecorderAccessible) {
    auto pipeServer = std::make_unique<PipeTransportServer>();

    ServerConfig config;
    config.maxClients = 1;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    EXPECT_NE(server.datalogRecorder(), nullptr);
}

TEST_F(ServerIntegrationTest, DoubleStopIsSafe) {
    auto pipeServer = std::make_unique<PipeTransportServer>();

    ServerConfig config;
    config.maxClients = 1;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();
    server.stop();
    server.stop();  // second stop should be a no-op
}

TEST_F(ServerIntegrationTest, ActiveSessionCountAfterStop) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    auto client = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client.get()));

    client->close();
    waitSessionCount(server, 0);
    server.stop();

    // After stop, all sessions are cleaned up
    EXPECT_EQ(server.activeSessionCount(), 0u);
}

TEST_F(ServerIntegrationTest, ServerWithLoggingAndClient) {
    auto pipeServer = std::make_unique<PipeTransportServer>();
    PipeTransportServer* ps = pipeServer.get();

    ServerConfig config;
    config.maxClients = 2;
    config.timestampFn = [this]() -> uint64_t { return fakeTs_++; };
    config.logFn = noopLogFn;

    Server server(registry_, std::move(pipeServer), config);
    server.start();

    // Connect a client while logging is enabled — exercises log paths
    auto client = ps->addPendingConnection();
    ASSERT_TRUE(waitForSessionAlive(client.get()));

    client->close();
    waitSessionCount(server, 0);
    server.stop();
}
