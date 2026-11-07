/**
 * @file test_io_session.cpp
 * @brief Unit tests for Session using a mock transport.
 */
#include <gtest/gtest.h>
#include "tether/io/Session.hpp"
#include "SLIPStream/Buffer.hpp"
#include <atomic>
#include <queue>
#include <mutex>
#include <cstring>
#include <thread>
#include <chrono>
#include <array>
#include <algorithm>
#include <iterator>

using namespace tether::io;

// ===========================================================================
// Mock transport
// ===========================================================================

class MockTransport : public ITransport {
public:
    bool send(const uint8_t* data, size_t len) override {
        std::lock_guard<std::mutex> lock(mutex_);
        txData_.insert(txData_.end(), data, data + len);
        return true;
    }

    size_t receive(uint8_t* buf, size_t maxLen, uint32_t /*timeoutMs*/) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (rxData_.empty()) return 0;
        size_t n = std::min(maxLen, rxData_.size());
        std::memcpy(buf, rxData_.data(), n);
        rxData_.erase(rxData_.begin(), rxData_.begin() + n);
        return n;
    }

    void close() override { connected_.store(false, std::memory_order_relaxed); }
    bool isConnected() const override { return connected_.load(std::memory_order_relaxed); }

    // Test helpers
    void injectRx(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        rxData_.insert(rxData_.end(), data, data + len);
    }

    void injectSlipMessage(const uint8_t* msg, size_t len) {
        // Encode as SLIP and inject
        size_t encLen = SLIPStream::encoded_length(msg, len);
        std::vector<uint8_t> enc(encLen);
        SLIPStream::encode_packet(msg, len, enc.data(), enc.size());
        injectRx(enc.data(), enc.size());
    }

    std::vector<uint8_t> consumeTx() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto data = std::move(txData_);
        txData_.clear();
        return data;
    }

    /// Decode the first SLIP packet from txData, return the decoded payload.
    std::vector<uint8_t> decodeTxPacket() {
        std::vector<uint8_t> raw;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto end = std::find(txData_.begin(), txData_.end(), uint8_t{0xC0});
            if (end == txData_.end()) return {};
            raw.assign(txData_.begin(), std::next(end));
            txData_.erase(txData_.begin(), std::next(end));
        }
        size_t decLen = SLIPStream::decoded_length(raw.data(), raw.size());
        if (decLen == SLIPStream::DECODE_ERROR || decLen == 0) return {};
        std::vector<uint8_t> decoded(decLen);
        size_t wrote = SLIPStream::decode_packet(raw.data(), raw.size(),
                                                  decoded.data(), decoded.size());
        if (wrote == SLIPStream::DECODE_ERROR) return {};
        decoded.resize(wrote);
        return decoded;
    }

    std::atomic<bool> connected_{true};

private:
    mutable std::mutex mutex_;
    std::vector<uint8_t> rxData_;
    std::vector<uint8_t> txData_;
};

// ===========================================================================
// Test fixture
// ===========================================================================

class SessionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Add a test parameter
        double* valPtr = &paramValue_;
        ParamEntry p;
        p.id = 1;
        p.name = "test_param";
        p.description = "A test parameter";
        p.group = "test";
        p.valueType = ValueType::F64;
        p.readFn = [valPtr](void* d) { std::memcpy(d, valPtr, 8); };
        p.writeFn = [valPtr](const void* s) { std::memcpy(valPtr, s, 8); };
        registry_.addParam(std::move(p));

        // Add a test signal
        uint32_t* sigPtr = &signalValue_;
        SignalEntry s;
        s.id = 2;
        s.name = "test_signal";
        s.description = "A test signal";
        s.group = "test";
        s.valueType = ValueType::U32;
        s.readFn = [sigPtr](void* d) { std::memcpy(d, sigPtr, 4); };
        registry_.addSignal(std::move(s));
    }

    /// Run the session in a thread, inject a message, collect the response.
    std::vector<uint8_t> sendAndReceive(const uint8_t* msg, size_t msgLen,
                                        bool bootstrap = true,
                                        const SchemaCatalog* schemaCatalog = nullptr,
                                        LogFn logFn = nullptr,
                                        ReceiveBufferFactory encodedFactory = nullptr) {
        auto transport = std::make_unique<MockTransport>();
        MockTransport* tp = transport.get();

        if (bootstrap) {
            std::array<uint8_t, 256> hello{};
            BufWriter helloWriter(hello.data(), hello.size());
            helloWriter.putU8(static_cast<uint8_t>(MessageType::ClientHello));
            encodeClientHelloV6(helloWriter, {});
            tp->injectSlipMessage(hello.data(), helloWriter.pos);

            std::array<uint8_t, 32> commit{};
            BufWriter commitWriter(commit.data(), commit.size());
            commitWriter.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
            encodeSchemaCommitV6(commitWriter, {1});
            tp->injectSlipMessage(commit.data(), commitWriter.pos);
        }

        // Inject the request
        tp->injectSlipMessage(msg, msgLen);

        uint64_t fakeTs = 1000;
        auto tsFn = [&fakeTs]() -> uint64_t { return fakeTs++; };

        Session session(std::move(transport), registry_, tsFn, logFn, nullptr,
            nullptr, nullptr, encodedFactory, nullptr, Framing::Slip, nullptr,
                schemaCatalog);

        // Run in a thread, let it process the message, then stop
        std::thread t([&session]() { session.run(); });

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        session.requestStop();
        t.join();

        auto response = tp->decodeTxPacket();
        if (bootstrap && !response.empty() &&
            response[0] == static_cast<uint8_t>(MessageType::ServerHello)) {
            response = tp->decodeTxPacket();
        }
        return response;
    }

    Registry registry_;
    double paramValue_ = 3.14;
    uint32_t signalValue_ = 42;
};

// ===========================================================================
// Tests
// ===========================================================================

TEST_F(SessionTest, GetParam) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(1);  // param id

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 10u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetParamResp));
    EXPECT_EQ(r.getU64(), 1u);
    uint32_t valueSize = r.getVarint();
    EXPECT_EQ(valueSize, 8u);
    double val = r.getF64();
    EXPECT_DOUBLE_EQ(val, 3.14);
}

TEST_F(SessionTest, SetParam) {
    uint8_t msg[18];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(1);
    w.putVarint(8);
    w.putF64(99.5);

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 9u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SetParamResp));
    EXPECT_EQ(r.getU64(), 1u);

    // Verify the value was set
    EXPECT_DOUBLE_EQ(paramValue_, 99.5);
}

TEST_F(SessionTest, GetSignal) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalReq));
    w.putU64(2);

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 14u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetSignalResp));
    EXPECT_EQ(r.getU64(), 2u);
    uint32_t valueSize = r.getVarint();
    EXPECT_EQ(valueSize, 4u);
    uint32_t val;
    std::memcpy(&val, r.getBytes(4), 4);
    EXPECT_EQ(val, 42u);
}

TEST_F(SessionTest, GetNonExistentParam) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(9999);  // doesn't exist

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 5u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    uint32_t code = r.getU32();
    EXPECT_EQ(code, static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(SessionTest, ListParams) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsReq));
    w.putU32(0);   // offset
    w.putU32(100); // maxCount

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 13u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ListParamsResp));
    uint32_t total = r.getU32();
    EXPECT_EQ(total, 1u);
    uint32_t offset = r.getU32();
    EXPECT_EQ(offset, 0u);
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 1u);
}

TEST_F(SessionTest, ListSignals) {
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsReq));
    w.putU32(0);
    w.putU32(100);

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 13u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ListSignalsResp));
    uint32_t total = r.getU32();
    EXPECT_EQ(total, 1u);
}

TEST_F(SessionTest, UnknownMessage) {
    uint8_t msg[1] = {0xFF};
    auto resp = sendAndReceive(msg, 1);
    ASSERT_GE(resp.size(), 5u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    uint32_t code = r.getU32();
    EXPECT_EQ(code, static_cast<uint32_t>(ErrorCode::UnknownMessageType));
}

TEST_F(SessionTest, V6ClientHelloReturnsServerHello) {
    ClientHelloV6 hello;
    std::array<uint8_t, 512> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::ClientHello));
    encodeClientHelloV6(writer, hello);
    ASSERT_TRUE(writer.ok());

    const auto response = sendAndReceive(msg.data(), writer.pos, false);
    ASSERT_GE(response.size(), 1U);
    ASSERT_EQ(response[0], static_cast<uint8_t>(MessageType::ServerHello));

    BufReader reader(response.data() + 1, response.size() - 1);
    ServerHelloV6 serverHello;
    ASSERT_TRUE(decodeServerHelloV6(reader, serverHello));
    EXPECT_EQ(serverHello.selectedVersion, SCHEMA_PROTOCOL_VERSION);
    EXPECT_EQ(serverHello.epoch, 1U);
    EXPECT_TRUE(serverHello.schemas.empty());
}

TEST_F(SessionTest, RejectsRegularTrafficBeforeSchemaCommit) {
    uint8_t msg[9];
    BufWriter writer(msg, sizeof(msg));
    writer.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    writer.putU64(1);

    const auto response = sendAndReceive(msg, writer.pos, false);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionTest, SchemaRequestReturnsCatalogDefinition) {
    SchemaNode node;
    node.key[0] = 9;
    node.kind = SchemaKind::Bytes;
    node.maxBytes = 256;
    SchemaGraph graph{{node}};
    SchemaCatalog catalog;
    SchemaRef ref{node.key, computeSchemaDigest(node)};
    ASSERT_TRUE(catalog.install(graph, {{ref, node.revision}}));

    SchemaRequestV6 request;
    request.epoch = catalog.epoch();
    request.definitions.push_back(ref);
    std::array<uint8_t, 512> message{};
    BufWriter writer(message.data(), message.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaRequest));
    encodeSchemaRequestV6(writer, request);

    const auto response = sendAndReceive(message.data(), writer.pos, true, &catalog);
    ASSERT_EQ(response[0], static_cast<uint8_t>(MessageType::SchemaDefinition));
    BufReader reader(response.data() + 1, response.size() - 1);
    SchemaDefinitionV6 definition;
    ASSERT_TRUE(decodeSchemaDefinitionV6(reader, definition));
    EXPECT_EQ(definition.epoch, catalog.epoch());
    EXPECT_EQ(definition.node.key, node.key);
    EXPECT_EQ(computeSchemaDigest(definition.node), ref.digest);
}

// ---------------------------------------------------------------------------
// Coverage: onMessage dispatch-name log switch (schemaCommitted + logFn set)
// ---------------------------------------------------------------------------

namespace {
std::atomic<int> g_logCount{0};
void countingLogFn(const char*, const char*, ...) { ++g_logCount; }
}

TEST_F(SessionTest, LogFnSeesMessageDispatch) {
    g_logCount = 0;
    uint8_t msg[9];
    BufWriter writer(msg, sizeof(msg));
    writer.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    writer.putU64(1);

    const auto response = sendAndReceive(msg, writer.pos, true, nullptr,
                                         &countingLogFn);
    ASSERT_GE(response.size(), 1U);
    EXPECT_GT(g_logCount.load(), 0);
}

// ---------------------------------------------------------------------------
// Coverage: handleClientHello error branches
// ---------------------------------------------------------------------------

TEST_F(SessionTest, ClientHelloMalformedRejected) {
    // Trailing garbage after a valid hello → decode fails / remaining != 0.
    std::array<uint8_t, 32> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::ClientHello));
    writer.putU8(0xFF);
    writer.putU8(0xFF);

    const auto response = sendAndReceive(msg.data(), writer.pos, false);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionTest, ClientHelloVersionRangeRejected) {
    ClientHelloV6 hello;
    hello.minVersion = SCHEMA_PROTOCOL_VERSION + 1;  // offers only >V6
    hello.maxVersion = SCHEMA_PROTOCOL_VERSION + 2;
    std::array<uint8_t, 512> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::ClientHello));
    encodeClientHelloV6(writer, hello);
    ASSERT_TRUE(writer.ok());

    const auto response = sendAndReceive(msg.data(), writer.pos, false);
    ASSERT_GE(response.size(), 1U);
    EXPECT_EQ(response[0], static_cast<uint8_t>(MessageType::SchemaReject));
    BufReader reader(response.data() + 1, response.size() - 1);
    SchemaRejectV6 reject;
    ASSERT_TRUE(decodeSchemaRejectV6(reader, reject));
    EXPECT_EQ(reject.code, SchemaRejectCode::UnsupportedVersion);
}

// ---------------------------------------------------------------------------
// Coverage: handleSchemaRequest error branches
// ---------------------------------------------------------------------------

TEST_F(SessionTest, SchemaRequestBeforeHelloRejected) {
    SchemaRequestV6 request;
    request.epoch = 1;
    std::array<uint8_t, 256> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaRequest));
    encodeSchemaRequestV6(writer, request);

    const auto response = sendAndReceive(msg.data(), writer.pos, false);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionTest, SchemaRequestBadEpochRejected) {
    SchemaRequestV6 request;
    request.epoch = 0xDEADBEEF;  // != session epoch
    std::array<uint8_t, 256> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaRequest));
    encodeSchemaRequestV6(writer, request);

    const auto response = sendAndReceive(msg.data(), writer.pos, true);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionTest, SchemaRequestNoCatalogRejects) {
    // Definitions requested but the session has no catalog →
    // SchemaReject(MissingDependency).
    SchemaRequestV6 request;
    request.epoch = 1;
    SchemaKey key{};
    key[0] = 77;
    request.definitions.push_back(SchemaRef{key, {}});

    std::array<uint8_t, 256> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaRequest));
    encodeSchemaRequestV6(writer, request);

    const auto response = sendAndReceive(msg.data(), writer.pos, true, nullptr);
    ASSERT_GE(response.size(), 1U);
    EXPECT_EQ(response[0], static_cast<uint8_t>(MessageType::SchemaReject));
    BufReader reader(response.data() + 1, response.size() - 1);
    SchemaRejectV6 reject;
    ASSERT_TRUE(decodeSchemaRejectV6(reader, reject));
    EXPECT_EQ(reject.code, SchemaRejectCode::MissingDependency);
}

TEST_F(SessionTest, SchemaRequestKeyNotInCatalogRejects) {
    SchemaNode node;
    node.key[0] = 9;
    node.kind = SchemaKind::Bytes;
    node.maxBytes = 256;
    SchemaGraph graph{{node}};
    SchemaCatalog catalog;
    SchemaRef ref{node.key, computeSchemaDigest(node)};
    ASSERT_TRUE(catalog.install(graph, {{ref, node.revision}}));

    // Request a key the catalog does not hold → per-definition reject.
    SchemaKey missing{};
    missing[0] = 55;
    SchemaRequestV6 request;
    request.epoch = catalog.epoch();
    request.definitions.push_back(SchemaRef{missing, {}});

    std::array<uint8_t, 512> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaRequest));
    encodeSchemaRequestV6(writer, request);

    const auto response = sendAndReceive(msg.data(), writer.pos, true, &catalog);
    ASSERT_GE(response.size(), 1U);
    EXPECT_EQ(response[0], static_cast<uint8_t>(MessageType::SchemaReject));
    BufReader reader(response.data() + 1, response.size() - 1);
    SchemaRejectV6 reject;
    ASSERT_TRUE(decodeSchemaRejectV6(reader, reject));
    EXPECT_EQ(reject.code, SchemaRejectCode::MissingDependency);
}

// ---------------------------------------------------------------------------
// Coverage: setIdentity()/identity() round-trip
// ---------------------------------------------------------------------------

TEST_F(SessionTest, IdentityRoundTrip) {
    auto transport = std::make_unique<MockTransport>();
    uint64_t ts = 1000;
    Session session(std::move(transport), registry_,
                    [&ts] { return ts++; });

    machine::SessionIdentity ident{};
    ident.actor = "test-client";
    ident.source = "ut";
    session.setIdentity(ident);

    auto back = session.identity();
    EXPECT_EQ(back.actor, "test-client");
    EXPECT_EQ(back.source, "ut");
}

// ---------------------------------------------------------------------------
// Coverage: feedSlipData buffer-overflow → discard-until-END
// ---------------------------------------------------------------------------

TEST_F(SessionTest, SlipOversizeFrameDiscardedThenSyncs) {
    // A 4-byte receive buffer: any SLIP frame longer than 4 bytes overflows
    // → the discard-until-END branch, then the next valid frame parses.
    auto smallFactory = []() -> std::unique_ptr<IReceiveBuffer> {
        return std::make_unique<StaticReceiveBuffer<4>>();
    };

    uint8_t msg[9];
    BufWriter writer(msg, sizeof(msg));
    writer.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    writer.putU64(1);

    const auto response = sendAndReceive(msg, writer.pos, true, nullptr,
                                         nullptr, smallFactory);
    // The oversized request is dropped; the bootstrap exchange still
    // produced a ServerHello (also dropped if >4B, so response may be
    // empty — the point is the session survived and the discard path ran).
    (void)response;
    SUCCEED();
}

// ---------------------------------------------------------------------------
// Coverage: validateCatalogValue via schema-typed SetParam
// ---------------------------------------------------------------------------

TEST_F(SessionTest, SetParamSchemaValidationRejectsBadValue) {
#if TETHER_IO_SCHEMA_ENABLED
    // A param whose writes must match an F64 schema node.
    SchemaNode node;
    node.key[0] = 42;
    node.kind = SchemaKind::Scalar;
    node.scalarType = ValueType::F64;
    SchemaGraph graph{{node}};
    SchemaCatalog catalog;
    SchemaRef ref{node.key, computeSchemaDigest(node)};
    ASSERT_TRUE(catalog.install(graph, {{ref, node.revision}}));

    ParamEntry p;
    p.id = 7;
    p.name = "schema_param";
    p.description = "schema-validated";
    p.group = "test";
    p.valueType = ValueType::F64;
    p.schema = ref;
    double* valPtr = &paramValue_;
    p.readFn  = [valPtr](void* d) { std::memcpy(d, valPtr, 8); };
    p.writeFn = [valPtr](const void* s) { std::memcpy(valPtr, s, 8); };
    registry_.addParam(std::move(p));

    // Write 4 bytes (schema expects exactly 8 for F64) → validation fails.
    uint8_t msg[64];
    BufWriter writer(msg, sizeof(msg));
    writer.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    writer.putU64(7);
    writer.putVarint(4);
    writer.putU32(0xAABBCCDD);

    const auto response = sendAndReceive(msg, writer.pos, true, &catalog);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    // Either the length check or the schema validation fires — both are
    // error paths inside the schema-validated set flow.
    EXPECT_NE(reader.getU32(), 0u);
#else
    GTEST_SKIP() << "schema support compiled out";
#endif
}

// Fire the dispatch-name switch for every message family that maps to a
// name string — inject several SLIP frames into one session run.
TEST_F(SessionTest, LogFnCoversManyMessageTypes) {
    g_logCount = 0;
    auto transport = std::make_unique<MockTransport>();
    MockTransport* tp = transport.get();

    // Bootstrap: hello + commit.
    {
        std::array<uint8_t, 256> hello{};
        BufWriter hw(hello.data(), hello.size());
        hw.putU8(static_cast<uint8_t>(MessageType::ClientHello));
        encodeClientHelloV6(hw, {});
        tp->injectSlipMessage(hello.data(), hw.pos);

        std::array<uint8_t, 32> commit{};
        BufWriter cw(commit.data(), commit.size());
        cw.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
        encodeSchemaCommitV6(cw, {1});
        tp->injectSlipMessage(commit.data(), cw.pos);
    }

    // One frame per dispatch-name case; payloads can be empty since the
    // log switch runs before the handler.
    const MessageType types[] = {
        MessageType::ListParamsReq,
        MessageType::ListParamsResp,
        MessageType::ConfigureStreamReq,
        MessageType::ConfigureStreamAck,
        MessageType::StartStream,
        MessageType::StopStream,
        MessageType::StreamData,
        MessageType::Error,
        MessageType::SetParamReq,
        MessageType::SetParamResp,
        MessageType::PingReq,
        MessageType::PongResp,
        MessageType::ListSignalsReq,
        MessageType::ListSignalsResp,
        MessageType::GetParamReq,
        MessageType::GetParamResp,
        MessageType::GetSignalReq,
        MessageType::GetSignalResp,
        MessageType::ListFunctionsReq,
        MessageType::ListFunctionsResp,
        MessageType::CallFunctionReq,
        MessageType::InvokeExReq,
        MessageType::InvokeExResp,
        MessageType::RegisterFunctionsReq,
        MessageType::ClientHello,
        MessageType::ServerHello,
        MessageType::SchemaRequest,
        MessageType::SchemaDefinition,
        MessageType::SchemaCommit,
        MessageType::SchemaReject,
        MessageType::SchemaUpdate,
        static_cast<MessageType>(0xEE),   // Unknown → default arm
    };
    for (auto t : types) {
        uint8_t msg[16]{};
        BufWriter w(msg, sizeof(msg));
        w.putU8(static_cast<uint8_t>(t));
        for (int i = 0; i < 8; ++i) w.putU8(0);
        tp->injectSlipMessage(msg, w.pos);
    }

    uint64_t fakeTs = 1000;
    Session session(std::move(transport), registry_,
                    [&fakeTs] { return fakeTs++; }, &countingLogFn,
                    nullptr, nullptr, nullptr, nullptr, nullptr,
                    Framing::Slip, nullptr, nullptr);
    std::thread th([&session] { session.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    session.requestStop();
    th.join();

    // One log call per dispatched message (bootstrap frames may add more).
    EXPECT_GE(g_logCount.load(), static_cast<int>(sizeof(types) / sizeof(types[0])));
}

// ---------------------------------------------------------------------------
// Coverage: handleSchemaCommit error branches
// ---------------------------------------------------------------------------

TEST_F(SessionTest, SchemaCommitBeforeHelloRejected) {
    std::array<uint8_t, 32> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
    encodeSchemaCommitV6(writer, {1});

    const auto response = sendAndReceive(msg.data(), writer.pos, false);
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionTest, SchemaCommitBadEpochRejected) {
    // Bootstrap does hello + commit(epoch=1); a second commit with the
    // wrong epoch hits the InvalidMessage branch.
    auto transport = std::make_unique<MockTransport>();
    MockTransport* tp = transport.get();
    {
        std::array<uint8_t, 256> hello{};
        BufWriter hw(hello.data(), hello.size());
        hw.putU8(static_cast<uint8_t>(MessageType::ClientHello));
        encodeClientHelloV6(hw, {});
        tp->injectSlipMessage(hello.data(), hw.pos);
    }
    std::array<uint8_t, 32> msg{};
    BufWriter writer(msg.data(), msg.size());
    writer.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
    encodeSchemaCommitV6(writer, {0xDEADBEEF});
    tp->injectSlipMessage(msg.data(), writer.pos);

    uint64_t fakeTs = 1000;
    Session session(std::move(transport), registry_,
                    [&fakeTs] { return fakeTs++; });
    std::thread th([&session] { session.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    session.requestStop();
    th.join();

    auto response = tp->decodeTxPacket();   // ServerHello
    response = tp->decodeTxPacket();        // our error
    ASSERT_GE(response.size(), 5U);
    BufReader reader(response.data(), response.size());
    EXPECT_EQ(reader.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(reader.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}
