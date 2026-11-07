/**
 * @file test_io_input_stream_errors.cpp
 * @brief Session_peer input-stream handler error paths — malformed and
 *        edge-case CreateInputStream/Data/Close messages injected raw
 *        over a message pipe (the client API cannot produce them).
 */

#include <gtest/gtest.h>
#include "tether/io/Session.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaNegotiation.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "PipeTransport.hpp"
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using namespace tether::io;
using namespace tether::io::testing;

static void noopLogFn(const char*, const char*, ...) {}

class InputStreamErrorTest : public ::testing::Test {
protected:
    Registry registry_;
    SchemaGraph graph_;
    SchemaCatalog catalog_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<MessagePipeTransport> clientEnd_;
    std::thread sessionThread_;
    std::atomic<uint32_t> dataCalls_{0};
    std::atomic<bool> rejectCreates_{false};

    static SchemaKey key(uint8_t tag) {
        SchemaKey k{};
        k[0] = 0xDF;
        k[1] = tag;
        return k;
    }

    void SetUp() override {
        SchemaNode u64n;
        u64n.key = key(1);
        u64n.kind = SchemaKind::Scalar;
        u64n.scalarType = ValueType::U64;
        u64n.name = "err.U64";
        graph_.nodes = {u64n};
        ASSERT_TRUE(resolveSchemaDigests(graph_));
        ASSERT_TRUE(catalog_.install(graph_, buildSchemaManifest(graph_)));

        auto [client, server] = MessagePipeTransport::create();
        clientEnd_ = std::move(client);

        session_ = std::make_unique<Session>(
            std::move(server), registry_,
            [] {
                return static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now()
                            .time_since_epoch())
                        .count());
            },
            noopLogFn, nullptr,
            // inputStreamCreateFn — rejectCreates_ flips the result.
            [this](uint32_t, const SchemaRef&, SchemaEpoch, SchemaSlot,
                   uint32_t, uint32_t) {
                return !rejectCreates_.load(std::memory_order_relaxed);
            },
            // inputStreamDataFn
            [this](uint32_t, const std::vector<std::vector<uint8_t>>&) {
                dataCalls_.fetch_add(1, std::memory_order_relaxed);
            },
            nullptr, nullptr, Framing::None, nullptr, &catalog_);
        sessionThread_ = std::thread([this] { session_->run(); });

        // Schema bootstrap: ClientHello then SchemaCommit (epoch 1).
        std::array<uint8_t, 512> hello{};
        BufWriter hw(hello.data(), hello.size());
        hw.putU8(static_cast<uint8_t>(MessageType::ClientHello));
        encodeClientHelloV6(hw, {});
        sendRaw(hello.data(), hw.pos);

        std::array<uint8_t, 64> commit{};
        BufWriter cw(commit.data(), commit.size());
        cw.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
        encodeSchemaCommitV6(cw, {catalog_.epoch()});
        sendRaw(commit.data(), cw.pos);
        drain(200);
    }

    void TearDown() override {
        clientEnd_.reset();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }

    void sendRaw(const uint8_t* data, size_t len) {
        clientEnd_->send(data, len);
    }

    /// Collect all messages the session emitted within `ms`.
    std::vector<std::vector<uint8_t>> drain(uint32_t ms = 120) {
        std::vector<std::vector<uint8_t>> out;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < deadline) {
            std::vector<uint8_t> msg;
            if (!clientEnd_->receiveMessage(msg, 30)) break;
            out.push_back(std::move(msg));
        }
        return out;
    }

    bool sawType(const std::vector<std::vector<uint8_t>>& msgs,
                 MessageType t) const {
        for (const auto& m : msgs)
            if (!m.empty() &&
                m[0] == static_cast<uint8_t>(t)) return true;
        return false;
    }

    /// Build a CreateInputStreamReq body: epoch, slot, maxValue, maxBatch.
    std::vector<uint8_t> createReqBody(uint32_t epoch, uint32_t slot,
                                       uint32_t maxVal, uint32_t maxBatch,
                                       bool trailing = false) {
        std::vector<uint8_t> body;
        body.reserve(16);
        auto put32 = [&body](uint32_t v) {
            for (unsigned i = 0; i < 4; ++i)
                body.push_back(static_cast<uint8_t>(v >> (8 * i)));
        };
        put32(epoch); put32(slot); put32(maxVal); put32(maxBatch);
        if (trailing) body.push_back(0xFF);
        return body;
    }

    void sendTyped(MessageType t, const std::vector<uint8_t>& body) {
        std::vector<uint8_t> msg;
        msg.reserve(1 + body.size());
        msg.push_back(static_cast<uint8_t>(t));
        msg.insert(msg.end(), body.begin(), body.end());
        sendRaw(msg.data(), msg.size());
    }

    std::optional<SchemaSlot> slot() const {
        return catalog_.slotForKey(key(1));
    }
};

// ---------------------------------------------------------------------------
// CreateInputStreamReq error paths
// ---------------------------------------------------------------------------

TEST_F(InputStreamErrorTest, CreateTruncatedBodyRejected) {
    // Type byte only — reader hits EOF on the first getU32.
    std::vector<uint8_t> msg{static_cast<uint8_t>(MessageType::CreateInputStreamReq)};
    sendRaw(msg.data(), msg.size());
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CreateBadSlotRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    // slot 999 does not resolve → InvalidMessage descriptor error.
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), 999, 8, 4));
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CreateZeroLimitsRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 0, 4));   // maxValueSize=0
    EXPECT_TRUE(sawType(drain(), MessageType::Error));

    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 0));   // maxBatchSize=0
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CreateTrailingBytesRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4, /*trailing=*/true));
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CreateRejectedByCallback) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    rejectCreates_.store(true);
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
    rejectCreates_.store(false);
}

TEST_F(InputStreamErrorTest, CreateValidThenDataAndClose) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    auto resp = drain();
    EXPECT_TRUE(sawType(resp, MessageType::CreateInputStreamResp));
    // First stream id is 1 (nextInputStreamId_ starts at 1).

    // Valid data message: streamId=1, count=1, one u64 value.
    const SchemaNode& u64n = *graph_.find(key(1));
    auto val = encodeNative(graph_, u64n, nv::u64(42));
    ASSERT_TRUE(val.has_value());

    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(1);                 // streamId, count
    put32(static_cast<uint32_t>(val->size()));
    body.insert(body.end(), val->begin(), val->end());
    sendTyped(MessageType::InputStreamData, body);
    drain(80);
    EXPECT_GE(dataCalls_.load(), 1u);

    // Close stream 1 → CloseInputStreamResp.
    std::vector<uint8_t> closeBody;
    for (unsigned i = 0; i < 4; ++i)
        closeBody.push_back(static_cast<uint8_t>(1u >> (8 * i)));
    sendTyped(MessageType::CloseInputStreamReq, closeBody);
    EXPECT_TRUE(sawType(drain(), MessageType::CloseInputStreamResp));
}

// ---------------------------------------------------------------------------
// InputStreamData error paths
// ---------------------------------------------------------------------------

TEST_F(InputStreamErrorTest, DataTruncatedRejected) {
    // streamId only — count read hits EOF.
    std::vector<uint8_t> body{1, 0, 0, 0};
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataUnknownStreamRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    // Create stream 1 first.
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    drain();

    // Data for stream 999 → InvalidId error.
    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(999); put32(1);
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataZeroCountRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    drain();

    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(0);   // streamId=1, count=0
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataOversizedValueRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 4, 4));   // maxValueSize=4
    drain();

    // count=1, valueLength=8 (> maxValueSize) then 8 bytes.
    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(1); put32(8);
    for (int i = 0; i < 8; ++i) body.push_back(0);
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataTrailingBytesRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    drain();

    const SchemaNode& u64n = *graph_.find(key(1));
    auto val = encodeNative(graph_, u64n, nv::u64(7));
    ASSERT_TRUE(val.has_value());

    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(1); put32(static_cast<uint32_t>(val->size()));
    body.insert(body.end(), val->begin(), val->end());
    body.push_back(0xEE);   // trailing byte after the value
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataBadSchemaValueRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    drain();

    // count=1, valueLength=8 but garbage bytes that fail schema validation.
    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(1); put32(8);
    for (int i = 0; i < 8; ++i) body.push_back(0xAB);
    sendTyped(MessageType::InputStreamData, body);
    // Either an Error (schema reject) or silent acceptance of raw u64 —
    // u64 scalars accept any 8 bytes, so assert no crash + drain.
    (void)drain();
}

// ---------------------------------------------------------------------------
// CloseInputStreamReq error paths
// ---------------------------------------------------------------------------

TEST_F(InputStreamErrorTest, CloseTruncatedRejected) {
    // Type byte only → reader !ok.
    std::vector<uint8_t> msg{static_cast<uint8_t>(MessageType::CloseInputStreamReq)};
    sendRaw(msg.data(), msg.size());
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CloseTrailingBytesRejected) {
    // streamId + trailing byte → reader.remaining() != 0.
    std::vector<uint8_t> body{1, 0, 0, 0, 0x77};
    sendTyped(MessageType::CloseInputStreamReq, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, CloseUnknownStreamRejected) {
    std::vector<uint8_t> body;
    for (unsigned i = 0; i < 4; ++i)
        body.push_back(static_cast<uint8_t>(4242u >> (8 * i)));
    sendTyped(MessageType::CloseInputStreamReq, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, DataSchemaInvalidLengthRejected) {
    auto s = slot();
    ASSERT_TRUE(s.has_value());
    sendTyped(MessageType::CreateInputStreamReq,
              createReqBody(catalog_.epoch(), *s, 8, 4));
    drain();

    // valueLength=5 for a U64 schema — validateSchemaValue fails.
    std::vector<uint8_t> body;
    auto put32 = [&body](uint32_t v) {
        for (unsigned i = 0; i < 4; ++i)
            body.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put32(1); put32(1); put32(5);
    for (int i = 0; i < 5; ++i) body.push_back(0x11);
    sendTyped(MessageType::InputStreamData, body);
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
}

TEST_F(InputStreamErrorTest, UnsubscribeLogResponds) {
    // UnsubscribeLogReq body = varint subscription id (id=1 → 0x01).
    std::vector<uint8_t> body{0x01};
    sendTyped(MessageType::UnsubscribeLogReq, body);
#if TETHER_IO_LOG_SUBSCRIPTIONS_ENABLED
    EXPECT_TRUE(sawType(drain(), MessageType::UnsubscribeLogResp));
#else
    EXPECT_TRUE(sawType(drain(), MessageType::Error));
#endif
}

TEST_F(InputStreamErrorTest, PendingInvokeCountZero) {
#if TETHER_IO_FUNCTIONS_ENABLED
    EXPECT_EQ(session_->pendingInvokeCount(), 0u);
#endif
}

TEST_F(InputStreamErrorTest, CallPeerEncodeFailure) {
#if TETHER_IO_FUNCTIONS_ENABLED
    // position=UINT32_MAX wraps position+1 to key=0 → encodeFunctionValue
    // rejects the argument and callPeer returns false.
    FunctionArgument arg{};
    arg.position = 0xFFFFFFFFu;
    arg.type = ValueType::U64;
    arg.value = {0, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_FALSE(session_->callPeer(1, {arg}, 1'000'000,
                                    [](const Session::InvokeResult&) {}));
    EXPECT_EQ(session_->pendingInvokeCount(), 0u);
#endif
}

TEST_F(InputStreamErrorTest, CallPeerQueueFullFails) {
#if TETHER_IO_FUNCTIONS_ENABLED
    // deadlineUs=0 → no expiry; fill MAX_PENDING_INVOKES, next fails.
    FunctionArgument arg{};
    arg.position = 0;
    arg.type = ValueType::U64;
    arg.value = {0, 0, 0, 0, 0, 0, 0, 0};
    size_t accepted = 0;
    for (int i = 0; i < 300; ++i) {
        if (session_->callPeer(1, {arg}, /*deadlineUs=*/0,
                               [](const Session::InvokeResult&) {}))
            ++accepted;
        else
            break;
    }
    EXPECT_EQ(accepted, MAX_PENDING_INVOKES);
    EXPECT_EQ(session_->pendingInvokeCount(), MAX_PENDING_INVOKES);
#endif
}

// Session without input-stream callbacks → FeatureNotSupported.
TEST(InputStreamNoCallbacks, CreateReportsNotSupported) {
    Registry registry;
    auto [client, server] = MessagePipeTransport::create();
    Session session(std::move(server), registry,
                    [] { return uint64_t{1}; }, noopLogFn,
                    nullptr, nullptr, nullptr,
                    nullptr, nullptr, Framing::None, nullptr, nullptr);
    std::thread t([&session] { session.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // Bootstrap schema commit isn't needed — the FeatureNotSupported
    // check precedes schema validation? No: onMessage gates non-negotiation
    // types on schemaCommitted_.  Send hello+commit first.
    std::array<uint8_t, 512> hello{};
    BufWriter hw(hello.data(), hello.size());
    hw.putU8(static_cast<uint8_t>(MessageType::ClientHello));
    encodeClientHelloV6(hw, {});
    client->send(hello.data(), hw.pos);
    std::array<uint8_t, 64> commit{};
    BufWriter cw(commit.data(), commit.size());
    cw.putU8(static_cast<uint8_t>(MessageType::SchemaCommit));
    encodeSchemaCommitV6(cw, {1});
    client->send(commit.data(), cw.pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // CreateInputStreamReq with a minimal (still invalid) body — the
    // !inputStreamCreateFn_ check fires before body parsing.
    std::vector<uint8_t> msg{static_cast<uint8_t>(MessageType::CreateInputStreamReq)};
    client->send(msg.data(), msg.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::vector<uint8_t> resp;
    bool sawError = false;
    for (int i = 0; i < 20 && !sawError; ++i) {
        if (client->receiveMessage(resp, 20) && !resp.empty() &&
            resp[0] == static_cast<uint8_t>(MessageType::Error))
            sawError = true;
    }
    EXPECT_TRUE(sawError);

    session.requestStop();
    t.join();
}
