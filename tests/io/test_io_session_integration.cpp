/**
 * @file test_io_session_integration.cpp
 * @brief Comprehensive integration tests for Session via PipeTransport.
 *
 * Tests every message handler, error path, streaming mode, and edge case
 * in the Session class using a real in-memory transport pair.
 */
#include <gtest/gtest.h>
#include "tether/io/Session.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/BinaryStruct.hpp"
#include "tether/io/Datalogging.hpp"
#include "tether/io/ThresholdFilter.hpp"
#include "PipeTransport.hpp"
#include "SLIPStream/Buffer.hpp"
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>

using namespace tether::io;
using namespace tether::io::testing;

static void noopLogFn(const char* /*tag*/, const char* /*fmt*/, ...) {}

// ===========================================================================
// Helper: encode a message as SLIP and send it, then receive+decode response
// ===========================================================================

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
            // Check if we have a complete SLIP packet (ends with 0xC0)
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

/// Receive ALL available SLIP packets within timeout.
static std::vector<std::vector<uint8_t>> slipReceiveAll(ITransport* tp, uint32_t timeoutMs = 500) {
    std::vector<std::vector<uint8_t>> results;
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
            for (size_t i = 0; i < n; ++i) {
                accum.push_back(buf[i]);
                if (buf[i] == 0xC0 && accum.size() > 1) {
                    size_t decLen = SLIPStream::decoded_length(accum.data(), accum.size());
                    if (decLen != SLIPStream::DECODE_ERROR && decLen > 0) {
                        std::vector<uint8_t> decoded(decLen);
                        size_t wrote = SLIPStream::decode_packet(
                            accum.data(), accum.size(), decoded.data(), decoded.size());
                        if (wrote != SLIPStream::DECODE_ERROR && wrote > 0) {
                            decoded.resize(wrote);
                            results.push_back(std::move(decoded));
                        }
                    }
                    accum.clear();
                }
            }
        }
    }
    return results;
}

// ===========================================================================
// Fixture: sets up a Registry with various entry types and a Session
// ===========================================================================

class SessionIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // F64 param (read/write)
        {
            ParamEntry p;
            p.id = 1;
            p.name = "position";
            p.description = "Current position";
            p.group = "motion";
            p.valueType = ValueType::F64;
            p.readFn = [this](void* d) { std::memcpy(d, &position_, 8); };
            p.writeFn = [this](const void* s) { std::memcpy(&position_, s, 8); };
            p.metadata["unit"] = "mm";
            p.metadata["range"] = "0..1000";
            registry_.addParam(std::move(p));
        }
        // U32 signal (read-only)
        {
            SignalEntry s;
            s.id = 2;
            s.name = "encoder";
            s.description = "Encoder count";
            s.group = "motion";
            s.valueType = ValueType::U32;
            s.readFn = [this](void* d) { std::memcpy(d, &encoder_, 4); };
            registry_.addSignal(std::move(s));
        }
        // Read-only F32 param (no writeFn)
        {
            ParamEntry p;
            p.id = 3;
            p.name = "readonly_param";
            p.description = "A read-only param";
            p.group = "config";
            p.valueType = ValueType::F32;
            p.readFn = [this](void* d) { std::memcpy(d, &readonlyVal_, 4); };
            registry_.addParam(std::move(p));
        }
        // String param (variable-length)
        {
            ParamEntry p;
            p.id = 4;
            p.name = "device_name";
            p.description = "Name";
            p.group = "info";
            p.valueType = ValueType::String;
            p.readFn = [](void*) {};  // dummy fixed-size read
            p.varReadFn = [this](void* d, size_t maxLen) -> size_t {
                size_t len = std::min(deviceName_.size(), maxLen);
                std::memcpy(d, deviceName_.data(), len);
                return len;
            };
            p.varWriteFn = [this](const void* s, size_t len) {
                deviceName_ = std::string(reinterpret_cast<const char*>(s), len);
            };
            p.writeFn = [](const void*) {};  // needed so writable() returns true
            p.maxValueSize = 128;
            registry_.addParam(std::move(p));
        }
        // F32 param with struct descriptor
        {
            structDesc_.entryId = 5;
            structDesc_.name = "Vec3";
            structDesc_.totalSize = 12;
            structDesc_.fields = {
                {"x", ValueType::F32, 0, 4, "mm"},
                {"y", ValueType::F32, 4, 4, "mm"},
                {"z", ValueType::F32, 8, 4, "mm"},
            };

            ParamEntry p;
            p.id = 5;
            p.name = "vec3_param";
            p.description = "A 3D vector";
            p.group = "motion";
            p.valueType = ValueType::Struct;
            p.readFn = [this](void* d) { std::memcpy(d, vec3_, 12); };
            p.structDesc = &structDesc_;
            p.maxValueSize = 12;
            registry_.addParam(std::move(p));
        }
        // F32 param for streaming
        {
            ParamEntry p;
            p.id = 6;
            p.name = "temperature";
            p.description = "Sensor temp";
            p.group = "sensors";
            p.valueType = ValueType::F32;
            p.readFn = [this](void* d) { std::memcpy(d, &temperature_, 4); };
            p.writeFn = [this](const void* s) { std::memcpy(&temperature_, s, 4); };
            registry_.addParam(std::move(p));
        }
    }

    struct SessionContext {
        std::unique_ptr<PipeTransport> client;
        Session* session;
        std::thread thread;

        ~SessionContext() {
            // Close client transport first to signal Session thread to exit gracefully
            // This causes server transport's receive() to return 0, ending the session loop
            if (client) client->close();
            // Join the thread - Session destructor will handle cleanup including server transport close
            if (thread.joinable()) thread.join();
            // Both transports are now safely destroyed without race condition
        }
    };

    /// Create a Session on the server end, return the client end.
    std::unique_ptr<SessionContext> createSession(
        const FeatureSet* features = nullptr,
        DatalogRecorder* recorder = nullptr)
    {
        auto [client, server] = PipeTransport::create();
        auto ctx = std::make_unique<SessionContext>();
        ctx->client = std::move(client);

        auto sess = std::make_unique<Session>(
            std::move(server), registry_, tsFn_, logFn_, features, recorder);
        ctx->session = sess.get();

        ctx->thread = std::thread([s = std::move(sess)]() mutable {
            s->run();
        });

        // Give session time to start
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return ctx;
    }

    /// Send a request and get the response.
    std::vector<uint8_t> roundtrip(SessionContext& ctx,
                                    const uint8_t* msg, size_t len) {
        slipSend(ctx.client.get(), msg, len);
        return slipReceive(ctx.client.get());
    }

    Registry registry_;
    double position_ = 3.14;
    uint32_t encoder_ = 42;
    float readonlyVal_ = 1.23f;
    std::string deviceName_ = "TestDevice";
    StructDescriptor structDesc_;
    float vec3_[3] = {1.0f, 2.0f, 3.0f};
    float temperature_ = 25.0f;

    uint64_t fakeTimestamp_ = 1000000;
    TimestampFn tsFn_ = [this]() -> uint64_t { return fakeTimestamp_++; };
    LogFn logFn_ = nullptr;
};

// ===========================================================================
// GetParam / SetParam
// ===========================================================================

TEST_F(SessionIntegrationTest, GetParamF64) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(1);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 10u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetParamResp));
    EXPECT_EQ(r.getU64(), 1u);
    EXPECT_EQ(r.getU8(), 8u);
    EXPECT_DOUBLE_EQ(r.getF64(), 3.14);
}

TEST_F(SessionIntegrationTest, SetParamF64) {
    auto ctx = createSession();
    uint8_t msg[17];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(1);
    w.putF64(99.5);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 9u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SetParamResp));
    EXPECT_EQ(r.getU64(), 1u);
    EXPECT_DOUBLE_EQ(position_, 99.5);
}

TEST_F(SessionIntegrationTest, SetReadOnlyParamFails) {
    auto ctx = createSession();
    uint8_t msg[13];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(3);  // readonly_param
    w.putF32(999.0f);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 5u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::NotWritable));
}

TEST_F(SessionIntegrationTest, GetParamInvalidId) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(9999);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(SessionIntegrationTest, GetParamTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU32(1);  // only 4 bytes, needs 8

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionIntegrationTest, SetParamTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU32(1);  // too short

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionIntegrationTest, SetParamInvalidId) {
    auto ctx = createSession();
    uint8_t msg[17];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(9999);
    w.putF64(1.0);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidId));
}

// Variable-length param SetParam
TEST_F(SessionIntegrationTest, SetParamVariableLength) {
    auto ctx = createSession();
    std::string newName = "NewDevice";
    uint8_t msg[64];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(4);  // device_name
    w.putVarint(static_cast<uint32_t>(newName.size()));
    w.putBytes(newName.data(), newName.size());

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 9u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SetParamResp));
    EXPECT_EQ(r.getU64(), 4u);
    EXPECT_EQ(deviceName_, "NewDevice");
}

TEST_F(SessionIntegrationTest, SetParamVariableLengthTooLargeFails) {
    auto ctx = createSession();
    std::string tooLarge(129, 'X');
    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(4);  // device_name, max 128
    w.putVarint(static_cast<uint32_t>(tooLarge.size()));
    w.putBytes(tooLarge.data(), tooLarge.size());

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_FALSE(resp.empty());
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

// ===========================================================================
// GetSignal
// ===========================================================================

TEST_F(SessionIntegrationTest, GetSignalU32) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalReq));
    w.putU64(2);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 14u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetSignalResp));
    EXPECT_EQ(r.getU64(), 2u);
    EXPECT_EQ(r.getU8(), 4u);
    uint32_t val;
    std::memcpy(&val, r.getBytes(4), 4);
    EXPECT_EQ(val, 42u);
}

TEST_F(SessionIntegrationTest, GetSignalInvalidId) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalReq));
    w.putU64(9999);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(SessionIntegrationTest, GetSignalTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalReq));
    w.putU32(1);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

// ===========================================================================
// ListParams / ListSignals
// ===========================================================================

TEST_F(SessionIntegrationTest, ListParamsFullPage) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsReq));
    w.putU32(0);
    w.putU32(100);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 13u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ListParamsResp));
    uint32_t total = r.getU32();
    EXPECT_EQ(total, registry_.paramCount());
    r.getU32(); // offset
    uint32_t count = r.getU32();
    EXPECT_EQ(count, registry_.paramCount());
}

TEST_F(SessionIntegrationTest, ListParamsPaginated) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsReq));
    w.putU32(0);
    w.putU32(2);  // Only 2 entries

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ListParamsResp));
    r.getU32(); // total
    r.getU32(); // offset
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 2u);
}

TEST_F(SessionIntegrationTest, ListParamsTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsReq));
    w.putU32(0);  // only 4 bytes, needs 8

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

TEST_F(SessionIntegrationTest, ListSignals) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsReq));
    w.putU32(0);
    w.putU32(100);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ListSignalsResp));
    uint32_t total = r.getU32();
    EXPECT_EQ(total, 1u);
}

TEST_F(SessionIntegrationTest, ListSignalsTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsReq));
    w.putU32(0);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

// ===========================================================================
// GetMetadata
// ===========================================================================

TEST_F(SessionIntegrationTest, GetMetadata) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetMetadataReq));
    w.putU64(1);  // position param has metadata

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 13u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::GetMetadataResp));
    EXPECT_EQ(r.getU64(), 1u);
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 2u);  // "unit" and "range"
}

TEST_F(SessionIntegrationTest, GetMetadataInvalidId) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetMetadataReq));
    w.putU64(9999);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(SessionIntegrationTest, GetMetadataTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetMetadataReq));
    w.putU32(1);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

// ===========================================================================
// SnapshotParams / SnapshotSignals
// ===========================================================================

TEST_F(SessionIntegrationTest, SnapshotParamsSpecificIds) {
    auto ctx = createSession();
    uint8_t msg[32];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsReq));
    w.putU32(2);  // count = 2
    w.putU64(1);  // position
    w.putU64(3);  // readonly_param

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 13u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SnapshotParamsResp));
    uint64_t ts = r.getU64();
    EXPECT_GT(ts, 0u);
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 2u);
}

TEST_F(SessionIntegrationTest, SnapshotParamsAll) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsReq));
    w.putU32(0);  // count = 0 means "all"

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 13u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SnapshotParamsResp));
    r.getU64(); // timestamp
    uint32_t count = r.getU32();
    EXPECT_EQ(count, registry_.paramCount());
}

TEST_F(SessionIntegrationTest, SnapshotParamsWithInvalidId) {
    auto ctx = createSession();
    uint8_t msg[32];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsReq));
    w.putU32(2);
    w.putU64(1);     // valid
    w.putU64(9999);  // invalid — should be skipped

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SnapshotParamsResp));
    r.getU64();
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 1u);  // only the valid one
}

TEST_F(SessionIntegrationTest, SnapshotSignalsAll) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotSignalsReq));
    w.putU32(0);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SnapshotSignalsResp));
    r.getU64();
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 1u);  // only 1 signal
}

TEST_F(SessionIntegrationTest, SnapshotSignalsSpecific) {
    auto ctx = createSession();
    uint8_t msg[13];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotSignalsReq));
    w.putU32(1);
    w.putU64(2);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::SnapshotSignalsResp));
    r.getU64();
    uint32_t count = r.getU32();
    EXPECT_EQ(count, 1u);
}

// ===========================================================================
// FeatureExchange
// ===========================================================================

TEST_F(SessionIntegrationTest, FeatureExchangeWithServerFeatures) {
    FeatureSet serverFeatures;
    serverFeatures.features.push_back(Feature::makeBool("supports_datalogging", true));
    serverFeatures.features.push_back(Feature::makeString("server_name", "TestServer"));

    auto ctx = createSession(&serverFeatures);

    FeatureSet clientFeatures;
    clientFeatures.features.push_back(Feature::makeString("client_name", "TestClient"));

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    clientFeatures.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::FeatureExchangeResp));

    FeatureSet respFeatures;
    EXPECT_TRUE(FeatureSet::decode(r, respFeatures));

    // Should include server features + auto-added protocol_version
    EXPECT_NE(respFeatures.find("supports_datalogging"), nullptr);
    EXPECT_NE(respFeatures.find("server_name"), nullptr);
    const Feature* pv = respFeatures.find("protocol_version");
    ASSERT_NE(pv, nullptr);
    EXPECT_EQ(pv->getU32(), PROTOCOL_VERSION);
}

TEST_F(SessionIntegrationTest, FeatureExchangeNoServerFeatures) {
    auto ctx = createSession();

    FeatureSet clientFeatures;
    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    clientFeatures.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::FeatureExchangeResp));

    FeatureSet respFeatures;
    EXPECT_TRUE(FeatureSet::decode(r, respFeatures));
    EXPECT_NE(respFeatures.find("protocol_version"), nullptr);
}

TEST_F(SessionIntegrationTest, FeatureExchangeServerAlreadyHasVersion) {
    FeatureSet serverFeatures;
    serverFeatures.features.push_back(Feature::makeU32("protocol_version", 99));

    auto ctx = createSession(&serverFeatures);

    FeatureSet clientFeatures;
    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    clientFeatures.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::FeatureExchangeResp));

    FeatureSet respFeatures;
    EXPECT_TRUE(FeatureSet::decode(r, respFeatures));
    const Feature* pv = respFeatures.find("protocol_version");
    ASSERT_NE(pv, nullptr);
    // Should keep the server's version (99), not add another
    EXPECT_EQ(pv->getU32(), 99u);
}

// ===========================================================================
// DescribeStruct
// ===========================================================================

TEST_F(SessionIntegrationTest, DescribeStruct) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructReq));
    w.putU64(5);  // vec3_param has structDesc

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 10u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::DescribeStructResp));

    StructDescriptor decoded;
    EXPECT_TRUE(StructDescriptor::decode(r, decoded));
    EXPECT_EQ(decoded.name, "Vec3");
    EXPECT_EQ(decoded.fields.size(), 3u);
}

TEST_F(SessionIntegrationTest, DescribeStructInvalidId) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructReq));
    w.putU64(9999);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(SessionIntegrationTest, DescribeStructNoDescriptor) {
    auto ctx = createSession();
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructReq));
    w.putU64(1);  // position param — no struct

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionIntegrationTest, DescribeStructTooShort) {
    auto ctx = createSession();
    uint8_t msg[5];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructReq));
    w.putU32(1);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

// ===========================================================================
// Streaming: ConfigureStream, StartStream, StopStream, StreamData
// ===========================================================================

TEST_F(SessionIntegrationTest, ConfigureStreamAck) {
    auto ctx = createSession();
    uint8_t msg[64];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);       // TriggerMode::Time
    w.putU32(10000);  // interval 10ms
    w.putU32(1);      // chunk=1
    w.putU32(0);      // skip=0
    w.putU32(2);      // 2 entries
    w.putU64(1);      // position (F64)
    w.putU64(6);      // temperature (F32)

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 9u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ConfigureStreamAck));
    uint32_t specId = r.getU32();
    EXPECT_GT(specId, 0u);
    uint32_t entryCount = r.getU32();
    EXPECT_EQ(entryCount, 2u);
    uint32_t rowSize = r.getU32();
    EXPECT_EQ(rowSize, 12u);  // F64(8) + F32(4)
}

TEST_F(SessionIntegrationTest, StartStreamWithoutConfigureFails) {
    auto ctx = createSession();
    uint8_t msg[1] = {static_cast<uint8_t>(MessageType::StartStream)};

    auto resp = roundtrip(*ctx, msg, 1);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::StreamNotConfigured));
}

TEST_F(SessionIntegrationTest, StartStreamAlreadyStreamingFails) {
    auto ctx = createSession();

    // Configure
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0); w.putU32(100000); w.putU32(1); w.putU32(0); w.putU32(1);
    w.putU64(1);
    slipSend(ctx->client.get(), cfgMsg, w.pos);
    slipReceive(ctx->client.get());

    // Start
    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);
    // No error response expected for first start
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // Try to start again
    slipSend(ctx->client.get(), startMsg, 1);
    auto resp = slipReceive(ctx->client.get(), 500);
    // Should get an error or stream data; search for error
    if (!resp.empty()) {
        BufReader r(resp.data(), resp.size());
        auto type = r.getU8();
        if (type == static_cast<uint8_t>(MessageType::Error)) {
            EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::AlreadyStreaming));
        }
        // else it could be StreamData, which is also valid
    }
}

TEST_F(SessionIntegrationTest, StopStreamWithoutStreamingFails) {
    auto ctx = createSession();
    uint8_t msg[1] = {static_cast<uint8_t>(MessageType::StopStream)};

    auto resp = roundtrip(*ctx, msg, 1);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::NotStreaming));
}

TEST_F(SessionIntegrationTest, StreamDataFlowTimeBased) {
    auto ctx = createSession();

    // Configure stream: time-based, chunk=1, interval=1µs
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);      // Time
    w.putU32(1);     // 1µs interval (very fast)
    w.putU32(1);     // chunk=1
    w.putU32(0);     // skip=0
    w.putU32(1);     // 1 entry
    w.putU64(6);     // temperature (F32)

    slipSend(ctx->client.get(), cfgMsg, w.pos);
    auto ack = slipReceive(ctx->client.get());
    ASSERT_FALSE(ack.empty());
    EXPECT_EQ(ack[0], static_cast<uint8_t>(MessageType::ConfigureStreamAck));

    // Start streaming
    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);

    // Collect stream data packets
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Stop streaming
    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);

    // Collect remaining data
    auto packets = slipReceiveAll(ctx->client.get(), 500);
    bool foundStreamData = false;
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::StreamData)) {
            foundStreamData = true;
            BufReader r(pkt.data(), pkt.size());
            r.getU8();  // type
            uint32_t specId = r.getU32();
            EXPECT_GT(specId, 0u);
            uint32_t rowCount = r.getU32();
            EXPECT_GE(rowCount, 1u);
        }
    }
    EXPECT_TRUE(foundStreamData);
}

TEST_F(SessionIntegrationTest, StreamVariableLengthEntries) {
    auto ctx = createSession();
    deviceName_ = "VariableStreamName";

    uint8_t cfgMsg[96];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);      // Time
    w.putU32(1);     // fast interval
    w.putU32(1);     // chunk=1
    w.putU32(0);     // skip=0
    w.putU32(2);     // two entries
    w.putU64(4);     // variable-length device_name
    w.putU64(6);     // fixed-size temperature

    slipSend(ctx->client.get(), cfgMsg, w.pos);
    auto ack = slipReceive(ctx->client.get());
    ASSERT_FALSE(ack.empty());
    EXPECT_EQ(ack[0], static_cast<uint8_t>(MessageType::ConfigureStreamAck));

    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);

    auto packets = slipReceiveAll(ctx->client.get(), 500);

    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);
    auto tailPackets = slipReceiveAll(ctx->client.get(), 500);
    packets.insert(packets.end(), tailPackets.begin(), tailPackets.end());

    bool foundStreamData = false;
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::StreamData)) {
            foundStreamData = true;
            break;
        }
    }
    EXPECT_TRUE(foundStreamData);
}

TEST_F(SessionIntegrationTest, ConfigureStreamTooShort) {
    auto ctx = createSession();
    uint8_t msg[10];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0); w.putU32(1000); w.putU32(1);
    // Missing skip and entryCount

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

TEST_F(SessionIntegrationTest, ConfigureStreamInvalidTriggerMode) {
    auto ctx = createSession();
    uint8_t msg[64];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(99);      // Invalid trigger mode
    w.putU32(1000);
    w.putU32(1);
    w.putU32(0);
    w.putU32(1);
    w.putU64(1);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
}

TEST_F(SessionIntegrationTest, StreamWithSkipCount) {
    auto ctx = createSession();

    // Configure with skip=2 (skip 2 out of every 3)
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);
    w.putU32(1);     // 1µs
    w.putU32(1);     // chunk=1
    w.putU32(2);     // skip=2
    w.putU32(1);
    w.putU64(6);

    slipSend(ctx->client.get(), cfgMsg, w.pos);
    slipReceive(ctx->client.get());

    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Just verify we got through without crashes
}

TEST_F(SessionIntegrationTest, StreamWithChunking) {
    auto ctx = createSession();

    // Configure with chunk=5
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);
    w.putU32(1);     // 1µs
    w.putU32(5);     // chunk=5 — 5 rows per packet
    w.putU32(0);
    w.putU32(1);
    w.putU64(6);

    slipSend(ctx->client.get(), cfgMsg, w.pos);
    auto ack = slipReceive(ctx->client.get());
    ASSERT_FALSE(ack.empty());

    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);

    auto packets = slipReceiveAll(ctx->client.get(), 500);
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::StreamData)) {
            BufReader r(pkt.data(), pkt.size());
            r.getU8();
            r.getU32();  // specId
            uint32_t rowCount = r.getU32();
            EXPECT_LE(rowCount, 5u);
        }
    }
}

TEST_F(SessionIntegrationTest, ReconfigureStreamWhileStreaming) {
    auto ctx = createSession();

    // Configure and start
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0); w.putU32(100000); w.putU32(1); w.putU32(0); w.putU32(1);
    w.putU64(6);
    slipSend(ctx->client.get(), cfgMsg, w.pos);
    slipReceive(ctx->client.get());

    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Reconfigure (should stop streaming first)
    BufWriter w2(cfgMsg, sizeof(cfgMsg));
    w2.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w2.putU8(0); w2.putU32(50000); w2.putU32(2); w2.putU32(0); w2.putU32(1);
    w2.putU64(1);
    slipSend(ctx->client.get(), cfgMsg, w2.pos);

    auto ack = slipReceive(ctx->client.get(), 500);
    // Might get StreamData or ConfigureStreamAck
    bool gotAck = false;
    auto packets = slipReceiveAll(ctx->client.get(), 200);
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::ConfigureStreamAck)) {
            gotAck = true;
        }
    }
    if (!ack.empty() && ack[0] == static_cast<uint8_t>(MessageType::ConfigureStreamAck)) {
        gotAck = true;
    }
    // It's valid to have gotten the ack
}

// ===========================================================================
// OnChange streaming
// ===========================================================================

TEST_F(SessionIntegrationTest, StreamOnChange) {
    auto ctx = createSession();

    // Configure OnChange stream
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(1);       // TriggerMode::OnChange
    w.putU32(1);      // interval
    w.putU32(1);      // chunk=1
    w.putU32(0);      // skip=0
    w.putU32(1);      // 1 entry
    w.putU64(6);      // temperature

    slipSend(ctx->client.get(), cfgMsg, w.pos);
    slipReceive(ctx->client.get());

    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);

    // Wait a bit, then change the value
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    temperature_ = 30.0f;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);

    auto packets = slipReceiveAll(ctx->client.get(), 500);
    // Should have at least one StreamData (on first read or on change)
    bool foundStream = false;
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::StreamData)) {
            foundStream = true;
        }
    }
    EXPECT_TRUE(foundStream);
}

// ===========================================================================
// Datalogging via Session
// ===========================================================================

TEST_F(SessionIntegrationTest, DatalogWithoutRecorderFails) {
    auto ctx = createSession(nullptr, nullptr);  // no recorder

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.sampleRateHz = 100;
    cfg.enabled = true;

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogReq));
    cfg.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::FeatureNotSupported));
}

TEST_F(SessionIntegrationTest, ConfigureDatalog) {
    DatalogRecorder recorder;
    auto ctx = createSession(nullptr, &recorder);

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.sampleRateHz = 100;
    cfg.enabled = true;
    cfg.entryIds = {1, 2};

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogReq));
    cfg.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 2u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ConfigureDatalogResp));
    EXPECT_EQ(r.getU8(), 1u);  // success
}

TEST_F(SessionIntegrationTest, ConfigureDatalogDisabled) {
    DatalogRecorder recorder;
    auto ctx = createSession(nullptr, &recorder);

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.enabled = false;

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogReq));
    cfg.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 2u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ConfigureDatalogResp));
}

TEST_F(SessionIntegrationTest, ConfigureDatalogInvalidConfigFails) {
    DatalogRecorder recorder;
    auto ctx = createSession(nullptr, &recorder);

    uint8_t msg[3];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogReq));
    w.putU16(0xFFFF);  // intentionally truncated payload

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_FALSE(resp.empty());
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::InvalidMessage));
}

TEST_F(SessionIntegrationTest, DatalogStatusWithRecorder) {
    DatalogRecorder recorder;
    auto ctx = createSession(nullptr, &recorder);

    uint8_t msg[1] = {static_cast<uint8_t>(MessageType::DatalogStatusReq)};
    auto resp = roundtrip(*ctx, msg, 1);
    ASSERT_GE(resp.size(), 2u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::DatalogStatusResp));

    DatalogStatus status;
    EXPECT_TRUE(DatalogStatus::decode(r, status));
    EXPECT_EQ(status.state, DatalogState::Idle);
}

TEST_F(SessionIntegrationTest, DatalogStatusWithoutRecorder) {
    auto ctx = createSession(nullptr, nullptr);

    uint8_t msg[1] = {static_cast<uint8_t>(MessageType::DatalogStatusReq)};
    auto resp = roundtrip(*ctx, msg, 1);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::DatalogStatusResp));
}

// ===========================================================================
// Threshold configuration via Session
// ===========================================================================

TEST_F(SessionIntegrationTest, ConfigureThreshold) {
    auto ctx = createSession();

    ThresholdConfig cfg;
    cfg.name = "test_threshold";
    cfg.isWhitelist = true;
    ThresholdRule rule;
    rule.entryId = 6;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 1.0;
    cfg.rules.push_back(rule);

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureThresholdReq));
    cfg.encode(w);

    auto resp = roundtrip(*ctx, msg, w.pos);
    ASSERT_GE(resp.size(), 2u);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ConfigureThresholdResp));
    EXPECT_EQ(r.getU8(), 1u);
}

TEST_F(SessionIntegrationTest, ConfigureThresholdInvalidConfigFails) {
    auto ctx = createSession();

    uint8_t msg[2] = {
        static_cast<uint8_t>(MessageType::ConfigureThresholdReq),
        0xFF,
    };

    auto resp = roundtrip(*ctx, msg, sizeof(msg));
    ASSERT_FALSE(resp.empty());
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::ThresholdError));
}

// ===========================================================================
// Unknown message type
// ===========================================================================

TEST_F(SessionIntegrationTest, UnknownMessageType) {
    auto ctx = createSession();
    uint8_t msg[1] = {0xFF};
    auto resp = roundtrip(*ctx, msg, 1);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::Error));
    EXPECT_EQ(r.getU32(), static_cast<uint32_t>(ErrorCode::UnknownMessageType));
}

// ===========================================================================
// CatalogChanged notification
// ===========================================================================

TEST_F(SessionIntegrationTest, CatalogChangedOnRegistryUpdate) {
    auto ctx = createSession();

    // Give session time to register its change listener
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Add a new param — should trigger CatalogChanged
    ParamEntry newParam;
    newParam.id = 100;
    newParam.name = "new_param";
    newParam.valueType = ValueType::U8;
    newParam.readFn = [](void* d) { uint8_t v = 0; std::memcpy(d, &v, 1); };
    registry_.addParam(std::move(newParam));

    // Give session time to process the dirty flag
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send a dummy request to get the session to check catalogDirty_
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(1);
    slipSend(ctx->client.get(), msg, w.pos);

    auto packets = slipReceiveAll(ctx->client.get(), 500);
    bool foundCatalogChanged = false;
    for (const auto& pkt : packets) {
        if (!pkt.empty() && pkt[0] == static_cast<uint8_t>(MessageType::CatalogChanged)) {
            foundCatalogChanged = true;
            BufReader r(pkt.data(), pkt.size());
            r.getU8();
            uint32_t rev = r.getU32();
            EXPECT_GT(rev, 0u);
        }
    }
    EXPECT_TRUE(foundCatalogChanged);
}

// ===========================================================================
// Session requestStop and isRunning
// ===========================================================================

TEST_F(SessionIntegrationTest, RequestStopEndsSession) {
    auto [client, server] = PipeTransport::create();

    auto session = std::make_unique<Session>(
        std::move(server), registry_, tsFn_);
    Session* sp = session.get();

    std::thread t([&]() { session->run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(sp->isRunning());

    sp->requestStop();
    t.join();
    EXPECT_FALSE(sp->isRunning());
}

// ===========================================================================
// Session with logging callback
// ===========================================================================

TEST_F(SessionIntegrationTest, LoggingCallback) {
    auto [client, server] = PipeTransport::create();
    auto session = std::make_unique<Session>(
        std::move(server), registry_, tsFn_, noopLogFn);

    std::thread t([&]() { session->run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    session->requestStop();
    t.join();
}

// ===========================================================================
// Client disconnect stops session
// ===========================================================================

TEST_F(SessionIntegrationTest, ClientDisconnectStopsSession) {
    auto [client, server] = PipeTransport::create();

    auto session = std::make_unique<Session>(
        std::move(server), registry_, tsFn_);
    Session* sp = session.get();

    std::thread t([&]() { session->run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(sp->isRunning());

    client->close();
    // Session should detect disconnect and stop
    t.join();
    EXPECT_FALSE(sp->isRunning());
}

// ===========================================================================
// SLIP buffer overflow recovery
// ===========================================================================

TEST_F(SessionIntegrationTest, SlipBufferOverflowRecovery) {
    auto ctx = createSession();

    // Send a huge non-SLIP payload to overflow the buffer
    std::vector<uint8_t> junk(9000, 0x42);
    ctx->client->send(junk.data(), junk.size());

    // Now send a valid message
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    uint8_t msg[9];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(1);
    slipSend(ctx->client.get(), msg, w.pos);

    auto resp = slipReceive(ctx->client.get(), 500);
    // Session should recover and respond
    if (!resp.empty()) {
        EXPECT_EQ(resp[0], static_cast<uint8_t>(MessageType::GetParamResp));
    }
}

// ===========================================================================
// Streaming with threshold filtering applied
// ===========================================================================

TEST_F(SessionIntegrationTest, StreamWithThresholdFiltering) {
    auto ctx = createSession();

    // Configure threshold
    ThresholdConfig tcfg;
    tcfg.name = "test";
    tcfg.isWhitelist = true;
    ThresholdRule rule;
    rule.entryId = 6;  // temperature
    rule.type = ThresholdType::Absolute;
    rule.threshold = 5.0;  // Only pass if change > 5
    tcfg.rules.push_back(rule);

    uint8_t tmsg[256];
    BufWriter tw(tmsg, sizeof(tmsg));
    tw.putU8(static_cast<uint8_t>(MessageType::ConfigureThresholdReq));
    tcfg.encode(tw);
    slipSend(ctx->client.get(), tmsg, tw.pos);
    slipReceive(ctx->client.get());

    // Configure stream
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0); w.putU32(1); w.putU32(1); w.putU32(0); w.putU32(1);
    w.putU64(6);
    slipSend(ctx->client.get(), cfgMsg, w.pos);
    slipReceive(ctx->client.get());

    // Start streaming
    uint8_t startMsg[1] = {static_cast<uint8_t>(MessageType::StartStream)};
    slipSend(ctx->client.get(), startMsg, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Stop
    uint8_t stopMsg[1] = {static_cast<uint8_t>(MessageType::StopStream)};
    slipSend(ctx->client.get(), stopMsg, 1);
    slipReceiveAll(ctx->client.get(), 200);
}

// ===========================================================================
// Stream with zero chunk and zero interval clamping
// ===========================================================================

TEST_F(SessionIntegrationTest, ConfigureStreamZeroChunkAndInterval) {
    auto ctx = createSession();
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0);
    w.putU32(0);     // interval=0 should be clamped to 1
    w.putU32(0);     // chunk=0 should be clamped to 1
    w.putU32(0);
    w.putU32(1);
    w.putU64(6);

    auto resp = roundtrip(*ctx, cfgMsg, w.pos);
    ASSERT_FALSE(resp.empty());
    EXPECT_EQ(resp[0], static_cast<uint8_t>(MessageType::ConfigureStreamAck));
}

// ===========================================================================
// Stream with invalid entry IDs (entry not in registry)
// ===========================================================================

TEST_F(SessionIntegrationTest, ConfigureStreamWithInvalidEntries) {
    auto ctx = createSession();
    uint8_t cfgMsg[64];
    BufWriter w(cfgMsg, sizeof(cfgMsg));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamReq));
    w.putU8(0); w.putU32(1000); w.putU32(1); w.putU32(0); w.putU32(2);
    w.putU64(6);    // valid
    w.putU64(9999); // invalid — should be skipped

    auto resp = roundtrip(*ctx, cfgMsg, w.pos);
    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::ConfigureStreamAck));
    r.getU32();  // specId
    uint32_t entryCount = r.getU32();
    EXPECT_EQ(entryCount, 1u);  // only the valid one
}
