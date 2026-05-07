/**
 * @file test_io_session.cpp
 * @brief Unit tests for Session using a mock transport.
 */
#include <gtest/gtest.h>
#include "tether/io/Session.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "SLIPStream/Buffer.hpp"
#include <queue>
#include <mutex>
#include <cstring>
#include <thread>
#include <chrono>

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

    void close() override { connected_ = false; }
    bool isConnected() const override { return connected_; }

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
        auto raw = consumeTx();
        if (raw.empty()) return {};
        size_t decLen = SLIPStream::decoded_length(raw.data(), raw.size());
        if (decLen == SLIPStream::DECODE_ERROR || decLen == 0) return {};
        std::vector<uint8_t> decoded(decLen);
        size_t wrote = SLIPStream::decode_packet(raw.data(), raw.size(),
                                                  decoded.data(), decoded.size());
        if (wrote == SLIPStream::DECODE_ERROR) return {};
        decoded.resize(wrote);
        return decoded;
    }

    bool connected_ = true;

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
    std::vector<uint8_t> sendAndReceive(const uint8_t* msg, size_t msgLen) {
        auto transport = std::make_unique<MockTransport>();
        MockTransport* tp = transport.get();

        // Inject the request
        tp->injectSlipMessage(msg, msgLen);

        uint64_t fakeTs = 1000;
        auto tsFn = [&fakeTs]() -> uint64_t { return fakeTs++; };

        Session session(std::move(transport), registry_, tsFn, nullptr, nullptr, nullptr);

        // Run in a thread, let it process the message, then stop
        std::thread t([&session]() { session.run(); });

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        session.requestStop();
        t.join();

        return tp->decodeTxPacket();
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
    uint8_t vs = r.getU8();
    EXPECT_EQ(vs, 8u);
    double val = r.getF64();
    EXPECT_DOUBLE_EQ(val, 3.14);
}

TEST_F(SessionTest, SetParam) {
    uint8_t msg[17];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(1);
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
    uint8_t vs = r.getU8();
    EXPECT_EQ(vs, 4u);
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

TEST_F(SessionTest, FeatureExchange) {
    FeatureSet clientFeatures;
    clientFeatures.features.push_back(Feature::makeString("client_name", "TestClient"));

    uint8_t msg[256];
    BufWriter w(msg, sizeof(msg));
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    clientFeatures.encode(w);

    auto resp = sendAndReceive(msg, w.pos);
    ASSERT_GE(resp.size(), 5u);

    BufReader r(resp.data(), resp.size());
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::FeatureExchangeResp));
    FeatureSet serverFeatures;
    EXPECT_TRUE(FeatureSet::decode(r, serverFeatures));
    // Should have protocol_version at minimum
    const Feature* pv = serverFeatures.find("protocol_version");
    ASSERT_NE(pv, nullptr);
    EXPECT_EQ(pv->getU32(), PROTOCOL_VERSION);
}
