/**
 * @file test_io_ring_stream.cpp
 * @brief Tests for ring-buffered stream sources in the IO session:
 *        schema binding, row projection, exclusive acquire, capacity-1
 *        best-effort semantics, and NoStream entry exclusion.
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "tether/io/Session.hpp"
#include "tether/io/RingStreamSource.hpp"
#include "SLIPStream/Buffer.hpp"

using namespace tether::io;

// ---------------------------------------------------------------------------
// Mock transport (same shape as test_io_session.cpp's MockTransport)
// ---------------------------------------------------------------------------
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

    void injectSlipMessage(const uint8_t* msg, size_t len) {
        size_t encLen = SLIPStream::encoded_length(msg, len);
        std::vector<uint8_t> enc(encLen);
        SLIPStream::encode_packet(msg, len, enc.data(), enc.size());
        std::lock_guard<std::mutex> lock(mutex_);
        rxData_.insert(rxData_.end(), enc.begin(), enc.end());
    }

    /// Decode and return all complete SLIP packets currently in txData_.
    /// libSLIPspeed frames are "...payload... 0xC0" (possibly preceded by a
    /// leading 0xC0); the decoder requires the END byte to be included.
    std::vector<std::vector<uint8_t>> drainTxPackets() {
        std::vector<std::vector<uint8_t>> out;
        std::lock_guard<std::mutex> lock(mutex_);
        size_t pos = 0;
        while (pos < txData_.size()) {
            size_t end = pos;
            while (end < txData_.size() && txData_[end] != 0xC0) ++end;
            if (end >= txData_.size()) break;          // incomplete packet
            const uint8_t* frame = txData_.data() + pos;
            const size_t frameLen = end + 1 - pos;     // include END
            pos = end + 1;
            size_t decLen = SLIPStream::decoded_length(frame, frameLen);
            if (decLen == SLIPStream::DECODE_ERROR || decLen == 0) continue;
            std::vector<uint8_t> dec(decLen);
            size_t wrote = SLIPStream::decode_packet(frame, frameLen,
                                                      dec.data(), dec.size());
            if (wrote != SLIPStream::DECODE_ERROR && wrote > 0) {
                dec.resize(wrote);
                out.push_back(std::move(dec));
            }
        }
        txData_.erase(txData_.begin(), txData_.begin() + static_cast<ptrdiff_t>(pos));
        return out;
    }

    std::atomic<bool> connected_{true};

private:
    mutable std::mutex mutex_;
    std::vector<uint8_t> rxData_;
    std::vector<uint8_t> txData_;
};

// ---------------------------------------------------------------------------
// Test ring source: 3 fields (u16, u32, i8) over a small ring
// ---------------------------------------------------------------------------
struct __attribute__((packed)) TestRow {
    uint64_t ts_us;
    uint16_t field_a;   // entry id 0x1001
    uint32_t field_b;   // entry id 0x1002
    int8_t   field_c;   // entry id 0x1003
};

static constexpr uint64_t kIdA = 0x1001;
static constexpr uint64_t kIdB = 0x1002;
static constexpr uint64_t kIdC = 0x1003;

template <size_t Cap>
using TestRingSource = SpscRingStreamSource<TestRow, Cap>;

static std::unique_ptr<TestRingSource<8>> makeSource() {
    auto src = std::make_unique<TestRingSource<8>>();
    src->setSchema({kIdA, kIdB, kIdC}, {2, 4, 1});
    return src;
}

// ---------------------------------------------------------------------------
// Fixture: registry with 3 ring-backed signals + 1 plain polled signal
// ---------------------------------------------------------------------------
class RingStreamTest : public ::testing::Test {
protected:
    void SetUp() override {
        registry_.addSignal({
            kIdA, "test.a", "field a", "test", ValueType::U16,
            [](void* d) { uint16_t v = 0xAAAA; std::memcpy(d, &v, 2); }
        });
        registry_.addSignal({
            kIdB, "test.b", "field b", "test", ValueType::U32,
            [](void* d) { uint32_t v = 0xBBBBBBBB; std::memcpy(d, &v, 4); }
        });
        registry_.addSignal({
            kIdC, "test.c", "field c", "test", ValueType::I8,
            [](void* d) { int8_t v = -7; std::memcpy(d, &v, 1); }
        });
        registry_.addSignal({
            kIdOther, "test.other", "not in ring", "test", ValueType::U8,
            [](void* d) { uint8_t v = 0x55; std::memcpy(d, &v, 1); }
        });
        // SDO-style entry: readable but excluded from streams.
        ParamEntry p;
        p.id = kIdSdo; p.name = "test.sdo"; p.group = "test";
        p.valueType = ValueType::U16;
        p.readFn = [](void* d) { uint16_t v = 0x1234; std::memcpy(d, &v, 2); };
        p.extraFlags = EntryFlags::NoStream;
        registry_.addParam(std::move(p));

        src_ = makeSource();
        ringSources_.push_back(src_.get());
    }

    /// Build a ConfigureStreamReq body for the given entry ids.
    static std::vector<uint8_t> configureStreamMsg(
        const std::vector<uint64_t>& ids, uint32_t intervalMs = 1,
        uint32_t chunk = 8, uint32_t skip = 0, uint8_t trig = 0) {
        std::vector<uint8_t> msg(64 + ids.size() * 8);
        BufWriter w(msg.data(), msg.size());
        w.putU8(static_cast<uint8_t>(MessageType::ConfigureStream));
        w.putU8(trig);
        w.putU32(intervalMs);
        w.putU32(chunk);
        w.putU32(skip);
        w.putU64(0);                       // triggerEntryId
        w.putU32(static_cast<uint32_t>(ids.size()));
        for (uint64_t id : ids) w.putU64(id);
        w.putU32(0);                       // filter count
        msg.resize(w.pos);
        return msg;
    }

    static std::vector<uint8_t> startStreamMsg() {
        return { static_cast<uint8_t>(MessageType::StartStream) };
    }

    /// Real steady-clock µs timestamp for the session's pacing/timing.
    static uint64_t realNowUs() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    /// RAII: stop the session + join its thread even when a test fails.
    struct SessionRunner {
        std::thread thread;
        Session* session = nullptr;
        ~SessionRunner() {
            if (session) session->requestStop();
            if (thread.joinable()) thread.join();
        }
    };

    static constexpr uint64_t kIdOther = 0x2001;
    static constexpr uint64_t kIdSdo   = 0x3001;

    Registry registry_;
    std::unique_ptr<TestRingSource<8>> src_;
    std::vector<IRingStreamSource*> ringSources_;
};

// ---------------------------------------------------------------------------
// Unit-level ring source behaviour
// ---------------------------------------------------------------------------

TEST_F(RingStreamTest, SourceSchemaAndExclusiveAcquire) {
    EXPECT_EQ(src_->rowSize(), sizeof(TestRow));
    ASSERT_EQ(src_->schemaEntryIds().size(), 3u);
    EXPECT_EQ(src_->schemaEntryIds()[0], kIdA);
    EXPECT_TRUE(src_->tryAcquire());
    EXPECT_FALSE(src_->tryAcquire());      // second session must fail
    src_->release();
    EXPECT_TRUE(src_->tryAcquire());
    src_->release();
}

TEST_F(RingStreamTest, SourceProduceGateAndDrain) {
    // Inactive: produce() is a no-op.
    EXPECT_FALSE(src_->produce([](TestRow& r) { r.field_a = 1; }));
    src_->start();
    EXPECT_TRUE(src_->active());
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(src_->produce([&](TestRow& r) {
            r.ts_us = 100 + i;
            r.field_a = static_cast<uint16_t>(i);
            r.field_b = static_cast<uint32_t>(i * 10);
            r.field_c = static_cast<int8_t>(-i);
        }));
    }
    std::vector<uint8_t> buf(4 * sizeof(TestRow));
    EXPECT_EQ(src_->drainRows(buf.data(), 4), 4u);
    const auto* rows = reinterpret_cast<const TestRow*>(buf.data());
    EXPECT_EQ(rows[0].ts_us, 100u);
    EXPECT_EQ(rows[3].field_b, 30u);
    // Bounded drain leaves nothing behind.
    EXPECT_EQ(src_->drainRows(buf.data(), 4), 0u);
    src_->stop();
    EXPECT_FALSE(src_->active());
}

TEST_F(RingStreamTest, SourceCapacityOneIsBestEffort) {
    SpscRingStreamSource<TestRow, 1> src({kIdA}, {2});
    src.start();
    EXPECT_TRUE(src.produce([](TestRow& r) { r.ts_us = 1; r.field_a = 1; }));
    // Ring full: second row dropped, counted.
    EXPECT_FALSE(src.produce([](TestRow& r) { r.ts_us = 2; r.field_a = 2; }));
    EXPECT_EQ(src.dropped(), 1u);
    uint8_t buf[sizeof(TestRow)];
    EXPECT_EQ(src.drainRows(buf, 1), 1u);
    EXPECT_EQ(reinterpret_cast<TestRow*>(buf)->ts_us, 1u);
    // After drain the producer can write again.
    EXPECT_TRUE(src.produce([](TestRow& r) { r.ts_us = 3; r.field_a = 3; }));
    EXPECT_EQ(src.drainRows(buf, 1), 1u);
    EXPECT_EQ(reinterpret_cast<TestRow*>(buf)->ts_us, 3u);
    src.stop();
}

// ---------------------------------------------------------------------------
// Session-level ring streaming
// ---------------------------------------------------------------------------

TEST_F(RingStreamTest, SessionStreamsRingRows) {
    auto transport = std::make_unique<MockTransport>();
    MockTransport* tp = transport.get();

    auto cfg = configureStreamMsg({kIdA, kIdB}, /*intervalMs*/1, /*chunk*/2);
    auto start = startStreamMsg();
    tp->injectSlipMessage(cfg.data(), cfg.size());
    tp->injectSlipMessage(start.data(), start.size());

    auto session = std::make_unique<Session>(
        std::move(transport), registry_, realNowUs, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr,
        Framing::Slip, &ringSources_);
    SessionRunner runner;
    runner.session = session.get();
    runner.thread = std::thread([&] { session->run(); });

    // Wait for the stream to bind + start, then produce rows.
    for (int i = 0; i < 200 && !src_->active(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_TRUE(src_->active());

    // 7 rows with chunk=2: three full chunks flush immediately, the last
    // row relies on the producer-quiet partial-chunk flush.
    for (int i = 0; i < 7; ++i) {
        src_->produce([&](TestRow& r) {
            r.ts_us = 9000 + i;
            r.field_a = static_cast<uint16_t>(100 + i);
            r.field_b = static_cast<uint32_t>(200 + i);
            r.field_c = static_cast<int8_t>(i);
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    session->requestStop();
    runner.thread.join();
    runner.session = nullptr;
    EXPECT_FALSE(src_->active());          // released on session end
    EXPECT_FALSE(src_->acquired());

    auto pkts = tp->drainTxPackets();
    // Expect at least: ConfigureAck + one StreamData with our rows.
    bool sawAck = false, sawData = false;
    uint32_t totalRows = 0;
    uint64_t expectedIdx = 0;
    for (auto& p : pkts) {
        BufReader r(p.data(), p.size());
        uint8_t type = r.getU8();
        if (type == static_cast<uint8_t>(MessageType::ConfigureAck))
            sawAck = true;
        else if (type == static_cast<uint8_t>(MessageType::StreamData)) {
            sawData = true;
            r.getU32();                    // specId
            uint32_t rows = r.getU32();
            totalRows += rows;
            // Row: u64 ts + u16 a + u32 b (subset projection: c excluded)
            for (uint32_t i = 0; i < rows; ++i) {
                uint64_t ts = r.getU64();
                uint16_t a = r.getU16();
                uint32_t b = r.getU32();
                // Must be the producer timestamp (9000..9006 in order), not
                // the session's real-clock timestamp (~1e15).
                EXPECT_EQ(ts, 9000u + expectedIdx);
                EXPECT_EQ(b, static_cast<uint32_t>(a + 100));
                ++expectedIdx;
            }
        }
    }
    EXPECT_TRUE(sawAck);
    EXPECT_TRUE(sawData);
    EXPECT_EQ(totalRows, 7u);
}

TEST_F(RingStreamTest, SessionFallsBackToPollingForUncoveredEntries) {
    auto transport = std::make_unique<MockTransport>();
    MockTransport* tp = transport.get();

    // kIdOther is not in the ring schema -> normal polling path.
    auto cfgMsg = configureStreamMsg({kIdOther});
    auto startMsg = startStreamMsg();
    tp->injectSlipMessage(cfgMsg.data(), cfgMsg.size());
    tp->injectSlipMessage(startMsg.data(), startMsg.size());

    auto session = std::make_unique<Session>(
        std::move(transport), registry_,
        realNowUs,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, Framing::Slip, &ringSources_);
    SessionRunner runner;
    runner.session = session.get();
    runner.thread = std::thread([&] { session->run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    EXPECT_FALSE(src_->acquired());        // ring not bound
    EXPECT_FALSE(src_->active());

    session->requestStop();
    runner.thread.join();
    runner.session = nullptr;

    auto pkts = tp->drainTxPackets();
    bool sawData = false;
    for (auto& p : pkts) {
        if (p.empty()) continue;
        BufReader r(p.data(), p.size());
        if (r.getU8() == static_cast<uint8_t>(MessageType::StreamData))
            sawData = true;                // polled rows still stream
    }
    EXPECT_TRUE(sawData);
}

TEST_F(RingStreamTest, NoStreamEntriesAreExcludedFromCollectPlan) {
    auto transport = std::make_unique<MockTransport>();
    MockTransport* tp = transport.get();

    // Stream contains the NoStream SDO param plus a normal signal:
    // the SDO entry resolves out of the collect plan entirely.
    auto cfgMsg = configureStreamMsg({kIdSdo, kIdA});
    auto startMsg = startStreamMsg();
    tp->injectSlipMessage(cfgMsg.data(), cfgMsg.size());
    tp->injectSlipMessage(startMsg.data(), startMsg.size());

    auto session = std::make_unique<Session>(
        std::move(transport), registry_,
        realNowUs,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, Framing::Slip, &ringSources_);
    SessionRunner runner;
    runner.session = session.get();
    runner.thread = std::thread([&] { session->run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    session->requestStop();
    runner.thread.join();
    runner.session = nullptr;

    auto pkts = tp->drainTxPackets();
    bool sawAck = false;
    uint32_t resolved = 0xFFFFFFFF;
    for (auto& p : pkts) {
        if (p.empty()) continue;
        BufReader r(p.data(), p.size());
        if (r.getU8() == static_cast<uint8_t>(MessageType::ConfigureAck)) {
            sawAck = true;
            r.getU32();                    // specId
            resolved = r.getU32();         // resolved entry count
        }
    }
    EXPECT_TRUE(sawAck);
    EXPECT_EQ(resolved, 1u);               // only kIdA made it into the plan
}
