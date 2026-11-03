/**
 * @file test_write_verify.cpp
 * @brief Coverage for the instance-based write-verifier
 *        (src/ethercat/WriteVerify.cpp) using a scripted fake transport.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <vector>

#include "tether/ethercat/WriteVerify.hpp"

using namespace EtherCAT::Verify;

namespace {

struct SentDatagram {
    uint8_t cmd; uint8_t idx; uint16_t adp; uint16_t ado;
    std::vector<uint8_t> data;
    bool roundtrip;
};

class FakeTransport : public IWriteVerifyTransport {
public:
    uint8_t allocIdx() override { return next_idx_++; }

    bool sendDatagram(uint8_t cmd, uint8_t idx, uint16_t adp,
                      uint16_t ado, const void* data, uint16_t len,
                      bool roundtrip) override {
        sent_.push_back({cmd, idx, adp, ado,
                         std::vector<uint8_t>(
                             static_cast<const uint8_t*>(data),
                             static_cast<const uint8_t*>(data) + len),
                         roundtrip});
        return send_ok_;
    }

    bool waitForResponse(uint8_t, unsigned int,
                         DatagramResponse& resp) override {
        if (responses_.empty()) return false;
        resp = responses_.front();
        responses_.pop_front();
        return true;
    }

    bool readRegister(uint16_t adp, uint16_t ado, void* out,
                      uint16_t len, unsigned int) override {
        last_read_ = {adp, ado, len};
        if (read_data_.size() >= len)
            std::memcpy(out, read_data_.data(), len);
        else
            std::memset(out, 0, len);
        return read_ok_;
    }

    void delayMs(unsigned int ms) override { delays_ += ms; }

    // Scripting knobs
    bool send_ok_ = true;
    bool read_ok_ = true;
    std::deque<DatagramResponse> responses_;
    std::vector<uint8_t> read_data_;
    std::vector<SentDatagram> sent_;
    struct { uint16_t adp, ado, len; } last_read_{0,0,0};
    uint8_t next_idx_ = 1;
    unsigned int delays_ = 0;
};

DatagramResponse respWkc(uint16_t wkc, const std::vector<uint8_t>& data = {}) {
    DatagramResponse r{};
    r.wkc = wkc;
    r.datalen = static_cast<uint16_t>(data.size());
    std::memcpy(r.data, data.data(), data.size());
    return r;
}

TEST(WriteVerifyTest, ApwrVerifySuccessRoundtrip) {
    FakeTransport t;
    WriteVerifier wv(t);

    // verifyApwr: write datagram (wkc=1), then read-back matches.
    t.responses_.push_back(respWkc(1));
    std::vector<uint8_t> payload{0x34, 0x12};
    t.read_data_ = payload;

    auto r = wv.apwrVerify(0x1000, 0x0120, payload.data(), payload.size(), 100);
    EXPECT_TRUE(r.success);
    EXPECT_TRUE(r.write_ok);
    EXPECT_TRUE(r.verify_ok);
    ASSERT_EQ(t.sent_.size(), 1u);
    EXPECT_EQ(t.sent_[0].cmd, EtherCATCmd::APWR);
    EXPECT_EQ(t.sent_[0].adp, 0x1000);
    EXPECT_EQ(t.sent_[0].ado, 0x0120);
    EXPECT_EQ(t.last_read_.ado, 0x0120);
}

TEST(WriteVerifyTest, ApwrVerifyMismatchDetected) {
    FakeTransport t;
    WriteVerifyConfig cfg{};
    cfg.retry_count = 0;      // single attempt — no response starvation
    cfg.log_failures = true;
    WriteVerifier wv(t, cfg);
    t.responses_.push_back(respWkc(1));
    t.read_data_ = {0xFF, 0xFF};  // different from what was written

    std::vector<uint8_t> payload{0x00, 0x00};
    auto r = wv.apwrVerify(0x1000, 0x0130, payload.data(), payload.size(), 100);
    EXPECT_FALSE(r.success);
    EXPECT_TRUE(r.write_ok);
    EXPECT_FALSE(r.verify_ok);
}

TEST(WriteVerifyTest, ApwrVerifyWriteFailure) {
    FakeTransport t;
    WriteVerifier wv(t);
    t.responses_.push_back(respWkc(0));  // wkc 0 → write didn't land

    std::vector<uint8_t> payload{1, 2, 3, 4};
    auto r = wv.apwrVerify(0x1000, 0x0400, payload.data(), payload.size(), 100);
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.write_ok);
}

TEST(WriteVerifyTest, TypedHelpersMatchWidths) {
    FakeTransport t;
    WriteVerifier wv(t);

    t.read_data_ = {0xAA, 0xBB};
    t.responses_.push_back(respWkc(1));
    EXPECT_TRUE(wv.apwrVerifyU16(0x1000, 0x0120, 0xBBAA, 100).success);
    EXPECT_EQ(t.sent_.back().data.size(), 2u);

    t.read_data_ = {0x78, 0x56, 0x34, 0x12};
    t.responses_.push_back(respWkc(1));
    EXPECT_TRUE(wv.apwrVerifyU32(0x1000, 0x0120, 0x12345678, 100).success);
    EXPECT_EQ(t.sent_.back().data.size(), 4u);

    t.read_data_ = {1,2,3,4,5,6,7,8};
    t.responses_.push_back(respWkc(1));
    EXPECT_TRUE(wv.apwrVerifyU64(0x1000, 0x0120, 0x0807060504030201ULL, 100).success);
    EXPECT_EQ(t.sent_.back().data.size(), 8u);
}

TEST(WriteVerifyTest, BwrVerifyBroadcasts) {
    FakeTransport t;
    WriteVerifier wv(t);
    t.responses_.push_back(respWkc(3));  // 3 slaves acked
    t.read_data_ = {9, 9};

    std::vector<uint8_t> payload{9, 9};
    auto r = wv.bwrVerify(0, 0x0120, payload.data(), payload.size(), 100);
    EXPECT_TRUE(r.success);
    ASSERT_FALSE(t.sent_.empty());
    EXPECT_EQ(t.sent_[0].cmd, EtherCATCmd::BWR);
}

TEST(WriteVerifyTest, DisabledSkipsWork) {
    FakeTransport t;
    WriteVerifier wv(t);
    wv.setEnabled(false);
    EXPECT_FALSE(wv.isEnabled());

    // Disabled still writes — it just skips the read-back verify.
    t.responses_.push_back(respWkc(1));
    std::vector<uint8_t> payload{1, 2};
    auto r = wv.apwrVerify(0x1000, 0x0120, payload.data(), payload.size(), 100);
    EXPECT_TRUE(r.success);
    ASSERT_EQ(t.sent_.size(), 1u);
    EXPECT_EQ(t.last_read_.ado, 0);   // no read-back happened
}

TEST(WriteVerifyTest, StatsTrackOperations) {
    FakeTransport t;
    WriteVerifier wv(t);

    t.responses_.push_back(respWkc(1));
    t.read_data_ = {1};
    EXPECT_TRUE(wv.apwrVerify(0x1000, 0x0120, std::vector<uint8_t>{1}.data(), 1, 100).success);

    t.responses_.push_back(respWkc(1));
    t.read_data_ = {9};
    EXPECT_FALSE(wv.apwrVerify(0x1000, 0x0120, std::vector<uint8_t>{1}.data(), 1, 100).success);

    const auto& s = wv.stats();
    EXPECT_EQ(s.total_writes, 2u);
    EXPECT_GE(s.verify_failures + s.write_failures, 1u);
    wv.resetStats();
    EXPECT_EQ(wv.stats().total_writes, 0u);
    wv.logStats();  // must not crash
}

TEST(WriteVerifyTest, SendFailurePropagates) {
    FakeTransport t;
    WriteVerifier wv(t);
    t.send_ok_ = false;

    std::vector<uint8_t> payload{1};
    auto r = wv.apwrVerify(0x1000, 0x0120, payload.data(), payload.size(), 100);
    EXPECT_FALSE(r.success);
}

TEST(WriteVerifyTest, SetConfigApplies) {
    FakeTransport t;
    WriteVerifier wv(t);
    WriteVerifyConfig cfg{};
    cfg.retry_count = 0;
    cfg.log_failures = false;
    wv.setConfig(cfg);
    t.responses_.push_back(respWkc(1));
    t.read_data_ = {5};
    EXPECT_TRUE(wv.apwrVerify(0x1000, 0x0120, std::vector<uint8_t>{5}.data(), 1, 100).success);
}

} // namespace
