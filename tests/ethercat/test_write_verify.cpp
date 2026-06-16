/**
 * @file test_write_verify.cpp
 * @brief Comprehensive tests for WriteVerifier (instance-based, no global state)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/WriteVerify.hpp"

#include <cstring>
#include <vector>
#include <functional>

using namespace EtherCAT::Verify;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::InSequence;
using ::testing::AnyNumber;

// ============================================================================
// MockWriteVerifyTransport
// ============================================================================

class MockWriteVerifyTransport : public IWriteVerifyTransport {
public:
    MOCK_METHOD(uint8_t, allocIdx, (), (override));
    MOCK_METHOD(bool, sendDatagram, (uint8_t cmd, uint8_t idx, uint16_t adp,
                uint16_t ado, const void* data, uint16_t len, bool roundtrip),
                (override));
    MOCK_METHOD(bool, waitForResponse, (uint8_t idx, unsigned int timeout_ms,
                DatagramResponse& response), (override));
    MOCK_METHOD(bool, readRegister, (uint16_t adp, uint16_t ado, void* out,
                uint16_t len, unsigned int timeout_ms), (override));
    MOCK_METHOD(void, delayMs, (unsigned int ms), (override));
};

// Helpers

/// Make waitForResponse return success with given WKC
static auto WaitOkWkc(uint16_t wkc) {
    return [wkc](uint8_t, unsigned int, DatagramResponse& resp) -> bool {
        resp.wkc = wkc;
        resp.datalen = 0;
        return true;
    };
}

/// Make waitForResponse return failure (timeout)
static auto WaitTimeout() {
    return [](uint8_t, unsigned int, DatagramResponse&) -> bool {
        return false;
    };
}

/// Make readRegister copy expected data into output buffer
static auto ReadBack(const void* data, uint16_t len) {
    return [data, len](uint16_t, uint16_t, void* out, uint16_t req_len,
                       unsigned int) -> bool {
        uint16_t to_copy = std::min(len, req_len);
        std::memcpy(out, data, to_copy);
        return true;
    };
}

/// Make readRegister return different data (mismatch)
static auto ReadBackMismatch(uint8_t fill_byte) {
    return [fill_byte](uint16_t, uint16_t, void* out, uint16_t len,
                       unsigned int) -> bool {
        std::memset(out, fill_byte, len);
        return true;
    };
}

/// Make readRegister fail
static auto ReadFail() {
    return [](uint16_t, uint16_t, void*, uint16_t, unsigned int) -> bool {
        return false;
    };
}

// ============================================================================
// Construction Tests
// ============================================================================

class WriteVerifierConstructTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;
};

TEST_F(WriteVerifierConstructTest, DefaultConfig) {
    WriteVerifier wv(transport_);
    EXPECT_TRUE(wv.isEnabled());

    const auto& cfg = wv.config();
    EXPECT_EQ(cfg.retry_count, 3u);
    EXPECT_EQ(cfg.retry_delay_ms, 10u);
    EXPECT_EQ(cfg.read_delay_ms, 1u);
    EXPECT_TRUE(cfg.log_failures);
}

TEST_F(WriteVerifierConstructTest, CustomConfig) {
    WriteVerifyConfig custom = {};
    custom.retry_count = 5;
    custom.retry_delay_ms = 20;
    custom.read_delay_ms = 2;
    custom.log_failures = false;

    WriteVerifier wv(transport_, custom);
    const auto& cfg = wv.config();
    EXPECT_EQ(cfg.retry_count, 5u);
    EXPECT_EQ(cfg.retry_delay_ms, 20u);
    EXPECT_EQ(cfg.read_delay_ms, 2u);
    EXPECT_FALSE(cfg.log_failures);
}

TEST_F(WriteVerifierConstructTest, StatsInitZero) {
    WriteVerifier wv(transport_);
    const auto& s = wv.stats();
    EXPECT_EQ(s.total_writes, 0u);
    EXPECT_EQ(s.successful_writes, 0u);
    EXPECT_EQ(s.verify_failures, 0u);
    EXPECT_EQ(s.write_failures, 0u);
    EXPECT_EQ(s.retries, 0u);
    EXPECT_EQ(s.eventual_success, 0u);
    EXPECT_EQ(s.permanent_failures, 0u);
}

// ============================================================================
// Enable / Disable Tests
// ============================================================================

class WriteVerifierEnableTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;
    WriteVerifier wv_{transport_};
};

TEST_F(WriteVerifierEnableTest, DefaultEnabled) {
    EXPECT_TRUE(wv_.isEnabled());
}

TEST_F(WriteVerifierEnableTest, DisableAndEnable) {
    wv_.setEnabled(false);
    EXPECT_FALSE(wv_.isEnabled());
    wv_.setEnabled(true);
    EXPECT_TRUE(wv_.isEnabled());
}

TEST_F(WriteVerifierEnableTest, DisabledSkipsVerify) {
    wv_.setEnabled(false);

    uint16_t data = 0x1234;

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::APWR, 1, _, _, _, _, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(1, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    // readRegister should NOT be called when disabled
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _)).Times(0);

    auto result = wv_.apwrVerify(0, 0x100, &data, 2, 100);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.write_wkc, 1);
    EXPECT_EQ(result.attempts, 1u);
}

TEST_F(WriteVerifierEnableTest, DisabledWriteFails) {
    wv_.setEnabled(false);

    uint16_t data = 0x1234;

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));

    auto result = wv_.apwrVerify(0, 0x100, &data, 2, 100);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv_.stats().write_failures, 1u);
}

TEST_F(WriteVerifierEnableTest, DisabledWkcZero) {
    wv_.setEnabled(false);

    uint16_t data = 0x1234;

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(0)));

    auto result = wv_.apwrVerify(0, 0x100, &data, 2, 100);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv_.stats().write_failures, 1u);
}

// ============================================================================
// Write Success on First Try
// ============================================================================

class WriteVerifierSuccessTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;
    WriteVerifyConfig cfg_ = WriteVerifyConfig::defaults();
    WriteVerifier wv_{transport_, cfg_};

    void SetUp() override {
        // Default: read_delay_ms = 1, so delayMs(1) is called
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierSuccessTest, SimpleWriteVerifySuccess) {
    uint8_t data[] = {0xAA, 0xBB};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::APWR, 42, 1, 0x100,
                                         _, 2, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(42, 100, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, delayMs(1));  // read_delay_ms
    EXPECT_CALL(transport_, readRegister(1, 0x100, _, 2, 100))
        .WillOnce(Invoke(ReadBack(data, 2)));

    auto result = wv_.apwrVerify(1, 0x100, data, 2, 100);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.write_ok);
    EXPECT_TRUE(result.verify_ok);
    EXPECT_EQ(result.write_wkc, 1);
    EXPECT_EQ(result.attempts, 1u);

    const auto& s = wv_.stats();
    EXPECT_EQ(s.total_writes, 1u);
    EXPECT_EQ(s.successful_writes, 1u);
    EXPECT_EQ(s.retries, 0u);
    EXPECT_EQ(s.verify_failures, 0u);
    EXPECT_EQ(s.permanent_failures, 0u);
}

TEST_F(WriteVerifierSuccessTest, WriteVerifyNoReadDelay) {
    // Config with read_delay_ms = 0 — delayMs should not be called for readback
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 1;
    cfg.retry_delay_ms = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    wv_.setConfig(cfg);

    uint8_t data[] = {0x55};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, delayMs(_)).Times(0);
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));

    auto result = wv_.apwrVerify(0, 0, data, 1, 50);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// Write Failure with Retries
// ============================================================================

class WriteVerifierRetryTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierRetryTest, SendFailRetries) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 2;
    cfg.retry_delay_ms = 5;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x11};

    // All 3 attempts (0, 1, 2) fail to send
    EXPECT_CALL(transport_, allocIdx()).Times(3).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .Times(3).WillRepeatedly(Return(false));
    EXPECT_CALL(transport_, delayMs(5)).Times(2);  // retries 1 and 2

    auto result = wv.apwrVerify(0, 0, data, 1, 100);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.attempts, 3u);  // retry_count + 1
    EXPECT_EQ(wv.stats().retries, 2u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierRetryTest, TimeoutRetries) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 1;
    cfg.retry_delay_ms = 10;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x22};

    EXPECT_CALL(transport_, allocIdx()).Times(2).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .Times(2).WillRepeatedly(Invoke(WaitTimeout()));
    EXPECT_CALL(transport_, delayMs(10)).Times(1);

    auto result = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierRetryTest, WkcZeroRetries) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 1;
    cfg.retry_delay_ms = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x33};

    EXPECT_CALL(transport_, allocIdx()).Times(2).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .Times(2).WillRepeatedly(Invoke(WaitOkWkc(0)));

    auto result = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv.stats().write_failures, 2u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierRetryTest, SuccessAfterRetry) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 3;
    cfg.retry_delay_ms = 5;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x44};
    int attempt_count = 0;

    EXPECT_CALL(transport_, allocIdx()).WillRepeatedly(Return(1));
    // First attempt: send fails. Second attempt: send succeeds.
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false))
        .WillRepeatedly(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillRepeatedly(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));
    EXPECT_CALL(transport_, delayMs(5)).Times(1);

    auto result = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.attempts, 2u);

    const auto& s = wv.stats();
    EXPECT_EQ(s.retries, 1u);
    EXPECT_EQ(s.eventual_success, 1u);
    EXPECT_EQ(s.successful_writes, 0u);  // not first try
}

// ============================================================================
// Verify Mismatch Detection
// ============================================================================

class WriteVerifierMismatchTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierMismatchTest, MismatchDetected) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;  // no retries
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0xAA, 0xBB, 0xCC};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    // Read back different data: 0xAA, 0xFF, 0xCC — mismatch at offset 1
    uint8_t readback[] = {0xAA, 0xFF, 0xCC};
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(readback, 3)));

    auto result = wv.apwrVerify(0, 0, data, 3, 50);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.write_ok);
    EXPECT_FALSE(result.verify_ok);
    EXPECT_EQ(result.mismatch_offset, 1u);
    EXPECT_EQ(result.expected_byte, 0xBB);
    EXPECT_EQ(result.actual_byte, 0xFF);
    EXPECT_EQ(result.write_wkc, 1);
    EXPECT_EQ(result.attempts, 1u);

    EXPECT_EQ(wv.stats().verify_failures, 1u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierMismatchTest, MismatchRetryThenSuccess) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 1;
    cfg.retry_delay_ms = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x55, 0x66};
    uint8_t bad_readback[] = {0x55, 0x00};

    EXPECT_CALL(transport_, allocIdx()).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillRepeatedly(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillRepeatedly(Invoke(WaitOkWkc(1)));

    // First read: mismatch. Second read: match.
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(bad_readback, 2)))
        .WillOnce(Invoke(ReadBack(data, 2)));

    auto result = wv.apwrVerify(0, 0, data, 2, 50);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.attempts, 2u);
    EXPECT_EQ(wv.stats().verify_failures, 1u);
    EXPECT_EQ(wv.stats().eventual_success, 1u);
}

TEST_F(WriteVerifierMismatchTest, ReadFailRetries) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 1;
    cfg.retry_delay_ms = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x77};

    EXPECT_CALL(transport_, allocIdx()).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillRepeatedly(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillRepeatedly(Invoke(WaitOkWkc(1)));

    // Both reads fail
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .Times(2).WillRepeatedly(Invoke(ReadFail()));

    auto result = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

// ============================================================================
// Stats Accumulation
// ============================================================================

class WriteVerifierStatsTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierStatsTest, AccumulateAcrossMultipleWrites) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x11};

    // Successful write
    EXPECT_CALL(transport_, allocIdx()).WillRepeatedly(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillRepeatedly(Return(true));

    // Write 1: success
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));

    auto r1 = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_TRUE(r1.success);

    // Write 2: WKC=0
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(0)));

    auto r2 = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(r2.success);

    // Write 3: success
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));

    auto r3 = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_TRUE(r3.success);

    const auto& s = wv.stats();
    EXPECT_EQ(s.total_writes, 3u);
    EXPECT_EQ(s.successful_writes, 2u);
    EXPECT_EQ(s.write_failures, 1u);
    EXPECT_EQ(s.permanent_failures, 1u);
}

// ============================================================================
// Multi-Instance Independence
// ============================================================================

TEST_F(WriteVerifierStatsTest, MultipleIndependentInstances) {
    MockWriteVerifyTransport transport2;

    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;

    WriteVerifier wv1(transport_, cfg);
    WriteVerifier wv2(transport2, cfg);

    uint8_t data[] = {0x88};

    // wv1: success
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));
    wv1.apwrVerify(0, 0, data, 1, 50);

    // wv2: failure
    EXPECT_CALL(transport2, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport2, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport2, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(0)));
    wv2.apwrVerify(0, 0, data, 1, 50);

    // Stats are independent
    EXPECT_EQ(wv1.stats().total_writes, 1u);
    EXPECT_EQ(wv1.stats().successful_writes, 1u);
    EXPECT_EQ(wv1.stats().write_failures, 0u);

    EXPECT_EQ(wv2.stats().total_writes, 1u);
    EXPECT_EQ(wv2.stats().successful_writes, 0u);
    EXPECT_EQ(wv2.stats().write_failures, 1u);
}

TEST_F(WriteVerifierStatsTest, IndependentConfig) {
    MockWriteVerifyTransport transport2;
    WriteVerifier wv1(transport_);
    WriteVerifier wv2(transport2);

    wv1.setEnabled(false);
    EXPECT_FALSE(wv1.isEnabled());
    EXPECT_TRUE(wv2.isEnabled());  // wv2 unaffected

    WriteVerifyConfig c = {};
    c.retry_count = 99;
    c.retry_delay_ms = 0;
    c.read_delay_ms = 0;
    c.log_failures = false;
    wv1.setConfig(c);
    EXPECT_EQ(wv1.config().retry_count, 99u);
    EXPECT_EQ(wv2.config().retry_count, 3u);  // default
}

// ============================================================================
// Edge Cases
// ============================================================================

class WriteVerifierEdgeCaseTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierEdgeCaseTest, DataTooLarge) {
    WriteVerifyConfig cfg = WriteVerifyConfig::defaults();
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t big[257];  // > kMaxDataLen (256)
    std::memset(big, 0, sizeof(big));

    auto result = wv.apwrVerify(0, 0, big, 257, 100);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.attempts, 0u);  // rejected before any attempt
    EXPECT_EQ(wv.stats().total_writes, 1u);
}

TEST_F(WriteVerifierEdgeCaseTest, MaxDataLenExact) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[256];
    std::memset(data, 0xAB, sizeof(data));

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 256, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, 256, _))
        .WillOnce(Invoke(ReadBack(data, 256)));

    auto result = wv.apwrVerify(0, 0, data, 256, 50);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierEdgeCaseTest, ZeroLenWrite) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data = 0;

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 0, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, 0, _))
        .WillOnce(Return(true));  // 0-length read succeeds

    auto result = wv.apwrVerify(0, 0, &data, 0, 50);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierEdgeCaseTest, ZeroRetryCount) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.retry_delay_ms = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x99};

    // Single attempt fails
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));

    auto result = wv.apwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.attempts, 1u);
    EXPECT_EQ(wv.stats().retries, 0u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

// ============================================================================
// Reset Stats
// ============================================================================

TEST_F(WriteVerifierEdgeCaseTest, ResetStatsClearsAll) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x11};

    // Do a failed write to set some stats
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));
    wv.apwrVerify(0, 0, data, 1, 50);

    EXPECT_EQ(wv.stats().total_writes, 1u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);

    wv.resetStats();

    const auto& s = wv.stats();
    EXPECT_EQ(s.total_writes, 0u);
    EXPECT_EQ(s.successful_writes, 0u);
    EXPECT_EQ(s.verify_failures, 0u);
    EXPECT_EQ(s.write_failures, 0u);
    EXPECT_EQ(s.retries, 0u);
    EXPECT_EQ(s.eventual_success, 0u);
    EXPECT_EQ(s.permanent_failures, 0u);
}

// ============================================================================
// U16 / U32 / U64 Verify Functions
// ============================================================================

class WriteVerifierTypedTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;
    WriteVerifyConfig cfg_{};
    WriteVerifier wv_{transport_, cfg_};

    void SetUp() override {
        cfg_.retry_count = 0;
        cfg_.read_delay_ms = 0;
        cfg_.log_failures = false;
        wv_.setConfig(cfg_);
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierTypedTest, U16Success) {
    uint16_t value = 0x1234;
    uint8_t le[] = {0x34, 0x12};  // little-endian

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::APWR, _, 5, 0x200, _, 2, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(5, 0x200, _, 2, _))
        .WillOnce(Invoke(ReadBack(le, 2)));

    auto result = wv_.apwrVerifyU16(5, 0x200, value, 100);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierTypedTest, U32Success) {
    uint32_t value = 0xDEADBEEF;
    uint8_t le[] = {0xEF, 0xBE, 0xAD, 0xDE};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 4, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, 4, _))
        .WillOnce(Invoke(ReadBack(le, 4)));

    auto result = wv_.apwrVerifyU32(0, 0, value, 100);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierTypedTest, U64Success) {
    uint64_t value = 0x0102030405060708ULL;
    uint8_t le[] = {0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 8, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, 8, _))
        .WillOnce(Invoke(ReadBack(le, 8)));

    auto result = wv_.apwrVerifyU64(0, 0, value, 100);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierTypedTest, U16Mismatch) {
    uint16_t value = 0xAABB;
    uint8_t readback[] = {0x00, 0x00};  // mismatch

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(readback, 2)));

    auto result = wv_.apwrVerifyU16(0, 0, value, 100);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.verify_ok);
    // Expected le[0] = 0xBB, actual = 0x00
    EXPECT_EQ(result.mismatch_offset, 0u);
    EXPECT_EQ(result.expected_byte, 0xBB);
    EXPECT_EQ(result.actual_byte, 0x00);
}

// ============================================================================
// BWR Verify Tests
// ============================================================================

class WriteVerifierBwrTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierBwrTest, BwrVerifySuccess) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x11, 0x22};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(5));
    // BWR: adp=0 for broadcast
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::BWR, 5, 0, 0x300, _, 2, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(5, 50, _))
        .WillOnce(Invoke(WaitOkWkc(3)));  // 3 slaves responded
    // Verify via APRD on slave 2
    EXPECT_CALL(transport_, readRegister(2, 0x300, _, 2, 50))
        .WillOnce(Invoke(ReadBack(data, 2)));

    auto result = wv.bwrVerify(2, 0x300, data, 2, 50);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.write_wkc, 3);
    EXPECT_EQ(result.attempts, 1u);
    EXPECT_EQ(wv.stats().successful_writes, 1u);
}

TEST_F(WriteVerifierBwrTest, BwrVerifySendFails) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x33};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));

    auto result = wv.bwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierBwrTest, BwrVerifyWkcZero) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0x44};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(0)));

    auto result = wv.bwrVerify(0, 0, data, 1, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(wv.stats().write_failures, 1u);
}

TEST_F(WriteVerifierBwrTest, BwrVerifyMismatch) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t data[] = {0xAA, 0xBB};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBackMismatch(0x00)));

    auto result = wv.bwrVerify(0, 0, data, 2, 50);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.verify_ok);
    EXPECT_EQ(result.mismatch_offset, 0u);
    EXPECT_EQ(wv.stats().verify_failures, 1u);
    EXPECT_EQ(wv.stats().permanent_failures, 1u);
}

TEST_F(WriteVerifierBwrTest, BwrVerifyDataTooLarge) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);

    uint8_t big[257];
    std::memset(big, 0, sizeof(big));

    auto result = wv.bwrVerify(0, 0, big, 257, 50);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.attempts, 0u);
}

TEST_F(WriteVerifierBwrTest, BwrVerifyDisabledSkipsVerify) {
    WriteVerifyConfig cfg = {};
    cfg.retry_count = 0;
    cfg.read_delay_ms = 0;
    cfg.log_failures = false;
    WriteVerifier wv(transport_, cfg);
    wv.setEnabled(false);

    uint8_t data[] = {0x55};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::BWR, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(2)));
    // No readRegister call when disabled
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _)).Times(0);

    auto result = wv.bwrVerify(0, 0, data, 1, 50);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.write_wkc, 2);
}

// ============================================================================
// Free-Function API
// ============================================================================

class WriteVerifierFreeFuncTest : public ::testing::Test {
protected:
    MockWriteVerifyTransport transport_;
    WriteVerifyConfig cfg_{};
    WriteVerifier wv_{transport_, cfg_};

    void SetUp() override {
        cfg_.retry_count = 0;
        cfg_.read_delay_ms = 0;
        cfg_.log_failures = false;
        wv_.setConfig(cfg_);
        ON_CALL(transport_, delayMs(_)).WillByDefault(Return());
    }
};

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionApwrVerify) {
    uint8_t data[] = {0x42};

    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));

    auto result = apwr_verify(wv_, 0, 0, data, 1, 50);
    EXPECT_TRUE(result.success);
}

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionConfigAndStats) {
    WriteVerifyConfig c = {};
    c.retry_count = 7;
    c.retry_delay_ms = 0;
    c.read_delay_ms = 0;
    c.log_failures = false;
    set_config(wv_, c);
    EXPECT_EQ(get_config(wv_).retry_count, 7u);

    set_enabled(wv_, false);
    EXPECT_FALSE(is_enabled(wv_));

    set_enabled(wv_, true);
    EXPECT_TRUE(is_enabled(wv_));

    EXPECT_EQ(get_stats(wv_).total_writes, 0u);
    reset_stats(wv_);
    EXPECT_EQ(get_stats(wv_).total_writes, 0u);
}

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionU16) {
    uint8_t le[] = {0x01, 0x00};
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 2, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(le, 2)));

    auto r = apwr_verify_u16(wv_, 0, 0, 1, 50);
    EXPECT_TRUE(r.success);
}

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionU32) {
    uint8_t le[] = {0x01, 0x00, 0x00, 0x00};
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 4, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(le, 4)));

    auto r = apwr_verify_u32(wv_, 0, 0, 1, 50);
    EXPECT_TRUE(r.success);
}

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionU64) {
    uint8_t le[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(_, _, _, _, _, 8, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(le, 8)));

    auto r = apwr_verify_u64(wv_, 0, 0, 1, 50);
    EXPECT_TRUE(r.success);
}

TEST_F(WriteVerifierFreeFuncTest, FreeFunctionBwrVerify) {
    uint8_t data[] = {0xAB};
    EXPECT_CALL(transport_, allocIdx()).WillOnce(Return(1));
    EXPECT_CALL(transport_, sendDatagram(EtherCATCmd::BWR, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport_, waitForResponse(_, _, _))
        .WillOnce(Invoke(WaitOkWkc(1)));
    EXPECT_CALL(transport_, readRegister(_, _, _, _, _))
        .WillOnce(Invoke(ReadBack(data, 1)));

    auto r = bwr_verify(wv_, 0, 0, data, 1, 50);
    EXPECT_TRUE(r.success);
}

// ============================================================================
// WriteVerifyResult Static Factory Tests
// ============================================================================

TEST(WriteVerifyResultTest, SuccessFactory) {
    auto r = WriteVerifyResult::Success(5, 3);
    EXPECT_TRUE(r.success);
    EXPECT_TRUE(r.write_ok);
    EXPECT_TRUE(r.verify_ok);
    EXPECT_EQ(r.write_wkc, 5);
    EXPECT_EQ(r.read_wkc, 5);
    EXPECT_EQ(r.attempts, 3u);
}

TEST(WriteVerifyResultTest, WriteFailedFactory) {
    auto r = WriteVerifyResult::WriteFailed(0, 2);
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.write_ok);
    EXPECT_EQ(r.write_wkc, 0);
    EXPECT_EQ(r.attempts, 2u);
}

TEST(WriteVerifyResultTest, VerifyFailedFactory) {
    auto r = WriteVerifyResult::VerifyFailed(1, 1, 4, 3, 0xAA, 0xBB);
    EXPECT_FALSE(r.success);
    EXPECT_TRUE(r.write_ok);
    EXPECT_FALSE(r.verify_ok);
    EXPECT_EQ(r.write_wkc, 1);
    EXPECT_EQ(r.read_wkc, 1);
    EXPECT_EQ(r.attempts, 4u);
    EXPECT_EQ(r.mismatch_offset, 3u);
    EXPECT_EQ(r.expected_byte, 0xAA);
    EXPECT_EQ(r.actual_byte, 0xBB);
}

// ============================================================================
// Config Struct Tests
// ============================================================================

TEST(WriteVerifyConfigTest, Defaults) {
    auto cfg = WriteVerifyConfig::defaults();
    EXPECT_EQ(cfg.retry_count, 3u);
    EXPECT_EQ(cfg.retry_delay_ms, 10u);
    EXPECT_EQ(cfg.read_delay_ms, 1u);
    EXPECT_TRUE(cfg.log_failures);
}

// ============================================================================
// SetConfig Tests
// ============================================================================

TEST(WriteVerifierSetConfigTest, ReconfigureAtRuntime) {
    MockWriteVerifyTransport transport;
    WriteVerifier wv(transport);

    EXPECT_EQ(wv.config().retry_count, 3u);

    WriteVerifyConfig c = {};
    c.retry_count = 10;
    c.retry_delay_ms = 50;
    c.read_delay_ms = 5;
    c.log_failures = false;
    wv.setConfig(c);

    EXPECT_EQ(wv.config().retry_count, 10u);
    EXPECT_EQ(wv.config().retry_delay_ms, 50u);
    EXPECT_EQ(wv.config().read_delay_ms, 5u);
    EXPECT_FALSE(wv.config().log_failures);
}

// ============================================================================
// LogStats (smoke test — just verifies it doesn't crash)
// ============================================================================

TEST(WriteVerifierLogTest, LogStatsDoesNotCrash) {
    MockWriteVerifyTransport transport;
    WriteVerifier wv(transport);
    EXPECT_NO_FATAL_FAILURE(wv.logStats());
}

// ============================================================================
// EtherCATCmd Constants
// ============================================================================

TEST(EtherCATCmdTest, Constants) {
    EXPECT_EQ(EtherCATCmd::APRD, 0x01);
    EXPECT_EQ(EtherCATCmd::APWR, 0x02);
    EXPECT_EQ(EtherCATCmd::BWR,  0x08);
}
