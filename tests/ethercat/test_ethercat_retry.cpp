#include <gtest/gtest.h>

#include "tether/ethercat/Retry.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using EtherCAT::ConditionalPacketRouter;
using EtherCAT::PacketFilter;
using EtherCAT::RxDatagram;

using EtherCAT::Raw::RetryExecutor;
using EtherCAT::Raw::RetryPolicy;
using EtherCAT::Raw::RetryResult;
using EtherCAT::Raw::StoredDatagram;

namespace {

class RetryTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(router_.init());
    }

    void TearDown() override {
        // Ensure no background thread is still trying to touch router.
        for (auto& t : threads_) {
            if (t.joinable()) t.join();
        }
        router_.shutdown();
    }

    void enqueueResponseForIdx(uint8_t idx, std::vector<uint8_t> payload, uint16_t wkc = 1) {
        responses_[idx] = std::move(payload);
        wkcs_[idx] = wkc;
    }

    bool sendAndScheduleResponse(const StoredDatagram& req) {
        // Wait until the executor has registered a waiter (avoid race).
        threads_.emplace_back([this, req]() {
            for (int i = 0; i < 200; ++i) {
                if (router_.hasWaiters()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            RxDatagram rx{};
            rx.idx = req.idx;
            rx.cmd = req.cmd;
            // PacketFilter::aprd/apwr matches slave_index by converting it via
            // Raw::adp_for_slave_index(). For our unit tests we primarily use
            // slave_position=0, so make the response ADP match that mapping.
            // (The request builder uses a different ADP convention for auto-increment.)
            if ((req.cmd == EtherCAT::Command::APRD || req.cmd == EtherCAT::Command::APWR) && req.adp == 0xFFFF) {
                rx.adp = 0;
            } else {
                rx.adp = req.adp;
            }
            rx.ado = req.ado;

            auto it = responses_.find(req.idx);
            if (it != responses_.end()) {
                rx.datalen = static_cast<uint16_t>(std::min<size_t>(it->second.size(), sizeof(rx.data)));
                std::memcpy(rx.data, it->second.data(), rx.datalen);
            } else {
                rx.datalen = 0;
            }

            auto wit = wkcs_.find(req.idx);
            rx.wkc = (wit != wkcs_.end()) ? wit->second : 1;

            (void)router_.routePacket(rx);
        });

        return true;
    }

    ConditionalPacketRouter router_;
    std::vector<std::thread> threads_;
    std::unordered_map<uint8_t, std::vector<uint8_t>> responses_;
    std::unordered_map<uint8_t, uint16_t> wkcs_;
};

} // namespace

TEST(EtherCATRetry, BuilderFunctionsSetFieldsAndCopyData) {
    // APRD: negative auto-increment addressing; buffers zeroed for reads.
    auto aprd = EtherCAT::Raw::buildAPRD(7, 2, 0x0123, 4);
    EXPECT_EQ(aprd.cmd, EtherCAT::Command::APRD);
    EXPECT_EQ(aprd.idx, 7);
    EXPECT_EQ(aprd.ado, 0x0123);
    EXPECT_EQ(aprd.datalen, 4);
    EXPECT_EQ(aprd.adp, static_cast<uint16_t>(-static_cast<int16_t>(2) - 1));
    EXPECT_EQ(aprd.data[0], 0);

    // APWR: copies payload.
    uint8_t payload[3] = {1, 2, 3};
    auto apwr = EtherCAT::Raw::buildAPWR(1, 0, 0x0500, payload, sizeof(payload));
    EXPECT_EQ(apwr.cmd, EtherCAT::Command::APWR);
    EXPECT_EQ(apwr.idx, 1);
    EXPECT_EQ(apwr.ado, 0x0500);
    EXPECT_EQ(apwr.datalen, 3);
    EXPECT_EQ(apwr.data[0], 1);
    EXPECT_EQ(apwr.data[1], 2);
    EXPECT_EQ(apwr.data[2], 3);

    // LRW: logical address split across ADP/ADO.
    uint8_t lrwPayload[2] = {9, 8};
    auto lrw = EtherCAT::Raw::buildLRW(3, 0xAABBCCDDu, lrwPayload, 2);
    EXPECT_EQ(lrw.cmd, EtherCAT::Command::LRW);
    EXPECT_EQ(lrw.adp, 0xAABB);
    EXPECT_EQ(lrw.ado, 0xCCDD);
    EXPECT_EQ(lrw.data[0], 9);
    EXPECT_EQ(lrw.data[1], 8);

    // FPRD: configured address read, buffer zeroed.
    auto fprd = EtherCAT::Raw::buildFPRD(2, 0x1000, 0x0123, 2);
    EXPECT_EQ(fprd.cmd, EtherCAT::Command::FPRD);
    EXPECT_EQ(fprd.adp, 0x1000);
    EXPECT_EQ(fprd.ado, 0x0123);
    EXPECT_EQ(fprd.datalen, 2);
    EXPECT_EQ(fprd.data[0], 0);

    // FPWR: configured address write.
    uint8_t fpwrPayload[2] = {5, 6};
    auto fpwr = EtherCAT::Raw::buildFPWR(4, 0x2000, 0x0456, fpwrPayload, 2);
    EXPECT_EQ(fpwr.cmd, EtherCAT::Command::FPWR);
    EXPECT_EQ(fpwr.adp, 0x2000);
    EXPECT_EQ(fpwr.ado, 0x0456);
    EXPECT_EQ(fpwr.data[0], 5);

    // BWR: broadcast write.
    uint8_t bwrPayload[1] = {0x7F};
    auto bwr = EtherCAT::Raw::buildBWR(9, 0x0120, bwrPayload, 1);
    EXPECT_EQ(bwr.cmd, EtherCAT::Command::BWR);
    EXPECT_EQ(bwr.adp, 0u);
    EXPECT_EQ(bwr.data[0], 0x7F);

    // LRD/LWR:
    auto lrd = EtherCAT::Raw::buildLRD(5, 0x11223344u, 4);
    EXPECT_EQ(lrd.cmd, EtherCAT::Command::LRD);
    EXPECT_EQ(lrd.adp, 0x1122);
    EXPECT_EQ(lrd.ado, 0x3344);
    EXPECT_EQ(lrd.data[0], 0);

    uint8_t lwrPayload[2] = {0xA, 0xB};
    auto lwr = EtherCAT::Raw::buildLWR(6, 0x55667788u, lwrPayload, 2);
    EXPECT_EQ(lwr.cmd, EtherCAT::Command::LWR);
    EXPECT_EQ(lwr.adp, 0x5566);
    EXPECT_EQ(lwr.ado, 0x7788);
    EXPECT_EQ(lwr.data[0], 0xA);
}

TEST_F(RetryTest, ExecuteSucceedsFirstTryAndStatsUpdate) {
    enqueueResponseForIdx(10, {0xDE, 0xAD, 0xBE, 0xEF}, 2);

    RetryExecutor exec(router_, [this](const StoredDatagram& req) {
        return sendAndScheduleResponse(req);
    });

    auto req = EtherCAT::Raw::buildBRD(10, 0x0120, 4);
    PacketFilter filter = PacketFilter::byIndex(10);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[8] = {};
    RetryPolicy pol = RetryPolicy::none();
    pol.initial_timeout_ms = 50;
    auto res = exec.execute(req, filter, buf, sizeof(buf), pol);

    EXPECT_TRUE(res.success);
    EXPECT_FALSE(res.timeout);
    EXPECT_EQ(res.attempts, 1u);
    EXPECT_EQ(res.wkc, 2u);
    EXPECT_EQ(res.data_length, 4u);
    EXPECT_EQ(buf[0], 0xDE);

    auto stats = exec.getStats();
    EXPECT_EQ(stats.total_requests, 1u);
    EXPECT_EQ(stats.first_try_success, 1u);
}

TEST_F(RetryTest, ExecuteRetriesAfterTimeoutThenSucceeds) {
    std::atomic<uint32_t> sendCount{0};
    enqueueResponseForIdx(11, {0xAA}, 1);

    RetryExecutor exec(router_, [this, &sendCount](const StoredDatagram& req) {
        auto n = ++sendCount;
        if (n == 1) {
            // First attempt: simulate successful send but no response.
            return true;
        }
        return sendAndScheduleResponse(req);
    });

    auto req = EtherCAT::Raw::buildBRD(11, 0x0120, 1);
    PacketFilter filter = PacketFilter::byIndex(11);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[4] = {};
    RetryPolicy pol;
    pol.max_retries = 1;
    pol.initial_timeout_ms = 5;
    pol.use_exponential_backoff = false;

    auto res = exec.execute(req, filter, buf, sizeof(buf), pol);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.attempts, 2u);
    EXPECT_EQ(buf[0], 0xAA);

    auto stats = exec.getStats();
    EXPECT_EQ(stats.total_requests, 2u);
    EXPECT_EQ(stats.retry_success, 1u);
    EXPECT_GE(stats.total_timeouts, 1u);
}

TEST_F(RetryTest, ExecuteWithWkcReportsMismatchButStillSuccess) {
    enqueueResponseForIdx(12, {0x00}, 1);

    RetryExecutor exec(router_, [this](const StoredDatagram& req) {
        return sendAndScheduleResponse(req);
    });

    auto req = EtherCAT::Raw::buildBRD(12, 0x0120, 1);
    PacketFilter filter = PacketFilter::byIndex(12);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[4] = {};
    RetryPolicy pol = RetryPolicy::none();
    pol.initial_timeout_ms = 50;
    auto res = exec.executeWithWkc(req, filter, buf, sizeof(buf), /*expected_wkc=*/2, pol);

    EXPECT_TRUE(res.success);
    EXPECT_TRUE(res.isWkcError(2));
}

TEST_F(RetryTest, ExecuteHandlesSendFailureThenSucceeds) {
    std::atomic<uint32_t> calls{0};
    enqueueResponseForIdx(13, {0x42}, 1);

    RetryExecutor exec(router_, [this, &calls](const StoredDatagram& req) {
        auto n = ++calls;
        if (n == 1) {
            // First attempt: send fails immediately.
            return false;
        }
        return sendAndScheduleResponse(req);
    });

    auto req = EtherCAT::Raw::buildBRD(13, 0x0120, 1);
    PacketFilter filter = PacketFilter::byIndex(13);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[4] = {};
    RetryPolicy pol;
    pol.max_retries = 1;
    pol.initial_timeout_ms = 50;
    pol.use_exponential_backoff = false;

    auto res = exec.execute(req, filter, buf, sizeof(buf), pol);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.attempts, 2u);

    // Exercise logStats() line coverage.
    exec.logStats();
}

TEST_F(RetryTest, HighLevelHelpersWork) {
    // Prepare predictable responses for multiple indices.
    // retryAPWRVerify uses idx for write and idx+1 for verify read.
    const uint8_t idxWrite = 20;
    const uint8_t idxRead = static_cast<uint8_t>((idxWrite + 1) & 0xFF);

    // Write response data is not used by the implementation beyond success.
    enqueueResponseForIdx(idxWrite, {0x11, 0x22, 0x33, 0x44}, 1);
    // Read-back for verify.
    enqueueResponseForIdx(idxRead, {0xCA, 0xFE}, 1);

    RetryExecutor exec(router_, [this](const StoredDatagram& req) {
        return sendAndScheduleResponse(req);
    });

    // BRD helper: cover expected_wkc > 0 branch.
    const uint8_t idxBrd = 21;
    enqueueResponseForIdx(idxBrd, {0x99}, 2);
    uint8_t brdBuf[2] = {};
    auto brdRes = EtherCAT::Raw::retryBRD(exec, idxBrd, 0x0120, brdBuf, 1, /*expected_wkc=*/2, RetryPolicy::none());
    EXPECT_TRUE(brdRes.success);
    EXPECT_EQ(brdBuf[0], 0x99);

    // BWR helper (write-only path uses dummy buffer inside helper).
    const uint8_t idxBwr = 22;
    enqueueResponseForIdx(idxBwr, {0x00}, 3);
    uint8_t bwrData[1] = {0x01};
    auto bwrRes = EtherCAT::Raw::retryBWR(exec, idxBwr, 0x0120, bwrData, 1, /*expected_wkc=*/3, RetryPolicy::none());
    EXPECT_TRUE(bwrRes.success);

    // LRW helper.
    const uint8_t idxLrw = 23;
    enqueueResponseForIdx(idxLrw, {0x10, 0x11}, 4);
    uint8_t rxBuf[4] = {};
    uint8_t txBuf[2] = {0xAA, 0xBB};
    auto lrwRes = EtherCAT::Raw::retryLRW(exec, idxLrw, 0x01020304u, txBuf, rxBuf, 2, /*expected_wkc=*/4, RetryPolicy::none());
    EXPECT_TRUE(lrwRes.success);
    EXPECT_EQ(rxBuf[0], 0x10);

    // APWR verify: first cover "data too large" branch.
    std::vector<uint8_t> big(65, 0xFF);
    auto tooLarge = EtherCAT::Raw::retryAPWRVerify(exec, 30, 0, 0x0500, big.data(), static_cast<uint16_t>(big.size()), RetryPolicy::none());
    EXPECT_FALSE(tooLarge.success);

    // APWR verify: cover "write failed" branch (WKC=0 → min_wkc filter fails).
    wkcs_[idxWrite] = 0;
    uint8_t writeFailData[2] = {0xDD, 0xEE};
    auto writeFail = EtherCAT::Raw::retryAPWRVerify(exec, idxWrite, 0, 0x0500,
                                                     writeFailData, sizeof(writeFailData),
                                                     RetryPolicy::none());
    EXPECT_FALSE(writeFail.success);
    wkcs_[idxWrite] = 1; // restore for subsequent tests

    uint8_t toWrite[2] = {0xAA, 0xBB};

    // Verify read failed path (no match due to WKC below min).
    wkcs_[idxRead] = 0;
    auto readFail = EtherCAT::Raw::retryAPWRVerify(exec, idxWrite, 0, 0x0500, toWrite, sizeof(toWrite), RetryPolicy::none());
    EXPECT_FALSE(readFail.success);

    // Verify mismatch path.
    wkcs_[idxRead] = 1;
    auto mismatch = EtherCAT::Raw::retryAPWRVerify(exec, idxWrite, 0, 0x0500, toWrite, sizeof(toWrite), RetryPolicy::none());
    EXPECT_FALSE(mismatch.success);

    // Now make verify succeed by aligning the read-back response.
    enqueueResponseForIdx(idxRead, {0xAA, 0xBB}, 1);
    auto ok = EtherCAT::Raw::retryAPWRVerify(exec, idxWrite, 0, 0x0500, toWrite, sizeof(toWrite), RetryPolicy::none());
    EXPECT_TRUE(ok.success);
}
