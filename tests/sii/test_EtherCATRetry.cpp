#include <gtest/gtest.h>

#include "tether/ethercat/EtherCATRetry.hpp"
#include "tether/ethercat/ConditionalPacketRouter.hpp"

#include <thread>
#include <chrono>
#include <atomic>

using EtherCAT::ConditionalPacketRouter;
using EtherCAT::RxDatagram;
using EtherCAT::PacketFilter;
using EtherCAT::Raw::RetryExecutor;
using EtherCAT::Raw::StoredDatagram;
using EtherCAT::Raw::RetryPolicy;
using EtherCAT::Raw::RetryResult;

// Minimal test fixture that mirrors the approach used in ethercat/* retry tests
class SiiRetryTest : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_TRUE(router_.init()); }
    void TearDown() override { router_.shutdown(); if (t_.joinable()) t_.join(); }

    // Schedule a response corresponding to the request after the executor registers waiters.
    void scheduleResponse(const StoredDatagram& req, const std::vector<uint8_t>& payload, uint16_t wkc = 1) {
        // schedule on background thread to avoid races with executor waiters
        t_ = std::thread([this, req, payload, wkc]() {
            for (int i = 0; i < 200; ++i) {
                if (router_.hasWaiters()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            RxDatagram rx{};
            rx.idx = req.idx;
            rx.cmd = req.cmd;
            rx.adp = req.adp == 0xFFFF ? 0 : req.adp;
            rx.ado = req.ado;
            rx.wkc = wkc;
            rx.datalen = static_cast<uint16_t>(std::min<size_t>(payload.size(), sizeof(rx.data)));
            if (rx.datalen > 0) std::memcpy(rx.data, payload.data(), rx.datalen);
            (void)router_.routePacket(rx);
        });
    }

    ConditionalPacketRouter router_;
    std::thread t_;
};

TEST_F(SiiRetryTest, RetrySucceedsAfterOneTimeout_andStatsUpdated) {
    // Arrange: first attempt will time out, second will deliver payload
    const uint8_t idx = 7;
    StoredDatagram req = EtherCAT::Raw::buildBRD(idx, 0x0120, 2);

    std::atomic<int> send_calls{0};
    RetryExecutor exec(router_, [&send_calls, this](const StoredDatagram& r) {
        ++send_calls;
        // schedule response only on second send
        if (send_calls.load() > 1) scheduleResponse(r, {0xAB, 0xCD}, /*wkc=*/1);
        return true; // send succeeds
    });

    PacketFilter filter = PacketFilter::byIndex(idx);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[4] = {};
    RetryPolicy pol;
    pol.max_retries = 1;              // allow one retry
    pol.initial_timeout_ms = 5;      // short timeout for test speed
    pol.use_exponential_backoff = false;

    // Act
    auto res = exec.execute(req, filter, buf, sizeof(buf), pol);

    // Assert
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.attempts, 2u);
    EXPECT_EQ(buf[0], 0xAB);

    auto stats = exec.getStats();
    EXPECT_EQ(stats.total_requests, 2u);
    EXPECT_EQ(stats.retry_success, 1u);
    EXPECT_GE(stats.total_timeouts, 1u);
}

TEST_F(SiiRetryTest, SendFailureThenRetrySucceeds_andLogStatsCallable) {
    // Arrange: first send() fails, subsequent send schedules a response
    const uint8_t idx = 8;
    StoredDatagram req = EtherCAT::Raw::buildBRD(idx, 0x0120, 1);

    std::atomic<int> attempts{0};
    RetryExecutor exec(router_, [&attempts, this](const StoredDatagram& r) {
        int n = ++attempts;
        if (n == 1) return false;             // simulate immediate send failure
        scheduleResponse(r, {0x42}, /*wkc=*/1);
        return true;
    });

    PacketFilter filter = PacketFilter::byIndex(idx);
    filter.match_command = true;
    filter.command = EtherCAT::Command::BRD;

    uint8_t buf[4] = {};
    RetryPolicy pol;
    pol.max_retries = 1;
    pol.initial_timeout_ms = 20;
    pol.use_exponential_backoff = false;

    // Act
    auto res = exec.execute(req, filter, buf, sizeof(buf), pol);

    // Assert
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.attempts, 2u);
    EXPECT_EQ(buf[0], 0x42);

    // Ensure logStats() is callable (exercise coverage) and does not throw
    EXPECT_NO_THROW(exec.logStats());
}
