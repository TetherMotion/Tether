/**
 * @file test_transaction_router_purge.cpp
 * @brief Tests for TransactionRouter::purgeAllPending() — post-stall
 *        cleanup of pending waiters and stale-echo aliasing protection.
 *
 * Motivation: after a host stall, EtherCAT response frames can arrive long
 * after their request timed out.  The 8-bit datagram index wraps after
 * ~224 allocations, so a late echo can otherwise satisfy a NEW waiter on a
 * reused idx — feeding stale PDO data to the caller.  purgeAllPending()
 * frees every slot (waking blocked waiters into their timeout path) so a
 * late echo is unrouted instead.
 */
#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "tether/ethercat/TransactionRouter.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

// ============================================================================
// Fixture: dedicated router per test
// ============================================================================
class RouterPurgeTest : public ::testing::Test {
protected:
    void SetUp() override { router_.init(); }
    void TearDown() override { router_.shutdown(); }

    TransactionRouter router_;

    size_t preRegister(uint8_t idx, uint8_t* buf, size_t sz) {
        return router_.preRegisterWaiter(PacketFilter::byIndex(idx), buf, sz);
    }

    size_t routeResponse(uint8_t idx, uint16_t wkc,
                         const uint8_t* data, uint16_t len) {
        RxDatagram dg{};
        dg.idx = idx;
        dg.cmd = Command::LRW;
        dg.wkc = wkc;
        dg.datalen = len;
        if (data && len) std::memcpy(dg.data, data, len);
        return router_.routePacket(dg);
    }
};

// ============================================================================
// purgeAllPending basics
// ============================================================================

TEST_F(RouterPurgeTest, PurgeOnEmptyRouter_NoCrash) {
    router_.purgeAllPending();
    EXPECT_EQ(router_.waiterCount(), 0u);
}

TEST_F(RouterPurgeTest, PurgeClearsAllPendingWaiters) {
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(10, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    ASSERT_NE(preRegister(20, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    ASSERT_NE(preRegister(30, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    ASSERT_EQ(router_.waiterCount(), 3u);

    router_.purgeAllPending();

    EXPECT_EQ(router_.waiterCount(), 0u);
    EXPECT_FALSE(router_.hasWaiters());
}

TEST_F(RouterPurgeTest, PurgedSlot_LateEchoIsUnrouted) {
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(7, buf, sizeof(buf)), TransactionRouter::kNumSlots);

    router_.purgeAllPending();

    // The stale echo arriving after the stall must NOT match any waiter —
    // it falls into the unrouted/dropped counter.
    uint8_t stale[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    EXPECT_EQ(routeResponse(7, 1, stale, sizeof(stale)), 0u);
    EXPECT_EQ(router_.getStats().packets_dropped, 1u);
}

TEST_F(RouterPurgeTest, ReRegisterAfterPurge_WorksNormally) {
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(9, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    router_.purgeAllPending();

    // Same idx re-registered — the exact aliasing scenario after idx wrap.
    ASSERT_NE(preRegister(9, buf, sizeof(buf)), TransactionRouter::kNumSlots);

    uint8_t data[4] = {0x11, 0x22, 0x33, 0x44};
    EXPECT_EQ(routeResponse(9, 1, data, sizeof(data)), 1u);

    auto r = router_.waitForPreRegistered(9, 100);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.wkc, 1u);
    EXPECT_EQ(buf[0], 0x11);
}

TEST_F(RouterPurgeTest, NewRegistration_DoesNotSeeStaleEcho) {
    // Order matters: purge BEFORE re-registering the idx — this is what
    // LogicalAddressManager::stallCheck() guarantees (purge + drain happen
    // before the next sendSingleDatagram).
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(5, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    router_.purgeAllPending();

    // Old frame arrives during/after purge — no waiter pending → dropped.
    uint8_t stale[4] = {0x99, 0x99, 0x99, 0x99};
    EXPECT_EQ(routeResponse(5, 1, stale, sizeof(stale)), 0u);

    // Now the new request reuses the idx; its fresh response is delivered.
    ASSERT_NE(preRegister(5, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    uint8_t fresh[4] = {0x01, 0x02, 0x03, 0x04};
    EXPECT_EQ(routeResponse(5, 1, fresh, sizeof(fresh)), 1u);

    auto r = router_.waitForPreRegistered(5, 100);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(buf[0], 0x01);  // fresh data, not 0x99
}

// ============================================================================
// Blocked waiters
// ============================================================================

TEST_F(RouterPurgeTest, Purge_WakesBlockedWaiterEarly) {
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(42, buf, sizeof(buf)), TransactionRouter::kNumSlots);

    std::atomic<bool> done{false};
    WaitResult wr{};
    std::thread waiter([&] {
        wr = router_.waitForPreRegistered(42, 5000);
        done.store(true);
    });

    std::this_thread::sleep_for(20ms);
    const auto t0 = std::chrono::steady_clock::now();
    router_.purgeAllPending();

    waiter.join();
    const auto elapsed = std::chrono::steady_clock::now() - t0;

    EXPECT_TRUE(done.load());
    EXPECT_TRUE(wr.timeout);   // wake → timeout path, not success
    // Purge must wake the waiter immediately, not let it sleep out 5 s.
    EXPECT_LT(elapsed, 2s);
}

TEST_F(RouterPurgeTest, PurgedWaiter_CannotClobberNewRegistration) {
    // Sequence: waiter blocked on idx 5 → purge → NEW request re-registers
    // idx 5 before the old waiter's cleanup runs.  The stale cleanup must
    // not clear the new registration's pending flag (reg_gen guard).
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(5, buf, sizeof(buf)), TransactionRouter::kNumSlots);

    std::thread waiter([&] {
        auto wr = router_.waitForPreRegistered(5, 5000);
        EXPECT_TRUE(wr.timeout);
    });
    std::this_thread::sleep_for(20ms);

    router_.purgeAllPending();
    waiter.join();

    // Re-register the same idx and complete it.
    uint8_t buf2[16]{};
    ASSERT_NE(preRegister(5, buf2, sizeof(buf2)), TransactionRouter::kNumSlots);
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    EXPECT_EQ(routeResponse(5, 2, data, sizeof(data)), 1u);

    auto r = router_.waitForPreRegistered(5, 100);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.wkc, 2u);
    EXPECT_EQ(buf2[0], 0xAA);
}

TEST_F(RouterPurgeTest, Purge_WakesWaitForAnyWaiter) {
    uint8_t bufs[2][16]{};
    size_t s1 = preRegister(1, bufs[0], sizeof(bufs[0]));
    size_t s2 = preRegister(2, bufs[1], sizeof(bufs[1]));
    ASSERT_NE(s1, TransactionRouter::kNumSlots);
    ASSERT_NE(s2, TransactionRouter::kNumSlots);

    std::atomic<bool> done{false};
    std::thread waiter([&] {
        size_t slots[2] = {s1, s2};
        auto r = router_.waitForAny(slots, 2, 5000);
        EXPECT_TRUE(r.timed_out);
        done.store(true);
    });

    std::this_thread::sleep_for(20ms);
    const auto t0 = std::chrono::steady_clock::now();
    router_.purgeAllPending();

    waiter.join();
    EXPECT_TRUE(done.load());
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 2s);
}

TEST_F(RouterPurgeTest, WaitOnPurgedSlot_ReturnsImmediately) {
    // Purge between preRegister and waitForPreRegistered (e.g. another
    // thread's stall check ran in between): the wait must not sleep out
    // its budget on a dead slot.
    uint8_t buf[16]{};
    size_t s = preRegister(77, buf, sizeof(buf));
    ASSERT_NE(s, TransactionRouter::kNumSlots);
    router_.purgeAllPending();

    const auto t0 = std::chrono::steady_clock::now();
    auto r = router_.waitForPreRegistered(s, 5000);
    EXPECT_TRUE(r.timeout);
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
}

TEST_F(RouterPurgeTest, RepeatedPurges_Safe) {
    uint8_t buf[16]{};
    ASSERT_NE(preRegister(3, buf, sizeof(buf)), TransactionRouter::kNumSlots);
    router_.purgeAllPending();
    router_.purgeAllPending();
    router_.purgeAllPending();
    EXPECT_EQ(router_.waiterCount(), 0u);
}
