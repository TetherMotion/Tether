/**
 * @file test_transaction_router_waitany.cpp
 * @brief Tests for TransactionRouter::waitForAny() multi-slot wait primitive
 */
#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>
#include "tether/ethercat/TransactionRouter.hpp"

using namespace EtherCAT;

// ============================================================================
// Fixture: dedicated router per test
// ============================================================================
class WaitForAnyTest : public ::testing::Test {
protected:
    void SetUp() override { router_.init(); }
    void TearDown() override { router_.shutdown(); }

    TransactionRouter router_;

    // Helper: pre-register a waiter with a given idx
    size_t preRegister(uint8_t idx, uint8_t* buf, size_t sz) {
        PacketFilter f = PacketFilter::byIndex(idx);
        return router_.preRegisterWaiter(f, buf, sz);
    }

    // Helper: route a fake response to a slot
    void routeResponse(uint8_t idx, uint16_t wkc, const uint8_t* data, uint16_t len) {
        RxDatagram dg{};
        dg.idx = idx;
        dg.cmd = Command::APRD;
        dg.adp = 0;
        dg.ado = 0;
        dg.wkc = wkc;
        dg.datalen = len;
        if (data && len) std::memcpy(dg.data, data, len);
        router_.routePacket(dg);
    }
};

// ============================================================================
// Basic waitForAny tests
// ============================================================================

TEST_F(WaitForAnyTest, EmptySlots_ReturnsTimedOut) {
    auto r = router_.waitForAny(nullptr, 0, 10);
    EXPECT_TRUE(r.timed_out);
}

TEST_F(WaitForAnyTest, NoCompletion_TimesOut) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(1, buf, sizeof(buf));
    size_t s2 = preRegister(2, buf, sizeof(buf));

    size_t slots[2] = {s1, s2};
    auto r = router_.waitForAny(slots, 2, 10);
    EXPECT_TRUE(r.timed_out);

    router_.cancelPreRegistered(s1);
    router_.cancelPreRegistered(s2);
}

TEST_F(WaitForAnyTest, FirstSlotAlreadyCompleted_ReturnsImmediately) {
    uint8_t buf1[16]{};
    uint8_t buf2[16]{};
    size_t s1 = preRegister(10, buf1, sizeof(buf1));
    size_t s2 = preRegister(20, buf2, sizeof(buf2));

    // Complete slot 1 before calling waitForAny
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    routeResponse(10, 1, data, sizeof(data));

    size_t slots[2] = {s1, s2};
    auto r = router_.waitForAny(slots, 2, 100);
    EXPECT_FALSE(r.timed_out);
    EXPECT_EQ(r.slot_index, 0u);  // s1 is at index 0
    EXPECT_EQ(r.result.wkc, 1u);

    router_.cancelPreRegistered(s2);
}

TEST_F(WaitForAnyTest, SecondSlotAlreadyCompleted_ReturnsSecond) {
    uint8_t buf1[16]{};
    uint8_t buf2[16]{};
    size_t s1 = preRegister(10, buf1, sizeof(buf1));
    size_t s2 = preRegister(20, buf2, sizeof(buf2));

    uint8_t data[4] = {0x11, 0x22, 0x33, 0x44};
    routeResponse(20, 1, data, sizeof(data));

    size_t slots[2] = {s1, s2};
    auto r = router_.waitForAny(slots, 2, 100);
    EXPECT_FALSE(r.timed_out);
    EXPECT_EQ(r.slot_index, 1u);  // s2 is at index 1
    EXPECT_EQ(r.result.wkc, 1u);

    router_.cancelPreRegistered(s1);
}

TEST_F(WaitForAnyTest, CompletionAfterWait_Returns) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(5, buf, sizeof(buf));

    // Spawn a thread that completes the slot after 20ms
    std::thread completer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        uint8_t data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
        routeResponse(5, 1, data, sizeof(data));
    });

    size_t slots[1] = {s1};
    auto r = router_.waitForAny(slots, 1, 500);
    EXPECT_FALSE(r.timed_out);
    EXPECT_EQ(r.slot_index, 0u);
    EXPECT_EQ(r.result.wkc, 1u);

    completer.join();
}

TEST_F(WaitForAnyTest, MultipleSlots_OneCompletes_ReturnsCorrectOne) {
    uint8_t bufs[3][16]{};
    size_t slots_arr[3];
    slots_arr[0] = preRegister(1, bufs[0], sizeof(bufs[0]));
    slots_arr[1] = preRegister(2, bufs[1], sizeof(bufs[1]));
    slots_arr[2] = preRegister(3, bufs[2], sizeof(bufs[2]));

    // Complete slot 2 (index 1 in array)
    std::thread completer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        uint8_t data[4] = {0x42, 0x42, 0x42, 0x42};
        routeResponse(2, 1, data, sizeof(data));
    });

    auto r = router_.waitForAny(slots_arr, 3, 500);
    EXPECT_FALSE(r.timed_out);
    EXPECT_EQ(r.slot_index, 1u);

    completer.join();

    router_.cancelPreRegistered(slots_arr[0]);
    router_.cancelPreRegistered(slots_arr[2]);
}

TEST_F(WaitForAnyTest, SlotConsumedOnce_SecondWaitTimesOut) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(7, buf, sizeof(buf));

    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};
    routeResponse(7, 1, data, sizeof(data));

    size_t slots[1] = {s1};
    auto r1 = router_.waitForAny(slots, 1, 100);
    EXPECT_FALSE(r1.timed_out);

    // Second wait should time out — slot was consumed
    auto r2 = router_.waitForAny(slots, 1, 10);
    EXPECT_TRUE(r2.timed_out);
}

TEST_F(WaitForAnyTest, Shutdown_WakesWaiter) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(9, buf, sizeof(buf));

    std::thread shutdowner([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        router_.shutdown();
    });

    size_t slots[1] = {s1};
    auto r = router_.waitForAny(slots, 1, 500);
    EXPECT_TRUE(r.timed_out);  // shutdown → timed_out

    shutdowner.join();
}

TEST_F(WaitForAnyTest, Cancel_WakesWaiter) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(11, buf, sizeof(buf));

    std::thread canceller([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        router_.cancel();
    });

    size_t slots[1] = {s1};
    auto r = router_.waitForAny(slots, 1, 500);
    EXPECT_TRUE(r.timed_out);  // cancel → timed_out

    canceller.join();
    router_.clearCancel();
    router_.cancelPreRegistered(s1);
}

TEST_F(WaitForAnyTest, MultipleCompletions_ReturnsOneAtATime) {
    uint8_t bufs[3][16]{};
    size_t slots_arr[3];
    slots_arr[0] = preRegister(1, bufs[0], sizeof(bufs[0]));
    slots_arr[1] = preRegister(2, bufs[1], sizeof(bufs[1]));
    slots_arr[2] = preRegister(3, bufs[2], sizeof(bufs[2]));

    // Complete all three slots
    uint8_t d1[4] = {1, 2, 3, 4};
    uint8_t d2[4] = {5, 6, 7, 8};
    uint8_t d3[4] = {9, 10, 11, 12};
    routeResponse(1, 1, d1, 4);
    routeResponse(2, 1, d2, 4);
    routeResponse(3, 1, d3, 4);

    // First call — should return one of the completed slots
    auto r1 = router_.waitForAny(slots_arr, 3, 100);
    EXPECT_FALSE(r1.timed_out);

    // Second call — should return another
    auto r2 = router_.waitForAny(slots_arr, 3, 100);
    EXPECT_FALSE(r2.timed_out);
    EXPECT_NE(r1.slot_index, r2.slot_index);

    // Third call — should return the last
    auto r3 = router_.waitForAny(slots_arr, 3, 100);
    EXPECT_FALSE(r3.timed_out);
    EXPECT_NE(r3.slot_index, r1.slot_index);
    EXPECT_NE(r3.slot_index, r2.slot_index);

    // Fourth call — all consumed, should time out
    auto r4 = router_.waitForAny(slots_arr, 3, 10);
    EXPECT_TRUE(r4.timed_out);
}

TEST_F(WaitForAnyTest, ConcurrentRouteAndWait_NoCrash) {
    // Stress test: multiple threads routing packets while one thread waits
    constexpr int N = 8;
    uint8_t bufs[N][16]{};
    size_t slots[N];
    for (int i = 0; i < N; ++i) {
        slots[i] = preRegister(static_cast<uint8_t>(i + 1), bufs[i], sizeof(bufs[i]));
    }

    std::atomic<int> routed{0};
    std::vector<std::thread> routers;
    for (int i = 0; i < N; ++i) {
        routers.emplace_back([&, i] {
            uint8_t data[4] = {static_cast<uint8_t>(i), 0, 0, 0};
            routeResponse(static_cast<uint8_t>(i + 1), 1, data, 4);
            routed.fetch_add(1);
        });
    }

    // Wait for all to complete
    int consumed = 0;
    while (consumed < N) {
        auto r = router_.waitForAny(slots, N, 500);
        if (!r.timed_out) {
            ++consumed;
        } else {
            break;
        }
    }
    EXPECT_EQ(consumed, N);

    for (auto& t : routers) t.join();
}

TEST_F(WaitForAnyTest, InvalidSlotIndex_Skipped) {
    uint8_t buf[16]{};
    size_t s1 = preRegister(1, buf, sizeof(buf));

    // Pass an invalid slot index alongside a valid one
    size_t slots[2] = {s1, 99999};  // 99999 is out of range
    auto r = router_.waitForAny(slots, 2, 10);
    EXPECT_TRUE(r.timed_out);  // s1 never completes

    router_.cancelPreRegistered(s1);
}
