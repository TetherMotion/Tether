/**
 * @file test_ConditionalPacketRouter.cpp
 * @brief Tests for ConditionalPacketRouter: slot management, cancel, routing
 */
#include <gtest/gtest.h>
#include <cstring>
#include "tether/ethercat/ConditionalPacketRouter.hpp"
#include "tether/ethercat/Master.hpp"

using namespace EtherCAT;

// ============================================================================
// Fixture: creates a dedicated router per test
// ============================================================================
class RouterTest : public ::testing::Test {
protected:
    void SetUp() override {
        router_.init();
    }

    void TearDown() override {
        router_.shutdown();
    }

    ConditionalPacketRouter router_;
};

// ============================================================================
// Slot management
// ============================================================================

TEST_F(RouterTest, InitialState_NoWaiters) {
    EXPECT_FALSE(router_.hasWaiters());
    EXPECT_EQ(router_.waiterCount(), 0u);
}

TEST_F(RouterTest, PreRegister_IncreasesWaiterCount) {
    uint8_t buf[64]{};
    PacketFilter f;
    f.match_idx = true;
    f.idx = 42;
    size_t slot = router_.preRegisterWaiter(f, buf, sizeof(buf));
    ASSERT_LT(slot, ConditionalPacketRouter::kMaxWaiters);

    EXPECT_TRUE(router_.hasWaiters());
    EXPECT_EQ(router_.waiterCount(), 1u);

    router_.cancelPreRegistered(slot);
}

TEST_F(RouterTest, CancelPreRegistered_ReleasesSlot) {
    uint8_t buf[64]{};
    PacketFilter f;
    f.match_idx = true;
    f.idx = 1;

    size_t slot = router_.preRegisterWaiter(f, buf, sizeof(buf));
    ASSERT_LT(slot, ConditionalPacketRouter::kMaxWaiters);
    EXPECT_EQ(router_.waiterCount(), 1u);

    router_.cancelPreRegistered(slot);
    EXPECT_EQ(router_.waiterCount(), 0u);
    EXPECT_FALSE(router_.hasWaiters());
}

TEST_F(RouterTest, CancelPreRegistered_InvalidSlot_NoOp) {
    // Should not crash or change state
    router_.cancelPreRegistered(ConditionalPacketRouter::kMaxWaiters);
    router_.cancelPreRegistered(ConditionalPacketRouter::kMaxWaiters + 100);
    EXPECT_EQ(router_.waiterCount(), 0u);
}

TEST_F(RouterTest, PreRegister_MultipleSlotsAndCancel) {
    constexpr size_t N = 5;
    uint8_t bufs[N][64]{};
    size_t slots[N];

    for (size_t i = 0; i < N; i++) {
        PacketFilter f;
        f.match_idx = true;
        f.idx = static_cast<uint8_t>(i);
        slots[i] = router_.preRegisterWaiter(f, bufs[i], sizeof(bufs[i]));
        ASSERT_LT(slots[i], ConditionalPacketRouter::kMaxWaiters);
    }
    EXPECT_EQ(router_.waiterCount(), N);

    // Cancel middle slot
    router_.cancelPreRegistered(slots[2]);
    EXPECT_EQ(router_.waiterCount(), N - 1);

    // Cancel remaining
    for (size_t i = 0; i < N; i++) {
        if (i != 2) router_.cancelPreRegistered(slots[i]);
    }
    EXPECT_EQ(router_.waiterCount(), 0u);
}

TEST_F(RouterTest, SlotExhaustion_ReturnsKMaxWaiters) {
    uint8_t bufs[ConditionalPacketRouter::kMaxWaiters + 1][64]{};
    size_t slots[ConditionalPacketRouter::kMaxWaiters];

    for (size_t i = 0; i < ConditionalPacketRouter::kMaxWaiters; i++) {
        PacketFilter f;
        f.match_idx = true;
        f.idx = static_cast<uint8_t>(i & 0xFF);
        slots[i] = router_.preRegisterWaiter(f, bufs[i], sizeof(bufs[i]));
        ASSERT_LT(slots[i], ConditionalPacketRouter::kMaxWaiters) << "Slot " << i;
    }

    // Next one should fail
    PacketFilter f;
    f.match_idx = true;
    f.idx = 0xFF;
    size_t overflow = router_.preRegisterWaiter(f, bufs[ConditionalPacketRouter::kMaxWaiters],
                                                sizeof(bufs[0]));
    EXPECT_EQ(overflow, ConditionalPacketRouter::kMaxWaiters);

    // After cancel, should be able to allocate again at the same slot index
    router_.cancelPreRegistered(slots[0]);
    PacketFilter f_reclaim;
    f_reclaim.match_idx = true;
    f_reclaim.idx = 0;  // Must match the canceled slot index
    size_t reclaimed = router_.preRegisterWaiter(f_reclaim, bufs[0], sizeof(bufs[0]));
    EXPECT_LT(reclaimed, ConditionalPacketRouter::kMaxWaiters);

    // Clean up
    for (size_t i = 1; i < ConditionalPacketRouter::kMaxWaiters; i++) {
        router_.cancelPreRegistered(slots[i]);
    }
    router_.cancelPreRegistered(reclaimed);
}

TEST_F(RouterTest, WaitForPreRegistered_TimesOut) {
    uint8_t buf[64]{};
    PacketFilter f;
    f.match_idx = true;
    f.idx = 99;

    size_t slot = router_.preRegisterWaiter(f, buf, sizeof(buf));
    ASSERT_LT(slot, ConditionalPacketRouter::kMaxWaiters);

    auto result = router_.waitForPreRegistered(slot, 1); // 1ms timeout
    EXPECT_FALSE(result.success);
}

TEST_F(RouterTest, Stats_TrackRegistrations) {
    router_.resetStats();
    uint8_t buf[64]{};
    PacketFilter f;
    f.match_idx = true;
    f.idx = 0;

    size_t slot = router_.preRegisterWaiter(f, buf, sizeof(buf));
    auto stats = router_.getStats();
    EXPECT_GE(stats.registrations, 1u);

    router_.cancelPreRegistered(slot);
}

// ============================================================================
// Master: allocIdx / resetIdx
// ============================================================================

TEST(EtherCATMasterBasic, AllocIdx_Increments) {
    Master m;
    m.resetIdx();
    uint8_t first = m.allocIdx();
    uint8_t second = m.allocIdx();
    EXPECT_NE(first, second);
}

TEST(EtherCATMasterBasic, ResetIdx_ResetsSequence) {
    Master m;
    m.allocIdx();
    m.allocIdx();
    m.resetIdx();
    uint8_t after_reset = m.allocIdx();
    m.resetIdx();
    uint8_t after_reset2 = m.allocIdx();
    EXPECT_EQ(after_reset, after_reset2);
}

TEST(EtherCATMasterBasic, AdpForSlaveIndex) {
    // Auto-increment addressing: adp = 0 - slave_index
    EXPECT_EQ(Master::adpForSlaveIndex(0), 0u);
    EXPECT_EQ(Master::adpForSlaveIndex(1), static_cast<uint16_t>(0u - 1u));
    EXPECT_EQ(Master::adpForSlaveIndex(10), static_cast<uint16_t>(0u - 10u));
}

// ============================================================================
// Master: test hook round-trip
// ============================================================================

TEST(EtherCATMasterBasic, AprdCallback_ShortCircuits) {
    Master m;
    m.packetRouter().init();

    bool called = false;
    m.setAprdTestCallback(
        [&](uint16_t, uint16_t, void*, uint16_t, unsigned int) -> bool {
            called = true;
            return false;
        });

    uint16_t dummy = 0;
    bool ok = m.readRegister(0, 0x1234, dummy, 10);
    EXPECT_TRUE(called);
    EXPECT_FALSE(ok);

    m.setAprdTestCallback(nullptr);
    m.packetRouter().shutdown();
}

TEST(EtherCATMasterBasic, ApwrCallback_ShortCircuits) {
    Master m;
    m.packetRouter().init();

    bool called = false;
    m.setApwrTestCallback(
        [&](uint16_t, uint16_t, const void*, uint16_t, unsigned int) -> bool {
            called = true;
            return true;
        });

    uint16_t val = 42;
    bool ok = m.writeRegister(0, 0x1234, &val, sizeof(val), 10);
    EXPECT_TRUE(called);
    EXPECT_TRUE(ok);

    m.setApwrTestCallback(nullptr);
    m.packetRouter().shutdown();
}

TEST(EtherCATMasterBasic, PushAndConsumeAprdResponse) {
    Master m;
    m.packetRouter().init();

    uint32_t data = 0xDEADBEEF;
    m.pushAprdResponse(true, 0x0000, 0x0508, &data, sizeof(data));

    uint32_t out = 0;
    bool ok = m.readRegister(0x0000, 0x0508, out, 10);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, 0xDEADBEEF);

    m.clearAprdResponses();
    m.packetRouter().shutdown();
}

// ============================================================================
// Direct master tests (formerly bridge tests)
// ============================================================================

namespace EtherCAT { namespace Raw {
    uint16_t adp_for_slave_index(uint16_t slave_index);
}}

TEST(MasterFunctions, AllocIdxAndReset) {
    EtherCAT::Master master;
    master.packetRouter().init();

    master.resetIdx();
    uint8_t a = master.allocIdx();
    uint8_t b = master.allocIdx();
    EXPECT_NE(a, b);

    master.packetRouter().shutdown();
}

TEST(MasterFunctions, AdpForSlaveIndex) {
    using namespace EtherCAT::Raw;
    EXPECT_EQ(adp_for_slave_index(0), 0u);
    EXPECT_EQ(adp_for_slave_index(3), static_cast<uint16_t>(0u - 3u));
}

TEST(MasterFunctions, SrcMacDefaultsToZero) {
    EtherCAT::Master master;

    const uint8_t* got = master.getSrcMac();
    ASSERT_NE(got, nullptr);
    // Default MAC is all zeros
    const uint8_t zeros[6] = {0};
    EXPECT_EQ(std::memcmp(got, zeros, 6), 0);
}