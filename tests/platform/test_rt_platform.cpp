/**
 * @file test_rt_platform.cpp
 * @brief Tests for CpuIsolation (runtime CPU claim allocator) and
 *        RtMemory (mlock sections + prefault + timer slack).
 *
 * Everything here is unprivileged-safe: claims are bookkeeping, mlock is
 * best-effort and only asserts when the environment permits it.
 */

#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>

#include "tether/platform/CpuIsolation.hpp"
#include "tether/platform/RtMemory.hpp"
#include "tether/platform/Platform.hpp"

using Tether::Platform::CpuIsolation;
using namespace Tether::Platform;

// ============================================================================
// CpuIsolation
// ============================================================================

class CpuIsolationTest : public ::testing::Test {
protected:
    void TearDown() override {
        CpuIsolation::instance().releaseAll();
    }
};

TEST_F(CpuIsolationTest, ReportsOnlineCpus) {
    auto& iso = CpuIsolation::instance();
    EXPECT_GE(iso.onlineCount(), 1);
    // Initially nothing claimed by this process (or left over).
    EXPECT_GE(iso.freeCount(), 1);
}

TEST_F(CpuIsolationTest, ClaimReturnsAValidCpu) {
    auto& iso = CpuIsolation::instance();
    if (iso.onlineCount() < 2) GTEST_SKIP() << "need >=2 CPUs for a claim";

    CpuIsolation::Spec spec;
    auto c = iso.claim(spec);
    ASSERT_TRUE(c.valid());
    EXPECT_GE(c.cpu, 0);
    EXPECT_LT(c.cpu, 256);
    // The claim is visible in the registry.
    auto claimed = iso.claimedCpus();
    EXPECT_NE(std::find(claimed.begin(), claimed.end(), c.cpu),
              claimed.end());
}

TEST_F(CpuIsolationTest, NeverGrantsLastCpu) {
    auto& iso = CpuIsolation::instance();
    const int online = iso.onlineCount();
    if (online < 2) GTEST_SKIP() << "single-CPU system — nothing claimable";

    // Claim until refusal — the allocator must always refuse the last one.
    int granted = 0;
    for (int i = 0; i < online + 2; ++i) {
        auto c = iso.claim(CpuIsolation::Spec{});
        if (!c.valid()) break;
        ++granted;
    }
    EXPECT_LE(granted, online - 1)
        << "allocator granted every online CPU — must keep one free";
    EXPECT_GE(iso.freeCount(), 1);
}

TEST_F(CpuIsolationTest, ExplicitRequestHonouredAndConflictDetected) {
    auto& iso = CpuIsolation::instance();
    if (iso.onlineCount() < 2) GTEST_SKIP();

    const int cpu = iso.onlineCount() - 1;  // last CPU: rarely CPU0
    CpuIsolation::Spec spec;
    spec.requested_cpu = cpu;
    auto c = iso.claim(spec);
    ASSERT_TRUE(c.valid());
    EXPECT_EQ(c.cpu, cpu);

    // Second claim for the same CPU must be denied.
    auto dup = iso.claim(spec);
    EXPECT_FALSE(dup.valid());
}

TEST_F(CpuIsolationTest, ReleaseReturnsCpuToPool) {
    auto& iso = CpuIsolation::instance();
    if (iso.onlineCount() < 2) GTEST_SKIP();

    auto c = iso.claim(CpuIsolation::Spec{});
    ASSERT_TRUE(c.valid());
    const int before = iso.freeCount();
    iso.release(c.cpu);
    EXPECT_EQ(iso.freeCount(), before + 1);
    // Idempotent — releasing again is a no-op.
    iso.release(c.cpu);
    EXPECT_EQ(iso.freeCount(), before + 1);
}

TEST_F(CpuIsolationTest, ReleaseAllClearsEverything) {
    auto& iso = CpuIsolation::instance();
    if (iso.onlineCount() < 3) GTEST_SKIP();
    iso.claim(CpuIsolation::Spec{});
    iso.claim(CpuIsolation::Spec{});
    iso.releaseAll();
    EXPECT_TRUE(iso.claimedCpus().empty());
}

TEST_F(CpuIsolationTest, OfflineCpuRequestDenied) {
    auto& iso = CpuIsolation::instance();
    CpuIsolation::Spec spec;
    spec.requested_cpu = 250;   // almost certainly not online
    // If the box genuinely has 251+ CPUs, pick one past the max instead.
    if (spec.requested_cpu < iso.onlineCount()) spec.requested_cpu = 255;
    auto c = iso.claim(spec);
    EXPECT_FALSE(c.valid());
}

// ============================================================================
// RtMemory
// ============================================================================

TEST(RtMemoryTest, PrefaultRegionIsHarmless) {
    // Unprivileged and always safe — just asserts it runs.
    auto buf = std::make_unique<uint8_t[]>(4 * 4096);
    prefaultMemory(buf.get(), 4 * 4096);
    buf[12345] = 42;
    EXPECT_EQ(buf[12345], 42);
}

TEST(RtMemoryTest, LockUnlockRegionBestEffort) {
    auto buf = std::make_unique<uint8_t[]>(4096);
    // mlock may fail without CAP_IPC_LOCK / RLIMIT_MEMLOCK — both are fine;
    // the contract is "best-effort, never fatal".
    const bool locked = lockMemory(buf.get(), 4096);
    if (locked) {
        EXPECT_TRUE(unlockMemory(buf.get(), 4096));
    }
    SUCCEED();
}

TEST(RtMemoryTest, LockAllBestEffort) {
    // Same contract — just exercise the call.
    (void)lockAllMemory();
    SUCCEED();
}

TEST(RtMemoryTest, PrefaultStackDoesNotCrash) {
    prefaultCurrentStack(64 * 1024);
    SUCCEED();
}

TEST(RtMemoryTest, TimerSlackApplies) {
    // 1ns slack — unprivileged, always succeeds on Linux.
    EXPECT_TRUE(setCurrentThreadTimerSlack(1));
}

TEST(RtMemoryTest, SchedDeadlineFallbackIsGraceful) {
    // SCHED_DEADLINE may be unavailable (policy/capability); the contract
    // is "returns false, caller falls back" — never crash.
    (void)setCurrentThreadDeadline(50'000, 100'000, 100'000);
    SUCCEED();
}
