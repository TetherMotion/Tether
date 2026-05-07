/**
 * @file test_jitter_monitor.cpp
 * @brief Unit tests for RealtimeJitterMonitor self-diagnosis
 *
 * Tests the jitter monitor independently — feeds synthetic timestamps
 * and verifies that jitter, warning, and critical counters behave as
 * expected.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/RealtimeJitterMonitor.hpp"

using namespace EtherCAT;

// ============================================================================
// Helper
// ============================================================================

static constexpr uint64_t US_TO_NS = 1000;

// ============================================================================
// Construction Tests
// ============================================================================

TEST(JitterMonitorConstruction, DefaultConfig) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    EXPECT_EQ(cfg.expected_period_us, 1000u);
    EXPECT_EQ(cfg.warning_threshold_us, 500u);      // 50 % of period
    EXPECT_EQ(cfg.critical_threshold_us, 2000u);     // 200 % of period
    EXPECT_EQ(cfg.log_interval_cycles, 10000u);
}

TEST(JitterMonitorConstruction, CustomPeriod) {
    JitterConfig cfg = JitterConfig::defaults(500);
    EXPECT_EQ(cfg.expected_period_us, 500u);
    EXPECT_EQ(cfg.warning_threshold_us, 250u);
    EXPECT_EQ(cfg.critical_threshold_us, 1000u);
}

TEST(JitterMonitorConstruction, StatsZeroInitially) {
    RealtimeJitterMonitor mon(JitterConfig::defaults(1000), "test");
    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 0u);
    EXPECT_EQ(s.max_jitter_us, 0u);
    EXPECT_EQ(s.avg_jitter_us, 0u);
    EXPECT_EQ(s.warning_count, 0u);
    EXPECT_EQ(s.critical_count, 0u);
    EXPECT_EQ(s.last_period_us, 0u);
    EXPECT_TRUE(s.realtime_ok);
}

// ============================================================================
// Perfect Timing — no jitter
// ============================================================================

TEST(JitterMonitorPerfect, NoJitterProduced) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0; // disable logging for tests
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 1'000'000'000ULL; // 1 s start
    for (int i = 0; i < 100; i++) {
        mon.recordCycle(t);
        t += 1000 * US_TO_NS; // exactly 1000 us later
    }

    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 100u);
    EXPECT_EQ(s.max_jitter_us, 0u);
    EXPECT_EQ(s.warning_count, 0u);
    EXPECT_EQ(s.critical_count, 0u);
    EXPECT_EQ(s.last_period_us, 1000u);
    EXPECT_TRUE(s.realtime_ok);
}

// ============================================================================
// Jitter within normal bounds
// ============================================================================

TEST(JitterMonitorNormal, SmallJitterIsTracked) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    // First cycle (baseline)
    uint64_t t = 0;
    mon.recordCycle(t);

    // Second cycle: 50 us late → jitter = 50 us (well below 500 us warning)
    t += 1050 * US_TO_NS;
    mon.recordCycle(t);

    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 2u);
    EXPECT_EQ(s.max_jitter_us, 50u);
    EXPECT_EQ(s.last_period_us, 1050u);
    EXPECT_EQ(s.warning_count, 0u);
    EXPECT_EQ(s.critical_count, 0u);
    EXPECT_TRUE(s.realtime_ok);
}

// ============================================================================
// Warning threshold
// ============================================================================

TEST(JitterMonitorWarning, WarningCountIncrements) {
    JitterConfig cfg;
    cfg.expected_period_us    = 1000;
    cfg.warning_threshold_us  = 200;
    cfg.critical_threshold_us = 5000;
    cfg.log_interval_cycles   = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    mon.recordCycle(t);

    // Normal cycle — 100 us jitter (below 200 warning)
    t += 1100 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().warning_count, 0u);

    // Late cycle — 300 us jitter (above 200 warning)
    t += 1300 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().warning_count, 1u);

    // Another late cycle — 250 us jitter
    t += 1250 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().warning_count, 2u);

    EXPECT_TRUE(mon.getStats().realtime_ok); // no critical overrun
}

// ============================================================================
// Critical threshold
// ============================================================================

TEST(JitterMonitorCritical, CriticalSetsRealtimeNotOk) {
    JitterConfig cfg;
    cfg.expected_period_us    = 1000;
    cfg.warning_threshold_us  = 200;
    cfg.critical_threshold_us = 1000;
    cfg.log_interval_cycles   = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    mon.recordCycle(t);

    // Massive delay: 3000 us jitter (above 1000 critical)
    t += 4000 * US_TO_NS;
    mon.recordCycle(t);

    auto s = mon.getStats();
    EXPECT_EQ(s.critical_count, 1u);
    EXPECT_GE(s.warning_count, 1u); // also above warning
    EXPECT_FALSE(s.realtime_ok);
    EXPECT_EQ(s.max_jitter_us, 3000u);
}

TEST(JitterMonitorCritical, RealtimeOkRemainsfalseAfterCritical) {
    JitterConfig cfg;
    cfg.expected_period_us    = 1000;
    cfg.warning_threshold_us  = 500;
    cfg.critical_threshold_us = 2000;
    cfg.log_interval_cycles   = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    mon.recordCycle(t);

    // One critical overrun
    t += 5000 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_FALSE(mon.getStats().realtime_ok);

    // Followed by many perfect cycles
    for (int i = 0; i < 50; i++) {
        t += 1000 * US_TO_NS;
        mon.recordCycle(t);
    }

    // realtime_ok remains false (sticky)
    EXPECT_FALSE(mon.getStats().realtime_ok);
    EXPECT_EQ(mon.getStats().critical_count, 1u);
}

// ============================================================================
// Early cycle (jitter calculated as absolute difference)
// ============================================================================

TEST(JitterMonitorEarly, EarlyCycleCountsAsJitter) {
    JitterConfig cfg;
    cfg.expected_period_us    = 1000;
    cfg.warning_threshold_us  = 200;
    cfg.critical_threshold_us = 5000;
    cfg.log_interval_cycles   = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 1'000'000'000ULL;
    mon.recordCycle(t);

    // Early by 400 us → period = 600 us, jitter = 400 us > 200 warning
    t += 600 * US_TO_NS;
    mon.recordCycle(t);

    EXPECT_EQ(mon.getStats().warning_count, 1u);
    EXPECT_EQ(mon.getStats().max_jitter_us, 400u);
    EXPECT_EQ(mon.getStats().last_period_us, 600u);
}

// ============================================================================
// Max Jitter
// ============================================================================

TEST(JitterMonitorMax, MaxJitterTracksWorstCase) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    mon.recordCycle(t);

    // 100 us jitter
    t += 1100 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().max_jitter_us, 100u);

    // 200 us jitter — new max
    t += 1200 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().max_jitter_us, 200u);

    // 50 us jitter — max stays at 200
    t += 1050 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().max_jitter_us, 200u);
}

// ============================================================================
// EWMA Average
// ============================================================================

TEST(JitterMonitorAvg, EwmaConverges) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    mon.recordCycle(t);

    // Constant 100 us jitter for many cycles
    for (int i = 0; i < 100; i++) {
        t += 1100 * US_TO_NS;
        mon.recordCycle(t);
    }

    // EWMA should converge close to 100 us
    auto avg = mon.getStats().avg_jitter_us;
    EXPECT_GE(avg, 80u);
    EXPECT_LE(avg, 120u);
}

// ============================================================================
// Reset
// ============================================================================

TEST(JitterMonitorReset, ResetClearsAllStats) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    uint64_t t = 0;
    for (int i = 0; i < 10; i++) {
        mon.recordCycle(t);
        t += 2000 * US_TO_NS; // 1000 us jitter (above warning)
    }

    EXPECT_GT(mon.getStats().cycle_count, 0u);
    EXPECT_GT(mon.getStats().max_jitter_us, 0u);

    mon.reset();

    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 0u);
    EXPECT_EQ(s.max_jitter_us, 0u);
    EXPECT_EQ(s.avg_jitter_us, 0u);
    EXPECT_EQ(s.warning_count, 0u);
    EXPECT_EQ(s.critical_count, 0u);
    EXPECT_EQ(s.last_period_us, 0u);
    EXPECT_TRUE(s.realtime_ok);
}

// ============================================================================
// First cycle is baseline (no jitter computed)
// ============================================================================

TEST(JitterMonitorFirstCycle, FirstCycleDoesNotComputeJitter) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "test");

    mon.recordCycle(999'999'999ULL);

    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 1u);
    EXPECT_EQ(s.max_jitter_us, 0u);
    EXPECT_EQ(s.last_period_us, 0u);
    EXPECT_EQ(s.warning_count, 0u);
}

// ============================================================================
// Different periods (e.g. DC thread at 10 ms)
// ============================================================================

TEST(JitterMonitorDCThread, TenMillisecondPeriod) {
    JitterConfig cfg = JitterConfig::defaults(10000); // 10 ms period
    cfg.log_interval_cycles = 0;
    RealtimeJitterMonitor mon(cfg, "dc");

    EXPECT_EQ(cfg.expected_period_us, 10000u);
    EXPECT_EQ(cfg.warning_threshold_us, 5000u);
    EXPECT_EQ(cfg.critical_threshold_us, 20000u);

    uint64_t t = 0;
    mon.recordCycle(t);

    // Perfect 10 ms cycle
    t += 10000 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().max_jitter_us, 0u);

    // 2 ms late → 2000 us jitter (below 5000 warning)
    t += 12000 * US_TO_NS;
    mon.recordCycle(t);
    EXPECT_EQ(mon.getStats().max_jitter_us, 2000u);
    EXPECT_EQ(mon.getStats().warning_count, 0u);
}

// ============================================================================
// Periodic log summary
// ============================================================================

TEST(JitterMonitorLog, PeriodicSummaryFiresAtInterval) {
    JitterConfig cfg = JitterConfig::defaults(1000);
    cfg.log_interval_cycles = 5; // Log every 5 cycles
    RealtimeJitterMonitor mon(cfg, "logtest");

    uint64_t t = 0;
    // Run 6 cycles: first is baseline, then 5 more → cycle_count will be 6
    // The log fires when cycle_count > 1 && cycle_count % 5 == 0, i.e. at cycle 5.
    for (int i = 0; i < 6; ++i) {
        mon.recordCycle(t);
        t += 1000 * US_TO_NS; // perfect 1 ms period
    }

    auto s = mon.getStats();
    EXPECT_EQ(s.cycle_count, 6u);
    EXPECT_EQ(s.max_jitter_us, 0u);
    // Test just verifies no crash and cycles are counted correctly.
    // The TETHER_LOGI is a side effect (printed to console).
}
