/**
 * @file test_dc_consistency_coverage.cpp
 * @brief Coverage tests for DCConsistency.cpp
 *
 * Exercises every function in the file: pure utilities, consistency checks
 * (host stub provides a valid SlaveDCState), logging functions, and the
 * dc_full_diagnostic orchestrator.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/DCConsistency.hpp"
#include <string>

using namespace EtherCAT::DC;

// ============================================================================
// dc_format_time — pure function
// ============================================================================

TEST(DCFormatTime, TimeBelowEpoch) {
    // A small value < kMasterEpochNs should take the raw-display branch
    std::string s = dc_format_time(1234567890123ULL);
    EXPECT_GT(s.size(), 0u);
    // Should contain "s" (raw seconds display)
    EXPECT_NE(s.find('s'), std::string::npos);
}

TEST(DCFormatTime, TimeAtOrAboveEpoch) {
    // kMasterEpochNs = 1767225600000000000ULL (2026-01-01 00:00:00 UTC)
    constexpr uint64_t kEpoch = 1767225600000000000ULL;
    // Exactly at epoch (0 offset)
    std::string s = dc_format_time(kEpoch);
    EXPECT_GT(s.size(), 0u);
    EXPECT_NE(s.find("2026-01-01"), std::string::npos);

    // 1 hour + 30 minutes + 15 seconds + 123456789 ns after epoch
    uint64_t offset = 1ULL * 3600 * 1000000000ULL + 30ULL * 60 * 1000000000ULL +
                      15ULL * 1000000000ULL + 123456789ULL;
    s = dc_format_time(kEpoch + offset);
    EXPECT_GT(s.size(), 0u);
    EXPECT_NE(s.find("2026-01-01"), std::string::npos);
    EXPECT_NE(s.find("1h30m15"), std::string::npos);
}

// ============================================================================
// dc_calc_propagation_delay — pure function
// ============================================================================

TEST(DCCalcPropagationDelay, Port1GreaterThanPort0) {
    EXPECT_EQ(dc_calc_propagation_delay(100, 500), 400u);
}

TEST(DCCalcPropagationDelay, Port0GreaterOrEqual) {
    EXPECT_EQ(dc_calc_propagation_delay(500, 100), 500u);
    EXPECT_EQ(dc_calc_propagation_delay(500, 500), 500u);
}

// ============================================================================
// dc_is_time_in_future — pure function
// ============================================================================

TEST(DCIsTimeInFuture, Ahead) {
    EXPECT_TRUE(dc_is_time_in_future(2000, 1000, 0));
}

TEST(DCIsTimeInFuture, Behind) {
    EXPECT_FALSE(dc_is_time_in_future(1000, 2000, 0));
}

TEST(DCIsTimeInFuture, WithinMargin) {
    // reference=2000, time=1990, margin=100 => diff=10 < 100 => true
    EXPECT_TRUE(dc_is_time_in_future(1990, 2000, 100));
}

TEST(DCIsTimeInFuture, OutsideMargin) {
    // reference=2000, time=1000, margin=100 => diff=1000 >= 100 => false
    EXPECT_FALSE(dc_is_time_in_future(1000, 2000, 100));
}

// ============================================================================
// dc_time_diff — pure function
// ============================================================================

TEST(DCTimeDiff, PositiveDifference) {
    EXPECT_EQ(dc_time_diff(2000, 1000), 1000);
}

TEST(DCTimeDiff, NegativeDifference) {
    EXPECT_EQ(dc_time_diff(1000, 2000), -1000);
}

TEST(DCTimeDiff, Zero) {
    EXPECT_EQ(dc_time_diff(1000, 1000), 0);
}

TEST(DCTimeDiff, LargePositiveClampedToMax) {
    // time1 >> time2, diff > INT64_MAX
    uint64_t big = static_cast<uint64_t>(INT64_MAX) + 1ULL;
    EXPECT_EQ(dc_time_diff(big, 0), INT64_MAX);
}

TEST(DCTimeDiff, LargeNegativeClampedToMin) {
    uint64_t big = static_cast<uint64_t>(INT64_MAX) + 1ULL;
    EXPECT_EQ(dc_time_diff(0, big), INT64_MIN);
}

// ============================================================================
// dc_get_master_time_with_epoch — host stub
// ============================================================================

TEST(DCMasterTime, ReturnsNonZero) {
    uint64_t t = dc_get_master_time_with_epoch();
    EXPECT_GT(t, 0u);
}

// ============================================================================
// dc_read_slave_state — host stub
// ============================================================================

TEST(DCReadSlaveState, HostStubReturnsTrue) {
    SlaveDCState state;
    EXPECT_TRUE(dc_read_slave_state(0, state));
    EXPECT_TRUE(state.dc_supported);
    EXPECT_GT(state.system_time, 0u);
    EXPECT_EQ(state.sync0_cycle_time, 1000000u); // 1ms
}

// ============================================================================
// dc_run_consistency_checks — uses host stub
// ============================================================================

TEST(DCConsistencyChecks, RunsAllChecks) {
    DCConsistencyReport report;
    // Host stub defaults: dc_supported=true, dc_active=false, sync0_active=false,
    // sync0_start_time=0, sync0_cycle_time=1000000, propagation_delay=0, system_time_diff=0
    dc_run_consistency_checks(0, report);
    EXPECT_EQ(report.check_count, 7u);
    // At least dc_supported should pass
    EXPECT_GT(report.passed_count, 0u);
}

// ============================================================================
// dc_check_* wrappers
// ============================================================================

TEST(DCCheckSync0StartTime, HostStub) {
    // Host stub: sync0_start_time=0, system_time>0 => false
    EXPECT_FALSE(dc_check_sync0_start_time(0));
}

TEST(DCCheckPropagationDelay, HostStub) {
    // Host stub: propagation_delay=0, which is < 10000000 => true
    EXPECT_TRUE(dc_check_propagation_delay(0));
    EXPECT_TRUE(dc_check_propagation_delay(0, 1)); // 0 < 1
}

TEST(DCCheckSystemTimeOffset, HostStub) {
    // Host stub: system_time_offset=0 => false (expects non-zero)
    EXPECT_FALSE(dc_check_system_time_offset(0));
}

TEST(DCCheckSyncCycleTimes, AlwaysFalse) {
    // After DCContext removal, always returns false
    EXPECT_FALSE(dc_check_sync_cycle_times(0));
}

// ============================================================================
// Logging functions — just exercise for coverage
// ============================================================================

TEST(DCLogFunctions, LogSyncActivation) {
    dc_log_sync_activation(0x00); // all disabled
    dc_log_sync_activation(0xFF); // all enabled
}

TEST(DCLogFunctions, LogCyclicUnitControl) {
    dc_log_cyclic_unit_control(0x00);
    dc_log_cyclic_unit_control(0x11);
}

TEST(DCLogFunctions, LogSlaveState) {
    dc_log_slave_state(0);
}

TEST(DCLogFunctions, LogConsistencyReport) {
    DCConsistencyReport report;
    dc_run_consistency_checks(0, report);
    dc_log_consistency_report(report);
}

TEST(DCLogFunctions, LogSync0Timing) {
    dc_log_sync0_timing(0, dc_get_master_time_with_epoch());
}

// ============================================================================
// dc_full_diagnostic
// ============================================================================

TEST(DCFullDiagnostic, WithBroadcastIndex) {
    // 0xFFFF triggers the "cannot iterate" warning
    dc_full_diagnostic(0xFFFF);
}

TEST(DCFullDiagnostic, WithSpecificSlave) {
    dc_full_diagnostic(0);
}
