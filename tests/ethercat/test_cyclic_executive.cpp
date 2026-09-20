/**
 * @file test_cyclic_executive.cpp
 * @brief Unit tests for CyclicExecutive — the deadline-driven cyclic thread.
 *
 * The executive runs REAL threads against REAL deadline timers (the HAL
 * factory + HostDeadlineTimer/HybridDeadlineTimer), so these tests exercise
 * the actual scheduling loop: phase ordering, decimated inline DC, the
 * dedicated DC thread, both sleep modes, the Deadline→Fifo→normal
 * scheduling fallback ladder, stats accounting, and error handling.
 *
 * SCHED_FIFO/SCHED_DEADLINE need privileges the test container may lack —
 * the code logs and runs on normal scheduling, which is exactly the
 * degraded path these tests assert (loop still ticks, stats still count).
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "tether/ethercat/CyclicExecutive.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

using Exec = CyclicExecutive;

Exec::Config fastConfig(uint32_t period_us = 500) {
    auto c = Exec::Config::defaults(period_us, 4);
    c.dc_placement        = Exec::DCPlacement::Disabled;
    c.priority            = 10;      // may fail → degraded path is the test
    c.dc_priority         = 10;
    c.stack_prefault_bytes = 16 * 1024;
    return c;
}

/// Spin until pred() is true or the deadline passes.  Avoids fixed sleeps.
template <typename F>
bool waitFor(F&& pred, std::chrono::milliseconds budget = 3'000ms) {
    const auto end = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(200us);
    }
    return pred();
}

// ============================================================================
// Registration / lifecycle
// ============================================================================

TEST(CyclicExecutive, AddTaskRejectsExchangePhase) {
    Exec e(nullptr, nullptr, nullptr, fastConfig());
    EXPECT_FALSE(e.addTask(TaskPhase::Exchange, [] { return true; }));
    EXPECT_FALSE(e.addTask(TaskPhase::PreExchange, nullptr));  // null fn
    EXPECT_TRUE(e.addTask(TaskPhase::PreExchange, [] { return true; }));
    EXPECT_TRUE(e.addTask(TaskPhase::PostExchange, [] { return true; }));
    EXPECT_TRUE(e.addTask(TaskPhase::MotionControl, [] { return true; }));
    EXPECT_TRUE(e.addTask(TaskPhase::Diagnostics, [] { return true; }));
}

TEST(CyclicExecutive, AddTaskRejectedWhileRunning) {
    Exec e(nullptr, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.start());
    EXPECT_TRUE(e.isRunning());
    EXPECT_FALSE(e.addTask(TaskPhase::Diagnostics, [] { return true; }));
    e.stop();
    EXPECT_FALSE(e.isRunning());
    // Registration re-opens once stopped.
    EXPECT_TRUE(e.addTask(TaskPhase::Diagnostics, [] { return true; }));
}

TEST(CyclicExecutive, DoubleStartRejected) {
    Exec e(nullptr, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.start());
    EXPECT_FALSE(e.start());          // already running
    e.stop();
}

TEST(CyclicExecutive, StopWithoutStartIsSafe) {
    Exec e(nullptr, nullptr, nullptr, fastConfig());
    e.stop();                          // must not crash/wedge
    EXPECT_FALSE(e.isRunning());
}

// ============================================================================
// Cyclic loop mechanics
// ============================================================================

TEST(CyclicExecutive, ExchangeRunsEveryCycle) {
    std::atomic<int> calls{0};
    Exec e([&calls] { ++calls; return true; }, nullptr, nullptr,
           fastConfig());
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 5; }));
    e.stop();
    const auto s = e.getStats();
    EXPECT_GE(s.cycle_count, 5u);
    EXPECT_EQ(s.exchange_errors, 0u);
    EXPECT_EQ(s.task_errors, 0u);
    EXPECT_GT(s.jitter.cycle_count, 0u);
}

TEST(CyclicExecutive, PhaseOrderingPreExchPostMotionDiag) {
    std::mutex m;
    std::vector<int> order;
    auto push = [&](int tag) {
        return [&m, &order, tag] {
            std::lock_guard<std::mutex> g(m);
            order.push_back(tag);
            return true;
        };
    };
    Exec e(push(2), nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.addTask(TaskPhase::PreExchange,   push(1)));
    ASSERT_TRUE(e.addTask(TaskPhase::PostExchange,  push(3)));
    ASSERT_TRUE(e.addTask(TaskPhase::MotionControl, push(4)));
    ASSERT_TRUE(e.addTask(TaskPhase::Diagnostics,   push(5)));
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> g(m);
        return order.size() >= 10;
    }));
    e.stop();
    std::lock_guard<std::mutex> g(m);
    // First full cycle: 1,2,3,4,5 in order.
    ASSERT_GE(order.size(), 5u);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(order[i], static_cast<int>(i + 1)) << "phase " << i;
    }
    // And the order repeats for the second cycle.
    for (size_t i = 5; i < 10 && i < order.size(); ++i) {
        EXPECT_EQ(order[i], static_cast<int>(i - 5 + 1));
    }
}

TEST(CyclicExecutive, ExchangeDisableSkipsOnlyExchange) {
    std::atomic<int> exch{0}, pre{0};
    Exec e([&] { ++exch; return true; }, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.addTask(TaskPhase::PreExchange, [&] { ++pre; return true; }));
    e.setExchangeEnabled(false);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return pre.load() >= 4; }));
    e.stop();
    EXPECT_EQ(exch.load(), 0);
    EXPECT_GE(pre.load(), 4);
    e.setExchangeEnabled(true);           // toggle back for coverage
}

TEST(CyclicExecutive, ExchangeErrorCountedAndLoopContinues) {
    std::atomic<int> calls{0};
    Exec e([&] { ++calls; return false; }, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 3; }));
    e.stop();
    const auto s = e.getStats();
    EXPECT_GE(s.exchange_errors, 3u);
    EXPECT_GE(s.cycle_count, 3u);         // continue_on_error=true default
}

TEST(CyclicExecutive, TaskErrorCountedAndLoopContinues) {
    std::atomic<int> calls{0};
    Exec e(nullptr, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.addTask(TaskPhase::Diagnostics,
                          [&] { ++calls; return false; }));
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 3; }));
    e.stop();
    EXPECT_GE(e.getStats().task_errors, 3u);
}

TEST(CyclicExecutive, ContinueOnErrorFalseStopsLoop) {
    auto cfg = fastConfig();
    cfg.continue_on_error = false;
    std::atomic<int> calls{0};
    Exec e([&] { ++calls; return false; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    // First exchange failure breaks the loop — the thread self-exits.
    ASSERT_TRUE(waitFor([&] { return !e.isRunning(); }));
    e.stop();
    EXPECT_EQ(calls.load(), 1);
    EXPECT_EQ(e.getStats().exchange_errors, 1u);
}

TEST(CyclicExecutive, TaskErrorStopsLoopWhenNotContinuing) {
    auto cfg = fastConfig();
    cfg.continue_on_error = false;
    Exec e(nullptr, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.addTask(TaskPhase::MotionControl,
                          [] { return false; }));
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return !e.isRunning(); }));
    e.stop();
    EXPECT_EQ(e.getStats().task_errors, 1u);
}

// ============================================================================
// DC placement
// ============================================================================

TEST(CyclicExecutive, InlineDcRunsOnDecimatedCycles) {
    std::atomic<int> dc{0}, exch{0};
    auto cfg = fastConfig();
    cfg.dc_placement       = Exec::DCPlacement::Inline;
    cfg.dc_interval_cycles = 3;
    Exec e([&] { ++exch; return true; },
           [&] { ++dc;   return true; }, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return exch.load() >= 6; }));
    e.stop();
    const auto s = e.getStats();
    // Inline DC fires on cycles 3, 6, 9, ... — count ≈ cycles/3.
    EXPECT_GE(s.dc_sync_count, 1u);
    EXPECT_LE(s.dc_sync_count, s.cycle_count / 3 + 1);
    EXPECT_EQ(s.dc_sync_errors, 0u);
}

TEST(CyclicExecutive, InlineDcErrorCounted) {
    auto cfg = fastConfig();
    cfg.dc_placement       = Exec::DCPlacement::Inline;
    cfg.dc_interval_cycles = 1;
    std::atomic<int> cycles{0};
    Exec e([&] { ++cycles; return true; },
           [] { return false; }, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return cycles.load() >= 3; }));
    e.stop();
    EXPECT_GE(e.getStats().dc_sync_errors, 3u);
}

TEST(CyclicExecutive, DedicatedDcThreadRuns) {
    std::atomic<int> dc{0}, exch{0};
    auto cfg = fastConfig();
    cfg.dc_placement       = Exec::DCPlacement::DedicatedThread;
    cfg.dc_interval_cycles = 2;           // DC period = 2×cycle
    Exec e([&] { ++exch; return true; },
           [&] { ++dc;   return true; }, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return dc.load() >= 2; }));
    e.stop();
    const auto s = e.getStats();
    EXPECT_GE(s.dc_sync_count, 2u);
    EXPECT_GE(s.cycle_count, 4u);          // cyclic kept ticking in parallel
    EXPECT_EQ(s.dc_sync_errors, 0u);
    EXPECT_GT(s.dc_jitter.cycle_count, 0u);
}

// ============================================================================
// Sleep modes + scheduling classes — the RT knobs
// ============================================================================

TEST(CyclicExecutive, HybridSpinModeRuns) {
    std::atomic<int> calls{0};
    auto cfg = fastConfig();
    cfg.sleep_mode      = Exec::SleepMode::HybridSpin;
    cfg.spin_window_us  = 100;
    Exec e([&] { ++calls; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 4; }));
    e.stop();
    EXPECT_GE(e.getStats().cycle_count, 4u);
}

TEST(CyclicExecutive, DeadlineClassFallsBackAndRuns) {
    // SCHED_DEADLINE admission almost certainly fails unprivileged —
    // the code must log and fall back to SCHED_FIFO (which also fails →
    // normal scheduling).  Either way the loop must still tick.
    std::atomic<int> calls{0};
    auto cfg = fastConfig();
    cfg.sched_class     = Exec::SchedClass::Deadline;
    cfg.dl_runtime_ns   = 0;              // defaults: period/2, period
    cfg.dl_deadline_ns  = 0;
    Exec e([&] { ++calls; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 3; }));
    e.stop();
    EXPECT_GE(e.getStats().cycle_count, 3u);
}

TEST(CyclicExecutive, DeadlineClassExplicitParams) {
    std::atomic<int> calls{0};
    auto cfg = fastConfig();
    cfg.sched_class     = Exec::SchedClass::Deadline;
    cfg.dl_runtime_ns   = 100'000;        // 100 µs runtime / 500 µs period
    cfg.dl_deadline_ns  = 400'000;
    Exec e([&] { ++calls; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 2; }));
    e.stop();
}

TEST(CyclicExecutive, CpuAffinityAndSlackAndPrefaultPaths) {
    // Knobs that run at thread start regardless of privilege: affinity
    // (valid CPU), timer slack, stack prefault.
    std::atomic<int> calls{0};
    auto cfg = fastConfig();
    cfg.cpu_affinity          = 0;        // pin to CPU0 — always exists
    cfg.low_timer_slack       = true;
    cfg.stack_prefault_bytes  = 64 * 1024;
    Exec e([&] { ++calls; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 3; }));
    e.stop();
}

TEST(CyclicExecutive, NoSlackNoPrefaultPaths) {
    std::atomic<int> calls{0};
    auto cfg = fastConfig();
    cfg.low_timer_slack      = false;
    cfg.stack_prefault_bytes = 0;
    Exec e([&] { ++calls; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 2; }));
    e.stop();
}

// ============================================================================
// Jitter / stats
// ============================================================================

TEST(CyclicExecutive, StatsAccumulateMaxCycleWork) {
    std::atomic<int> calls{0};
    Exec e([&] {
        ++calls;
        std::this_thread::sleep_for(150us);   // measurable in-cycle work
        return true;
    }, nullptr, nullptr, fastConfig());
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 2; }));
    e.stop();
    const auto s = e.getStats();
    EXPECT_GT(s.max_cycle_work_us, 0u);
}

TEST(CyclicExecutive, TimeSourceOverrideUsedForJitter) {
    std::atomic<uint64_t> fake_now{0};
    std::atomic<int> calls{0};
    Exec e([&] { ++calls; return true; }, nullptr,
           [&] { return fake_now.load(); }, fastConfig());
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return calls.load() >= 2; }));
    e.stop();
    // A frozen time source still lets the loop run (jitter just looks odd).
    EXPECT_GE(e.getStats().cycle_count, 2u);
}

TEST(CyclicExecutive, ZeroDcIntervalNormalizedToOne) {
    auto cfg = fastConfig();
    cfg.dc_placement       = Exec::DCPlacement::Inline;
    cfg.dc_interval_cycles = 0;           // ctor clamps to 1
    std::atomic<int> dc{0};
    Exec e(nullptr, [&] { ++dc; return true; }, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return dc.load() >= 2; }));
    e.stop();
    EXPECT_GE(e.getStats().dc_sync_count, 2u);
}

TEST(CyclicExecutive, DestructorStopsRunningLoop) {
    std::atomic<int> calls{0};
    {
        Exec e([&] { ++calls; return true; }, nullptr, nullptr, fastConfig());
        ASSERT_TRUE(e.start());
        ASSERT_TRUE(waitFor([&] { return calls.load() >= 2; }));
        // dtor → stop() → join — no hang, no leak
    }
    SUCCEED();
}

} // namespace
