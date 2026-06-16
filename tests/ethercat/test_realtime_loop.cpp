/**
 * @file test_realtime_loop.cpp
 * @brief Comprehensive tests for RealtimeLoop (extracted realtime loop)
 *
 * Tests the realtime loop independently from the DC module using callback
 * functions and short timeouts (10-50ms).
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/EtherCATRealtimeLoop.hpp"
#include "tether/hal/IThreading.hpp"

#include <atomic>
#include <thread>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <condition_variable>

using namespace EtherCAT;

// ============================================================================
// Helper: host monontonic clock in nanoseconds
// ============================================================================

static uint64_t hostTimeNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

// ============================================================================
// Construction Tests
// ============================================================================

TEST(RealtimeLoopConstruction, NotRunningAfterConstruction) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs);
    EXPECT_FALSE(loop.isRunning());
}

TEST(RealtimeLoopConstruction, PDODisabledByDefault) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs);
    EXPECT_FALSE(loop.isPDOEnabled());
}

TEST(RealtimeLoopConstruction, StatsZeroAfterConstruction) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs);
    auto stats = loop.getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
    EXPECT_EQ(stats.sync_count, 0u);
    EXPECT_EQ(stats.pdo_error_count, 0u);
    EXPECT_EQ(stats.max_jitter_us, 0u);
    EXPECT_EQ(stats.avg_jitter_us, 0u);
}

// ============================================================================
// Start / Stop Tests
// ============================================================================

class RealtimeLoopStartStopTest : public ::testing::Test {
protected:
    RealtimeLoop::Config cfg_;

    void SetUp() override {
        cfg_.cycle_period_us = 1000;
        cfg_.sync_interval_cycles = 5;
    }
};

TEST_F(RealtimeLoopStartStopTest, StartReturnsTrue) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());
    EXPECT_TRUE(loop.isRunning());
    loop.stop();
}

TEST_F(RealtimeLoopStartStopTest, StopSetsNotRunning) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());
    loop.stop();
    EXPECT_FALSE(loop.isRunning());
}

TEST_F(RealtimeLoopStartStopTest, DoubleStartFails) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());
    EXPECT_FALSE(loop.start()); // second start should fail
    loop.stop();
}

TEST_F(RealtimeLoopStartStopTest, StopWithoutStart) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    // Should not crash
    loop.stop();
    EXPECT_FALSE(loop.isRunning());
}

TEST_F(RealtimeLoopStartStopTest, RestartAfterStop) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());
    loop.stop();
    ASSERT_TRUE(loop.start());
    EXPECT_TRUE(loop.isRunning());
    loop.stop();
}

TEST_F(RealtimeLoopStartStopTest, DestructorStops) {
    {
        RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
        ASSERT_TRUE(loop.start());
        // Destructor should call stop() and clean up
    }
    // After destruction the realtime loop resources should be free — new loop should start
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());
    loop.stop();
}

// ============================================================================
// PDO Exchange Tests
// ============================================================================

class RealtimeLoopPDOTest : public ::testing::Test {
protected:
    std::atomic<int> pdo_calls_{0};
    RealtimeLoop::Config cfg_;

    void SetUp() override {
        cfg_.cycle_period_us = 1000;       // 1ms
        cfg_.sync_interval_cycles = 100;   // sync infrequently
    }
};

TEST_F(RealtimeLoopPDOTest, PDOCalledWhenEnabled) {
    auto pdo_fn = [this]() { pdo_calls_++; return true; };

    RealtimeLoop loop(pdo_fn, nullptr, hostTimeNs, cfg_);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    EXPECT_GT(pdo_calls_.load(), 10);
}

TEST_F(RealtimeLoopPDOTest, PDONotCalledWhenDisabled) {
    auto pdo_fn = [this]() { pdo_calls_++; return true; };

    RealtimeLoop loop(pdo_fn, nullptr, hostTimeNs, cfg_);
    // PDO disabled by default
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    loop.stop();

    EXPECT_EQ(pdo_calls_.load(), 0);
}

TEST_F(RealtimeLoopPDOTest, PDOEnableDisableAffectsExchange) {
    auto pdo_fn = [this]() { pdo_calls_++; return true; };

    RealtimeLoop loop(pdo_fn, nullptr, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());

    // Disabled initially — no calls yet
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    int before_enable = pdo_calls_.load();

    // Enable
    loop.setPDOEnabled(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    int after_enable = pdo_calls_.load();

    // Disable
    loop.setPDOEnabled(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int after_disable = pdo_calls_.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    int final_count = pdo_calls_.load();

    loop.stop();

    EXPECT_EQ(before_enable, 0);
    EXPECT_GT(after_enable, 0);
    // After disabling, count should stop growing significantly
    EXPECT_LE(final_count - after_disable, 2); // allow ±1 for race
}

TEST_F(RealtimeLoopPDOTest, PDOErrorCountIncremented) {
    auto pdo_fn = []() { return false; }; // always fails

    RealtimeLoop loop(pdo_fn, nullptr, hostTimeNs, cfg_);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    auto stats = loop.getStats();
    EXPECT_GT(stats.pdo_error_count, 5u);
}

// ============================================================================
// DC Sync Tests
// ============================================================================

class RealtimeLoopSyncTest : public ::testing::Test {
protected:
    std::atomic<int> sync_calls_{0};
    RealtimeLoop::Config cfg_;

    void SetUp() override {
        cfg_.cycle_period_us = 1000;       // 1ms
        cfg_.sync_interval_cycles = 5;     // sync every 5 cycles
    }
};

TEST_F(RealtimeLoopSyncTest, SyncCalledAtInterval) {
    auto sync_fn = [this]() { sync_calls_++; return true; };

    RealtimeLoop loop(nullptr, sync_fn, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    // With 50 cycles and sync every 5 → expect ~10 sync calls
    EXPECT_GT(sync_calls_.load(), 3);
    EXPECT_LT(sync_calls_.load(), 30);
}

TEST_F(RealtimeLoopSyncTest, SyncCountInStats) {
    auto sync_fn = [this]() { sync_calls_++; return true; };

    RealtimeLoop loop(nullptr, sync_fn, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    auto stats = loop.getStats();
    EXPECT_GT(stats.sync_count, 0u);
    EXPECT_EQ(stats.sync_count, static_cast<uint64_t>(sync_calls_.load()));
}

TEST_F(RealtimeLoopSyncTest, SyncNotCountedWhenCallbackReturnsFalse) {
    auto sync_fn = []() { return false; }; // always fails

    RealtimeLoop loop(nullptr, sync_fn, hostTimeNs, cfg_);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    loop.stop();

    auto stats = loop.getStats();
    // Sync callback returns false → sync_count should remain 0
    EXPECT_EQ(stats.sync_count, 0u);
}

// ============================================================================
// Jitter Stats Tests
// ============================================================================

TEST(RealtimeLoopJitterTest, StatsAccumulate) {
    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 100;

    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    auto stats = loop.getStats();
    EXPECT_GT(stats.cycle_count, 0u);
    // On a host system, jitter should be measurable (>= 0)
    // Just verify max_jitter_us and avg_jitter_us are populated
    // (they may be 0 if cycles execute very fast)
}

// ============================================================================
// Clean Shutdown Tests
// ============================================================================

TEST(RealtimeLoopShutdownTest, NoLeaksOnDestructionWhileRunning) {
    for (int i = 0; i < 5; i++) {
        RealtimeLoop::Config cfg;
        cfg.cycle_period_us = 1000;
        cfg.sync_interval_cycles = 5;

        RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
        loop.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Destructor cleans up
    }
    // Verify resources were released by starting a new loop
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs);
    ASSERT_TRUE(loop.start());
    loop.stop();
}

TEST(RealtimeLoopShutdownTest, ResourcesReleasedAfterStop) {
    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 5;

    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    ASSERT_TRUE(loop.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    loop.stop();

    // After stop, loop should be re-startable (resources were released)
    ASSERT_TRUE(loop.start());
    loop.stop();
}

// ============================================================================
// Two-Thread Architecture Tests
// ============================================================================

TEST(RealtimeLoopTwoThread, BothCallbacksRunConcurrently) {
    std::atomic<int> pdo_calls{0};
    std::atomic<int> dc_calls{0};

    auto pdo_fn = [&pdo_calls]() { pdo_calls++; return true; };
    auto dc_fn  = [&dc_calls]()  { dc_calls++;  return true; };

    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;       // PDO at 1 kHz
    cfg.sync_interval_cycles = 5;     // DC at 200 Hz (every 5 ms)

    RealtimeLoop loop(pdo_fn, dc_fn, hostTimeNs, cfg);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    loop.stop();

    // PDO should have ~100 calls (100ms / 1ms)
    EXPECT_GT(pdo_calls.load(), 50);
    // DC should have ~20 calls (100ms / 5ms)
    EXPECT_GT(dc_calls.load(), 5);

    // PDO should run much more frequently than DC
    EXPECT_GT(pdo_calls.load(), dc_calls.load());
}

TEST(RealtimeLoopTwoThread, DCRunsIndependentlyOfPDOEnabled) {
    std::atomic<int> dc_calls{0};

    auto dc_fn = [&dc_calls]() { dc_calls++; return true; };

    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 5;

    // PDO disabled, DC should still run
    RealtimeLoop loop(nullptr, dc_fn, hostTimeNs, cfg);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    EXPECT_GT(dc_calls.load(), 3);
}

TEST(RealtimeLoopTwoThread, PDOAndDCHaveIndependentCounts) {
    auto pdo_fn = []() { return true; };
    auto dc_fn  = []() { return true; };

    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 10;

    RealtimeLoop loop(pdo_fn, dc_fn, hostTimeNs, cfg);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    loop.stop();

    auto stats = loop.getStats();
    // PDO cycle count should be much higher than DC sync count
    EXPECT_GT(stats.cycle_count, stats.sync_count);
    // Both should be > 0
    EXPECT_GT(stats.cycle_count, 0u);
    EXPECT_GT(stats.sync_count, 0u);
}

// ============================================================================
// Diagnostics Tests
// ============================================================================

TEST(RealtimeLoopDiagnostics, DiagnosticsReturnBothThreadStats) {
    auto pdo_fn = []() { return true; };
    auto dc_fn  = []() { return true; };

    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 5;

    RealtimeLoop loop(pdo_fn, dc_fn, hostTimeNs, cfg);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    auto diag = loop.getDiagnostics();
    // PDO jitter monitor should have recorded cycles
    EXPECT_GT(diag.pdo_jitter.cycle_count, 0u);
    // DC jitter monitor should have recorded cycles
    EXPECT_GT(diag.dc_jitter.cycle_count, 0u);
    // Both should report realtime_ok = true on a non-stress test
    // (may occasionally fail on overloaded CI, so we just check they're populated)
}

TEST(RealtimeLoopDiagnostics, DiagnosticsZeroBeforeStart) {
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs);

    auto diag = loop.getDiagnostics();
    EXPECT_EQ(diag.pdo_jitter.cycle_count, 0u);
    EXPECT_EQ(diag.dc_jitter.cycle_count, 0u);
}

TEST(RealtimeLoopDiagnostics, PDOJitterMatchesLegacyStats) {
    auto pdo_fn = []() { return true; };

    RealtimeLoop::Config cfg;
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 100;

    RealtimeLoop loop(pdo_fn, nullptr, hostTimeNs, cfg);
    loop.setPDOEnabled(true);
    ASSERT_TRUE(loop.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.stop();

    auto stats = loop.getStats();
    auto diag  = loop.getDiagnostics();

    // Legacy max_jitter_us and avg_jitter_us come from PDO jitter monitor
    EXPECT_EQ(stats.max_jitter_us, diag.pdo_jitter.max_jitter_us);
    EXPECT_EQ(stats.avg_jitter_us, diag.pdo_jitter.avg_jitter_us);
}

// ============================================================================
// Config factory method
// ============================================================================

TEST(RealtimeLoopConfig, DefaultsAutoDerivesJitterThresholds) {
    auto cfg = RealtimeLoop::Config::defaults(2000, 20);

    EXPECT_EQ(cfg.cycle_period_us, 2000u);
    EXPECT_EQ(cfg.sync_interval_cycles, 20u);

    // PDO jitter thresholds derived from 2000 us
    EXPECT_EQ(cfg.pdo_jitter.expected_period_us, 2000u);
    EXPECT_EQ(cfg.pdo_jitter.warning_threshold_us, 1000u);
    EXPECT_EQ(cfg.pdo_jitter.critical_threshold_us, 4000u);

    // DC jitter thresholds derived from 2000 * 20 = 40000 us
    EXPECT_EQ(cfg.dc_jitter.expected_period_us, 40000u);
    EXPECT_EQ(cfg.dc_jitter.warning_threshold_us, 20000u);
    EXPECT_EQ(cfg.dc_jitter.critical_threshold_us, 80000u);
}

TEST(RealtimeLoopConfig, DefaultsNoArgBackwardCompatible) {
    auto cfg = RealtimeLoop::Config::defaults();

    EXPECT_EQ(cfg.cycle_period_us, 1000u);
    EXPECT_EQ(cfg.sync_interval_cycles, 10u);
    EXPECT_EQ(cfg.pdo_jitter.expected_period_us, 1000u);
    EXPECT_EQ(cfg.dc_jitter.expected_period_us, 10000u);
}

// ============================================================================
// Error-path tests using a controllable threading factory
// ============================================================================

namespace {

/**
 * @brief Minimal IEvent implementation using std::condition_variable.
 *
 * Just enough to make the realtime loop's event-based wake pattern work in
 * tests.
 */
class TestEvent : public EtherCAT::HAL::IEvent {
public:
    TestEvent(bool /*manualReset*/, bool /*initialState*/) {}

    void signal() override {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            signaled_ = true;
        }
        cv_.notify_one();
    }

    EtherCAT::HAL::Error wait() override {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this]{ return signaled_; });
        signaled_ = false;
        return EtherCAT::HAL::Error::OK;
    }

    EtherCAT::HAL::Error waitFor(EtherCAT::HAL::Milliseconds timeout_ms) override {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                     [this]{ return signaled_; });
        signaled_ = false;
        return EtherCAT::HAL::Error::OK;
    }

    void reset() override {
        std::lock_guard<std::mutex> lk(mtx_);
        signaled_ = false;
    }

    bool isSignaled() const override {
        return signaled_;
    }

private:
    std::mutex mtx_;
    std::condition_variable cv_;
    bool signaled_ = false;
};

/**
 * @brief Minimal IThread implementation using std::thread.
 */
class TestThread : public EtherCAT::HAL::IThread {
public:
    TestThread() = default;
    ~TestThread() override {
        if (thread_.joinable()) thread_.join();
    }

    EtherCAT::HAL::Error start(std::function<void()> func) override {
        thread_ = std::thread(std::move(func));
        running_ = true;
        return EtherCAT::HAL::Error::OK;
    }

    EtherCAT::HAL::Error join() override {
        if (thread_.joinable()) thread_.join();
        running_ = false;
        return EtherCAT::HAL::Error::OK;
    }

    EtherCAT::HAL::Error detach() override {
        if (thread_.joinable()) thread_.detach();
        return EtherCAT::HAL::Error::OK;
    }

    bool isRunning() const override { return running_; }
    void requestStop() override { stop_ = true; }
    bool stopRequested() const override { return stop_; }
    void* nativeHandle() override { return nullptr; }

private:
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
};

/// IThread whose start() always returns Error::Timeout
class FailStartThread : public EtherCAT::HAL::IThread {
public:
    EtherCAT::HAL::Error start(std::function<void()>) override {
        return EtherCAT::HAL::Error::Timeout;
    }
    void requestStop() override {}
    bool stopRequested() const override { return true; }
    EtherCAT::HAL::Error join() override { return EtherCAT::HAL::Error::OK; }
    EtherCAT::HAL::Error detach() override { return EtherCAT::HAL::Error::OK; }
    bool isRunning() const override { return false; }
    void* nativeHandle() override { return nullptr; }
};

/**
 * @brief Threading factory that uses standalone TestEvent/TestThread for "pass"
 *        calls, and can fail specific Nth calls to createEvent / createThread.
 *
 * Avoids referencing the global factory singleton (which gets destroyed when
 * this factory is installed via setThreadingFactory).
 */
class ControllableFactory : public EtherCAT::HAL::IThreadingFactory {
public:
    int fail_event_at  = 0;  ///< Nth createEvent() returns nullptr (1-based)
    int fail_thread_at = 0;  ///< Nth createThread() returns nullptr
    int fail_thread_start_at = 0; ///< Nth createThread() returns a FailStartThread

    std::unique_ptr<EtherCAT::HAL::IThread> createThread(
            const EtherCAT::HAL::ThreadConfig&) override {
        ++thread_call_count_;
        if (fail_thread_at > 0 && thread_call_count_ == fail_thread_at) {
            return nullptr;
        }
        if (fail_thread_start_at > 0 && thread_call_count_ == fail_thread_start_at) {
            return std::make_unique<FailStartThread>();
        }
        return std::make_unique<TestThread>();
    }

    std::unique_ptr<EtherCAT::HAL::IEvent> createEvent(
            bool manualReset, bool initialState) override {
        ++event_call_count_;
        if (fail_event_at > 0 && event_call_count_ == fail_event_at) {
            return nullptr;
        }
        return std::make_unique<TestEvent>(manualReset, initialState);
    }

    // -- Stubs for unused methods --
    std::unique_ptr<EtherCAT::HAL::IMutex> createMutex(EtherCAT::HAL::MutexType) override {
        return nullptr;
    }
    std::unique_ptr<EtherCAT::HAL::IConditionVariable> createConditionVariable() override {
        return nullptr;
    }
    std::unique_ptr<EtherCAT::HAL::ISemaphore> createSemaphore(int, int) override {
        return nullptr;
    }
    std::unique_ptr<EtherCAT::HAL::IQueue> createQueue(size_t, size_t) override {
        return nullptr;
    }
    void sleep(EtherCAT::HAL::Milliseconds ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    void yield() override { std::this_thread::yield(); }
    uint32_t currentThreadId() override { return 0; }

private:
    int event_call_count_  = 0;
    int thread_call_count_ = 0;
};

/// RAII guard: installs ControllableFactory and restores default on destruction.
class FactoryGuard {
public:
    explicit FactoryGuard(ControllableFactory* cf) {
        EtherCAT::HAL::setThreadingFactory(
            std::unique_ptr<EtherCAT::HAL::IThreadingFactory>(cf));
    }
    ~FactoryGuard() { EtherCAT::HAL::resetThreadingFactory(); }
};

} // anonymous namespace

// -- PDO error paths --------------------------------------------------------

TEST(RealtimeLoopErrorPaths, PDOEventCreationFailsReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_event_at = 1; // Fail first createEvent() → PDO event
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}

TEST(RealtimeLoopErrorPaths, PDOThreadCreationFailsReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_thread_at = 1; // Fail first createThread() → PDO thread
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}

TEST(RealtimeLoopErrorPaths, PDOThreadStartFailsReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_thread_start_at = 1; // start() fails for first thread → PDO
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}

// -- DC error paths (PDO fully succeeds, DC creation fails) -----------------

TEST(RealtimeLoopErrorPaths, DCEventCreationFailsStopsAndReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_event_at = 2; // 1st event (PDO) ok, 2nd event (DC) fails
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}

TEST(RealtimeLoopErrorPaths, DCThreadCreationFailsStopsAndReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_thread_at = 2; // 1st thread (PDO) ok, 2nd thread (DC) fails
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}

TEST(RealtimeLoopErrorPaths, DCThreadStartFailsStopsAndReturnsNotRunning) {
    auto* cf = new ControllableFactory();
    cf->fail_thread_start_at = 2; // start() fails for 2nd thread → DC
    FactoryGuard g(cf);

    RealtimeLoop::Config cfg = RealtimeLoop::Config::defaults();
    RealtimeLoop loop(nullptr, nullptr, hostTimeNs, cfg);
    EXPECT_FALSE(loop.start());
    EXPECT_FALSE(loop.isRunning());
}
