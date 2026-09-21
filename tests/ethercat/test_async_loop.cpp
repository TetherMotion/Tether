/**
 * @file test_async_loop.cpp
 * @brief Async send-on-change loop + the ProcessImage send trigger.
 *
 * Coverage:
 *  - ProcessImage::triggerSend/sendSeq/waitSend — in-process atomic wait,
 *    timed waits, and the shm cross-boundary wake (attached producer wakes
 *    the exporting image's waiter).
 *  - AsyncCyclicLoop — OnSend and Periodic collect modes, the send rate
 *    limiter (coalescing + trailing send), idle keep-alive collects, error
 *    accounting, stop-from-unbounded-wait, stats.
 *  - Master::startAsyncLoop — lifecycle, stats plumbing, and mutual
 *    exclusion with the cyclic loop.
 *
 * The loop runs a REAL thread (HAL factory) against a REAL ProcessImage —
 * no mocking of the wait path.  Scheduling degradation (no CAP_SYS_NICE)
 * is the normal-test-environment path: the loop still runs correctly.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#endif

#include "tether/ethercat/AsyncCyclicLoop.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "logging/Logger.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

template <typename F>
bool waitFor(F&& pred, std::chrono::milliseconds budget = 3'000ms) {
    const auto end = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(100us);
    }
    return pred();
}

uint64_t monoNs() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull + ts.tv_nsec;
}

std::unique_ptr<ProcessImage> makeImage() {
    auto img = std::make_unique<ProcessImage>();
    ProcessImage::Config c;
    c.mode = ImageMode::Direct;
    c.rx_bytes = 8; c.tx_bytes = 8;
    EXPECT_TRUE(img->configure(c));
    return img;
}

// ============================================================================
// ProcessImage send trigger
// ============================================================================

TEST(ProcessImageSendTrigger, TriggerBumpsSendSeq) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const uint32_t s0 = img.sendSeq();
    img.triggerSend();
    EXPECT_EQ(img.sendSeq(), s0 + 1);
    img.triggerSend();
    EXPECT_EQ(img.sendSeq(), s0 + 2);
}

TEST(ProcessImageSendTrigger, WaitSendReturnsImmediatelyWhenChanged) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const uint32_t s0 = img.sendSeq();
    img.triggerSend();
    // Zero timeout — the seq already differs, so no blocking.
    EXPECT_TRUE(img.waitSend(s0, 0));
}

TEST(ProcessImageSendTrigger, WaitSendTimesOut) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const uint32_t s0 = img.sendSeq();
    const uint64_t t0 = monoNs();
    // 20 ms relative timeout.
    EXPECT_FALSE(img.waitSend(s0, 20'000'000ull));
    EXPECT_GE(monoNs() - t0, 15'000'000ull);   // actually waited
}

TEST(ProcessImageSendTrigger, WaitSendWakesOnTrigger) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const uint32_t s0 = img.sendSeq();
    std::atomic<bool> woke{false};
    std::thread w([&] {
        // Unbounded wait — triggerSend must wake it.
        woke = img.waitSend(s0, UINT64_MAX);
    });
    // Give the waiter time to register before triggering.
    std::this_thread::sleep_for(10ms);
    img.triggerSend();
    w.join();
    EXPECT_TRUE(woke);
}

TEST(ProcessImageSendTrigger, WaitSendWakesOnTimedTrigger) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const uint32_t s0 = img.sendSeq();
    std::atomic<bool> woke{false};
    std::thread w([&] {
        woke = img.waitSend(s0, 2'000'000'000ull);
    });
    std::this_thread::sleep_for(10ms);
    img.triggerSend();
    w.join();
    EXPECT_TRUE(woke);
}

#ifdef __linux__
TEST(ProcessImageSendTrigger, ShmAttachedProducerWakesExporter) {
    // The cross-process path: an shm-attached producer's triggerSend()
    // bumps the shared send_seq and wakes the exporting image's waitSend()
    // via the shared futex word.
    const char* name = "tether-test-async-img";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.shm_name = name;

    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));
    ASSERT_TRUE(server.shmBacked());

    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    ASSERT_TRUE(client.shmBacked());

    const uint32_t s0 = server.sendSeq();
    EXPECT_EQ(s0, client.sendSeq());           // same shm word

    std::atomic<bool> woke{false};
    std::thread w([&] {
        woke = server.waitSend(s0, 2'000'000'000ull);
    });
    std::this_thread::sleep_for(10ms);
    client.triggerSend();                      // producer side (attached)
    w.join();
    EXPECT_TRUE(woke);
    EXPECT_EQ(server.sendSeq(), s0 + 1);       // visible on the exporter
}

TEST(ProcessImageSendTrigger, ShmTriggerVisibleWithoutWaiter) {
    // triggerSend with no registered waiter must still bump the shared
    // counter (the wake syscall is gated, the bump is not).
    const char* name = "tether-test-async-img2";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.shm_name = name;

    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));
    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));

    const uint32_t s0 = server.sendSeq();
    client.triggerSend();                      // nobody waiting
    EXPECT_EQ(server.sendSeq(), s0 + 1);
    // The exporter sees it on the next (immediate) waitSend.
    EXPECT_TRUE(server.waitSend(s0, 0));
}
#endif

// ============================================================================
// AsyncCyclicLoop — behavior
// ============================================================================

AsyncCyclicLoop::Config loopCfg(AsyncCyclicLoop::CollectMode m =
                                    AsyncCyclicLoop::CollectMode::OnSend) {
    AsyncCyclicLoop::Config c;
    c.collect_mode         = m;
    c.collect_period_us    = 500;        // 2 kHz collect tick when Periodic
    c.priority             = 10;         // may degrade — correctness holds
    c.stack_prefault_bytes = 16 * 1024;
    return c;
}

TEST(AsyncCyclicLoop, OnSendSendsAndCollectsPerTrigger) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0}, collects{0};
    AsyncCyclicLoop loop(img,
        [&] { ++sends; return true; },
        [&] { ++collects; return true; },
        nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.isRunning());

    // Fire each trigger only after the previous send landed — otherwise
    // back-to-back triggers coalesce into one send carrying the latest
    // image (send-latest: the desired RT behavior, covered separately).
    for (int i = 1; i <= 3; ++i) {
        img.triggerSend();
        ASSERT_TRUE(waitFor([&] { return sends.load() >= i; }));
    }
    // OnSend: every send is paired with a collect.
    EXPECT_TRUE(waitFor([&] { return collects.load() >= 3; }));

    const auto st = loop.getStats();
    EXPECT_GE(st.wakes, 3u);
    EXPECT_EQ(st.sends, 3u);
    EXPECT_EQ(st.sends_coalesced, 0u);
    EXPECT_EQ(st.send_errors, 0u);
    EXPECT_EQ(st.collect_errors, 0u);
    loop.stop();
    EXPECT_FALSE(loop.isRunning());
}

TEST(AsyncCyclicLoop, OnSendIsSilentWithoutTriggers) {
    // No trigger → no send: proves the loop is event-driven, not
    // free-running.  Give it real time to (not) fire.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0};
    AsyncCyclicLoop loop(img, [&] { ++sends; return true; }, nullptr,
                         nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(sends.load(), 0);
    loop.stop();
}

TEST(AsyncCyclicLoop, PeriodicCollectsWithoutTriggers) {
    // Periodic: TxPDO ticks on its own clock even with zero sends.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0}, collects{0};
    auto c = loopCfg(AsyncCyclicLoop::CollectMode::Periodic);
    c.collect_period_us = 200;                       // 5 kHz tick
    AsyncCyclicLoop loop(img,
        [&] { ++sends; return true; },
        [&] { ++collects; return true; },
        nullptr, nullptr, c);
    ASSERT_TRUE(loop.start());
    // Collects accumulate on the tick with no triggers at all.
    ASSERT_TRUE(waitFor([&] { return collects.load() >= 3; }));
    EXPECT_EQ(sends.load(), 0);

    // A trigger still drives a send — independent of the collect clock.
    img.triggerSend();
    EXPECT_TRUE(waitFor([&] { return sends.load() >= 1; }));
    const auto st = loop.getStats();
    EXPECT_GE(st.collects, 3u);
    EXPECT_EQ(st.idle_collects, 0u);   // idle is an OnSend-only concept
    loop.stop();
}

TEST(AsyncCyclicLoop, RateLimiterCoalescesBursts) {
    // min_send_interval merges a trigger burst into fewer sends, and the
    // trailing send still goes out carrying the latest image.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0};
    auto c = loopCfg();
    c.min_send_interval_ns = 30'000'000;             // 30 ms window
    AsyncCyclicLoop loop(img, [&] { ++sends; return true; }, nullptr,
                         nullptr, nullptr, c);
    ASSERT_TRUE(loop.start());

    img.triggerSend();                               // send #1 fires now
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 1; }));
    // Rapid burst inside the window → coalesced to one trailing send.
    img.triggerSend();
    img.triggerSend();
    img.triggerSend();
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 2; }));  // trailing
    const auto st = loop.getStats();
    EXPECT_GE(st.sends_coalesced, 1u);
    EXPECT_LT(sends.load(), 5);                      // merged, not per-trigger
    loop.stop();
}

TEST(AsyncCyclicLoop, BackToBackTriggersCoalesceToLatest) {
    // Two triggers landing inside one wait window merge into a single
    // send carrying the latest image — counted as coalesced.  Firing
    // them from inside the send callback pins the ordering: they land
    // while the loop's 'last' snapshot is still the pre-burst seq.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0};
    std::atomic<bool> burst{true};
    AsyncCyclicLoop loop(img,
        [&] {
            if (burst.exchange(false)) {
                img.triggerSend();
                img.triggerSend();
            }
            ++sends; return true;
        },
        nullptr, nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());

    img.triggerSend();                   // wake 1 → send_ fires 2 more
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 2; }, 2'000ms));
    const auto st = loop.getStats();
    EXPECT_EQ(st.sends, 2u);             // 3 triggers → 2 sends
    EXPECT_GE(st.sends_coalesced, 1u);   // the merged pair counted
    loop.stop();
}

TEST(AsyncCyclicLoop, IdleKeepAliveCollectsWhenSilent) {
    // OnSend + max_idle: with no triggers the loop still collects to keep
    // the wire alive.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> collects{0};
    auto c = loopCfg();
    c.max_idle_ns = 20'000'000;                      // 20 ms keep-alive
    AsyncCyclicLoop loop(img, [&] { return true; },
                         [&] { ++collects; return true; },
                         nullptr, nullptr, c);
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(waitFor([&] { return collects.load() >= 2; }, 2'000ms));
    const auto st = loop.getStats();
    EXPECT_GE(st.idle_collects, 2u);
    loop.stop();
}

TEST(AsyncCyclicLoop, ActivityResetsIdleDeadline) {
    // Triggered sends push the idle deadline out — no keep-alive collects
    // while the producer is active; idle collects resume once it stops.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0}, collects{0};
    auto c = loopCfg();
    c.max_idle_ns = 100'000'000;                     // 100 ms
    AsyncCyclicLoop loop(img,
        [&] { ++sends; return true; },
        [&] { ++collects; return true; },
        nullptr, nullptr, c);
    ASSERT_TRUE(loop.start());
    // A producer thread triggers every ~5 ms — 20× below the idle
    // deadline, so even a starved loop thread keeps resetting it.
    std::atomic<bool> run{true};
    std::thread prod([&] {
        while (run.load()) {
            img.triggerSend();
            std::this_thread::sleep_for(5ms);
        }
    });
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 6; }, 5'000ms));
    EXPECT_EQ(loop.getStats().idle_collects, 0u);
    run = false;
    prod.join();
    // Quiesce → the keep-alive collect must fire (~idle period).
    ASSERT_TRUE(waitFor([&] {
        return loop.getStats().idle_collects >= 1; }, 5'000ms));
    loop.stop();
}

TEST(AsyncCyclicLoop, SendErrorCountedAndLoopContinues) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0};
    AsyncCyclicLoop loop(img,
        [&] { ++sends; return false; },              // always fail
        nullptr, nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    // Space the triggers so each is a distinct wake — back-to-back ones
    // would coalesce into a single send (and a single error).
    for (int i = 1; i <= 2; ++i) {
        img.triggerSend();
        ASSERT_TRUE(waitFor([&] { return sends.load() >= i; }));
    }
    const auto st = loop.getStats();
    EXPECT_EQ(st.sends, 0u);
    EXPECT_EQ(st.send_errors, 2u);
    EXPECT_TRUE(loop.isRunning());                   // continue_on_error
    loop.stop();
}

TEST(AsyncCyclicLoop, StopFromUnboundedWait) {
    // OnSend + no idle → the loop sits in the unbounded atomic wait;
    // stop() must break it cleanly.
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    AsyncCyclicLoop loop(img, [&] { return true; }, nullptr,
                         nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    std::this_thread::sleep_for(10ms);               // let it reach the wait
    const auto t0 = std::chrono::steady_clock::now();
    loop.stop();                                     // must not hang
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 2s);
    EXPECT_FALSE(loop.isRunning());
}

TEST(AsyncCyclicLoop, DoubleStartRejected) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    AsyncCyclicLoop loop(img, [&] { return true; }, nullptr,
                         nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    EXPECT_FALSE(loop.start());
    loop.stop();
}

// ============================================================================
// Master wiring
// ============================================================================

TEST(MasterAsyncLoop, StartStopLifecycleAndStats) {
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    Master::AsyncLoopConfig cfg;
    cfg.collect_mode = AsyncCyclicLoop::CollectMode::OnSend;
    ASSERT_TRUE(master.startAsyncLoop(cfg));
    EXPECT_TRUE(master.isAsyncLoopRunning());

    // Producer triggers on the image → the loop counts a send.
    master.processImage().triggerSend();
    ASSERT_TRUE(waitFor([&] {
        return master.getAsyncLoopStats().sends >= 1;
    }));

    master.stopAsyncLoop();
    EXPECT_FALSE(master.isAsyncLoopRunning());
    master.stop();
}

TEST(MasterAsyncLoop, CyclicAndAsyncAreMutuallyExclusive) {
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    ASSERT_TRUE(master.startAsyncLoop(Master::AsyncLoopConfig{}));
    EXPECT_TRUE(master.isAsyncLoopRunning());
    // Starting the cyclic loop must stop the async loop.
    ASSERT_TRUE(master.startCyclicLoop(Master::CyclicLoopConfig{}));
    EXPECT_TRUE(master.isCyclicLoopRunning());
    EXPECT_FALSE(master.isAsyncLoopRunning());
    // And vice versa.
    ASSERT_TRUE(master.startAsyncLoop(Master::AsyncLoopConfig{}));
    EXPECT_TRUE(master.isAsyncLoopRunning());
    EXPECT_FALSE(master.isCyclicLoopRunning());
    master.stop();
}

TEST(MasterAsyncLoop, StopAlsoStopsAsyncLoop) {
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);
    ASSERT_TRUE(master.startAsyncLoop(Master::AsyncLoopConfig{}));
    EXPECT_TRUE(master.isAsyncLoopRunning());
    master.stop();
    EXPECT_FALSE(master.isAsyncLoopRunning());
}

TEST(MasterAsyncLoop, RestartWhileAsyncBlockedInWait) {
    // Regression: a running async loop sits inside waitSend() on the
    // process image's send word.  Re-starting must kill that thread
    // *before* the shared datapath teardown reconfigures the image —
    // otherwise the waiter reads reconfigured/unmapped state.
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    Master::AsyncLoopConfig cfg;
    cfg.collect_mode = AsyncCyclicLoop::CollectMode::OnSend;
    ASSERT_TRUE(master.startAsyncLoop(cfg));
    std::this_thread::sleep_for(5ms);    // let it reach waitSend

    ASSERT_TRUE(master.startAsyncLoop(cfg));   // restart under it
    EXPECT_TRUE(master.isAsyncLoopRunning());
    master.processImage().triggerSend();       // new loop still works
    ASSERT_TRUE(waitFor([&] {
        return master.getAsyncLoopStats().sends >= 1;
    }));
    master.stop();
}

TEST(MasterAsyncLoop, StopCyclicLoopLeavesAsyncDatapathAlive) {
    // stopCyclicLoop() while only the async loop runs must not tear down
    // the shared datapath out from under the async thread.
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    Master::AsyncLoopConfig cfg;
    cfg.collect_mode = AsyncCyclicLoop::CollectMode::OnSend;
    ASSERT_TRUE(master.startAsyncLoop(cfg));
    std::this_thread::sleep_for(5ms);

    master.stopCyclicLoop();                   // cyclic isn't running —
    EXPECT_TRUE(master.isAsyncLoopRunning());  // async must survive

    master.processImage().triggerSend();
    ASSERT_TRUE(waitFor([&] {
        return master.getAsyncLoopStats().sends >= 1;
    }));
    master.stop();
}

// ============================================================================
// Q24 — trigger stamp + trigger→send latency stat
// ============================================================================

TEST(ProcessImageSendTrigger, SendStampRecorded) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    EXPECT_EQ(img.sendStampNs(), 0u);
    const uint64_t before = monoNs();
    img.triggerSend();
    const uint64_t stamp = img.sendStampNs();
    EXPECT_GE(stamp, before);
    EXPECT_LE(stamp, monoNs());
}

TEST(AsyncCyclicLoop, SendLatencyStatTracked) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    AsyncCyclicLoop loop(img, [&] { return true; },
                         [&] { return true; }, nullptr, nullptr, loopCfg());
    ASSERT_TRUE(loop.start());
    img.triggerSend();
    ASSERT_TRUE(waitFor([&] { return loop.getStats().sends >= 1; }));
    // Latency measured: trigger stamp → send wake, strictly positive.
    EXPECT_GT(loop.getStats().max_send_latency_ns, 0u);
    loop.stop();
}

// ============================================================================
// Q26 — per-producer trigger accounting
// ============================================================================

TEST(ProcessImageSendTrigger, ProducerSlotClaimAndCount) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    const int slot = img.claimProducerSlot();
    EXPECT_GE(slot, 1);                    // 0 is the anonymous bucket
    EXPECT_LE(slot, 7);
    img.triggerSend();                     // anonymous
    img.triggerSend();
    img.triggerSend(static_cast<uint32_t>(slot));
    EXPECT_EQ(img.producerTriggerCount(0), 2u);
    EXPECT_EQ(img.producerTriggerCount(static_cast<uint32_t>(slot)), 1u);
    // An out-of-range id is ignored.
    img.triggerSend(77);
    EXPECT_EQ(img.sendSeq(), 4u);          // still bumps the shared seq
}

TEST(ProcessImageSendTrigger, ProducerSlotsExhaustToAnonymous) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    int seen = 0;
    for (int i = 0; i < 7; ++i) {
        const int s = img.claimProducerSlot();
        ASSERT_GE(s, 1);
        ++seen;
    }
    EXPECT_EQ(seen, 7);                    // slots 1..7
    EXPECT_EQ(img.claimProducerSlot(), -1);// exhausted → caller falls back to 0
}

// ============================================================================
// Q27 — one-shot warning when a cyclic loop owns the wire
// ============================================================================

TEST(ProcessImageSendTrigger, TriggerUnderCyclicWarnsOnce) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> warns{0};
    auto& logger = Tether::Platform::Logger::instance();
    const auto hid = logger.addHandler(
        [&](Tether::Platform::LogLevel lvl, const char* tag,
            const char* msg) {
            if (lvl == Tether::Platform::LogLevel::Warn &&
                std::string_view(tag) == "process_image" &&
                std::string_view(msg).find("cyclic loop") !=
                    std::string_view::npos)
                ++warns;
        });
    img.setSendConsumer(ProcessImage::SendConsumer::Cyclic);
    img.triggerSend();
    img.triggerSend();
    img.triggerSend();
    logger.removeHandler(hid);
    EXPECT_EQ(warns.load(), 1);            // one-shot, not per-call
    // Async consumer → no warning.
    warns = 0;
    const auto hid2 = logger.addHandler(
        [&](Tether::Platform::LogLevel lvl, const char* tag, const char*) {
            if (lvl == Tether::Platform::LogLevel::Warn &&
                std::string_view(tag) == "process_image")
                ++warns;
        });
    img.setSendConsumer(ProcessImage::SendConsumer::Async);
    img.triggerSend();
    logger.removeHandler(hid2);
    EXPECT_EQ(warns.load(), 0);
}

// ============================================================================
// Q23 — independent DC thread alongside the async loop
// ============================================================================

TEST(AsyncCyclicLoop, DedicatedDcThreadTicks) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> dc_ticks{0};
    auto c = loopCfg();
    c.dc_interval_us = 5'000;              // 5 ms DC tick
    AsyncCyclicLoop loop(img,
        [&] { return true; }, [&] { return true; },
        [&] { ++dc_ticks; return true; },
        nullptr, c);
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(waitFor([&] { return dc_ticks.load() >= 2; }));
    EXPECT_GE(loop.getStats().dc_sync_count, 2u);
    // Triggers still drive sends — DC is independent of them.
    img.triggerSend();
    ASSERT_TRUE(waitFor([&] { return loop.getStats().sends >= 1; }));
    loop.stop();
}

TEST(AsyncCyclicLoop, DcIntervalWithoutTaskWarnsButRuns) {
    auto imgp = makeImage();
    ProcessImage& img = *imgp;
    std::atomic<int> sends{0};
    auto c = loopCfg();
    c.dc_interval_us = 5'000;              // set but no dc_sync task
    AsyncCyclicLoop loop(img, [&] { ++sends; return true; },
                         nullptr, nullptr, nullptr, c);
    ASSERT_TRUE(loop.start());             // warns, still runs
    img.triggerSend();
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 1; }));
    EXPECT_EQ(loop.getStats().dc_sync_count, 0u);
    loop.stop();
}

// ============================================================================
// Q25 — shm waiter-epoch reclaim (Linux shm only)
// ============================================================================

#ifdef __linux__
TEST(ProcessImageSendTrigger, ShmWaiterEpochReclaimsStaleCount) {
    const char* name = "tether-test-epoch-reclaim";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.shm_name = name;
    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));

    // Simulate a crashed waiter: a registered count that never unregisters.
    auto* hdr = reinterpret_cast<ShmImageHeader*>(
        const_cast<uint8_t*>(server.acquireSendImage())
            - ShmImageLayout::outputOff());
    hdr->send_waiters.fetch_add(1, std::memory_order_acq_rel);
    const uint32_t gen0 = hdr->send_epoch.load();
    ASSERT_GT(hdr->send_waiters.load() & 0xFFFFFFu, 0u);

    // A fresh attach reclaims the stale count into a new generation.
    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    EXPECT_EQ(hdr->send_waiters.load() & 0xFFFFFFu, 0u);
    EXPECT_GT(hdr->send_epoch.load(), gen0);
    ::shm_unlink("/tether-test-epoch-reclaim");
}
#endif

} // namespace
