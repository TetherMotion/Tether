#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#ifdef __linux__
#include <sched.h>
#endif
#include "tether/platform/Platform.hpp"

using namespace Tether::Platform;

TEST(PlatformLogger, HandlerAndLevelFiltering) {
    Logger& logger = Logger::instance();

    // capture calls
    struct Captured { LogLevel lvl; std::string tag; std::string msg; } cap;
    bool called = false;

    logger.setHandler([&](LogLevel lvl, const char* tag, const char* msg){
        called = true;
        cap.lvl = lvl;
        cap.tag = tag;
        cap.msg = msg;
    });

    logger.setLevel(LogLevel::Info);
    called = false;
    TETHER_LOGI("TestTag", "hello %s", "world");
    EXPECT_TRUE(called);
    EXPECT_EQ(cap.lvl, LogLevel::Info);
    EXPECT_EQ(cap.tag, "TestTag");
    EXPECT_EQ(cap.msg, "hello world");

    // Filtering: set level to Warn, Info should not be delivered
    logger.setLevel(LogLevel::Warn);
    called = false;
    TETHER_LOGI("TestTag", "should not appear");
    EXPECT_FALSE(called);

    // Error should still be delivered
    called = false;
    TETHER_LOGE("ErrTag", "bad %d", 42);
    EXPECT_TRUE(called);
    EXPECT_EQ(cap.lvl, LogLevel::Error);
    EXPECT_EQ(cap.tag, "ErrTag");
    EXPECT_EQ(cap.msg, "bad 42");

    // Restore defaults so later tests don't invoke a dangling handler
    logger.setHandler(nullptr);
    logger.setLevel(LogLevel::Info);
}

TEST(Clock, GetTimeAndDelayAndYield) {
    Clock& clk = Clock::instance();

    // Use a controllable get time function
    int64_t t = 1000;
    // Capture by value to avoid dangling reference when the test function returns
    clk.setGetMicroseconds([t]() -> int64_t { return t; });

    EXPECT_EQ(clk.getMicroseconds(), 1000);
    EXPECT_EQ(clk.getMilliseconds(), 1);

    // Test delay function gets called with expected value
    auto observed_delay = std::make_shared<uint32_t>(0u);
    clk.setDelayMicroseconds([observed_delay](uint32_t us){ *observed_delay = us; });
    clk.delayMicroseconds(123);
    EXPECT_EQ(*observed_delay, 123u);

    // Test milliseconds wrapper
    *observed_delay = 0;
    clk.delayMilliseconds(2);
    EXPECT_EQ(*observed_delay, 2000u);

    // Test yield invocation
    auto yield_count = std::make_shared<int>(0);
    clk.setYield([yield_count]() { (*yield_count)++; });
    clk.yield();
    EXPECT_EQ(*yield_count, 1);

    // Restore default implementations so that later tests using
    // xTaskGetTickCount / esp_rom_delay_us see real elapsed time.
    auto startTime = std::chrono::steady_clock::now();
    clk.setGetMicroseconds([startTime]() -> int64_t {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - startTime).count();
    });
    clk.setDelayMicroseconds([](uint32_t us) {
        if (us < 1000) {
            auto s = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - s).count() < static_cast<int64_t>(us)) {}
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(us));
        }
    });
    clk.setYield([]() { std::this_thread::yield(); });
}

TEST(PlatformRealtime, SetCurrentThreadRealtime) {
    // Best-effort: calling must not crash. On Linux it will attempt SCHED_FIFO.
    bool ok = Tether::Platform::setCurrentThreadRealtime(-1);
#ifdef __linux__
    int pol = sched_getscheduler(0);
    if (ok) {
        EXPECT_EQ(pol, SCHED_FIFO);
    } else {
        EXPECT_NE(pol, SCHED_FIFO);
    }
#else
    EXPECT_TRUE(ok); // non-Linux platforms are expected to be no-op and return true
#endif
}
