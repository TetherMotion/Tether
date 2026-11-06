// DeduplicatingLogger: per-window emission budget, summary emission, flush/reset.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "logging/DeduplicatingLogger.hpp"
#include "logging/Logger.hpp"
#include "tether/platform/Platform.hpp"

using Tether::Logging::DedupLogConfig;
using Tether::Logging::DeduplicatingLogger;
using Tether::Platform::LogLevel;

namespace {

// Installs a fake millisecond clock and a capturing log handler for the
// duration of a test; restores both on destruction.
struct DedupEnv {
    DedupEnv() {
        savedClock = Tether::Platform::Clock::instance().getGetMicroseconds();
        Tether::Platform::Clock::instance().setGetMicroseconds(
            [this] { return nowMs * 1000; });
        savedLevel = Tether::Platform::Logger::instance().getLevel();
        Tether::Platform::Logger::instance().setLevel(LogLevel::Debug);
        Tether::Platform::Logger::instance().setHandler(
            [this](LogLevel lvl, const char* tag, const char* msg) {
                emitted.push_back({lvl, tag ? tag : "", msg ? msg : ""});
            });
    }
    ~DedupEnv() {
        Tether::Platform::Clock::instance().setGetMicroseconds(savedClock);
        Tether::Platform::Logger::instance().setHandler(nullptr);
        Tether::Platform::Logger::instance().setLevel(savedLevel);
    }
    struct Entry { LogLevel level; std::string tag; std::string msg; };
    std::vector<Entry> emitted;
    Tether::Platform::Clock::GetTimeFn savedClock;
    LogLevel savedLevel = LogLevel::Info;
    int64_t nowMs = 0;
};

DedupLogConfig cfg(uint32_t maxPer = 2, uint32_t windowMs = 10000,
                   bool summary = true) {
    DedupLogConfig c;
    c.max_per_window = maxPer;
    c.window_ms = windowMs;
    c.emit_summary = summary;
    c.summary_level = LogLevel::Warn;
    return c;
}

TEST(DeduplicatingLogger, AllowsUpToMaxPerWindow) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(2));
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m1"));
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m2"));
    EXPECT_FALSE(d.log(LogLevel::Info, "T", "m3"));   // suppressed
    EXPECT_FALSE(d.log(LogLevel::Info, "T", "m4"));   // suppressed
    EXPECT_EQ(env.emitted.size(), 2u);
    EXPECT_EQ(env.emitted[0].msg, "m1");
}

TEST(DeduplicatingLogger, WindowRolloverEmitsSummary) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(1, 10000));
    d.log(LogLevel::Info, "T", "m1");
    d.log(LogLevel::Info, "T", "suppressed-a");
    d.log(LogLevel::Info, "T", "suppressed-b");

    env.nowMs += 10001;                                // next window
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m-after"));

    // m1 + summary line + the new message
    ASSERT_EQ(env.emitted.size(), 3u);
    EXPECT_EQ(env.emitted[0].msg, "m1");
    EXPECT_EQ(env.emitted[1].level, LogLevel::Warn);   // summary_level
    EXPECT_NE(env.emitted[1].msg.find("suppressed 2 messages"), std::string::npos);
    EXPECT_NE(env.emitted[1].msg.find("grp"), std::string::npos);
    EXPECT_EQ(env.emitted[2].msg, "m-after");
}

TEST(DeduplicatingLogger, NoSummaryWhenDisabled) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(1, 10000, /*summary=*/false));
    d.log(LogLevel::Info, "T", "m1");
    d.log(LogLevel::Info, "T", "suppressed");
    env.nowMs += 10001;
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m2"));
    ASSERT_EQ(env.emitted.size(), 2u);                 // no summary line
    EXPECT_EQ(env.emitted[1].msg, "m2");
}

TEST(DeduplicatingLogger, FlushEmitsPendingSummary) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(1));
    d.log(LogLevel::Info, "T", "m1");
    d.log(LogLevel::Info, "T", "suppressed");
    d.flush("TAG");
    ASSERT_EQ(env.emitted.size(), 2u);
    EXPECT_EQ(env.emitted[1].tag, "TAG");
    EXPECT_NE(env.emitted[1].msg.find("suppressed 1 messages"), std::string::npos);
    // After flush the window counters restart — next log is emitted.
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m2"));
}

TEST(DeduplicatingLogger, ResetClearsCounters) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(1));
    d.log(LogLevel::Info, "T", "m1");
    d.log(LogLevel::Info, "T", "suppressed");
    d.reset();
    // reset() clears without emitting a summary
    EXPECT_EQ(env.emitted.size(), 1u);
    EXPECT_TRUE(d.log(LogLevel::Info, "T", "m2"));
}

TEST(DeduplicatingLogger, LegacyLevelMapping) {
    DedupEnv env;
    DeduplicatingLogger d("grp", cfg(4));
    d.logLegacy(0, "T", "info");
    d.logLegacy(1, "T", "warn");
    d.logLegacy(2, "T", "err");
    d.logLegacy(99, "T", "dbg");
    ASSERT_EQ(env.emitted.size(), 4u);
    EXPECT_EQ(env.emitted[0].level, LogLevel::Info);
    EXPECT_EQ(env.emitted[1].level, LogLevel::Warn);
    EXPECT_EQ(env.emitted[2].level, LogLevel::Error);
    EXPECT_EQ(env.emitted[3].level, LogLevel::Debug);
}

TEST(DeduplicatingLogger, NullTagAndMsgSafe) {
    DedupEnv env;
    DeduplicatingLogger d(nullptr, cfg(2));
    EXPECT_TRUE(d.log(LogLevel::Info, nullptr, nullptr));
    EXPECT_EQ(env.emitted[0].tag, "");
    EXPECT_EQ(env.emitted[0].msg, "");
    d.flush(nullptr);                                  // empty tag path
}

}  // namespace
