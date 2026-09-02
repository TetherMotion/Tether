/**
 * @file test_logging.cpp
 * @brief Tests for Logger
 */

#include "logging/Logger.hpp"

#include <gtest/gtest.h>
#include <string>
#include <regex>

using namespace Tether::Platform;

TEST(LoggerTest, Singleton) {
    auto& l1 = Logger::instance();
    auto& l2 = Logger::instance();
    EXPECT_EQ(&l1, &l2);
}

TEST(LoggerTest, SetLevel) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Verbose);
    logger.setLevel(LogLevel::Debug);
    logger.setLevel(LogLevel::Info);
    logger.setLevel(LogLevel::Warn);
    logger.setLevel(LogLevel::Error);
    logger.setLevel(LogLevel::Verbose);
}

TEST(LoggerTest, GetLevel) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Warn);
    EXPECT_EQ(logger.getLevel(), LogLevel::Warn);
    logger.setLevel(LogLevel::Error);
    EXPECT_EQ(logger.getLevel(), LogLevel::Error);
    logger.setLevel(LogLevel::Verbose);
    EXPECT_EQ(logger.getLevel(), LogLevel::Verbose);
}

TEST(LoggerTest, LogAtVariousLevels) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Verbose);

    logger.log(LogLevel::Verbose, "TestTag", "Verbose message %d", 1);
    logger.log(LogLevel::Debug, "TestTag", "Debug message %d", 2);
    logger.log(LogLevel::Info, "TestTag", "Info message %d", 3);
    logger.log(LogLevel::Warn, "TestTag", "Warn message %d", 4);
    logger.log(LogLevel::Error, "TestTag", "Error message %d", 5);
}

TEST(LoggerTest, LogFiltering) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Error);

    // These should be filtered (below Error level)
    logger.log(LogLevel::Verbose, "TestTag", "Should be filtered");
    logger.log(LogLevel::Debug, "TestTag", "Should be filtered");
    logger.log(LogLevel::Info, "TestTag", "Should be filtered");
    logger.log(LogLevel::Warn, "TestTag", "Should be filtered");

    // This should pass
    logger.log(LogLevel::Error, "TestTag", "Should pass");

    // Reset to allow all
    logger.setLevel(LogLevel::Verbose);
}

TEST(LoggerTest, NoneLevel) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::None);
    EXPECT_EQ(logger.getLevel(), LogLevel::None);
    logger.log(LogLevel::Error, "Test", "Should not appear");
    logger.setLevel(LogLevel::Verbose);
}

TEST(LoggerTest, LogMacros) {
    TETHER_LOGE("MacroTest", "Error message {}", 42);
    TETHER_LOGW("MacroTest", "Warning message {}", "test");
    TETHER_LOGI("MacroTest", "Info message");
    TETHER_LOGD("MacroTest", "Debug message");
}

TEST(LoggerTest, SetHandler) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Verbose);
    logger.setHandler(nullptr);
}

TEST(LoggerTest, LogWithNewlines_HandlerCalledPerLine) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Verbose);

    struct Entry { LogLevel lvl; std::string tag; std::string msg; };
    std::vector<Entry> received;

    logger.setHandler([&received](LogLevel lvl, const char* tag, const char* msg) {
        received.push_back({lvl, std::string(tag), std::string(msg)});
    });

    logger.log(LogLevel::Info, "NLTag", "first\nsecond\nthird");

    ASSERT_EQ(received.size(), 3u);
    EXPECT_EQ(received[0].tag, "NLTag");
    EXPECT_EQ(received[0].msg, "first");
    EXPECT_EQ(received[1].msg, "second");
    EXPECT_EQ(received[2].msg, "third");

    logger.setHandler(nullptr);
}

TEST(LoggerTest, LogWithTrailingNewline_IgnoresTrailingEmptyLine) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Verbose);

    std::vector<std::string> received;
    logger.setHandler([&received](LogLevel, const char*, const char* msg) { received.emplace_back(msg); });

    logger.log(LogLevel::Info, "NLTag", "one\ntwo\n");
    ASSERT_EQ(received.size(), 2u);
    EXPECT_EQ(received[0], "one");
    EXPECT_EQ(received[1], "two");

    logger.setHandler(nullptr);
}

TEST(LoggerTest, TimestampPrintedWhenEnabled_DefaultOutput) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Info);
    logger.setHandler(nullptr); // use default printf output

    testing::internal::CaptureStdout();
    logger.log(LogLevel::Info, "TS", "timestamp test");
    std::string out = testing::internal::GetCapturedStdout();

    // strip ANSI color codes if present (test runner might force color)
    out = std::regex_replace(out, std::regex("\\x1b\\[[0-9;]*m"), "");

    // Basic structural checks for ISO8601-like UTC timestamp + level + tag + message
    // (avoid brittle full-regex matching across different stdlib regex implementations)
    // Verify required separator characters at fixed offsets
    ASSERT_GE(out.size(), 24u);
    EXPECT_EQ(out[4], '-');   // YYYY-
    EXPECT_EQ(out[7], '-');   // YYYY-MM-
    EXPECT_EQ(out[10], 'T');  // date/time separator
    EXPECT_EQ(out[13], ':');  // HH:
    EXPECT_EQ(out[16], ':');  // HH:MM:
    EXPECT_EQ(out[19], '.');  // seconds.milliseconds
    EXPECT_EQ(out[23], 'Z');  // UTC marker

    EXPECT_NE(out.find("[I] TS: timestamp test"), std::string::npos) << "output: " << out;
}

TEST(LoggerTest, TimestampNotAddedToHandler) {
    auto& logger = Logger::instance();
    logger.setLevel(LogLevel::Info);

    std::string received;
    logger.setHandler([&received](LogLevel, const char*, const char* msg) { received = msg; });

    logger.log(LogLevel::Info, "TSH", "handler test");
    // handler messages must not include the timestamp (timestamp only affects default output)
    EXPECT_EQ(received, "handler test");

    logger.setHandler(nullptr);
}
