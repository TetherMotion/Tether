/**
 * @file test_slave_logger.cpp
 * @brief Coverage for EtherCAT::slave::SlaveLogger: filtering, queue,
 *        sinks (callback/file/console), frame+hex logging, pcap hook,
 *        stats, and the processing-thread lifecycle.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "tether/slave/logging/SlaveLogger.hpp"

using namespace EtherCAT::slave;
using Tether::PacketLoggers::FrameDirection;

namespace {

// Collect entries through the custom-callback sink without the bg thread.
struct Capture {
    SlaveLogConfig cfg;
    std::vector<SlaveLogEntry> entries;

    Capture() {
        cfg.consoleEnabled = false;
        cfg.consoleColors = false;
        cfg.fileEnabled = false;
        cfg.minLevel = SlaveLogLevel::Trace;     // accept everything
        cfg.enabledCategories = SlaveLogCategory::All;
        cfg.queueSize = 64;
        cfg.customCallback = [this](const SlaveLogEntry& e) { entries.push_back(e); };
    }
};

// ============================================================================
// Filtering
// ============================================================================

TEST(SlaveLoggerCov, CategoryFilterDropsDisabled) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.setCategoryEnabled(SlaveLogCategory::CoE, false);
    log.log(SlaveLogCategory::CoE, SlaveLogLevel::Info, "coe msg");
    log.log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info, "sm msg");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_STREQ(cap.entries[0].message, "sm msg");
}

TEST(SlaveLoggerCov, MinLevelFilter) {
    Capture cap;
    cap.cfg.minLevel = SlaveLogLevel::Warning;
    SlaveLogger log(cap.cfg);
    log.info(SlaveLogCategory::StateMachine, "info msg");     // below min
    log.warn(SlaveLogCategory::StateMachine, "warn msg");
    log.error(SlaveLogCategory::StateMachine, "err msg");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 2u);
    EXPECT_STREQ(cap.entries[0].message, "warn msg");
    EXPECT_STREQ(cap.entries[1].message, "err msg");
}

TEST(SlaveLoggerCov, SetMinLevelRuntime) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.setMinLevel(SlaveLogLevel::Error);
    EXPECT_EQ(log.getMinLevel(), SlaveLogLevel::Error);
    log.warn(SlaveLogCategory::StateMachine, "warn");
    log.critical(SlaveLogCategory::StateMachine, "crit");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_STREQ(cap.entries[0].message, "crit");
}

TEST(SlaveLoggerCov, IsCategoryEnabledReflectsMask) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    EXPECT_TRUE(log.isCategoryEnabled(SlaveLogCategory::DC));
    log.setEnabledCategories(SlaveLogCategory::Minimal);
    EXPECT_TRUE(log.isCategoryEnabled(SlaveLogCategory::StateMachine));
    EXPECT_FALSE(log.isCategoryEnabled(SlaveLogCategory::DC));
}

// ============================================================================
// Convenience methods + levels
// ============================================================================

TEST(SlaveLoggerCov, AllConvenienceLevels) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.trace(SlaveLogCategory::CoE, "t");
    log.debug(SlaveLogCategory::CoE, "d");
    log.info(SlaveLogCategory::CoE, "i");
    log.warn(SlaveLogCategory::CoE, "w");
    log.error(SlaveLogCategory::CoE, "e");
    log.critical(SlaveLogCategory::CoE, "c");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 6u);
    EXPECT_EQ(cap.entries[0].level, SlaveLogLevel::Trace);
    EXPECT_EQ(cap.entries[5].level, SlaveLogLevel::Critical);
}

TEST(SlaveLoggerCov, LogFormatArgs) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.log(SlaveLogCategory::Register, SlaveLogLevel::Info,
            "wrote %d bytes to 0x%04X", 4, 0x6040);
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_STREQ(cap.entries[0].message, "wrote 4 bytes to 0x6040");
}

TEST(SlaveLoggerCov, SlaveAddressStampedOnEntries) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.setSlaveAddress(0x1234);
    log.info(SlaveLogCategory::DC, "x");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_EQ(cap.entries[0].slaveAddress, 0x1234);
    EXPECT_EQ(cap.entries[0].category, SlaveLogCategory::DC);
}

// ============================================================================
// Frames + hex
// ============================================================================

TEST(SlaveLoggerCov, LogFrameWithDescriptionAndData) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    uint8_t frame[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    log.logFrame(SlaveLogCategory::FrameRx, frame, sizeof(frame), "APRD");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_EQ(cap.entries[0].level, SlaveLogLevel::Debug);    // frames always Debug
    EXPECT_NE(std::strstr(cap.entries[0].message, "APRD"), nullptr);
    EXPECT_EQ(cap.entries[0].dataLen, 4u);
    EXPECT_EQ(cap.entries[0].data[0], 0xDE);
    EXPECT_EQ(cap.entries[0].data[3], 0xEF);
}

TEST(SlaveLoggerCov, LogFrameNullDescription) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    uint8_t frame[2] = {1, 2};
    log.logFrame(SlaveLogCategory::FrameTx, frame, sizeof(frame));
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_NE(std::strstr(cap.entries[0].message, "Frame"), nullptr);
    EXPECT_EQ(cap.entries[0].dataLen, 2u);
}

TEST(SlaveLoggerCov, LogFrameTruncatesDataAt64Bytes) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    uint8_t frame[128] = {};
    log.logFrame(SlaveLogCategory::FrameRx, frame, sizeof(frame));
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_EQ(cap.entries[0].dataLen, 64u);     // capped at entry.data[]
}

TEST(SlaveLoggerCov, LogFrameBypassesCategoryFilter) {
    Capture cap;
    cap.cfg.enabledCategories = SlaveLogCategory::StateMachine;  // not Frames
    SlaveLogger log(cap.cfg);
    uint8_t frame[2] = {9, 9};
    log.logFrame(SlaveLogCategory::FrameRx, frame, sizeof(frame));  // still queued
    log.flush();
    EXPECT_EQ(cap.entries.size(), 1u);
}

TEST(SlaveLoggerCov, LogHexWithPrefix) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    uint8_t data[3] = {0x0A, 0x0B, 0x0C};
    log.logHex(SlaveLogCategory::Datagram, SlaveLogLevel::Info, data, 3, "SDO");
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_NE(std::strstr(cap.entries[0].message, "SDO:"), nullptr);
    EXPECT_NE(std::strstr(cap.entries[0].message, "0A 0B 0C"), nullptr);
}

TEST(SlaveLoggerCov, LogHexNoPrefixAndFiltered) {
    Capture cap;
    cap.cfg.minLevel = SlaveLogLevel::Warning;
    SlaveLogger log(cap.cfg);
    uint8_t data[2] = {1, 2};
    log.logHex(SlaveLogCategory::Datagram, SlaveLogLevel::Info, data, 2);  // below min
    log.logHex(SlaveLogCategory::Datagram, SlaveLogLevel::Error, data, 2);
    log.flush();
    ASSERT_EQ(cap.entries.size(), 1u);
    EXPECT_NE(std::strstr(cap.entries[0].message, "01 02"), nullptr);
}

// ============================================================================
// Queue / stats / lifecycle
// ============================================================================

TEST(SlaveLoggerCov, QueueFullCountsDrops) {
    Capture cap;
    cap.cfg.queueSize = 4;
    SlaveLogger log(cap.cfg);           // not started → nothing drains
    for (int i = 0; i < 10; ++i)
        log.info(SlaveLogCategory::CoE, "m%d", i);
    EXPECT_EQ(log.getLogCount(), 4u);    // only 4 fit
    EXPECT_EQ(log.getDropCount(), 6u);
}

TEST(SlaveLoggerCov, ResetStatsZeroesCounters) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.info(SlaveLogCategory::CoE, "x");
    EXPECT_GT(log.getLogCount(), 0u);
    log.resetStats();
    EXPECT_EQ(log.getLogCount(), 0u);
    EXPECT_EQ(log.getDropCount(), 0u);
}

TEST(SlaveLoggerCov, StartStopRunsDrainThread) {
    Capture cap;
    {
        SlaveLogger log(cap.cfg);
        log.start();
        EXPECT_TRUE(log.isRunning());
        log.info(SlaveLogCategory::StateMachine, "threaded");
        log.stop();                      // joins + flushes tail
        EXPECT_FALSE(log.isRunning());
    }
    EXPECT_GE(cap.entries.size(), 1u);   // drained by thread or stop-flush
}

TEST(SlaveLoggerCov, DoubleStartIsNoop) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    log.start();
    log.start();                         // second start returns early
    EXPECT_TRUE(log.isRunning());
    log.stop();
    log.stop();                          // second stop returns early
}

// ============================================================================
// Console + file sinks
// ============================================================================

TEST(SlaveLoggerCov, ConsoleSinkWritesToStderr) {
    SlaveLogConfig cfg;
    cfg.consoleEnabled = true;
    cfg.consoleColors = true;
    cfg.fileEnabled = false;
    cfg.minLevel = SlaveLogLevel::Trace;
    cfg.enabledCategories = SlaveLogCategory::All;
    SlaveLogger log(cfg);
    // Redirect stderr to /dev/null via dup so test output stays clean.
    int devnull = ::open("/dev/null", O_WRONLY);
    int saved = ::dup(STDERR_FILENO);
    ::dup2(devnull, STDERR_FILENO);
    uint8_t f[3] = {0xAA, 0xBB, 0xCC};
    log.warn(SlaveLogCategory::Watchdog, "console test");
    log.logFrame(SlaveLogCategory::FrameRx, f, sizeof(f), "with data");
    log.flush();                          // exercises writeToConsole incl. data line
    ::fflush(stderr);
    ::dup2(saved, STDERR_FILENO);
    ::close(saved);
    ::close(devnull);
}

TEST(SlaveLoggerCov, ConsoleSinkNoColors) {
    SlaveLogConfig cfg;
    cfg.consoleEnabled = true;
    cfg.consoleColors = false;             // "" color branch
    cfg.minLevel = SlaveLogLevel::Trace;
    cfg.enabledCategories = SlaveLogCategory::All;
    SlaveLogger log(cfg);
    int devnull = ::open("/dev/null", O_WRONLY);
    int saved = ::dup(STDERR_FILENO);
    ::dup2(devnull, STDERR_FILENO);
    log.error(SlaveLogCategory::SII, "nocolor");
    log.flush();
    ::fflush(stderr);
    ::dup2(saved, STDERR_FILENO);
    ::close(saved);
    ::close(devnull);
}

TEST(SlaveLoggerCov, FileSinkWritesAndRotates) {
    std::string path = "/tmp/tether_slog_test.log";
    ::unlink(path.c_str());
    for (int i = 1; i <= 6; ++i)
        ::unlink((path + "." + std::to_string(i)).c_str());

    SlaveLogConfig cfg;
    cfg.consoleEnabled = false;
    cfg.fileEnabled = true;
    cfg.logFilePath = path;
    cfg.maxFileSize = 256;                 // tiny → forces rotation
    cfg.maxRotatedFiles = 3;
    cfg.minLevel = SlaveLogLevel::Trace;
    cfg.enabledCategories = SlaveLogCategory::All;

    {
        SlaveLogger log(cfg);
        log.start();                       // opens file
        for (int i = 0; i < 40; ++i)
            log.info(SlaveLogCategory::CoE, "log line %d with some padding to grow the file quickly", i);
        log.stop();                        // flushes + closes
    }

    // Main file exists and has content; rotation produced .1 (and maybe .2).
    FILE* f = std::fopen(path.c_str(), "r");
    ASSERT_NE(f, nullptr);
    char buf[256];
    ASSERT_NE(std::fgets(buf, sizeof(buf), f), nullptr);
    EXPECT_NE(std::strstr(buf, "CoE"), nullptr);
    std::fclose(f);
    EXPECT_EQ(::access((path + ".1").c_str(), F_OK), 0);

    for (int i = 1; i <= 6; ++i)
        ::unlink((path + "." + std::to_string(i)).c_str());
    ::unlink(path.c_str());
}

TEST(SlaveLoggerCov, FileSinkDisabledWithoutPath) {
    SlaveLogConfig cfg;
    cfg.consoleEnabled = false;
    cfg.fileEnabled = true;
    cfg.logFilePath.clear();               // enabled but no path → no fopen
    cfg.minLevel = SlaveLogLevel::Trace;
    cfg.enabledCategories = SlaveLogCategory::All;
    SlaveLogger log(cfg);
    log.start();
    log.info(SlaveLogCategory::CoE, "nowhere to go");
    log.stop();
    SUCCEED();
}

// ============================================================================
// Pcap hook + macros
// ============================================================================

TEST(SlaveLoggerCov, PcapHookNoLoggerIsNoop) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    uint8_t f[4] = {1, 2, 3, 4};
    EXPECT_EQ(log.getPcapLogger(), nullptr);
    log.logFrameToPcap(f, sizeof(f), FrameDirection::Rx);   // pcapLogger_ null
    log.logFrameToPcap(f, sizeof(f), FrameDirection::Tx, 12345);
    SUCCEED();
}

TEST(SlaveLoggerCov, LoggingMacros) {
    Capture cap;
    SlaveLogger log(cap.cfg);
    SLAVE_TRACE(&log, SlaveLogCategory::DC, "tr");
    SLAVE_DEBUG(&log, SlaveLogCategory::DC, "db");
    SLAVE_INFO(&log, SlaveLogCategory::DC, "in");
    SLAVE_WARN(&log, SlaveLogCategory::DC, "wa");
    SLAVE_ERROR(&log, SlaveLogCategory::DC, "er");
    SLAVE_CRITICAL(&log, SlaveLogCategory::DC, "cr");
    SlaveLogger* nullLog = nullptr;
    SLAVE_INFO(nullLog, SlaveLogCategory::DC, "null-safe");
    log.flush();
    EXPECT_EQ(cap.entries.size(), 6u);
}

}  // namespace
