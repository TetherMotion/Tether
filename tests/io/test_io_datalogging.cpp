/**
 * @file test_io_datalogging.cpp
 * @brief Unit tests for DatalogMetadata, DatalogConfig, DatalogRecorder.
 */
#include <gtest/gtest.h>
#include "tether/io/Datalogging.hpp"
#include <vector>

using namespace tether::io;

// ===========================================================================
// DatalogMetadata encode/decode
// ===========================================================================

TEST(IODatalog, MetadataRoundtrip) {
    DatalogMetadata meta;
    meta.logName = "test_log";
    meta.recordSize = 24;
    meta.sampleRateHz = 1000;
    meta.fields = {
        {0x01, "timestamp", ValueType::U64, 0, 8, EntryKind::Signal},
        {0x02, "position", ValueType::F64, 8, 8, EntryKind::Signal},
        {0x03, "velocity", ValueType::F64, 16, 8, EntryKind::Signal},
    };

    uint8_t buf[1024];
    BufWriter w(buf, sizeof(buf));
    meta.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogMetadata decoded;
    EXPECT_TRUE(DatalogMetadata::decode(r, decoded));

    EXPECT_EQ(decoded.logName, "test_log");
    EXPECT_EQ(decoded.recordSize, 24u);
    EXPECT_EQ(decoded.sampleRateHz, 1000u);
    ASSERT_EQ(decoded.fields.size(), 3u);
    EXPECT_EQ(decoded.fields[0].name, "timestamp");
    EXPECT_EQ(decoded.fields[1].type, ValueType::F64);
    EXPECT_EQ(decoded.fields[2].offset, 16u);
}

// ===========================================================================
// DatalogConfig encode/decode
// ===========================================================================

TEST(IODatalog, ConfigRoundtrip) {
    DatalogConfig cfg;
    cfg.logName = "perf_log";
    cfg.sampleRateHz = 500;
    cfg.enabled = true;
    cfg.entryIds = {10, 20, 30};

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    cfg.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogConfig decoded;
    EXPECT_TRUE(DatalogConfig::decode(r, decoded));

    EXPECT_EQ(decoded.logName, "perf_log");
    EXPECT_EQ(decoded.sampleRateHz, 500u);
    EXPECT_TRUE(decoded.enabled);
    EXPECT_EQ(decoded.entryIds, (std::vector<uint64_t>{10, 20, 30}));
}

// ===========================================================================
// DatalogStatus encode/decode
// ===========================================================================

TEST(IODatalog, StatusRoundtrip) {
    DatalogStatus status;
    status.state = DatalogState::Recording;
    status.recordsWritten = 42;
    status.bytesWritten = 1024;
    status.metadata.logName = "s";
    status.metadata.recordSize = 8;
    status.metadata.sampleRateHz = 100;

    uint8_t buf[512];
    BufWriter w(buf, sizeof(buf));
    status.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogStatus decoded;
    EXPECT_TRUE(DatalogStatus::decode(r, decoded));

    EXPECT_EQ(decoded.state, DatalogState::Recording);
    EXPECT_EQ(decoded.recordsWritten, 42u);
    EXPECT_EQ(decoded.bytesWritten, 1024u);
    EXPECT_EQ(decoded.metadata.logName, "s");
}

// ===========================================================================
// DatalogRecorder
// ===========================================================================

TEST(IODatalog, RecorderBasicFlow) {
    DatalogRecorder recorder;

    double value1 = 1.5;
    double value2 = 2.5;

    auto readFn = [&](uint64_t id, void* dest, size_t maxSize) -> bool {
        if (id == 1 && maxSize >= 8) { std::memcpy(dest, &value1, 8); return true; }
        if (id == 2 && maxSize >= 8) { std::memcpy(dest, &value2, 8); return true; }
        return false;
    };

    uint64_t fakeTime = 1000000;
    auto tsFn = [&fakeTime]() -> uint64_t { return fakeTime; };

    std::vector<std::vector<uint8_t>> records;
    auto sink = [&records](const uint8_t* data, size_t size) {
        records.emplace_back(data, data + size);
    };

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.sampleRateHz = 100;
    cfg.enabled = true;
    cfg.entryIds = {1, 2};

    recorder.configure(cfg, readFn, tsFn, sink);
    recorder.start();

    auto status = recorder.status();
    EXPECT_EQ(status.state, DatalogState::Recording);

    recorder.sampleOnce();
    fakeTime += 10000;
    recorder.sampleOnce();

    EXPECT_EQ(records.size(), 2u);

    recorder.stop();
    status = recorder.status();
    EXPECT_EQ(status.state, DatalogState::Stopped);
    EXPECT_EQ(status.recordsWritten, 2u);
}

TEST(IODatalog, RecorderIdleNoRecords) {
    DatalogRecorder recorder;
    recorder.sampleOnce();  // Should be a no-op when not configured

    auto status = recorder.status();
    EXPECT_EQ(status.state, DatalogState::Idle);
    EXPECT_EQ(status.recordsWritten, 0u);
}
