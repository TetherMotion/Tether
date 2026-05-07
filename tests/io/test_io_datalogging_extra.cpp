/**
 * @file test_io_datalogging_extra.cpp
 * @brief Additional coverage tests for DatalogRecorder.
 */
#include <gtest/gtest.h>
#include "tether/io/Datalogging.hpp"
#include <cstring>

using namespace tether::io;

// ===========================================================================
// DatalogRecorder edge cases
// ===========================================================================

TEST(IODatalogExtra, StopWhenIdle) {
    DatalogRecorder recorder;
    recorder.stop();  // should be a no-op
    auto status = recorder.status();
    EXPECT_EQ(status.state, DatalogState::Idle);
}

TEST(IODatalogExtra, StartWhenStopped) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.sampleRateHz = 100;
    cfg.entryIds = {};

    auto tsFn = []() -> uint64_t { return 0; };
    auto sink = [](const uint8_t*, size_t) {};
    auto readFn = [](uint64_t, void*, size_t) -> bool { return false; };

    recorder.configure(cfg, readFn, tsFn, sink);
    recorder.start();
    EXPECT_EQ(recorder.status().state, DatalogState::Recording);

    recorder.stop();
    EXPECT_EQ(recorder.status().state, DatalogState::Stopped);

    recorder.start();  // Can restart from Stopped
    EXPECT_EQ(recorder.status().state, DatalogState::Recording);
}

TEST(IODatalogExtra, SampleOnceWhenNotRecording) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.entryIds = {};

    auto tsFn = []() -> uint64_t { return 0; };
    auto sink = [](const uint8_t*, size_t) {};
    auto readFn = [](uint64_t, void*, size_t) -> bool { return false; };

    recorder.configure(cfg, readFn, tsFn, sink);
    // Don't start
    recorder.sampleOnce();
    EXPECT_EQ(recorder.status().recordsWritten, 0u);
}

TEST(IODatalogExtra, SampleWithNoSink) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.sampleRateHz = 100;
    cfg.entryIds = {};

    auto tsFn = []() -> uint64_t { return 0; };

    recorder.configure(cfg, nullptr, tsFn, nullptr);
    recorder.start();
    recorder.sampleOnce();  // no sink — should not crash
    EXPECT_EQ(recorder.status().recordsWritten, 0u);
}

TEST(IODatalogExtra, SampleWithNoTimestampFn) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "test";
    cfg.entryIds = {};

    auto sink = [](const uint8_t*, size_t) {};

    recorder.configure(cfg, nullptr, nullptr, sink);
    recorder.start();
    recorder.sampleOnce();  // no tsFn — should not crash
    EXPECT_EQ(recorder.status().recordsWritten, 0u);
}

TEST(IODatalogExtra, MetadataAccess) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "detailed";
    cfg.sampleRateHz = 500;
    cfg.entryIds = {10, 20};

    auto tsFn = []() -> uint64_t { return 0; };
    auto sink = [](const uint8_t*, size_t) {};
    auto readFn = [](uint64_t, void*, size_t) -> bool { return false; };

    recorder.configure(cfg, readFn, tsFn, sink);

    const auto& meta = recorder.metadata();
    EXPECT_EQ(meta.logName, "detailed");
    EXPECT_EQ(meta.sampleRateHz, 500u);
    EXPECT_EQ(meta.fields.size(), 2u);
}

TEST(IODatalogExtra, StatusBytesWritten) {
    DatalogRecorder recorder;

    DatalogConfig cfg;
    cfg.logName = "bytes_test";
    cfg.sampleRateHz = 100;
    cfg.entryIds = {};

    uint64_t ts = 1000;
    auto tsFn = [&ts]() -> uint64_t { return ts++; };

    std::vector<std::vector<uint8_t>> records;
    auto sink = [&records](const uint8_t* data, size_t size) {
        records.emplace_back(data, data + size);
    };
    auto readFn = [](uint64_t, void*, size_t) -> bool { return false; };

    recorder.configure(cfg, readFn, tsFn, sink);
    recorder.start();
    recorder.sampleOnce();
    recorder.sampleOnce();
    recorder.sampleOnce();

    auto status = recorder.status();
    EXPECT_EQ(status.recordsWritten, 3u);
    EXPECT_GT(status.bytesWritten, 0u);
}

// ===========================================================================
// DatalogMetadata with no fields
// ===========================================================================

TEST(IODatalogExtra, EmptyMetadataRoundtrip) {
    DatalogMetadata meta;
    meta.logName = "empty";
    meta.recordSize = 0;
    meta.sampleRateHz = 0;

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    meta.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogMetadata decoded;
    EXPECT_TRUE(DatalogMetadata::decode(r, decoded));
    EXPECT_EQ(decoded.logName, "empty");
    EXPECT_EQ(decoded.fields.size(), 0u);
}

// ===========================================================================
// DatalogConfig with empty entryIds
// ===========================================================================

TEST(IODatalogExtra, ConfigEmptyIdsRoundtrip) {
    DatalogConfig cfg;
    cfg.logName = "all_entries";
    cfg.sampleRateHz = 1000;
    cfg.enabled = true;
    cfg.entryIds = {};

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    cfg.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogConfig decoded;
    EXPECT_TRUE(DatalogConfig::decode(r, decoded));
    EXPECT_EQ(decoded.entryIds.size(), 0u);
    EXPECT_TRUE(decoded.enabled);
}

// ===========================================================================
// DatalogStatus all states
// ===========================================================================

TEST(IODatalogExtra, StatusErrorState) {
    DatalogStatus s;
    s.state = DatalogState::Error;
    s.recordsWritten = 0;
    s.bytesWritten = 0;

    uint8_t buf[512];
    BufWriter w(buf, sizeof(buf));
    s.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogStatus decoded;
    EXPECT_TRUE(DatalogStatus::decode(r, decoded));
    EXPECT_EQ(decoded.state, DatalogState::Error);
}

TEST(IODatalogExtra, StatusIdleState) {
    DatalogStatus s;
    s.state = DatalogState::Idle;

    uint8_t buf[512];
    BufWriter w(buf, sizeof(buf));
    s.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    DatalogStatus decoded;
    EXPECT_TRUE(DatalogStatus::decode(r, decoded));
    EXPECT_EQ(decoded.state, DatalogState::Idle);
}
