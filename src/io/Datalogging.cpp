/**
 * @file Datalogging.cpp
 * @brief Binary datalogging recorder implementation.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Datalogging.hpp"
#include <cstring>

namespace tether { namespace io {

void DatalogRecorder::configure(const DatalogConfig& config,
                                 ReadEntryFn readFn,
                                 TimestampFn tsFn,
                                 DatalogSinkFn sink) {
    config_ = config;
    readFn_ = std::move(readFn);
    tsFn_ = tsFn;
    sink_ = std::move(sink);

    // Build the metadata (record layout)
    metadata_.logName = config_.logName;
    metadata_.sampleRateHz = config_.sampleRateHz;
    metadata_.fields.clear();

    uint16_t offset = 8;  // First 8 bytes are the timestamp
    for (uint64_t id : config_.entryIds) {
        DatalogField field;
        field.entryId = id;
        field.name = "";  // Will be filled if registry is available
        field.type = ValueType::U8;  // Placeholder
        field.offset = offset;
        field.size = 0;
        field.kind = EntryKind::Signal;
        metadata_.fields.push_back(field);
    }
    // Note: The actual field metadata (name, type, size) should be set
    // by the caller or filled from the registry before starting.

    // Calculate record size
    uint32_t totalSize = 8;  // timestamp
    for (const auto& f : metadata_.fields) {
        totalSize += f.size;
    }
    metadata_.recordSize = totalSize;

    recordBuf_.resize(totalSize, 0);
    state_ = DatalogState::Idle;
    recordsWritten_ = 0;
    bytesWritten_ = 0;
}

void DatalogRecorder::start() {
    if (state_ == DatalogState::Idle || state_ == DatalogState::Stopped) {
        state_ = DatalogState::Recording;
    }
}

void DatalogRecorder::stop() {
    if (state_ == DatalogState::Recording) {
        state_ = DatalogState::Stopped;
    }
}

void DatalogRecorder::sampleOnce() {
    if (state_ != DatalogState::Recording) return;
    if (!tsFn_ || !sink_) return;

    // Write timestamp
    uint64_t ts = tsFn_();
    std::memcpy(recordBuf_.data(), &ts, 8);

    // Read each field
    for (const auto& field : metadata_.fields) {
        if (readFn_ && field.size > 0) {
            readFn_(field.entryId, recordBuf_.data() + field.offset, field.size);
        }
    }

    // Write record to sink
    sink_(recordBuf_.data(), recordBuf_.size());
    ++recordsWritten_;
    bytesWritten_ += recordBuf_.size();
}

DatalogStatus DatalogRecorder::status() const {
    DatalogStatus s;
    s.state = state_;
    s.recordsWritten = recordsWritten_;
    s.bytesWritten = bytesWritten_;
    s.metadata = metadata_;
    return s;
}

}} // namespace tether::io
