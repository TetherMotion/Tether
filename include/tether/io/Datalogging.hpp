/**
 * @file Datalogging.hpp
 * @brief Binary datalogging subsystem for the Tether IO protocol.
 *
 * Provides efficient binary recording of parameter and signal values.
 * The datalog produces fixed-layout binary records whose format is
 * described by a metadata structure (field names, offsets, types, sizes),
 * allowing offline tools to decode the log without runtime access to the
 * server's parameter catalog.
 *
 * ## Design
 *
 * - The server-side `DatalogRecorder` accumulates raw binary rows at the
 *   configured rate, writing them to a caller-supplied output sink
 *   (file, ring buffer, etc.).
 * - A `DatalogMetadata` structure describes the binary layout and is
 *   written at the head of the log or sent to the client on request.
 * - Clients can configure which entries are logged, the sample rate,
 *   and the output destination via `ConfigureDatalogReq`.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <functional>

namespace tether { namespace io {

/// Describes one field in a binary datalog record.
struct DatalogField {
    uint64_t    entryId;    ///< Parameter or signal ID
    std::string name;       ///< Human-readable name
    ValueType   type;       ///< Data type
    uint16_t    offset;     ///< Byte offset in the record
    uint16_t    size;       ///< Byte size of this field in the record
    EntryKind   kind;       ///< Parameter or Signal
};

/// Metadata describing the binary layout of a datalog.
struct DatalogMetadata {
    std::string       logName;      ///< Name/identifier for this log
    uint32_t          recordSize;   ///< Total bytes per record (including timestamp)
    uint32_t          sampleRateHz; ///< Nominal sample rate
    std::vector<DatalogField> fields;

    /// Encode metadata to a buffer (for DatalogStatusResp or log file header).
    void encode(BufWriter& w) const {
        w.putStr16(logName.c_str(), logName.size());
        w.putU32(recordSize);
        w.putU32(sampleRateHz);
        w.putU32(static_cast<uint32_t>(fields.size()));
        for (const auto& f : fields) {
            w.putU64(f.entryId);
            w.putStr16(f.name.c_str(), f.name.size());
            w.putU8(static_cast<uint8_t>(f.type));
            w.putU16(f.offset);
            w.putU16(f.size);
            w.putU8(static_cast<uint8_t>(f.kind));
        }
    }

    /// Decode metadata from a buffer. Returns true on success.
    static bool decode(BufReader& r, DatalogMetadata& out) {
        uint16_t nl = r.getU16();
        auto* nb = r.getBytes(nl);
        if (!r.ok()) return false;
        out.logName.assign(reinterpret_cast<const char*>(nb), nl);
        out.recordSize = r.getU32();
        out.sampleRateHz = r.getU32();
        uint32_t fc = r.getU32();
        if (!r.ok() || fc > MAX_COLLECTION_COUNT) return false;
        out.fields.resize(fc);
        for (uint32_t i = 0; i < fc; ++i) {
            out.fields[i].entryId = r.getU64();
            uint16_t fnl = r.getU16();
            auto* fnb = r.getBytes(fnl);
            if (!r.ok()) return false;
            out.fields[i].name.assign(reinterpret_cast<const char*>(fnb), fnl);
            out.fields[i].type = static_cast<ValueType>(r.getU8());
            out.fields[i].offset = r.getU16();
            out.fields[i].size = r.getU16();
            out.fields[i].kind = static_cast<EntryKind>(r.getU8());
        }
        return r.ok();
    }
};

/// Datalog configuration (sent via ConfigureDatalogReq).
struct DatalogConfig {
    std::string logName;
    uint32_t    sampleRateHz = 1000;
    bool        enabled = false;
    std::vector<uint64_t> entryIds;     ///< IDs to log (empty = all)

    void encode(BufWriter& w) const {
        w.putStr16(logName.c_str(), logName.size());
        w.putU32(sampleRateHz);
        w.putU8(enabled ? 1 : 0);
        w.putU32(static_cast<uint32_t>(entryIds.size()));
        for (uint64_t id : entryIds) {
            w.putU64(id);
        }
    }

    static bool decode(BufReader& r, DatalogConfig& out) {
        uint16_t nl = r.getU16();
        auto* nb = r.getBytes(nl);
        if (!r.ok()) return false;
        out.logName.assign(reinterpret_cast<const char*>(nb), nl);
        out.sampleRateHz = r.getU32();
        out.enabled = (r.getU8() != 0);
        uint32_t ec = r.getU32();
        if (!r.ok() || ec > MAX_COLLECTION_COUNT) return false;
        out.entryIds.resize(ec);
        for (uint32_t i = 0; i < ec; ++i) {
            out.entryIds[i] = r.getU64();
        }
        return r.ok();
    }
};

/// Datalog status (sent via DatalogStatusResp).
struct DatalogStatus {
    DatalogState state = DatalogState::Idle;
    uint64_t     recordsWritten = 0;
    uint64_t     bytesWritten = 0;
    DatalogMetadata metadata;

    void encode(BufWriter& w) const {
        w.putU8(static_cast<uint8_t>(state));
        w.putU64(recordsWritten);
        w.putU64(bytesWritten);
        metadata.encode(w);
    }

    static bool decode(BufReader& r, DatalogStatus& out) {
        out.state = static_cast<DatalogState>(r.getU8());
        out.recordsWritten = r.getU64();
        out.bytesWritten = r.getU64();
        if (!r.ok()) return false;
        return DatalogMetadata::decode(r, out.metadata);
    }
};

/// Output sink callback: receives a raw binary record of recordSize bytes.
using DatalogSinkFn = std::function<void(const uint8_t* record, size_t size)>;

/**
 * @class DatalogRecorder
 * @brief Collects and writes binary datalog records.
 *
 * Call configure() with a DatalogConfig, then call sampleOnce() periodically.
 * Each call to sampleOnce() reads all configured entries and writes one
 * binary record to the output sink.
 */
class DatalogRecorder {
public:
    using TimestampFn = std::function<uint64_t()>;
    using ReadEntryFn = std::function<bool(uint64_t entryId, void* dest, size_t maxSize)>;

    /// Configure the recorder. Builds the record layout.
    /// @param config   Logging configuration
    /// @param readFn   Callback to read an entry value by ID
    /// @param tsFn     Timestamp callback (microseconds)
    /// @param sink     Output sink for binary records
    void configure(const DatalogConfig& config,
                   ReadEntryFn readFn,
                   TimestampFn tsFn,
                   DatalogSinkFn sink);

    /// Sample all configured entries once and write a record.
    void sampleOnce();

    /// Start recording.
    void start();

    /// Stop recording.
    void stop();

    /// Get current metadata.
    const DatalogMetadata& metadata() const { return metadata_; }

    /// Get current status.
    DatalogStatus status() const;

private:
    DatalogConfig   config_;
    DatalogMetadata metadata_;
    DatalogState    state_ = DatalogState::Idle;
    ReadEntryFn     readFn_;
    TimestampFn     tsFn_ = nullptr;
    DatalogSinkFn   sink_;
    std::vector<uint8_t> recordBuf_;
    uint64_t        recordsWritten_ = 0;
    uint64_t        bytesWritten_ = 0;
};

}} // namespace tether::io
