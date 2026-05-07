/**
 * @file Protocol.hpp
 * @brief Tether IO Protocol — wire format definitions, value types, and buffer utilities.
 *
 * This header defines all message types, value types, trigger modes, error codes,
 * threshold types, feature descriptors, and lightweight binary buffer reader/writer
 * utilities used by the tether_io_protocol module.
 *
 * ## Design overview
 *
 * The protocol separates two first-class entities:
 *  - **Parameters** — values that can be read, written, and streamed in both
 *    directions (get / set / stream-get / stream-set).
 *  - **Signals** — values that are read-only from the client perspective
 *    (get / stream-get only).
 *
 * Wire format conventions:
 *  - All multi-byte integers are **little-endian**.
 *  - Each SLIP packet carries exactly one protocol message.
 *  - The first byte of every message is the `MessageType` discriminator.
 *  - Parameter/signal counts are 4 bytes (uint32) throughout.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace tether { namespace io {

// ---------------------------------------------------------------------------
// Protocol version
// ---------------------------------------------------------------------------
inline constexpr uint8_t PROTOCOL_VERSION = 1;

/// Default TCP port for the tether IO protocol server
inline constexpr uint16_t DEFAULT_PORT = 4000;

// ---------------------------------------------------------------------------
// Message types (first byte of every SLIP payload)
// ---------------------------------------------------------------------------
enum class MessageType : uint8_t {
    // --- Parameter catalog ---
    ListParamsReq       = 0x01, ///< Client → Server: request parameter catalog page
    ListParamsResp      = 0x02, ///< Server → Client: parameter catalog page

    // --- Signal catalog ---
    ListSignalsReq      = 0x03, ///< Client → Server: request signal catalog page
    ListSignalsResp     = 0x04, ///< Server → Client: signal catalog page

    // --- Parameter get/set ---
    GetParamReq         = 0x05, ///< Client → Server: read one parameter value
    GetParamResp        = 0x06, ///< Server → Client: parameter value
    SetParamReq         = 0x07, ///< Client → Server: write one parameter value
    SetParamResp        = 0x08, ///< Server → Client: acknowledge parameter write

    // --- Signal get ---
    GetSignalReq        = 0x09, ///< Client → Server: read one signal value
    GetSignalResp       = 0x0A, ///< Server → Client: signal value

    // --- Streaming ---
    ConfigureStreamReq  = 0x0B, ///< Client → Server: define stream spec (params + signals)
    ConfigureStreamAck  = 0x0C, ///< Server → Client: acknowledge stream config
    StartStream         = 0x0D, ///< Client → Server: begin streaming
    StopStream          = 0x0E, ///< Client → Server: stop streaming
    StreamData          = 0x0F, ///< Server → Client: timestamped data rows

    // --- Error ---
    Error               = 0x10, ///< Server → Client: error response

    // --- Metadata ---
    GetMetadataReq      = 0x11, ///< Client → Server: request metadata for a param or signal
    GetMetadataResp     = 0x12, ///< Server → Client: metadata key/value pairs

    // --- Snapshots ---
    SnapshotParamsReq   = 0x13, ///< Client → Server: snapshot one or more params
    SnapshotParamsResp  = 0x14, ///< Server → Client: snapshot results (params)
    SnapshotSignalsReq  = 0x15, ///< Client → Server: snapshot one or more signals
    SnapshotSignalsResp = 0x16, ///< Server → Client: snapshot results (signals)

    // --- Feature exchange ---
    FeatureExchangeReq  = 0x17, ///< Client → Server: list client features
    FeatureExchangeResp = 0x18, ///< Server → Client: list server features

    // --- Catalog change notification ---
    CatalogChanged      = 0x19, ///< Server → Client: available params/signals changed

    // --- Datalogging ---
    ConfigureDatalogReq = 0x1A, ///< Client → Server: configure datalogging
    ConfigureDatalogResp= 0x1B, ///< Server → Client: acknowledge datalog config
    DatalogStatusReq    = 0x1C, ///< Client → Server: query datalog state
    DatalogStatusResp   = 0x1D, ///< Server → Client: datalog state

    // --- Threshold configuration ---
    ConfigureThresholdReq  = 0x1E, ///< Client → Server: configure threshold filter
    ConfigureThresholdResp = 0x1F, ///< Server → Client: acknowledge threshold config

    // --- Binary struct description ---
    DescribeStructReq   = 0x20, ///< Client → Server: request struct layout for a param/signal
    DescribeStructResp  = 0x21, ///< Server → Client: struct field descriptions
};

// ---------------------------------------------------------------------------
// Value type identifiers
// ---------------------------------------------------------------------------
enum class ValueType : uint8_t {
    U8      = 1,
    U16     = 2,
    U32     = 3,
    U64     = 4,
    I8      = 5,
    I16     = 6,
    I32     = 7,
    I64     = 8,
    F32     = 9,
    F64     = 10,
    Bool    = 11,
    String  = 12,
    Binary  = 13,
    Struct  = 14,   ///< Composite binary struct (described by DescribeStructResp)
    Enum    = 15,   ///< Enum value (integer with named labels)
};

/// Returns true for variable-length types (String, Binary).
inline constexpr bool isVariableLength(ValueType t) {
    return t == ValueType::String || t == ValueType::Binary;
}

/// Returns the byte size of a fixed-size ValueType, or 0 for variable-length/unknown.
inline constexpr uint8_t valueTypeSize(ValueType t) {
    switch (t) {
        case ValueType::U8:   case ValueType::I8:  case ValueType::Bool: return 1;
        case ValueType::U16:  case ValueType::I16:  return 2;
        case ValueType::U32:  case ValueType::I32:  case ValueType::F32: case ValueType::Enum: return 4;
        case ValueType::U64:  case ValueType::I64:  case ValueType::F64: return 8;
        case ValueType::String: case ValueType::Binary: case ValueType::Struct: return 0;
        default: return 0;
    }
}

/// Returns a human-readable name for a ValueType.
inline const char* valueTypeName(ValueType t) {
    switch (t) {
        case ValueType::U8:     return "u8";
        case ValueType::U16:    return "u16";
        case ValueType::U32:    return "u32";
        case ValueType::U64:    return "u64";
        case ValueType::I8:     return "i8";
        case ValueType::I16:    return "i16";
        case ValueType::I32:    return "i32";
        case ValueType::I64:    return "i64";
        case ValueType::F32:    return "f32";
        case ValueType::F64:    return "f64";
        case ValueType::Bool:   return "bool";
        case ValueType::String: return "string";
        case ValueType::Binary: return "binary";
        case ValueType::Struct: return "struct";
        case ValueType::Enum:   return "enum";
        default: return "unknown";
    }
}

// ---------------------------------------------------------------------------
// Entry kind (parameter vs signal)
// ---------------------------------------------------------------------------
enum class EntryKind : uint8_t {
    Parameter = 0,
    Signal    = 1,
};

// ---------------------------------------------------------------------------
// Stream trigger modes
// ---------------------------------------------------------------------------
enum class TriggerMode : uint8_t {
    Time     = 0,  ///< Periodic sampling at interval_us
    OnChange = 1,  ///< Trigger when any sampled entry value changes
};

// ---------------------------------------------------------------------------
// Threshold types for change detection / skip filtering
// ---------------------------------------------------------------------------
enum class ThresholdType : uint8_t {
    None     = 0,  ///< No threshold — always send
    Absolute = 1,  ///< Send when |new − old| > threshold
    Relative = 2,  ///< Send when |new − old| / |old| > threshold
    Custom   = 3,  ///< Implementation-defined logic (name + config primitives)
};

// ---------------------------------------------------------------------------
// Datalogging state
// ---------------------------------------------------------------------------
enum class DatalogState : uint8_t {
    Idle      = 0,
    Recording = 1,
    Stopped   = 2,
    Error     = 3,
};

// ---------------------------------------------------------------------------
// Error codes (used in Error messages)
// ---------------------------------------------------------------------------
enum class ErrorCode : uint32_t {
    None                = 0,
    InvalidMessage      = 1,
    UnknownMessageType  = 2,
    InvalidId           = 3,
    StreamNotConfigured = 4,
    AlreadyStreaming     = 5,
    NotStreaming         = 6,
    TooManyEntries      = 7,
    InternalError       = 8,
    NotWritable         = 9,
    FeatureNotSupported = 10,
    DatalogError        = 11,
    ThresholdError      = 12,
};

// ---------------------------------------------------------------------------
// Entry flags
// ---------------------------------------------------------------------------
namespace EntryFlags {
    inline constexpr uint8_t Readable      = 0x01;
    inline constexpr uint8_t Writable      = 0x02;
    inline constexpr uint8_t VariableLen   = 0x04;
    inline constexpr uint8_t HasStruct     = 0x08;
    inline constexpr uint8_t HasEnum       = 0x10;
} // namespace EntryFlags

// ---------------------------------------------------------------------------
// Varint encoding / decoding (protobuf-style, 7-bit groups, LSB first)
// ---------------------------------------------------------------------------

inline constexpr size_t MAX_VARINT_SIZE = 5;

/// Encode a uint32_t as a varint into buf (max 5 bytes). Returns bytes written, 0 on overflow.
inline size_t encodeVarint(uint8_t* buf, size_t cap, uint32_t value) {
    size_t pos = 0;
    while (value >= 0x80) {
        if (pos >= cap) return 0;
        buf[pos++] = static_cast<uint8_t>(value & 0x7F) | 0x80;
        value >>= 7;
    }
    if (pos >= cap) return 0;
    buf[pos++] = static_cast<uint8_t>(value);
    return pos;
}

/// Decode a varint from buf. Returns bytes consumed, 0 on error.
inline size_t decodeVarint(const uint8_t* buf, size_t len, uint32_t& value) {
    value = 0;
    uint32_t shift = 0;
    for (size_t i = 0; i < len && i < 5; ++i) {
        value |= static_cast<uint32_t>(buf[i] & 0x7F) << shift;
        shift += 7;
        if ((buf[i] & 0x80) == 0) return i + 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Lightweight binary buffer writer (no allocation, bounds-checked)
// ---------------------------------------------------------------------------
struct BufWriter {
    uint8_t* buf;
    size_t   pos;
    size_t   cap;
    bool     overflow;

    BufWriter(uint8_t* b, size_t c) : buf(b), pos(0), cap(c), overflow(false) {}

    bool ok() const { return !overflow; }

    void putU8(uint8_t v) {
        if (pos + 1 > cap) { overflow = true; return; }
        buf[pos++] = v;
    }
    void putU16(uint16_t v) {
        if (pos + 2 > cap) { overflow = true; return; }
        std::memcpy(buf + pos, &v, 2); pos += 2;
    }
    void putU32(uint32_t v) {
        if (pos + 4 > cap) { overflow = true; return; }
        std::memcpy(buf + pos, &v, 4); pos += 4;
    }
    void putU64(uint64_t v) {
        if (pos + 8 > cap) { overflow = true; return; }
        std::memcpy(buf + pos, &v, 8); pos += 8;
    }
    void putI32(int32_t v) {
        uint32_t u;
        std::memcpy(&u, &v, 4);
        putU32(u);
    }
    void putF32(float v) {
        uint32_t u;
        std::memcpy(&u, &v, 4);
        putU32(u);
    }
    void putF64(double v) {
        uint64_t u;
        std::memcpy(&u, &v, 8);
        putU64(u);
    }
    void putBytes(const void* d, size_t n) {
        if (pos + n > cap) { overflow = true; return; }
        std::memcpy(buf + pos, d, n); pos += n;
    }
    /// Write a length-prefixed string (uint16_t length + bytes)
    void putStr16(const char* s, size_t len) {
        putU16(static_cast<uint16_t>(len));
        putBytes(s, len);
    }
    /// Write a varint-encoded uint32_t.
    void putVarint(uint32_t v) {
        uint8_t tmp[MAX_VARINT_SIZE];
        size_t n = encodeVarint(tmp, sizeof(tmp), v);
        putBytes(tmp, n);
    }
};

// ---------------------------------------------------------------------------
// Lightweight binary buffer reader (bounds-checked)
// ---------------------------------------------------------------------------
struct BufReader {
    const uint8_t* buf;
    size_t         pos;
    size_t         len;
    bool           error;

    BufReader(const uint8_t* b, size_t l) : buf(b), pos(0), len(l), error(false) {}

    bool ok() const { return !error; }
    size_t remaining() const { return (pos <= len) ? (len - pos) : 0; }

    uint8_t getU8() {
        if (pos + 1 > len) { error = true; return 0; }
        return buf[pos++];
    }
    uint16_t getU16() {
        if (pos + 2 > len) { error = true; return 0; }
        uint16_t v; std::memcpy(&v, buf + pos, 2); pos += 2; return v;
    }
    uint32_t getU32() {
        if (pos + 4 > len) { error = true; return 0; }
        uint32_t v; std::memcpy(&v, buf + pos, 4); pos += 4; return v;
    }
    uint64_t getU64() {
        if (pos + 8 > len) { error = true; return 0; }
        uint64_t v; std::memcpy(&v, buf + pos, 8); pos += 8; return v;
    }
    int32_t getI32() {
        uint32_t u = getU32();
        int32_t v;
        std::memcpy(&v, &u, 4);
        return v;
    }
    float getF32() {
        uint32_t u = getU32();
        float v;
        std::memcpy(&v, &u, 4);
        return v;
    }
    double getF64() {
        uint64_t u = getU64();
        double v;
        std::memcpy(&v, &u, 8);
        return v;
    }
    const uint8_t* getBytes(size_t n) {
        if (pos + n > len) { error = true; return nullptr; }
        const uint8_t* p = buf + pos; pos += n; return p;
    }
    bool skip(size_t n) {
        if (pos + n > len) { error = true; return false; }
        pos += n; return true;
    }
    uint32_t getVarint() {
        uint32_t value = 0;
        uint32_t shift = 0;
        for (int i = 0; i < 5; ++i) {
            uint8_t b = getU8();
            if (error) return 0;
            value |= static_cast<uint32_t>(b & 0x7F) << shift;
            shift += 7;
            if ((b & 0x80) == 0) return value;
        }
        error = true;
        return 0;
    }
};

}} // namespace tether::io
