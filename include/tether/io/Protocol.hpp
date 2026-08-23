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
 *  - Parameter/signal counts are 4 bytes (uint32) throughout. This intentionally
 *    uses the larger Tether limits where the original ParameterStream format
 *    used uint16_t counts.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <limits>
#include <magic_enum/magic_enum.hpp>

namespace tether { namespace io {

// ---------------------------------------------------------------------------
// Protocol version
// ---------------------------------------------------------------------------
// Version 5 is the Tether merge revision. It uses ParameterStreamProtocol's
// message/value IDs but intentionally widens its count fields to uint32_t and
// retains Tether catalog extensions.
inline constexpr uint8_t PROTOCOL_VERSION = 5;

/// Default TCP port for the tether IO protocol server
inline constexpr uint16_t DEFAULT_PORT = 4000;

// ---------------------------------------------------------------------------
// Message types (first byte of every SLIP payload)
// ---------------------------------------------------------------------------
enum class MessageType : uint8_t {
    // ParameterStreamProtocol v4 wire IDs. Keep these values stable.
    ListParamsReq       = 0x01,
    ListParamsResp      = 0x02,
    ConfigureStream     = 0x03,
    ConfigureAck        = 0x04,
    StartStream         = 0x05,
    StopStream          = 0x06,
    StreamData          = 0x07,
    Error               = 0x08,
    GetMetadataReq      = 0x09,
    GetMetadataResp     = 0x0A,
    SetParameterReq     = 0x0B,
    SetParameterResp    = 0x0C,
    PingReq             = 0x0D,
    PongResp            = 0x0E,
    SubscribeLogReq     = 0x0F,
    SubscribeLogResp    = 0x10,
    UnsubscribeLogReq   = 0x11,
    UnsubscribeLogResp  = 0x12,
    LogData             = 0x13,

    // Tether extensions. They deliberately start outside the v4 namespace.
    ListSignalsReq      = 0x20,
    ListSignalsResp     = 0x21,
    GetParamReq         = 0x22,
    GetParamResp        = 0x23,
    GetSignalReq        = 0x24,
    GetSignalResp       = 0x25,
    SnapshotParamsReq   = 0x26,
    SnapshotParamsResp  = 0x27,
    SnapshotSignalsReq  = 0x28,
    SnapshotSignalsResp = 0x29,
    FeatureExchangeReq  = 0x2A,
    FeatureExchangeResp = 0x2B,
    CatalogChanged      = 0x2C,
    ConfigureDatalogReq = 0x2D,
    ConfigureDatalogResp= 0x2E,
    DatalogStatusReq    = 0x2F,
    DatalogStatusResp   = 0x30,
    ConfigureThresholdReq  = 0x31,
    ConfigureThresholdResp = 0x32,
    DescribeStructReq   = 0x33,
    DescribeStructResp  = 0x34,
    ListFunctionsReq    = 0x35,
    ListFunctionsResp   = 0x36,
    CallFunctionReq     = 0x37,
    CallFunctionResp    = 0x38,
    CreateInputStreamReq  = 0x39,
    CreateInputStreamResp = 0x3A,
    InputStreamData       = 0x3B,
    CloseInputStreamReq   = 0x3C,
    CloseInputStreamResp  = 0x3D,

    // Source-compatible Tether names for the canonical v4 operations.
    ConfigureStreamReq  = ConfigureStream,
    ConfigureStreamAck  = ConfigureAck,
    SetParamReq         = SetParameterReq,
    SetParamResp        = SetParameterResp,
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
    IPv4    = 14,
    IPv6    = 15,
    MAC     = 16,
    Enum    = 17,
    UVarint = 18,
    IVarint = 19,
    Struct  = 20,   ///< Tether extension: composite binary struct
    Array   = 21,   ///< Length-prefixed sequence of values
    Stream  = 22,   ///< U32 stream handle
};

/// Returns true for variable-length types in the canonical v4 value namespace.
inline constexpr bool isVariableLength(ValueType t) {
    return t == ValueType::String || t == ValueType::Binary ||
           t == ValueType::Enum || t == ValueType::UVarint ||
           t == ValueType::IVarint || t == ValueType::Struct ||
           t == ValueType::Array;
}

/// Returns the byte size of a fixed-size ValueType, or 0 for variable-length/unknown.
inline constexpr uint8_t valueTypeSize(ValueType t) {
    switch (t) {
        case ValueType::U8:   case ValueType::I8:  case ValueType::Bool: return 1;
        case ValueType::U16:  case ValueType::I16:  return 2;
        case ValueType::U32:  case ValueType::I32:  case ValueType::F32: return 4;
        case ValueType::U64:  case ValueType::I64:  case ValueType::F64: return 8;
        case ValueType::IPv4: return 4;
        case ValueType::IPv6: return 16;
        case ValueType::MAC: return 6;
        case ValueType::String: case ValueType::Binary: case ValueType::Enum:
            case ValueType::UVarint: case ValueType::IVarint: case ValueType::Struct:
            case ValueType::Array: return 0;
            case ValueType::Stream: return 4;
        default: return 0;
    }
}

// ---------------------------------------------------------------------------
// Entry kind (parameter vs signal)
// ---------------------------------------------------------------------------
enum class EntryKind : uint8_t {
    Parameter = 0,
    Signal    = 1,
    Function  = 2,
};

// ---------------------------------------------------------------------------
// Stream trigger modes
// ---------------------------------------------------------------------------
enum class TriggerMode : uint8_t {
    Time     = 0,  ///< Periodic sampling at interval_ms on the wire
    OnChange = 1,  ///< Trigger when the configured trigger entry changes
};

// ---------------------------------------------------------------------------
// Log streaming
// ---------------------------------------------------------------------------
enum class LogSeverity : uint8_t {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Critical = 5,
};

/// A ParameterStream-compatible log subscription.
struct LogSubscription {
    uint32_t id = 0;
    LogSeverity minSeverity = LogSeverity::Trace;
    std::string componentFilter;
    std::string messageFilter;
    std::string locationFilter;
};

/// A log record delivered to matching subscriptions.
struct LogRecord {
    uint64_t timestampUs = 0;
    LogSeverity severity = LogSeverity::Info;
    std::string component;
    std::string message;
    std::string location;
};

enum class FilterPropertyErrorType : uint8_t {
    ParameterUnknown = 0,
    NotImplemented = 1,
    WrongDataType = 2,
    OutsideRange = 3,
    InvalidValue = 4,
    Other = 5,
    None = 6,
};

struct FilterPropertyValue {
    ValueType type = ValueType::Binary;
    std::vector<uint8_t> data;
};

struct FilterProperty {
    std::string name;
    FilterPropertyValue value;
};

struct FilterPropertyDef {
    std::string name;
    ValueType valueType = ValueType::Binary;
    bool implemented = true;
    bool hasRange = false;
    double minValue = 0.0;
    double maxValue = 0.0;
};

class StreamFilterSchema {
public:
    void defineProperty(FilterPropertyDef definition);

    struct Result {
        bool ok = false;
        FilterPropertyErrorType errorType = FilterPropertyErrorType::Other;
        std::string message;
    };

    Result validate(const FilterProperty& property) const;
    bool empty() const;

private:
    std::map<std::string, FilterPropertyDef> definitions_;
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
    InvalidParameterId  = InvalidId,
    StreamNotConfigured = 4,
    AlreadyStreaming     = 5,
    NotStreaming         = 6,
    TooManyEntries      = 7,
    TooManyParameters   = TooManyEntries,
    InternalError       = 8,
    NotWritable         = 9,
    FeatureNotSupported = 10,
    DatalogError        = 11,
    ThresholdError      = 12,
    FunctionInvocationError = 13,
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
inline constexpr size_t MAX_MESSAGE_SIZE = 1024 * 1024;
inline constexpr size_t MAX_STRING_SIZE = UINT16_MAX;
inline constexpr size_t MAX_VARIABLE_VALUE_SIZE = 1024 * 1024;
inline constexpr uint32_t MAX_COLLECTION_COUNT = 1'000'000;

inline uint32_t zigzagEncode32(int32_t value) {
    return (static_cast<uint32_t>(value) << 1) ^
           static_cast<uint32_t>(value >> 31);
}

inline int32_t zigzagDecode32(uint32_t value) {
    return static_cast<int32_t>((value >> 1) ^ -(value & 1));
}

inline uint64_t zigzagEncode64(int64_t value) {
    return (static_cast<uint64_t>(value) << 1) ^
           static_cast<uint64_t>(value >> 63);
}

inline int64_t zigzagDecode64(uint64_t value) {
    return static_cast<int64_t>((value >> 1) ^ -(value & 1));
}

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
        if (i == 4 && (buf[i] & 0x7F) > 0x0F) return 0;
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
        if (pos >= cap) { overflow = true; return; }
        buf[pos++] = v;
    }
    void putU16(uint16_t v) {
        if (pos > cap || 2 > cap - pos) { overflow = true; return; }
        buf[pos++] = static_cast<uint8_t>(v);
        buf[pos++] = static_cast<uint8_t>(v >> 8);
    }
    void putU32(uint32_t v) {
        if (pos > cap || 4 > cap - pos) { overflow = true; return; }
        for (unsigned i = 0; i < 4; ++i) buf[pos++] = static_cast<uint8_t>(v >> (8 * i));
    }
    void putU64(uint64_t v) {
        if (pos > cap || 8 > cap - pos) { overflow = true; return; }
        for (unsigned i = 0; i < 8; ++i) buf[pos++] = static_cast<uint8_t>(v >> (8 * i));
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
        if (n != 0 && d == nullptr) { overflow = true; return; }
        if (pos > cap || n > cap - pos) { overflow = true; return; }
        std::memcpy(buf + pos, d, n); pos += n;
    }
    /// Write a length-prefixed string (uint16_t length + bytes)
    void putStr16(const char* s, size_t len) {
        if (len > UINT16_MAX) { overflow = true; return; }
        putU16(static_cast<uint16_t>(len));
        putBytes(s, len);
    }
    /// Write a varint-encoded uint32_t.
    void putVarint(uint32_t v) {
        uint8_t tmp[MAX_VARINT_SIZE];
        size_t n = encodeVarint(tmp, sizeof(tmp), v);
        if (n == 0) { overflow = true; return; }
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
        if (pos >= len) { error = true; return 0; }
        return buf[pos++];
    }
    uint16_t getU16() {
        if (pos > len || 2 > len - pos) { error = true; return 0; }
        uint16_t v = static_cast<uint16_t>(buf[pos]) |
                     (static_cast<uint16_t>(buf[pos + 1]) << 8);
        pos += 2; return v;
    }
    uint32_t getU32() {
        if (pos > len || 4 > len - pos) { error = true; return 0; }
        uint32_t v = 0;
        for (unsigned i = 0; i < 4; ++i) v |= static_cast<uint32_t>(buf[pos + i]) << (8 * i);
        pos += 4; return v;
    }
    uint64_t getU64() {
        if (pos > len || 8 > len - pos) { error = true; return 0; }
        uint64_t v = 0;
        for (unsigned i = 0; i < 8; ++i) v |= static_cast<uint64_t>(buf[pos + i]) << (8 * i);
        pos += 8; return v;
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
        if (pos > len || n > len - pos) { error = true; return nullptr; }
        const uint8_t* p = buf + pos; pos += n; return p;
    }
    bool skip(size_t n) {
        if (pos > len || n > len - pos) { error = true; return false; }
        pos += n; return true;
    }
    uint32_t getVarint() {
        uint32_t value = 0;
        uint32_t shift = 0;
        for (int i = 0; i < 5; ++i) {
            uint8_t b = getU8();
            if (error) return 0;
            if (i == 4 && (b & 0x7F) > 0x0F) { error = true; return 0; }
            value |= static_cast<uint32_t>(b & 0x7F) << shift;
            shift += 7;
            if ((b & 0x80) == 0) return value;
        }
        error = true;
        return 0;
    }
};

}} // namespace tether::io
