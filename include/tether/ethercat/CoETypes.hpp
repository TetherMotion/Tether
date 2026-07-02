/**
 * @file CoETypes.hpp
 * @brief CoE (CANopen over EtherCAT) mailbox transaction types
 *
 * Provides the core data types for the CoEManager async mailbox system:
 * - CoEError: transaction error codes
 * - CoETransactionOptions: per-transaction timing and priority
 * - CoETransaction<T>: a single queued CoE read/write operation
 * - CoEResult<T>: std::expected<T, CoEError> alias
 * - SDOAbortCode: standard CoE SDO abort codes
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <expected>
#include <future>
#include <memory>
#include <vector>
#include <chrono>

namespace EtherCAT {
namespace CoE {

// ============================================================================
// SDO Abort Codes (CoE specification)
// ============================================================================

enum class SDOAbortCode : uint32_t {
    Success                  = 0x00000000,
    ToggleBitNotChanged      = 0x05030000,
    Timeout                  = 0x05040000,
    InvalidCommand           = 0x05040001,
    InvalidBlockSize         = 0x05040002,
    InvalidSequenceNumber    = 0x05040003,
    CrcError                 = 0x05040004,
    OutOfMemory              = 0x05040005,
    UnsupportedAccess        = 0x06010000,
    ReadOnlyObject           = 0x06010001,
    WriteOnlyObject          = 0x06010002,
    ObjectNotFound           = 0x06020000,
    PdoMappingError          = 0x06040041,
    PdoLengthExceeded        = 0x06040042,
    ParameterIncompatible    = 0x06040043,
    InternalError            = 0x06040047,
    HardwareError            = 0x06060000,
    DataTypeMismatch         = 0x06070010,
    DataTypeTooLong          = 0x06070012,
    DataTypeTooShort         = 0x06070013,
    SubindexNotFound         = 0x06090011,
    InvalidValue             = 0x06090030,
    ValueTooHigh             = 0x06090031,
    ValueTooLow              = 0x06090032,
    MaxLessThanMin           = 0x06090036,
    ResourceNotAvailable     = 0x060A0023,
    GeneralError             = 0x08000000,
    TransferAborted          = 0x08000020,
    LocalControlError        = 0x08000021,
    DeviceStateError         = 0x08000022,
    DictionaryNotPresent     = 0x08000023,
    NoDataAvailable          = 0x08000024,
};

const char* sdoAbortCodeStr(SDOAbortCode code);

// ============================================================================
// CoE Error Codes
// ============================================================================

enum class CoEError : uint8_t {
    Ok = 0,
    Timeout,
    Aborted,
    TransportError,
    QueueFull,
    NotConfigured,
    SlaveNotFound,
    InternalError,
};

const char* coeErrorStr(CoEError error);

// ============================================================================
// CoE Transaction Options
// ============================================================================

struct CoETransactionOptions {
    uint32_t poll_interval_ms = 5;
    uint32_t timeout_ms = 1000;
    uint8_t priority = 0;
    uint8_t max_retries = 3;
};

struct BehaviourOptions {
    bool request_al_status_after_coe_requests = false;
};

// ============================================================================
// CoE Transaction (base for queue entries)
// ============================================================================

struct CoETransactionBase {
    uint16_t index = 0;
    uint8_t subindex = 0;
    CoETransactionOptions options{};
    std::chrono::steady_clock::time_point enqueue_time{};

    bool operator<(const CoETransactionBase& other) const {
        if (options.priority != other.options.priority) {
            return options.priority < other.options.priority;
        }
        return enqueue_time > other.enqueue_time;
    }
};

// ============================================================================
// CoE Read Transaction
// ============================================================================

template<typename T>
struct CoEReadTransaction : CoETransactionBase {
    std::promise<std::expected<T, CoEError>> promise;
};

// ============================================================================
// CoE Write Transaction
// ============================================================================

struct CoEWriteTransaction : CoETransactionBase {
    std::vector<uint8_t> data;
    std::promise<std::expected<void, CoEError>> promise;
};

// ============================================================================
// CoE Result alias
// ============================================================================

template<typename T>
using CoEResult = std::expected<T, CoEError>;

} // namespace CoE
} // namespace EtherCAT
