/**
 * @file CoETypes.hpp
 * @brief CoE (CANopen over EtherCAT) mailbox transaction types
 *
 * Provides the core data types for the CoEManager async mailbox system:
 * - CoEError: transaction error payload (category + SDO abort detail)
 * - CoETransactionOptions: per-transaction timing and priority
 * - CoETransaction<T>: a single queued CoE read/write operation
 * - CoEResult<T>: std::expected<T, CoEError> alias
 *
 * SDOAbortCode itself lives in tether/ethercat/SDOAbortCodes.hpp at
 * EtherCAT scope (shared with the SDO layer and slave emulation).
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <expected>
#include <future>
#include <memory>
#include <vector>
#include <chrono>

#include "tether/ethercat/SDOAbortCodes.hpp"

namespace EtherCAT {
namespace CoE {

// ============================================================================
// CoE Error
// ============================================================================

/**
 * @brief Broad failure category of a CoE transaction.
 */
enum class CoEErrorCode : uint8_t {
    Ok = 0,
    Timeout,
    Aborted,          ///< Slave replied with an SDO abort — see abort/abort_code
    TransportError,
    QueueFull,
    NotConfigured,
    ShuttingDown,
    SlaveNotFound,
    InternalError,
};

/**
 * @brief Error payload carried in CoEResult's unexpected value.
 *
 * `code` is the broad category.  When `code == CoEErrorCode::Aborted`,
 * `abort`/`abort_code` carry the slave's SDO abort: `abort` is the
 * classified enum (e.g. SDOAbortCode::ObjectNotFound — it may hold a
 * vendor-specific value not in the standard table) and `abort_code` is
 * the raw 32-bit wire value for logging.
 */
struct CoEError {
    CoEErrorCode code      = CoEErrorCode::Ok;
    SDOAbortCode abort     = SDOAbortCode::Success;
    uint32_t     abort_code = 0;

    constexpr CoEError() = default;
    /// Implicit: a bare category means "no abort detail".
    constexpr CoEError(CoEErrorCode c) : code(c) {}
    constexpr CoEError(CoEErrorCode c, SDOAbortCode a, uint32_t raw)
        : code(c), abort(a), abort_code(raw) {}

    /// Build an Aborted error carrying the slave's raw abort code.
    static constexpr CoEError aborted(uint32_t raw) {
        return CoEError(CoEErrorCode::Aborted, static_cast<SDOAbortCode>(raw), raw);
    }

    /// Compare against a bare category: `err == CoEErrorCode::Timeout`.
    constexpr bool operator==(CoEErrorCode c) const { return code == c; }
    bool operator==(const CoEError&) const = default;
};

/**
 * @brief Human-readable description of a CoE error.
 *
 * For aborts this returns the decoded abort string (e.g. "Object does not
 * exist") so callers can print one informative line with abort_code.
 */
const char* coeErrorStr(CoEError error);

// ============================================================================
// CoE Transaction Options
// ============================================================================

struct CoETransactionOptions {
    uint32_t poll_interval_ms = 5;
    uint32_t timeout_ms = 1000;
    uint8_t priority = 0;
    uint8_t max_retries = 3;
    bool complete_access = false;
    /// @brief When true, suppress the "trailing bytes discarded" warning
    /// emitted when a typed read (e.g. readU8) returns more bytes than the
    /// requested C++ type. Set this when the caller knowingly reads a narrow
    /// value from a wider SDO entry (e.g. a 1-byte module ID stored in a
    /// 4-byte OD entry). The leading sizeof(T) bytes are still copied
    /// regardless; this flag only silences the warning. The hard error path
    /// for short reads (out_len < sizeof(T)) is unaffected.
    bool allow_trailing_bytes = false;
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
