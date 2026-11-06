/**
 * @file ModbusError.hpp
 * @brief Structured error type for the tether::modbus component.
 *
 * All fallible public APIs return `std::expected<T, ModbusError>` (aliased
 * as `Expected<T>`). Categories mirror the layering in MODBUS-PLAN §5:
 * Transport (link I/O), TimedOut, Codec (PDU/framing), Config, Modbus
 * (peer exception response), Busy, Cancelled, Assurance (robust layer).
 */
#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <utility>

namespace tether::modbus {

/**@brief Modbus error category. */
enum class ErrorCategory : uint8_t {
    Transport,   ///< link I/O error (open/read/write/connect)
    TimedOut,    ///< operation deadline elapsed
    Codec,       ///< PDU/framing encode/decode error
    Config,      ///< invalid configuration
    Modbus,      ///< peer returned a Modbus exception response
    Busy,        ///< queue/executor busy; try later
    Cancelled,   ///< operation cancelled by caller/shutdown
    Assurance,   ///< robust-strategy verdict (mismatch, quorum, confirm)
    Unsupported, ///< feature/function not supported
    Corrupt,     ///< data arrived but failed integrity check (CRC/mark)
};

inline std::string_view categoryToString(ErrorCategory c) {
    switch (c) {
        case ErrorCategory::Transport:   return "Transport";
        case ErrorCategory::TimedOut:    return "TimedOut";
        case ErrorCategory::Codec:       return "Codec";
        case ErrorCategory::Config:      return "Config";
        case ErrorCategory::Modbus:      return "Modbus";
        case ErrorCategory::Busy:        return "Busy";
        case ErrorCategory::Cancelled:   return "Cancelled";
        case ErrorCategory::Assurance:   return "Assurance";
        case ErrorCategory::Unsupported: return "Unsupported";
        case ErrorCategory::Corrupt:     return "Corrupt";
    }
    return "Unknown";
}

/**@brief Numeric detail codes for ErrorCategory::Assurance / others. */
enum class ErrorCode : int32_t {
    None = 0,
    // ---- Assurance (negative range per plan §27.5) ------------------------
    AssuranceMismatch     = -6001,  ///< samples disagreed
    AssuranceUnstable     = -6002,  ///< UntilStable window exceeded
    ConfirmFailed         = -6003,  ///< readback differs after write
    ConfirmUnavailable    = -6004,  ///< confirm op itself failed
    QuorumUnreachable     = -6005,  ///< <q samples could ever agree
    StrategyNotCompiled   = -6006,  ///< requested strategy not built in
    // ---- generic ---------------------------------------------------------
    Unknown               = -1,
};

/**
 * @brief A structured error for tether::modbus.
 *
 * Carries category, human-readable message, and a numeric detail code
 * (ExceptionCode value for Modbus category; ErrorCode otherwise).
 */
class ModbusError {
public:
    ModbusError() = default;
    ModbusError(ErrorCategory cat, std::string msg, int32_t code = 0)
        : category_(cat), message_(std::move(msg)), code_(code) {}

    static ModbusError transport(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Transport, std::move(msg), code};
    }
    static ModbusError timedOut(std::string msg = "timeout") {
        return {ErrorCategory::TimedOut, std::move(msg)};
    }
    static ModbusError codec(std::string msg) {
        return {ErrorCategory::Codec, std::move(msg)};
    }
    static ModbusError config(std::string msg) {
        return {ErrorCategory::Config, std::move(msg)};
    }
    /// Peer exception response. `code` is the wire ExceptionCode byte.
    static ModbusError exception(uint8_t code, std::string msg) {
        return {ErrorCategory::Modbus, std::move(msg), code};
    }
    static ModbusError busy(std::string msg = "busy") {
        return {ErrorCategory::Busy, std::move(msg)};
    }
    static ModbusError cancelled(std::string msg = "cancelled") {
        return {ErrorCategory::Cancelled, std::move(msg)};
    }
    static ModbusError assurance(ErrorCode detail, std::string msg) {
        return {ErrorCategory::Assurance, std::move(msg),
                static_cast<int32_t>(detail)};
    }
    static ModbusError unsupported(std::string msg) {
        return {ErrorCategory::Unsupported, std::move(msg)};
    }
    /// Frame data arrived but failed integrity validation (bad CRC).
    static ModbusError corrupt(std::string msg) {
        return {ErrorCategory::Corrupt, std::move(msg)};
    }

    ErrorCategory category() const { return category_; }
    const std::string& message() const { return message_; }
    int32_t code() const { return code_; }

    bool isTransport() const { return category_ == ErrorCategory::Transport; }
    bool isTimeout() const { return category_ == ErrorCategory::TimedOut; }
    bool isCodec() const { return category_ == ErrorCategory::Codec; }
    bool isConfig() const { return category_ == ErrorCategory::Config; }
    bool isException() const { return category_ == ErrorCategory::Modbus; }
    bool isBusy() const { return category_ == ErrorCategory::Busy; }
    bool isCancelled() const { return category_ == ErrorCategory::Cancelled; }
    bool isAssurance() const { return category_ == ErrorCategory::Assurance; }
    bool isCorrupt() const { return category_ == ErrorCategory::Corrupt; }

    /// True when a retry could plausibly succeed (transport/timeout/busy/
    /// corrupt — a mangled response is worth a re-read).
    bool retryable() const {
        return category_ == ErrorCategory::Transport ||
               category_ == ErrorCategory::TimedOut ||
               category_ == ErrorCategory::Busy ||
               category_ == ErrorCategory::Corrupt;
    }

    std::string format() const {
        std::string s(categoryToString(category_));
        s += ": " + message_;
        if (code_ != 0) s += " (code=" + std::to_string(code_) + ")";
        return s;
    }
    std::string toString() const { return format(); }

private:
    ErrorCategory category_ = ErrorCategory::Codec;
    std::string   message_;
    int32_t       code_ = 0;
};

/**@brief Expected alias used across the component. */
template <typename T>
using Expected = std::expected<T, ModbusError>;

/**@brief Error-return helper: `return err(ModbusError::codec("x"));` */
template <typename T = void>
inline Expected<T> err(ModbusError e) {
    return std::unexpected(std::move(e));
}

/**@brief Unwrap or propagate — `TETHER_MODBUS_TRY(x)` inside Expected fns. */
#define TETHER_MODBUS_TRY(expr)                          \
    ({ auto _r = (expr);                                 \
       if (!_r) return std::unexpected(_r.error());      \
       std::move(*_r); })

} // namespace tether::modbus
