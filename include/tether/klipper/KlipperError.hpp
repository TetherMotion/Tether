/**
 * @file KlipperError.hpp
 * @brief Error types for std::expected-based error handling in the Klipper module.
 *
 * @details
 * This header defines a unified error type `KlipperError` that is used with
 * `std::expected<T, KlipperError>` for fallible operations across the Klipper
 * module. It replaces the ad-hoc bool-return / silent-failure patterns with
 * a structured error that carries:
 *   - An error category (enum)
 *   - A human-readable message
 *   - An optional error code
 *
 * Migration is incremental: new overloads return `std::expected<T, KlipperError>`,
 * while existing bool-returning functions remain for backward compatibility.
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace tether::klipper {

/// @brief Error category for Klipper module operations.
enum class ErrorCategory : uint8_t {
    /// Transport I/O error (open, read, write, connection failure).
    Transport,
    /// Protocol error (malformed message, unknown command, CRC mismatch).
    Protocol,
    /// Data dictionary error (parse failure, unknown format string).
    Dictionary,
    /// Configuration error (file not found, parse error, invalid value).
    Config,
    /// Clock synchronization error (insufficient samples, convergence failure).
    ClockSync,
    /// G-code execution error (unknown command, invalid parameters).
    Gcode,
    /// UDS server error (endpoint not found, serialization failure).
    Uds,
    /// Device error (shutdown, not finalized, OID allocation failure).
    Device,
    /// Timeout error (operation did not complete in time).
    Timeout,
    /// Generic/unknown error.
    Unknown,
};

/// @brief Convert an error category to a string name.
inline std::string_view categoryToString(ErrorCategory c) {
    switch (c) {
        case ErrorCategory::Transport:   return "Transport";
        case ErrorCategory::Protocol:    return "Protocol";
        case ErrorCategory::Dictionary:  return "Dictionary";
        case ErrorCategory::Config:      return "Config";
        case ErrorCategory::ClockSync:   return "ClockSync";
        case ErrorCategory::Gcode:       return "Gcode";
        case ErrorCategory::Uds:         return "Uds";
        case ErrorCategory::Device:      return "Device";
        case ErrorCategory::Timeout:     return "Timeout";
        case ErrorCategory::Unknown:     return "Unknown";
    }
    return "Unknown";
}

/// @brief A structured error for the Klipper module.
///
/// Used as the error type in `std::expected<T, KlipperError>`.
/// Carries a category, message, and optional numeric code.
class KlipperError {
public:
    KlipperError() = default;

    KlipperError(ErrorCategory cat, std::string msg, int32_t code = 0)
        : category_(cat), message_(std::move(msg)), code_(code) {}

    /// @brief Construct a transport error.
    static KlipperError transport(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Transport, std::move(msg), code};
    }

    /// @brief Construct a protocol error.
    static KlipperError protocol(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Protocol, std::move(msg), code};
    }

    /// @brief Construct a dictionary error.
    static KlipperError dictionary(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Dictionary, std::move(msg), code};
    }

    /// @brief Construct a config error.
    static KlipperError config(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Config, std::move(msg), code};
    }

    /// @brief Construct a clock sync error.
    static KlipperError clockSync(std::string msg, int32_t code = 0) {
        return {ErrorCategory::ClockSync, std::move(msg), code};
    }

    /// @brief Construct a G-code error.
    static KlipperError gcode(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Gcode, std::move(msg), code};
    }

    /// @brief Construct a UDS error.
    static KlipperError uds(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Uds, std::move(msg), code};
    }

    /// @brief Construct a device error.
    static KlipperError device(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Device, std::move(msg), code};
    }

    /// @brief Construct a timeout error.
    static KlipperError timeout(std::string msg, int32_t code = 0) {
        return {ErrorCategory::Timeout, std::move(msg), code};
    }

    /// @return The error category.
    ErrorCategory category() const { return category_; }

    /// @return The human-readable error message.
    const std::string& message() const { return message_; }

    /// @return The numeric error code (0 if not set).
    int32_t code() const { return code_; }

    /// @return A formatted string: "Category: message (code)".
    std::string format() const {
        std::string s;
        s += std::string(categoryToString(category_));
        s += ": ";
        s += message_;
        if (code_ != 0) {
            s += " (code=";
            s += std::to_string(code_);
            s += ")";
        }
        return s;
    }

    /// @brief Check if this is a transport error.
    bool isTransport() const { return category_ == ErrorCategory::Transport; }

    /// @brief Check if this is a protocol error.
    bool isProtocol() const { return category_ == ErrorCategory::Protocol; }

    /// @brief Check if this is a timeout error.
    bool isTimeout() const { return category_ == ErrorCategory::Timeout; }

private:
    ErrorCategory category_ = ErrorCategory::Unknown;
    std::string message_;
    int32_t code_ = 0;
};

} // namespace tether::klipper
