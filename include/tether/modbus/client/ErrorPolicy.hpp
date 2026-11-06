/**
 * @file ErrorPolicy.hpp
 * @brief User-selectable strategy for reacting to transaction errors
 *        (parity/framing corruption, CRC failures, timeouts, transport
 *        faults, busy exceptions).
 *
 * A corrupted RTU response — e.g. a byte the UART flagged with a parity
 * error — surfaces as `ErrorCategory::Corrupt`; silence as `TimedOut`;
 * link faults as `Transport`. The policy decides what the executor does
 * about it. Default: `Retry` with `maxRetries = 8` (re-issue the
 * transaction up to 8 extra times, then fail).
 *
 * Layering (code-first; INI is an optional thin mapping, see
 * config/Config.cpp) — the connection owns defaults; calls override:
 *   - `BusExecutor::setDefaultOptions(o)` — connection-wide defaults
 *     (`setErrorPolicy(p)` is sugar for `o.errorPolicy = p`).
 *   - `ModbusClient` ctor defaults      — optional per-client layer.
 *   - per-call `TransactionOptions`      — unset fields inherit.
 *   - `TransactionOptions::retries ≥ 0` — flat override of maxRetries
 *     (0 = explicitly no retries for that call).
 */
#pragma once

#include <cstdint>
#include <string_view>

#include "tether/modbus/ModbusError.hpp"

namespace tether::modbus {

/**
 * @brief How the executor reacts when a transaction attempt fails.
 *
 *  - Retry:           re-issue up to maxRetries more times on a retryable
 *                     error (corrupt/timeout/transport/busy per flags),
 *                     then fail with the last error. DEFAULT.
 *  - RetryBackoff:    same attempts as Retry, but the inter-attempt delay
 *                     grows exponentially (backoffBaseMs × 2ⁿ capped at
 *                     backoffMaxMs).
 *  - FailFast:        exactly one attempt — the error propagates now.
 *  - DropAndContinue: exactly one attempt; failure is counted in
 *                     ExecutorStats::dropped and returned as-is — the
 *                     caller (e.g. a Poller) treats it as "no data this
 *                     cycle" rather than an alarm condition.
 *  - CircuitBreaker:  Retry semantics per call; when circuitThreshold
 *                     consecutive calls have exhausted all attempts, the
 *                     circuit opens for circuitResetMs and calls fail
 *                     fast (Cancelled) — a dead device isn't hammered.
 */
enum class ErrorStrategy : uint8_t {
    Retry,
    RetryBackoff,
    FailFast,
    DropAndContinue,
    CircuitBreaker,
};

struct ErrorPolicy {
    ErrorStrategy strategy = ErrorStrategy::Retry;
    /// Extra attempts after the first try (total attempts = 1 + N).
    /// Default 8 per spec — "re-read up to 8 times, then fail".
    uint8_t maxRetries = 8;
    /// Fixed delay inserted between attempts under ErrorStrategy::Retry.
    int retryDelayMs = 0;
    /// First retry delay under RetryBackoff (attempt 2), then doubled.
    int backoffBaseMs = 2;
    /// Backoff cap.
    int backoffMaxMs = 500;
    /// CircuitBreaker: consecutive fully-failed calls before opening.
    int circuitThreshold = 5;
    /// CircuitBreaker: ms to stay open before a half-open probe call.
    int circuitResetMs = 30000;

    /// Per-cause retry toggles — the user chooses what is worth re-trying.
    bool retryOnCorrupt   = true;  ///< mangled frame seen (CRC/parity)
    bool retryOnTimeout   = true;  ///< silence / no response
    bool retryOnTransport = true;  ///< link I/O faults
    bool retryOnBusy      = true;  ///< Busy + exception 0x05/0x06
};

/// Which error categories the policy allows a retry for.
inline bool policyRetryable(const ErrorPolicy& p, const ModbusError& e) {
    if (e.isCorrupt())   return p.retryOnCorrupt;
    if (e.isTimeout())   return p.retryOnTimeout;
    if (e.isTransport()) return p.retryOnTransport;
    if (e.isBusy())      return p.retryOnBusy;
    return false;
}

/// Attempt count for the first strategy pass (FailFast/Drop = 1).
inline int policyAttempts(const ErrorPolicy& p) {
    switch (p.strategy) {
        case ErrorStrategy::FailFast:
        case ErrorStrategy::DropAndContinue:
            return 1;
        default:
            return 1 + p.maxRetries;
    }
}

/// Delay inserted before attempt `attempt` (1-based; attempt ≥ 2).
inline int policyDelayMs(const ErrorPolicy& p, int attempt) {
    if (p.strategy == ErrorStrategy::RetryBackoff) {
        // attempt 2 → base, 3 → 2·base, 4 → 4·base … capped.
        long long d = static_cast<long long>(p.backoffBaseMs)
                      << (attempt - 2);
        if (d > p.backoffMaxMs) d = p.backoffMaxMs;
        return int(d);
    }
    return p.retryDelayMs;
}

/// INI / CLI spelling ↔ enum.
inline Expected<ErrorStrategy>
errorStrategyFromString(std::string_view s) {
    if (s == "retry" || s == "reread")           return ErrorStrategy::Retry;
    if (s == "retry_backoff" || s == "backoff")  return ErrorStrategy::RetryBackoff;
    if (s == "fail_fast" || s == "failfast")     return ErrorStrategy::FailFast;
    if (s == "drop" || s == "drop_and_continue") return ErrorStrategy::DropAndContinue;
    if (s == "circuit_breaker" || s == "circuit")return ErrorStrategy::CircuitBreaker;
    return err<ErrorStrategy>(ModbusError::config(
        "bad error_strategy: " + std::string(s)));
}
inline std::string_view toString(ErrorStrategy s) {
    switch (s) {
        case ErrorStrategy::Retry:           return "retry";
        case ErrorStrategy::RetryBackoff:    return "retry_backoff";
        case ErrorStrategy::FailFast:        return "fail_fast";
        case ErrorStrategy::DropAndContinue: return "drop_and_continue";
        case ErrorStrategy::CircuitBreaker:  return "circuit_breaker";
    }
    return "?";
}

} // namespace tether::modbus
