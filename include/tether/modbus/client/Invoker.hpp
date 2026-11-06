/**
 * @file Invoker.hpp
 * @brief IModbusInvoker — the universal raw-PDU operation contract.
 *        Every higher layer (typed, robust, polling) composes over it.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <span>
#include <vector>

#include <optional>

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/client/ErrorPolicy.hpp"
#include "tether/modbus/protocol/FunctionCode.hpp"
#include "tether/modbus/protocol/Pdu.hpp"

namespace tether::modbus {

/**
 * @brief Options governing one transaction (request → response).
 *
 * A Modbus connection owns a DEFAULT TransactionOptions (set on the
 * BusExecutor — see setDefaultOptions); every call may override any
 * subset. Fields are sentinel-based: a negative/empty value means
 * "inherit the connection default", a concrete value overrides it.
 *
 * Merge order per call: built-in fallback → executor defaults → call.
 * So `TransactionOptions{}` = pure connection defaults (1000 ms timeout,
 * Retry ×8); `TransactionOptions{.timeoutMs = 50}` = 50 ms timeout with
 * everything else inherited.
 */
struct TransactionOptions {
    /// Per-attempt response deadline; <0 → executor default (1000 ms).
    int      timeoutMs = -1;
    /// ≥0 overrides errorPolicy.maxRetries (0 = no retries this call).
    int      retries = -1;
    /// ≥0 overrides errorPolicy.retryDelayMs.
    int      retryDelayMs = -1;
    /// Full error-handling strategy override; unset → inherit.
    std::optional<ErrorPolicy> errorPolicy;
    /// Called on each failed attempt (attempt 1..n-1) before retry.
    std::function<void(int attempt, const ModbusError&)> onRetry;
};

/// Overlay `over` onto `base`: every set field of `over` wins; unset
/// fields fall back to `base`. Layers client defaults → connection
/// defaults → per-call overrides.
inline TransactionOptions overlayOpts(const TransactionOptions& base,
                                      const TransactionOptions& over) {
    TransactionOptions m = base;
    if (over.timeoutMs >= 0)    m.timeoutMs    = over.timeoutMs;
    if (over.retries >= 0)      m.retries      = over.retries;
    if (over.retryDelayMs >= 0) m.retryDelayMs = over.retryDelayMs;
    if (over.errorPolicy)       m.errorPolicy  = over.errorPolicy;
    if (over.onRetry)           m.onRetry      = over.onRetry;
    return m;
}

/// Raw invoker: send a PDU, get the response PDU (async-capable).
class IModbusInvoker {
public:
    virtual ~IModbusInvoker() = default;

    /// Fire-and-forget for broadcast (unit 0). Default: invoke + ignore rsp.
    virtual Expected<void> invokeBroadcast(
        uint8_t unit, FunctionCode fc, std::span<const uint8_t> body,
        const TransactionOptions& o = {});

    /// Synchronous invoke: request body → response PDU.
    virtual Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                                 std::span<const uint8_t> body,
                                 const TransactionOptions& o = {}) = 0;

    /// Async invoke (default: wrap in std::async).
    virtual std::future<Expected<Pdu>> invokeAsync(
        uint8_t unit, FunctionCode fc, std::vector<uint8_t> body,
        TransactionOptions o = {});
};

} // namespace tether::modbus
