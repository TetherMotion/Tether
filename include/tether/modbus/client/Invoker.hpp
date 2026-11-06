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

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/protocol/FunctionCode.hpp"
#include "tether/modbus/protocol/Pdu.hpp"

namespace tether::modbus {

struct InvokeOptions {
    int      timeoutMs   = 1000;   ///< per-attempt response deadline
    int      retries     = 0;      ///< extra attempts on retryable error
    int      retryDelayMs = 0;     ///< delay between attempts
    /// Called on each failed attempt (attempt 1..n-1) before retry.
    std::function<void(int attempt, const ModbusError&)> onRetry;
};

/// Raw invoker: send a PDU, get the response PDU (async-capable).
class IModbusInvoker {
public:
    virtual ~IModbusInvoker() = default;

    /// Fire-and-forget for broadcast (unit 0). Default: invoke + ignore rsp.
    virtual Expected<void> invokeBroadcast(
        uint8_t unit, FunctionCode fc, std::span<const uint8_t> body,
        const InvokeOptions& o = {});

    /// Synchronous invoke: request body → response PDU.
    virtual Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                                 std::span<const uint8_t> body,
                                 const InvokeOptions& o = {}) = 0;

    /// Async invoke (default: wrap in std::async).
    virtual std::future<Expected<Pdu>> invokeAsync(
        uint8_t unit, FunctionCode fc, std::vector<uint8_t> body,
        InvokeOptions o = {});
};

} // namespace tether::modbus
