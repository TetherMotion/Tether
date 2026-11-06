/**
 * @file BusExecutor.hpp
 * @brief Serializes invocations onto one channel with retry/backoff.
 *
 * RTU: strictly one transaction on the wire at a time (plan §8.4).
 * TCP: still serializes in this impl (pipelining is a later refinement —
 *      correctness first; max_inflight lands with TransactionManager).
 */
#pragma once

#include <atomic>
#include <mutex>
#include <queue>

#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/protocol/ExceptionCode.hpp"

namespace tether::modbus {

struct ExecutorStats {
    std::atomic<uint64_t> ops{0}, retries{0}, timeouts{0},
                          exceptions{0}, failures{0};
};

/**
 * @brief Executes invocations on a channel: retry loop + exception-PDU
 * translation + broadcast sendOnly path.
 */
class BusExecutor final : public IModbusInvoker {
public:
    explicit BusExecutor(IChannel& ch) : ch_(ch) {}

    Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                         std::span<const uint8_t> body,
                         const InvokeOptions& o = {}) override;

    Expected<void> invokeBroadcast(uint8_t, FunctionCode fc,
                                   std::span<const uint8_t> body,
                                   const InvokeOptions& = {}) override {
        return ch_.sendOnly(0, prepend(fc, body));
    }

    ExecutorStats& stats() { return stats_; }

    /// Serialize access for callers that must do multi-op critical
    /// sections (e.g. robust strategies that must not interleave).
    std::unique_lock<std::mutex> lockBus() {
        return std::unique_lock{mu_};
    }

private:
    static std::vector<uint8_t> prepend(FunctionCode fc,
                                        std::span<const uint8_t> body) {
        std::vector<uint8_t> p(body.size() + 1);
        p[0] = uint8_t(fc);
        std::copy(body.begin(), body.end(), p.begin() + 1);
        return p;
    }
    /// ADU → Pdu or translated exception error.
    static Expected<Pdu> toPdu(FunctionCode fc, Adu a);

    IChannel&     ch_;
    std::mutex    mu_;
    ExecutorStats stats_;
};

} // namespace tether::modbus
