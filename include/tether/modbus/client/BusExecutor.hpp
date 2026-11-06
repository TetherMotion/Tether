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
    std::atomic<uint64_t> ops{0}, retries{0}, timeouts{0}, corrupt{0},
                          exceptions{0}, failures{0}, dropped{0},
                          circuitTrips{0}, circuitRejected{0};
};

/**
 * @brief Executes invocations on a channel: error-policy retry loop +
 * exception-PDU translation + broadcast sendOnly path.
 *
 * The ErrorPolicy decides how transaction errors are handled — corrupt
 * frames (parity/CRC), timeouts, transport faults — defaulting to Retry
 * with 8 extra attempts. Strategy FailFast → one shot; DropAndContinue →
 * one shot + dropped stat; RetryBackoff → exponential delay;
 * CircuitBreaker → opens after `circuitThreshold` exhausted calls.
 */
class BusExecutor final : public IModbusInvoker {
public:
    explicit BusExecutor(IChannel& ch) : ch_(ch) {}

    Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                         std::span<const uint8_t> body,
                         const TransactionOptions& o = {}) override;

    Expected<void> invokeBroadcast(uint8_t, FunctionCode fc,
                                   std::span<const uint8_t> body,
                                   const TransactionOptions& = {}) override {
        return ch_.sendOnly(0, prepend(fc, body));
    }

    ExecutorStats& stats() { return stats_; }

    /// The connection's DEFAULT transaction options — every invoke()
    /// merges its per-call TransactionOptions over these (unset fields
    /// inherit). Default: timeout 1000 ms, Retry ×8 on corrupt/timeout.
    void setDefaultOptions(TransactionOptions o) {
        std::lock_guard lk(mu_);
        defaults_ = std::move(o);
    }
    TransactionOptions defaultOptions() const {
        std::lock_guard lk(mu_);
        return defaults_;
    }
    /// Sugar for setDefaultOptions({.errorPolicy = p}).
    void setErrorPolicy(ErrorPolicy p) {
        std::lock_guard lk(mu_);
        defaults_.errorPolicy = p;
    }
    ErrorPolicy errorPolicy() const {
        std::lock_guard lk(mu_);
        return defaults_.errorPolicy.value_or(ErrorPolicy{});
    }

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
    mutable std::mutex mu_;
    ExecutorStats stats_;
    /// Connection default options — per-call fields override non-neg.
    TransactionOptions defaults_{.timeoutMs = 1000};
    // CircuitBreaker state (guarded by mu_)
    int     consecFails_ = 0;
    int64_t circuitOpenNs_ = 0;   // steady-ns when opened; 0 = closed
};

} // namespace tether::modbus
