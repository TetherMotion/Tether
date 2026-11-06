#include "tether/modbus/client/BusExecutor.hpp"

#include <algorithm>
#include <thread>

namespace tether::modbus {

Expected<void> IModbusInvoker::invokeBroadcast(
    uint8_t, FunctionCode fc, std::span<const uint8_t> body,
    const TransactionOptions& o) {
    // Generic fallback: unit 0 invoke, ignore any straggler response.
    (void)invoke(0, fc, body, o);
    return {};
}

std::future<Expected<Pdu>> IModbusInvoker::invokeAsync(
    uint8_t unit, FunctionCode fc, std::vector<uint8_t> body,
    TransactionOptions o) {
    return std::async(std::launch::async,
        [this, unit, fc, b = std::move(body), o]() mutable {
            return invoke(unit, fc, b, o);
        });
}

Expected<Pdu> BusExecutor::toPdu(FunctionCode fc, Adu a) {
    if (a.pdu.empty())
        return err<Pdu>(ModbusError::codec("empty pdu"));
    const uint8_t rfc = a.pdu[0];
    if (rfc == (uint8_t(fc) | 0x80)) {
        const uint8_t code = a.pdu.size() > 1 ? a.pdu[1] : 0;
        return err<Pdu>(exceptionToError(code));
    }
    if (rfc != uint8_t(fc))
        return err<Pdu>(ModbusError::codec("function mismatch"));
    Pdu p{fc};
    if (!p.append(std::span(a.pdu.data() + 1, a.pdu.size() - 1)))
        return err<Pdu>(ModbusError::codec("pdu oversize"));
    return p;
}

Expected<Pdu> BusExecutor::invoke(uint8_t unit, FunctionCode fc,
                                  std::span<const uint8_t> body,
                                  const TransactionOptions& o) {
    std::unique_lock lk(mu_);
    // Merge connection defaults + per-call overrides (negative/empty
    // fields inherit), then resolve concrete values: built-in Retry×8
    // policy unless overridden, 1000 ms timeout unless set anywhere.
    const TransactionOptions m = overlayOpts(defaults_, o);
    ErrorPolicy pol = m.errorPolicy.value_or(ErrorPolicy{});
    if (m.retries >= 0)      pol.maxRetries   = uint8_t(std::min(m.retries, 255));
    if (m.retryDelayMs >= 0) pol.retryDelayMs = m.retryDelayMs;
    const int timeoutMs = m.timeoutMs >= 0 ? m.timeoutMs : 1000;

    // CircuitBreaker gate: reject fast while the circuit is open; after
    // circuitResetMs one half-open probe call is allowed through.
    if (pol.strategy == ErrorStrategy::CircuitBreaker &&
        circuitOpenNs_ != 0) {
        const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now - circuitOpenNs_ < int64_t(pol.circuitResetMs) * 1'000'000) {
            stats_.circuitRejected++;
            return err<Pdu>(ModbusError::cancelled(
                "circuit breaker open"));
        }
        // half-open: fall through — this call is the probe.
    }

    const int attempts = policyAttempts(pol);
    ModbusError last = ModbusError::codec("no attempt");
    for (int i = 0; i < attempts; ++i) {
        if (i > 0) {
            const int delayMs = policyDelayMs(pol, i + 1);
            if (delayMs > 0)
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(delayMs));
        }
        stats_.ops++;
        auto adu = ch_.transact(unit, prepend(fc, body), timeoutMs);
        if (!adu) {
            last = adu.error();
            if (last.isTimeout()) stats_.timeouts++;
            else if (last.isCorrupt()) stats_.corrupt++;
            else stats_.failures++;
            if (!policyRetryable(pol, last) || i + 1 == attempts) break;
            stats_.retries++;
            if (m.onRetry) m.onRetry(i, last);
            continue;
        }
        auto pdu = toPdu(fc, std::move(*adu));
        if (!pdu) {
            last = pdu.error();
            if (last.isException()) {
                stats_.exceptions++;
                // Retryable exceptions: 0x05 ack, 0x06 busy.
                if (pol.retryOnBusy &&
                    exceptionRetryable(uint8_t(last.code() & 0xFF)) &&
                    i + 1 < attempts) {
                    stats_.retries++;
                    if (m.onRetry) m.onRetry(i, last);
                    continue;
                }
            }
            return err<Pdu>(last);
        }
        consecFails_ = 0;
        if (circuitOpenNs_ != 0) circuitOpenNs_ = 0;   // half-open probe OK
        return pdu;
    }

    // All attempts exhausted (or single shot) — strategy epilogue.
    if (pol.strategy == ErrorStrategy::DropAndContinue)
        stats_.dropped++;
    if (pol.strategy == ErrorStrategy::CircuitBreaker) {
        if (++consecFails_ >= pol.circuitThreshold &&
            circuitOpenNs_ == 0) {
            circuitOpenNs_ = std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            stats_.circuitTrips++;
        } else if (circuitOpenNs_ != 0) {
            // half-open probe failed → stay open, restart the window.
            circuitOpenNs_ = std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
        }
    }
    return err<Pdu>(last);
}

} // namespace tether::modbus
