#include "tether/modbus/client/BusExecutor.hpp"

#include <thread>

namespace tether::modbus {

Expected<void> IModbusInvoker::invokeBroadcast(
    uint8_t, FunctionCode fc, std::span<const uint8_t> body,
    const InvokeOptions& o) {
    // Generic fallback: unit 0 invoke, ignore any straggler response.
    (void)invoke(0, fc, body, o);
    return {};
}

std::future<Expected<Pdu>> IModbusInvoker::invokeAsync(
    uint8_t unit, FunctionCode fc, std::vector<uint8_t> body,
    InvokeOptions o) {
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
                                  const InvokeOptions& o) {
    std::unique_lock lk(mu_);
    const int attempts = 1 + std::max(0, o.retries);
    ModbusError last = ModbusError::codec("no attempt");
    for (int i = 0; i < attempts; ++i) {
        if (i > 0 && o.retryDelayMs > 0)
            std::this_thread::sleep_for(
                std::chrono::milliseconds(o.retryDelayMs));
        stats_.ops++;
        auto adu = ch_.transact(unit, prepend(fc, body), o.timeoutMs);
        if (!adu) {
            last = adu.error();
            if (last.isTimeout()) stats_.timeouts++;
            else stats_.failures++;
            if (!last.retryable() || i + 1 == attempts) break;
            stats_.retries++;
            if (o.onRetry) o.onRetry(i, last);
            continue;
        }
        auto pdu = toPdu(fc, std::move(*adu));
        if (!pdu) {
            last = pdu.error();
            if (last.isException()) {
                stats_.exceptions++;
                // Retryable exceptions: 0x05 ack, 0x06 busy.
                if (exceptionRetryable(uint8_t(last.code() & 0xFF)) &&
                    i + 1 < attempts) {
                    stats_.retries++;
                    if (o.onRetry) o.onRetry(i, last);
                    continue;
                }
            }
            return err<Pdu>(last);
        }
        return pdu;
    }
    return err<Pdu>(last);
}

} // namespace tether::modbus
