/**
 * @file RobustInvoker.hpp
 * @brief IModbusInvoker decorator implementing the assurance strategies.
 */
#pragma once

#include <atomic>
#include <optional>
#include <vector>

#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/robust/Strategy.hpp"

namespace tether::modbus {

struct AssuranceStats {
    std::atomic<uint64_t> ops{0}, samples{0}, mismatches{0},
        confirms{0}, confirmFailures{0}, staleServed{0},
        notCompiled{0}, votesWon{0}, votesLost{0};
};

/**
 * @brief Wraps an inner invoker and applies an AssuranceOpts strategy to
 * read-style invokes; write-confirm strategies apply via `assuredWrite`.
 *
 * Read strategy is selected per-invoke by InvokeOptions extension? No —
 * the strategy is bound at decoration time (per-endpoint), matching
 * "flexible logic": chain invokers for different policies.
 */
class RobustInvoker final : public IModbusInvoker {
public:
    RobustInvoker(IModbusInvoker& inner, AssuranceOpts opts)
        : inner_(inner), opts_(opts) {
        lastGood_.resize(1);
    }

    AssuranceOpts& opts() { return opts_; }
    static AssuranceStats& stats() { return stats_; }

    Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                         std::span<const uint8_t> body,
                         const InvokeOptions& o = {}) override;

    /// Explicit assured write: sends `body` per strategy then confirms
    /// by re-invoking `confirmBody` (a matching read request).
    Expected<Pdu> assuredWrite(uint8_t unit, FunctionCode writeFc,
                               std::span<const uint8_t> body,
                               FunctionCode readFc,
                               std::span<const uint8_t> confirmBody,
                               std::span<const uint8_t> expectedRespBody,
                               const InvokeOptions& o = {});

    Expected<void> invokeBroadcast(
        uint8_t unit, FunctionCode fc, std::span<const uint8_t> body,
        const InvokeOptions& o = {}) override;

private:
    Expected<Pdu> readOnce(uint8_t u, FunctionCode fc,
                           std::span<const uint8_t> b,
                           const InvokeOptions& o);
    Expected<Pdu> applyRead(uint8_t u, FunctionCode fc,
                            std::span<const uint8_t> b,
                            const InvokeOptions& o);

    bool same(const Pdu& a, const Pdu& b) const;
    static ModbusError notCompiled(Strategy s);

    IModbusInvoker& inner_;
    AssuranceOpts   opts_;
    std::vector<std::optional<Pdu>> lastGood_;
    static AssuranceStats stats_;
};

} // namespace tether::modbus
