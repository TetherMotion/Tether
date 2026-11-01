/**
 * @file MonitoredRemoteFunctionInvocation.hpp
 * @brief Serialized asynchronous remote-function invocation with
 *        tick-stamped results and freshness accounting
 *
 * MonitoredRemoteFunctionInvocation wraps an invoker (e.g.
 * Server::callPeerFunction for server→peer calls, or
 * TetherIOClient::invokeEx for client→server calls) and provides the
 * bookkeeping every caller-side "remote stage" needs:
 *
 *   - At most one call is in flight; call() during a call returns false.
 *   - The newest successful result is stored stamped with the caller's
 *     producer tick, so a consumer at tick T can classify freshness by
 *     age: age(T) = T − producerTick.
 *   - Timeouts, transport/decode errors, issued and completed calls are
 *     counted separately.
 *   - The result snapshot and all counters are safe to read from any
 *     thread.
 *
 * ## Usage
 *
 * @code
 *   MonitoredRemoteFunctionInvocation<std::vector<double>> mon(
 *       // invoker: issue one call, report through done exactly once
 *       [&](std::vector<uint8_t> arg, uint64_t timeoutUs, auto done) {
 *           return server.callPeerFunction("fn", {arg}, deadline, done);
 *       },
 *       // decoder: bytes -> payload, false = malformed
 *       [](const uint8_t* d, size_t n, std::vector<double>& out) {
 *           return decode(d, n, out);
 *       },
 *       /*maxAgeTicks=* / 5, /*timeoutUs=* / 2000000);
 *
 *   // fast loop:
 *   if (!mon.inFlight()) mon.call(tick, argBytes);
 *   if (mon.fresh(tick)) { const auto& payload = mon.result().payload; ... }
 * @endcode
 *
 * ## Threading contract
 *
 *   - call() may run on any thread (typically a control loop).
 *   - The invoker's done callback may run on any thread (typically an
 *     IO/session thread) and MAY run inline inside call() for synchronous
 *     invokers — everything done-touched is shared state; no caller
 *     context is assumed.
 *   - result()/producerTick()/age()/fresh()/the counters may run
 *     concurrently with call() and the done callback.
 *
 * ## Lifetime contract
 *
 * The done callback holds the invocation state via weak_ptr — a call
 * completing after this object is destroyed drops its result instead of
 * touching freed memory. Destruction does NOT cancel an in-flight call;
 * invoke() must remain callable until its last callback fires, so the
 * invoker (session/client) should outlive the object or be stopped first.
 *
 * ## Decode contract
 *
 * `decode` is invoked on the done thread with the raw return value; it
 * must validate sizes itself and return false on any malformed input
 * (counted as an error). Exceptions from decode/invoke are caught and
 * counted as errors — the control path never throws.
 *
 * @tparam Payload  decoded result type; must be default-constructible
 *                  for the result slot.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace tether::io {

template <typename Payload>
class MonitoredRemoteFunctionInvocation {
    static_assert(std::is_default_constructible_v<Payload>,
                  "Payload must be default-constructible for slot storage");

public:
    /**Caller-chosen tick/stamp type (e.g. a loop sequence number). */
    using Tick = uint64_t;
    /**producerTick sentinel until the first successful result lands. */
    static constexpr Tick kNoResult = ~Tick{0};

    /**
     * @brief Normalized call outcome the invoker reports through `done`.
     *
     * Field mapping for the common invokers:
     *   - Session::InvokeResult maps field-for-field
     *     (timedOut/success/hasReturnValue/returnValue).
     *   - A client-side invokeEx adapter fills the same fields from its
     *     InvokeResult.
     *
     * Exactly one of: timedOut, !success/!hasReturnValue, or a decoded
     * result applies per call.
     */
    struct Outcome {
        bool timedOut = false; /**< transport deadline hit */
        bool success = false; /**< callee executed without RPC error */
        bool hasReturnValue = false; /**< return payload present */
        std::vector<uint8_t> returnValue; /**< raw encoded return value */
    };

    /**Invoker completion sink: called exactly once per issued call. */
    using DoneFn = std::function<void(const Outcome&)>;
    /**
     * @brief Invoker: issue one call carrying `arg`, honoring `timeoutUs`,
     *        and report through `done` exactly once.
     * @return false when the call cannot be issued (e.g. no peer hosts
     *         the function).
     *
     * `timeoutUs` is a relative budget the invoker maps to its own
     * deadline scheme; 0 = no deadline.
     */
    using InvokeFn = std::function<bool(std::vector<uint8_t> arg,
                                        uint64_t timeoutUs, DoneFn done)>;
    /**
     * @brief Decoder: translate `returnValue` bytes into `out`.
     * @return false on any malformed input (counted as an error).
     */
    using DecodeFn = std::function<bool(const uint8_t* data, size_t len,
                                        Payload& out)>;

    /**
     * @param invoke       issue-fn; empty → call() always returns false
     * @param decode       decode-fn; empty → every successful call counts
     *                     as an error
     * @param maxAgeTicks  freshness window for fresh(); tunable at runtime
     * @param timeoutUs    per-call deadline budget passed to the invoker
     */
    MonitoredRemoteFunctionInvocation(InvokeFn invoke, DecodeFn decode,
                                      uint32_t maxAgeTicks, uint64_t timeoutUs)
        : invoke_(std::move(invoke)), decode_(std::move(decode)),
          state_(std::make_shared<State>(maxAgeTicks, timeoutUs)) {}

    MonitoredRemoteFunctionInvocation(
        const MonitoredRemoteFunctionInvocation&) = delete;
    MonitoredRemoteFunctionInvocation& operator=(
        const MonitoredRemoteFunctionInvocation&) = delete;

    /**
     * @brief Issue one call stamped `tick`.
     * @param tick  caller-chosen stamp (a loop sequence number); stored
     *              onto the result so consumers classify freshness at
     *              their own current tick
     * @param arg   raw argument bytes handed to the invoker
     * @return false when `invoke` is empty, a call is already in flight,
     *         or the invoker refused the call.
     */
    bool call(Tick tick, std::vector<uint8_t> arg) {
        if (!invoke_ || state_->in_flight.exchange(true)) return false;
        const std::weak_ptr<State> weak = state_;
        const DecodeFn decode = decode_;
        const auto timeoutUs = state_->timeout_us.load();
        bool sent = false;
        try {
            sent = invoke_(std::move(arg), timeoutUs,
                           [weak, tick, decode](const Outcome& res) {
                               const auto st = weak.lock();
                               if (!st) return;  // owner gone — drop result
                               st->in_flight.store(false);
                               if (res.timedOut) {
                                   ++st->timeouts;
                                   return;
                               }
                               if (!res.success || !res.hasReturnValue) {
                                   ++st->errors;
                                   return;
                               }
                               Payload payload{};
                               bool ok = false;
                               try {
                                   ok = decode &&
                                        decode(res.returnValue.data(),
                                               res.returnValue.size(), payload);
                               } catch (...) {
                                   ok = false;
                               }
                               if (!ok) {
                                   ++st->errors;
                                   return;
                               }
                               {
                                   std::lock_guard lk(st->result_mtx);
                                   st->result = {tick, std::move(payload), true};
                               }
                               // Ordered after the result write: a reader
                               // seeing the new tick is guaranteed the
                               // payload is already stored.
                               st->producer_tick.store(tick);
                               ++st->completed;
                           });
        } catch (...) {
            sent = false;
        }
        if (!sent) {
            state_->in_flight.store(false);
            return false;
        }
        ++state_->issued;
        return true;
    }

    /**Snapshot of the newest successful result (copies the payload). */
    struct Result {
        Tick producerTick = 0; /**< tick stamped by call() */
        Payload payload{}; /**< decoded return value */
        bool valid = false; /**< false until the first success */
    };

    /**
     * Latest completed result. `valid` is false until the first
     * successful call completes.
     */
    Result result() const {
        std::lock_guard lk(state_->result_mtx);
        return state_->result;
    }

    /**Tick stamped on the newest result; kNoResult until the first one. */
    Tick producerTick() const { return state_->producer_tick.load(); }

    /**
     * @brief Ticks between `tick` and the newest result.
     * @param tick  consumer's current tick
     * @return ~Tick{0} when there is no result or the stamp is ahead of
     *         `tick` (clock skew guard).
     */
    uint64_t age(Tick tick) const {
        const Tick produced = producerTick();
        if (produced == kNoResult || produced > tick) return ~Tick{0};
        return tick - produced;
    }

    /**
     * @brief True when a result exists and its age is within maxAgeTicks
     *        (inclusive bound).
     * @param tick  consumer's current tick
     */
    bool fresh(Tick tick) const {
        return age(tick) <= state_->max_age_ticks.load();
    }

    /**True while a call awaits its done callback. */
    bool inFlight() const { return state_->in_flight.load(); }
    /**Calls successfully handed to the invoker. */
    uint64_t issued() const { return state_->issued.load(); }
    /**Calls that produced a decoded result. */
    uint64_t completed() const { return state_->completed.load(); }
    /**Calls whose outcome was a transport timeout. */
    uint64_t timeouts() const { return state_->timeouts.load(); }
    /**Calls that failed, lacked a return value, or failed to decode. */
    uint64_t errors() const { return state_->errors.load(); }

    /**Freshness window for fresh(), in ticks (inclusive bound). */
    uint32_t maxAgeTicks() const { return state_->max_age_ticks.load(); }
    /**
     * @brief Retune the freshness window.
     * @param ticks  new maxAgeTicks value
     */
    void setMaxAgeTicks(uint32_t ticks) {
        state_->max_age_ticks.store(ticks);
    }
    /**Per-call deadline budget handed to the invoker (µs). */
    uint64_t timeoutUs() const { return state_->timeout_us.load(); }
    /**
     * @brief Retune the per-call deadline budget.
     * @param us  new timeoutUs value
     */
    void setTimeoutUs(uint64_t us) { state_->timeout_us.store(us); }

    /**
     * Drop the stored result and all counters. An in-flight call is not
     * cancelled — its outcome can still land and become the new result.
     */
    void reset() {
        std::lock_guard lk(state_->result_mtx);
        state_->result = {};
        state_->producer_tick.store(kNoResult);
        state_->issued.store(0);
        state_->completed.store(0);
        state_->timeouts.store(0);
        state_->errors.store(0);
    }

private:
    /**
     * Shared state — captured weakly by done callbacks so a call
     * completing after destruction cannot touch freed memory.
     */
    struct State {
        explicit State(uint32_t maxAgeTicks, uint64_t timeoutUs)
            : max_age_ticks(maxAgeTicks), timeout_us(timeoutUs) {}

        std::atomic<bool> in_flight{false};
        std::atomic<Tick> producer_tick{kNoResult};
        std::atomic<uint32_t> max_age_ticks;
        std::atomic<uint64_t> timeout_us;
        std::atomic<uint64_t> issued{0}, completed{0};
        std::atomic<uint64_t> timeouts{0}, errors{0};
        mutable std::mutex result_mtx;
        Result result;
    };

    InvokeFn invoke_;
    DecodeFn decode_;
    std::shared_ptr<State> state_;
};

}  // namespace tether::io
