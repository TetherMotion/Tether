// MonitoredRemoteFunctionInvocation — serialize one-at-a-time remote function
// calls with tick-stamped results and freshness accounting.
//
// The component wraps an invoker (e.g. Server::callPeerFunction for
// server→peer calls, or TetherIOClient::invokeEx for client→server calls)
// and provides the bookkeeping every caller-side "remote stage" needs:
//
//   * At most one call is in flight; call() during a call returns false.
//   * The newest successful result is stored stamped with the caller's
//     tick, so a consumer at tick T can classify freshness by age.
//   * Timeouts, transport/decode errors, issued and completed calls are
//     counted separately.
//   * Counters and the result snapshot are safe to read from any thread.
//
// Threading contract:
//   * call() may run on any thread (typically a control loop).
//   * The invoker's done callback may run on any thread (typically an
//     IO/session thread) and MAY run inline inside call() for synchronous
//     invokers. Everything done-touched is shared state; no caller context
//     is assumed.
//   * result()/fresh()/age() may run concurrently with call().
//
// Lifetime contract: the done callback holds the invocation state via
// weak_ptr — a call completing after this object is destroyed drops its
// result instead of touching freed memory. Destruction does NOT cancel an
// in-flight call; invoke() must remain callable until its last callback
// fires, so the invoker (session/client) should outlive the object or be
// stopped first.
//
// Decode contract: `decode` is invoked on the done thread with the raw
// return value; it must validate sizes itself and return false on any
// malformed input (counted as an error). Exceptions from decode/invoke are
// caught and counted as errors — the control path never throws.
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
    using Tick = uint64_t;
    /// producerTick sentinel until the first successful result lands.
    static constexpr Tick kNoResult = ~Tick{0};

    /// Normalized call outcome the invoker reports through `done`.
    struct Outcome {
        bool timedOut = false;        // transport deadline hit
        bool success = false;         // callee executed without RPC error
        bool hasReturnValue = false;  // return payload present
        std::vector<uint8_t> returnValue;
    };

    using DoneFn = std::function<void(const Outcome&)>;
    /// Invoker: issue one call carrying `arg`, honoring `timeoutUs`, and
    /// report through `done` exactly once. Return false when the call
    /// cannot be issued (e.g. no peer hosts the function).
    using InvokeFn = std::function<bool(std::vector<uint8_t> arg,
                                        uint64_t timeoutUs, DoneFn done)>;
    /// Decoder: translate `returnValue` bytes into `out`; false = malformed.
    using DecodeFn = std::function<bool(const uint8_t* data, size_t len,
                                        Payload& out)>;

    /// `invoke`/`decode` may be empty: an empty invoke makes call() return
    /// false; an empty decode counts every successful call as an error.
    MonitoredRemoteFunctionInvocation(InvokeFn invoke, DecodeFn decode,
                                      uint32_t maxAgeTicks, uint64_t timeoutUs)
        : invoke_(std::move(invoke)), decode_(std::move(decode)),
          state_(std::make_shared<State>(maxAgeTicks, timeoutUs)) {}

    /// Issue one call stamped `tick`. Returns false when `invoke` is empty,
    /// a call is already in flight, or the invoker refused the call.
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

    /// Latest completed result. `valid` is false until the first
    /// successful call completes.
    struct Result {
        Tick producerTick = 0;
        Payload payload{};
        bool valid = false;
    };

    /// Snapshot of the newest successful result (copies the payload).
    Result result() const {
        std::lock_guard lk(state_->result_mtx);
        return state_->result;
    }

    /// Tick stamped on the newest result; kNoResult until the first one.
    Tick producerTick() const { return state_->producer_tick.load(); }

    /// Ticks between `tick` and the newest result; ~Tick{0} when there is
    /// no result or the stamp is ahead of `tick` (clock skew guard).
    uint64_t age(Tick tick) const {
        const Tick produced = producerTick();
        if (produced == kNoResult || produced > tick) return ~Tick{0};
        return tick - produced;
    }

    /// True when a result exists and its age is within maxAgeTicks.
    bool fresh(Tick tick) const {
        return age(tick) <= state_->max_age_ticks.load();
    }

    bool inFlight() const { return state_->in_flight.load(); }
    uint64_t issued() const { return state_->issued.load(); }
    uint64_t completed() const { return state_->completed.load(); }
    uint64_t timeouts() const { return state_->timeouts.load(); }
    uint64_t errors() const { return state_->errors.load(); }

    uint32_t maxAgeTicks() const { return state_->max_age_ticks.load(); }
    void setMaxAgeTicks(uint32_t ticks) {
        state_->max_age_ticks.store(ticks);
    }
    uint64_t timeoutUs() const { return state_->timeout_us.load(); }
    void setTimeoutUs(uint64_t us) { state_->timeout_us.store(us); }

    /// Drop the stored result and all counters. An in-flight call is not
    /// cancelled — its outcome can still land and become the new result.
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
    /// Shared state — captured weakly by done callbacks so a call
    /// completing after destruction cannot touch freed memory.
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
