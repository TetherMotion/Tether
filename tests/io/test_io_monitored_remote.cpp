// MonitoredRemoteFunctionInvocation: serialization, freshness accounting,
// error/timeout counters, done-after-destruction safety.
#include <gtest/gtest.h>
#include "tether/io/MonitoredRemoteFunctionInvocation.hpp"

using tether::io::MonitoredRemoteFunctionInvocation;
using Mon = MonitoredRemoteFunctionInvocation<std::vector<double>>;

namespace {

/// Decode a little-endian f64 blob; false on malformed input.
bool decodeF64(const uint8_t* d, size_t n, std::vector<double>& out) {
    if (n % 8 != 0) return false;
    out.assign(reinterpret_cast<const double*>(d),
               reinterpret_cast<const double*>(d) + n / 8);
    return true;
}

/// Synchronous invoker stub: records calls, reports a scripted Outcome
/// inline through done (exercises the "done runs inside call()" path).
struct StubInvoker {
    Mon::Outcome outcome;
    bool refuse = false;
    int calls = 0;

    Mon::InvokeFn fn() {
        return [this](std::vector<uint8_t> arg, uint64_t timeoutUs,
                      Mon::DoneFn done) {
            ++calls;
            last_arg = std::move(arg);
            last_timeout = timeoutUs;
            if (refuse) return false;
            done(outcome);
            return true;
        };
    }
    std::vector<uint8_t> last_arg;
    uint64_t last_timeout = 0;
};

Mon::Outcome okResult(std::initializer_list<double> vals) {
    Mon::Outcome o;
    o.success = true;
    o.hasReturnValue = true;
    for (double v : vals) {
        const uint8_t* b = reinterpret_cast<const uint8_t*>(&v);
        o.returnValue.insert(o.returnValue.end(), b, b + 8);
    }
    return o;
}

TEST(MonitoredRemote, SerializesCallsAndReportsFreshResult) {
    StubInvoker inv;
    inv.outcome = okResult({1.5, 2.5});
    Mon mon(inv.fn(), decodeF64, 5, 1000);

    EXPECT_TRUE(mon.call(10, {0xAA}));
    // Done ran inline — the call already completed, not still in flight.
    EXPECT_FALSE(mon.inFlight());
    EXPECT_EQ(mon.issued(), 1u);
    EXPECT_EQ(mon.completed(), 1u);
    EXPECT_EQ(inv.last_timeout, 1000u);

    const auto res = mon.result();
    ASSERT_TRUE(res.valid);
    EXPECT_EQ(res.producerTick, 10u);
    ASSERT_EQ(res.payload.size(), 2u);
    EXPECT_DOUBLE_EQ(res.payload[0], 1.5);

    EXPECT_EQ(mon.producerTick(), 10u);
    EXPECT_TRUE(mon.fresh(12));
    EXPECT_FALSE(mon.fresh(16));  // age 6 > max 5
    EXPECT_EQ(mon.age(15), 5u);
}

TEST(MonitoredRemote, SecondCallWhileInFlightRejected) {
    Mon::DoneFn held;
    Mon mon(
        [&held](std::vector<uint8_t>, uint64_t, Mon::DoneFn done) {
            held = done;  // never completes
            return true;
        },
        decodeF64, 5, 0);
    EXPECT_TRUE(mon.call(1, {}));
    EXPECT_TRUE(mon.inFlight());
    EXPECT_FALSE(mon.call(2, {}));  // serialized: one at a time
    held(Mon::Outcome{});
}

TEST(MonitoredRemote, TimeoutErrorAndBadDecodeCounted) {
    StubInvoker inv;
    Mon mon(inv.fn(), decodeF64, 5, 0);

    inv.outcome.timedOut = true;
    EXPECT_TRUE(mon.call(1, {}));
    EXPECT_EQ(mon.timeouts(), 1u);
    EXPECT_FALSE(mon.result().valid);

    inv.outcome = Mon::Outcome{};  // !success
    EXPECT_TRUE(mon.call(2, {}));
    EXPECT_EQ(mon.errors(), 1u);

    inv.outcome = okResult({});  // valid but wrong size for decode? 0%8==0 → ok
    inv.outcome.returnValue = {1, 2, 3};
    inv.outcome.success = true;
    EXPECT_TRUE(mon.call(3, {}));
    EXPECT_EQ(mon.errors(), 2u);  // malformed payload counted
    EXPECT_FALSE(mon.result().valid);
}

TEST(MonitoredRemote, EmptyInvokeAndRefusalReturnFalse) {
    StubInvoker inv;
    Mon mon({}, decodeF64, 5, 0);
    EXPECT_FALSE(mon.call(1, {}));

    Mon mon2(inv.fn(), decodeF64, 5, 0);
    inv.refuse = true;
    EXPECT_FALSE(mon2.call(1, {}));
    EXPECT_FALSE(mon2.inFlight());
}

TEST(MonitoredRemote, OutcomeAfterDestroyIsDropped) {
    StubInvoker inv;
    Mon::DoneFn held;
    {
        Mon mon(
            [&inv, &held](std::vector<uint8_t>, uint64_t, Mon::DoneFn done) {
                held = done;
                ++inv.calls;
                return true;
            },
            decodeF64, 5, 0);
        EXPECT_TRUE(mon.call(7, {}));
    }
    // Owner destroyed — completing the call must not touch freed state.
    held(okResult({1.0}));
    SUCCEED();
}

TEST(MonitoredRemote, ResetClearsResultAndCounters) {
    StubInvoker inv;
    inv.outcome = okResult({1.0});
    Mon mon(inv.fn(), decodeF64, 5, 0);
    EXPECT_TRUE(mon.call(4, {}));
    mon.reset();
    EXPECT_FALSE(mon.result().valid);
    EXPECT_EQ(mon.producerTick(), Mon::kNoResult);
    EXPECT_EQ(mon.issued(), 0u);
    EXPECT_EQ(mon.completed(), 0u);
}

}  // namespace
