// ErrorPolicy + TransactionOptions defaults/override merge tests.
#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/link/TestLinks.hpp"

using namespace tether::modbus;

namespace {
std::vector<uint8_t> rtuFrame(uint8_t unit, uint8_t fc,
                              std::vector<uint8_t> body) {
    std::vector<uint8_t> f(body.size() + 4);
    f[0] = unit; f[1] = fc;
    std::copy(body.begin(), body.end(), f.begin() + 2);
    const uint16_t c = Crc16::of(std::span(f.data(), f.size() - 2));
    f[f.size() - 2] = uint8_t(c); f[f.size() - 1] = uint8_t(c >> 8);
    return f;
}
const RtuTiming kFast = RtuTiming::forParams(1000000, 10);
const std::vector<uint8_t> kReadBody{0x00, 0x00, 0x00, 0x01};

/// RtuChannel + BusExecutor over a ScriptedLink; returns executor.
struct Rig {
    std::unique_ptr<ScriptedLink> linkOwner; // keep alive via channel
    ScriptedLink* link = nullptr;
    std::unique_ptr<RtuChannel> ch;
    std::unique_ptr<BusExecutor> ex;
    Rig() {
        auto l = std::make_unique<ScriptedLink>();
        link = l.get();
        ch = std::make_unique<RtuChannel>(std::move(l), kFast);
        ch->open();
        ex = std::make_unique<BusExecutor>(*ch);
    }
    std::vector<uint8_t> corrupt() {
        auto f = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
        f.back() ^= 0xFF;
        return f;
    }
    std::vector<uint8_t> good() {
        return rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    }
};
} // namespace

// ---------- connection default: Retry ×8 then fail ----------

TEST(ErrorPolicy, DefaultRetriesCorruptThenSucceeds) {
    Rig r;
    r.link->push(r.corrupt());               // attempt 1 → corrupt
    TransactionOptions o{.timeoutMs = 30};
    // Stage each retry's response inside onRetry: push() is released on
    // the next write() — deterministic per-attempt scripting.
    int n = 0;
    o.onRetry = [&](int, const ModbusError&) {
        r.link->push(++n < 8 ? r.corrupt() : r.good());
    };
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, o);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(r.link->txLog().size(), 9u);   // 1 attempt + 8 retries
    EXPECT_EQ(r.ex->stats().corrupt, 8);
    EXPECT_EQ(r.ex->stats().retries, 8);
}

TEST(ErrorPolicy, DefaultExhaustsThenFails) {
    Rig r;
    r.link->push(r.corrupt());
    TransactionOptions o{.timeoutMs = 30};
    o.onRetry = [&](int, const ModbusError&) {
        r.link->push(r.corrupt());           // never answers cleanly
    };
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, o);
    ASSERT_FALSE(p.has_value());
    EXPECT_TRUE(p.error().isCorrupt());
    EXPECT_EQ(r.link->txLog().size(), 9u);   // gave up after 9 attempts
    EXPECT_EQ(r.ex->stats().corrupt, 9);
}

TEST(ErrorPolicy, DefaultRetriesTimeoutToo) {
    Rig r;                                  // nothing staged → silence
    TransactionOptions o{.timeoutMs = 20};
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, o);
    ASSERT_FALSE(p.has_value());
    EXPECT_TRUE(p.error().isTimeout());
    EXPECT_EQ(r.link->txLog().size(), 9u);   // timeouts also retry ×8
    EXPECT_EQ(r.ex->stats().timeouts, 9);
}

// ---------- user-selectable strategies ----------

TEST(ErrorPolicy, FailFastSingleAttempt) {
    Rig r;
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::FailFast});
    r.link->push(r.corrupt());
    r.link->push(r.good());
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, {.timeoutMs = 30});
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(r.link->txLog().size(), 1u);
    EXPECT_EQ(r.ex->stats().retries, 0);
}

TEST(ErrorPolicy, ConfigurableRetryCount) {
    Rig r;
    ErrorPolicy p{.strategy = ErrorStrategy::Retry, .maxRetries = 2};
    r.ex->setErrorPolicy(p);
    r.link->push(r.corrupt());
    TransactionOptions o{.timeoutMs = 30};
    o.onRetry = [&](int, const ModbusError&) {
        r.link->push(r.corrupt());
    };
    auto rsp = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                            kReadBody, o);
    ASSERT_FALSE(rsp.has_value());
    EXPECT_EQ(r.link->txLog().size(), 3u);   // 1 + 2
}

TEST(ErrorPolicy, PerCauseToggleCorruptOnly) {
    Rig r;
    // Retry corruption but not timeouts.
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::Retry,
                          .retryOnTimeout = false});
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, {.timeoutMs = 20});
    ASSERT_FALSE(p.has_value());
    EXPECT_TRUE(p.error().isTimeout());
    EXPECT_EQ(r.link->txLog().size(), 1u);   // timeout not retried
}

TEST(ErrorPolicy, PerCauseToggleTimeoutOnly) {
    Rig r;
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::Retry,
                          .retryOnCorrupt = false});
    r.link->push(r.corrupt());
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, {.timeoutMs = 30});
    ASSERT_FALSE(p.has_value());
    EXPECT_TRUE(p.error().isCorrupt());
    EXPECT_EQ(r.link->txLog().size(), 1u);   // corruption not retried
}

TEST(ErrorPolicy, CircuitBreakerOpensAndRejects) {
    Rig r;
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::CircuitBreaker,
                          .maxRetries = 0,
                          .circuitThreshold = 2,
                          .circuitResetMs = 40});
    TransactionOptions o{.timeoutMs = 20};
    // Two failing invokes → circuit opens (threshold 2).
    EXPECT_FALSE(r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                              kReadBody, o).has_value());
    EXPECT_FALSE(r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                              kReadBody, o).has_value());
    EXPECT_EQ(r.ex->stats().circuitTrips, 1);
    const auto writes = r.link->txLog().size();
    // Third call rejected instantly — no wire traffic.
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, o);
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(r.ex->stats().circuitRejected, 1);
    EXPECT_EQ(r.link->txLog().size(), writes);
    // After circuitResetMs a half-open probe goes through.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    r.link->push(r.good());
    p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters, kReadBody, o);
    ASSERT_TRUE(p.has_value());
}

// ---------- TransactionOptions: connection defaults + overrides ----------

TEST(TransactionOpts, OverlayFieldWise) {
    TransactionOptions base{.timeoutMs = 100, .retries = 3};
    auto a = overlayOpts(base, TransactionOptions{.retries = 0});
    EXPECT_EQ(a.timeoutMs, 100);   // unset in over → inherited
    EXPECT_EQ(a.retries, 0);       // explicit 0 wins (disables retry)
    auto b = overlayOpts(base, TransactionOptions{});
    EXPECT_EQ(b.timeoutMs, 100);
    EXPECT_EQ(b.retries, 3);
    auto c = overlayOpts(TransactionOptions{},
                         TransactionOptions{.timeoutMs = 50});
    EXPECT_EQ(c.timeoutMs, 50);
    EXPECT_EQ(c.retries, -1);      // still unset → deeper default
}

TEST(TransactionOpts, ConnectionDefaultApplies) {
    Rig r;
    r.ex->setDefaultOptions({.timeoutMs = 25, .retries = 1});
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody);          // no call options
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(r.link->txLog().size(), 2u);     // 1 + connection retry
    EXPECT_EQ(r.ex->stats().timeouts, 2);
}

TEST(TransactionOpts, PerCallRetriesOverride) {
    Rig r;
    r.ex->setDefaultOptions({.retries = 5});   // connection default
    // Per-call retries=0 disables retries entirely for this call.
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, {.timeoutMs = 20, .retries = 0});
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(r.link->txLog().size(), 1u);
}

TEST(TransactionOpts, PerCallPolicyOverride) {
    Rig r;
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::FailFast});
    // Per-call full-policy override beats the connection default.
    TransactionOptions o{.timeoutMs = 30};
    o.errorPolicy = ErrorPolicy{.strategy = ErrorStrategy::Retry,
                                .maxRetries = 1};
    r.link->push(r.corrupt());
    o.onRetry = [&](int, const ModbusError&) { r.link->push(r.good()); };
    auto p = r.ex->invoke(1, FunctionCode::ReadHoldingRegisters,
                          kReadBody, o);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(r.link->txLog().size(), 2u);
}

TEST(TransactionOpts, ClientDefaultsPropagate) {
    Rig r;
    // Client-level defaults layer between call opts + connection opts.
    ModbusClient cli(*r.ex, 1, TransactionOptions{.timeoutMs = 20,
                                                  .retries = 1});
    auto v = cli.readHolding(0, 1);            // silent → timeout path
    ASSERT_FALSE(v.has_value());
    EXPECT_EQ(r.link->txLog().size(), 2u);     // client default retries=1
    // Per-call override beats client default.
    v = cli.readHolding(0, 1, {.retries = 0});
    EXPECT_EQ(r.link->txLog().size(), 3u);     // +1 attempt only
}

TEST(TransactionOpts, DefaultOptionsGetter) {
    Rig r;
    r.ex->setDefaultOptions({.timeoutMs = 77});
    EXPECT_EQ(r.ex->defaultOptions().timeoutMs, 77);
    // errorPolicy sugar: setErrorPolicy lands inside defaults_.
    r.ex->setErrorPolicy({.strategy = ErrorStrategy::FailFast});
    EXPECT_EQ(r.ex->errorPolicy().strategy, ErrorStrategy::FailFast);
    EXPECT_EQ(r.ex->defaultOptions().errorPolicy->strategy,
              ErrorStrategy::FailFast);
}
