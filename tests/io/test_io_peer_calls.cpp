/**
 * @file test_io_peer_calls.cpp
 * @brief Tests for symmetric (peer) function invocation: RegisterFunctions,
 *        InvokeEx request/response correlation, deadlines, and the
 *        consumer-side DeadlineLatch.
 *
 * Topology under test: a Session (server side) and a TetherIOClient
 * (client side) connected over MessagePipeTransport with Framing::None.
 * The client hosts functions via registerFunctions(); the session issues
 * reverse calls via callPeer().  Forward calls go through invokeEx().
 */
#include <gtest/gtest.h>
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/DeadlineLatch.hpp"
#include "PipeTransport.hpp"
#include <cstdarg>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <future>

using namespace tether::io;
using namespace tether::io::testing;
using namespace std::chrono_literals;

static void noopLogFn(const char*, const char*, ...) {}
static void stderrLogFn(const char* tag, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    fprintf(stderr, "[io-test %s] %s\n", tag, buf);
}

static uint64_t nowUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

// ===========================================================================
// Fixture
// ===========================================================================

class IOPeerCallsTest : public ::testing::Test {
protected:
    Registry registry_;
    FeatureSet features_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<TetherIOClient> client_;
    std::thread sessionThread_;
    std::thread pumpThread_;
    std::atomic<bool> pumpRunning_{false};

    void SetUp() override {
        // Server-side function: add(a: U32, b: U32=2) -> sum: U32
        {
            FunctionEntry fn;
            fn.id = 100;
            fn.name = "add";
            fn.parameters = {
                {"a", "First operand", ValueType::U32},
                {"b", "Second operand", ValueType::U32, true, true, {2, 0, 0, 0}},
            };
            fn.returnValue.present = true;
            fn.returnValue.type = ValueType::U32;
            fn.callback = [](const std::vector<FunctionArgument>& args) {
                FunctionCallResult result;
                uint32_t a = 0, b = 0;
                for (const auto& arg : args) {
                    uint32_t v = 0;
                    std::memcpy(&v, arg.value.data(), 4);
                    if (arg.position == 0) a = v; else b = v;
                }
                uint32_t sum = a + b;
                result.success = true;
                result.returnValue.resize(4);
                std::memcpy(result.returnValue.data(), &sum, 4);
                return result;
            };
            registry_.addFunction(std::move(fn));
        }

        auto [clientEnd, serverEnd] = MessagePipeTransport::create();
        session_ = std::make_unique<Session>(
            std::move(serverEnd), registry_, nowUs, noopLogFn,
            &features_, nullptr, nullptr, nullptr, nullptr, nullptr,
            Framing::None);
        sessionThread_ = std::thread([this] { session_->run(); });
        client_ = std::make_unique<TetherIOClient>(std::move(clientEnd), 5000);

        for (int i = 0; i < 50; ++i) {
            auto result = client_->ping(42);
            if (result && *result == 42) break;
            std::this_thread::sleep_for(10ms);
        }
    }

    void TearDown() override {
        stopPump();
        if (client_) client_->close();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }

    /// Receive pump: services incoming InvokeExReq while the test thread
    /// does other work.  Dispatch happens inside receiveMessage(), so a
    /// client without a pump cannot answer reverse calls.
    void startPump() {
        pumpRunning_ = true;
        pumpThread_ = std::thread([this] {
            while (pumpRunning_) {
                auto msg = client_->receiveMessage(50);
                (void)msg;
            }
        });
    }

    void stopPump() {
        pumpRunning_ = false;
        if (pumpThread_.joinable()) pumpThread_.join();
    }

    static std::vector<uint8_t> encodeU32(uint32_t v) {
        std::vector<uint8_t> bytes(4);
        std::memcpy(bytes.data(), &v, 4);
        return bytes;
    }
    static uint32_t decodeU32(const std::vector<uint8_t>& bytes) {
        uint32_t v = 0;
        if (bytes.size() >= 4) std::memcpy(&v, bytes.data(), 4);
        return v;
    }

    /// A client-hosted function: scale(x: F64) -> 2x: F64
    static FunctionEntry makeScaleFunction(uint64_t id, double factor = 2.0) {
        FunctionEntry fn;
        fn.id = id;
        fn.name = "scale";
        fn.description = "Scale an F64";
        fn.group = "peer";
        fn.parameters = {{"x", "Input", ValueType::F64}};
        fn.returnValue.present = true;
        fn.returnValue.type = ValueType::F64;
        fn.callback = [factor](const std::vector<FunctionArgument>& args) {
            FunctionCallResult result;
            double x = 0;
            for (const auto& arg : args) {
                if (arg.position == 0 && arg.value.size() == 8) {
                    std::memcpy(&x, arg.value.data(), 8);
                }
            }
            double y = x * factor;
            result.success = true;
            result.returnValue.resize(8);
            std::memcpy(result.returnValue.data(), &y, 8);
            return result;
        };
        return fn;
    }

    bool waitForCatalogSize(size_t n, int timeoutMs = 2000) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeoutMs);
        while (std::chrono::steady_clock::now() < deadline) {
            if (session_->peerFunctions().size() == n) return true;
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }
};

// ===========================================================================
// Descriptor codec (unit)
// ===========================================================================

TEST(FunctionDescriptorCodec, RoundTrip) {
    FunctionEntry fn;
    fn.id = 42;
    fn.name = "filter";
    fn.description = "Torque filter";
    fn.group = "control";
    fn.metadata["role"] = "blackbox";

    FunctionParameter state;
    state.name = "state";
    state.description = "Robot state";
    state.type = ValueType::Struct;
    state.valueDescriptor = std::make_shared<ValueDescriptor>(
        ValueDescriptor::structure({
            {"qpos", ValueDescriptor::array(ValueDescriptor::scalar(ValueType::F64))},
            {"seq",  ValueDescriptor::scalar(ValueType::U64)},
        }));
    FunctionParameter scale;
    scale.name = "scale";
    scale.type = ValueType::F64;
    scale.optional = true;
    scale.hasDefault = true;
    scale.defaultValue.resize(8);
    double dv = 1.5;
    std::memcpy(scale.defaultValue.data(), &dv, 8);
    fn.parameters = {state, scale};

    fn.returnValue.present = true;
    fn.returnValue.name = "tau";
    fn.returnValue.type = ValueType::Array;
    fn.returnValue.valueDescriptor = std::make_shared<ValueDescriptor>(
        ValueDescriptor::array(ValueDescriptor::scalar(ValueType::F64)));

    std::vector<uint8_t> buf(4096);
    BufWriter w(buf.data(), buf.size());
    ASSERT_TRUE(encodeFunctionDescriptor(w, FunctionView(&fn)));

    BufReader r(buf.data(), w.pos);
    FunctionDescriptor d;
    ASSERT_TRUE(decodeFunctionDescriptor(r, d));
    EXPECT_EQ(r.remaining(), 0u);

    EXPECT_EQ(d.id, 42u);
    EXPECT_EQ(d.name, "filter");
    EXPECT_EQ(d.group, "control");
    ASSERT_EQ(d.parameters.size(), 2u);
    EXPECT_EQ(d.parameters[0].name, "state");
    EXPECT_TRUE(d.parameters[0].valueDescriptor &&
                d.parameters[0].valueDescriptor->type == ValueType::Struct);
    EXPECT_EQ(d.parameters[0].valueDescriptor->fields.size(), 2u);
    EXPECT_TRUE(d.parameters[1].optional);
    EXPECT_TRUE(d.parameters[1].hasDefault);
    ASSERT_EQ(d.parameters[1].defaultValue.size(), 8u);
    double decodedDefault = 0;
    std::memcpy(&decodedDefault, d.parameters[1].defaultValue.data(), 8);
    EXPECT_DOUBLE_EQ(decodedDefault, 1.5);
    EXPECT_TRUE(d.returnValue.present);
    EXPECT_EQ(d.returnValue.type, ValueType::Array);
    ASSERT_TRUE(d.returnValue.valueDescriptor &&
                d.returnValue.valueDescriptor->element);
    EXPECT_EQ(d.returnValue.valueDescriptor->element->type, ValueType::F64);
    EXPECT_EQ(d.metadata.at("role"), "blackbox");
}

// ===========================================================================
// DeadlineLatch (unit)
// ===========================================================================

TEST(DeadlineLatchTest, ConsumeBeforeOfferIsInvalid) {
    DeadlineLatch<int> latch;
    auto s = latch.consume(1000, 500);
    EXPECT_FALSE(s.valid);
    EXPECT_FALSE(s.fresh);
}

TEST(DeadlineLatchTest, FreshWithinDeadline) {
    DeadlineLatch<int> latch;
    latch.offer(1, 7, 1000);
    auto s = latch.consume(1300, 500);
    ASSERT_TRUE(s.valid);
    EXPECT_TRUE(s.fresh);
    EXPECT_EQ(s.seq, 1u);
    EXPECT_EQ(s.ageUs, 300u);
    EXPECT_EQ(s.value, 7);
}

TEST(DeadlineLatchTest, StalePastDeadline) {
    DeadlineLatch<int> latch;
    latch.offer(1, 7, 1000);
    auto s = latch.consume(2000, 500);
    ASSERT_TRUE(s.valid);
    EXPECT_FALSE(s.fresh);
    EXPECT_EQ(s.ageUs, 1000u);
}

TEST(DeadlineLatchTest, OutOfOrderOfferDropped) {
    DeadlineLatch<int> latch;
    latch.offer(5, 50, 1000);
    latch.offer(3, 30, 1100);   // stale response — must not win
    auto s = latch.consume(1200, 500);
    EXPECT_EQ(s.seq, 5u);
    EXPECT_EQ(s.value, 50);
}

TEST(DeadlineLatchTest, ResetDiscardsValue) {
    DeadlineLatch<int> latch;
    latch.offer(1, 7, 1000);
    latch.reset();
    EXPECT_FALSE(latch.consume(1100, 500).valid);
}

// ===========================================================================
// RegisterFunctions + callPeer (server → client)
// ===========================================================================

TEST_F(IOPeerCallsTest, RegisterFunctionsPopulatesSessionCatalog) {
    auto count = client_->registerFunctions({makeScaleFunction(500)});
    ASSERT_TRUE(count.has_value()) << count.error().message;
    EXPECT_EQ(*count, 1u);
    ASSERT_TRUE(waitForCatalogSize(1));
    auto catalog = session_->peerFunctions();
    EXPECT_EQ(catalog[0].id, 500u);
    EXPECT_EQ(catalog[0].name, "scale");
    EXPECT_EQ(catalog[0].parameters.size(), 1u);
    auto id = session_->findPeerFunctionId("scale");
    ASSERT_TRUE(id.has_value());
    EXPECT_EQ(*id, 500u);
    EXPECT_FALSE(session_->findPeerFunctionId("nonexistent").has_value());
}

TEST_F(IOPeerCallsTest, CallPeerSuccess) {
    ASSERT_TRUE(client_->registerFunctions({makeScaleFunction(500)}).has_value());
    ASSERT_TRUE(waitForCatalogSize(1));
    startPump();

    std::promise<Session::InvokeResult> promise;
    FunctionArgument arg;
    arg.position = 0;
    arg.type = ValueType::F64;
    arg.value.resize(8);
    double x = 3.5;
    std::memcpy(arg.value.data(), &x, 8);

    ASSERT_TRUE(session_->callPeer(500, {arg}, 2'000'000,
                [&promise](const Session::InvokeResult& r) { promise.set_value(r); }));
    auto result = promise.get_future().get();
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.timedOut);
    ASSERT_TRUE(result.hasReturnValue);
    double y = 0;
    std::memcpy(&y, result.returnValue.data(), 8);
    EXPECT_DOUBLE_EQ(y, 7.0);
    EXPECT_EQ(result.functionId, 500u);
    EXPECT_GT(result.rttUs, 0u);
}

TEST_F(IOPeerCallsTest, CallPeerUnknownFunction) {
    // No catalog registered: response comes back as "function not found"
    // — the client-side dispatch answers with a correlated error.
    startPump();
    std::promise<Session::InvokeResult> promise;
    ASSERT_TRUE(session_->callPeer(999, {}, 2'000'000,
                [&promise](const Session::InvokeResult& r) { promise.set_value(r); }));
    auto result = promise.get_future().get();
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.timedOut);
    EXPECT_EQ(result.error, ErrorCode::InvalidId);
}

TEST_F(IOPeerCallsTest, CallPeerDeadlineExpires) {
    // Slow responder: callback sleeps past the deadline — the session
    // sweeps the pending call and reports a timeout; the late response
    // is dropped afterwards.
    FunctionEntry slow = makeScaleFunction(500);
    slow.callback = [](const std::vector<FunctionArgument>&) {
        std::this_thread::sleep_for(300ms);
        FunctionCallResult result;
        result.success = true;
        result.returnValue.resize(8);
        return result;
    };
    ASSERT_TRUE(client_->registerFunctions({slow}).has_value());
    ASSERT_TRUE(waitForCatalogSize(1));
    startPump();

    FunctionArgument arg;
    arg.position = 0;
    arg.type = ValueType::F64;
    arg.value.resize(8);

    std::promise<Session::InvokeResult> promise;
    ASSERT_TRUE(session_->callPeer(500, {arg}, 50'000,
                [&promise](const Session::InvokeResult& r) { promise.set_value(r); }));
    auto future = promise.get_future();
    ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
    auto result = future.get();
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.timedOut);
    EXPECT_EQ(result.error, ErrorCode::Timeout);
    EXPECT_EQ(session_->pendingInvokeCount(), 0u);
}

TEST_F(IOPeerCallsTest, PipelinedCallPeerAllComplete) {
    ASSERT_TRUE(client_->registerFunctions({makeScaleFunction(500)}).has_value());
    ASSERT_TRUE(waitForCatalogSize(1));
    startPump();

    constexpr int kCalls = 8;
    std::vector<std::promise<Session::InvokeResult>> promises(kCalls);
    std::vector<std::future<Session::InvokeResult>> futures;
    for (int i = 0; i < kCalls; ++i) futures.push_back(promises[i].get_future());

    FunctionArgument arg;
    arg.position = 0;
    arg.type = ValueType::F64;
    arg.value.resize(8);
    for (int i = 0; i < kCalls; ++i) {
        double x = static_cast<double>(i);
        std::memcpy(arg.value.data(), &x, 8);
        ASSERT_TRUE(session_->callPeer(500, {arg}, 2'000'000,
            [&promises, i](const Session::InvokeResult& r) {
                promises[i].set_value(r);
            }));
    }
    for (int i = 0; i < kCalls; ++i) {
        ASSERT_EQ(futures[i].wait_for(2s), std::future_status::ready);
        EXPECT_TRUE(futures[i].get().success);
    }
}

TEST_F(IOPeerCallsTest, NormalRequestsProceedWhilePeerCallPending) {
    // A slow peer call must not head-of-line block unrelated requests:
    // while callPeer is outstanding, a PingReq on the same connection is
    // answered.  (Concurrent-with-other-request-IDs requirement.)
    FunctionEntry slow = makeScaleFunction(500);
    slow.callback = [](const std::vector<FunctionArgument>&) {
        std::this_thread::sleep_for(200ms);
        FunctionCallResult result;
        result.success = true;
        result.returnValue.resize(8);
        return result;
    };
    ASSERT_TRUE(client_->registerFunctions({slow}).has_value());
    ASSERT_TRUE(waitForCatalogSize(1));
    startPump();

    FunctionArgument arg;
    arg.position = 0;
    arg.type = ValueType::F64;
    arg.value.resize(8);

    std::promise<Session::InvokeResult> promise;
    ASSERT_TRUE(session_->callPeer(500, {arg}, 2'000'000,
                [&promise](const Session::InvokeResult& r) { promise.set_value(r); }));

    auto ping = client_->ping(77);
    ASSERT_TRUE(ping.has_value()) << ping.error().message;
    EXPECT_EQ(*ping, 77u);

    auto future = promise.get_future();
    ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
    EXPECT_TRUE(future.get().success);
}

// ===========================================================================
// invokeEx (client → server, correlated)
// ===========================================================================

TEST_F(IOPeerCallsTest, InvokeExCallsServerFunction) {
    std::vector<TetherIOClient::FunctionArg> args = {
        {0, ValueType::U32, encodeU32(40)},
        // b omitted: optional with default 2
    };
    auto result = client_->invokeEx(100, args);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_TRUE(result->success);
    ASSERT_TRUE(result->hasReturnValue);
    EXPECT_EQ(decodeU32(result->returnValue), 42u);
    EXPECT_EQ(result->functionId, 100u);
}

TEST_F(IOPeerCallsTest, InvokeExUnknownFunctionReturnsCorrelatedError) {
    auto result = client_->invokeEx(9999);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_FALSE(result->success);
    EXPECT_EQ(result->errorCode, static_cast<uint32_t>(ErrorCode::InvalidId));
}

TEST_F(IOPeerCallsTest, InvokeExArgumentValidation) {
    // Wrong type for a U32 parameter → correlated invocation error.
    auto result = client_->invokeEx(100, {{0, ValueType::F64, {0,0,0,0,0,0,0,0}}});
    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->success);
    EXPECT_EQ(result->errorCode,
              static_cast<uint32_t>(ErrorCode::FunctionInvocationError));
}
