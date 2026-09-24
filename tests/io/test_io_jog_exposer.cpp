/**
 * @file test_io_jog_exposer.cpp
 * @brief Unit tests for JogExposer: registered jog functions and per-axis
 *        state signals wired through the IO registry.
 */

#include <gtest/gtest.h>

#include "tether/control/JogController.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/exposers/JogExposer.hpp"

#include <cstring>
#include <string>

using namespace tether::io;
using namespace tether::io::exposers;
using namespace tether::control;

namespace {

JogController::Config testConfig() {
    JogController::Config cfg;
    JogController::AxisConfig x;
    x.name = "x";
    x.maxRate = 50.0;
    x.maxLeaseMs = 500;
    x.maxIncrement = 10.0;
    JogController::AxisConfig e;
    e.name = "e";
    e.maxRate = 5.0;
    cfg.axes = {x, e};
    cfg.groups = {JogController::Group{"linear", {0}, {}, {}},
                  JogController::Group{"extruder", {1}, {0.5, 1.0}, {}}};
    return cfg;
}

FunctionArgument f64Arg(uint32_t pos, double v) {
    FunctionArgument a;
    a.position = pos;
    a.type = ValueType::F64;
    a.value.resize(sizeof(double));
    std::memcpy(a.value.data(), &v, sizeof(double));
    return a;
}

FunctionArgument u32Arg(uint32_t pos, uint32_t v) {
    FunctionArgument a;
    a.position = pos;
    a.type = ValueType::U32;
    a.value.resize(sizeof(uint32_t));
    std::memcpy(a.value.data(), &v, sizeof(uint32_t));
    return a;
}

class JogExposerTest : public ::testing::Test {
protected:
    static constexpr uint64_t kBase = 0x60000000;

    void SetUp() override {
        exposer_ = std::make_unique<JogExposer>(jog_, [this] { return now_; });
        exposer_->expose(registry_, kBase);
    }

    FunctionView fn(uint32_t localId) {
        return registry_.findFunction(makeId(kBase, localId));
    }

    JogController jog_{testConfig()};
    int64_t now_ = 0;
    Registry registry_;
    std::unique_ptr<JogExposer> exposer_;
};

// Local-id layout mirrors JogExposer: 1=describe, 2=stop_all,
// per-axis block at 0x100 + i*16 (+0 jog, +1 move, +2 stop, +3 snapshot).
constexpr uint32_t kAxis0 = 0x100;
constexpr uint32_t kAxis1 = 0x110;

TEST_F(JogExposerTest, DescribeReturnsJsonMetadata) {
    FunctionView describe = fn(1);
    ASSERT_TRUE(static_cast<bool>(describe));
    EXPECT_EQ(describe.name(), std::string_view("jog.describe"));

    auto result = describe.invoke({});
    ASSERT_TRUE(result.success);
    const std::string json(result.returnValue.begin(),
                           result.returnValue.end());
    EXPECT_NE(json.find("\"name\":\"x\""), std::string::npos);
    EXPECT_NE(json.find("\"name\":\"e\""), std::string::npos);
    EXPECT_NE(json.find("\"name\":\"linear\""), std::string::npos);
    EXPECT_NE(json.find("\"name\":\"extruder\""), std::string::npos);
}

TEST_F(JogExposerTest, JogFunctionActivatesAxisWithLease) {
    FunctionView jogFn = fn(kAxis0 + 0);
    ASSERT_TRUE(static_cast<bool>(jogFn));
    EXPECT_EQ(jogFn.name(), std::string_view("jog.x.jog"));
    ASSERT_EQ(jogFn.parameterCount(), 2u);

    auto result = jogFn.invoke({f64Arg(0, 25.0), u32Arg(1, 300)});
    ASSERT_TRUE(result.success);
    EXPECT_EQ(jog_.axisMode(0), JogController::Mode::Continuous);
    EXPECT_EQ(jog_.axisLeaseLeftMs(0, now_), 300u);

    // Advance: rate ramps toward 25.
    for (int i = 0; i < 100; ++i) jog_.update(++now_, 0.001);
    EXPECT_NEAR(jog_.axisRate(0), 25.0, 1e-6);

    // Lease expires without refresh — axis coasts to a stop.
    for (int i = 0; i < 500; ++i) jog_.update(++now_, 0.001);
    EXPECT_EQ(jog_.axisMode(0), JogController::Mode::Idle);
}

TEST_F(JogExposerTest, MoveFunctionReturnsAppliedDistance) {
    FunctionView moveFn = fn(kAxis0 + 1);
    ASSERT_TRUE(static_cast<bool>(moveFn));

    auto result = moveFn.invoke({f64Arg(0, 4.0)});
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.returnValue.size(), sizeof(double));
    double applied = 0.0;
    std::memcpy(&applied, result.returnValue.data(), sizeof(double));
    EXPECT_DOUBLE_EQ(applied, 4.0);
    EXPECT_EQ(jog_.axisMode(0), JogController::Mode::Incremental);
}

TEST_F(JogExposerTest, MoveRejectedWhileJoggingReturnsError) {
    ASSERT_TRUE(fn(kAxis0 + 0)
                    .invoke({f64Arg(0, 10.0), u32Arg(1, 300)})
                    .success);
    auto result = fn(kAxis0 + 1).invoke({f64Arg(0, 1.0)});
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, ErrorCode::ResourceBusy);
}

TEST_F(JogExposerTest, StopFunctionsHaltMotion) {
    ASSERT_TRUE(fn(kAxis0 + 0)
                    .invoke({f64Arg(0, 20.0), u32Arg(1, 400)})
                    .success);
    ASSERT_TRUE(fn(kAxis1 + 0)
                    .invoke({f64Arg(0, 5.0), u32Arg(1, 200)})
                    .success);

    ASSERT_TRUE(fn(2).invoke({}).success);  // jog.stop_all
    EXPECT_EQ(jog_.axisMode(0), JogController::Mode::Stopping);
    EXPECT_EQ(jog_.axisMode(1), JogController::Mode::Stopping);
}

TEST_F(JogExposerTest, StateSignalsReflectAxis) {
    ASSERT_TRUE(fn(kAxis0 + 0)
                    .invoke({f64Arg(0, 15.0), u32Arg(1, 400)})
                    .success);
    for (int i = 0; i < 100; ++i) jog_.update(++now_, 0.001);

    // Binary snapshot at +3.
    EntryView snap = registry_.findSignal(makeId(kBase, kAxis0 + 3));
    ASSERT_TRUE(static_cast<bool>(snap));
    EXPECT_EQ(snap.name(), std::string_view("jog.x.state"));
    JogController::AxisSnapshot buf{};
    ASSERT_EQ(snap.readVar(&buf, sizeof(buf)), sizeof(buf));
    EXPECT_EQ(buf.mode,
              static_cast<uint8_t>(JogController::Mode::Continuous));
    EXPECT_EQ(buf.active, 1);
    EXPECT_GT(buf.leaseLeftMs, 0u);

    // Scalar fields at +4.. : mode, active, rate, position, remaining,
    // leaseLeftMs.
    uint8_t mode = 0;
    registry_.findSignal(makeId(kBase, kAxis0 + 4)).read(&mode);
    EXPECT_EQ(mode,
              static_cast<uint8_t>(JogController::Mode::Continuous));
    double rate = 0.0;
    registry_.findSignal(makeId(kBase, kAxis0 + 6)).read(&rate);
    EXPECT_NEAR(rate, 15.0, 1e-6);
    uint32_t lease = 0;
    registry_.findSignal(makeId(kBase, kAxis0 + 9)).read(&lease);
    EXPECT_GT(lease, 0u);
}

} // namespace
