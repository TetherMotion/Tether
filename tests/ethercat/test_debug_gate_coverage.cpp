/**
 * @file test_debug_gate_coverage.cpp
 * @brief Coverage for the conditional-debug framework (DebugGate.cpp,
 *        DebugConditions.cpp) and the CLI helpers (DebugHelpers.cpp).
 *
 * Everything here is pure logic except the two Master-taking helpers
 * (applyFlags / applyGateConditions), which run against an emulated bus.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/DebugHelpers.hpp"
#include "tether/ethercat/Master.hpp"

#if TETHER_DEBUG_GATE_ENABLED

using namespace EtherCAT;

namespace {

// Small helper: encode a value little-endian into a byte buffer.
std::vector<uint8_t> le(uint64_t v, size_t n) {
    std::vector<uint8_t> d(n);
    for (size_t i = 0; i < n; ++i) d[i] = static_cast<uint8_t>(v >> (8 * i));
    return d;
}

}  // namespace

// ============================================================================
// parseCondition — spec grammar
// ============================================================================

TEST(DebugGateParseTest, EmptySpecReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition(""), nullptr);
}

TEST(DebugGateParseTest, StateConditionParsesAllAliases) {
    struct Case { const char* spec; SlaveState want; };
    const Case cases[] = {
        {"state:init", SlaveState::INIT},
        {"state:INIT", SlaveState::INIT},
        {"state:pre-op", SlaveState::PRE_OP},
        {"state:preop", SlaveState::PRE_OP},
        {"state:PRE-OP", SlaveState::PRE_OP},
        {"state:safe-op", SlaveState::SAFE_OP},
        {"state:safeop", SlaveState::SAFE_OP},
        {"state:op", SlaveState::OP},
        {"state:OP", SlaveState::OP},
        {"state:boot", SlaveState::BOOT},
    };
    for (const auto& c : cases) {
        auto cond = DebugGate::parseCondition(c.spec);
        ASSERT_NE(cond, nullptr) << c.spec;
        EXPECT_EQ(cond->type(), DebugConditionType::State) << c.spec;
        auto* sc = static_cast<StateCondition*>(cond.get());
        EXPECT_EQ(sc->state(), c.want) << c.spec;
        EXPECT_EQ(sc->slaveIndex(), 0xFFFFu) << c.spec;
    }
}

TEST(DebugGateParseTest, StateConditionWithSlaveIndex) {
    auto cond = DebugGate::parseCondition("state:op:2");
    ASSERT_NE(cond, nullptr);
    auto* sc = static_cast<StateCondition*>(cond.get());
    EXPECT_EQ(sc->state(), SlaveState::OP);
    EXPECT_EQ(sc->slaveIndex(), 2u);
}

TEST(DebugGateParseTest, StateConditionUnknownStateReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition("state:bogus"), nullptr);
}

TEST(DebugGateParseTest, StateConditionMissingStateReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition("state:"), nullptr);
}

TEST(DebugGateParseTest, CheckpointConditionParses) {
    auto cond = DebugGate::parseCondition("checkpoint:before-pdo");
    ASSERT_NE(cond, nullptr);
    EXPECT_EQ(cond->type(), DebugConditionType::Checkpoint);
    auto* cc = static_cast<CheckpointCondition*>(cond.get());
    EXPECT_EQ(cc->checkpointName(), "before-pdo");
}

TEST(DebugGateParseTest, CheckpointConditionWithSlaveIndex) {
    auto cond = DebugGate::parseCondition("checkpoint:cp:3");
    ASSERT_NE(cond, nullptr);
    EXPECT_EQ(cond->type(), DebugConditionType::Checkpoint);
}

TEST(DebugGateParseTest, CheckpointConditionMissingNameReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition("checkpoint:"), nullptr);
}

TEST(DebugGateParseTest, RegisterConditionAddressFirstForm) {
    auto cond = DebugGate::parseCondition("reg:0x0130:==:0x0002");
    ASSERT_NE(cond, nullptr);
    EXPECT_EQ(cond->type(), DebugConditionType::RegisterIntercept);

    auto* rc = static_cast<RegisterInterceptCondition*>(cond.get());
    auto data = le(0x0002, 2);
    rc->onRegisterRead(0, 0x0130, data.data(), 2);
    EXPECT_TRUE(rc->hasFired());
}

TEST(DebugGateParseTest, RegisterConditionSlaveFirstForm) {
    auto cond = DebugGate::parseCondition("reg:0:0x0130:>=:2");
    ASSERT_NE(cond, nullptr);

    auto* rc = static_cast<RegisterInterceptCondition*>(cond.get());
    auto data = le(5, 2);
    rc->onRegisterRead(1, 0x0130, data.data(), 2);  // wrong slave
    EXPECT_FALSE(rc->hasFired());
    rc->onRegisterRead(0, 0x0130, data.data(), 2);  // right slave
    EXPECT_TRUE(rc->hasFired());
}

TEST(DebugGateParseTest, RegisterConditionTooShortReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition("reg:0x0130"), nullptr);
}

TEST(DebugGateParseTest, CoEConditionWithSubIndex) {
    auto cond = DebugGate::parseCondition("coe:0x6051:0x00:==:0x0001");
    ASSERT_NE(cond, nullptr);
    EXPECT_EQ(cond->type(), DebugConditionType::CoEIntercept);

    auto* cc = static_cast<CoEInterceptCondition*>(cond.get());
    auto data = le(1, 2);
    cc->onCoERead(0, 0x6051, 1, data.data(), 2);  // wrong sub
    EXPECT_FALSE(cc->hasFired());
    cc->onCoERead(0, 0x6051, 0, data.data(), 2);  // right sub
    EXPECT_TRUE(cc->hasFired());
}

TEST(DebugGateParseTest, CoEConditionWithoutSubIndexMatchesAnySub) {
    auto cond = DebugGate::parseCondition("coe:0x6051:==:0x0001");
    ASSERT_NE(cond, nullptr);

    auto* cc = static_cast<CoEInterceptCondition*>(cond.get());
    auto data = le(1, 2);
    cc->onCoERead(0, 0x6051, 7, data.data(), 2);
    EXPECT_TRUE(cc->hasFired());
}

TEST(DebugGateParseTest, CoEConditionSlaveFirstForm) {
    auto cond = DebugGate::parseCondition("coe:0:0x6051:0x00:==:0x0001");
    ASSERT_NE(cond, nullptr);

    auto* cc = static_cast<CoEInterceptCondition*>(cond.get());
    auto data = le(1, 2);
    cc->onCoERead(1, 0x6051, 0, data.data(), 2);  // wrong slave
    EXPECT_FALSE(cc->hasFired());
    cc->onCoERead(0, 0x6051, 0, data.data(), 2);
    EXPECT_TRUE(cc->hasFired());
}

TEST(DebugGateParseTest, UnknownPrefixReturnsNull) {
    EXPECT_EQ(DebugGate::parseCondition("bogus:foo"), nullptr);
}

// ============================================================================
// Condition classes
// ============================================================================

TEST(DebugConditionTest, StateConditionFiresOnMatchingCheckpoint) {
    StateCondition cond(SlaveState::PRE_OP);
    EXPECT_EQ(cond.type(), DebugConditionType::State);
    EXPECT_FALSE(cond.hasFired());

    cond.onCheckpoint("state:op", 0xFFFF);
    EXPECT_FALSE(cond.hasFired());

    cond.onCheckpoint("state:pre-op", 0xFFFF);
    EXPECT_TRUE(cond.hasFired());

    cond.reset();
    EXPECT_FALSE(cond.hasFired());
}

TEST(DebugConditionTest, StateConditionSlaveFilter) {
    StateCondition cond(SlaveState::OP, 2);
    cond.onCheckpoint("state:op", 3);
    EXPECT_FALSE(cond.hasFired());
    cond.onCheckpoint("state:op", 2);
    EXPECT_TRUE(cond.hasFired());
}

TEST(DebugConditionTest, CheckpointConditionFiresOnName) {
    CheckpointCondition cond("my-cp");
    EXPECT_EQ(cond.type(), DebugConditionType::Checkpoint);
    cond.onCheckpoint("other", 0xFFFF);
    EXPECT_FALSE(cond.hasFired());
    cond.onCheckpoint("my-cp", 0xFFFF);
    EXPECT_TRUE(cond.hasFired());
    cond.reset();
    EXPECT_FALSE(cond.hasFired());
}

TEST(DebugConditionTest, RegisterConditionCompareOps) {
    struct Case { CompareOp op; uint64_t value; uint16_t read; bool want; };
    const Case cases[] = {
        {CompareOp::Eq, 2, 2, true},
        {CompareOp::Eq, 2, 3, false},
        {CompareOp::Ne, 2, 3, true},
        {CompareOp::Ne, 2, 2, false},
        {CompareOp::Bitmask, 0x04, 0x06, true},
        {CompareOp::Bitmask, 0x04, 0x02, false},
        {CompareOp::Ge, 5, 5, true},
        {CompareOp::Ge, 5, 4, false},
        {CompareOp::Le, 5, 4, true},
        {CompareOp::Le, 5, 6, false},
        {CompareOp::Gt, 5, 6, true},
        {CompareOp::Gt, 5, 5, false},
        {CompareOp::Lt, 5, 4, true},
        {CompareOp::Lt, 5, 5, false},
    };
    for (const auto& c : cases) {
        RegisterInterceptCondition cond(0x0130, c.op, c.value);
        auto data = le(c.read, 2);
        cond.onRegisterRead(0, 0x0130, data.data(), 2);
        EXPECT_EQ(cond.hasFired(), c.want)
            << "op=" << static_cast<int>(c.op) << " read=" << c.read;
    }
}

TEST(DebugConditionTest, RegisterConditionGuardsAddrSlaveLen) {
    RegisterInterceptCondition cond(0x0130, CompareOp::Eq, 1, 0);
    auto data = le(1, 2);

    cond.onRegisterRead(0, 0x0131, data.data(), 2);  // wrong addr
    EXPECT_FALSE(cond.hasFired());
    cond.onRegisterRead(1, 0x0130, data.data(), 2);  // wrong slave
    EXPECT_FALSE(cond.hasFired());
    cond.onRegisterRead(0, 0x0130, nullptr, 2);      // null data
    EXPECT_FALSE(cond.hasFired());
    cond.onRegisterRead(0, 0x0130, data.data(), 0);  // zero len
    EXPECT_FALSE(cond.hasFired());
    cond.onRegisterRead(0, 0x0130, data.data(), 2);
    EXPECT_TRUE(cond.hasFired());
}

TEST(DebugConditionTest, RegisterConditionReadsLEValue) {
    // 4-byte LE read: 0x00000100 == 256
    RegisterInterceptCondition cond(0x0130, CompareOp::Eq, 0x100);
    auto data = le(0x100, 4);
    cond.onRegisterRead(0, 0x0130, data.data(), 4);
    EXPECT_TRUE(cond.hasFired());
}

TEST(DebugConditionTest, CoEConditionGuardsIndexAndSub) {
    CoEInterceptCondition cond(0x6051, 0, true, CompareOp::Eq, 1, 0);
    auto data = le(1, 2);

    cond.onCoERead(0, 0x6052, 0, data.data(), 2);  // wrong index
    EXPECT_FALSE(cond.hasFired());
    cond.onCoERead(0, 0x6051, 1, data.data(), 2);  // wrong sub
    EXPECT_FALSE(cond.hasFired());
    cond.onCoERead(1, 0x6051, 0, data.data(), 2);  // wrong slave
    EXPECT_FALSE(cond.hasFired());
    cond.onCoERead(0, 0x6051, 0, nullptr, 2);      // null data
    EXPECT_FALSE(cond.hasFired());
    cond.onCoERead(0, 0x6051, 0, data.data(), 2);
    EXPECT_TRUE(cond.hasFired());
}

TEST(DebugConditionTest, CoEConditionWithoutSubMatchesAnySub) {
    CoEInterceptCondition cond(0x6051, 0, false, CompareOp::Eq, 1);
    auto data = le(1, 2);
    cond.onCoERead(0, 0x6051, 9, data.data(), 2);
    EXPECT_TRUE(cond.hasFired());
}

TEST(DebugConditionTest, CustomConditionLatchesCallbackResult) {
    bool ready = false;
    int calls = 0;
    CustomCondition cond([&] { ++calls; return ready; });
    EXPECT_EQ(cond.type(), DebugConditionType::Custom);
    EXPECT_FALSE(cond.hasFired());  // callback false

    ready = true;
    EXPECT_TRUE(cond.hasFired());   // callback true
    const int latched_calls = calls;
    EXPECT_TRUE(cond.hasFired());   // latched: no further callback
    EXPECT_EQ(calls, latched_calls);

    cond.reset();
    ready = false;
    EXPECT_FALSE(cond.hasFired());
}

TEST(DebugConditionTest, CustomConditionWithoutCallbackNeverFires) {
    CustomCondition cond(nullptr);
    EXPECT_FALSE(cond.hasFired());
}

// ============================================================================
// DebugGate — start/stop gating
// ============================================================================

TEST(DebugGateTest, FreshGateIsActive) {
    DebugGate gate;
    EXPECT_FALSE(gate.hasAnyConditions());
    EXPECT_TRUE(gate.isActive("any"));
    EXPECT_TRUE(gate.isActiveForSlave(0));
}

TEST(DebugGateTest, GlobalStartClosesGateUntilFired) {
    DebugGate gate;
    gate.addGlobalStart(std::make_unique<StateCondition>(SlaveState::PRE_OP));
    EXPECT_TRUE(gate.hasAnyConditions());
    EXPECT_FALSE(gate.isActive("any"));

    gate.notifyCheckpoint("state:pre-op");
    EXPECT_TRUE(gate.isActive("any"));
}

TEST(DebugGateTest, GlobalStopClosesGate) {
    DebugGate gate;
    gate.addGlobalStart(std::make_unique<StateCondition>(SlaveState::PRE_OP));
    gate.addGlobalStop(std::make_unique<CheckpointCondition>("stop-here"));

    gate.notifyCheckpoint("state:pre-op");
    EXPECT_TRUE(gate.isActive("any"));

    gate.notifyCheckpoint("stop-here");
    EXPECT_FALSE(gate.isActive("any"));
}

TEST(DebugGateTest, PerFlagConditionsAreIndependent) {
    DebugGate gate;
    gate.addStartCondition("flagA",
                           std::make_unique<CheckpointCondition>("a-start"));
    EXPECT_FALSE(gate.isActive("flagA"));
    EXPECT_TRUE(gate.isActive("flagB"));  // no per-flag conditions → global

    gate.notifyCheckpoint("a-start");
    EXPECT_TRUE(gate.isActive("flagA"));
}

TEST(DebugGateTest, PerFlagStopClosesOnlyThatFlag) {
    DebugGate gate;
    gate.addStartCondition("flagA",
                           std::make_unique<CheckpointCondition>("a-start"));
    gate.addStopCondition("flagA",
                          std::make_unique<CheckpointCondition>("a-stop"));
    gate.notifyCheckpoint("a-start");
    EXPECT_TRUE(gate.isActive("flagA"));
    gate.notifyCheckpoint("a-stop");
    EXPECT_FALSE(gate.isActive("flagA"));
}

TEST(DebugGateTest, GateChangedCallbackFiresOnTransition) {
    DebugGate gate;
    int changes = 0;
    gate.setGateChangedCallback([&] { ++changes; });

    gate.addGlobalStart(std::make_unique<StateCondition>(SlaveState::OP));
    const int before = changes;
    gate.notifyCheckpoint("state:op");
    EXPECT_GT(changes, before);
}

TEST(DebugGateTest, NotifyCheckpointReachesRegisterAndCoEConditions) {
    DebugGate gate;
    auto reg = std::make_unique<RegisterInterceptCondition>(0x0130, CompareOp::Eq, 1);
    auto coe = std::make_unique<CoEInterceptCondition>(0x6051, 0, true, CompareOp::Eq, 1);
    auto* reg_raw = reg.get();
    auto* coe_raw = coe.get();
    gate.addGlobalStart(std::move(reg));
    gate.addGlobalStop(std::move(coe));

    auto data = le(1, 2);
    gate.onRegisterRead(0, 0x0130, data.data(), 2);
    EXPECT_TRUE(reg_raw->hasFired());
    EXPECT_TRUE(gate.isActive("x"));

    gate.onCoERead(0, 0x6051, 0, data.data(), 2);
    EXPECT_TRUE(coe_raw->hasFired());
    EXPECT_FALSE(gate.isActive("x"));
}

TEST(DebugGateTest, IsActiveForSlaveRespectsGlobalGate) {
    DebugGate gate;
    gate.addGlobalStart(std::make_unique<StateCondition>(SlaveState::OP));
    EXPECT_FALSE(gate.isActiveForSlave(0));
    gate.notifyCheckpoint("state:op");
    EXPECT_TRUE(gate.isActiveForSlave(0));
}

// ============================================================================
// debug:: helpers — pure parts
// ============================================================================

TEST(DebugHelpersTest, ParseFlagsTrimsAndDedups) {
    auto flags = debug::parseFlags(" a , b ,,a,  c ");
    ASSERT_EQ(flags.size(), 3u);
    EXPECT_TRUE(flags.count("a"));
    EXPECT_TRUE(flags.count("b"));
    EXPECT_TRUE(flags.count("c"));
}

TEST(DebugHelpersTest, ParseFlagsEmptySpec) {
    EXPECT_TRUE(debug::parseFlags("").empty());
}

TEST(DebugHelpersTest, PrintFlagsHelpRecognisesHelpToken) {
    EXPECT_TRUE(debug::printFlagsHelp("help"));
    EXPECT_FALSE(debug::printFlagsHelp("rx-pdo"));
}

TEST(DebugHelpersTest, PrintConditionHelpRecognisesHelpToken) {
    EXPECT_TRUE(debug::printConditionHelp("help"));
    EXPECT_FALSE(debug::printConditionHelp("state:op"));
}

// ============================================================================
// debug:: helpers — Master-taking paths over an emulated bus
// ============================================================================

namespace {

class DebugHelpersMasterTest : public ::testing::Test {
protected:
    void SetUp() override {
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

TEST_F(DebugHelpersMasterTest, ApplyGateConditionsEmptySpecsSucceeds) {
    EXPECT_TRUE(debug::applyGateConditions("", "", master_, "test"));
    EXPECT_FALSE(master_.debugGate().hasAnyConditions());
}

TEST_F(DebugHelpersMasterTest, ApplyGateConditionsAddsGlobalConditions) {
    EXPECT_TRUE(debug::applyGateConditions("state:pre-op", "checkpoint:done",
                                           master_, "test"));
    EXPECT_TRUE(master_.debugGate().hasAnyConditions());
    EXPECT_FALSE(master_.debugGate().isActive("any"));
    master_.debugGate().notifyCheckpoint("state:pre-op");
    EXPECT_TRUE(master_.debugGate().isActive("any"));
    master_.debugGate().notifyCheckpoint("done");
    EXPECT_FALSE(master_.debugGate().isActive("any"));
}

TEST_F(DebugHelpersMasterTest, ApplyGateConditionsRejectsBadStartSpec) {
    EXPECT_FALSE(debug::applyGateConditions("bogus", "", master_, "test"));
}

TEST_F(DebugHelpersMasterTest, ApplyGateConditionsRejectsBadStopSpec) {
    EXPECT_FALSE(debug::applyGateConditions("", "nope:1", master_, "test"));
}

TEST_F(DebugHelpersMasterTest, ApplyFlagsKnownFlag) {
    debug::applyFlags({"rx-pdo"}, master_, "test");
}

TEST_F(DebugHelpersMasterTest, ApplyFlagsUnknownFlagLogsButDoesNotThrow) {
    debug::applyFlags({"not-a-real-flag"}, master_, "test");
}

TEST_F(DebugHelpersMasterTest, ApplyFlagsEmptySet) {
    debug::applyFlags({}, master_, "test");
}

#endif  // TETHER_DEBUG_GATE_ENABLED
