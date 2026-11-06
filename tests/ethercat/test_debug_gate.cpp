/**
 * @file test_debug_gate.cpp
 * @brief Coverage for the EtherCAT debug-flag/condition-gate system:
 *        DebugFlags.cpp (isEnabled/setFlag/setFilter/applyFromString/
 *        resizeFilters/allDebugFlags), DebugGate.cpp (global + per-flag
 *        start/stop gates, parseCondition for all spec types,
 *        gate-changed callback), and DebugConditions.cpp (State/
 *        Checkpoint/RegisterIntercept/CoEIntercept/Custom conditions).
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"

using namespace EtherCAT;

#if TETHER_DEBUG_GATE_ENABLED

namespace {

// ============================================================================
// SlaveFilter
// ============================================================================

TEST(DebugGateCov, SlaveFilterPassAll) {
    SlaveFilter f;
    EXPECT_TRUE(f.pass_all);
    EXPECT_TRUE(f.allows(0));
    EXPECT_TRUE(f.allows(65000));
}

TEST(DebugGateCov, SlaveFilterMask) {
    SlaveFilter f;
    f.setMask({0, 2}, 4);
    EXPECT_FALSE(f.pass_all);
    EXPECT_TRUE(f.allows(0));
    EXPECT_FALSE(f.allows(1));
    EXPECT_TRUE(f.allows(2));
    EXPECT_FALSE(f.allows(3));
    EXPECT_FALSE(f.allows(9));   // out of mask range
    EXPECT_EQ(f.filter_indices.size(), 2u);
}

TEST(DebugGateCov, SlaveFilterResize) {
    SlaveFilter f;
    f.setMask({0}, 2);
    f.resize(5);                 // resize keeps semantics sane
    EXPECT_TRUE(f.allows(0));
}

// ============================================================================
// EtherCATMasterDebugFlags — setFlag / isEnabled
// ============================================================================

TEST(DebugGateCov, FlagsDefaultDisabled) {
    EtherCATMasterDebugFlags f;
    for (const auto& info : debug::allDebugFlags()) {
        EXPECT_FALSE(f.isEnabled(info.name, 0)) << info.name;
    }
    EXPECT_FALSE(f.isEnabled("no-such-flag", 0));
}

TEST(DebugGateCov, SetFlagEnablesNamedFlag) {
    EtherCATMasterDebugFlags f;
    f.setFlag("rx-pdo", true);
    EXPECT_TRUE(f.isEnabled("rx-pdo", 0));
    EXPECT_FALSE(f.isEnabled("tx-pdo", 0));
    f.setFlag("rx-pdo", false);
    EXPECT_FALSE(f.isEnabled("rx-pdo", 0));
}

TEST(DebugGateCov, EveryKnownFlagToggles) {
    EtherCATMasterDebugFlags f;
    for (const auto& info : debug::allDebugFlags()) {
        // sii-derivation is driven directly, not through setFlag/isEnabled.
        if (info.name == "sii-derivation") continue;
        f.setFlag(info.name, true);
        EXPECT_TRUE(f.isEnabled(info.name, 3)) << info.name;
        f.setFlag(info.name, false);
    }
}

TEST(DebugGateCov, UnknownSetFlagIgnored) {
    EtherCATMasterDebugFlags f;
    f.setFlag("bogus", true);    // no crash, no effect
    EXPECT_FALSE(f.isEnabled("bogus", 0));
}

TEST(DebugGateCov, FilterNarrowsFlag) {
    EtherCATMasterDebugFlags f;
    f.setFlag("coe-reads", true);
    EXPECT_TRUE(f.isEnabled("coe-reads", 7));    // pass_all filter

    SlaveFilter filt;
    filt.setMask({1}, 3);
    f.setFilter("coe-reads", filt);
    EXPECT_FALSE(f.isEnabled("coe-reads", 7));
    EXPECT_TRUE(f.isEnabled("coe-reads", 1));
    EXPECT_FALSE(f.isEnabled("coe-reads", 0));
}

TEST(DebugGateCov, SetFilterOnUnknownFlagIgnored) {
    EtherCATMasterDebugFlags f;
    SlaveFilter filt;
    filt.setMask({0}, 1);
    f.setFilter("nope", filt);   // ignored
}

TEST(DebugGateCov, ResizeFiltersKeepsFlags) {
    EtherCATMasterDebugFlags f;
    f.setFlag("dc", true);
    f.resizeFilters(8);
    EXPECT_TRUE(f.isEnabled("dc", 4));
}

// ============================================================================
// applyFromString
// ============================================================================

TEST(DebugGateCov, ApplyFromStringBasic) {
    EtherCATMasterDebugFlags f;
    f.applyFromString("rx-pdo,tx-pdo", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("rx-pdo", 0));
    EXPECT_TRUE(f.isEnabled("tx-pdo", 0));
    EXPECT_FALSE(f.isEnabled("dc", 0));
}

TEST(DebugGateCov, ApplyFromStringEmptyIsNoop) {
    EtherCATMasterDebugFlags f;
    f.applyFromString("", 4, nullptr);
    EXPECT_FALSE(f.isEnabled("rx-pdo", 0));
}

TEST(DebugGateCov, ApplyFromStringWhitespaceAndEmptyEntries) {
    EtherCATMasterDebugFlags f;
    f.applyFromString("  rx-pdo , , dc ", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("rx-pdo", 0));
    EXPECT_TRUE(f.isEnabled("dc", 0));
}

TEST(DebugGateCov, ApplyFromStringSlaveFilter) {
    EtherCATMasterDebugFlags f;
    // Comma-separated index lists can't appear inside a spec entry — ','
    // already separates entries — so filter lists use ranges or single
    // indices: "(slaves:0-2)", "(slaves:2)".
    f.applyFromString("coe-reads:(slaves:0)", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("coe-reads", 0));
    EXPECT_FALSE(f.isEnabled("coe-reads", 1));
    EXPECT_FALSE(f.isEnabled("coe-reads", 2));
}

TEST(DebugGateCov, ApplyFromStringSlaveRange) {
    EtherCATMasterDebugFlags f;
    f.applyFromString("fmmu:(slaves:1-3)", 5, nullptr);
    EXPECT_FALSE(f.isEnabled("fmmu", 0));
    EXPECT_TRUE(f.isEnabled("fmmu", 1));
    EXPECT_TRUE(f.isEnabled("fmmu", 3));
    EXPECT_FALSE(f.isEnabled("fmmu", 4));
}

TEST(DebugGateCov, ApplyFromStringMalformedFilterBits) {
    EtherCATMasterDebugFlags f;
    // No closing paren → filter ignored, flag still set
    f.applyFromString("dc:(slaves:0", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("dc", 0));
    // Non-slaves filter type → ignored
    f.applyFromString("eeprom:(other:0)", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("eeprom", 0));
    // Group without colon → skipped
    f.applyFromString("sii-eeprom:(noval)", 4, nullptr);
    EXPECT_TRUE(f.isEnabled("sii-eeprom", 0));
}

TEST(DebugGateCov, ParseIndexListThroughApply) {
    EtherCATMasterDebugFlags f;
    // Indices beyond slave_count are masked off (idx < count guard).
    f.applyFromString("tx-pdo:(slaves:3-9)", 5, nullptr);
    EXPECT_TRUE(f.isEnabled("tx-pdo", 3));
    EXPECT_TRUE(f.isEnabled("tx-pdo", 4));
    // 5..9 parsed but clipped by the slave_count bound.
    EXPECT_FALSE(f.isEnabled("tx-pdo", 0));
}

// ============================================================================
// DebugGate — base behavior
// ============================================================================

TEST(DebugGateCov, NoConditionsAlwaysActive) {
    DebugGate g;
    EXPECT_FALSE(g.hasAnyConditions());
    EXPECT_TRUE(g.isActive("rx-pdo"));
    EXPECT_TRUE(g.isActiveForSlave(0));
    // Event hooks are no-ops with no conditions
    g.notifyCheckpoint("state:op", 0);
    uint8_t d[2] = {0x01, 0x00};
    g.onRegisterRead(0, 0x0130, d, 2);
    g.onCoERead(0, 0x6051, 0, d, 2);
    EXPECT_TRUE(g.isActive("rx-pdo"));
}

TEST(DebugGateCov, GlobalStartBlocksUntilFired) {
    DebugGate g;
    g.addGlobalStart(DebugGate::parseCondition("checkpoint:armed"));
    EXPECT_TRUE(g.hasAnyConditions());
    EXPECT_FALSE(g.isActive("rx-pdo"));
    EXPECT_FALSE(g.isActiveForSlave(0));

    g.notifyCheckpoint("other", 0);          // wrong name
    EXPECT_FALSE(g.isActive("rx-pdo"));

    g.notifyCheckpoint("armed", 0);          // fires
    EXPECT_TRUE(g.isActive("rx-pdo"));
    EXPECT_TRUE(g.isActiveForSlave(0));
}

TEST(DebugGateCov, GlobalStopClosesGate) {
    DebugGate g;
    g.addGlobalStart(DebugGate::parseCondition("checkpoint:go"));
    g.addGlobalStop(DebugGate::parseCondition("checkpoint:done"));
    g.notifyCheckpoint("go", 0);
    EXPECT_TRUE(g.isActive("rx-pdo"));
    g.notifyCheckpoint("done", 0);
    EXPECT_FALSE(g.isActive("rx-pdo"));
}

TEST(DebugGateCov, PerFlagStartOnlyAffectsThatFlag) {
    DebugGate g;
    g.addStartCondition("rx-pdo",
                        DebugGate::parseCondition("checkpoint:rx-ready"));
    EXPECT_FALSE(g.isActive("rx-pdo"));      // gated
    EXPECT_TRUE(g.isActive("tx-pdo"));       // follows open global gate

    g.notifyCheckpoint("rx-ready", 0);
    EXPECT_TRUE(g.isActive("rx-pdo"));
}

TEST(DebugGateCov, PerFlagStopClosesOnlyThatFlag) {
    DebugGate g;
    g.addStopCondition("rx-pdo", DebugGate::parseCondition("checkpoint:halt"));
    EXPECT_TRUE(g.isActive("rx-pdo"));
    g.notifyCheckpoint("halt", 0);
    EXPECT_FALSE(g.isActive("rx-pdo"));
    EXPECT_TRUE(g.isActive("tx-pdo"));
}

TEST(DebugGateCov, PerFlagStartPlusGlobalStart) {
    DebugGate g;
    g.addGlobalStart(DebugGate::parseCondition("checkpoint:global-go"));
    g.addStartCondition("rx-pdo",
                        DebugGate::parseCondition("checkpoint:flag-go"));

    g.notifyCheckpoint("flag-go", 0);
    EXPECT_FALSE(g.isActive("rx-pdo"));      // global still closed
    g.notifyCheckpoint("global-go", 0);
    EXPECT_TRUE(g.isActive("rx-pdo"));       // both open
}

TEST(DebugGateCov, GateChangedCallbackFiresOnTransition) {
    DebugGate g;
    int calls = 0;
    bool last = false;
    g.setGateChangedCallback([&] { ++calls; last = g.isActive("x"); });

    g.addGlobalStart(DebugGate::parseCondition("checkpoint:go"));
    g.notifyCheckpoint("nope", 0);
    // Still inactive — no transition, implementation-defined whether it
    // notifies; just ensure no crash, then fire for a real transition.
    g.notifyCheckpoint("go", 0);
    EXPECT_GE(calls, 1);
    EXPECT_TRUE(last);
}

// ============================================================================
// parseCondition — all spec types
// ============================================================================

TEST(DebugGateCov, ParseConditionTypes) {
    EXPECT_EQ(DebugGate::parseCondition("state:op")->type(),
              DebugConditionType::State);
    EXPECT_EQ(DebugGate::parseCondition("checkpoint:foo")->type(),
              DebugConditionType::Checkpoint);
    EXPECT_EQ(DebugGate::parseCondition("reg:0x0130:==:0x02")->type(),
              DebugConditionType::RegisterIntercept);
    EXPECT_EQ(DebugGate::parseCondition("coe:0x6051:==:0x01")->type(),
              DebugConditionType::CoEIntercept);
}

TEST(DebugGateCov, ParseConditionRejectsBadSpecs) {
    EXPECT_EQ(DebugGate::parseCondition(""), nullptr);
    EXPECT_EQ(DebugGate::parseCondition("bogus:x"), nullptr);
    EXPECT_EQ(DebugGate::parseCondition("state:badstate"), nullptr);
    EXPECT_EQ(DebugGate::parseCondition("state:"), nullptr);
    EXPECT_EQ(DebugGate::parseCondition("checkpoint:"), nullptr); // empty name ok? parts>=2 → cond w/ empty name
    EXPECT_EQ(DebugGate::parseCondition("reg:0x01"), nullptr);    // too few parts
    EXPECT_EQ(DebugGate::parseCondition("coe:0x6051"), nullptr);  // too few parts
}

TEST(DebugGateCov, ParseStateAllNames) {
    for (const char* s : {"init", "INIT", "pre-op", "PRE-OP", "preop",
                          "boot", "BOOT", "safe-op", "SAFE-OP", "safeop",
                          "op", "OP"}) {
        std::string spec = std::string("state:") + s;
        auto c = DebugGate::parseCondition(spec);
        ASSERT_NE(c, nullptr) << spec;
        EXPECT_EQ(c->type(), DebugConditionType::State);
    }
}

TEST(DebugGateCov, ParseStateWithSlaveIndex) {
    auto c = DebugGate::parseCondition("state:op:2");
    ASSERT_NE(c, nullptr);
    // Fires only for slave 2 — verified via gate below
    DebugGate g;
    g.addGlobalStart(std::move(c));
    g.notifyCheckpoint("state:op", 1);       // wrong slave
    EXPECT_FALSE(g.isActive("x"));
    g.notifyCheckpoint("state:op", 2);       // right slave
    EXPECT_TRUE(g.isActive("x"));
}

TEST(DebugGateCov, ParseRegSlaveIndexed) {
    auto c = DebugGate::parseCondition("reg:1:0x0130:==:0x02");
    ASSERT_NE(c, nullptr);
    DebugGate g;
    g.addGlobalStart(std::move(c));
    uint8_t d[2] = {0x02, 0x00};
    g.onRegisterRead(0, 0x0130, d, 2);       // wrong slave
    EXPECT_FALSE(g.isActive("x"));
    g.onRegisterRead(1, 0x0130, d, 2);       // right slave
    EXPECT_TRUE(g.isActive("x"));
}

TEST(DebugGateCov, ParseCoESubindexForm) {
    auto c = DebugGate::parseCondition("coe:0x6051:0x00:==:0x01");
    ASSERT_NE(c, nullptr);
    DebugGate g;
    g.addGlobalStart(std::move(c));
    uint8_t d[2] = {0x01, 0x00};
    g.onCoERead(0, 0x6051, 1, d, 2);         // wrong subindex
    EXPECT_FALSE(g.isActive("x"));
    g.onCoERead(0, 0x6051, 0, d, 2);         // sub 0 matches
    EXPECT_TRUE(g.isActive("x"));
}

TEST(DebugGateCov, ParseCoESlaveIndexedForm) {
    auto c = DebugGate::parseCondition("coe:0:0x6051:0x00:==:0x01");
    ASSERT_NE(c, nullptr);
    auto c2 = DebugGate::parseCondition("coe:0:0x6051:==:0x01");  // no sub
    ASSERT_NE(c2, nullptr);
}

// ============================================================================
// Condition firing — operators
// ============================================================================

TEST(DebugGateCov, RegConditionCompareOps) {
    struct { const char* spec; uint8_t val; bool expect; } cases[] = {
        {"reg:0x10:==:5",  5, true},
        {"reg:0x10:==:5",  6, false},
        {"reg:0x10:!=:5",  6, true},
        {"reg:0x10:&:4",   5, true},   // 5 & 4 = 4
        {"reg:0x10:&:8",   5, false},
        {"reg:0x10:>=:5",  6, true},
        {"reg:0x10:<=:5",  4, true},
        {"reg:0x10:>:5",   6, true},
        {"reg:0x10:<:5",   4, true},
        {"reg:0x10:??:5",  5, true},   // unknown op → Eq
    };
    for (auto& tc : cases) {
        auto c = DebugGate::parseCondition(tc.spec);
        ASSERT_NE(c, nullptr) << tc.spec;
        uint8_t d[1] = {tc.val};
        c->onRegisterRead(0, 0x10, d, 1);
        EXPECT_EQ(c->hasFired(), tc.expect) << tc.spec << " val=" << int(tc.val);
    }
}

TEST(DebugGateCov, RegConditionWrongAddrAndNullData) {
    auto c = DebugGate::parseCondition("reg:0x0130:==:0x02");
    ASSERT_NE(c, nullptr);
    uint8_t d[2] = {0x02, 0x00};
    c->onRegisterRead(0, 0x9999, d, 2);      // wrong addr
    EXPECT_FALSE(c->hasFired());
    c->onRegisterRead(0, 0x0130, nullptr, 0); // null data
    EXPECT_FALSE(c->hasFired());
}

TEST(DebugGateCov, RegConditionMultiByteLEValue) {
    auto c = DebugGate::parseCondition("reg:0x10:==:0x1234");
    ASSERT_NE(c, nullptr);
    uint8_t d[2] = {0x34, 0x12};             // little-endian 0x1234
    c->onRegisterRead(0, 0x10, d, 2);
    EXPECT_TRUE(c->hasFired());
}

TEST(DebugGateCov, CoEConditionSubFilterAndNull) {
    auto c = DebugGate::parseCondition("coe:0x6051:0x01:==:0xAA");
    ASSERT_NE(c, nullptr);
    uint8_t d[1] = {0xAA};
    c->onCoERead(0, 0x6052, 1, d, 1);        // wrong index
    EXPECT_FALSE(c->hasFired());
    c->onCoERead(0, 0x6051, 0, d, 1);        // wrong sub
    EXPECT_FALSE(c->hasFired());
    c->onCoERead(0, 0x6051, 1, nullptr, 0);  // null data
    EXPECT_FALSE(c->hasFired());
    c->onCoERead(0, 0x6051, 1, d, 1);        // match
    EXPECT_TRUE(c->hasFired());
}

TEST(DebugGateCov, CoENoSubMatchesAnySub) {
    auto c = DebugGate::parseCondition("coe:0x6051:==:0xAA");
    ASSERT_NE(c, nullptr);
    uint8_t d[1] = {0xAA};
    c->onCoERead(0, 0x6051, 9, d, 1);        // any sub
    EXPECT_TRUE(c->hasFired());
}

TEST(DebugGateCov, ConditionsReset) {
    auto c = DebugGate::parseCondition("checkpoint:x");
    ASSERT_NE(c, nullptr);
    c->onCheckpoint("x", 0);
    EXPECT_TRUE(c->hasFired());
    c->reset();
    EXPECT_FALSE(c->hasFired());
}

TEST(DebugGateCov, ConditionFiresOnce) {
    auto c = DebugGate::parseCondition("checkpoint:x:5");
    ASSERT_NE(c, nullptr);
    c->onCheckpoint("x", 4);                 // wrong slave
    EXPECT_FALSE(c->hasFired());
    c->onCheckpoint("x", 5);
    EXPECT_TRUE(c->hasFired());
    c->onCheckpoint("x", 5);                 // already fired, stays fired
    EXPECT_TRUE(c->hasFired());
}

TEST(DebugGateCov, CustomConditionCallback) {
    bool armed = false;
    CustomCondition c([&] { return armed; });
    EXPECT_FALSE(c.hasFired());
    armed = true;
    EXPECT_TRUE(c.hasFired());               // latches
    armed = false;
    EXPECT_TRUE(c.hasFired());               // stays fired
    c.reset();
    EXPECT_FALSE(c.hasFired());
}

TEST(DebugGateCov, CustomConditionNullCallback) {
    CustomCondition c(nullptr);
    EXPECT_FALSE(c.hasFired());
}

TEST(DebugGateCov, StateConditionCheckpointNames) {
    // Fire via the state checkpoint names the master emits
    struct { SlaveState st; const char* cp; } cases[] = {
        {SlaveState::INIT, "state:init"},
        {SlaveState::PRE_OP, "state:pre-op"},
        {SlaveState::BOOT, "state:boot"},
        {SlaveState::SAFE_OP, "state:safe-op"},
        {SlaveState::OP, "state:op"},
    };
    for (auto& tc : cases) {
        StateCondition c(tc.st);
        c.onCheckpoint("wrong", 0);
        EXPECT_FALSE(c.hasFired()) << tc.cp;
        c.onCheckpoint(tc.cp, 0);
        EXPECT_TRUE(c.hasFired()) << tc.cp;
    }
}

TEST(DebugGateCov, DebugGateEventHooksWithPerFlagConds) {
    DebugGate g;
    g.addStartCondition("coe-reads",
                        DebugGate::parseCondition("coe:0x1018:==:0x01"));
    uint8_t d[1] = {0x01};
    g.onCoERead(0, 0x1018, 0, d, 1);         // fires per-flag start
    EXPECT_TRUE(g.isActive("coe-reads"));

    DebugGate g2;
    g2.addStartCondition("dc",
                         DebugGate::parseCondition("reg:0x0910:!=:0"));
    g2.onRegisterRead(0, 0x0910, d, 1);
    EXPECT_TRUE(g2.isActive("dc"));
}

}  // namespace

#else  // !TETHER_DEBUG_GATE_ENABLED

TEST(DebugGateCov, StubGateAlwaysActive) {
    DebugGate g;
    EXPECT_TRUE(g.isActive("rx-pdo"));
    EXPECT_TRUE(g.isActiveForSlave(0));
    EXPECT_FALSE(g.hasAnyConditions());
    EXPECT_EQ(DebugGate::parseCondition("state:op"), nullptr);
}

#endif
