/**
 * @file test_io_threshold_filter.cpp
 * @brief Unit tests for ThresholdFilter and ThresholdConfig.
 */
#include <gtest/gtest.h>
#include "tether/io/ThresholdFilter.hpp"
#include <cstring>

using namespace tether::io;

// ===========================================================================
// ThresholdRule encode/decode roundtrip
// ===========================================================================

TEST(IOThreshold, RuleRoundtrip) {
    ThresholdRule rule;
    rule.entryId = 0x1234;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.01;
    rule.customName = "";

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    rule.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    ThresholdRule decoded;
    EXPECT_TRUE(ThresholdRule::decode(r, decoded));
    EXPECT_EQ(decoded.entryId, 0x1234u);
    EXPECT_EQ(decoded.type, ThresholdType::Absolute);
    EXPECT_DOUBLE_EQ(decoded.threshold, 0.01);
}

TEST(IOThreshold, RuleWithCustomConfig) {
    ThresholdRule rule;
    rule.entryId = 42;
    rule.type = ThresholdType::Custom;
    rule.threshold = 0;
    rule.customName = "my_filter";
    ConfigPrimitive cp;
    cp.setFloat(0.5f);
    cp.name = "alpha";
    rule.customConfig.push_back(cp);

    uint8_t buf[512];
    BufWriter w(buf, sizeof(buf));
    rule.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    ThresholdRule decoded;
    EXPECT_TRUE(ThresholdRule::decode(r, decoded));
    EXPECT_EQ(decoded.customName, "my_filter");
    ASSERT_EQ(decoded.customConfig.size(), 1u);
    EXPECT_EQ(decoded.customConfig[0].name, "alpha");
    EXPECT_FLOAT_EQ(decoded.customConfig[0].getFloat(), 0.5f);
}

// ===========================================================================
// ThresholdFilter passes()
// ===========================================================================

TEST(IOThreshold, AbsolutePassesLargeChange) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.5;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 1.0f;
    float newVal = 2.0f;
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

TEST(IOThreshold, AbsoluteRejectsSmallChange) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.5;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 1.0f;
    float newVal = 1.1f;  // change = 0.1 < 0.5
    EXPECT_FALSE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

TEST(IOThreshold, NoRuleAlwaysPasses) {
    ThresholdFilter filter;
    float old_ = 1.0f;
    float new_ = 1.0001f;
    EXPECT_TRUE(filter.passes(99, &old_, &new_, sizeof(float)));
}

TEST(IOThreshold, DefaultRuleForAllEntries) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 0;  // default rule
    rule.type = ThresholdType::Absolute;
    rule.threshold = 1.0;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 5.0f;
    float new_ = 5.5f;
    // entry 42 has no specific rule, but default applies
    EXPECT_FALSE(filter.passes(42, &old_, &new_, sizeof(float)));

    new_ = 7.0f;
    EXPECT_TRUE(filter.passes(42, &old_, &new_, sizeof(float)));
}

// ===========================================================================
// ConfigPrimitive helpers
// ===========================================================================

TEST(IOThreshold, ConfigPrimitiveFloat) {
    ConfigPrimitive cp;
    cp.setFloat(3.14f);
    EXPECT_EQ(cp.type, ValueType::F32);
    EXPECT_FLOAT_EQ(cp.getFloat(), 3.14f);
}

TEST(IOThreshold, ConfigPrimitiveString) {
    ConfigPrimitive cp;
    cp.setString("hello");
    EXPECT_EQ(cp.type, ValueType::String);
    EXPECT_EQ(cp.getString(), "hello");
}

// ===========================================================================
// ThresholdConfig encode/decode roundtrip
// ===========================================================================

TEST(IOThreshold, ConfigRoundtrip) {
    ThresholdConfig cfg;
    cfg.name = "precision_mode";
    cfg.isWhitelist = false;
    ThresholdRule r1;
    r1.entryId = 100;
    r1.type = ThresholdType::Relative;
    r1.threshold = 0.05;
    cfg.rules.push_back(r1);

    uint8_t buf[1024];
    BufWriter w(buf, sizeof(buf));
    cfg.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    ThresholdConfig decoded;
    EXPECT_TRUE(ThresholdConfig::decode(r, decoded));
    EXPECT_EQ(decoded.name, "precision_mode");
    EXPECT_FALSE(decoded.isWhitelist);
    ASSERT_EQ(decoded.rules.size(), 1u);
    EXPECT_EQ(decoded.rules[0].entryId, 100u);
    EXPECT_EQ(decoded.rules[0].type, ThresholdType::Relative);
    EXPECT_DOUBLE_EQ(decoded.rules[0].threshold, 0.05);
}
