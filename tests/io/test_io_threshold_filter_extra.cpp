/**
 * @file test_io_threshold_filter_extra.cpp
 * @brief Additional ThresholdFilter coverage tests.
 */
#include <gtest/gtest.h>
#include "tether/io/ThresholdFilter.hpp"
#include <cstring>
#include <cmath>

using namespace tether::io;

// ===========================================================================
// Relative threshold
// ===========================================================================

TEST(IOThresholdExtra, RelativePassesLargeChange) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.1;  // 10% change required
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 10.0f;
    float newVal = 12.0f;  // 20% change
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

TEST(IOThresholdExtra, RelativeRejectsSmallChange) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.1;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 10.0f;
    float newVal = 10.5f;  // 5% change
    EXPECT_FALSE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

TEST(IOThresholdExtra, RelativeWithNearZeroOldValue) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.1;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 0.0f;   // near zero
    float newVal = 0.5f;
    // When old is ~0, should compare |new| > threshold
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

TEST(IOThresholdExtra, RelativeWithNearZeroOldAndSmallNew) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.5;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 0.0f;
    float newVal = 0.01f;
    EXPECT_FALSE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

// ===========================================================================
// Relative with double (8-byte values)
// ===========================================================================

TEST(IOThresholdExtra, RelativeWithDouble) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.01;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    double oldVal = 100.0;
    double newVal = 102.0;  // 2% change
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(double)));
}

// ===========================================================================
// Absolute with double (8-byte values)
// ===========================================================================

TEST(IOThresholdExtra, AbsoluteWithDouble) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.5;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    double oldVal = 1.0;
    double newVal = 2.0;
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(double)));

    newVal = 1.1;
    EXPECT_FALSE(filter.passes(1, &oldVal, &newVal, sizeof(double)));
}

// ===========================================================================
// Integer types (non-4/8 byte, raw memcmp path)
// ===========================================================================

TEST(IOThresholdExtra, AbsoluteIntegerTypeChanged) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.01;  // threshold doesn't matter for raw comparison
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    uint16_t old_ = 100;
    uint16_t new_ = 200;
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(uint16_t)));
}

TEST(IOThresholdExtra, AbsoluteIntegerTypeUnchanged) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 0.01;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    uint16_t old_ = 100;
    uint16_t new_ = 100;
    EXPECT_FALSE(filter.passes(1, &old_, &new_, sizeof(uint16_t)));
}

TEST(IOThresholdExtra, RelativeIntegerTypeChanged) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.01;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    uint8_t old_ = 10;
    uint8_t new_ = 20;
    // Integer path uses memcmp, not relative math
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(uint8_t)));
}

TEST(IOThresholdExtra, RelativeIntegerTypeUnchanged) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.01;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    uint8_t old_ = 10;
    uint8_t new_ = 10;
    EXPECT_FALSE(filter.passes(1, &old_, &new_, sizeof(uint8_t)));
}

// ===========================================================================
// ThresholdType::None
// ===========================================================================

TEST(IOThresholdExtra, NoneTypeAlwaysPasses) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::None;
    rule.threshold = 0;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f;
    float new_ = 1.0f;  // same value
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(float)));
}

// ===========================================================================
// Custom threshold evaluator
// ===========================================================================

TEST(IOThresholdExtra, CustomEvaluatorCalled) {
    ThresholdFilter filter;

    bool evaluatorCalled = false;
    filter.registerCustom("my_custom", [&evaluatorCalled](
        uint64_t /*entryId*/, const void* /*old*/, const void* /*newV*/,
        size_t /*valSize*/, const std::vector<ConfigPrimitive>& /*config*/) -> bool {
        evaluatorCalled = true;
        return true;
    });

    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Custom;
    rule.customName = "my_custom";
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f, new_ = 2.0f;
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(float)));
    EXPECT_TRUE(evaluatorCalled);
}

TEST(IOThresholdExtra, CustomEvaluatorNotFound) {
    ThresholdFilter filter;

    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Custom;
    rule.customName = "nonexistent";
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f, new_ = 2.0f;
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(float)));
}

TEST(IOThresholdExtra, CustomEvaluatorReturningFalse) {
    ThresholdFilter filter;

    filter.registerCustom("strict_filter", [](
        uint64_t, const void*, const void*,
        size_t, const std::vector<ConfigPrimitive>&) -> bool {
        return false;  // always reject
    });

    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Custom;
    rule.customName = "strict_filter";
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f, new_ = 2.0f;
    EXPECT_FALSE(filter.passes(1, &old_, &new_, sizeof(float)));
}

TEST(IOThresholdExtra, InvalidThresholdTypeFallsBackToPass) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = static_cast<ThresholdType>(99);
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float oldVal = 1.0f;
    float newVal = 1.0f;
    EXPECT_TRUE(filter.passes(1, &oldVal, &newVal, sizeof(float)));
}

// ===========================================================================
// Whitelist / blacklist mode
// ===========================================================================

TEST(IOThresholdExtra, WhitelistModeNoRuleExcludes) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    cfg.isWhitelist = true;
    // Add a rule for entry 1 only
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::None;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f, new_ = 2.0f;
    // Entry 1 has a rule (None → passes)
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(float)));
    // Entry 99 has no rule: whitelist mode → exclude
    EXPECT_FALSE(filter.passes(99, &old_, &new_, sizeof(float)));
}

TEST(IOThresholdExtra, BlacklistModeNoRuleIncludes) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    cfg.isWhitelist = false;  // blacklist mode
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::None;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);

    float old_ = 1.0f, new_ = 2.0f;
    // Entry 99 has no rule: blacklist mode → include
    EXPECT_TRUE(filter.passes(99, &old_, &new_, sizeof(float)));
}

// ===========================================================================
// Reset
// ===========================================================================

TEST(IOThresholdExtra, ResetDoesNotCrash) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    ThresholdRule rule;
    rule.entryId = 1;
    rule.type = ThresholdType::Absolute;
    rule.threshold = 1.0;
    cfg.rules.push_back(rule);
    filter.setConfig(cfg);
    filter.reset();

    // Still works after reset
    float old_ = 1.0f, new_ = 3.0f;
    EXPECT_TRUE(filter.passes(1, &old_, &new_, sizeof(float)));
}

// ===========================================================================
// Config accessor
// ===========================================================================

TEST(IOThresholdExtra, ConfigAccessor) {
    ThresholdFilter filter;
    ThresholdConfig cfg;
    cfg.name = "test_config";
    filter.setConfig(cfg);
    EXPECT_EQ(filter.config().name, "test_config");
}

// ===========================================================================
// ThresholdRule encode/decode None type
// ===========================================================================

TEST(IOThresholdExtra, RuleNoneTypeRoundtrip) {
    ThresholdRule rule;
    rule.entryId = 10;
    rule.type = ThresholdType::None;
    rule.threshold = 0;

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    rule.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    ThresholdRule decoded;
    EXPECT_TRUE(ThresholdRule::decode(r, decoded));
    EXPECT_EQ(decoded.type, ThresholdType::None);
}

// ===========================================================================
// ThresholdRule encode/decode Relative type
// ===========================================================================

TEST(IOThresholdExtra, RuleRelativeTypeRoundtrip) {
    ThresholdRule rule;
    rule.entryId = 20;
    rule.type = ThresholdType::Relative;
    rule.threshold = 0.05;

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    rule.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    ThresholdRule decoded;
    EXPECT_TRUE(ThresholdRule::decode(r, decoded));
    EXPECT_EQ(decoded.type, ThresholdType::Relative);
    EXPECT_DOUBLE_EQ(decoded.threshold, 0.05);
}

// ===========================================================================
// ConfigPrimitive edge cases
// ===========================================================================

TEST(IOThresholdExtra, ConfigPrimitiveEmptyString) {
    ConfigPrimitive cp;
    cp.setString("");
    EXPECT_EQ(cp.getString(), "");
}

TEST(IOThresholdExtra, ConfigPrimitiveNegativeFloat) {
    ConfigPrimitive cp;
    cp.setFloat(-99.9f);
    EXPECT_FLOAT_EQ(cp.getFloat(), -99.9f);
}
