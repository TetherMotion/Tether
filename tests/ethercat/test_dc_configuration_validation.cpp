#include <gtest/gtest.h>

#include "tether/ethercat/DCConfigurationValidation.hpp"

using EtherCAT::DC::DCConfig;
using EtherCAT::DC::DCConfigurationValidation;

namespace {

DCConfig makeBaseConfig() {
    DCConfig cfg{};
    cfg.cycle_period_us = 1000;
    cfg.sync_interval_cycles = 1;
    cfg.enable_sync0 = false;
    cfg.enable_sync1 = false;
    cfg.sync0_cycle_time_ns = 0;
    cfg.sync0_shift_ns = 0;
    cfg.sync1_cycle_time_ns = 0;
    return cfg;
}

} // namespace

TEST(DCConfigurationValidation, ValidWhenSyncsDisabledAndCycleNonZero) {
    auto cfg = makeBaseConfig();
    auto res = DCConfigurationValidation::validate(cfg);
    EXPECT_TRUE(res.valid);
    EXPECT_TRUE(res.error_message.empty());

    auto text = DCConfigurationValidation::toString(cfg);
    EXPECT_NE(text.find("Master Cycle"), std::string::npos);
    EXPECT_NE(text.find("SYNC0"), std::string::npos);
    EXPECT_NE(text.find("DISABLED"), std::string::npos);
}

TEST(DCConfigurationValidation, DetectsZeroMasterCycle) {
    auto cfg = makeBaseConfig();
    cfg.cycle_period_us = 0;
    auto res = DCConfigurationValidation::validate(cfg);
    EXPECT_FALSE(res.valid);
    EXPECT_NE(res.error_message.find("Master cycle period is 0"), std::string::npos);
}

TEST(DCConfigurationValidation, Sync0EnabledRejectsZeroCycleTime) {
    auto cfg = makeBaseConfig();
    cfg.enable_sync0 = true;
    cfg.sync0_cycle_time_ns = 0;
    auto res = DCConfigurationValidation::validate(cfg);
    EXPECT_FALSE(res.valid);
    EXPECT_NE(res.error_message.find("SYNC0 enabled but cycle time is 0"), std::string::npos);
}

TEST(DCConfigurationValidation, Sync0EnabledRejectsShiftAtOrBeyondCycle) {
    auto cfg = makeBaseConfig();
    cfg.enable_sync0 = true;
    cfg.sync0_cycle_time_ns = 100;

    cfg.sync0_shift_ns = 100;
    auto res1 = DCConfigurationValidation::validate(cfg);
    EXPECT_FALSE(res1.valid);
    EXPECT_NE(res1.error_message.find("SYNC0 shift"), std::string::npos);

    cfg.sync0_shift_ns = -100;
    auto res2 = DCConfigurationValidation::validate(cfg);
    EXPECT_FALSE(res2.valid);
    EXPECT_NE(res2.error_message.find("SYNC0 shift"), std::string::npos);
}

TEST(DCConfigurationValidation, Sync1EnabledRejectsZeroCycleAndSync0Off) {
    auto cfg = makeBaseConfig();
    cfg.enable_sync1 = true;
    cfg.sync1_cycle_time_ns = 0;
    cfg.enable_sync0 = false;
    auto res = DCConfigurationValidation::validate(cfg);
    EXPECT_FALSE(res.valid);
    EXPECT_NE(res.error_message.find("SYNC1 enabled but cycle time is 0"), std::string::npos);
    EXPECT_NE(res.error_message.find("SYNC1 enabled without SYNC0"), std::string::npos);
}

TEST(DCConfigurationValidation, ToStringIncludesSyncDetailsWhenEnabled) {
    auto cfg = makeBaseConfig();
    cfg.enable_sync0 = true;
    cfg.sync0_cycle_time_ns = 1000;
    cfg.sync0_shift_ns = -100;
    cfg.enable_sync1 = true;
    cfg.sync1_cycle_time_ns = 2000;

    auto text = DCConfigurationValidation::toString(cfg);
    EXPECT_NE(text.find("SYNC0"), std::string::npos);
    EXPECT_NE(text.find("ENABLED"), std::string::npos);
    EXPECT_NE(text.find("Cycle=1000"), std::string::npos);
    EXPECT_NE(text.find("Shift=-100"), std::string::npos);
    EXPECT_NE(text.find("SYNC1"), std::string::npos);
    EXPECT_NE(text.find("Cycle=2000"), std::string::npos);
}
