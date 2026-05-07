/// @file test_default_limits.cpp
/// @brief Unit tests for per-system default perturbation limits.

#include <gtest/gtest.h>
#include "tether/destabilizer/DefaultLimits.hpp"

using namespace Destabilizer;

TEST(DefaultLimits, AllSystemsHaveDefaults) {
    // Systems 1-65 should all return non-empty channel lists
    for (int id = 1; id <= 65; ++id) {
        auto channels = getDefaultChannels(id);
        EXPECT_GT(channels.size(), 0u)
            << "System " << id << " has no default channels";
        for (const auto& ch : channels) {
            EXPECT_GT(ch.constraints.amplitudeMax, 0.0)
                << "System " << id << " channel " << ch.name << " has zero amplitudeMax";
            EXPECT_FALSE(ch.name.empty())
                << "System " << id << " has unnamed channel";
            EXPECT_FALSE(ch.tooltipRationale.empty())
                << "System " << id << " channel " << ch.name << " has no rationale";
        }
    }
}

TEST(DefaultLimits, FallbackForUnknownSystem) {
    auto channels = getDefaultChannels(999);
    EXPECT_GT(channels.size(), 0u);
    EXPECT_GT(channels[0].constraints.amplitudeMax, 0.0);
}

TEST(DefaultLimits, GetDefaultRationale) {
    auto rationale = getDefaultRationale(1);
    EXPECT_FALSE(rationale.empty());
}

TEST(DefaultLimits, GetDefaultConstraints) {
    auto c = getDefaultConstraints(3, 0); // Inverted pendulum
    EXPECT_GT(c.amplitudeMax, 0.0);
    EXPECT_GT(c.energyMax, 0.0);
}

TEST(DefaultLimits, GetDefaultConstraintsOutOfRange) {
    auto c = getDefaultConstraints(3, 99); // Invalid channel index
    EXPECT_GT(c.amplitudeMax, 0.0); // Fallback
}

TEST(DefaultLimits, MultiChannelSystems) {
    // Ball on Plate (system 10) should have 2 channels
    auto channels = getDefaultChannels(10);
    EXPECT_EQ(channels.size(), 2u);
    EXPECT_NE(channels[0].inputIndex, channels[1].inputIndex);
}

TEST(DefaultLimits, ConstraintsPhysicallyReasonable) {
    // Mass-spring-damper: amplitude should be small (a few N for 1kg)
    auto ch = getDefaultChannels(1);
    ASSERT_EQ(ch.size(), 1u);
    EXPECT_LE(ch[0].constraints.amplitudeMax, 50.0);
    EXPECT_GT(ch[0].constraints.amplitudeMax, 0.01);

    // Gantry crane: larger forces
    ch = getDefaultChannels(13);
    ASSERT_EQ(ch.size(), 1u);
    EXPECT_GE(ch[0].constraints.amplitudeMax, 10.0);
}
