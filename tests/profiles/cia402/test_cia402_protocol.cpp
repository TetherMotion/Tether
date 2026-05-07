#include <gtest/gtest.h>
#include "tether/profiles/cia402/MotionController.hpp"

using namespace CiA402;

TEST(CiA402Protocol, Smoke) {
    MotionCommand cmd;
    EXPECT_EQ(cmd.profileType, ProfileType::Trapezoidal);
    EXPECT_FALSE(cmd.relative);
    EXPECT_FALSE(cmd.immediate);
    EXPECT_FALSE(cmd.buffered);
    EXPECT_GT(CIA402_DEFAULT_CYCLE_TIME_US, 0);
}
