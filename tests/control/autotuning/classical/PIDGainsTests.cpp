#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;

TEST(PIDGainsTest, DefaultConstruction) {
    PIDGains gains;
    EXPECT_DOUBLE_EQ(gains.Kp, 0.0);
    EXPECT_DOUBLE_EQ(gains.Ki, 0.0);
    EXPECT_DOUBLE_EQ(gains.Kd, 0.0);
}

TEST(PIDGainsTest, FieldAssignment) {
    PIDGains gains;
    gains.Kp = 1.0;
    gains.Ki = 0.1;
    gains.Kd = 0.01;
    gains.Ti = 10.0;
    gains.Td = 1.0;

    EXPECT_DOUBLE_EQ(gains.Kp, 1.0);
    EXPECT_DOUBLE_EQ(gains.Ki, 0.1);
    EXPECT_DOUBLE_EQ(gains.Kd, 0.01);
    EXPECT_DOUBLE_EQ(gains.Ti, 10.0);
    EXPECT_DOUBLE_EQ(gains.Td, 1.0);
}

TEST(PIDGainsTest, IsValid) {
    PIDGains gains;
    gains.Kp = 1.0;
    gains.Ki = 0.1;
    gains.Kd = 0.01;

    EXPECT_TRUE(gains.isValid());
}

TEST(PIDGainsTest, IsValidNegativeKp) {
    PIDGains gains;
    gains.Kp = -1.0;
    gains.Ki = 0.1;
    gains.Kd = 0.01;

    EXPECT_FALSE(gains.isValid());
}

TEST(PIDGainsTest, ToStandardForm) {
    PIDGains gains;
    gains.Kp = 2.0;
    gains.Ki = 0.2;  // Ki = Kp/Ti -> Ti = Kp/Ki = 10
    gains.Kd = 0.4;  // Kd = Kp*Td -> Td = Kd/Kp = 0.2

    gains.toStandardForm();

    EXPECT_NEAR(gains.Ti, 10.0, 1e-6);
    EXPECT_NEAR(gains.Td, 0.2, 1e-6);
}

TEST(PIDGainsTest, ToParallelForm) {
    PIDGains gains;
    gains.Kp = 2.0;
    gains.Ti = 10.0;
    gains.Td = 0.2;

    gains.toParallelForm();

    EXPECT_NEAR(gains.Ki, 0.2, 1e-6);
    EXPECT_NEAR(gains.Kd, 0.4, 1e-6);
}
