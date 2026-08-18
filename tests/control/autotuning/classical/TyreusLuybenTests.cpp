#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class TyreusLuybenTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<TyreusLuyben>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    std::unique_ptr<TyreusLuyben> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(TyreusLuybenTest, Name) { EXPECT_EQ(tuner->getName(), "Tyreus-Luyben"); }
TEST_F(TyreusLuybenTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(TyreusLuybenTest, SetUltimateParameters) { tuner->setUltimateParameters(5.0, 2.0); }

TEST_F(TyreusLuybenTest, CalculateGains) { auto gains = TyreusLuyben::calculateGains(5.0, 2.0, false); EXPECT_NEAR(gains.Kp, 5.0/3.2, 0.01); }
