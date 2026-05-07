#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class ZNUltimateCycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<ZieglerNicholsUltimateCycle>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    std::unique_ptr<ZieglerNicholsUltimateCycle> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(ZNUltimateCycleTest, Name) { EXPECT_EQ(tuner->getName(), "Ziegler-Nichols Ultimate Cycle"); }
TEST_F(ZNUltimateCycleTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(ZNUltimateCycleTest, SetUltimateParameters) { tuner->setUltimateParameters(5.0, 2.0); }
TEST_F(ZNUltimateCycleTest, SetControllerForm) { tuner->setControllerForm(PIDForm::Parallel); }
TEST_F(ZNUltimateCycleTest, SetVariant) { tuner->setVariant("original"); tuner->setVariant("some"); tuner->setVariant("none"); }

TEST_F(ZNUltimateCycleTest, CalculateGains) {
    auto gains = ZieglerNicholsUltimateCycle::calculateGains(5.0, 2.0, PIDForm::Parallel);
    EXPECT_GT(gains.Kp, 0.0); EXPECT_GT(gains.Ki, 0.0); EXPECT_GT(gains.Kd, 0.0);
}

TEST_F(ZNUltimateCycleTest, Tune) { tuner->setUltimateParameters(5.0, 2.0); auto result = tuner->tune(*controller, nullptr); }
