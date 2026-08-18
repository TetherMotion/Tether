#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class CohenCoonTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<CohenCoon>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    std::unique_ptr<CohenCoon> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(CohenCoonTest, Name) { EXPECT_EQ(tuner->getName(), "Cohen-Coon"); }
TEST_F(CohenCoonTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(CohenCoonTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto gains = CohenCoon::calculateGains(m, PIDForm::Parallel);
    EXPECT_GT(gains.Kp, 0.0);
}

TEST_F(CohenCoonTest, Tune) { auto result = tuner->tune(*controller, nullptr); }
