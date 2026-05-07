#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class LambdaTest : public ::testing::Test {
protected:
    void SetUp() override { tuner = std::make_unique<LambdaTuning>(); }
    std::unique_ptr<LambdaTuning> tuner;
};

TEST_F(LambdaTest, Name) { EXPECT_EQ(tuner->getName(), "Lambda/IMC Tuning"); }
TEST_F(LambdaTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(LambdaTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto g = LambdaTuning::calculateGains(m, 2.0, false);
    EXPECT_GT(g.Kp, 0.0);
}
