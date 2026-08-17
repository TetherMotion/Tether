#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class LopezTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<LopezMethod>();
    }
    std::unique_ptr<LopezMethod> tuner;
};

TEST_F(LopezTest, Name) { EXPECT_EQ(tuner->getName(), "Lopez (ITAE/IAE/ISE)"); }
TEST_F(LopezTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(LopezTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto g = LopezMethod::calculateGains(m, PIDForm::Parallel, LopezMethod::Criterion::ITAE, LopezMethod::ResponseType::Setpoint);
    EXPECT_GT(g.Kp, 0.0);
}
