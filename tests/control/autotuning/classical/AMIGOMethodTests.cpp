#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class AMIGOTest : public ::testing::Test {
protected:
    void SetUp() override { tuner = std::make_unique<AMIGOMethod>(); }
    std::unique_ptr<AMIGOMethod> tuner;
};

TEST_F(AMIGOTest, Name) { EXPECT_EQ(tuner->getName(), "AMIGO"); }
TEST_F(AMIGOTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(AMIGOTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto g = AMIGOMethod::calculateGains(m, PIDForm::Parallel);
    EXPECT_GT(g.Kp, 0.0);
}
