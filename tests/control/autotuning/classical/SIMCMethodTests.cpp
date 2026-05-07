#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class SIMCTest : public ::testing::Test {
protected:
    void SetUp() override { tuner = std::make_unique<SIMCMethod>(); }
    std::unique_ptr<SIMCMethod> tuner;
};

TEST_F(SIMCTest, Name) { EXPECT_EQ(tuner->getName(), "SIMC (Skogestad)"); }
TEST_F(SIMCTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(SIMCTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto g = SIMCMethod::calculateGains(m, 1.0);
    EXPECT_GT(g.Kp, 0.0);
}
