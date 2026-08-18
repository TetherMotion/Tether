#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/MinimumVarianceControl.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class MinimumVarianceControlTest : public ::testing::Test {
protected:
    std::unique_ptr<MinimumVarianceControl> tuner;
    void SetUp() override { tuner = std::make_unique<MinimumVarianceControl>(); }
};

TEST_F(MinimumVarianceControlTest, GetName) { EXPECT_EQ(tuner->getName(), "Minimum Variance Control"); }
TEST_F(MinimumVarianceControlTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(MinimumVarianceControlTest, SetARMAXModel) {
    std::vector<double> A = {1.0, -0.9};
    std::vector<double> B = {0.0, 0.5};
    std::vector<double> C = {1.0, 0.1};
    EXPECT_NO_THROW(tuner->setARMAXModel(A, B, C, 1));
}

TEST_F(MinimumVarianceControlTest, SetControlWeight) {
    // control weight should change the tuning result
    std::vector<double> A = {1.0, -0.8};
    std::vector<double> B = {0.0, 0.4};
    std::vector<double> C = {1.0};
    tuner->setARMAXModel(A, B, C, 1);

    tuner->setControlWeight(0.01);
    TestPIDController controller;
    auto r1 = tuner->tune(controller, nullptr);

    tuner->setControlWeight(0.5);
    auto r2 = tuner->tune(controller, nullptr);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(MinimumVarianceControlTest, Tune) {
    std::vector<double> A = {1.0, -0.8};
    std::vector<double> B = {0.0, 0.4};
    std::vector<double> C = {1.0};
    tuner->setARMAXModel(A, B, C, 1);
    tuner->setControlWeight(0.05);
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

TEST(MinimumVarianceCoverage, DifferentControlWeights) {
    std::vector<double> lambdas = {0.0, 0.01, 0.1, 0.5, 1.0};
    std::vector<double> A = {1.0, -0.8};
    std::vector<double> B = {0.0, 0.4};
    std::vector<double> C = {1.0, 0.2};
    for (double lambda : lambdas) {
        MinimumVarianceControl tuner;
        tuner.setARMAXModel(A, B, C, 1);
        tuner.setControlWeight(lambda);
        TestPIDController controller;
        auto result = tuner.tune(controller, nullptr);
    }
}
