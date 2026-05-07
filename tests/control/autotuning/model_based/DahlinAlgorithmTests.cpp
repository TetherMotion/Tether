#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/DahlinAlgorithm.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class DahlinAlgorithmTest : public ::testing::Test {
protected:
    std::unique_ptr<DahlinAlgorithm> tuner;
    void SetUp() override { tuner = std::make_unique<DahlinAlgorithm>(); }
};

TEST_F(DahlinAlgorithmTest, GetName) { EXPECT_EQ(tuner->getName(), "Dahlin's Algorithm"); }
TEST_F(DahlinAlgorithmTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(DahlinAlgorithmTest, SetSampleTime) {
    // sample time should affect digital design results
    FOPDTModel model; model.K = 1.0; model.tau = 5.0; model.L = 0.5;

    auto c1 = DahlinAlgorithm::designDigital(model, 0.05, 1.0);
    auto c2 = DahlinAlgorithm::designDigital(model, 0.10, 1.0);

    EXPECT_EQ(c1.size(), 3u);
    EXPECT_EQ(c2.size(), 3u);
    EXPECT_NE(c1[0], c2[0]);
}

TEST_F(DahlinAlgorithmTest, SetLambda) {
    // lambda influences the design — different lambda => different coefficients
    FOPDTModel model; model.K = 1.0; model.tau = 5.0; model.L = 0.5;

    auto c1 = DahlinAlgorithm::designDigital(model, 0.1, 0.5);
    auto c2 = DahlinAlgorithm::designDigital(model, 0.1, 2.0);

    EXPECT_EQ(c1.size(), 3u);
    EXPECT_EQ(c2.size(), 3u);
    EXPECT_NE(c1[0], c2[0]);
}

TEST_F(DahlinAlgorithmTest, Tune) {
    tuner->setSampleTime(0.1);
    tuner->setLambda(1.0);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0,10.0,1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST_F(DahlinAlgorithmTest, DesignDigital) {
    FOPDTModel model; model.K = 1.0; model.tau = 5.0; model.L = 0.5;
    auto coeffs = DahlinAlgorithm::designDigital(model, 0.1, 1.0);
    EXPECT_EQ(coeffs.size(), 3u);
    EXPECT_TRUE(std::isfinite(coeffs[0]));
    EXPECT_TRUE(std::isfinite(coeffs[1]));
    EXPECT_TRUE(std::isfinite(coeffs[2]));
}

TEST(DahlinCoverage, StaticDesign) {
    FOPDTModel model; model.K = 1.0; model.tau = 5.0; model.L = 1.0;
    auto coeffs = DahlinAlgorithm::designDigital(model, 0.1, 1.0);
    EXPECT_TRUE(std::isfinite(coeffs[0]));
    EXPECT_TRUE(std::isfinite(coeffs[1]));
    EXPECT_TRUE(std::isfinite(coeffs[2]));
}

TEST(DahlinCoverage, DifferentSampleTimes) {
    std::vector<double> sampleTimes = {0.01,0.05,0.1,0.5};
    for (double Ts : sampleTimes) {
        DahlinAlgorithm tuner;
        tuner.setSampleTime(Ts);
        tuner.setLambda(1.0);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.0,5.0,0.5);
        auto result = tuner.tune(controller, &processModel);
    }
}
