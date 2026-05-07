#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/LoopShaping.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class LoopShapingTest : public ::testing::Test {
protected:
    std::unique_ptr<LoopShaping> tuner;
    void SetUp() override { tuner = std::make_unique<LoopShaping>(); }
};

TEST_F(LoopShapingTest, GetName) { EXPECT_EQ(tuner->getName(), "Loop Shaping"); }
TEST_F(LoopShapingTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(LoopShapingTest, SetCrossoverFrequency) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,10.0,1.0);

    tuner->setCrossoverFrequency(0.5);
    tuner->setPhaseMargin(50.0);
    auto r1 = tuner->tune(controller, &model);

    tuner->setCrossoverFrequency(2.0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(LoopShapingTest, SetPhaseMargin) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,10.0,1.0);

    tuner->setCrossoverFrequency(1.0);
    tuner->setPhaseMargin(30.0);
    auto r1 = tuner->tune(controller, &model);

    tuner->setPhaseMargin(60.0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(LoopShapingTest, SetGainMargin) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,10.0,1.0);

    tuner->setGainMargin(6.0);
    tuner->setPhaseMargin(50.0);
    auto r1 = tuner->tune(controller, &model);

    tuner->setGainMargin(12.0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(LoopShapingTest, SetLowFreqSlope) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,10.0,1.0);

    tuner->setLowFreqSlope(-1);
    tuner->setCrossoverFrequency(1.0);
    auto r1 = tuner->tune(controller, &model);

    tuner->setLowFreqSlope(0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(LoopShapingTest, SetHighFreqRolloff) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,10.0,1.0);

    tuner->setHighFreqRolloff(-1);
    tuner->setCrossoverFrequency(1.0);
    auto r1 = tuner->tune(controller, &model);

    tuner->setHighFreqRolloff(-3);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(LoopShapingTest, Tune) {
    tuner->setCrossoverFrequency(1.0);
    tuner->setPhaseMargin(45.0);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0,10.0,1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST(LoopShapingCoverage, DifferentCrossoverFrequencies) {
    std::vector<double> omegas = {0.5,1.0,2.0,5.0};
    for (double omega : omegas) {
        LoopShaping tuner;
        tuner.setCrossoverFrequency(omega);
        tuner.setPhaseMargin(50.0);
        tuner.setGainMargin(8.0);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.0,5.0,0.5);
        auto result = tuner.tune(controller, &processModel);
    }
}

TEST(LoopShapingCoverage, StaticLeadLagDesign) {
    TestFOPDTProcessModel processModel(1.0,5.0,0.5);
    auto tf = LoopShaping::designLeadLag(processModel, 2.0, 45.0);
}
