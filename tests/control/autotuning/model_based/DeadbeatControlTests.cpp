#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/DeadbeatControl.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class DeadbeatControlTest : public ::testing::Test {
protected:
    std::unique_ptr<DeadbeatControl> tuner;
    void SetUp() override { tuner = std::make_unique<DeadbeatControl>(); }
};

TEST_F(DeadbeatControlTest, GetName) { EXPECT_EQ(tuner->getName(), "Deadbeat Control"); }
TEST_F(DeadbeatControlTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(DeadbeatControlTest, SetSampleTime) {
    // changing sample time should change the tuned parameters
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 10.0, 1.0);

    tuner->setSampleTime(0.05);
    auto r1 = tuner->tune(controller, &model);

    tuner->setSampleTime(0.10);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);  // Kp should change with Ts
    EXPECT_NE(r1.parameters[1], r2.parameters[1]);  // Ki should change with Ts
}

TEST_F(DeadbeatControlTest, SetSettlingSamples) {
    // changing settling samples affects the resulting controller and reported settlingTime
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 10.0, 1.0);

    tuner->setSampleTime(0.1);
    tuner->setSettlingSamples(0);
    auto r0 = tuner->tune(controller, &model);

    tuner->setSettlingSamples(3);
    auto r3 = tuner->tune(controller, &model);

    EXPECT_TRUE(r0.success);
    EXPECT_TRUE(r3.success);
    EXPECT_NE(r0.parameters[0], r3.parameters[0]);
    EXPECT_DOUBLE_EQ(r3.settlingTime, 0.1 * 3);
}

TEST_F(DeadbeatControlTest, Tune) {
    tuner->setSampleTime(0.1);
    tuner->setSettlingSamples(0);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0,10.0,1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST(DeadbeatCoverage, DifferentSettlingSamples) {
    std::vector<int> samples = {0,1,2,3};
    for (int N : samples) {
        DeadbeatControl tuner;
        tuner.setSampleTime(0.1);
        tuner.setSettlingSamples(N);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.0,2.0,0.1);
        auto result = tuner.tune(controller, &processModel);
    }
}
