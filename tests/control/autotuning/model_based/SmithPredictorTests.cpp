#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/SmithPredictor.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class SmithPredictorTest : public ::testing::Test {
protected:
    std::unique_ptr<SmithPredictor> tuner;
    void SetUp() override { tuner = std::make_unique<SmithPredictor>(); }
};

TEST_F(SmithPredictorTest, GetName) { EXPECT_EQ(tuner->getName(), "Smith Predictor"); }
TEST_F(SmithPredictorTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(SmithPredictorTest, SetModel) {
    // setting the internal model should allow tune() to succeed
    FOPDTModel model; model.K = 1.0; model.tau = 10.0; model.L = 2.0;
    tuner->setModel(model);
    TestPIDController controller;
    auto r = tuner->tune(controller, nullptr);
    EXPECT_TRUE(r.success);
}

TEST_F(SmithPredictorTest, SetLambda) {
    // lambda influences resulting parameters
    FOPDTModel model; model.K = 1.0; model.tau = 10.0; model.L = 2.0; tuner->setModel(model);
    TestPIDController controller;

    tuner->setLambda(0.5);
    auto r1 = tuner->tune(controller, nullptr);

    tuner->setLambda(2.0);
    auto r2 = tuner->tune(controller, nullptr);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(SmithPredictorTest, Tune) {
    FOPDTModel model; model.K = 1.0; model.tau = 10.0; model.L = 2.0; tuner->setModel(model); tuner->setLambda(0.5);
    TestPIDController controller; auto result = tuner->tune(controller, nullptr); EXPECT_TRUE(result.success);
}

TEST_F(SmithPredictorTest, Design) {
    FOPDTModel model; model.K = 2.0; model.tau = 5.0; model.L = 1.0;
    auto structure = SmithPredictor::design(model, 1.0);
    EXPECT_GT(structure.innerController.Kp, 0.0);
    EXPECT_EQ(structure.delay, model.L);
}

TEST(SmithPredictorCoverage, DifferentDelays) {
    std::vector<double> delays = {0.5,1.0,2.0,5.0,10.0};
    for (double delay : delays) {
        SmithPredictor tuner;
        FOPDTModel model; model.K = 1.0; model.tau = 5.0; model.L = delay;
        tuner.setModel(model);
        tuner.setLambda(model.tau / 4.0);
        TestPIDController controller;
        auto result = tuner.tune(controller, nullptr);
    }
}
