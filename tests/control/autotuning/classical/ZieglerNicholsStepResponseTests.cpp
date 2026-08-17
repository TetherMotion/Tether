#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class ZNStepResponseTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<ZieglerNicholsStepResponse>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
        model = std::make_unique<TestFOPDTProcessModel>(1.0, 10.0, 2.0);
    }
    
    std::unique_ptr<ZieglerNicholsStepResponse> tuner;
    std::shared_ptr<TestPIDController> controller;
    std::unique_ptr<TestFOPDTProcessModel> model;
};

TEST_F(ZNStepResponseTest, Name) { EXPECT_EQ(tuner->getName(), "Ziegler-Nichols Step Response"); }
TEST_F(ZNStepResponseTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(ZNStepResponseTest, Mode) { EXPECT_EQ(tuner->getMode(), AutotuningMode::Offline); }
TEST_F(ZNStepResponseTest, IsCompatible) { EXPECT_TRUE(tuner->isCompatible(*controller)); }
TEST_F(ZNStepResponseTest, SetControllerForm) { tuner->setControllerForm(PIDForm::Parallel); tuner->setControllerForm(PIDForm::Standard); tuner->setControllerForm(PIDForm::Series); }

TEST_F(ZNStepResponseTest, CalculateGainsFromModel) {
    FOPDTModel mdl; mdl.K = 1.0; mdl.tau = 10.0; mdl.L = 2.0;
    auto gains = ZieglerNicholsStepResponse::calculateGains(mdl, PIDForm::Parallel);
    EXPECT_GT(gains.Kp, 0.0); EXPECT_GT(gains.Ki, 0.0);
}

TEST_F(ZNStepResponseTest, Tune) { auto result = tuner->tune(*controller, model.get()); }
