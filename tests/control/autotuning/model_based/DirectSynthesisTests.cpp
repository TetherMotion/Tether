#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/DirectSynthesis.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class DirectSynthesisTest : public ::testing::Test {
protected:
    std::unique_ptr<DirectSynthesis> tuner;
    void SetUp() override { tuner = std::make_unique<DirectSynthesis>(); }
};

TEST_F(DirectSynthesisTest, GetName) { EXPECT_EQ(tuner->getName(), "Direct Synthesis"); }
TEST_F(DirectSynthesisTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(DirectSynthesisTest, SetClosedLoopTimeConstant) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,5.0,0.5);

    tuner->setClosedLoopTimeConstant(0.5);
    auto r1 = tuner->tune(controller, &model);

    tuner->setClosedLoopTimeConstant(2.0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(DirectSynthesisTest, SetClosedLoopDamping) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0,5.0,0.5);

    tuner->setClosedLoopDamping(0.5);
    auto r1 = tuner->tune(controller, &model);

    tuner->setClosedLoopDamping(1.0);
    auto r2 = tuner->tune(controller, &model);

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_NE(r1.parameters[0], r2.parameters[0]);
}

TEST_F(DirectSynthesisTest, Tune) {
    tuner->setClosedLoopTimeConstant(2.0);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0,10.0,1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST_F(DirectSynthesisTest, DesignForFOPDT) {
    FOPDTModel model;
    model.K = 1.0; model.tau = 10.0; model.L = 1.0;
    auto gains = DirectSynthesis::designForFOPDT(model, 2.0);
    EXPECT_GT(gains.Kp, 0.0);
}

TEST(DirectSynthesisCoverage, StaticDesignMethods) {
    FOPDTModel fopdtModel; fopdtModel.K = 1.0; fopdtModel.tau = 5.0; fopdtModel.L = 1.0;
    auto gainsF = DirectSynthesis::designForFOPDT(fopdtModel, 1.0);
    EXPECT_GT(gainsF.Kp, 0.0);

    SOPDTModel sopdtModel; sopdtModel.K = 1.0; sopdtModel.tau1 = 3.0; sopdtModel.tau2 = 2.0; sopdtModel.L = 0.5;
    auto gainsS = DirectSynthesis::designForSOPDT(sopdtModel, 1.0, 0.707);
    EXPECT_GT(gainsS.Kp, 0.0);
}

TEST(DirectSynthesisCoverage, DifferentCLTimeConstants) {
    std::vector<double> tauCLs = {0.5,1.0,2.0,5.0};
    for (double tauCL : tauCLs) {
        DirectSynthesis tuner;
        tuner.setClosedLoopTimeConstant(tauCL);
        tuner.setClosedLoopDamping(0.707);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.0,3.0,0.5);
        auto result = tuner.tune(controller, &processModel);
    }
}
