#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/IMCDesign.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class IMCDesignTest : public ::testing::Test {
protected:
    std::unique_ptr<IMCDesign> tuner;
    void SetUp() override { tuner = std::make_unique<IMCDesign>(); }
};

TEST_F(IMCDesignTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Internal Model Control");
}

TEST_F(IMCDesignTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(IMCDesignTest, SetFilterTimeConstant) {
    EXPECT_NO_THROW(tuner->setFilterTimeConstant(0.5));
}

TEST_F(IMCDesignTest, SetFilterOrder) {
    EXPECT_NO_THROW(tuner->setFilterOrder(2));
}

TEST_F(IMCDesignTest, Tune) {
    tuner->setFilterTimeConstant(0.5);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST_F(IMCDesignTest, DesignForFOPDT) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 1.0;
    auto gains = IMCDesign::designForFOPDT(model, 0.5);
    EXPECT_GT(gains.Kp, 0.0);
    EXPECT_GT(gains.Ti, 0.0);
}

TEST(IMCDesignCoverage, DifferentFilterOrders) {
    for (int order = 1; order <= 2; ++order) {
        IMCDesign tuner;
        tuner.setFilterTimeConstant(0.8);
        tuner.setFilterOrder(order);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.5, 4.0, 0.3);
        auto result = tuner.tune(controller, &processModel);
    }
}

TEST(IMCDesignCoverage, DifferentLambdaValues) {
    std::vector<double> lambdas = {0.1, 0.5, 1.0, 2.0, 5.0};
    for (double lambda : lambdas) {
        IMCDesign tuner;
        tuner.setFilterTimeConstant(lambda);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(2.0, 5.0, 0.5);
        auto result = tuner.tune(controller, &processModel);
        EXPECT_FALSE(result.message.empty());
    }
}

TEST(IMCDesignCoverage, HighDelayProcess) {
    IMCDesign tuner;
    tuner.setFilterTimeConstant(1.0);
    tuner.setFilterOrder(2);
    TestFOPDTProcessModel processModel(1.5, 5.0, 3.0);
    TestPIDController controller;
    auto result = tuner.tune(controller, &processModel);
}

TEST(IMCDesignCoverage, FastProcess) {
    IMCDesign tuner;
    tuner.setFilterTimeConstant(0.3);
    tuner.setFilterOrder(1);
    TestFOPDTProcessModel processModel(1.0, 2.0, 0.5);
    TestPIDController controller;
    auto result = tuner.tune(controller, &processModel);
}
