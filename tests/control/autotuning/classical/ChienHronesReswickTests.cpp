#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

class CHRTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<ChienHronesReswick>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    std::unique_ptr<ChienHronesReswick> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(CHRTest, Name) { EXPECT_EQ(tuner->getName(), "Chien-Hrones-Reswick"); }
TEST_F(CHRTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(CHRTest, CalculateGains) {
    FOPDTModel m; m.K = 1.0; m.tau = 10.0; m.L = 2.0;
    auto g = ChienHronesReswick::calculateGains(m, PIDForm::Parallel, ChienHronesReswick::Mode::SetpointNoOvershoot);
    EXPECT_GT(g.Kp, 0.0);
}

TEST_F(CHRTest, Tune) { auto result = tuner->tune(*controller, nullptr); }
