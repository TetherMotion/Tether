#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/model_based/PolePlacement.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class PolePlacementTest : public ::testing::Test {
protected:
    std::unique_ptr<PolePlacement> tuner;
    void SetUp() override { tuner = std::make_unique<PolePlacement>(); }
};

TEST_F(PolePlacementTest, GetName) { EXPECT_EQ(tuner->getName(), "Pole Placement"); }
TEST_F(PolePlacementTest, GetDescription) { EXPECT_FALSE(tuner->getDescription().empty()); }

TEST_F(PolePlacementTest, SetDesiredPoles) {
    std::vector<std::complex<double>> poles = {{-1.0,1.0},{-1.0,-1.0}};
    EXPECT_NO_THROW(tuner->setDesiredPoles(poles));
}


TEST_F(PolePlacementTest, SetPolesFromSpecs) {
    tuner->setPolesFromSpecs(1.0, 0.1);
    // design should be runnable after setting specs
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST_F(PolePlacementTest, SetSystemMatrices) {
    std::vector<double> A = {0.0,1.0,-1.0,-1.0};
    std::vector<double> B = {0.0,1.0};
    EXPECT_NO_THROW(tuner->setSystemMatrices(A.data(), B.data(), 2, 1));
}

TEST_F(PolePlacementTest, Tune) {
    std::vector<std::complex<double>> poles = {{-2.0,0.0},{-3.0,0.0}};
    tuner->setDesiredPoles(poles);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST(PolePlacementCoverage, DifferentSpecs) {
    std::vector<std::pair<double,double>> specs = {{0.5,0.0},{1.0,0.05},{2.0,0.1},{0.3,0.02}};
    for (auto& spec: specs) {
        PolePlacement tuner;
        tuner.setPolesFromSpecs(spec.first, spec.second);
        TestPIDController controller;
        TestFOPDTProcessModel processModel(1.0,2.0,0.2);
        auto result = tuner.tune(controller, &processModel);
    }
}
