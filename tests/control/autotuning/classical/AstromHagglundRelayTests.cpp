#include <gtest/gtest.h>
#include "../TestHelpers.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

class RelayTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<AstromHagglundRelay>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    std::unique_ptr<AstromHagglundRelay> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(RelayTest, Name) { EXPECT_EQ(tuner->getName(), "Åström-Hägglund Relay"); }
TEST_F(RelayTest, Description) { EXPECT_FALSE(tuner->getDescription().empty()); }
TEST_F(RelayTest, Config) { AstromHagglundRelay::Config c; c.relayAmplitude = 1.2; tuner->setConfig(c); }
TEST_F(RelayTest, SetTuningRule) { tuner->setTuningRule(AstromHagglundRelay::TuningRule::ZieglerNichols); }
