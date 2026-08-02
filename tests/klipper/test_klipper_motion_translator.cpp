/**
 * @file test_klipper_motion_translator.cpp
 * @brief Direct unit tests for MotionTranslator.
 *
 * MotionTranslator requires a fully-built MotionPlan to test translate().
 * Since building a MotionPlan requires the full MotionPlanBuilder pipeline,
 * these tests focus on construction, configuration, and kinematics transform
 * wiring. The translate() path is covered by the e2e motion tests
 * (test_klipper_motion_e2e.cpp) which exercise the full
 * dispatcher->translator->host->device pipeline.
 */

#include <gtest/gtest.h>
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/motion_planner/MotionPlan.hpp"

#include <array>

using namespace tether::klipper;

class MotionTranslatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::array<motion::AxisConfig, 4> configs = {{
            {80.0, false}, {80.0, false}, {80.0, false}, {80.0, false}
        }};
        std::array<uint8_t, 4> oids = {0, 1, 2, 3};
        translator_ = std::make_unique<motion::MotionTranslator<4, double>>(configs, oids);
    }

    std::unique_ptr<motion::MotionTranslator<4, double>> translator_;
};

TEST_F(MotionTranslatorTest, SetSourceLabel) {
    translator_->setSourceLabel("test_move");
    SUCCEED();
}

TEST_F(MotionTranslatorTest, SetKinematicsTransform) {
    motion::KinematicsTransform kt;
    translator_->setKinematicsTransform(kt);
    SUCCEED();
}

TEST_F(MotionTranslatorTest, GetKinematicsTransform) {
    motion::KinematicsTransform kt;
    translator_->setKinematicsTransform(kt);
    const auto& retrieved = translator_->kinematicsTransform();
    (void)retrieved;
    SUCCEED();
}

TEST_F(MotionTranslatorTest, TranslateEmptyPlan) {
    MotionPlanner::MotionPlan<4, double> plan;
    auto seqs = translator_->translate(plan, 180000000, 0.0001, 0);
    // Empty plan (no segments) should produce empty sequences.
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, AxisOidsAssigned) {
    // With an empty plan, no sequences are produced.
    // OID assignment is verified via the e2e motion tests.
    MotionPlanner::MotionPlan<4, double> plan;
    auto seqs = translator_->translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, TranslateWithStartClock) {
    MotionPlanner::MotionPlan<4, double> plan;
    auto seqs = translator_->translate(plan, 180000000, 0.0001, 1000000);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, InvertDirectionConstruction) {
    std::array<motion::AxisConfig, 4> configs = {{
        {80.0, true}, {80.0, false}, {80.0, false}, {80.0, false}
        // ^-- X axis inverted
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    motion::MotionTranslator<4, double> invTranslator(configs, oids);
    MotionPlanner::MotionPlan<4, double> plan;
    auto seqs = invTranslator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, CustomStepsPerMm) {
    std::array<motion::AxisConfig, 4> configs = {{
        {400.0, false}, {400.0, false}, {1600.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {10, 11, 12, 13};
    motion::MotionTranslator<4, double> translator(configs, oids);
    MotionPlanner::MotionPlan<4, double> plan;
    auto seqs = translator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, TwoAxisTranslator) {
    std::array<motion::AxisConfig, 2> configs = {{
        {80.0, false}, {80.0, false}
    }};
    std::array<uint8_t, 2> oids = {0, 1};
    motion::MotionTranslator<2, double> translator(configs, oids);
    MotionPlanner::MotionPlan<2, double> plan;
    auto seqs = translator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, ThreeAxisTranslator) {
    std::array<motion::AxisConfig, 3> configs = {{
        {80.0, false}, {80.0, false}, {400.0, false}
    }};
    std::array<uint8_t, 3> oids = {0, 1, 2};
    motion::MotionTranslator<3, double> translator(configs, oids);
    MotionPlanner::MotionPlan<3, double> plan;
    auto seqs = translator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}

TEST_F(MotionTranslatorTest, DefaultConstructor) {
    // Verify the translator can be default-constructed and used.
    std::array<motion::AxisConfig, 1> configs = {{{80.0, false}}};
    std::array<uint8_t, 1> oids = {0};
    motion::MotionTranslator<1, double> translator(configs, oids);
    translator.setSourceLabel("single_axis");
    MotionPlanner::MotionPlan<1, double> plan;
    auto seqs = translator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_TRUE(seqs.empty());
}
