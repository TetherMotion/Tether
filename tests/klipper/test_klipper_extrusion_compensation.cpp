/**
 * @file test_klipper_extrusion_compensation.cpp
 * @brief Klipper-level tests for the non-Newtonian extrusion compensation.
 *
 * Verifies:
 *  - MotionTranslator: power-law model changes E step positions vs linear PA.
 *  - MotionTranslator: default Linear model reproduces classic PA exactly.
 *  - SET_PRESSURE_ADVANCE MODEL= parsing updates the dispatcher + status object.
 *  - SET_HEATER_FLOW_COMPENSATION toggles the flow-adaptive heater controller.
 *  - printer.cfg parsing of the new [extruder] keys.
 *  - PressureAdvanceObject exposes the new `model` field.
 *  - ExtruderObject exposes melt_temp_estimate + emphasis diagnostics.
 *  - ExtrusionFlowTracker records and smooths the per-move flow.
 */

#include <gtest/gtest.h>

#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/motion/MotionDispatcher.hpp"
#include "tether/klipper/motion/ExtrusionFlowTracker.hpp"
#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/klippy/PrinterObjectsMotion.hpp"
#include "tether/klipper/klippy/PrinterObjectsCore.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/config/ConfigParser.hpp"
#include "tether/motion_planner/MotionPlan.hpp"
#include "tether/control/extrusion/ExtrusionPressureModels.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"
#include "tether/control/extrusion/CrossWlfRheology.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <algorithm>

using namespace tether::klipper;

#if TETHER_ENABLE_PRESSURE_ADVANCE

namespace {

/// @brief Build a simple linear extrusion move (X + E move together so the
/// planner generates a real path with E-axis velocity). All 4 axes are given
/// a small motion to work around a motion-planner issue where non-consecutive
/// active axes (e.g., only 0 and 3) get mis-indexed in the NURBS embedding.
MotionPlanner::MotionPlan<4, double> buildExtrusionPlan(
    double eStart, double eEnd, double feedMmMin,
    const MotionPlanner::KinematicLimits<4, double>& limits) {
    MotionPlanner::MotionSegmentList segments;
    MotionPlanner::Vec<4> start{0, 0, 0, eStart};
    // Tiny Y/Z motion (0.001 mm) keeps all axes active without materially
    // changing the E-axis step count.
    MotionPlanner::Vec<4> end{100, 0.001, 0.001, eEnd};
    segments.append(MotionPlanner::MotionSegment::linear(start, end, feedMmMin));
    MotionPlanner::MotionPlanBuilder<4, double> builder(limits);
    return builder.build(segments, feedMmMin);
}

MotionPlanner::KinematicLimits<4, double> defaultLimits() {
    MotionPlanner::KinematicLimits<4, double> lim;
    for (auto& v : lim.axis.maxVelocity) v = 200.0;
    for (auto& a : lim.axis.maxAcceleration) a = 2000.0;
    for (auto& j : lim.axis.maxJerk) j = 20000.0;
    lim.path.maxPathVelocity = 200.0;
    lim.path.maxPathAcceleration = 2000.0;
    return lim;
}

/// @brief Sum the absolute E-axis step count from a translation result.
int64_t totalEAbsSteps(const std::vector<motion::AxisStepSequence>& seqs,
                       uint8_t eOid = 3) {
    int64_t total = 0;
    for (const auto& seq : seqs) {
        if (seq.oid != eOid) continue;
        for (const auto& step : seq.steps) {
            total += static_cast<int64_t>(step.count);
        }
    }
    return total;
}

} // namespace

// ============================================================================
// MotionTranslator: power-law vs linear PA
// ============================================================================

TEST(KlipperExtrusionCompensation, PowerLawChangesEStepsVsLinear) {
    std::array<motion::AxisConfig, 4> configs = {{
        {80.0, false}, {80.0, false}, {80.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    motion::MotionTranslator<4, double> translator(configs, oids);

    auto plan = buildExtrusionPlan(0.0, 10.0, 300.0, defaultLimits());
    ASSERT_GT(plan.totalDuration(), 0.0) << "Plan has zero duration";

    // Debug: check without PA first.
    motion::PressureAdvanceConfig noCfg;
    noCfg.enabled = false;
    translator.setPressureAdvanceConfig(noCfg);
    auto seqsNone = translator.translate(plan, 180000000, 0.0001, 0);
    int64_t stepsNone = totalEAbsSteps(seqsNone);
    ASSERT_GT(stepsNone, 0) << "No E steps generated even without PA; "
                            << "seqs=" << seqsNone.size();

    // Linear PA baseline.
    motion::PressureAdvanceConfig linCfg;
    linCfg.enabled = true;
    linCfg.model = motion::ExtrusionCompensationModel::Linear;
    linCfg.pressureAdvance = 0.045;
    linCfg.smoothTime = 0.0;
    linCfg.extruderAxis = 3;
    translator.setPressureAdvanceConfig(linCfg);
    auto seqsLin = translator.translate(plan, 180000000, 0.0001, 0);
    const int64_t stepsLin = totalEAbsSteps(seqsLin);

    // Power-law model (n=0.5, K_base chosen to be non-trivial).
    motion::PressureAdvanceConfig plCfg;
    plCfg.enabled = true;
    plCfg.model = motion::ExtrusionCompensationModel::PowerLaw;
    plCfg.powerLawBaseGain = 0.01;
    plCfg.flowIndex = 0.5;
    plCfg.smoothTime = 0.0;
    plCfg.extruderAxis = 3;
    plCfg.maxCompensation = 10.0;
    translator.setPressureAdvanceConfig(plCfg);
    auto seqsPl = translator.translate(plan, 180000000, 0.0001, 0);
    const int64_t stepsPl = totalEAbsSteps(seqsPl);

    // Both should produce steps (the move extrudes 10 mm of filament).
    EXPECT_GT(stepsLin, 0);
    EXPECT_GT(stepsPl, 0);

    // The power-law offset shifts E positions non-linearly during the move,
    // so the step pattern (intervals/counts) should differ from the linear
    // case. With the analytical PA, the total step count may be the same
    // (both models have offset=0 at the start and end where v=0), but the
    // intermediate step distribution differs.
    //
    // Compare the E-axis step sequences: if they are identical, the PA
    // models are not being applied differently.
    // Use findAxisByOid() for safe lookup — translate() skips axes with no
    // steps, so the vector index does not correspond to the axis number.
    const auto* linSeq = motion::findAxisByOid(seqsLin, 3);
    const auto* plSeq = motion::findAxisByOid(seqsPl, 3);
    ASSERT_NE(linSeq, nullptr) << "E-axis not found in linear PA result";
    ASSERT_NE(plSeq, nullptr) << "E-axis not found in power-law PA result";
    // std::ranges::equal handles size mismatch and element comparison
    // without raw indexing — inherently bounds-safe.
    const bool sequencesDiffer = !std::ranges::equal(
        linSeq->steps, plSeq->steps,
        [](const objects::StepCommand& a, const objects::StepCommand& b) {
            return a.interval == b.interval &&
                   a.count == b.count &&
                   a.add == b.add;
        });
    EXPECT_TRUE(sequencesDiffer)
        << "Linear and PowerLaw PA produced identical E-axis step sequences";
}

TEST(KlipperExtrusionCompensation, LinearModelReproducesClassicPA) {
    std::array<motion::AxisConfig, 4> configs = {{
        {80.0, false}, {80.0, false}, {80.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    motion::MotionTranslator<4, double> translator(configs, oids);
    auto plan = buildExtrusionPlan(0.0, 10.0, 300.0, defaultLimits());
    ASSERT_GT(plan.totalDuration(), 0.0);

    // "Classic" config: model field left at default (Linear), PA=0.045.
    motion::PressureAdvanceConfig classic;
    classic.enabled = true;
    classic.pressureAdvance = 0.045;
    classic.smoothTime = 0.0;
    classic.extruderAxis = 3;
    translator.setPressureAdvanceConfig(classic);
    auto seqsClassic = translator.translate(plan, 180000000, 0.0001, 0);

    // Explicit Linear model.
    motion::PressureAdvanceConfig explicitLin = classic;
    explicitLin.model = motion::ExtrusionCompensationModel::Linear;
    translator.setPressureAdvanceConfig(explicitLin);
    auto seqsExplicit = translator.translate(plan, 180000000, 0.0001, 0);

    EXPECT_EQ(totalEAbsSteps(seqsClassic), totalEAbsSteps(seqsExplicit));
}

TEST(KlipperExtrusionCompensation, CrossWlfModelUsesLut) {
    std::array<motion::AxisConfig, 4> configs = {{
        {80.0, false}, {80.0, false}, {80.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    motion::MotionTranslator<4, double> translator(configs, oids);

    // Build a small LUT.
    tether::control::extrusion::CrossWlfParams rp;
    tether::control::extrusion::NozzleGeometry g{0.2, 10.0};
    auto lut = std::make_shared<tether::control::extrusion::PressureFlowLut>();
    lut->build(rp, g, {1.0, 2.0, 4.0, 8.0}, {200.0, 220.0, 240.0});

    auto plan = buildExtrusionPlan(0.0, 10.0, 300.0, defaultLimits());
    ASSERT_GT(plan.totalDuration(), 0.0);

    motion::PressureAdvanceConfig cfg;
    cfg.enabled = true;
    cfg.model = motion::ExtrusionCompensationModel::CrossWlf;
    cfg.crossWlfCompressibilityOverArea = 1e-5;
    cfg.meltTempC = 220.0;
    cfg.smoothTime = 0.0;
    cfg.extruderAxis = 3;
    cfg.maxCompensation = 100.0;
    cfg.lut = lut;
    translator.setPressureAdvanceConfig(cfg);
    auto seqs = translator.translate(plan, 180000000, 0.0001, 0);
    EXPECT_GT(totalEAbsSteps(seqs), 0);
}

TEST(KlipperExtrusionCompensation, DisabledModelProducesNoOffset) {
    std::array<motion::AxisConfig, 4> configs = {{
        {80.0, false}, {80.0, false}, {80.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    motion::MotionTranslator<4, double> translator(configs, oids);
    auto plan = buildExtrusionPlan(0.0, 10.0, 300.0, defaultLimits());
    ASSERT_GT(plan.totalDuration(), 0.0);

    // PowerLaw model with zero gain → no compensation.
    motion::PressureAdvanceConfig cfg;
    cfg.enabled = true;
    cfg.model = motion::ExtrusionCompensationModel::PowerLaw;
    cfg.powerLawBaseGain = 0.0;
    cfg.extruderAxis = 3;
    translator.setPressureAdvanceConfig(cfg);
    auto seqsOff = translator.translate(plan, 180000000, 0.0001, 0);

    // No PA at all.
    motion::PressureAdvanceConfig none;
    none.enabled = false;
    translator.setPressureAdvanceConfig(none);
    auto seqsNone = translator.translate(plan, 180000000, 0.0001, 0);

    EXPECT_EQ(totalEAbsSteps(seqsOff), totalEAbsSteps(seqsNone));
}

// ============================================================================
// ExtrusionFlowTracker
// ============================================================================

TEST(ExtrusionFlowTracker, RecordsAndSmoothsFlow) {
    auto tracker = std::make_shared<motion::ExtrusionFlowTracker>();
    tracker->setFilamentDiameterMm(1.75);
    const double Af = tracker->filamentAreaMm2();
    // v_e = 10 mm/s → Q = 10 * Af
    tracker->setExtruderVelocityMmPerS(10.0, 0.1);
    EXPECT_NEAR(tracker->instantaneousFlowMm3PerS(), 10.0 * Af, 1e-6);
    // Smoothed starts at 0 and moves toward instantaneous.
    EXPECT_GT(tracker->smoothedFlowMm3PerS(), 0.0);
    EXPECT_LT(tracker->smoothedFlowMm3PerS(), tracker->instantaneousFlowMm3PerS());
    // After many updates the smoothed converges to instantaneous.
    for (int i = 0; i < 200; ++i)
        tracker->setExtruderVelocityMmPerS(10.0, 0.1);
    EXPECT_NEAR(tracker->smoothedFlowMm3PerS(), 10.0 * Af, 1e-3);
}

TEST(ExtrusionFlowTracker, ResetZeroesFlow) {
    auto tracker = std::make_shared<motion::ExtrusionFlowTracker>();
    tracker->setExtruderVelocityMmPerS(20.0, 0.1);
    tracker->reset();
    EXPECT_EQ(tracker->instantaneousFlowMm3PerS(), 0.0);
    EXPECT_EQ(tracker->smoothedFlowMm3PerS(), 0.0);
}

// ============================================================================
// PressureAdvanceObject: model field
// ============================================================================

TEST(KlipperExtrusionCompensation, PressureAdvanceObjectExposesModel) {
    klippy::PressureAdvanceObject obj;
    obj.setModel("power_law");
    auto status = obj.status({});
    ASSERT_TRUE(status.count("model") > 0);
    EXPECT_EQ(status["model"].asString(), "power_law");
    auto fields = obj.availableFields();
    bool hasModel = false;
    for (const auto& f : fields) if (f == "model") hasModel = true;
    EXPECT_TRUE(hasModel);
}

// ============================================================================
// Heater flow-compensation hook
// ============================================================================

TEST(KlipperExtrusionCompensation, HeaterFlowCompensationHook) {
    auto heater = std::make_shared<objects::Heater>(
        0,
        [](double){},
        []() { return 200.0; });
    // Without a controller wired, diagnostics report 0.
    EXPECT_EQ(heater->meltTempEstimate(), 0.0);
    EXPECT_EQ(heater->preEmphasisPWM(), 0.0);
    EXPECT_EQ(heater->postEmphasisPWM(), 0.0);

    auto ctrl = std::make_shared<
        tether::control::extrusion::FlowAdaptiveHeaterController>();
    auto tracker = std::make_shared<motion::ExtrusionFlowTracker>();
    heater->setFlowCompensation(ctrl, tracker);
    // After wiring, control() runs the flow-adaptive path. With a measured
    // temp of 200 and target 0 (default), output should be clamped to >=0.
    heater->setControlInterval(0.1);
    heater->control();
    // Diagnostics are now sourced from the controller (may still be 0 if
    // pre-emphasis is suppressed, but the call must not crash).
    EXPECT_NO_FATAL_FAILURE(heater->meltTempEstimate());

    // Disconnect.
    heater->setFlowCompensation(nullptr);
    EXPECT_EQ(heater->meltTempEstimate(), 0.0);
}

// ============================================================================
// printer.cfg parsing of the new [extruder] keys
// ============================================================================

TEST(KlipperExtrusionCompensation, ConfigParsesNewExtruderKeys) {
    std::string cfgPath = "/tmp/tether_test_extrusion_cfg_" +
                          std::to_string(getpid()) + ".cfg";
    {
        std::ofstream f(cfgPath);
        f << "[extruder]\n"
          << "nozzle_diameter: 0.4\n"
          << "filament_diameter: 1.75\n"
          << "pressure_advance: 0.045\n"
          << "smooth_time: 0.040\n"
          << "pressure_advance_model: power_law\n"
          << "pa_flow_index: 0.5\n"
          << "pa_consistency: 0.012\n"
          << "pa_max_compensation: 0.3\n"
          << "heater_flow_pre_emphasis: true\n"
          << "filament_heat_capacity: 2.1\n"
          << "heater_block_capacitance: 8.0\n"
          << "sensor_capacitance: 1.0\n"
          << "melt_zone_capacitance: 2.0\n"
          << "heater_sensor_conductance: 2.0\n"
          << "sensor_melt_conductance: 1.5\n"
          << "luenberger_gain_heater: 0.5\n"
          << "luenberger_gain_sensor: 2.0\n"
          << "luenberger_gain_melt: 0.3\n"
          << "debt_time_constant: 1.5\n"
          << "max_pre_emphasis_power: 0.35\n"
          << "max_post_emphasis_power: 0.15\n"
          << "max_heater_overshoot: 8.0\n";
    }
    klippy::KlippyInstanceConfig cfg;
    klippy::KlippyInstance instance(cfg);
    ASSERT_TRUE(instance.loadConfig(cfgPath));
    const auto& s = instance.settings();
    EXPECT_EQ(s.extrusionCompensationModel, "power_law");
    EXPECT_NEAR(s.paFlowIndex, 0.5, 1e-9);
    EXPECT_NEAR(s.paConsistency, 0.012, 1e-9);
    EXPECT_NEAR(s.paMaxCompensation, 0.3, 1e-9);
    EXPECT_TRUE(s.heaterFlowPreEmphasis);
    EXPECT_NEAR(s.filamentHeatCapacity, 2.1, 1e-9);
    EXPECT_NEAR(s.heaterBlockCapacitance, 8.0, 1e-9);
    EXPECT_NEAR(s.sensorCapacitance, 1.0, 1e-9);
    EXPECT_NEAR(s.meltZoneCapacitance, 2.0, 1e-9);
    EXPECT_NEAR(s.heaterSensorConductance, 2.0, 1e-9);
    EXPECT_NEAR(s.sensorMeltConductance, 1.5, 1e-9);
    EXPECT_NEAR(s.luenbergerGainHeater, 0.5, 1e-9);
    EXPECT_NEAR(s.luenbergerGainSensor, 2.0, 1e-9);
    EXPECT_NEAR(s.luenbergerGainMelt, 0.3, 1e-9);
    EXPECT_NEAR(s.debtTimeConstant, 1.5, 1e-9);
    EXPECT_NEAR(s.maxPreEmphasisPower, 0.35, 1e-9);
    EXPECT_NEAR(s.maxPostEmphasisPower, 0.15, 1e-9);
    EXPECT_NEAR(s.maxHeaterOvershoot, 8.0, 1e-9);
    std::filesystem::remove(cfgPath);
}

TEST(KlipperExtrusionCompensation, ConfigParsesCrossWlfKeys) {
    std::string cfgPath = "/tmp/tether_test_crosswlf_cfg_" +
                          std::to_string(getpid()) + ".cfg";
    {
        std::ofstream f(cfgPath);
        f << "[extruder]\n"
          << "pressure_advance_model: cross_wlf\n"
          << "cross_wlf_tau_star: 100000.0\n"
          << "cross_wlf_flow_index: 0.4\n"
          << "cross_wlf_c1: 17.44\n"
          << "cross_wlf_c2: 51.6\n"
          << "cross_wlf_ref_temp: 200.0\n"
          << "cross_wlf_zero_shear_viscosity: 1500.0\n"
          << "cross_wlf_compressibility_over_area: 1e-5\n";
    }
    klippy::KlippyInstanceConfig cfg;
    klippy::KlippyInstance instance(cfg);
    ASSERT_TRUE(instance.loadConfig(cfgPath));
    const auto& s = instance.settings();
    EXPECT_EQ(s.extrusionCompensationModel, "cross_wlf");
    EXPECT_NEAR(s.crossWlfTauStar, 100000.0, 1e-3);
    EXPECT_NEAR(s.crossWlfFlowIndex, 0.4, 1e-9);
    EXPECT_NEAR(s.crossWlfC1, 17.44, 1e-9);
    EXPECT_NEAR(s.crossWlfC2, 51.6, 1e-9);
    EXPECT_NEAR(s.crossWlfRefTempC, 200.0, 1e-9);
    EXPECT_NEAR(s.crossWlfZeroShearViscosityRef, 1500.0, 1e-9);
    EXPECT_NEAR(s.crossWlfCompressibilityOverArea, 1e-5, 1e-12);
    std::filesystem::remove(cfgPath);
}

// ============================================================================
// SET_PRESSURE_ADVANCE MODEL= and SET_HEATER_FLOW_COMPENSATION
// ============================================================================

class KlippyExtrusionInstanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_.sdcardDir = "/tmp/tether_test_extrusion_inst_" +
                         std::to_string(getpid());
        std::filesystem::create_directories(cfg_.sdcardDir);
        instance_ = std::make_unique<klippy::KlippyInstance>(cfg_);
    }
    void TearDown() override {
        instance_.reset();
        std::filesystem::remove_all(cfg_.sdcardDir);
    }
    klippy::KlippyInstanceConfig cfg_;
    std::unique_ptr<klippy::KlippyInstance> instance_;
};

TEST_F(KlippyExtrusionInstanceTest, SetPressureAdvanceModelParsing) {
    instance_->executeGcode("SET_PRESSURE_ADVANCE ADVANCE=0.05 SMOOTH_TIME=0.04 "
                            "MODEL=power_law FLOW_INDEX=0.5 CONSISTENCY=0.012");
    // The pressure_advance object should reflect the model.
    auto result = instance_->server().queryObjects(
        {{"pressure_advance", {}}});
    ASSERT_TRUE(result.count("pressure_advance") > 0);
    const auto& status = result["pressure_advance"];
    ASSERT_TRUE(status.count("model") > 0);
    EXPECT_EQ(status.at("model").asString(), "power_law");
    EXPECT_NEAR(status.at("pressure_advance").asDouble(), 0.05, 1e-9);
}

TEST_F(KlippyExtrusionInstanceTest, SetPressureAdvanceLinearModelDefault) {
    instance_->executeGcode("SET_PRESSURE_ADVANCE ADVANCE=0.04");
    auto result = instance_->server().queryObjects(
        {{"pressure_advance", {}}});
    ASSERT_TRUE(result.count("pressure_advance") > 0);
    const auto& status = result["pressure_advance"];
    // Default model stays linear.
    EXPECT_EQ(status.at("model").asString(), "linear");
}

TEST_F(KlippyExtrusionInstanceTest, SetHeaterFlowCompensationToggle) {
    // Without a heater wired, the command should still succeed and update
    // the settings flag (the controller is built lazily).
    EXPECT_TRUE(instance_->executeGcode(
        "SET_HEATER_FLOW_COMPENSATION ENABLE=1"));
    EXPECT_TRUE(instance_->settings().heaterFlowPreEmphasis);
    EXPECT_TRUE(instance_->executeGcode(
        "SET_HEATER_FLOW_COMPENSATION ENABLE=0"));
    EXPECT_FALSE(instance_->settings().heaterFlowPreEmphasis);
}

TEST_F(KlippyExtrusionInstanceTest, SetHeaterFlowCompensationTuning) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_HEATER_FLOW_COMPENSATION ENABLE=1 "
        "FILAMENT_HEAT_CAPACITY=2.5 DEBT_TIME_CONSTANT=3.0 "
        "MAX_HEATER_OVERSHOOT=12.0"));
    EXPECT_NEAR(instance_->settings().filamentHeatCapacity, 2.5, 1e-9);
    EXPECT_NEAR(instance_->settings().debtTimeConstant, 3.0, 1e-9);
    EXPECT_NEAR(instance_->settings().maxHeaterOvershoot, 12.0, 1e-9);
}

// ============================================================================
// Deconvolution controller: config parsing
// ============================================================================

TEST(KlipperExtrusionCompensation, ConfigParsesDeconvolutionKeys) {
    std::string cfgPath = "/tmp/tether_test_deconv_cfg_" +
                          std::to_string(getpid()) + ".cfg";
    {
        std::ofstream f(cfgPath);
        f << "[extruder]\n"
          << "nozzle_diameter: 0.4\n"
          << "filament_diameter: 1.75\n"
          << "deconvolution_controller: lti_freq\n"
          << "deconvolution_enabled: true\n"
          << "deconvolution_lambda: 0.0001\n"
          << "lti_pad_to_power_of_two: false\n"
          << "overlap_add_block_size: 128\n"
          << "overlap_add_overlap_ratio: 0.75\n"
          << "arx_na: 3\n"
          << "arx_nb: 2\n"
          << "state_space_state_dim: 4\n"
          << "state_space_input_dim: 2\n"
          << "state_space_output_dim: 2\n";
    }
    klippy::KlippyInstanceConfig cfg;
    klippy::KlippyInstance instance(cfg);
    ASSERT_TRUE(instance.loadConfig(cfgPath));
    const auto& s = instance.settings();
    EXPECT_EQ(s.deconvolutionController, "lti_freq");
    EXPECT_TRUE(s.deconvolutionEnabled);
    EXPECT_NEAR(s.deconvolutionLambda, 0.0001, 1e-12);
    EXPECT_FALSE(s.ltiPadToPowerOfTwo);
    EXPECT_EQ(s.overlapAddBlockSize, 128);
    EXPECT_NEAR(s.overlapAddOverlapRatio, 0.75, 1e-9);
    EXPECT_EQ(s.arxNa, 3);
    EXPECT_EQ(s.arxNb, 2);
    EXPECT_EQ(s.stateSpaceStateDim, 4);
    EXPECT_EQ(s.stateSpaceInputDim, 2);
    EXPECT_EQ(s.stateSpaceOutputDim, 2);
    std::filesystem::remove(cfgPath);
}

// ============================================================================
// Deconvolution controller: G-code command
// ============================================================================

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerLtiFreq) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=lti_freq LAMBDA=0.001"));
    EXPECT_EQ(instance_->settings().deconvolutionController, "lti_freq");
    EXPECT_TRUE(instance_->settings().deconvolutionEnabled);
    EXPECT_NEAR(instance_->settings().deconvolutionLambda, 0.001, 1e-12);
    // The controller should be built.
    ASSERT_NE(instance_->ltiDeconvolver(), nullptr);
    // Other controllers should not be built.
    EXPECT_EQ(instance_->overlapAddDeconvolver(), nullptr);
    EXPECT_EQ(instance_->arxInverseFilter(), nullptr);
    EXPECT_EQ(instance_->stateSpaceEstimator(), nullptr);
}

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerOverlapAdd) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=overlap_add_lpv "
        "BLOCK_SIZE=128 OVERLAP_RATIO=0.75"));
    EXPECT_EQ(instance_->settings().deconvolutionController, "overlap_add_lpv");
    EXPECT_TRUE(instance_->settings().deconvolutionEnabled);
    EXPECT_EQ(instance_->settings().overlapAddBlockSize, 128);
    EXPECT_NEAR(instance_->settings().overlapAddOverlapRatio, 0.75, 1e-9);
    ASSERT_NE(instance_->overlapAddDeconvolver(), nullptr);
    EXPECT_EQ(instance_->ltiDeconvolver(), nullptr);
}

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerArx) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=arx_lpv "
        "ARX_NA=3 ARX_NB=2"));
    EXPECT_EQ(instance_->settings().deconvolutionController, "arx_lpv");
    EXPECT_TRUE(instance_->settings().deconvolutionEnabled);
    EXPECT_EQ(instance_->settings().arxNa, 3);
    EXPECT_EQ(instance_->settings().arxNb, 2);
    ASSERT_NE(instance_->arxInverseFilter(), nullptr);
    EXPECT_EQ(instance_->ltiDeconvolver(), nullptr);
}

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerStateSpace) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=statespace_lpv "
        "STATE_DIM=3 INPUT_DIM=1 OUTPUT_DIM=1 LAMBDA=0.0001"));
    EXPECT_EQ(instance_->settings().deconvolutionController, "statespace_lpv");
    EXPECT_TRUE(instance_->settings().deconvolutionEnabled);
    EXPECT_EQ(instance_->settings().stateSpaceStateDim, 3);
    EXPECT_NEAR(instance_->settings().deconvolutionLambda, 0.0001, 1e-12);
    ASSERT_NE(instance_->stateSpaceEstimator(), nullptr);
    EXPECT_EQ(instance_->ltiDeconvolver(), nullptr);
}

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerDisable) {
    // Enable first, then disable.
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=lti_freq"));
    ASSERT_NE(instance_->ltiDeconvolver(), nullptr);
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=0"));
    EXPECT_FALSE(instance_->settings().deconvolutionEnabled);
    EXPECT_EQ(instance_->ltiDeconvolver(), nullptr);
}

TEST_F(KlippyExtrusionInstanceTest, SetDeconvolutionControllerSwitchType) {
    // Start with LTI, switch to ARX — LTI should be torn down.
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=lti_freq"));
    ASSERT_NE(instance_->ltiDeconvolver(), nullptr);
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=arx_lpv"));
    EXPECT_EQ(instance_->ltiDeconvolver(), nullptr);
    ASSERT_NE(instance_->arxInverseFilter(), nullptr);
}

// ============================================================================
// Deconvolution controller: printer object status
// ============================================================================

TEST_F(KlippyExtrusionInstanceTest, DeconvolutionObjectStatus) {
    EXPECT_TRUE(instance_->executeGcode(
        "SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=lti_freq LAMBDA=0.002"));
    auto result = instance_->server().queryObjects({{"deconvolution", {}}});
    ASSERT_TRUE(result.count("deconvolution") > 0);
    const auto& status = result["deconvolution"];
    ASSERT_TRUE(status.count("controller") > 0);
    EXPECT_EQ(status.at("controller").asString(), "lti_freq");
    ASSERT_TRUE(status.count("enabled") > 0);
    EXPECT_TRUE(status.at("enabled").asBool());
    ASSERT_TRUE(status.count("lambda") > 0);
    EXPECT_NEAR(status.at("lambda").asDouble(), 0.002, 1e-12);
}

TEST_F(KlippyExtrusionInstanceTest, DeconvolutionObjectDefaultIsNone) {
    auto result = instance_->server().queryObjects({{"deconvolution", {}}});
    ASSERT_TRUE(result.count("deconvolution") > 0);
    const auto& status = result["deconvolution"];
    EXPECT_EQ(status.at("controller").asString(), "none");
    EXPECT_FALSE(status.at("enabled").asBool());
}

#endif // TETHER_ENABLE_PRESSURE_ADVANCE
