/**
 * @file test_QFT_coverage.cpp
 * @brief Extended coverage tests for QFT.cpp — hit uncovered branches in
 *        NicholsPoint, PlantTemplate::contains, QFTBound::isSatisfied,
 *        TransferFunction edge cases, designPrefilter orders, mCircle/nCircle
 *        non-zero M/N, isUnstableRegion combos, convexHull edge cases,
 *        interpolateTemplate clamping, computeImpl, resetImpl, analyzeMargins.
 */

#include "tether/control/autotuning/QFT.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <complex>

using namespace Control;

// ============================================================================
// NicholsPoint — near-zero magnitude
// ============================================================================

TEST(QFTCovTest, NicholsPoint_FromComplex_NearZero) {
    // Near-zero magnitude — triggers 1e-15 guard
    auto pt = NicholsPoint::fromComplex({1e-20, 1e-20});
    // Should not crash; gain is very negative
    EXPECT_LT(pt.gain, -200.0);
}

TEST(QFTCovTest, NicholsPoint_FromComplex_PureImaginary) {
    auto pt = NicholsPoint::fromComplex({0.0, 1.0});
    EXPECT_NEAR(pt.gain, 0.0, 0.01);
    EXPECT_NEAR(pt.phase, 90.0, 0.01);
}

TEST(QFTCovTest, NicholsPoint_FromComplex_Negative) {
    auto pt = NicholsPoint::fromComplex({-1.0, 0.0});
    EXPECT_NEAR(pt.gain, 0.0, 0.01);
    EXPECT_NEAR(std::abs(pt.phase), 180.0, 0.01);
}

// ============================================================================
// PlantTemplate — boundary + contains
// ============================================================================

TEST(QFTCovTest, PlantTemplate_BoundaryEmpty) {
    PlantTemplate pt;
    auto b = pt.boundary();
    EXPECT_TRUE(b.empty());
}

TEST(QFTCovTest, PlantTemplate_ContainsEmpty) {
    PlantTemplate pt;
    EXPECT_FALSE(pt.contains(NicholsPoint(0.0, 0.0)));
}

TEST(QFTCovTest, PlantTemplate_ContainsTwoPoints) {
    PlantTemplate pt;
    pt.points.push_back(NicholsPoint(0.0, 0.0));
    pt.points.push_back(NicholsPoint(6.0, -90.0));
    // Hull <3 → should return false
    EXPECT_FALSE(pt.contains(NicholsPoint(3.0, -45.0)));
}

TEST(QFTCovTest, PlantTemplate_ContainsInside) {
    PlantTemplate pt;
    // Create a triangle in gain-phase space
    pt.points.push_back(NicholsPoint(0.0, 0.0));
    pt.points.push_back(NicholsPoint(10.0, 0.0));
    pt.points.push_back(NicholsPoint(5.0, 10.0));
    // Center point should be contained
    EXPECT_TRUE(pt.contains(NicholsPoint(5.0, 3.0)));
}

TEST(QFTCovTest, PlantTemplate_ContainsOutside) {
    PlantTemplate pt;
    pt.points.push_back(NicholsPoint(0.0, 0.0));
    pt.points.push_back(NicholsPoint(10.0, 0.0));
    pt.points.push_back(NicholsPoint(5.0, 10.0));
    EXPECT_FALSE(pt.contains(NicholsPoint(-20.0, -90.0)));
}

TEST(QFTCovTest, PlantTemplate_ContainsOnEdge) {
    PlantTemplate pt;
    pt.points.push_back(NicholsPoint(0.0, 0.0));
    pt.points.push_back(NicholsPoint(10.0, 0.0));
    pt.points.push_back(NicholsPoint(5.0, 10.0));
    pt.points.push_back(NicholsPoint(0.0, 10.0));
    // Point on edge
    NicholsPoint edge(5.0, 0.0);
    pt.contains(edge); // Just exercise
}

// ============================================================================
// QFTBound::isSatisfied — ray-cast logic  
// ============================================================================

TEST(QFTCovTest, QFTBound_IsSatisfied_EmptyBoundary) {
    QFTBound bound;
    bound.boundary.clear();
    // Empty boundary → satisfied (true)
    EXPECT_TRUE(bound.isSatisfied(NicholsPoint(0.0, -90.0)));
}

TEST(QFTCovTest, QFTBound_IsSatisfied_InsideBound) {
    QFTBound bound;
    // Create square boundary in gain-phase space
    bound.boundary.push_back(NicholsPoint(-20.0, -200.0));
    bound.boundary.push_back(NicholsPoint(20.0, -200.0));
    bound.boundary.push_back(NicholsPoint(20.0, -100.0));
    bound.boundary.push_back(NicholsPoint(-20.0, -100.0));
    // Point inside
    auto result = bound.isSatisfied(NicholsPoint(0.0, -150.0));
    // Exercise crossing logic
    (void)result;
}

TEST(QFTCovTest, QFTBound_IsSatisfied_OutsideBound) {
    QFTBound bound;
    bound.boundary.push_back(NicholsPoint(-20.0, -200.0));
    bound.boundary.push_back(NicholsPoint(20.0, -200.0));
    bound.boundary.push_back(NicholsPoint(20.0, -100.0));
    bound.boundary.push_back(NicholsPoint(-20.0, -100.0));
    auto result = bound.isSatisfied(NicholsPoint(0.0, 0.0));
    (void)result;
}

TEST(QFTCovTest, QFTBound_AllTypes) {
    for (auto type : {QFTBound::Type::Tracking, QFTBound::Type::Stability,
                      QFTBound::Type::Disturbance, QFTBound::Type::ControlEffort,
                      QFTBound::Type::Combined}) {
        QFTBound b;
        b.type = type;
        b.boundary.push_back(NicholsPoint(0.0, 0.0));
        b.isSatisfied(NicholsPoint(1.0, -45.0));
    }
}

// ============================================================================
// TransferFunction — edge cases
// ============================================================================

TEST(QFTCovTest, TransferFunction_EvaluateEmpty) {
    TransferFunction tf;
    // Empty num/den
    auto r = tf.evaluate(1.0);
    (void)r;
}

TEST(QFTCovTest, TransferFunction_EvaluateDenNearZero) {
    TransferFunction tf;
    // Numerator = [1], Denominator = [0] (near-zero den)
    tf.num = {1.0};
    tf.den = {0.0};
    auto r = tf.evaluate(0.0);
    // Should return large value (1e15 guard)
    EXPECT_GT(std::abs(r), 1e10);
}

TEST(QFTCovTest, TransferFunction_EvaluateNormal) {
    TransferFunction tf;
    tf.num = {1.0};
    tf.den = {1.0, 1.0}; // 1/(s+1)
    auto r = tf.evaluate(1.0);
    EXPECT_GT(std::abs(r), 0.0);
}

TEST(QFTCovTest, TransferFunction_EvaluateHighOrder) {
    TransferFunction tf;
    tf.num = {1.0, 2.0, 1.0};     // s^2+2s+1
    tf.den = {1.0, 3.0, 3.0, 1.0}; // s^3+3s^2+3s+1
    auto r = tf.evaluate(10.0);
    EXPECT_GT(std::abs(r), 0.0);
}

TEST(QFTCovTest, TransferFunction_FromZPK_NoZeros) {
    std::vector<std::complex<double>> zeros;
    std::vector<std::complex<double>> poles = {{-1.0, 0.0}};
    auto tf = TransferFunction::fromZPK(zeros, poles, 2.0);
    EXPECT_EQ(tf.order(), 1);
}

TEST(QFTCovTest, TransferFunction_FromZPK_MultiZerosMultiPoles) {
    std::vector<std::complex<double>> zeros = {{-1.0, 0.0}, {-2.0, 0.0}};
    std::vector<std::complex<double>> poles = {{-3.0, 0.0}, {-4.0, 0.0}, {-5.0, 0.0}};
    auto tf = TransferFunction::fromZPK(zeros, poles, 1.0);
    EXPECT_EQ(tf.order(), 3);
}

TEST(QFTCovTest, TransferFunction_FromZPK_ComplexPair) {
    std::vector<std::complex<double>> zeros;
    std::vector<std::complex<double>> poles = {{-1.0, 1.0}, {-1.0, -1.0}};
    auto tf = TransferFunction::fromZPK(zeros, poles, 1.0);
    EXPECT_EQ(tf.order(), 2);
}

TEST(QFTCovTest, TransferFunction_OrderZero) {
    TransferFunction tf;
    tf.num = {5.0};
    tf.den = {1.0};
    EXPECT_EQ(tf.order(), 0);
}

// ============================================================================
// QFTController — plant setup and templates
// ============================================================================

TEST(QFTCovTest, Controller_SetNominalPlant_FOPDT_VariousL) {
    QFTController ctrl;
    // L = 0 (no delay)
    ctrl.setNominalPlant(1.0, 1.0, 0.0);
    // L > 0 (Padé approximation)
    ctrl.setNominalPlant(1.0, 1.0, 0.5);
    // Large L
    ctrl.setNominalPlant(2.0, 0.5, 2.0);
}

TEST(QFTCovTest, Controller_SetNominalPlant_SOPDT) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.5, 0.1);
}

TEST(QFTCovTest, Controller_SetNominalPlant_StateSpace) {
    QFTController ctrl;
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    ctrl.setNominalPlant(A, B, C, D, 1, 1, 1);
}

TEST(QFTCovTest, Controller_AutoSelectFrequencies_WithPlant) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.autoSelectFrequencies(10);
}

TEST(QFTCovTest, Controller_AutoSelectFrequencies_NoPlant) {
    QFTController ctrl;
    // No plant set → tau ≤ 0 branch
    ctrl.autoSelectFrequencies(10);
}

TEST(QFTCovTest, Controller_SetPlantSamples) {
    QFTController ctrl;
    TransferFunction tf1, tf2;
    tf1.num = {1.0}; tf1.den = {1.0, 1.0};
    tf2.num = {2.0}; tf2.den = {1.0, 2.0};
    ctrl.setPlantSamples({tf1, tf2});
}

// ============================================================================
// Tracking/Disturbance specs
// ============================================================================

TEST(QFTCovTest, Controller_TrackingSpec_Bandwidth) {
    QFTController ctrl;
    ctrl.setTrackingSpec(10.0, {0.3, 0.9});
}

TEST(QFTCovTest, Controller_DisturbanceSpec_Simple) {
    QFTController ctrl;
    // Tests omega < crossover (flat) and omega >= crossover (rolloff)
    ctrl.setDisturbanceSpec(0.5, 5.0);
}

TEST(QFTCovTest, Controller_DisturbanceSpec_Detailed) {
    QFTController ctrl;
    ctrl.setDisturbanceSpec({0.1, 1.0, 10.0}, {1.0, 0.5, 0.1});
}

TEST(QFTCovTest, Controller_SetControlEffortSpec) {
    QFTController ctrl;
    TransferFunction bound;
    bound.num = {1.0};
    bound.den = {1.0, 1.0};
    ctrl.setControlEffortSpec(bound);
}

TEST(QFTCovTest, Controller_SetMultiplicativeUncertainty) {
    QFTController ctrl;
    TransferFunction w;
    w.num = {0.5};
    w.den = {1.0, 1.0};
    ctrl.setMultiplicativeUncertainty(w);
}

TEST(QFTCovTest, Controller_AddParametricUncertainty) {
    QFTController ctrl;
    ctrl.addParametricUncertainty(0, 0.5, 2.0, 5);
}

// ============================================================================
// Compute templates/bounds — branch coverage
// ============================================================================

TEST(QFTCovTest, Controller_ComputeTemplates_AutoFreqs) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    // No design frequencies set → auto branch
    ctrl.computeTemplates();
    EXPECT_FALSE(ctrl.getTemplates().empty());
}

TEST(QFTCovTest, Controller_ComputeTemplates_ManualFreqs) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setDesignFrequencies({0.1, 1.0, 10.0});
    ctrl.computeTemplates();
    EXPECT_FALSE(ctrl.getTemplates().empty());
}

TEST(QFTCovTest, Controller_ComputeBounds_WithSpecs) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setTrackingSpec(10.0);
    ctrl.setDisturbanceSpec(0.5, 5.0);
    ctrl.setStabilityMargins(6.0, 45.0);
    ctrl.setDesignFrequencies({0.1, 1.0, 10.0});
    ctrl.computeTemplates();
    ctrl.computeBounds();
    EXPECT_FALSE(ctrl.getBounds().empty());
}

// ============================================================================
// Controller pole/zero additions — complex branches
// ============================================================================

TEST(QFTCovTest, Controller_AddComplexPole_RealPart) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    // Complex pole with near-zero imaginary → real pole branch
    ctrl.addControllerPole(std::complex<double>(-1.0, 0.0));
}

TEST(QFTCovTest, Controller_AddComplexPole_ConjPair) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    // Complex with non-zero imaginary → conjugate pair branch
    ctrl.addControllerPole(std::complex<double>(-1.0, 1.0));
}

TEST(QFTCovTest, Controller_AddComplexZero_RealPart) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.addControllerZero(std::complex<double>(-2.0, 0.0));
}

TEST(QFTCovTest, Controller_AddComplexZero_ConjPair) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.addControllerZero(std::complex<double>(-2.0, 3.0));
}

TEST(QFTCovTest, Controller_SetControllerGain_EmptyNum) {
    QFTController ctrl;
    // Set empty controller first
    ctrl.setController(TransferFunction{{}, {}});
    ctrl.setControllerGain(5.0);
    // Gain applied to empty num → push back 5.0
}

TEST(QFTCovTest, Controller_SetControllerGain_NonEmpty) {
    QFTController ctrl;
    ctrl.setController(TransferFunction{{1.0, 2.0}, {1.0}});
    ctrl.setControllerGain(3.0);
    auto& c = ctrl.getController();
    EXPECT_NEAR(c.num[0], 3.0, 0.01);
}

// ============================================================================
// checkBounds / getBoundSatisfaction
// ============================================================================

TEST(QFTCovTest, Controller_CheckBounds_NoBounds) {
    QFTController ctrl;
    // No bounds → trivially satisfied
    EXPECT_TRUE(ctrl.checkBounds());
}

TEST(QFTCovTest, Controller_CheckBounds_WithSetup) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setStabilityMargins(6.0, 45.0);
    ctrl.setDesignFrequencies({1.0});
    ctrl.computeTemplates();
    ctrl.computeBounds();
    ctrl.setController(TransferFunction{{10.0, 5.0}, {1.0, 1.0}});
    auto satisfied = ctrl.checkBounds();
    (void)satisfied;
}

TEST(QFTCovTest, Controller_GetBoundSatisfaction_Empty) {
    QFTController ctrl;
    auto sat = ctrl.getBoundSatisfaction();
    EXPECT_TRUE(sat.empty());
}

TEST(QFTCovTest, Controller_GetBoundSatisfaction_WithBounds) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setStabilityMargins(6.0, 45.0);
    ctrl.setDesignFrequencies({0.1, 1.0, 10.0});
    ctrl.computeTemplates();
    ctrl.computeBounds();
    ctrl.setController(TransferFunction{{10.0}, {1.0, 1.0}});
    auto sat = ctrl.getBoundSatisfaction();
    // There should be some bound satisfaction values
    (void)sat;
}

// ============================================================================
// designPrefilter — all order branches
// ============================================================================

TEST(QFTCovTest, Controller_DesignPrefilter_Auto_NoTracking) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    // No tracking spec set → default BW branch
    ctrl.designPrefilter();
}

TEST(QFTCovTest, Controller_DesignPrefilter_Auto_WithTracking) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.setTrackingSpec(5.0);
    ctrl.designPrefilter();
}

TEST(QFTCovTest, Controller_DesignPrefilter_Order1) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.designPrefilter(10.0, 1);
    auto& f = ctrl.getPrefilter();
    EXPECT_FALSE(f.num.empty());
}

TEST(QFTCovTest, Controller_DesignPrefilter_Order2) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.designPrefilter(10.0, 2);
    auto& f = ctrl.getPrefilter();
    EXPECT_FALSE(f.num.empty());
}

TEST(QFTCovTest, Controller_DesignPrefilter_Order3) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    // Order > 2 → cascaded sections loop
    ctrl.designPrefilter(10.0, 3);
    auto& f = ctrl.getPrefilter();
    EXPECT_FALSE(f.num.empty());
}

TEST(QFTCovTest, Controller_DesignPrefilter_Order4) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{1.0}, {1.0}});
    ctrl.designPrefilter(10.0, 4);
}

// ============================================================================
// Evaluate functions
// ============================================================================

TEST(QFTCovTest, Controller_EvaluateLoop) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{10.0, 5.0}, {1.0, 1.0}});
    auto r = ctrl.evaluateLoop(1.0);
    EXPECT_GT(std::abs(r), 0.0);
}

TEST(QFTCovTest, Controller_EvaluateClosedLoop) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{10.0, 5.0}, {1.0, 1.0}});
    auto r = ctrl.evaluateClosedLoop(1.0);
    EXPECT_GT(std::abs(r), 0.0);
}

TEST(QFTCovTest, Controller_EvaluateSensitivity) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{10.0, 5.0}, {1.0, 1.0}});
    auto r = ctrl.evaluateSensitivity(1.0);
    EXPECT_GT(std::abs(r), 0.0);
}

// ============================================================================
// analyzeMargins — phase/gain crossover branches
// ============================================================================

TEST(QFTCovTest, Controller_AnalyzeMargins_HighGain) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // High-gain controller → should find crossovers
    ctrl.setController(TransferFunction{{100.0, 50.0, 10.0}, {1.0, 1.0, 1.0}});
    auto margins = ctrl.analyzeMargins();
    // Exercise phase/gain crossover detection branches
    (void)margins;
}

TEST(QFTCovTest, Controller_AnalyzeMargins_LowGain) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // Very low gain → no crossover → defaults to 0
    ctrl.setController(TransferFunction{{0.001}, {1.0, 1.0}});
    auto margins = ctrl.analyzeMargins();
    (void)margins;
}

TEST(QFTCovTest, Controller_AnalyzeMargins_Integrator) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // Controller with integrator-like behavior
    ctrl.setController(TransferFunction{{10.0, 1.0}, {1.0, 0.0}});
    auto margins = ctrl.analyzeMargins();
    (void)margins;
}

// ============================================================================
// worstCaseResponse
// ============================================================================

TEST(QFTCovTest, Controller_WorstCase_Empty) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // No plant samples → empty return
    auto [gains, phases] = ctrl.worstCaseResponse(1.0);
    (void)gains;
    (void)phases;
}

TEST(QFTCovTest, Controller_WorstCase_WithSamples) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    TransferFunction tf1, tf2;
    tf1.num = {1.0}; tf1.den = {1.0, 1.0};
    tf2.num = {2.0}; tf2.den = {1.0, 0.5};
    ctrl.setPlantSamples({tf1, tf2});
    ctrl.setController(TransferFunction{{5.0}, {1.0}});
    auto [gains, phases] = ctrl.worstCaseResponse(1.0);
    EXPECT_FALSE(gains.empty());
}

// ============================================================================
// autoShapeLoop — trigger auto template/bound computation
// ============================================================================

TEST(QFTCovTest, Controller_AutoShapeLoop_EmptyTemplates) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setStabilityMargins(6.0, 45.0);
    // No templates/bounds → should auto-compute
    auto result = ctrl.autoShapeLoop();
    (void)result;
}

TEST(QFTCovTest, Controller_AutoShapeLoop_WithConfig) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setFOPDTUncertainty(0.5, 2.0, 0.5, 2.0, 0.0, 0.2);
    ctrl.setStabilityMargins(6.0, 45.0);
    QFTController::LoopShapingConfig config;
    config.maxPoles = 4;
    config.maxZeros = 3;
    config.optimizationIterations = 50; // Reduce for speed
    auto result = ctrl.autoShapeLoop(config);
    (void)result;
}

// ============================================================================
// NicholsChart — mCircle / nCircle non-zero branches
// ============================================================================

TEST(QFTCovTest, NicholsChart_MCircle_0dB) {
    auto pts = NicholsChart::mCircle(0.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_MCircle_Positive) {
    auto pts = NicholsChart::mCircle(3.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_MCircle_Negative) {
    auto pts = NicholsChart::mCircle(-3.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_MCircle_Large) {
    auto pts = NicholsChart::mCircle(20.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_NCircle_0) {
    // N≈0 → horizontal line branch
    auto pts = NicholsChart::nCircle(0.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_NCircle_180) {
    // N≈180 → horizontal line branch
    auto pts = NicholsChart::nCircle(180.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_NCircle_90) {
    auto pts = NicholsChart::nCircle(90.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_NCircle_Neg90) {
    auto pts = NicholsChart::nCircle(-90.0);
    // Already tested in existing tests but exercise again for coverage
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_NCircle_45) {
    // Non-special N → parametric circle
    auto pts = NicholsChart::nCircle(45.0);
    EXPECT_FALSE(pts.empty());
}

TEST(QFTCovTest, NicholsChart_ClosedLoopMagnitude) {
    auto clm = NicholsChart::closedLoopMagnitude(NicholsPoint(6.0, -90.0));
    // Result is in dB, can be negative
    EXPECT_TRUE(std::isfinite(clm));
}

TEST(QFTCovTest, NicholsChart_ClosedLoopPhase) {
    auto clp = NicholsChart::closedLoopPhase(NicholsPoint(6.0, -90.0));
    (void)clp;
}

// ============================================================================
// isUnstableRegion — all combos
// ============================================================================

TEST(QFTCovTest, NicholsChart_IsUnstable_TrueCase) {
    // gain>0, phase in (-180, 0) → unstable
    EXPECT_TRUE(NicholsChart::isUnstableRegion(NicholsPoint(6.0, -90.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_NegativeGain) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(-6.0, -90.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_PosPhase) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(6.0, 90.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_PhaseBelow180) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(6.0, -200.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_ZeroGain) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(0.0, -90.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_PhaseExactly0) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(6.0, 0.0)));
}

TEST(QFTCovTest, NicholsChart_IsUnstable_PhaseExactly180) {
    EXPECT_FALSE(NicholsChart::isUnstableRegion(NicholsPoint(6.0, -180.0)));
}

// ============================================================================
// TemplateUtils — convexHull edge cases
// ============================================================================

TEST(QFTCovTest, ConvexHull_Empty) {
    auto hull = TemplateUtils::convexHull({});
    EXPECT_TRUE(hull.empty());
}

TEST(QFTCovTest, ConvexHull_OnePoint) {
    auto hull = TemplateUtils::convexHull({NicholsPoint(0.0, 0.0)});
    EXPECT_EQ(hull.size(), 1u);
}

TEST(QFTCovTest, ConvexHull_TwoPoints) {
    auto hull = TemplateUtils::convexHull({NicholsPoint(0.0, 0.0), NicholsPoint(1.0, 1.0)});
    EXPECT_EQ(hull.size(), 2u);
}

TEST(QFTCovTest, ConvexHull_Collinear) {
    // Collinear points → cross == 0 branch
    auto hull = TemplateUtils::convexHull({
        NicholsPoint(0.0, 0.0),
        NicholsPoint(1.0, 1.0),
        NicholsPoint(2.0, 2.0),
        NicholsPoint(3.0, 3.0)
    });
    EXPECT_GE(hull.size(), 2u);
}

TEST(QFTCovTest, ConvexHull_Square) {
    auto hull = TemplateUtils::convexHull({
        NicholsPoint(0.0, 0.0),
        NicholsPoint(10.0, 0.0),
        NicholsPoint(10.0, 10.0),
        NicholsPoint(0.0, 10.0),
        NicholsPoint(5.0, 5.0)  // Interior point
    });
    EXPECT_LE(hull.size(), 4u);
}

// ============================================================================
// TemplateUtils — interpolateTemplate
// ============================================================================

TEST(QFTCovTest, InterpolateTemplate_Normal) {
    PlantTemplate t1, t2;
    t1.frequency = 1.0;
    t1.points = {NicholsPoint(0.0, 0.0), NicholsPoint(5.0, -45.0)};
    t2.frequency = 10.0;
    t2.points = {NicholsPoint(-10.0, -90.0), NicholsPoint(0.0, -135.0)};
    auto result = TemplateUtils::interpolateTemplate(t1, t2, 5.0);
    EXPECT_EQ(result.points.size(), 2u);
}

TEST(QFTCovTest, InterpolateTemplate_AlphaClampLow) {
    PlantTemplate t1, t2;
    t1.frequency = 1.0;
    t1.points = {NicholsPoint(0.0, 0.0)};
    t2.frequency = 10.0;
    t2.points = {NicholsPoint(-10.0, -90.0)};
    // omega below t1 → alpha clamped to 0
    auto result = TemplateUtils::interpolateTemplate(t1, t2, 0.01);
    EXPECT_EQ(result.points.size(), 1u);
}

TEST(QFTCovTest, InterpolateTemplate_AlphaClampHigh) {
    PlantTemplate t1, t2;
    t1.frequency = 1.0;
    t1.points = {NicholsPoint(0.0, 0.0)};
    t2.frequency = 10.0;
    t2.points = {NicholsPoint(-10.0, -90.0)};
    // omega above t2 → alpha clamped to 1
    auto result = TemplateUtils::interpolateTemplate(t1, t2, 100.0);
    EXPECT_EQ(result.points.size(), 1u);
}

TEST(QFTCovTest, InterpolateTemplate_MismatchedSizes) {
    PlantTemplate t1, t2;
    t1.frequency = 1.0;
    t1.points = {NicholsPoint(0.0, 0.0), NicholsPoint(5.0, -45.0)};
    t2.frequency = 10.0;
    t2.points = {NicholsPoint(-10.0, -90.0)};
    auto result = TemplateUtils::interpolateTemplate(t1, t2, 5.0);
    (void)result;
}

// ============================================================================
// TemplateUtils — shiftTemplate
// ============================================================================

TEST(QFTCovTest, ShiftTemplate) {
    PlantTemplate pt;
    pt.frequency = 1.0;
    pt.points = {NicholsPoint(0.0, 0.0), NicholsPoint(5.0, -45.0)};
    pt.nominal = NicholsPoint(2.0, -20.0);
    auto shifted = TemplateUtils::shiftTemplate(pt, NicholsPoint(3.0, -10.0));
    EXPECT_EQ(shifted.points.size(), 2u);
}

// ============================================================================
// QFTLoopShaper — optimize
// ============================================================================

TEST(QFTCovTest, LoopShaper_Optimize_Short) {
    TransferFunction plant{{1.0}, {1.0, 1.0}};
    std::vector<QFTBound> bounds;
    QFTBound b;
    b.frequency = 1.0;
    b.boundary = {NicholsPoint(-10.0, -200.0), NicholsPoint(20.0, -200.0),
                  NicholsPoint(20.0, -100.0), NicholsPoint(-10.0, -100.0)};
    bounds.push_back(b);
    
    QFTLoopShaper::Config cfg;
    cfg.populationSize = 10;
    cfg.generations = 5;
    auto result = QFTLoopShaper::optimize(plant, bounds, {1.0}, cfg);
    EXPECT_FALSE(result.num.empty());
}

TEST(QFTCovTest, LoopShaper_Optimize_Default) {
    TransferFunction plant{{1.0}, {1.0, 1.0}};
    std::vector<QFTBound> bounds;
    auto result = QFTLoopShaper::optimize(plant, bounds, {1.0});
    EXPECT_FALSE(result.num.empty());
}

TEST(QFTCovTest, LoopShaper_ConfigDefaults) {
    auto cfg = QFTLoopShaper::Config::defaultConfig();
    EXPECT_GT(cfg.populationSize, 0);
    EXPECT_GT(cfg.generations, 0);
    EXPECT_GT(cfg.gainMax, cfg.gainMin);
}

// ============================================================================
// computeImpl / resetImpl (via ControllerBase::compute and reset)
// ============================================================================

TEST(QFTCovTest, Controller_ComputeImpl_NoPrefilter) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // Simple proportional controller
    ctrl.setController(TransferFunction{{10.0}, {1.0}});
    
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.5;
    input.dt = 0.001;
    
    auto output = ctrl.compute(input);
    (void)output;
}

TEST(QFTCovTest, Controller_ComputeImpl_WithPrefilter) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{10.0, 5.0}, {1.0, 1.0}});
    ctrl.designPrefilter(10.0, 2);
    
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.5;
    input.dt = 0.001;
    
    auto output = ctrl.compute(input);
    (void)output;
}

TEST(QFTCovTest, Controller_ComputeImpl_WithKd) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // Controller with num.size() > 1 → Kd branch
    ctrl.setController(TransferFunction{{1.0, 2.0, 3.0}, {1.0, 1.0}});
    
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    auto out1 = ctrl.compute(input);
    input.measured = 0.5;
    auto out2 = ctrl.compute(input);
    (void)out1;
    (void)out2;
}

TEST(QFTCovTest, Controller_ComputeImpl_WithKi) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    // Controller with den.size() > 1 → Ki branch
    ctrl.setController(TransferFunction{{5.0}, {1.0, 0.1}});
    
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    auto out1 = ctrl.compute(input);
    auto out2 = ctrl.compute(input);
    auto out3 = ctrl.compute(input);
    // Integral should accumulate
    (void)out1;
    (void)out2;
    (void)out3;
}

TEST(QFTCovTest, Controller_ResetImpl) {
    QFTController ctrl;
    ctrl.setNominalPlant(1.0, 1.0, 0.1);
    ctrl.setController(TransferFunction{{10.0}, {1.0}});
    
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.0;
    input.dt = 0.01;
    ctrl.compute(input);
    
    ctrl.reset();
    // After reset, internal state should be cleared
}
