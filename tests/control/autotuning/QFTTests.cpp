/**
 * @file QFTTests.cpp
 * @brief Unit tests for QFT (Quantitative Feedback Theory) components
 */

#include <gtest/gtest.h>
#include "tether/control/autotuning/QFT.hpp"
#include <complex>
#include <vector>
#include <cmath>

using namespace tether::control;

// ============================================================================
// NicholsPoint Tests
// ============================================================================

TEST(NicholsPointTest, DefaultConstruction) {
    NicholsPoint point;
    EXPECT_DOUBLE_EQ(point.gain, 0.0);
    EXPECT_DOUBLE_EQ(point.phase, 0.0);
}

TEST(NicholsPointTest, ParameterizedConstruction) {
    NicholsPoint point(10.0, -90.0);
    EXPECT_DOUBLE_EQ(point.gain, 10.0);
    EXPECT_DOUBLE_EQ(point.phase, -90.0);
}

TEST(NicholsPointTest, FromComplex) {
    std::complex<double> z(0.0, 1.0);  // j
    NicholsPoint point = NicholsPoint::fromComplex(z);
    EXPECT_NEAR(point.gain, 0.0, 1e-9);  // |j| = 1, 20*log10(1) = 0 dB
    EXPECT_NEAR(point.phase, 90.0, 0.01);  // arg(j) = 90 degrees
}

TEST(NicholsPointTest, ToComplex) {
    NicholsPoint point(0.0, 90.0);  // 0 dB, 90 degrees
    std::complex<double> z = point.toComplex();
    EXPECT_NEAR(std::abs(z), 1.0, 0.01);
    EXPECT_NEAR(std::arg(z) * 180.0 / M_PI, 90.0, 0.01);
}

// ============================================================================
// PlantTemplate Tests
// ============================================================================

TEST(PlantTemplateTest, DefaultConstruction) {
    PlantTemplate templ;
    EXPECT_DOUBLE_EQ(templ.frequency, 0.0);
    EXPECT_TRUE(templ.points.empty());
}

TEST(PlantTemplateTest, AddPoints) {
    PlantTemplate templ;
    templ.frequency = 1.0;
    templ.points.push_back(NicholsPoint(0.0, -45.0));
    templ.points.push_back(NicholsPoint(3.0, -90.0));
    templ.points.push_back(NicholsPoint(-3.0, -135.0));
    
    EXPECT_EQ(templ.points.size(), 3);
}

TEST(PlantTemplateTest, Boundary) {
    PlantTemplate templ;
    templ.points.push_back(NicholsPoint(0.0, -45.0));
    templ.points.push_back(NicholsPoint(3.0, -90.0));
    templ.points.push_back(NicholsPoint(-3.0, -135.0));
    
    auto boundary = templ.boundary();
    // Boundary should be computed (convex hull)
}

TEST(PlantTemplateTest, Contains) {
    PlantTemplate templ;
    templ.points.push_back(NicholsPoint(0.0, -45.0));
    templ.points.push_back(NicholsPoint(3.0, -90.0));
    templ.points.push_back(NicholsPoint(-3.0, -135.0));
    
    // Test containment check
    NicholsPoint testPoint(0.0, -90.0);
    // bool inside = templ.contains(testPoint);
}

// ============================================================================
// QFTBound Tests
// ============================================================================

TEST(QFTBoundTest, DefaultConstruction) {
    QFTBound bound;
    EXPECT_DOUBLE_EQ(bound.frequency, 0.0);
    EXPECT_EQ(bound.type, QFTBound::Combined);
}

TEST(QFTBoundTest, TypeEnumeration) {
    EXPECT_EQ(static_cast<int>(QFTBound::Tracking), 0);
    EXPECT_NE(QFTBound::Tracking, QFTBound::Stability);
    EXPECT_NE(QFTBound::Disturbance, QFTBound::ControlEffort);
}

TEST(QFTBoundTest, IsSatisfied) {
    QFTBound bound;
    bound.boundary.push_back(NicholsPoint(10.0, -90.0));
    bound.boundary.push_back(NicholsPoint(10.0, -180.0));
    bound.boundary.push_back(NicholsPoint(-10.0, -180.0));
    bound.boundary.push_back(NicholsPoint(-10.0, -90.0));
    
    NicholsPoint loopPoint(0.0, -135.0);
    bool satisfied = bound.isSatisfied(loopPoint);
}

// ============================================================================
// TrackingSpec Tests
// ============================================================================

TEST(TrackingSpecTest, DefaultConstruction) {
    TrackingSpec spec;
    EXPECT_TRUE(spec.frequencies.empty());
    EXPECT_TRUE(spec.lowerBound.empty());
    EXPECT_TRUE(spec.upperBound.empty());
}

TEST(TrackingSpecTest, SetBounds) {
    TrackingSpec spec;
    spec.frequencies = {0.1, 1.0, 10.0};
    spec.lowerBound = {-3.0, -3.0, -6.0};
    spec.upperBound = {1.0, 1.0, -1.0};
    
    EXPECT_EQ(spec.frequencies.size(), 3);
    EXPECT_EQ(spec.lowerBound.size(), 3);
    EXPECT_EQ(spec.upperBound.size(), 3);
}

// ============================================================================
// DisturbanceSpec Tests
// ============================================================================

TEST(DisturbanceSpecTest, DefaultConstruction) {
    DisturbanceSpec spec;
    EXPECT_TRUE(spec.frequencies.empty());
    EXPECT_TRUE(spec.maxSensitivity.empty());
}

TEST(DisturbanceSpecTest, SetSpec) {
    DisturbanceSpec spec;
    spec.frequencies = {0.1, 1.0, 10.0, 100.0};
    spec.maxSensitivity = {-40.0, -20.0, 0.0, 6.0};
    
    EXPECT_EQ(spec.frequencies.size(), 4);
    EXPECT_EQ(spec.maxSensitivity.size(), 4);
}

// ============================================================================
// TransferFunction Tests
// ============================================================================

TEST(TransferFunctionTest, DefaultConstruction) {
    TransferFunction tf;
    EXPECT_TRUE(tf.num.empty());
    EXPECT_TRUE(tf.den.empty());
}

TEST(TransferFunctionTest, FirstOrderSystem) {
    TransferFunction tf;
    tf.num = {1.0};        // 1
    tf.den = {1.0, 1.0};   // s + 1
    
    EXPECT_EQ(tf.order(), 1);
}

TEST(TransferFunctionTest, SecondOrderSystem) {
    TransferFunction tf;
    tf.num = {1.0};              // 1
    tf.den = {1.0, 2.0, 1.0};    // s^2 + 2s + 1
    
    EXPECT_EQ(tf.order(), 2);
}

TEST(TransferFunctionTest, Evaluate) {
    TransferFunction tf;
    tf.num = {1.0};        // 1
    tf.den = {1.0, 1.0};   // s + 1
    
    // At omega = 1: G(j) = 1 / (j + 1)
    std::complex<double> result = tf.evaluate(1.0);
    EXPECT_GT(std::abs(result), 0.0);
}

TEST(TransferFunctionTest, FromZPK) {
    std::vector<std::complex<double>> zeros;
    std::vector<std::complex<double>> poles = {{-1.0, 0.0}};
    double gain = 1.0;
    
    TransferFunction tf = TransferFunction::fromZPK(zeros, poles, gain);
}

// ============================================================================
// QFTController Tests
// ============================================================================

class QFTControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<QFTController>();
    }
    
    std::unique_ptr<QFTController> controller_;
};

TEST_F(QFTControllerTest, GetName) {
    EXPECT_STREQ(controller_->getName(), "QFT Controller");
}

TEST_F(QFTControllerTest, GetDescription) {
    EXPECT_NE(controller_->getDescription(), nullptr);
    EXPECT_GT(strlen(controller_->getDescription()), 0);
}

TEST_F(QFTControllerTest, GetType) {
    EXPECT_EQ(controller_->getType(), ControllerType::Custom);
}

TEST_F(QFTControllerTest, SetNominalPlantFOPDT) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);  // K=1, tau=10, L=2
}

TEST_F(QFTControllerTest, SetNominalPlantSOPDT) {
    controller_->setNominalPlant(1.0, 10.0, 5.0, 2.0);  // K=1, tau1=10, tau2=5, L=2
}

TEST_F(QFTControllerTest, SetNominalPlantTransferFunction) {
    TransferFunction tf;
    tf.num = {1.0};
    tf.den = {1.0, 1.0};
    controller_->setNominalPlant(tf);
}

TEST_F(QFTControllerTest, SetNominalPlantStateSpace) {
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller_->setNominalPlant(A, B, C, D, 1, 1, 1);
}

TEST_F(QFTControllerTest, SetFOPDTUncertainty) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
}

TEST_F(QFTControllerTest, AddParametricUncertainty) {
    controller_->addParametricUncertainty(0, 0.5, 2.0, 5);
}

TEST_F(QFTControllerTest, SetMultiplicativeUncertainty) {
    TransferFunction W;
    W.num = {0.1, 1.0};
    W.den = {1.0, 10.0};
    controller_->setMultiplicativeUncertainty(W);
}

TEST_F(QFTControllerTest, SetPlantSamples) {
    std::vector<TransferFunction> plants;
    TransferFunction tf1, tf2;
    tf1.num = {1.0}; tf1.den = {1.0, 1.0};
    tf2.num = {2.0}; tf2.den = {1.0, 2.0};
    plants.push_back(tf1);
    plants.push_back(tf2);
    controller_->setPlantSamples(plants);
}

TEST_F(QFTControllerTest, SetTrackingSpecDetailed) {
    std::vector<double> freqs = {0.1, 1.0, 10.0};
    std::vector<double> lower = {-3.0, -3.0, -6.0};
    std::vector<double> upper = {1.0, 1.0, -1.0};
    controller_->setTrackingSpec(freqs, lower, upper);
}

TEST_F(QFTControllerTest, SetTrackingSpecBandwidth) {
    controller_->setTrackingSpec(10.0, {0.4, 0.8});
}

TEST_F(QFTControllerTest, SetDisturbanceSpecDetailed) {
    std::vector<double> freqs = {0.1, 1.0, 10.0};
    std::vector<double> maxSens = {-40.0, -20.0, 0.0};
    controller_->setDisturbanceSpec(freqs, maxSens);
}

TEST_F(QFTControllerTest, SetDisturbanceSpecSimple) {
    controller_->setDisturbanceSpec(-6.0, 10.0);  // -6 dB below 10 rad/s
}

TEST_F(QFTControllerTest, SetStabilityMargins) {
    controller_->setStabilityMargins(6.0, 45.0);  // 6 dB GM, 45 deg PM
}

TEST_F(QFTControllerTest, SetControlEffortSpec) {
    TransferFunction bound;
    bound.num = {100.0};
    bound.den = {1.0, 10.0};
    controller_->setControlEffortSpec(bound);
}

TEST_F(QFTControllerTest, SetDesignFrequencies) {
    std::vector<double> freqs = {0.1, 0.5, 1.0, 5.0, 10.0, 50.0};
    controller_->setDesignFrequencies(freqs);
}

TEST_F(QFTControllerTest, AutoSelectFrequencies) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->autoSelectFrequencies(20);
}

TEST_F(QFTControllerTest, ComputeTemplates) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
    controller_->setDesignFrequencies({0.1, 1.0, 10.0});
    controller_->computeTemplates();
    
    const auto& templates = controller_->getTemplates();
}

TEST_F(QFTControllerTest, ComputeBounds) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
    controller_->setTrackingSpec(10.0);
    controller_->setStabilityMargins(6.0, 45.0);
    controller_->setDesignFrequencies({0.1, 1.0, 10.0});
    controller_->computeTemplates();
    controller_->computeBounds();
    
    const auto& bounds = controller_->getBounds();
}

TEST_F(QFTControllerTest, AutoShapeLoop) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
    controller_->setStabilityMargins(6.0, 45.0);
    controller_->setDesignFrequencies({0.1, 1.0, 10.0});
    controller_->computeTemplates();
    controller_->computeBounds();
    
    bool success = controller_->autoShapeLoop();
}

TEST_F(QFTControllerTest, AutoShapeLoopWithConfig) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    
    QFTController::LoopShapingConfig config;
    config.maxPoles = 4;
    config.maxZeros = 3;
    config.allowUnstablePoles = false;
    config.optimizationIterations = 100;
    
    bool success = controller_->autoShapeLoop(config);
}

TEST_F(QFTControllerTest, SetController) {
    TransferFunction C;
    C.num = {10.0, 1.0};
    C.den = {1.0, 0.1};
    controller_->setController(C);
}

TEST_F(QFTControllerTest, AddControllerElements) {
    controller_->addControllerPole(-10.0);
    controller_->addControllerZero(-1.0);
    controller_->addControllerPole(std::complex<double>(-5.0, 5.0));
    controller_->addControllerZero(std::complex<double>(-2.0, 2.0));
    controller_->setControllerGain(2.0);
}

TEST_F(QFTControllerTest, CheckBounds) {
    bool satisfied = controller_->checkBounds();
}

TEST_F(QFTControllerTest, GetBoundSatisfaction) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setDesignFrequencies({0.1, 1.0, 10.0});
    
    auto satisfaction = controller_->getBoundSatisfaction();
}

TEST_F(QFTControllerTest, DesignPrefilter) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setTrackingSpec(10.0);
    controller_->designPrefilter();
}

TEST_F(QFTControllerTest, DesignPrefilterWithBandwidth) {
    controller_->designPrefilter(10.0, 2);
}

TEST_F(QFTControllerTest, SetPrefilter) {
    TransferFunction F;
    F.num = {100.0};
    F.den = {1.0, 10.0};
    controller_->setPrefilter(F);
}

TEST_F(QFTControllerTest, EvaluateLoop) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    TransferFunction C;
    C.num = {10.0};
    C.den = {1.0, 1.0};
    controller_->setController(C);
    
    std::complex<double> L = controller_->evaluateLoop(1.0);
}

TEST_F(QFTControllerTest, EvaluateClosedLoop) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    TransferFunction C;
    C.num = {10.0};
    C.den = {1.0, 1.0};
    controller_->setController(C);
    
    std::complex<double> T = controller_->evaluateClosedLoop(1.0);
}

TEST_F(QFTControllerTest, EvaluateSensitivity) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    TransferFunction C;
    C.num = {10.0};
    C.den = {1.0, 1.0};
    controller_->setController(C);
    
    std::complex<double> S = controller_->evaluateSensitivity(1.0);
}

TEST_F(QFTControllerTest, AnalyzeMargins) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    TransferFunction C;
    C.num = {10.0};
    C.den = {1.0, 1.0};
    controller_->setController(C);
    
    auto margins = controller_->analyzeMargins();
    // margins.gainMargin, margins.phaseMargin should be meaningful
}

TEST_F(QFTControllerTest, WorstCaseResponse) {
    controller_->setNominalPlant(1.0, 10.0, 2.0);
    controller_->setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
    
    auto [minResp, maxResp] = controller_->worstCaseResponse(1.0);
}

// ============================================================================
// NicholsChart Tests
// ============================================================================

TEST(NicholsChartTest, MCircle) {
    auto circle = NicholsChart::mCircle(0.0);  // 0 dB M-circle
    EXPECT_FALSE(circle.empty());
}

TEST(NicholsChartTest, NCircle) {
    auto circle = NicholsChart::nCircle(-90.0);  // -90 degrees N-circle
}

TEST(NicholsChartTest, ClosedLoopMagnitude) {
    NicholsPoint point(0.0, -90.0);
    double mag = NicholsChart::closedLoopMagnitude(point);
}

TEST(NicholsChartTest, ClosedLoopPhase) {
    NicholsPoint point(0.0, -90.0);
    double phase = NicholsChart::closedLoopPhase(point);
}

TEST(NicholsChartTest, IsUnstableRegion) {
    NicholsPoint stable(0.0, -135.0);
    NicholsPoint unstable(0.0, -181.0);  // Past -180 degrees
    
    // Check unstable region detection
    bool isStable = !NicholsChart::isUnstableRegion(stable);
}

// ============================================================================
// TemplateUtils Tests
// ============================================================================

TEST(TemplateUtilsTest, ConvexHull) {
    std::vector<NicholsPoint> points = {
        NicholsPoint(0.0, -90.0),
        NicholsPoint(3.0, -100.0),
        NicholsPoint(1.0, -95.0),  // Interior point
        NicholsPoint(-3.0, -80.0)
    };
    
    auto hull = TemplateUtils::convexHull(points);
}

TEST(TemplateUtilsTest, ShiftTemplate) {
    PlantTemplate templ;
    templ.frequency = 1.0;
    templ.points.push_back(NicholsPoint(0.0, -90.0));
    templ.points.push_back(NicholsPoint(3.0, -100.0));
    
    NicholsPoint loopGain(6.0, -45.0);
    PlantTemplate shifted = TemplateUtils::shiftTemplate(templ, loopGain);
}

TEST(TemplateUtilsTest, InterpolateTemplate) {
    PlantTemplate t1, t2;
    t1.frequency = 1.0;
    t2.frequency = 10.0;
    t1.points.push_back(NicholsPoint(0.0, -45.0));
    t2.points.push_back(NicholsPoint(-20.0, -135.0));
    
    PlantTemplate interp = TemplateUtils::interpolateTemplate(t1, t2, 3.0);
}

// ============================================================================
// QFTLoopShaper Tests
// ============================================================================

TEST(QFTLoopShaperTest, ConfigDefaults) {
    QFTLoopShaper::Config config;
    EXPECT_EQ(config.maxPoles, 6);
    EXPECT_EQ(config.maxZeros, 5);
    EXPECT_DOUBLE_EQ(config.gainMin, 0.001);
    EXPECT_DOUBLE_EQ(config.gainMax, 1000.0);
}

TEST(QFTLoopShaperTest, DefaultConfig) {
    auto config = QFTLoopShaper::Config::defaultConfig();
    EXPECT_EQ(config.maxPoles, 6);
}

TEST(QFTLoopShaperTest, Optimize) {
    TransferFunction nominalPlant;
    nominalPlant.num = {1.0};
    nominalPlant.den = {10.0, 1.0};  // 1/(10s + 1)
    
    std::vector<QFTBound> bounds;
    std::vector<double> frequencies = {0.1, 1.0, 10.0};
    
    QFTLoopShaper::Config config;
    config.generations = 10;  // Quick test
    
    TransferFunction controller = QFTLoopShaper::optimize(
        nominalPlant, bounds, frequencies, config);
}

TEST(QFTLoopShaperTest, OptimizeDefaultConfig) {
    TransferFunction nominalPlant;
    nominalPlant.num = {1.0};
    nominalPlant.den = {10.0, 1.0};
    
    std::vector<QFTBound> bounds;
    std::vector<double> frequencies = {0.1, 1.0};
    
    TransferFunction controller = QFTLoopShaper::optimize(
        nominalPlant, bounds, frequencies);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(QFTIntegration, FullDesignWorkflow) {
    QFTController controller;
    
    // Step 1: Define uncertain plant
    controller.setNominalPlant(1.0, 10.0, 2.0);
    controller.setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 3.0);
    
    // Step 2: Set specifications
    controller.setTrackingSpec(10.0, {0.4, 0.8});
    controller.setDisturbanceSpec(-20.0, 1.0);
    controller.setStabilityMargins(6.0, 45.0);
    
    // Step 3: Select frequencies and compute
    controller.autoSelectFrequencies(10);
    controller.computeTemplates();
    controller.computeBounds();
    
    // Step 4: Design
    QFTController::LoopShapingConfig config;
    config.maxPoles = 3;
    config.optimizationIterations = 50;
    controller.autoShapeLoop(config);
    controller.designPrefilter();
    
    // Step 5: Analyze
    auto margins = controller.analyzeMargins();
    EXPECT_GE(margins.gainMargin, 0.0);
}

TEST(QFTIntegration, StabilityMarginsValid) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    
    // Set a simple controller
    TransferFunction C;
    C.num = {10.0, 1.0};
    C.den = {1.0, 0.1};
    controller.setController(C);
    
    auto margins = controller.analyzeMargins();
    
    // Should produce finite margins
    EXPECT_TRUE(std::isfinite(margins.gainMargin) || margins.gainMargin == 0.0);
    EXPECT_TRUE(std::isfinite(margins.phaseMargin) || margins.phaseMargin == 0.0);
}

// ============================================================================
// Additional Coverage Tests for QFT
// ============================================================================

TEST(NicholsPointCoverage, VaryingComplex) {
    std::vector<std::complex<double>> testValues = {
        {1.0, 0.0},     // Real positive
        {-1.0, 0.0},    // Real negative  
        {0.0, 1.0},     // Imaginary positive
        {0.0, -1.0},    // Imaginary negative
        {1.0, 1.0},     // First quadrant
        {-1.0, 1.0},    // Second quadrant
        {-1.0, -1.0},   // Third quadrant
        {1.0, -1.0},    // Fourth quadrant
        {0.1, 0.0},     // Small magnitude
        {10.0, 0.0}     // Large magnitude
    };
    
    for (const auto& z : testValues) {
        NicholsPoint point = NicholsPoint::fromComplex(z);
        std::complex<double> recovered = point.toComplex();
        
        EXPECT_NEAR(std::abs(recovered), std::abs(z), 0.01 * std::abs(z) + 1e-9);
    }
}

TEST(NicholsPointCoverage, RoundTripConversion) {
    std::vector<NicholsPoint> testPoints = {
        {0.0, 0.0},
        {6.0, -45.0},
        {-6.0, -90.0},
        {12.0, -135.0},
        {-12.0, -180.0},
        {20.0, -270.0}
    };
    
    for (const auto& original : testPoints) {
        auto complex = original.toComplex();
        auto recovered = NicholsPoint::fromComplex(complex);
        
        EXPECT_NEAR(recovered.gain, original.gain, 0.1);
    }
}

TEST(PlantTemplateCoverage, LargeTemplate) {
    PlantTemplate templ;
    templ.frequency = 1.0;
    
    // Create a template with many points
    for (int i = 0; i < 100; ++i) {
        double gain = -6.0 + 0.2 * (i % 10);
        double phase = -180.0 + 3.6 * i;
        templ.points.push_back(NicholsPoint(gain, phase));
    }
    
    auto boundary = templ.boundary();
    EXPECT_FALSE(boundary.empty());
}

TEST(PlantTemplateCoverage, DegenerateTemplate) {
    PlantTemplate templ;
    templ.frequency = 1.0;
    
    // Single point
    templ.points.push_back(NicholsPoint(0.0, -90.0));
    auto boundary1 = templ.boundary();
    
    // Two points
    templ.points.push_back(NicholsPoint(3.0, -120.0));
    auto boundary2 = templ.boundary();
}

TEST(QFTBoundCoverage, AllBoundTypes) {
    // QFTBound has only: Tracking, Stability, Disturbance, ControlEffort, Combined
    // (No Sensitivity type)
    std::vector<QFTBound::Type> types = {
        QFTBound::Tracking,
        QFTBound::Stability,
        QFTBound::Disturbance,
        QFTBound::ControlEffort,
        QFTBound::Combined
    };
    
    for (auto type : types) {
        QFTBound bound;
        bound.type = type;
        bound.frequency = 1.0;
        // QFTBound has 'boundary' vector, not 'points'
        bound.boundary.push_back(NicholsPoint(0.0, -180.0));
        bound.boundary.push_back(NicholsPoint(-6.0, -135.0));
        
        EXPECT_EQ(bound.type, type);
    }
}

TEST(QFTControllerCoverage, VaryingUncertainty) {
    std::vector<std::pair<double, double>> kRanges = {
        {0.5, 2.0},
        {0.8, 1.2},
        {0.1, 10.0}
    };
    
    for (const auto& [kMin, kMax] : kRanges) {
        QFTController controller;
        controller.setNominalPlant(1.0, 10.0, 2.0);
        controller.setFOPDTUncertainty(kMin, kMax, 5.0, 20.0, 1.0, 3.0);
        
        controller.autoSelectFrequencies(5);
        controller.computeTemplates();
        
        auto templates = controller.getTemplates();
    }
}

TEST(QFTControllerCoverage, AllSpecifications) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    
    // Tracking spec
    controller.setTrackingSpec(20.0, {0.3, 0.9});
    controller.setTrackingSpec(10.0, {0.5, 0.95});
    
    // Disturbance rejection
    controller.setDisturbanceSpec(-40.0, 0.1);
    controller.setDisturbanceSpec(-20.0, 1.0);
    controller.setDisturbanceSpec(-10.0, 10.0);
    
    // Stability margins
    controller.setStabilityMargins(3.0, 30.0);
    controller.setStabilityMargins(6.0, 45.0);
    controller.setStabilityMargins(10.0, 60.0);
    
    controller.autoSelectFrequencies(8);
    controller.computeBounds();
    
    auto bounds = controller.getBounds();
}

TEST(QFTControllerCoverage, ManualFrequencySelection) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    
    std::vector<double> frequencies = {0.01, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0};
    controller.setDesignFrequencies(frequencies);  // Use setDesignFrequencies
    
    controller.computeTemplates();
    
    auto templates = controller.getTemplates();
    EXPECT_EQ(templates.size(), frequencies.size());
}

TEST(QFTControllerCoverage, ControllerDesignOptions) {
    QFTController controller;
    controller.setNominalPlant(1.0, 5.0, 1.0);
    controller.setFOPDTUncertainty(0.8, 1.2, 4.0, 6.0, 0.5, 1.5);
    controller.setStabilityMargins(6.0, 45.0);
    
    controller.autoSelectFrequencies(5);
    controller.computeTemplates();
    controller.computeBounds();
    
    // Different loop shaping configurations
    QFTController::LoopShapingConfig config1;
    config1.maxPoles = 2;
    config1.maxZeros = 1;
    config1.optimizationIterations = 20;
    controller.autoShapeLoop(config1);
    
    auto ctrl1 = controller.getController();
    
    QFTController::LoopShapingConfig config2;
    config2.maxPoles = 4;
    config2.maxZeros = 3;
    config2.optimizationIterations = 50;
    controller.autoShapeLoop(config2);
    
    auto ctrl2 = controller.getController();
}

TEST(QFTControllerCoverage, PrefilterDesign) {
    QFTController controller;
    controller.setNominalPlant(2.0, 8.0, 1.5);
    controller.setTrackingSpec(10.0, {0.4, 0.8});
    
    controller.autoSelectFrequencies(5);
    controller.computeTemplates();
    controller.computeBounds();
    
    // Set controller first
    TransferFunction C;
    C.num = {5.0, 1.0};
    C.den = {1.0, 0.1};
    controller.setController(C);
    
    controller.designPrefilter();
    auto prefilter = controller.getPrefilter();
}

TEST(QFTControllerCoverage, MarginAnalysis) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    
    // Various controller configurations
    std::vector<TransferFunction> controllers;
    
    TransferFunction c1;
    c1.num = {1.0};
    c1.den = {1.0};
    controllers.push_back(c1);
    
    TransferFunction c2;
    c2.num = {10.0};
    c2.den = {1.0};
    controllers.push_back(c2);
    
    TransferFunction c3;
    c3.num = {5.0, 1.0};
    c3.den = {1.0, 0.5};
    controllers.push_back(c3);
    
    for (const auto& C : controllers) {
        controller.setController(C);
        auto margins = controller.analyzeMargins();
    }
}

// Tests removed - TemplateGenerator and BoundComputer are not directly accessible
// These are internal classes used by QFTController
// Template generation is tested through QFTController::computeTemplates()
TEST(QFTControllerCoverage, TemplateGeneration) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    controller.setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 4.0);
    controller.autoSelectFrequencies(5);
    
    controller.computeTemplates();
    auto templates = controller.getTemplates();
    
    EXPECT_GE(templates.size(), 1);
    for (const auto& templ : templates) {
        EXPECT_GT(templ.frequency, 0.0);
        EXPECT_FALSE(templ.points.empty());
    }
}

TEST(QFTControllerCoverage, BoundComputation) {
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    controller.setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 4.0);
    controller.setTrackingSpec(10.0, {0.4, 0.8});
    controller.setStabilityMargins(6.0, 45.0);
    controller.autoSelectFrequencies(5);
    
    controller.computeTemplates();
    controller.computeBounds();
    auto bounds = controller.getBounds();
    
    EXPECT_FALSE(bounds.empty());
    for (const auto& bound : bounds) {
        EXPECT_GT(bound.frequency, 0.0);
        EXPECT_FALSE(bound.boundary.empty());
    }
}

TEST(QFTLoopShaperCoverage, OptimizationParams) {
    // QFTLoopShaper is also internal - test through QFTController
    QFTController controller;
    controller.setNominalPlant(1.0, 10.0, 2.0);
    controller.setStabilityMargins(6.0, 45.0);
    controller.autoSelectFrequencies(5);
    controller.computeTemplates();
    controller.computeBounds();
    
    QFTController::LoopShapingConfig config;
    config.maxPoles = 3;
    config.maxZeros = 2;
    config.optimizationIterations = 50;
    controller.autoShapeLoop(config);
    
    auto ctrl = controller.getController();
}

TEST(QFTIntegration, RobustDesign) {
    QFTController controller;
    
    // Large parameter uncertainty
    controller.setNominalPlant(1.0, 10.0, 2.0);
    controller.setFOPDTUncertainty(0.5, 2.0, 5.0, 20.0, 1.0, 5.0);
    
    // Demanding specs
    controller.setTrackingSpec(20.0, {0.4, 0.8});
    controller.setDisturbanceSpec(-30.0, 0.5);
    controller.setStabilityMargins(8.0, 50.0);
    
    controller.autoSelectFrequencies(10);
    controller.computeTemplates();
    controller.computeBounds();
    
    QFTController::LoopShapingConfig config;
    config.maxPoles = 4;
    config.maxZeros = 3;
    config.optimizationIterations = 100;
    controller.autoShapeLoop(config);
    controller.designPrefilter();
    
    auto margins = controller.analyzeMargins();
    auto ctrl = controller.getController();
    auto pf = controller.getPrefilter();
}

TEST(QFTIntegration, FastProcess) {
    QFTController controller;
    
    // Fast process (small time constants)
    controller.setNominalPlant(0.5, 0.5, 0.1);
    controller.setFOPDTUncertainty(0.4, 0.6, 0.3, 0.7, 0.05, 0.15);
    controller.setStabilityMargins(6.0, 45.0);
    
    controller.setDesignFrequencies({0.1, 1.0, 5.0, 10.0, 50.0});
    controller.computeTemplates();
    controller.computeBounds();
    
    QFTController::LoopShapingConfig config;
    config.maxPoles = 3;
    controller.autoShapeLoop(config);
    
    auto margins = controller.analyzeMargins();
}

TEST(QFTIntegration, SlowProcess) {
    QFTController controller;
    
    // Slow process (large time constants)
    controller.setNominalPlant(5.0, 100.0, 20.0);
    controller.setFOPDTUncertainty(3.0, 7.0, 80.0, 120.0, 15.0, 25.0);
    controller.setStabilityMargins(6.0, 45.0);
    
    controller.setDesignFrequencies({0.001, 0.01, 0.05, 0.1, 0.5});
    controller.computeTemplates();
    controller.computeBounds();
    
    QFTController::LoopShapingConfig config;
    config.maxPoles = 3;
    controller.autoShapeLoop(config);
    
    auto margins = controller.analyzeMargins();
}
