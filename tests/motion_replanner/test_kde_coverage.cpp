/**
 * @file test_kde_coverage.cpp
 * @brief Coverage tests for KdeGridEvaluator, BandwidthSelector and
 *        KdeDerivativeAnalyzer config-dispatch branches.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/KdeGridEvaluator.hpp"
#include "tether/motion_replanner/BandwidthSelector.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>

using namespace tether::motion::replanner;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.acceleration = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.jerk = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = 0.0;
    return s;
}

/// Straight line path (0,0)→(100,10) with populated derivatives.
std::vector<TrajectorySample> makeLinePath(int n = 64) {
    std::vector<TrajectorySample> samples;
    double dt = 0.01;
    double dx = 100.0 / static_cast<double>(n - 1);
    double dy = 10.0 / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        double x = i * dx;
        double y = i * dy;
        double s = std::sqrt(x * x + y * y);
        TrajectorySample sm = makeSample(i * dt, x, y, 0, 1, s);
        sm.velocity[0] = 99.5;
        sm.velocity[1] = 9.95;
        sm.acceleration[0] = 10.0;
        sm.acceleration[1] = 1.0;
        sm.jerk[0] = 2.0;
        sm.jerk[1] = 0.2;
        sm.curvature = 0.001 * i;
        samples.push_back(sm);
    }
    return samples;
}

std::vector<TrajectorySample> makeNoisyActual(
    const std::vector<TrajectorySample>& desired,
    double noiseStdDev,
    unsigned seed = 42) {
    std::vector<TrajectorySample> actual = desired;
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, noiseStdDev);
    double nx = -0.0995, ny = 0.995;
    for (auto& s : actual) {
        double offset = noise(rng);
        s.position[0] += offset * nx;
        s.position[1] += offset * ny;
        s.velocity[0] += noise(rng);
        s.acceleration[0] += noise(rng);
        s.jerk[0] += noise(rng);
    }
    return actual;
}

std::vector<double> makeNormalData(int n, unsigned seed = 7) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> dist(0.0, 1.0);
    std::vector<double> v;
    for (int i = 0; i < n; ++i) v.push_back(dist(rng));
    return v;
}

} // anonymous namespace

//=============================================================================
// KdeGridEvaluator::kernelValue — all kernel shapes
//=============================================================================

TEST(KdeGridEvaluatorCov, KernelValues_AllTypes) {
    // Bounded kernels are zero outside |u| > 1 and positive inside.
    for (KernelType k : {KernelType::Epanechnikov, KernelType::Uniform,
                         KernelType::Triangular, KernelType::Quartic,
                         KernelType::Cosine}) {
        EXPECT_GT(KdeGridEvaluator::kernelValue(k, 0.0), 0.0);
        EXPECT_GT(KdeGridEvaluator::kernelValue(k, 0.5), 0.0);
        EXPECT_EQ(KdeGridEvaluator::kernelValue(k, 1.5), 0.0);
    }
    EXPECT_NEAR(KdeGridEvaluator::kernelValue(KernelType::Gaussian, 0.0),
                1.0 / std::sqrt(2.0 * M_PI), 1e-12);
}

TEST(KdeGridEvaluatorCov, Evaluate_NonGaussianKernel) {
    std::vector<double> dx = {0.1, 0.3, 0.5, 0.7, 0.9};
    std::vector<double> dy = {0.01, 0.02, 0.03, 0.02, 0.01};
    auto grid = KdeGridEvaluator::evaluate(dx, dy, 0.2, 0.01,
                                           0.0, 1.0, 0.0, 0.05, 16, 8,
                                           KernelType::Epanechnikov);
    ASSERT_EQ(grid.density.size(), 16u * 8u);
    EXPECT_GT(grid.totalMass(), 0.0);
}

//=============================================================================
// KdeGridEvaluator::binnedEvaluate
//=============================================================================

TEST(KdeGridEvaluatorCov, BinnedEvaluate_Gaussian) {
    std::vector<double> dx = makeNormalData(200);
    std::vector<double> dy = makeNormalData(200, 99);
    auto grid = KdeGridEvaluator::binnedEvaluate(dx, dy, 0.5, 0.5,
                                                 -4.0, 4.0, -4.0, 4.0,
                                                 32, 32, KernelType::Gaussian,
                                                 64);
    ASSERT_EQ(grid.density.size(), 32u * 32u);
    EXPECT_EQ(grid.sampleCount, 200u);
    EXPECT_GT(grid.totalMass(), 0.0);
}

TEST(KdeGridEvaluatorCov, BinnedEvaluate_CompactKernel) {
    std::vector<double> dx = makeNormalData(150);
    std::vector<double> dy = makeNormalData(150, 55);
    auto grid = KdeGridEvaluator::binnedEvaluate(dx, dy, 1.0, 1.0,
                                                 -4.0, 4.0, -4.0, 4.0,
                                                 24, 24, KernelType::Uniform,
                                                 32);
    ASSERT_EQ(grid.density.size(), 24u * 24u);
    EXPECT_GT(grid.totalMass(), 0.0);
}

TEST(KdeGridEvaluatorCov, BinnedEvaluate_DegenerateRange) {
    // Zero-width range forces the dxHist/dyHist fallback.
    std::vector<double> dx(20, 1.0);
    std::vector<double> dy(20, 2.0);
    auto grid = KdeGridEvaluator::binnedEvaluate(dx, dy, 0.5, 0.5,
                                                 1.0, 1.0, 2.0, 2.0,
                                                 8, 8, KernelType::Gaussian, 16);
    EXPECT_EQ(grid.density.size(), 64u);
}

//=============================================================================
// BandwidthSelector — every selection method
//=============================================================================

TEST(BandwidthSelectorCov, ISJ) {
    auto data = makeNormalData(80);
    double h = BandwidthSelector::isj(data);
    EXPECT_GT(h, 0.0);
}

TEST(BandwidthSelectorCov, ISJ_ConstantData) {
    std::vector<double> data(50, 3.0);
    EXPECT_GE(BandwidthSelector::isj(data), 0.0);
}

TEST(BandwidthSelectorCov, LSCV) {
    auto data = makeNormalData(60);
    double h = BandwidthSelector::lscv(data);
    EXPECT_GT(h, 0.0);
}

TEST(BandwidthSelectorCov, LSCV_TinyInputFallsBack) {
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0};
    EXPECT_EQ(BandwidthSelector::lscv(data),
              BandwidthSelector::silverman(data));
}

TEST(BandwidthSelectorCov, LikelihoodCV) {
    auto data = makeNormalData(60);
    double h = BandwidthSelector::likelihoodCv(data);
    EXPECT_GT(h, 0.0);
}

TEST(BandwidthSelectorCov, LikelihoodCV_TinyInputFallsBack) {
    std::vector<double> data = {1.0, 2.0};
    EXPECT_EQ(BandwidthSelector::likelihoodCv(data),
              BandwidthSelector::silverman(data));
}

TEST(BandwidthSelectorCov, Compute_AllMethods) {
    auto data = makeNormalData(80);
    for (BandwidthMethod m : {BandwidthMethod::Silverman, BandwidthMethod::Scott,
                            BandwidthMethod::ISJ, BandwidthMethod::LeastSquaresCV,
                            BandwidthMethod::LikelihoodCV}) {
        double h = BandwidthSelector::compute(data, m, 0.0, 1.0);
        EXPECT_GT(h, 0.0);
    }
    EXPECT_DOUBLE_EQ(BandwidthSelector::compute(data, BandwidthMethod::Fixed,
                                                0.75, 1.0), 0.75);
}

TEST(BandwidthSelectorCov, Compute_DegenerateInputs) {
    EXPECT_EQ(BandwidthSelector::compute({}, BandwidthMethod::Silverman, 0, 1),
              1e-6);
    std::vector<double> one = {5.0};
    EXPECT_EQ(BandwidthSelector::compute(one, BandwidthMethod::Scott, 0, 1),
              1e-6);
    std::vector<double> flat(10, 2.0);
    EXPECT_EQ(BandwidthSelector::compute(flat, BandwidthMethod::Silverman, 0, 1),
              1e-6);
}

// Note: BandwidthSelector::quantileSorted/stdDev/mean are private helpers,
// exercised indirectly through isj/silverman calls above.

//=============================================================================
// KdeDerivativeAnalyzer — derivative/deviation axis dispatch
//=============================================================================

class KdeAxisDispatch : public ::testing::Test {
protected:
    std::vector<TrajectorySample> desired = makeLinePath(64);
    std::vector<TrajectorySample> actual = makeNoisyActual(desired, 0.002);
};

TEST_F(KdeAxisDispatch, DerivativeAxes_All) {
    for (DerivativeAxis axis :
         {DerivativeAxis::Velocity, DerivativeAxis::Acceleration,
          DerivativeAxis::Jerk, DerivativeAxis::Curvature,
          DerivativeAxis::FeedRate, DerivativeAxis::ArcLength,
          DerivativeAxis::Time}) {
        KdeConfig config;
        config.derivativeAxis = axis;
        config.deviationAxis = DeviationAxis::CombinedError;
        config.useCertifiedContourError = false;
        KdeDerivativeAnalyzer analyzer(config);
        auto pairs = analyzer.extractPairs(desired, actual);
        ASSERT_EQ(pairs.derivatives.size(), desired.size());
    }
}

TEST_F(KdeAxisDispatch, DeviationAxes_All) {
    for (DeviationAxis axis :
         {DeviationAxis::ContourError, DeviationAxis::LagError,
          DeviationAxis::CombinedError, DeviationAxis::BinormalError,
          DeviationAxis::TrackingError, DeviationAxis::VelocityError,
          DeviationAxis::AccelerationError}) {
        KdeConfig config;
        config.derivativeAxis = DerivativeAxis::Velocity;
        config.deviationAxis = axis;
        config.useCertifiedContourError = false;
        KdeDerivativeAnalyzer analyzer(config);
        auto pairs = analyzer.extractPairs(desired, actual);
        ASSERT_EQ(pairs.deviations.size(), desired.size());
    }
}

TEST_F(KdeAxisDispatch, LagError_CertifiedAndUncertified) {
    for (bool certified : {true, false}) {
        KdeConfig config;
        config.deviationAxis = DeviationAxis::LagError;
        config.useCertifiedContourError = certified;
        KdeDerivativeAnalyzer analyzer(config);
        auto pairs = analyzer.extractPairs(desired, actual);
        ASSERT_EQ(pairs.deviations.size(), desired.size());
    }
}

TEST_F(KdeAxisDispatch, Evaluate_BinnedPathViaThreshold) {
    // Lower the binned threshold so evaluate() takes the binned branch.
    KdeConfig config;
    config.binnedThreshold = 10;
    config.binnedHistogramSize = 32;
    config.kernel = KernelType::Uniform;
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);
    EXPECT_TRUE(eval.hasSufficientData);
    EXPECT_EQ(eval.grid.density.size(),
              config.gridX * config.gridY);
}

TEST_F(KdeAxisDispatch, Evaluate_ExplicitRanges) {
    KdeConfig config;
    config.xRange = {0.0, 150.0};
    config.yRange = {0.0, 0.1};
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);
    EXPECT_TRUE(eval.hasSufficientData);
}

TEST_F(KdeAxisDispatch, Evaluate_ConstantDataExpandsRange) {
    // Identical deviations force the mx-mn < eps range expansion.
    auto actualFlat = desired;
    KdeConfig config;
    config.useCertifiedContourError = false;
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actualFlat);
    EXPECT_TRUE(eval.hasSufficientData);
}

//=============================================================================
// KdeDerivativeAnalyzer — delegate methods
//=============================================================================

// Note: spearmanRank/kendallTau/distanceCorrelation/mutualInformationFromGrid/
// entropyFromGrid are private delegates with no internal callers — dead code
// that cannot be covered without changing the public API. binnedKde is
// private but reached via evaluate() when binnedThreshold is exceeded.

TEST(KdeDelegates, ConditionalStatsDirect) {
    std::vector<double> dx = makeNormalData(100);
    std::vector<double> dy = makeNormalData(100, 31);
    KdeDerivativeAnalyzer analyzer;
    auto grid = analyzer.evaluateKde(dx, dy, 1.0, 1.0,
                                     -4.0, 4.0, -4.0, 4.0, 16, 16,
                                     KernelType::Gaussian);
    auto cond = analyzer.computeConditionalStats(
        grid, {0.25, 0.5, 0.75}, 0.001);
    EXPECT_FALSE(cond.empty());
}

//=============================================================================
// Static helpers — density stats + colormaps
//=============================================================================

TEST(KdeStatics, DensityHelpers) {
    std::vector<double> bins = {0.0, 1.0, 2.0};
    std::vector<double> density = {0.1, 0.7, 0.2};
    EXPECT_NEAR(KdeDerivativeAnalyzer::densityMode(bins, density), 1.0, 1e-12);
    EXPECT_GT(KdeDerivativeAnalyzer::densityQuantile(bins, density, 0.9),
              KdeDerivativeAnalyzer::densityQuantile(bins, density, 0.1));
    EXPECT_GT(KdeDerivativeAnalyzer::densityEntropy(density), 0.0);
}

TEST(KdeStatics, ColormapColors) {
    for (KdeColormap c : {KdeColormap::Viridis, KdeColormap::Inferno,
                          KdeColormap::Plasma, KdeColormap::Magma,
                          KdeColormap::Jet, KdeColormap::Hot,
                          KdeColormap::Cool, KdeColormap::Grayscale,
                          KdeColormap::BlueRed}) {
        auto lo = KdeDerivativeAnalyzer::colormapColor(c, 0.0);
        auto hi = KdeDerivativeAnalyzer::colormapColor(c, 1.0);
        for (int v : lo) EXPECT_GE(v, 0);
        for (int v : hi) EXPECT_GE(v, 0);
        EXPECT_NE(lo, hi);
    }
}

//=============================================================================
// toString / unitString — full enum tables
//=============================================================================

TEST(KdeStrings, DerivativeAxisTable) {
    EXPECT_EQ(toString(DerivativeAxis::Velocity), "Velocity");
    EXPECT_EQ(toString(DerivativeAxis::Acceleration), "Acceleration");
    EXPECT_EQ(toString(DerivativeAxis::Jerk), "Jerk");
    EXPECT_EQ(toString(DerivativeAxis::Curvature), "Curvature");
    EXPECT_EQ(toString(DerivativeAxis::FeedRate), "FeedRate");
    EXPECT_EQ(toString(DerivativeAxis::ArcLength), "ArcLength");
    EXPECT_EQ(toString(DerivativeAxis::Time), "Time");
}

TEST(KdeStrings, DeviationAxisTable) {
    EXPECT_EQ(toString(DeviationAxis::ContourError), "ContourError");
    EXPECT_EQ(toString(DeviationAxis::LagError), "LagError");
    EXPECT_EQ(toString(DeviationAxis::CombinedError), "CombinedError");
    EXPECT_EQ(toString(DeviationAxis::BinormalError), "BinormalError");
    EXPECT_EQ(toString(DeviationAxis::TrackingError), "TrackingError");
    EXPECT_EQ(toString(DeviationAxis::VelocityError), "VelocityError");
    EXPECT_EQ(toString(DeviationAxis::AccelerationError), "AccelerationError");
}

TEST(KdeStrings, KernelAndMethodTables) {
    EXPECT_EQ(toString(KernelType::Triangular), "Triangular");
    EXPECT_EQ(toString(KernelType::Quartic), "Quartic");
    EXPECT_EQ(toString(KernelType::Cosine), "Cosine");
    EXPECT_EQ(toString(BandwidthMethod::LeastSquaresCV), "LeastSquaresCV");
    EXPECT_EQ(toString(BandwidthMethod::LikelihoodCV), "LikelihoodCV");
}

TEST(KdeStrings, ColormapTable) {
    EXPECT_EQ(toString(KdeColormap::Viridis), "Viridis");
    EXPECT_EQ(toString(KdeColormap::Inferno), "Inferno");
    EXPECT_EQ(toString(KdeColormap::Plasma), "Plasma");
    EXPECT_EQ(toString(KdeColormap::Magma), "Magma");
    EXPECT_EQ(toString(KdeColormap::Jet), "Jet");
    EXPECT_EQ(toString(KdeColormap::Hot), "Hot");
    EXPECT_EQ(toString(KdeColormap::Cool), "Cool");
    EXPECT_EQ(toString(KdeColormap::Grayscale), "Grayscale");
    EXPECT_EQ(toString(KdeColormap::BlueRed), "BlueRed");
}

TEST(KdeStrings, UnitStrings) {
    EXPECT_EQ(unitString(DerivativeAxis::Velocity), "mm/s");
    EXPECT_EQ(unitString(DerivativeAxis::Acceleration), "mm/s^2");
    EXPECT_EQ(unitString(DerivativeAxis::Jerk), "mm/s^3");
    EXPECT_EQ(unitString(DerivativeAxis::Curvature), "1/mm");
    EXPECT_EQ(unitString(DerivativeAxis::FeedRate), "mm/s");
    EXPECT_EQ(unitString(DerivativeAxis::ArcLength), "mm");
    EXPECT_EQ(unitString(DerivativeAxis::Time), "s");

    EXPECT_EQ(unitString(DeviationAxis::ContourError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::LagError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::CombinedError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::BinormalError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::TrackingError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::VelocityError), "mm/s");
    EXPECT_EQ(unitString(DeviationAxis::AccelerationError), "mm/s^2");
}
