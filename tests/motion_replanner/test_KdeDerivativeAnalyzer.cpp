/**
 * @file test_KdeDerivativeAnalyzer.cpp
 * @brief Tests for the KdeDerivativeAnalyzer.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <random>

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
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = 0.0;
    return s;
}

/// Build a straight line path: (0,0)→(100,10) with velocity, both X and Y vary.
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
        // Set velocity along the path direction
        sm.velocity[0] = 99.5;  // ≈ 100 mm/s * cos(atan(0.1))
        sm.velocity[1] = 9.95;  // ≈ 100 mm/s * sin(atan(0.1))
        samples.push_back(sm);
    }
    return samples;
}

/// Build actual trajectory with velocity-dependent deviation.
/// Deviation increases linearly with velocity.
std::vector<TrajectorySample> makeVelocityDependentActual(
    const std::vector<TrajectorySample>& desired,
    double scaleFactor) {
    std::vector<TrajectorySample> actual = desired;
    double nx = -0.0995, ny = 0.995; // Normal to the (100,10) line
    for (std::size_t i = 0; i < actual.size(); ++i) {
        double vel = std::sqrt(desired[i].velocity[0] * desired[i].velocity[0] +
                               desired[i].velocity[1] * desired[i].velocity[1]);
        double offset = vel * scaleFactor;
        actual[i].position[0] += offset * nx;
        actual[i].position[1] += offset * ny;
    }
    return actual;
}

/// Build actual trajectory with random noise deviation.
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
    }
    return actual;
}

} // anonymous namespace

//=============================================================================
// Basic functionality
//=============================================================================

TEST(KdeDerivativeAnalyzer, EmptyInput_ReturnsEmpty) {
    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    std::vector<TrajectorySample> empty;
    auto eval = analyzer.evaluate(empty, empty);

    EXPECT_FALSE(eval.hasSufficientData);
    EXPECT_TRUE(eval.derivatives.empty());
    EXPECT_TRUE(eval.deviations.empty());
}

TEST(KdeDerivativeAnalyzer, TooFewSamples_HasInsufficientData) {
    auto desired = makeLinePath(10);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);

    EXPECT_FALSE(eval.hasSufficientData);
}

TEST(KdeDerivativeAnalyzer, SufficientData_HasSufficientData) {
    auto desired = makeLinePath(64);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);

    EXPECT_TRUE(eval.hasSufficientData);
    EXPECT_EQ(eval.derivatives.size(), 64u);
    EXPECT_EQ(eval.deviations.size(), 64u);
}

//=============================================================================
// Pair extraction
//=============================================================================

TEST(KdeDerivativeAnalyzer, ExtractPairs_VelocityVsContour) {
    auto desired = makeLinePath(64);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::Velocity;
    config.deviationAxis = DeviationAxis::ContourError;
    config.useCertifiedContourError = false;
    KdeDerivativeAnalyzer analyzer(config);

    auto pairs = analyzer.extractPairs(desired, actual);

    EXPECT_EQ(pairs.derivatives.size(), 64u);
    EXPECT_EQ(pairs.deviations.size(), 64u);

    // Velocity should be approximately 100 mm/s for all samples
    for (double d : pairs.derivatives) {
        EXPECT_NEAR(d, 100.0, 1.0);
    }

    // Deviations should be positive (contour error)
    for (double e : pairs.deviations) {
        EXPECT_GE(e, 0.0);
    }
}

TEST(KdeDerivativeAnalyzer, ExtractPairs_AccelerationVsCombined) {
    auto desired = makeLinePath(64);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::Acceleration;
    config.deviationAxis = DeviationAxis::CombinedError;
    config.useCertifiedContourError = false;
    KdeDerivativeAnalyzer analyzer(config);

    auto pairs = analyzer.extractPairs(desired, actual);

    EXPECT_EQ(pairs.derivatives.size(), 64u);
    EXPECT_EQ(pairs.deviations.size(), 64u);
}

//=============================================================================
// Bandwidth selection
//=============================================================================

TEST(KdeDerivativeAnalyzer, Bandwidth_Silverman) {
    std::vector<double> data;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 100; ++i) data.push_back(dist(rng));

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double h = analyzer.computeBandwidth(data, BandwidthMethod::Silverman, 0.0, 1.0);
    EXPECT_GT(h, 0.0);
    // Silverman: h = 1.06 * sigma * n^(-1/5), for n=100, sigma≈1: h ≈ 0.336
    EXPECT_NEAR(h, 0.336, 0.1);
}

TEST(KdeDerivativeAnalyzer, Bandwidth_Scott) {
    std::vector<double> data;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 100; ++i) data.push_back(dist(rng));

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double h = analyzer.computeBandwidth(data, BandwidthMethod::Scott, 0.0, 1.0);
    EXPECT_GT(h, 0.0);
    // Scott: h = sigma * n^(-1/5), for n=100, sigma≈1: h ≈ 0.317
    EXPECT_NEAR(h, 0.317, 0.1);
}

TEST(KdeDerivativeAnalyzer, Bandwidth_Fixed) {
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double h = analyzer.computeBandwidth(data, BandwidthMethod::Fixed, 0.5, 1.0);
    EXPECT_EQ(h, 0.5);
}

TEST(KdeDerivativeAnalyzer, Bandwidth_ScaleFactor) {
    std::vector<double> data;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 100; ++i) data.push_back(dist(rng));

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double h1 = analyzer.computeBandwidth(data, BandwidthMethod::Silverman, 0.0, 1.0);
    double h2 = analyzer.computeBandwidth(data, BandwidthMethod::Silverman, 0.0, 2.0);
    EXPECT_NEAR(h2, 2.0 * h1, 1e-10);
}

//=============================================================================
// Kernel functions
//=============================================================================

TEST(KdeDerivativeAnalyzer, Kernel_Gaussian_PeakAtZero) {
    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double k0 = analyzer.kernelValue(KernelType::Gaussian, 0.0);
    double k1 = analyzer.kernelValue(KernelType::Gaussian, 1.0);
    double k2 = analyzer.kernelValue(KernelType::Gaussian, 2.0);

    // Gaussian peak at u=0: 1/sqrt(2*pi) ≈ 0.399
    EXPECT_NEAR(k0, 1.0 / std::sqrt(2.0 * M_PI), 1e-6);
    EXPECT_GT(k0, k1);
    EXPECT_GT(k1, k2);
}

TEST(KdeDerivativeAnalyzer, Kernel_Epanechnikov_ZeroOutsideRange) {
    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double kIn = analyzer.kernelValue(KernelType::Epanechnikov, 0.5);
    double kOut = analyzer.kernelValue(KernelType::Epanechnikov, 1.5);

    EXPECT_GT(kIn, 0.0);
    EXPECT_EQ(kOut, 0.0);
}

TEST(KdeDerivativeAnalyzer, Kernel_Uniform_HalfInside) {
    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    double kIn = analyzer.kernelValue(KernelType::Uniform, 0.5);
    double kOut = analyzer.kernelValue(KernelType::Uniform, 1.5);

    EXPECT_NEAR(kIn, 0.5, 1e-10);
    EXPECT_EQ(kOut, 0.0);
}

TEST(KdeDerivativeAnalyzer, Kernel_AllTypes_PeakAtZero) {
    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    for (auto k : {KernelType::Gaussian, KernelType::Epanechnikov,
                   KernelType::Uniform, KernelType::Triangular,
                   KernelType::Quartic, KernelType::Cosine}) {
        double atZero = analyzer.kernelValue(k, 0.0);
        double atHalf = analyzer.kernelValue(k, 0.5);
        EXPECT_GE(atZero, atHalf)
            << "Kernel should peak at u=0";
    }
}

//=============================================================================
// KDE grid evaluation
//=============================================================================

TEST(KdeDerivativeAnalyzer, EvaluateKde_ProducesValidGrid) {
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> y = {0.1, 0.2, 0.3, 0.4, 0.5};

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    auto grid = analyzer.evaluateKde(x, y, 0.5, 0.1, 0.0, 6.0, 0.0, 0.6, 32, 32, KernelType::Gaussian);

    EXPECT_EQ(grid.xBins.size(), 32u);
    EXPECT_EQ(grid.yBins.size(), 32u);
    EXPECT_EQ(grid.density.size(), 32u * 32u);
    EXPECT_GT(grid.maxDensity(), 0.0);
}

TEST(KdeDerivativeAnalyzer, EvaluateKde_TotalMassApproximatelyOne) {
    // For a proper PDF, the total mass should be ≈ 1
    std::vector<double> x, y;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(5.0, 1.0);
    for (int i = 0; i < 200; ++i) {
        x.push_back(dist(rng));
        y.push_back(dist(rng));
    }

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    auto grid = analyzer.evaluateKde(x, y, 0.5, 0.5, 0.0, 10.0, 0.0, 10.0, 64, 64, KernelType::Gaussian);

    double mass = grid.totalMass();
    EXPECT_NEAR(mass, 1.0, 0.15);  // Allow 15% tolerance due to grid discretization
}

TEST(KdeDerivativeAnalyzer, EvaluateKde_ModeNearDataCenter) {
    // Generate data centered at (5, 3)
    std::vector<double> x, y;
    std::mt19937 rng(42);
    std::normal_distribution<double> distX(5.0, 0.5);
    std::normal_distribution<double> distY(3.0, 0.5);
    for (int i = 0; i < 500; ++i) {
        x.push_back(distX(rng));
        y.push_back(distY(rng));
    }

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    auto grid = analyzer.evaluateKde(x, y, 0.3, 0.3, 2.0, 8.0, 1.0, 5.0, 64, 64, KernelType::Gaussian);

    // Find the mode
    auto maxIt = std::max_element(grid.density.begin(), grid.density.end());
    std::size_t maxIdx = static_cast<std::size_t>(maxIt - grid.density.begin());
    std::size_t iy = maxIdx / grid.xBins.size();
    std::size_t ix = maxIdx % grid.xBins.size();

    EXPECT_NEAR(grid.xBins[ix], 5.0, 0.5);
    EXPECT_NEAR(grid.yBins[iy], 3.0, 0.5);
}

//=============================================================================
// Marginal statistics
//=============================================================================

TEST(KdeDerivativeAnalyzer, MarginalStats_NormalDistribution) {
    std::vector<double> data;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(5.0, 2.0);
    for (int i = 0; i < 1000; ++i) data.push_back(dist(rng));

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);
    auto stats = analyzer.computeMarginalStats(data);

    EXPECT_NEAR(stats.mean, 5.0, 0.2);
    EXPECT_NEAR(stats.stdDev, 2.0, 0.2);
    // Skewness of normal = 0
    EXPECT_NEAR(stats.skewness, 0.0, 0.15);
    // Excess kurtosis of normal = 0
    EXPECT_NEAR(stats.kurtosis, 0.0, 0.3);
    // Median ≈ mean for symmetric
    EXPECT_NEAR(stats.median, 5.0, 0.2);
}

//=============================================================================
// Conditional statistics
//=============================================================================

TEST(KdeDerivativeAnalyzer, ConditionalStats_VelocityDependentDeviation) {
    auto desired = makeLinePath(128);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::Velocity;
    config.deviationAxis = DeviationAxis::ContourError;
    config.useCertifiedContourError = false;
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);

    ASSERT_TRUE(eval.hasSufficientData);
    EXPECT_FALSE(eval.conditional.empty());

    // At least some conditional stats should be valid
    int validCount = 0;
    for (const auto& cs : eval.conditional) {
        if (cs.valid) ++validCount;
    }
    EXPECT_GT(validCount, 0);
}

//=============================================================================
// Dependence metrics
//=============================================================================

TEST(KdeDerivativeAnalyzer, Dependence_LinearRelationship_HighCorrelation) {
    // Create data with a strong linear relationship
    std::vector<double> x, y;
    for (int i = 0; i < 100; ++i) {
        double xi = static_cast<double>(i) * 0.1;
        x.push_back(xi);
        y.push_back(xi * 0.5 + 0.01);  // y = 0.5*x + small noise
    }

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    auto grid = analyzer.evaluateKde(x, y, 1.0, 0.1, 0.0, 10.0, 0.0, 5.0, 64, 64, KernelType::Gaussian);
    auto dep = analyzer.computeDependence(x, y, grid);

    // Strong linear relationship → high Pearson
    EXPECT_GT(std::abs(dep.pearson), 0.9);
    // Spearman should also be high
    EXPECT_GT(std::abs(dep.spearman), 0.9);
    // Kendall's tau should be high
    EXPECT_GT(std::abs(dep.kendall), 0.8);
    // Correlation ratio should be high
    EXPECT_GT(dep.correlationRatio, 0.7);
}

TEST(KdeDerivativeAnalyzer, Dependence_Independent_LowCorrelation) {
    // Create independent data
    std::vector<double> x, y;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 200; ++i) {
        x.push_back(dist(rng));
        y.push_back(dist(rng));
    }

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);

    auto grid = analyzer.evaluateKde(x, y, 0.5, 0.5, -3.0, 3.0, -3.0, 3.0, 64, 64, KernelType::Gaussian);
    auto dep = analyzer.computeDependence(x, y, grid);

    // Independent → low correlation
    EXPECT_LT(std::abs(dep.pearson), 0.3);
    EXPECT_LT(dep.correlationRatio, 0.2);
}

//=============================================================================
// Tail risk
//=============================================================================

TEST(KdeDerivativeAnalyzer, TailRisk_NormalData) {
    std::vector<double> data;
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 1000; ++i) data.push_back(std::abs(dist(rng)));

    KdeConfig config;
    KdeDerivativeAnalyzer analyzer(config);
    auto risk = analyzer.computeTailRisk(data, 0.95);

    // Tail fraction should be 5%
    EXPECT_NEAR(risk.tailFraction, 0.05, 0.02);
    // VaR95 should be around 1.645 (95th percentile of |N(0,1)|)
    EXPECT_GT(risk.var95, 1.0);
    // CVaR should be > VaR
    EXPECT_GE(risk.conditionalVar95, risk.var95);
}

//=============================================================================
// Threshold extraction
//=============================================================================

TEST(KdeDerivativeAnalyzer, Thresholds_VelocityDependent) {
    auto desired = makeLinePath(128);
    // Large deviation that scales with velocity
    auto actual = makeVelocityDependentActual(desired, 0.01);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::Velocity;
    config.deviationAxis = DeviationAxis::ContourError;
    config.useCertifiedContourError = false;
    config.tolerances = {0.05, 0.1, 0.5};
    KdeDerivativeAnalyzer analyzer(config);
    auto eval = analyzer.evaluate(desired, actual);

    ASSERT_TRUE(eval.hasSufficientData);
    EXPECT_EQ(eval.thresholds.size(), 3u);
}

//=============================================================================
// Utility functions
//=============================================================================

TEST(KdeDerivativeAnalyzer, ToString_DerivativeAxis) {
    EXPECT_EQ(toString(DerivativeAxis::Velocity), "Velocity");
    EXPECT_EQ(toString(DerivativeAxis::Acceleration), "Acceleration");
    EXPECT_EQ(toString(DerivativeAxis::Jerk), "Jerk");
    EXPECT_EQ(toString(DerivativeAxis::Curvature), "Curvature");
}

TEST(KdeDerivativeAnalyzer, ToString_DeviationAxis) {
    EXPECT_EQ(toString(DeviationAxis::ContourError), "ContourError");
    EXPECT_EQ(toString(DeviationAxis::LagError), "LagError");
    EXPECT_EQ(toString(DeviationAxis::CombinedError), "CombinedError");
}

TEST(KdeDerivativeAnalyzer, ToString_KernelType) {
    EXPECT_EQ(toString(KernelType::Gaussian), "Gaussian");
    EXPECT_EQ(toString(KernelType::Epanechnikov), "Epanechnikov");
    EXPECT_EQ(toString(KernelType::Uniform), "Uniform");
}

TEST(KdeDerivativeAnalyzer, ToString_BandwidthMethod) {
    EXPECT_EQ(toString(BandwidthMethod::Silverman), "Silverman");
    EXPECT_EQ(toString(BandwidthMethod::Scott), "Scott");
    EXPECT_EQ(toString(BandwidthMethod::ISJ), "ISJ");
}

TEST(KdeDerivativeAnalyzer, UnitString_DerivativeAxis) {
    EXPECT_EQ(unitString(DerivativeAxis::Velocity), "mm/s");
    EXPECT_EQ(unitString(DerivativeAxis::Acceleration), "mm/s^2");
    EXPECT_EQ(unitString(DerivativeAxis::Time), "s");
}

TEST(KdeDerivativeAnalyzer, UnitString_DeviationAxis) {
    EXPECT_EQ(unitString(DeviationAxis::ContourError), "mm");
    EXPECT_EQ(unitString(DeviationAxis::VelocityError), "mm/s");
}

//=============================================================================
// Colormap
//=============================================================================

TEST(KdeDerivativeAnalyzer, Colormap_Viridis_Endpoints) {
    auto [r0, g0, b0] = KdeDerivativeAnalyzer::colormapColor(KdeColormap::Viridis, 0.0);
    auto [r1, g1, b1] = KdeDerivativeAnalyzer::colormapColor(KdeColormap::Viridis, 1.0);

    // At 0: dark purple
    EXPECT_GE(r0, 50 && r0 <= 100);
    // At 1: bright yellow
    EXPECT_GT(r1, 200);
    EXPECT_GT(g1, 180);
}

TEST(KdeDerivativeAnalyzer, Colormap_Grayscale) {
    auto [r0, g0, b0] = KdeDerivativeAnalyzer::colormapColor(KdeColormap::Grayscale, 0.0);
    auto [r1, g1, b1] = KdeDerivativeAnalyzer::colormapColor(KdeColormap::Grayscale, 1.0);

    EXPECT_EQ(r0, g0);
    EXPECT_EQ(g0, b0);
    EXPECT_EQ(r1, g1);
    EXPECT_EQ(g1, b1);
    EXPECT_LT(r0, r1);
}

TEST(KdeDerivativeAnalyzer, Colormap_AllTypes_ValidRGB) {
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        for (auto cmap : {KdeColormap::Viridis, KdeColormap::Inferno, KdeColormap::Plasma,
                          KdeColormap::Magma, KdeColormap::Jet, KdeColormap::Hot,
                          KdeColormap::Cool, KdeColormap::Grayscale, KdeColormap::BlueRed}) {
            auto [r, g, b] = KdeDerivativeAnalyzer::colormapColor(cmap, t);
            EXPECT_GE(r, 0);
            EXPECT_LE(r, 255);
            EXPECT_GE(g, 0);
            EXPECT_LE(g, 255);
            EXPECT_GE(b, 0);
            EXPECT_LE(b, 255);
        }
    }
}

//=============================================================================
// Static helpers
//=============================================================================

TEST(KdeDerivativeAnalyzer, DensityMode_PeakAtCenter) {
    std::vector<double> bins = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<double> density = {0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.15, 0.08, 0.03, 0.01};

    double mode = KdeDerivativeAnalyzer::densityMode(bins, density);
    EXPECT_EQ(mode, 5.0); // Peak at index 5
}

TEST(KdeDerivativeAnalyzer, DensityQuantile_Median) {
    std::vector<double> bins = {0, 1, 2, 3, 4};
    std::vector<double> density = {0.1, 0.2, 0.4, 0.2, 0.1};

    double q50 = KdeDerivativeAnalyzer::densityQuantile(bins, density, 0.5);
    // CDF: 0.1, 0.3, 0.7, 0.9, 1.0
    // 50% falls between bin 1 (CDF 0.3) and bin 2 (CDF 0.7)
    // Interpolation: t = (0.5-0.3)/(0.7-0.3) = 0.5, so q50 = 1 + 0.5*(2-1) = 1.5
    EXPECT_NEAR(q50, 1.5, 0.1);
}

TEST(KdeDerivativeAnalyzer, DensityEntropy_Uniform_MaxEntropy) {
    std::vector<double> density = {0.25, 0.25, 0.25, 0.25};
    double h = KdeDerivativeAnalyzer::densityEntropy(density);
    // Max entropy for 4 bins = log2(4) = 2 bits
    EXPECT_NEAR(h, 2.0, 0.01);
}

TEST(KdeDerivativeAnalyzer, DensityEntropy_Peaked_LowEntropy) {
    std::vector<double> density = {0.97, 0.01, 0.01, 0.01};
    double h = KdeDerivativeAnalyzer::densityEntropy(density);
    // Peaked distribution → low entropy
    EXPECT_LT(h, 0.5);
}

//=============================================================================
// Full evaluation
//=============================================================================

TEST(KdeDerivativeAnalyzer, Evaluate_FullPipeline) {
    auto desired = makeLinePath(128);
    auto actual = makeVelocityDependentActual(desired, 0.001);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::Velocity;
    config.deviationAxis = DeviationAxis::ContourError;
    config.useCertifiedContourError = false;
    config.kernel = KernelType::Gaussian;
    config.bandwidthMethod = BandwidthMethod::Silverman;
    KdeDerivativeAnalyzer analyzer(config);

    auto eval = analyzer.evaluate(desired, actual);

    EXPECT_TRUE(eval.hasSufficientData);
    EXPECT_FALSE(eval.grid.density.empty());
    EXPECT_GT(eval.maxDensity, 0.0);
    EXPECT_FALSE(eval.summary.empty());
    EXPECT_FALSE(eval.conditional.empty());
}

TEST(KdeDerivativeAnalyzer, Evaluate_DifferentKernels) {
    auto desired = makeLinePath(64);
    auto actual = makeNoisyActual(desired, 0.05, 123);

    for (auto kernel : {KernelType::Gaussian, KernelType::Epanechnikov,
                        KernelType::Uniform, KernelType::Triangular,
                        KernelType::Quartic, KernelType::Cosine}) {
        KdeConfig config;
        config.derivativeAxis = DerivativeAxis::Velocity;
        config.deviationAxis = DeviationAxis::ContourError;
        config.useCertifiedContourError = false;
        config.kernel = kernel;
        KdeDerivativeAnalyzer analyzer(config);

        auto eval = analyzer.evaluate(desired, actual);
        EXPECT_TRUE(eval.hasSufficientData);
        EXPECT_GT(eval.maxDensity, 0.0);
    }
}

TEST(KdeDerivativeAnalyzer, Evaluate_DifferentBandwidthMethods) {
    auto desired = makeLinePath(128);
    auto actual = makeNoisyActual(desired, 0.05, 456);

    for (auto method : {BandwidthMethod::Silverman, BandwidthMethod::Scott,
                        BandwidthMethod::ISJ}) {
        KdeConfig config;
        config.derivativeAxis = DerivativeAxis::Velocity;
        config.deviationAxis = DeviationAxis::ContourError;
        config.useCertifiedContourError = false;
        config.bandwidthMethod = method;
        KdeDerivativeAnalyzer analyzer(config);

        auto eval = analyzer.evaluate(desired, actual);
        EXPECT_TRUE(eval.hasSufficientData);
        EXPECT_GT(eval.recommendedBandwidthX, 0.0);
        EXPECT_GT(eval.recommendedBandwidthY, 0.0);
    }
}

TEST(KdeDerivativeAnalyzer, Evaluate_ArcLengthVsTime) {
    auto desired = makeLinePath(64);
    auto actual = makeNoisyActual(desired, 0.05, 789);

    KdeConfig config;
    config.derivativeAxis = DerivativeAxis::ArcLength;
    config.deviationAxis = DeviationAxis::ContourError;
    config.useCertifiedContourError = false;
    KdeDerivativeAnalyzer analyzer(config);

    auto eval = analyzer.evaluate(desired, actual);

    EXPECT_TRUE(eval.hasSufficientData);
    // Arc length should range from 0 to ~100
    EXPECT_GT(eval.derivativeMarginal.max, 50.0);
}
