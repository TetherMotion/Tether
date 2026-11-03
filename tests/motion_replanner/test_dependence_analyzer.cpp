/**
 * @file test_dependence_analyzer.cpp
 * @brief Pure-statistics coverage for DependenceAnalyzer
 *        (src/replanner/DependenceAnalyzer.cpp).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tether/motion_replanner/DependenceAnalyzer.hpp"

using namespace tether::motion::replanner;

namespace {

/// Deterministic ramp 0..n-1.
std::vector<double> ramp(size_t n, double scale = 1.0) {
    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = static_cast<double>(i) * scale;
    return v;
}

/// Deterministic pseudo-noise (no rand() — reproducible).
std::vector<double> noise(size_t n) {
    std::vector<double> v(n);
    uint32_t s = 12345;
    for (size_t i = 0; i < n; ++i) {
        s = s * 1103515245 + 12345;
        v[i] = static_cast<double>((s >> 16) & 0x7FFF) / 16384.0 - 1.0;
    }
    return v;
}

/// Uniform 2x2 KdeGrid with all mass in the diagonal cells.
KdeGrid diagonalGrid() {
    KdeGrid g;
    g.xBins = {0.5, 1.5};
    g.yBins = {0.5, 1.5};
    g.density = {0.5, 0.0,
                 0.0, 0.5};
    g.bandwidthX = 1.0;
    g.bandwidthY = 1.0;
    g.sampleCount = 4;
    return g;
}

/// Flat uniform grid — zero mutual information.
KdeGrid uniformGrid() {
    KdeGrid g;
    g.xBins = {0.5, 1.5};
    g.yBins = {0.5, 1.5};
    g.density = {0.25, 0.25,
                 0.25, 0.25};
    g.bandwidthX = 1.0;
    g.bandwidthY = 1.0;
    g.sampleCount = 4;
    return g;
}

// ============================================================================
// Pearson
// ============================================================================

TEST(DependenceAnalyzerTest, PearsonPerfectPositive) {
    auto x = ramp(10);
    auto y = ramp(10, 2.5);
    EXPECT_NEAR(DependenceAnalyzer::pearson(x, y), 1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, PearsonPerfectNegative) {
    auto x = ramp(10);
    auto y = ramp(10, -1.0);
    EXPECT_NEAR(DependenceAnalyzer::pearson(x, y), -1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, PearsonUncorrelatedNearZero) {
    auto x = ramp(64);
    auto y = noise(64);
    EXPECT_LT(std::abs(DependenceAnalyzer::pearson(x, y)), 0.4);
}

TEST(DependenceAnalyzerTest, PearsonConstantInputIsZero) {
    std::vector<double> x(8, 3.0);
    auto y = ramp(8);
    EXPECT_DOUBLE_EQ(DependenceAnalyzer::pearson(x, y), 0.0);
}

TEST(DependenceAnalyzerTest, PearsonMismatchedOrEmptyIsHandled) {
    // Mismatched lengths are truncated to the shorter vector, not an
    // error — result stays a valid coefficient.
    std::vector<double> x{1, 2, 3};
    std::vector<double> y{1, 2};
    const double r = DependenceAnalyzer::pearson(x, y);
    EXPECT_GE(r, -1.0);
    EXPECT_LE(r, 1.0);
    EXPECT_DOUBLE_EQ(DependenceAnalyzer::pearson({}, {}), 0.0);
}

// ============================================================================
// Spearman
// ============================================================================

TEST(DependenceAnalyzerTest, SpearmanMonotonicNonlinearIsOne) {
    auto x = ramp(10);
    std::vector<double> y(10);
    for (size_t i = 0; i < 10; ++i) y[i] = x[i] * x[i] * x[i];
    EXPECT_NEAR(DependenceAnalyzer::spearmanRank(x, y), 1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, SpearmanTiesUseAverageRanks) {
    std::vector<double> x{1, 1, 2, 3};
    std::vector<double> y{1, 1, 2, 3};
    // Identical tied data must still correlate perfectly.
    EXPECT_NEAR(DependenceAnalyzer::spearmanRank(x, y), 1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, SpearmanReversedIsMinusOne) {
    auto x = ramp(8);
    auto y = ramp(8, -1.0);
    EXPECT_NEAR(DependenceAnalyzer::spearmanRank(x, y), -1.0, 1e-9);
}

// ============================================================================
// Kendall tau
// ============================================================================

TEST(DependenceAnalyzerTest, KendallPerfectConcordance) {
    auto x = ramp(10);
    auto y = ramp(10);
    EXPECT_NEAR(DependenceAnalyzer::kendallTau(x, y), 1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, KendallPerfectDiscordance) {
    auto x = ramp(10);
    auto y = ramp(10, -1.0);
    EXPECT_NEAR(DependenceAnalyzer::kendallTau(x, y), -1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, KendallBoundsUnitInterval) {
    auto x = noise(32);
    auto y = noise(32);
    const double tau = DependenceAnalyzer::kendallTau(x, y);
    EXPECT_GE(tau, -1.0);
    EXPECT_LE(tau, 1.0);
}

// ============================================================================
// Distance correlation
// ============================================================================

TEST(DependenceAnalyzerTest, DistanceCorrelationDependentIsPositive) {
    auto x = ramp(20);
    auto y = ramp(20, 3.0);
    EXPECT_NEAR(DependenceAnalyzer::distanceCorrelation(x, y), 1.0, 1e-9);
}

TEST(DependenceAnalyzerTest, DistanceCorrelationBounds) {
    auto x = noise(30);
    auto y = noise(30);
    const double d = DependenceAnalyzer::distanceCorrelation(x, y);
    EXPECT_GE(d, 0.0);
    EXPECT_LE(d, 1.0);
}

// ============================================================================
// Grid entropy / mutual information
// ============================================================================

TEST(DependenceAnalyzerTest, UniformGridHasZeroMI) {
    const double mi =
        DependenceAnalyzer::mutualInformationFromGrid(uniformGrid());
    EXPECT_NEAR(mi, 0.0, 1e-9);
}

TEST(DependenceAnalyzerTest, DiagonalGridHasPositiveMI) {
    const double mi =
        DependenceAnalyzer::mutualInformationFromGrid(diagonalGrid());
    EXPECT_GT(mi, 0.5);  // ~ln(2) nats for a perfect 2x2 diagonal
}

TEST(DependenceAnalyzerTest, EntropyFromGridSplitsJointAndConditional) {
    auto e = DependenceAnalyzer::entropyFromGrid(diagonalGrid());
    EXPECT_GT(e.joint, 0.0);
    EXPECT_GE(e.joint, e.conditional);
    EXPECT_GT(e.normalizedMI, 0.0);
    EXPECT_LE(e.normalizedMI, 1.0);
}

TEST(DependenceAnalyzerTest, EmptyGridGivesZeroEntropy) {
    KdeGrid g;
    auto e = DependenceAnalyzer::entropyFromGrid(g);
    EXPECT_DOUBLE_EQ(e.joint, 0.0);
    EXPECT_DOUBLE_EQ(e.conditional, 0.0);
}

// ============================================================================
// Correlation ratio
// ============================================================================

TEST(DependenceAnalyzerTest, CorrelationRatioSeparatedGroupsIsOne) {
    // Two groups at x=0 (y~0) and x=1 (y~10) — all variance is between
    // groups, so eta-squared -> 1.
    std::vector<double> y = {0, 0.01, -0.01, 10, 10.01, 9.99};
    std::vector<ConditionalStats> cond(2);
    cond[0].xValue = 0; cond[0].meanY = 0.0;  cond[0].mass = 0.5;
    cond[0].valid = true;
    cond[1].xValue = 1; cond[1].meanY = 10.0; cond[1].mass = 0.5;
    cond[1].valid = true;
    const double eta = DependenceAnalyzer::correlationRatio(y, cond);
    EXPECT_NEAR(eta, 1.0, 1e-3);
}

TEST(DependenceAnalyzerTest, CorrelationRatioEmptyIsZero) {
    auto y = ramp(8);
    EXPECT_DOUBLE_EQ(DependenceAnalyzer::correlationRatio(y, {}), 0.0);
}

// ============================================================================
// Composite
// ============================================================================

TEST(DependenceAnalyzerTest, ComputeAggregatesAllMetrics) {
    auto x = ramp(20);
    auto y = ramp(20);
    std::vector<ConditionalStats> cond(1);
    cond[0].xValue = 0; cond[0].meanY = 0; cond[0].mass = 1.0;
    cond[0].valid = true;
    auto m = DependenceAnalyzer::compute(x, y, diagonalGrid(), cond);
    EXPECT_NEAR(m.pearson, 1.0, 1e-9);
    EXPECT_NEAR(m.spearman, 1.0, 1e-9);
    EXPECT_GT(m.mutualInformation, 0.0);
    EXPECT_GT(m.distanceCorrelation, 0.0);
    EXPECT_GT(m.dependenceIndex, 0.0);
}

TEST(DependenceAnalyzerTest, ComputeEmptyInputsDoesNotCrash) {
    KdeGrid g;
    auto m = DependenceAnalyzer::compute({}, {}, g, {});
    EXPECT_DOUBLE_EQ(m.pearson, 0.0);
    EXPECT_DOUBLE_EQ(m.dependenceIndex, 0.0);
}

} // namespace
