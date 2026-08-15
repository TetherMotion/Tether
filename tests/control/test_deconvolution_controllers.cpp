/**
 * @file test_deconvolution_controllers.cpp
 * @brief Unit tests for LTI and LPV deconvolution controllers.
 *
 * Tests:
 *   - LTIFrequencyDomainDeconvolver: regularized spectral deconvolution
 *   - OverlapAddLPVDeconvolver: gain-scheduled overlap-add
 *   - ARXLPVInverseFilter: time-domain IIR inverse with delay
 *   - StateSpaceLPVInputEstimator: state-space input estimation
 */

#include <gtest/gtest.h>

#include "tether/control/extrusion/LTIFrequencyDomainDeconvolver.hpp"
#include "tether/control/extrusion/OverlapAddLPVDeconvolver.hpp"
#include "tether/control/extrusion/ARXLPVInverseFilter.hpp"
#include "tether/control/extrusion/StateSpaceLPVInputEstimator.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace tether::control::extrusion;

// ============================================================================
// Helper: generate a simple low-pass impulse response (exponential decay)
// ============================================================================
static std::vector<double> makeLowPassImpulseResponse(int len, double tau) {
    std::vector<double> h(len, 0.0);
    for (int i = 0; i < len; ++i) {
        h[i] = std::exp(-static_cast<double>(i) / tau) / tau;
    }
    // Normalize so the DC gain = 1 (sum = 1).
    double sum = 0.0;
    for (double v : h) sum += v;
    if (sum > 0) for (double& v : h) v /= sum;
    return h;
}

// ============================================================================
// Helper: time-domain convolution
// ============================================================================
static std::vector<double> convolve(const std::vector<double>& a,
                                    const std::vector<double>& b) {
    if (a.empty() || b.empty()) return {};
    std::vector<double> result(a.size() + b.size() - 1, 0.0);
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < b.size(); ++j)
            result[i + j] += a[i] * b[j];
    return result;
}

// ============================================================================
// LTIFrequencyDomainDeconvolver
// ============================================================================

TEST(LTIFrequencyDomainDeconvolver, IdentityImpulseResponseReturnsInput) {
    // h = [1, 0, 0, ...] → inverse is also identity → x_req = y_tgt.
    std::vector<double> h(16, 0.0);
    h[0] = 1.0;

    LTIDeconvolutionParams params;
    params.lambda = 1e-10;
    LTIFrequencyDomainDeconvolver deconv(params);
    deconv.setImpulseResponse(h);
    deconv.precomputeInverseFilter();

    std::vector<double> y_tgt = {1.0, 2.0, 3.0, 4.0, 5.0, 0.0, 0.0, 0.0};
    auto x_req = deconv.deconvolve(y_tgt);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    // With identity impulse response, x_req should ≈ y_tgt (shifted by 0).
    for (size_t i = 0; i < y_tgt.size(); ++i) {
        EXPECT_NEAR(x_req[i], y_tgt[i], 1e-3)
            << "Mismatch at index " << i;
    }
}

TEST(LTIFrequencyDomainDeconvolver, RecoversInputFromLowPassOutput) {
    // Create a known input, convolve with a low-pass h to get y,
    // then deconvolve y to recover x.
    auto h = makeLowPassImpulseResponse(32, 5.0);

    std::vector<double> x(64, 0.0);
    for (int i = 10; i < 30; ++i) x[i] = 1.0;  // step input

    auto y = convolve(x, h);
    y.resize(64);  // truncate to same length as x

    LTIDeconvolutionParams params;
    params.lambda = 1e-6;
    LTIFrequencyDomainDeconvolver deconv(params);
    auto x_req = deconv.deconvolve(y, h);

    ASSERT_EQ(x_req.size(), x.size());
    // The recovered input should be close to the original in the middle
    // of the step (edges are harder due to regularization).
    for (int i = 15; i < 25; ++i) {
        EXPECT_NEAR(x_req[i], x[i], 0.2)
            << "Mismatch at index " << i;
    }
}

TEST(LTIFrequencyDomainDeconvolver, LargerLambdaProducesSmootherResult) {
    auto h = makeLowPassImpulseResponse(32, 5.0);
    std::vector<double> x(64, 0.0);
    for (int i = 10; i < 40; ++i) x[i] = 1.0;
    auto y = convolve(x, h);
    y.resize(64);

    LTIDeconvolutionParams p1;
    p1.lambda = 1e-8;
    LTIFrequencyDomainDeconvolver deconv1(p1);
    auto x1 = deconv1.deconvolve(y, h);

    LTIDeconvolutionParams p2;
    p2.lambda = 1e-2;
    LTIFrequencyDomainDeconvolver deconv2(p2);
    auto x2 = deconv2.deconvolve(y, h);

    // Compute roughness: sum of |second differences|.
    auto roughness = [](const std::vector<double>& v) {
        double r = 0.0;
        for (size_t i = 1; i + 1 < v.size(); ++i)
            r += std::abs(v[i + 1] - 2 * v[i] + v[i - 1]);
        return r;
    };

    // Larger lambda → smoother (less rough) result.
    EXPECT_LT(roughness(x2), roughness(x1));
}

TEST(LTIFrequencyDomainDeconvolver, GroupDelayIsPeakIndex) {
    std::vector<double> h(32, 0.0);
    h[5] = 1.0;  // peak at index 5
    LTIFrequencyDomainDeconvolver deconv;
    deconv.setImpulseResponse(h);
    EXPECT_EQ(deconv.groupDelay(), 5);
}

TEST(LTIFrequencyDomainDeconvolver, EmptyInputReturnsEmpty) {
    LTIFrequencyDomainDeconvolver deconv;
    deconv.setImpulseResponse({1.0, 0.0, 0.0});
    deconv.precomputeInverseFilter();
    auto result = deconv.deconvolve({});
    EXPECT_TRUE(result.empty());
}

TEST(LTIFrequencyDomainDeconvolver, SetLambdaRecomputesInverse) {
    std::vector<double> h = makeLowPassImpulseResponse(16, 3.0);
    LTIFrequencyDomainDeconvolver deconv;
    deconv.setImpulseResponse(h);
    deconv.setLambda(1e-4);
    EXPECT_FALSE(deconv.inverseFilter().empty());
}

// ============================================================================
// OverlapAddLPVDeconvolver
// ============================================================================

TEST(OverlapAddLPVDeconvolver, SingleOperatingPointMatchesLTI) {
    // With a single operating point, the overlap-add result should
    // approximate the LTI deconvolution (with some windowing artifacts).
    auto h = makeLowPassImpulseResponse(32, 5.0);

    OverlapAddLPVParams params;
    params.blockSize = 128;
    params.overlapRatio = 0.5;
    params.lambda = 1e-6;
    OverlapAddLPVDeconvolver lpv(params);
    lpv.addOperatingPoint(50.0, h);

    EXPECT_EQ(lpv.numOperatingPoints(), 1u);

    std::vector<double> y_tgt(128, 0.0);
    for (int i = 20; i < 80; ++i) y_tgt[i] = 1.0;
    std::vector<double> p(128, 50.0);

    auto x_req = lpv.deconvolve(y_tgt, p);
    ASSERT_EQ(x_req.size(), y_tgt.size());

    // In the middle of the step, the result should be positive.
    EXPECT_GT(x_req[50], 0.0);
}

TEST(OverlapAddLPVDeconvolver, InterpolatesBetweenOperatingPoints) {
    // Two operating points with different impulse responses.
    auto h1 = makeLowPassImpulseResponse(32, 3.0);  // faster
    auto h2 = makeLowPassImpulseResponse(32, 8.0);  // slower

    OverlapAddLPVParams params;
    params.blockSize = 64;
    params.overlapRatio = 0.5;
    params.lambda = 1e-6;
    OverlapAddLPVDeconvolver lpv(params);
    lpv.addOperatingPoint(20.0, h1);
    lpv.addOperatingPoint(100.0, h2);

    EXPECT_EQ(lpv.numOperatingPoints(), 2u);

    // Trajectory with varying scheduling parameter.
    std::vector<double> y_tgt(128, 0.0);
    for (int i = 20; i < 100; ++i) y_tgt[i] = 1.0;
    std::vector<double> p(128);
    for (int i = 0; i < 128; ++i) p[i] = 20.0 + 80.0 * i / 127.0;

    auto x_req = lpv.deconvolve(y_tgt, p);
    ASSERT_EQ(x_req.size(), y_tgt.size());
    // Result should be non-trivial (not all zeros).
    bool hasNonZero = false;
    for (double v : x_req) if (std::abs(v) > 1e-6) hasNonZero = true;
    EXPECT_TRUE(hasNonZero);
}

TEST(OverlapAddLPVDeconvolver, EmptyLutReturnsEmpty) {
    OverlapAddLPVDeconvolver lpv;
    std::vector<double> y_tgt = {1.0, 2.0, 3.0};
    std::vector<double> p = {1.0, 1.0, 1.0};
    auto result = lpv.deconvolve(y_tgt, p);
    EXPECT_TRUE(result.empty());
}

TEST(OverlapAddLPVDeconvolver, MismatchedSizesReturnsEmpty) {
    OverlapAddLPVDeconvolver lpv;
    lpv.addInverseFilter(1.0, {1.0, 0.5, 0.25});
    std::vector<double> y_tgt = {1.0, 2.0, 3.0};
    std::vector<double> p = {1.0, 1.0};  // wrong size
    auto result = lpv.deconvolve(y_tgt, p);
    EXPECT_TRUE(result.empty());
}

TEST(OverlapAddLPVDeconvolver, ResetClearsLut) {
    OverlapAddLPVDeconvolver lpv;
    lpv.addInverseFilter(1.0, {1.0, 0.5});
    EXPECT_EQ(lpv.numOperatingPoints(), 1u);
    lpv.reset();
    EXPECT_EQ(lpv.numOperatingPoints(), 0u);
}

// ============================================================================
// ARXLPVInverseFilter
// ============================================================================

TEST(ARXLPVInverseFilter, FirstOrderSystemInversion) {
    // Simple first-order system: y[n] = b0 * x[n] - a1 * y[n-1]
    // With a1 = -0.5, b0 = 0.5:
    //   y[n] = 0.5 * x[n] + 0.5 * y[n-1]
    // Inverse: x[n] = (y[n] - 0.5 * y[n-1]) / 0.5 = 2*y[n] - y[n-1]

    ARXLPVInverseFilter filter(1, 0);  // na=1, nb=0
    // aCoeffs = [a1] = [-0.5] (A(z) = 1 + a1*z^-1 = 1 - 0.5*z^-1)
    // bCoeffs = [b0] = [0.5]
    filter.addModelPoint(0.0, {-0.5}, {0.5}, 0);

    // Target: y_tgt = [1, 1, 1, 1, 1]
    // Expected: x_req = [2*1 - 0, 2*1 - 1, 2*1 - 1, ...] = [2, 1, 1, 1, 1]
    std::vector<double> y_tgt = {1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> p = {0.0, 0.0, 0.0, 0.0, 0.0};
    auto x_req = filter.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    EXPECT_NEAR(x_req[0], 2.0, 1e-9);
    EXPECT_NEAR(x_req[1], 1.0, 1e-9);
    EXPECT_NEAR(x_req[2], 1.0, 1e-9);
    EXPECT_NEAR(x_req[3], 1.0, 1e-9);
    EXPECT_NEAR(x_req[4], 1.0, 1e-9);
}

TEST(ARXLPVInverseFilter, DelayCompensation) {
    // System with delay d=1: y[n] = b0 * x[n-1] - a1 * y[n-1]
    // A(z) = 1 + a1*z^-1, B(z) = z^-1 * b0
    // Forward: y[n] = 0.7*x[n-1] - (-0.3)*y[n-1] = 0.7*x[n-1] + 0.3*y[n-1]
    // Inverse: x[n] = (y[n+1] + a1*y[n]) / b0 = (y[n+1] - 0.3*y[n]) / 0.7

    ARXLPVInverseFilter filter(1, 0);  // na=1, nb=0
    filter.addModelPoint(0.0, {-0.3}, {0.7}, 1);  // delay=1

    // y_tgt = [1, 1, 1, 1, 1]
    // x[0] = (y[1] + (-0.3)*y[0]) / 0.7 = (1 - 0.3) / 0.7 = 1.0
    // x[1] = (y[2] + (-0.3)*y[1]) / 0.7 = (1 - 0.3) / 0.7 = 1.0
    std::vector<double> y_tgt = {1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> p = {0.0, 0.0, 0.0, 0.0, 0.0};
    auto x_req = filter.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    double expected = (1.0 - 0.3) / 0.7;  // = 1.0
    EXPECT_NEAR(x_req[0], expected, 1e-9);
    EXPECT_NEAR(x_req[1], expected, 1e-9);
}

TEST(ARXLPVInverseFilter, ParameterInterpolation) {
    // Two operating points with different b0.
    ARXLPVInverseFilter filter(0, 0);  // na=0, nb=0
    filter.addModelPoint(10.0, {}, {1.0}, 0);   // x = y / 1.0
    filter.addModelPoint(20.0, {}, {2.0}, 0);   // x = y / 2.0

    // At p=15 (midpoint), b0 should be 1.5.
    // x = y / 1.5
    std::vector<double> y_tgt = {3.0, 3.0, 3.0};
    std::vector<double> p = {15.0, 15.0, 15.0};
    auto x_req = filter.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    EXPECT_NEAR(x_req[0], 3.0 / 1.5, 1e-9);
    EXPECT_NEAR(x_req[1], 3.0 / 1.5, 1e-9);
}

TEST(ARXLPVInverseFilter, ResetClearsState) {
    ARXLPVInverseFilter filter(1, 0);
    filter.addModelPoint(0.0, {-0.5}, {0.5}, 0);

    // Process some samples to build up state.
    filter.process({1.0, 1.0, 1.0}, {0.0, 0.0, 0.0});
    filter.reset();

    // After reset, processing the same input should give the same result
    // as the first time.
    auto x1 = filter.process({1.0, 1.0}, {0.0, 0.0});
    filter.reset();
    auto x2 = filter.process({1.0, 1.0}, {0.0, 0.0});
    ASSERT_EQ(x1.size(), x2.size());
    for (size_t i = 0; i < x1.size(); ++i)
        EXPECT_NEAR(x1[i], x2[i], 1e-12);
}

TEST(ARXLPVInverseFilter, EmptyLutReturnsZero) {
    ARXLPVInverseFilter filter(1, 1);
    auto result = filter.process({1.0, 2.0}, {0.0, 0.0});
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0], 0.0);
}

// ============================================================================
// StateSpaceLPVInputEstimator
// ============================================================================

TEST(StateSpaceLPVInputEstimator, FirstOrderSystem) {
    // First-order system: v[n+1] = a*v[n] + b*x[n], y[n] = c*v[n]
    // With a=0.5, b=1.0, c=1.0:
    //   v[n+1] = 0.5*v[n] + 1.0*x[n]
    //   y[n+1] = v[n+1] = 0.5*v[n] + x[n]
    // Inverse: x[n] = y[n+1] - 0.5*v[n]
    // At n=0 with v[0]=0: x[0] = y[1]

    StateSpaceLPVInputEstimator estimator(1, 1, 1);
    Eigen::MatrixXd A(1, 1), B(1, 1), C(1, 1);
    A << 0.5; B << 1.0; C << 1.0;
    estimator.addModelPoint({0.0, A, B, C});

    // Target: y_tgt = [1, 1, 1, 1]
    // x[0] = y[1] - 0.5*v[0] = 1 - 0 = 1
    // v[1] = 0.5*0 + 1*1 = 1
    // x[1] = y[2] - 0.5*v[1] = 1 - 0.5 = 0.5
    // v[2] = 0.5*1 + 1*0.5 = 1
    // x[2] = y[3] - 0.5*v[2] = 1 - 0.5 = 0.5
    std::vector<double> y_tgt = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> p = {0.0, 0.0, 0.0, 0.0};
    auto x_req = estimator.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    // Tikhonov regularization (lambda=1e-8) introduces a small bias,
    // so we use 1e-6 tolerance.
    EXPECT_NEAR(x_req[0], 1.0, 1e-6);
    EXPECT_NEAR(x_req[1], 0.5, 1e-6);
    EXPECT_NEAR(x_req[2], 0.5, 1e-6);
}

TEST(StateSpaceLPVInputEstimator, TikhonovRegularizationStabilizes) {
    // System where C*B is very small (near-singular).
    // Without regularization, the inverse would amplify noise.
    StateSpaceLPVParams params;
    params.lambda = 1e-4;
    StateSpaceLPVInputEstimator estimator(1, 1, 1, params);
    Eigen::MatrixXd A(1, 1), B(1, 1), C(1, 1);
    A << 0.9; B << 1e-6; C << 1.0;
    estimator.addModelPoint({0.0, A, B, C});

    // With C*B = 1e-6 and lambda = 1e-4:
    // x = (1e-6 * b) / (1e-12 + 1e-4) ≈ 1e-6 * b / 1e-4 = 0.01 * b
    // Without regularization: x = b / 1e-6 = 1e6 * b (explosive)
    double yNext = 1.0;
    double x = estimator.process(yNext, 0.0, 0.0);
    // Should be bounded, not explosive.
    EXPECT_LT(std::abs(x), 100.0);
    EXPECT_GT(std::abs(x), 0.0);
}

TEST(StateSpaceLPVInputEstimator, ParameterInterpolation) {
    // Two operating points with different B matrices.
    StateSpaceLPVInputEstimator estimator(1, 1, 1);

    Eigen::MatrixXd A(1, 1), B1(1, 1), B2(1, 1), C(1, 1);
    A << 0.5; B1 << 1.0; B2 << 2.0; C << 1.0;
    estimator.addModelPoint({10.0, A, B1, C});
    estimator.addModelPoint({20.0, A, B2, C});

    // At p=15 (midpoint), B should be 1.5.
    // x[0] = y[1] - 0.5*0 = y[1], with C*B = 1.5
    // x = (1.5 * b) / (1.5^2 + lambda) ≈ b / 1.5 (for small lambda)
    std::vector<double> y_tgt = {1.0, 1.0, 1.0};
    std::vector<double> p = {15.0, 15.0, 15.0};
    auto x_req = estimator.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    // x[0] = 1 / 1.5 ≈ 0.667
    EXPECT_NEAR(x_req[0], 1.0 / 1.5, 0.01);
}

TEST(StateSpaceLPVInputEstimator, ResetClearsState) {
    StateSpaceLPVInputEstimator estimator(2, 1, 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd B(2, 1); B << 1.0, 0.0;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;
    estimator.addModelPoint({0.0, A, B, C});

    // Run some steps to build up state.
    estimator.process({1.0, 1.0, 1.0}, {0.0, 0.0, 0.0});
    EXPECT_GT(estimator.state().norm(), 0.0);

    estimator.reset();
    EXPECT_EQ(estimator.state().norm(), 0.0);
}

TEST(StateSpaceLPVInputEstimator, EmptyLutReturnsZero) {
    StateSpaceLPVInputEstimator estimator(1, 1, 1);
    auto result = estimator.process({1.0, 2.0}, {0.0, 0.0});
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0], 0.0);
}

TEST(StateSpaceLPVInputEstimator, SecondOrderSystem) {
    // Second-order system with two states.
    // A = [[0.8, 0.1], [0, 0.9]], B = [[1], [0]], C = [[1, 0]]
    StateSpaceLPVInputEstimator estimator(2, 1, 1);
    Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2);
    A << 0.8, 0.1, 0.0, 0.9;
    B << 1.0, 0.0;
    C << 1.0, 0.0;
    estimator.addModelPoint({0.0, A, B, C});

    // Constant target y = 1.
    // At steady state: v = (I-A)^{-1} B x, y = C v = 1
    // x_ss = 1 / (C (I-A)^{-1} B)
    std::vector<double> y_tgt(10, 1.0);
    std::vector<double> p(10, 0.0);
    auto x_req = estimator.process(y_tgt, p);

    ASSERT_EQ(x_req.size(), y_tgt.size());
    // The input should converge toward the steady-state value.
    // Steady state: (I-A)^{-1} = [[5, 5], [0, 10]], C*(I-A)^{-1}*B = 5
    // x_ss = 1/5 = 0.2
    // Check the last few samples are close to steady state.
    EXPECT_NEAR(x_req[8], 0.2, 0.1);
    EXPECT_NEAR(x_req[9], 0.2, 0.1);
}
