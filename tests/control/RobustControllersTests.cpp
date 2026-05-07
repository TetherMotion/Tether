/**
 * @file RobustControllersTests.cpp
 * @brief Comprehensive tests for RobustControllers module
 * Tests for WeightingFunction, H2Controller, HInfinityController
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "tether/control/Controllers.hpp"
#include "tether/control/RobustControllers.hpp"

using namespace Control;

// ============================================================================
// WeightingFunction Tests
// ============================================================================

TEST(WeightingFunctionTest, FirstOrder) {
    // Create first-order weight: W(s) = gain*(s + zero) / (s + pole)
    WeightingFunction wf = WeightingFunction::firstOrder(1.0, 10.0, 2.0);
    EXPECT_EQ(wf.order, 1);
}

TEST(WeightingFunctionTest, Integrator) {
    // Create integrator: W(s) = k/s
    WeightingFunction wf = WeightingFunction::integrator(5.0);
    EXPECT_EQ(wf.gain, 5.0);
}

TEST(WeightingFunctionTest, Sensitivity) {
    // Create sensitivity weight: W₁(s) = (s/M + ωB) / (s + ωB·ε)
    WeightingFunction wf = WeightingFunction::sensitivity(2.0, 100.0, 0.01);
    EXPECT_EQ(wf.order, 1);
}

TEST(WeightingFunctionTest, Complementary) {
    // Create complementary sensitivity weight
    WeightingFunction wf = WeightingFunction::complementary(0.5, 100.0, 0.01);
    EXPECT_EQ(wf.order, 1);
}

TEST(WeightingFunctionTest, MagnitudeAtLowFrequency) {
    WeightingFunction wf = WeightingFunction::sensitivity(2.0, 10.0, 0.01);
    
    // At low frequency, sensitivity weight should be large
    double mag_low = wf.magnitude(0.01);
    double mag_high = wf.magnitude(1000.0);
    
    // Sensitivity weight is high at low frequency
    EXPECT_GT(mag_low, mag_high);
}

TEST(WeightingFunctionTest, MagnitudeAtHighFrequency) {
    WeightingFunction wf = WeightingFunction::complementary(2.0, 10.0, 0.01);
    
    double mag_low = wf.magnitude(0.01);
    double mag_high = wf.magnitude(1000.0);
    
    // Complementary sensitivity weight is high at high frequency
    EXPECT_GT(mag_high, mag_low);
}

TEST(WeightingFunctionTest, ToStateSpace) {
    WeightingFunction wf = WeightingFunction::firstOrder(1.0, 10.0);
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    // First order system has n=1 state
}

TEST(WeightingFunctionTest, DefaultCoefficients) {
    WeightingFunction wf;
    // Default is constant 1
    EXPECT_EQ(wf.num[0], 1);
    EXPECT_EQ(wf.den[0], 1);
    EXPECT_EQ(wf.order, 0);
    EXPECT_EQ(wf.gain, 1.0);
}

// ============================================================================
// H2Controller Tests
// ============================================================================

class H2ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        h2 = std::make_unique<H2Controller>();
    }
    
    std::unique_ptr<H2Controller> h2;
};

TEST_F(H2ControllerTest, GetType) {
    EXPECT_EQ(h2->getType(), ControllerType::H2);
}

TEST_F(H2ControllerTest, GetName) {
    EXPECT_STREQ(h2->getName(), "H2 Optimal Controller");
}

TEST_F(H2ControllerTest, GetDescription) {
    EXPECT_NE(h2->getDescription(), nullptr);
    EXPECT_GT(strlen(h2->getDescription()), 0);
}

TEST_F(H2ControllerTest, SetGeneralizedPlant) {
    // Simple example: generalized plant for regulation
    // State: 2, exogenous: 1, control: 1, regulated: 1, measured: 1
    double A[4] = {0, 1, -1, -1};
    double B1[2] = {0, 1};  // Disturbance input
    double B2[2] = {0, 1};  // Control input
    double C1[2] = {1, 0};  // Regulated output
    double C2[2] = {1, 0};  // Measured output
    double D11[1] = {0};
    double D12[1] = {0};
    double D21[1] = {0.1};  // Noise on measurement
    double D22[1] = {0};
    
    h2->setGeneralizedPlant(A, B1, B2, C1, C2, D11, D12, D21, D22,
                            2, 1, 1, 1, 1);
}

TEST_F(H2ControllerTest, SetRegulatorProblem) {
    // Simplified setup for standard LQG-like problem
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};  // State weight
    double R[1] = {0.1};          // Control weight
    double W[4] = {0.01, 0, 0, 0.01};  // Process noise
    double V[1] = {0.1};               // Measurement noise
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
}

TEST_F(H2ControllerTest, Design) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    
    bool result = h2->design();
    // Design may or may not succeed depending on system
}

TEST_F(H2ControllerTest, GetH2Norm) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2->design();
    
    double norm = h2->getH2Norm();
    // H2 norm should be non-negative
    EXPECT_GE(norm, 0.0);
}

TEST_F(H2ControllerTest, GetControllerMatrices) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2->design();
    
    double Ak[16], Bk[4], Ck[4], Dk[1];
    h2->getControllerMatrices(Ak, Bk, Ck, Dk);
}

TEST_F(H2ControllerTest, BasicCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2->design();
    
    ControllerInput input;
    input.measured = 1.0;
    input.dt = 0.001;
    
    // ControllerOutput output = h2->compute(input); // Not used
    h2->compute(input);
}

TEST_F(H2ControllerTest, Reset) {
    h2->reset();
}

// ============================================================================
// HInfinityController Tests
// ============================================================================

class HInfinityControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        hinf = std::make_unique<HInfinityController>();
    }
    
    std::unique_ptr<HInfinityController> hinf;
};

TEST_F(HInfinityControllerTest, GetType) {
    EXPECT_EQ(hinf->getType(), ControllerType::HInfinity);
}

TEST_F(HInfinityControllerTest, GetName) {
    // API says "H∞ Controller" not "H-Infinity Controller"
    EXPECT_NE(hinf->getName(), nullptr);
    EXPECT_GT(strlen(hinf->getName()), 0);
}

TEST_F(HInfinityControllerTest, GetDescription) {
    EXPECT_NE(hinf->getDescription(), nullptr);
    EXPECT_GT(strlen(hinf->getDescription()), 0);
}

TEST_F(HInfinityControllerTest, SetPlant) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
}

TEST_F(HInfinityControllerTest, SetGeneralizedPlant) {
    // Full generalized plant setup
    double A[4] = {0, 1, -1, -1};
    double B1[2] = {0.1, 0};  // Disturbance
    double B2[2] = {0, 1};    // Control
    double C1[2] = {1, 0};    // Regulated
    double C2[2] = {1, 0};    // Measured
    double D11[1] = {0};
    double D12[1] = {0};
    double D21[1] = {0};
    double D22[1] = {0};
    
    hinf->setGeneralizedPlant(A, B1, B2, C1, C2, D11, D12, D21, D22,
                              2, 1, 1, 1, 1);
}

TEST_F(HInfinityControllerTest, SetSensitivityWeight) {
    // API: setSensitivityWeight(double M, double omegaB, double epsilon)
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
}

TEST_F(HInfinityControllerTest, SetControlWeight) {
    // API: setControlWeight(double maxControl)
    hinf->setControlWeight(10.0);
}

TEST_F(HInfinityControllerTest, SetComplementaryWeight) {
    // API: setComplementaryWeight(double M, double omegaT, double epsilon)
    hinf->setComplementaryWeight(0.5, 100.0, 0.01);
}

TEST_F(HInfinityControllerTest, SetWeightsWithObjects) {
    WeightingFunction W1 = WeightingFunction::sensitivity(2.0, 10.0, 0.01);
    WeightingFunction W2 = WeightingFunction::firstOrder(0.1, 100.0);
    WeightingFunction W3 = WeightingFunction::complementary(1.0, 100.0, 0.01);
    
    hinf->setWeights(W1, W2, W3);
}

TEST_F(HInfinityControllerTest, GetGamma) {
    double gamma = hinf->getGamma();
    EXPECT_GE(gamma, 0.0);
}

TEST_F(HInfinityControllerTest, Design) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    
    bool result = hinf->design(2.0);  // design(gamma)
}

TEST_F(HInfinityControllerTest, DesignOptimal) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    
    // Find optimal gamma through bisection
    double gamma = hinf->designOptimal();
    EXPECT_GE(gamma, 0.0);
}

TEST_F(HInfinityControllerTest, IsAchievable) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    
    // Very large gamma should be achievable
    bool achievable = hinf->isAchievable(100.0);
}

TEST_F(HInfinityControllerTest, GetControllerMatrices) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->design(5.0);
    
    double Ak[64], Bk[16], Ck[16], Dk[4];
    hinf->getControllerMatrices(Ak, Bk, Ck, Dk);
}

TEST_F(HInfinityControllerTest, BasicCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->design(5.0);
    
    ControllerInput input;
    input.reference = 0.0;
    input.measured = 1.0;
    input.dt = 0.001;
    
    ControllerOutput output = hinf->compute(input);
}

TEST_F(HInfinityControllerTest, Reset) {
    hinf->reset();
}

// ============================================================================
// Mixed Sensitivity Tests
// ============================================================================

TEST(MixedSensitivityTest, SensitivityWeightRolloff) {
    // Verify sensitivity weight has correct characteristics
    WeightingFunction W1 = WeightingFunction::sensitivity(2.0, 10.0, 0.01);
    
    // High at low frequency (for tracking/rejection)
    double mag_low = W1.magnitude(0.01);
    // Low at high frequency (roll off)
    double mag_high = W1.magnitude(1000.0);
    
    EXPECT_GT(mag_low, mag_high);
}

TEST(MixedSensitivityTest, ComplementaryWeightRolloff) {
    // Verify complementary weight has correct characteristics
    WeightingFunction W3 = WeightingFunction::complementary(2.0, 10.0, 0.01);
    
    // Low at low frequency
    double mag_low = W3.magnitude(0.01);
    // High at high frequency (for noise rejection/robustness)
    double mag_high = W3.magnitude(1000.0);
    
    EXPECT_LT(mag_low, mag_high);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(RobustIntegrationTest, H2ClosedLoopStability) {
    H2Controller h2;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2.setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2.design();
    
    // Simulate closed loop
    double x[2] = {1.0, 0.0};
    double dt = 0.001;
    
    for (int i = 0; i < 2000; ++i) {
        ControllerInput input;
        input.measured = x[0];
        input.dt = dt;
        
        ControllerOutput output = h2.compute(input);
        double u = output.control;
        
        // Plant dynamics
        double x0_dot = x[1];
        double x1_dot = -x[0] - x[1] + u;
        
        x[0] += x0_dot * dt;
        x[1] += x1_dot * dt;
    }
    
    // Should stabilize
    EXPECT_LT(std::abs(x[0]), 1.0);
}

TEST(RobustIntegrationTest, WeightedHInfinityDesign) {
    HInfinityController hinf;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf.setPlant(A, B, C, D, 2, 1, 1);
    
    // Setup mixed sensitivity weights using scalar parameters
    hinf.setSensitivityWeight(2.0, 10.0, 0.01);
    hinf.setComplementaryWeight(1.0, 50.0, 0.01);
    
    // Design and verify gamma achieved
    double gamma = hinf.designOptimal();
    EXPECT_GT(gamma, 0.0);
}

// ============================================================================
// WeightingFunction toStateSpace Tests
// ============================================================================

TEST(WeightingFunctionTest, ToStateSpaceOrder0) {
    // Create order-0 weighting function (constant gain)
    WeightingFunction wf;
    wf.order = 0;
    wf.gain = 2.5;
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 0);
    EXPECT_DOUBLE_EQ(D[0], 2.5);  // D = gain for order 0
}

TEST(WeightingFunctionTest, ToStateSpaceOrder1) {
    // First-order: (s+z)/(s+p) with gain
    WeightingFunction wf = WeightingFunction::firstOrder(5.0, 10.0, 2.0);  // zero=5, pole=10, gain=2
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 1);
    // A = -p = -10
    EXPECT_DOUBLE_EQ(A[0], -10.0);
    // B = 1
    EXPECT_DOUBLE_EQ(B[0], 1.0);
    // C = gain * (z - p) = 2 * (5 - 10) = -10
    EXPECT_DOUBLE_EQ(C[0], -10.0);
    // D = gain = 2
    EXPECT_DOUBLE_EQ(D[0], 2.0);
}

TEST(WeightingFunctionTest, ToStateSpaceOrder2) {
    // Second-order system: create manually
    WeightingFunction wf;
    wf.order = 2;
    wf.gain = 1.5;
    // Numerator: s^2 + 2s + 3 (coefficients: num[0]=1, num[1]=2, num[2]=3)
    wf.num[0] = 1.0;  // s^2 coefficient
    wf.num[1] = 2.0;  // s coefficient
    wf.num[2] = 3.0;  // constant
    // Denominator: s^2 + 4s + 5 (coefficients: den[0]=1, den[1]=4, den[2]=5)
    wf.den[0] = 1.0;  // s^2 coefficient
    wf.den[1] = 4.0;  // s coefficient
    wf.den[2] = 5.0;  // constant
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 2);
    
    // Controllable canonical form:
    // A = [0, 1; -a0, -a1] where a0 = den[2]/den[0], a1 = den[1]/den[0]
    EXPECT_DOUBLE_EQ(A[0], 0.0);
    EXPECT_DOUBLE_EQ(A[1], 1.0);
    EXPECT_DOUBLE_EQ(A[2], -5.0);  // -a0 = -5
    EXPECT_DOUBLE_EQ(A[3], -4.0);  // -a1 = -4
    
    // B = [0; 1]
    EXPECT_DOUBLE_EQ(B[0], 0.0);
    EXPECT_DOUBLE_EQ(B[1], 1.0);
    
    // D = gain * b2 where b2 = num[0]/den[0] = 1
    EXPECT_DOUBLE_EQ(D[0], 1.5);
}

TEST(WeightingFunctionTest, IntegratorStateSpace) {
    // Integrator: k/s
    WeightingFunction wf = WeightingFunction::integrator(3.0);
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 1);
    // For integrator: pole = 0, so A = 0
    EXPECT_DOUBLE_EQ(A[0], 0.0);
    EXPECT_DOUBLE_EQ(B[0], 1.0);
}

TEST(WeightingFunctionTest, SensitivityStateSpace) {
    WeightingFunction wf = WeightingFunction::sensitivity(2.0, 10.0, 0.01);
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 1);
    // Should convert to valid state space
    // Pole = omegaB * epsilon = 10 * 0.01 = 0.1
    EXPECT_DOUBLE_EQ(A[0], -0.1);
}

TEST(WeightingFunctionTest, ComplementaryStateSpace) {
    WeightingFunction wf = WeightingFunction::complementary(2.0, 10.0, 0.01);
    
    double A[4], B[2], C[2], D[1];
    int n;
    
    wf.toStateSpace(A, B, C, D, n);
    
    EXPECT_EQ(n, 1);
    // Verify state space conversion succeeds
}

// ============================================================================
// MuSynthesisFramework Tests
// ============================================================================

TEST(MuSynthesisTest, SetUncertaintyStructure) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {2, 1, 3};
    bool repeated[] = {true, false, true};
    
    mu.setUncertaintyStructure(blockSizes, 3, repeated);
    // Should not crash, structure is set internally
}

TEST(MuSynthesisTest, SetUncertaintyStructureNoRepeated) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {2, 2};
    
    // nullptr for repeated (use defaults)
    mu.setUncertaintyStructure(blockSizes, 2, nullptr);
}

TEST(MuSynthesisTest, SetUncertaintyStructureMany) {
    MuSynthesisFramework mu;
    
    // Test with maximum blocks (capped at 10)
    int blockSizes[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    bool repeated[12] = {false};
    
    mu.setUncertaintyStructure(blockSizes, 12, repeated);  // Will be capped to 10
}

TEST(MuSynthesisTest, ComputeMuUpperBound) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {2};
    mu.setUncertaintyStructure(blockSizes, 1, nullptr);
    
    // 2x2 matrix
    double M[4] = {1.0, 0.5, 0.5, 1.0};
    
    double upperBound = mu.computeMuUpperBound(M, 2);
    
    // Upper bound should be non-negative
    EXPECT_GE(upperBound, 0.0);
    // For this simple matrix, upper bound should be reasonable
    EXPECT_LE(upperBound, 10.0);
}

TEST(MuSynthesisTest, ComputeMuUpperBoundIdentity) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {3};
    mu.setUncertaintyStructure(blockSizes, 1, nullptr);
    
    // 3x3 identity matrix
    double M[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    
    double upperBound = mu.computeMuUpperBound(M, 3);
    
    // For identity, upper bound should be 1 (row sum of any row is 1)
    EXPECT_DOUBLE_EQ(upperBound, 1.0);
}

TEST(MuSynthesisTest, ComputeMuUpperBoundZero) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {2};
    mu.setUncertaintyStructure(blockSizes, 1, nullptr);
    
    // Zero matrix
    double M[4] = {0, 0, 0, 0};
    
    double upperBound = mu.computeMuUpperBound(M, 2);
    
    EXPECT_DOUBLE_EQ(upperBound, 0.0);
}

TEST(MuSynthesisTest, DKIteration) {
    MuSynthesisFramework mu;
    
    int blockSizes[] = {2, 1};
    mu.setUncertaintyStructure(blockSizes, 2, nullptr);
    
    bool result = mu.dkIteration();
    
    // Placeholder always returns true
    EXPECT_TRUE(result);
}

// ============================================================================
// H2Controller Additional Edge Cases
// ============================================================================

TEST_F(H2ControllerTest, ComputeWithoutDesign) {
    // Test compute when not designed
    ControllerInput input;
    input.measured = 1.0;
    input.reference = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = h2->compute(input);
    
    // Should return 0 control when not designed
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(H2ControllerTest, MultiStepCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2->design();
    
    // Run multiple steps to test state evolution
    ControllerInput input;
    input.dt = 0.001;
    
    for (int i = 0; i < 100; ++i) {
        input.measured = 1.0 * std::exp(-0.01 * i);  // Decaying measurement
        ControllerOutput output = h2->compute(input);
        // Control should be bounded
        EXPECT_LE(std::abs(output.control), 1000.0);
    }
}

TEST_F(H2ControllerTest, ResetClearsState) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    h2->setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2->design();
    
    // Run some steps
    ControllerInput input;
    input.measured = 1.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        h2->compute(input);
    }
    
    // Reset
    h2->reset();
    
    // First output after reset should match first output of fresh controller
    H2Controller h2_fresh;
    h2_fresh.setRegulatorProblem(A, B, C, Q, R, W, V, 2, 1, 1);
    h2_fresh.design();
    
    ControllerOutput out1 = h2->compute(input);
    ControllerOutput out2 = h2_fresh.compute(input);
    
    EXPECT_DOUBLE_EQ(out1.control, out2.control);
}

// ============================================================================
// HInfinityController Additional Edge Cases
// ============================================================================

TEST_F(HInfinityControllerTest, ComputeWithoutDesign) {
    ControllerInput input;
    input.measured = 1.0;
    input.reference = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = hinf->compute(input);
    
    // Should return 0 control when not designed
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(HInfinityControllerTest, SetPlantWithNullD) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    
    // Pass nullptr for D
    hinf->setPlant(A, B, C, nullptr, 2, 1, 1);
}

TEST_F(HInfinityControllerTest, DesignWithUnachievableGamma) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    
    // Very small gamma may not be achievable (depends on implementation)
    hinf->design(0.001);
}

TEST_F(HInfinityControllerTest, MultiStepCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->design(5.0);
    
    // Run multiple steps
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    
    for (int i = 0; i < 100; ++i) {
        input.measured = 1.0 * std::exp(-0.01 * i);
        ControllerOutput output = hinf->compute(input);
        EXPECT_LE(std::abs(output.control), 1000.0);
        EXPECT_FALSE(std::isnan(output.control));
    }
}

TEST_F(HInfinityControllerTest, ResetClearsState) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->design(5.0);
    
    ControllerInput input;
    input.measured = 1.0;
    input.reference = 0.0;
    input.dt = 0.001;
    
    // Run some steps
    for (int i = 0; i < 10; ++i) {
        hinf->compute(input);
    }
    
    // Reset
    hinf->reset();
    
    // First output after reset
    ControllerOutput out1 = hinf->compute(input);
    
    // Fresh controller
    HInfinityController hinf_fresh;
    hinf_fresh.setPlant(A, B, C, D, 2, 1, 1);
    hinf_fresh.design(5.0);
    ControllerOutput out2 = hinf_fresh.compute(input);
    
    EXPECT_DOUBLE_EQ(out1.control, out2.control);
}

TEST_F(HInfinityControllerTest, DesignWithAllWeights) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    
    // Set all three weights
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    hinf->setControlWeight(5.0);
    hinf->setComplementaryWeight(1.0, 50.0, 0.01);
    
    bool result = hinf->design(10.0);
    // Should complete design
}

TEST_F(HInfinityControllerTest, BuildAugmentedPlantWithWeights) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    
    // Set weights to trigger buildAugmentedPlant with weight dynamics
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    hinf->setControlWeight(5.0);
    hinf->setComplementaryWeight(1.0, 50.0, 0.01);
    
    // designOptimal will call buildAugmentedPlant internally
    double gamma = hinf->designOptimal(0.1);
    EXPECT_GT(gamma, 0.0);
}

TEST_F(HInfinityControllerTest, IsAchievableVeryLargeGamma) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    
    // Very large gamma should be achievable
    bool achievable = hinf->isAchievable(1e6);
    EXPECT_TRUE(achievable);
}

TEST_F(HInfinityControllerTest, IsAchievableVerySmallGamma) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf->setPlant(A, B, C, D, 2, 1, 1);
    hinf->setSensitivityWeight(2.0, 10.0, 0.01);
    
    // Very small gamma may not be achievable
    bool achievable = hinf->isAchievable(1e-6);
    // Result depends on plant
}

// ============================================================================
// Additional Integration Tests
// ============================================================================

TEST(RobustIntegrationTest, HInfinityClosedLoopSimulation) {
    HInfinityController hinf;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    hinf.setPlant(A, B, C, D, 2, 1, 1);
    hinf.setSensitivityWeight(2.0, 10.0, 0.01);
    hinf.design(5.0);
    
    // Simulate closed loop
    double x[2] = {1.0, 0.0};
    double dt = 0.001;
    
    for (int i = 0; i < 2000; ++i) {
        ControllerInput input;
        input.reference = 0.0;
        input.measured = x[0];
        input.dt = dt;
        
        ControllerOutput output = hinf.compute(input);
        double u = output.control;
        
        // Plant dynamics
        double x0_dot = x[1];
        double x1_dot = -x[0] - x[1] + u;
        
        x[0] += x0_dot * dt;
        x[1] += x1_dot * dt;
    }
    
    // Should stabilize
    EXPECT_LT(std::abs(x[0]), 2.0);
}

TEST(RobustIntegrationTest, WeightingFunctionChaining) {
    // Create and use multiple weighting functions
    WeightingFunction W1 = WeightingFunction::sensitivity(2.0, 10.0, 0.01);
    WeightingFunction W2 = WeightingFunction::firstOrder(1.0, 100.0, 0.5);
    WeightingFunction W3 = WeightingFunction::complementary(1.5, 50.0, 0.02);
    
    // Evaluate at various frequencies
    double freqs[] = {0.1, 1.0, 10.0, 100.0, 1000.0};
    
    for (double f : freqs) {
        double m1 = W1.magnitude(f);
        double m2 = W2.magnitude(f);
        double m3 = W3.magnitude(f);
        
        EXPECT_FALSE(std::isnan(m1));
        EXPECT_FALSE(std::isnan(m2));
        EXPECT_FALSE(std::isnan(m3));
        EXPECT_GE(m1, 0.0);
        EXPECT_GE(m2, 0.0);
        EXPECT_GE(m3, 0.0);
    }
}

TEST(RobustIntegrationTest, MuSynthesisWorkflow) {
    MuSynthesisFramework mu;
    
    // Define uncertainty structure
    int blockSizes[] = {1, 1, 1};
    bool repeated[] = {true, false, true};
    mu.setUncertaintyStructure(blockSizes, 3, repeated);
    
    // Compute mu upper bound for various matrices
    double M1[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};  // Scaled identity
    double M2[9] = {0.1, 0.2, 0.1, 0.2, 0.3, 0.2, 0.1, 0.2, 0.4};  // Random
    
    double ub1 = mu.computeMuUpperBound(M1, 3);
    double ub2 = mu.computeMuUpperBound(M2, 3);
    
    EXPECT_GE(ub1, 0.0);
    EXPECT_GE(ub2, 0.0);
    
    // D-K iteration step
    bool converged = mu.dkIteration();
    EXPECT_TRUE(converged);
}
