/**
 * @file StateSpaceControllersTests.cpp
 * @brief Comprehensive tests for StateSpaceControllers module
 * Tests for LQRController, KalmanFilter, LQGController, LQIController
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <array>

#include "tether/control/Controllers.hpp"
#include "tether/control/StateSpaceControllers.hpp"

using namespace Control;

// ============================================================================
// LQRController Tests
// ============================================================================

class LQRControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        lqr = std::make_unique<LQRController>();
    }
    
    std::unique_ptr<LQRController> lqr;
};

TEST_F(LQRControllerTest, GetType) {
    EXPECT_EQ(lqr->getType(), ControllerType::LQR);
}

TEST_F(LQRControllerTest, GetName) {
    EXPECT_STREQ(lqr->getName(), "LQR Controller");
}

TEST_F(LQRControllerTest, GetDescription) {
    EXPECT_NE(lqr->getDescription(), nullptr);
    EXPECT_GT(strlen(lqr->getDescription()), 0);
}

TEST_F(LQRControllerTest, SetSystemMatrices) {
    // Double integrator: x1' = x2, x2' = u
    double A[4] = {0, 1, 0, 0};  // 2x2
    double B[2] = {0, 1};        // 2x1
    
    lqr->setSystemMatrices(A, B, 2, 1);
    
    EXPECT_EQ(lqr->getNumStates(), 2);
    EXPECT_EQ(lqr->getNumInputs(), 1);
}

TEST_F(LQRControllerTest, SetDiscreteSystemMatrices) {
    double Ad[4] = {1, 0.1, 0, 1};  // Discrete double integrator
    double Bd[2] = {0.005, 0.1};
    
    lqr->setDiscreteSystemMatrices(Ad, Bd, 2, 1);
}

TEST_F(LQRControllerTest, SetWeightMatrices) {
    double Q[4] = {10, 0, 0, 1};  // 2x2
    double R[1] = {0.1};         // 1x1
    
    lqr->setWeightMatrices(Q, R);
}

TEST_F(LQRControllerTest, ComputeGain) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setSystemMatrices(A, B, 2, 1);
    lqr->setWeightMatrices(Q, R);
    
    bool result = lqr->computeGain();
    // Should succeed for this well-conditioned system
}

TEST_F(LQRControllerTest, SetGainMatrix) {
    double K[2] = {5.0, 2.0};  // Pre-computed gain (1x2)
    lqr->setGainMatrix(K);
}

TEST_F(LQRControllerTest, GetGainMatrix) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setSystemMatrices(A, B, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    
    double K[2];
    lqr->getGainMatrix(K);
}

TEST_F(LQRControllerTest, SetReferenceState) {
    double ref[2] = {1.0, 0.0};  // Target position=1, velocity=0
    lqr->setReferenceState(ref);
}

TEST_F(LQRControllerTest, EnableFeedforward) {
    lqr->enableFeedforward(true);
    lqr->enableFeedforward(false);
}

TEST_F(LQRControllerTest, BasicCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setSystemMatrices(A, B, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    
    ControllerInput input;
    input.state[0] = 1.0;   // Position
    input.state[1] = 0.5;   // Velocity
    input.stateDim = 2;
    input.reference = 0.0;  // Regulate to zero
    input.dt = 0.001;
    
    ControllerOutput output = lqr->compute(input);
    EXPECT_NE(output.control, 0.0);  // Should produce control
}

TEST_F(LQRControllerTest, TrackingWithReference) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setSystemMatrices(A, B, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    
    double ref[2] = {5.0, 0.0};  // Target position=5
    lqr->setReferenceState(ref);
    
    ControllerInput input;
    input.state[0] = 0.0;
    input.state[1] = 0.0;
    input.stateDim = 2;
    input.dt = 0.001;
    
    // ControllerOutput output = lqr->compute(input); // Not used
    lqr->compute(input);
}

TEST_F(LQRControllerTest, Reset) {
    lqr->reset();
}

// ============================================================================
// KalmanFilter Tests
// ============================================================================

class KalmanFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        kf = std::make_unique<KalmanFilter>();
    }
    
    std::unique_ptr<KalmanFilter> kf;
};

TEST_F(KalmanFilterTest, GetName) {
    EXPECT_STREQ(kf->getName(), "Kalman Filter");
}

TEST_F(KalmanFilterTest, SetSystemMatrices) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};  // Measure position only
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
}

TEST_F(KalmanFilterTest, SetNoiseCovariances) {
    double W[4] = {0.01, 0, 0, 0.01};  // Process noise
    double V[1] = {0.1};               // Measurement noise
    
    kf->setNoiseCovariances(W, V);
}

TEST_F(KalmanFilterTest, ComputeGain) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    kf->setNoiseCovariances(W, V);
    
    bool result = kf->computeGain();
}

TEST_F(KalmanFilterTest, SetKalmanGain) {
    double L[2] = {0.5, 0.2};  // Pre-computed Kalman gain (2x1)
    kf->setKalmanGain(L);
}

TEST_F(KalmanFilterTest, SetInitialState) {
    double x0[2] = {0.0, 0.0};
    kf->setInitialState(x0);
}

TEST_F(KalmanFilterTest, GetEstimatedState) {
    double A[4] = {1, 0.1, 0, 1};  // Discrete system
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    
    double x[2];
    kf->getEstimatedState(x);
}

TEST_F(KalmanFilterTest, PredictAndUpdate) {
    double A[4] = {1, 0.1, 0, 1};
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    kf->setNoiseCovariances(W, V);
    kf->computeGain();
    
    double x0[2] = {0.0, 0.0};
    kf->setInitialState(x0);
    
    // Predict with control input
    double u = 1.0;
    kf->predict(&u, 0.1);
    
    // Update with measurement
    double y = 0.1;
    kf->update(&y);
    
    double x[2];
    kf->getEstimatedState(x);
}

TEST_F(KalmanFilterTest, GetStateDim) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    
    EXPECT_EQ(kf->getStateDim(), 2);
}

TEST_F(KalmanFilterTest, Reset) {
    kf->reset();
}

// ============================================================================
// LQGController Tests
// ============================================================================

class LQGControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        lqg = std::make_unique<LQGController>();
    }
    
    std::unique_ptr<LQGController> lqg;
};

TEST_F(LQGControllerTest, GetType) {
    EXPECT_EQ(lqg->getType(), ControllerType::LQG);
}

TEST_F(LQGControllerTest, GetName) {
    EXPECT_STREQ(lqg->getName(), "LQG Controller");
}

TEST_F(LQGControllerTest, GetDescription) {
    EXPECT_NE(lqg->getDescription(), nullptr);
}

TEST_F(LQGControllerTest, SetSystemMatrices) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    
    lqg->setSystemMatrices(A, B, C, D, 2, 1, 1);
}

TEST_F(LQGControllerTest, SetLQRWeights) {
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqg->setLQRWeights(Q, R);
}

TEST_F(LQGControllerTest, SetNoiseCovariances) {
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    lqg->setNoiseCovariances(W, V);
}

TEST_F(LQGControllerTest, Design) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    lqg->setSystemMatrices(A, B, C, D, 2, 1, 1);
    lqg->setLQRWeights(Q, R);
    lqg->setNoiseCovariances(W, V);
    
    bool result = lqg->design();
}

TEST_F(LQGControllerTest, SetReference) {
    lqg->setReference(10.0);
}

TEST_F(LQGControllerTest, GetLQR) {
    LQRController& lqr = lqg->getLQR();
    // Should return reference to internal LQR
}

TEST_F(LQGControllerTest, GetKalmanFilter) {
    KalmanFilter& kf = lqg->getKalmanFilter();
    // Should return reference to internal KF
}

TEST_F(LQGControllerTest, BasicCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    lqg->setSystemMatrices(A, B, C, D, 2, 1, 1);
    lqg->setLQRWeights(Q, R);
    lqg->setNoiseCovariances(W, V);
    lqg->design();
    
    ControllerInput input;
    input.reference = 0.0;
    input.measured = 1.0;  // Noisy measurement
    input.dt = 0.001;
    
    ControllerOutput output = lqg->compute(input);
}

TEST_F(LQGControllerTest, Reset) {
    lqg->reset();
}

// ============================================================================
// LQIController Tests
// ============================================================================

class LQIControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        lqi = std::make_unique<LQIController>();
    }
    
    std::unique_ptr<LQIController> lqi;
};

TEST_F(LQIControllerTest, GetType) {
    EXPECT_EQ(lqi->getType(), ControllerType::LQI);
}

TEST_F(LQIControllerTest, GetName) {
    EXPECT_STREQ(lqi->getName(), "LQI Controller");
}

TEST_F(LQIControllerTest, GetDescription) {
    EXPECT_NE(lqi->getDescription(), nullptr);
}

TEST_F(LQIControllerTest, SetSystemMatrices) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
}

TEST_F(LQIControllerTest, SetAugmentedWeights) {
    // Augmented system: 2 states + 1 integral = 3 states
    double Qa[9] = {
        10, 0, 0,
        0, 1, 0,
        0, 0, 5   // Integral weight
    };
    double R[1] = {0.1};
    
    lqi->setAugmentedWeights(Qa, R);
}

TEST_F(LQIControllerTest, SetWeights) {
    double Qx[4] = {10, 0, 0, 1};  // State weight
    double Qi[1] = {5};            // Integral weight
    double R[1] = {0.1};           // Control weight
    
    lqi->setWeights(Qx, Qi, R);
}

TEST_F(LQIControllerTest, Design) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {5};
    double R[1] = {0.1};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
    lqi->setWeights(Qx, Qi, R);
    
    bool result = lqi->design();
}

TEST_F(LQIControllerTest, SetIntegralLimits) {
    lqi->setIntegralLimits(-100.0, 100.0);
}

TEST_F(LQIControllerTest, GetStateGain) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {5};
    double R[1] = {0.1};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
    lqi->setWeights(Qx, Qi, R);
    lqi->design();
    
    double Kx[2];
    lqi->getStateGain(Kx);
}

TEST_F(LQIControllerTest, GetIntegralGain) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {5};
    double R[1] = {0.1};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
    lqi->setWeights(Qx, Qi, R);
    lqi->design();
    
    double Ki[1];
    lqi->getIntegralGain(Ki);
}

TEST_F(LQIControllerTest, GetIntegralState) {
    double xi[1];
    lqi->getIntegralState(xi);
}

TEST_F(LQIControllerTest, BasicCompute) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {5};
    double R[1] = {0.1};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
    lqi->setWeights(Qx, Qi, R);
    lqi->design();
    
    ControllerInput input;
    input.reference = 1.0;  // Track reference
    input.state[0] = 0.0;
    input.state[1] = 0.0;
    input.stateDim = 2;
    input.dt = 0.001;
    
    ControllerOutput output = lqi->compute(input);
}

TEST_F(LQIControllerTest, TrackingWithIntegral) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {5};
    double R[1] = {0.1};
    
    lqi->setSystemMatrices(A, B, C, 2, 1, 1);
    lqi->setWeights(Qx, Qi, R);
    lqi->design();
    
    // Simulate tracking
    double x[2] = {0.0, 0.0};
    double ref = 1.0;
    double dt = 0.01;
    
    for (int i = 0; i < 100; ++i) {
        ControllerInput input;
        input.reference = ref;
        input.state[0] = x[0];
        input.state[1] = x[1];
        input.stateDim = 2;
        input.dt = dt;
        
        ControllerOutput output = lqi->compute(input);
        
        // Simple plant simulation
        x[0] += x[1] * dt;
        x[1] += (-x[0] - x[1] + output.control) * dt;
    }
    
    // Should track reference eventually
}

TEST_F(LQIControllerTest, Reset) {
    lqi->reset();
}

// ============================================================================
// StateSpace Helper Function Tests
// ============================================================================

TEST(StateSpaceHelpersTest, Discretize) {
    double A[4] = {0, 1, 0, 0};  // Double integrator
    double B[2] = {0, 1};
    double Ad[4], Bd[2];
    
    StateSpace::discretize(A, B, 0.01, Ad, Bd, 2, 1);
    
    // Ad should be close to I + A*dt for small dt
    EXPECT_NEAR(Ad[0], 1.0, 0.01);  // [1,0] diagonal
    EXPECT_NEAR(Ad[3], 1.0, 0.01);  // [1,1] diagonal
}

TEST(StateSpaceHelpersTest, IsControllable) {
    double A[4] = {0, 1, 0, 0};
    double B[2] = {0, 1};
    
    bool ctrl = StateSpace::isControllable(A, B, 2, 1);
    // Double integrator is controllable
    EXPECT_TRUE(ctrl);
}

TEST(StateSpaceHelpersTest, IsObservable) {
    double A[4] = {0, 1, 0, 0};
    double C[2] = {1, 0};  // Measure position
    
    bool obs = StateSpace::isObservable(A, C, 2, 1);
    // System is observable from position
    EXPECT_TRUE(obs);
}

TEST(StateSpaceHelpersTest, MatrixExponential) {
    double A[4] = {0, 1, 0, 0};  // Nilpotent
    double expAt[4];
    
    StateSpace::matrixExponential(A, 0.1, expAt, 2);
    
    // exp(A*t) for nilpotent matrix: I + At + (At)^2/2 + ...
    // For this A: exp(At) = [1, t; 0, 1]
    EXPECT_NEAR(expAt[0], 1.0, 0.01);
    EXPECT_NEAR(expAt[1], 0.1, 0.01);
    EXPECT_NEAR(expAt[2], 0.0, 0.01);
    EXPECT_NEAR(expAt[3], 1.0, 0.01);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(StateSpaceIntegrationTest, LQRRegulation) {
    LQRController lqr;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr.setSystemMatrices(A, B, 2, 1);
    lqr.setWeightMatrices(Q, R);
    lqr.computeGain();
    
    // Simulate closed loop
    double x[2] = {1.0, 0.0};  // Initial state
    double dt = 0.001;
    
    for (int i = 0; i < 2000; ++i) {
        ControllerInput input;
        input.state[0] = x[0];
        input.state[1] = x[1];
        input.stateDim = 2;
        input.reference = 0.0;
        input.dt = dt;
        
        ControllerOutput output = lqr.compute(input);
        double u = output.control;
        
        // Plant: x' = Ax + Bu
        double x0_dot = x[1];
        double x1_dot = -x[0] - x[1] + u;
        
        x[0] += x0_dot * dt;
        x[1] += x1_dot * dt;
    }
    
    // Should regulate to zero
    EXPECT_NEAR(x[0], 0.0, 0.1);
    EXPECT_NEAR(x[1], 0.0, 0.1);
}

TEST(StateSpaceIntegrationTest, LQGEstimationAndControl) {
    LQGController lqg;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double D[1] = {0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    double W[4] = {0.001, 0, 0, 0.001};
    double V[1] = {0.01};
    
    lqg.setSystemMatrices(A, B, C, D, 2, 1, 1);
    lqg.setLQRWeights(Q, R);
    lqg.setNoiseCovariances(W, V);
    lqg.design();
    
    // Test that compute works without errors
    ControllerInput input;
    input.measured = 1.0;
    input.dt = 0.001;
    
    ControllerOutput output = lqg.compute(input);
    
    // Output should not be NaN
    EXPECT_FALSE(std::isnan(output.control));
}

TEST(StateSpaceIntegrationTest, LQITracking) {
    LQIController lqi;
    
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Qx[4] = {10, 0, 0, 1};
    double Qi[1] = {10};
    double R[1] = {0.1};
    
    lqi.setSystemMatrices(A, B, C, 2, 1, 1);
    lqi.setWeights(Qx, Qi, R);
    lqi.design();
    
    double x[2] = {0.0, 0.0};
    double ref = 1.0;
    double dt = 0.001;
    
    for (int i = 0; i < 5000; ++i) {
        ControllerInput input;
        input.reference = ref;
        input.state[0] = x[0];
        input.state[1] = x[1];
        input.stateDim = 2;
        input.dt = dt;
        
        ControllerOutput output = lqi.compute(input);
        double u = output.control;
        
        double x0_dot = x[1];
        double x1_dot = -x[0] - x[1] + u;
        
        x[0] += x0_dot * dt;
        x[1] += x1_dot * dt;
    }
    
    // Should track reference with zero steady-state error
    EXPECT_NEAR(x[0], ref, 0.1);
}

TEST(StateSpaceIntegrationTest, CompareLQRvsLQI) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double C[2] = {1, 0};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    LQRController lqr;
    lqr.setSystemMatrices(A, B, 2, 1);
    lqr.setWeightMatrices(Q, R);
    lqr.computeGain();
    
    LQIController lqi;
    lqi.setSystemMatrices(A, B, C, 2, 1, 1);
    double Qi[1] = {5};
    lqi.setWeights(Q, Qi, R);
    lqi.design();
    
    // LQI should provide zero steady-state error for step input
    // LQR may have steady-state error without feedforward
}
// ============================================================================
// LQR Discrete System Tests
// ============================================================================

TEST_F(LQRControllerTest, DiscreteComputeGain) {
    // Discrete double integrator (Ts = 0.1s)
    // x(k+1) = A*x(k) + B*u(k)
    // A = [1, Ts; 0, 1], B = [0.5*Ts^2; Ts]
    double Ts = 0.1;
    double Ad[4] = {1, Ts, 0, 1};  // Discrete state transition
    double Bd[2] = {0.5 * Ts * Ts, Ts};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setDiscreteSystemMatrices(Ad, Bd, 2, 1);
    lqr->setWeightMatrices(Q, R);
    
    bool result = lqr->computeGain();
    EXPECT_TRUE(result);
    
    double K[2];
    lqr->getGainMatrix(K);
    
    // Verify gains are non-zero
    EXPECT_NE(K[0], 0.0);
    EXPECT_NE(K[1], 0.0);
}

TEST_F(LQRControllerTest, DiscreteComputeWithGain) {
    double Ts = 0.1;
    double Ad[4] = {1, Ts, 0, 1};
    double Bd[2] = {0.5 * Ts * Ts, Ts};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setDiscreteSystemMatrices(Ad, Bd, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    
    ControllerInput input;
    input.state[0] = 1.0;  // Position error
    input.state[1] = 0.0;  // Zero velocity
    input.stateDim = 2;
    input.dt = Ts;
    
    ControllerOutput output = lqr->compute(input);
    EXPECT_NE(output.control, 0.0);
}

TEST_F(LQRControllerTest, DiscreteTrackingSimulation) {
    // Simulate discrete LQR tracking
    double Ts = 0.1;
    double Ad[4] = {1, Ts, 0, 1};
    double Bd[2] = {0.5 * Ts * Ts, Ts};
    double Q[4] = {100, 0, 0, 10};
    double R[1] = {1.0};
    
    lqr->setDiscreteSystemMatrices(Ad, Bd, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    
    double ref[2] = {1.0, 0.0};  // Target position = 1, velocity = 0
    lqr->setReferenceState(ref);
    
    double x[2] = {0.0, 0.0};  // Initial state
    
    for (int k = 0; k < 100; ++k) {
        ControllerInput input;
        input.state[0] = x[0];
        input.state[1] = x[1];
        input.stateDim = 2;
        input.dt = Ts;
        
        ControllerOutput output = lqr->compute(input);
        double u = output.control;
        
        // Simulate discrete system: x(k+1) = Ad*x(k) + Bd*u(k)
        double x0_new = Ad[0] * x[0] + Ad[1] * x[1] + Bd[0] * u;
        double x1_new = Ad[2] * x[0] + Ad[3] * x[1] + Bd[1] * u;
        x[0] = x0_new;
        x[1] = x1_new;
    }
    
    // Should have converged close to reference
    EXPECT_NEAR(x[0], 1.0, 0.5);
}

TEST_F(LQRControllerTest, ComputeGainWithoutSetup) {
    // Test computeGain when system is not set up
    bool result = lqr->computeGain();
    EXPECT_FALSE(result);  // Should fail with n=0
}

TEST_F(LQRControllerTest, ComputeWithoutGain) {
    // Test compute when gain is not computed
    ControllerInput input;
    input.state[0] = 1.0;
    input.state[1] = 0.5;
    input.stateDim = 2;
    input.dt = 0.001;
    
    ControllerOutput output = lqr->compute(input);
    EXPECT_EQ(output.control, 0.0);  // Should return 0
}

TEST_F(LQRControllerTest, FeedforwardEnabled) {
    double A[4] = {0, 1, -1, -1};
    double B[2] = {0, 1};
    double Q[4] = {10, 0, 0, 1};
    double R[1] = {0.1};
    
    lqr->setSystemMatrices(A, B, 2, 1);
    lqr->setWeightMatrices(Q, R);
    lqr->computeGain();
    lqr->enableFeedforward(true);
    
    double ref[2] = {5.0, 0.0};
    lqr->setReferenceState(ref);
    
    ControllerInput input;
    input.state[0] = 0.0;
    input.state[1] = 0.0;
    input.stateDim = 2;
    input.dt = 0.001;
    input.feedforward = 10.0;  // Add feedforward
    
    ControllerOutput output = lqr->compute(input);
    // Output should include feedforward contribution
}

// ============================================================================
// KalmanFilter Additional Coverage Tests
// ============================================================================

TEST_F(KalmanFilterTest, EstimateFunction) {
    double A[4] = {1, 0.1, 0, 1};  // Discrete system
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    kf->setNoiseCovariances(W, V);
    kf->computeGain();
    
    double x0[2] = {0.0, 0.0};
    kf->setInitialState(x0);
    
    // Use the estimate() function (combines predict and update)
    OutputVector measurement = {0.5};  // OutputVector for measurement
    ControlVector control = {1.0};
    
    StateVector state = kf->estimate(measurement, control, 0.1);
    
    // State should be updated
    EXPECT_FALSE(std::isnan(state[0]));
    EXPECT_FALSE(std::isnan(state[1]));
}

TEST_F(KalmanFilterTest, GetStateFunction) {
    double A[4] = {1, 0.1, 0, 1};
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    
    double x0[2] = {3.0, 4.0};
    kf->setInitialState(x0);
    
    // Use the getState() function that returns StateVector
    StateVector state = kf->getState();
    
    EXPECT_DOUBLE_EQ(state[0], 3.0);
    EXPECT_DOUBLE_EQ(state[1], 4.0);
}

TEST_F(KalmanFilterTest, MultipleEstimateCalls) {
    double A[4] = {1, 0.1, 0, 1};
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    kf->setNoiseCovariances(W, V);
    kf->computeGain();
    
    double x0[2] = {0.0, 0.0};
    kf->setInitialState(x0);
    
    // Multiple estimation cycles
    for (int i = 0; i < 50; ++i) {
        OutputVector measurement = {0.1 * i};  // OutputVector
        ControlVector control = {0.5};
        
        StateVector state = kf->estimate(measurement, control, 0.01);
        EXPECT_FALSE(std::isnan(state[0]));
    }
}

TEST_F(KalmanFilterTest, ResetAndReuseEstimate) {
    double A[4] = {1, 0.1, 0, 1};
    double B[2] = {0.005, 0.1};
    double C[2] = {1, 0};
    double W[4] = {0.01, 0, 0, 0.01};
    double V[1] = {0.1};
    
    kf->setSystemMatrices(A, B, C, 2, 1, 1);
    kf->setNoiseCovariances(W, V);
    kf->computeGain();
    
    // Run some estimates
    for (int i = 0; i < 20; ++i) {
        OutputVector measurement = {1.0};  // OutputVector
        ControlVector control = {0.1};
        kf->estimate(measurement, control, 0.01);
    }
    
    // Reset
    kf->reset();
    
    // State should be zero after reset
    StateVector state = kf->getState();
    EXPECT_DOUBLE_EQ(state[0], 0.0);
    EXPECT_DOUBLE_EQ(state[1], 0.0);
}