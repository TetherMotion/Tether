/**
 * @file StateSpaceControllers.hpp
 * @brief Modern State-Space Controllers: LQR, LQG, LQI
 * 
 * @details
 * This file implements modern state-space control methods based on
 * optimal control theory and state estimation.
 * 
 * ## State-Space Representation
 * 
 * Linear time-invariant (LTI) systems are represented as:
 * ```
 * ẋ = Ax + Bu    (state equation)
 * y = Cx + Du    (output equation)
 * 
 * Where:
 *   x ∈ ℝⁿ = state vector
 *   u ∈ ℝᵐ = control input vector
 *   y ∈ ℝᵖ = output vector
 *   A = system matrix (n×n)
 *   B = input matrix (n×m)
 *   C = output matrix (p×n)
 *   D = feedthrough matrix (p×m), often zero
 * ```
 * 
 * ## LQR: Linear Quadratic Regulator
 * 
 * ### Problem Formulation
 * Minimize the quadratic cost function:
 * ```
 *      ∞
 * J = ∫ (x'Qx + u'Ru) dt
 *      0
 * 
 * Where:
 *   Q = state weighting matrix (n×n, positive semi-definite)
 *   R = control weighting matrix (m×m, positive definite)
 * ```
 * 
 * ### Solution
 * The optimal control law is:
 * ```
 * u = -Kx
 * 
 * Where K = R⁻¹B'P
 * ```
 * P is the solution to the Algebraic Riccati Equation (ARE):
 * ```
 * A'P + PA - PBR⁻¹B'P + Q = 0
 * ```
 * 
 * ### Characteristics
 * - Optimal state feedback
 * - Requires full state measurement
 * - Guarantees stability for controllable systems
 * - Q and R are design parameters
 * 
 * ## LQG: Linear Quadratic Gaussian
 * 
 * LQG combines:
 * 1. **LQR** for optimal control
 * 2. **Kalman Filter** for optimal state estimation
 * 
 * ### Problem Formulation
 * System with noise:
 * ```
 * ẋ = Ax + Bu + w   (process noise w ~ N(0, W))
 * y = Cx + v        (measurement noise v ~ N(0, V))
 * ```
 * 
 * ### Kalman Filter (State Estimator)
 * ```
 * x̂̇ = Ax̂ + Bu + L(y - Cx̂)
 * 
 * Where L = PC'V⁻¹ (Kalman gain)
 * ```
 * P solves the filter Riccati equation:
 * ```
 * AP + PA' - PC'V⁻¹CP + W = 0
 * ```
 * 
 * ### Separation Principle
 * LQR and Kalman filter can be designed independently!
 * The combined LQG controller is optimal despite this separation.
 * 
 * ## LQI: Linear Quadratic Integrator
 * 
 * LQI augments LQR with integral action for:
 * - Zero steady-state error for step references
 * - Disturbance rejection
 * 
 * ### Augmented System
 * ```
 * [ẋ ]   [A  0][x ]   [B]
 * [  ] = [    ][  ] + [ ]u
 * [ẋᵢ]   [C  0][xᵢ]   [0]
 * 
 * Where xᵢ = ∫(r - y)dt (integral state)
 * ```
 * 
 * Then apply LQR to the augmented system.
 * 
 * ## Usage Examples
 * 
 * ### LQR for DC Motor Position Control
 * ```cpp
 * // Motor model: ẋ = [0 1; 0 -b/J][x] + [0; K/J]u
 * // x = [position; velocity]
 * 
 * LQRController lqr;
 * 
 * // Define system matrices
 * double A[4] = {0, 1, 0, -b/J};  // 2x2
 * double B[2] = {0, K/J};         // 2x1
 * 
 * lqr.setSystemMatrices(A, B, 2, 1);
 * 
 * // Design weights
 * double Q[4] = {100, 0, 0, 1};   // Position important
 * double R[1] = {0.1};             // Control effort cheap
 * 
 * lqr.setWeightMatrices(Q, R);
 * lqr.computeGain();  // Solves Riccati equation
 * 
 * // Control loop
 * double state[2] = {position, velocity};
 * ControllerInput input;
 * input.state = state;
 * input.reference = targetPosition;
 * 
 * auto output = lqr.compute(input);
 * motor.setVoltage(output.control);
 * ```
 * 
 * ### LQG with Noisy Measurements
 * ```cpp
 * LQGController lqg;
 * 
 * // Same system as above
 * lqg.setSystemMatrices(A, B, C, D, n, m, p);
 * 
 * // LQR weights
 * lqg.setLQRWeights(Q, R);
 * 
 * // Noise covariances
 * double W[4] = {0.01, 0, 0, 0.01};  // Process noise
 * double V[1] = {1.0};                // Measurement noise
 * 
 * lqg.setNoiseCovariances(W, V);
 * lqg.design();  // Computes both K and L
 * 
 * // Control loop
 * ControllerInput input;
 * input.measured = sensorReading;  // Noisy measurement
 * input.dt = 0.001;
 * 
 * auto output = lqg.compute(input);  // Internally estimates state
 * ```
 * 
 * ### LQI for Reference Tracking
 * ```cpp
 * LQIController lqi;
 * lqi.setSystemMatrices(A, B, C, n, m, p);
 * 
 * // Augmented Q matrix includes integral states
 * double Qa[9];  // (n+p) × (n+p)
 * // Original states: penalize position and velocity errors
 * // Integral state: penalize accumulated error
 * 
 * lqi.setAugmentedWeights(Qa, R);
 * lqi.design();
 * 
 * // Control loop - tracks reference
 * ControllerInput input;
 * input.reference = targetPosition;
 * input.state = state;
 * 
 * auto output = lqi.compute(input);
 * ```
 * 
 * ## Tuning Guidelines
 * 
 * ### Choosing Q and R
 * 
 * **Bryson's Rule (Starting Point)**
 * ```
 * Q_ii = 1 / (max acceptable x_i)²
 * R_jj = 1 / (max acceptable u_j)²
 * ```
 * 
 * **Intuition**
 * - Large Q → aggressive state tracking, large control effort
 * - Large R → conservative control, slow response
 * - Q/R ratio determines speed vs. effort trade-off
 * 
 * ### Noise Covariances (for LQG)
 * - W: Process noise - represents model uncertainty
 * - V: Measurement noise - from sensor specifications
 * - W/V ratio: trust model more (low) or measurements more (high)
 * 
 * @see ControllerBase
 * @see PIDController
 */

#pragma once

#include "ControllerBase.hpp"
#include "KalmanFilter.hpp"
#include <array>
#include <functional>

namespace tether::control {

// ============================================================================
// LQR Controller
// ============================================================================

/**
 * @brief Linear Quadratic Regulator (LQR) Controller
 * 
 * Optimal state feedback controller that minimizes a quadratic cost
 * function. Requires full state measurement (or estimation).
 * 
 * @note This implementation uses a simplified Riccati solver suitable
 *       for small systems. For large systems, consider using external
 *       numerical libraries.
 */
class LQRController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::LQR; }
    const char* getName() const override { return "LQR Controller"; }
    const char* getDescription() const override {
        return "Linear Quadratic Regulator. Optimal state feedback minimizing "
               "J = ∫(x'Qx + u'Ru)dt. Requires full state. Set Q high for "
               "aggressive tracking, R high for conservative control.";
    }
    
    /**
     * @brief Set system matrices (continuous-time)
     * 
     * System: ẋ = Ax + Bu
     * 
     * @param A System matrix (n×n, row-major)
     * @param B Input matrix (n×m, row-major)
     * @param n Number of states
     * @param m Number of inputs
     */
    void setSystemMatrices(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set discrete-time system matrices
     * 
     * System: x[k+1] = Ad*x[k] + Bd*u[k]
     * 
     * @param Ad Discrete system matrix
     * @param Bd Discrete input matrix
     * @param n Number of states
     * @param m Number of inputs
     */
    void setDiscreteSystemMatrices(const double* Ad, const double* Bd, int n, int m);
    
    /**
     * @brief Set weight matrices
     * 
     * @param Q State weighting (n×n, positive semi-definite)
     * @param R Control weighting (m×m, positive definite)
     */
    void setWeightMatrices(const double* Q, const double* R);
    
    /**
     * @brief Compute optimal gain matrix K
     * 
     * Solves the Algebraic Riccati Equation (ARE) and computes
     * the optimal feedback gain K = R⁻¹B'P.
     * 
     * @return true if computation succeeded
     */
    bool computeGain();
    
    /**
     * @brief Set gain matrix directly (if pre-computed)
     * @param K Gain matrix (m×n, row-major)
     */
    void setGainMatrix(const double* K);
    
    /**
     * @brief Get the computed gain matrix
     * @param K Output buffer (m×n)
     */
    void getGainMatrix(double* K) const;
    
    /**
     * @brief Get number of states
     */
    int getNumStates() const { return m_n; }
    
    /**
     * @brief Get number of inputs
     */
    int getNumInputs() const { return m_m; }
    
    /**
     * @brief Set reference state (for tracking)
     * @param ref Reference state vector (n elements)
     */
    void setReferenceState(const double* ref);
    
    /**
     * @brief Enable feedforward from reference (for tracking)
     */
    void enableFeedforward(bool enable) { m_useFeedforward = enable; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0};  // Number of states
    int m_m{0};  // Number of inputs
    
    // System matrices (stored in row-major format)
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    
    // Weight matrices
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_Q{};
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> m_R{};
    
    // Computed gain
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> m_K{};
    
    // Riccati solution
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_P{};
    
    // Reference state
    std::array<double, MAX_STATE_DIM> m_xRef{};
    
    bool m_gainComputed{false};
    bool m_isDiscrete{false};
    bool m_useFeedforward{false};
    
    // Riccati equation solver
    bool solveRiccati();
    bool solveDiscreteRiccati();
};

// ============================================================================
// LQG Controller
// ============================================================================

/**
 * @brief Linear Quadratic Gaussian (LQG) Controller
 * 
 * Combines LQR optimal control with Kalman filter state estimation.
 * Optimal for linear systems with Gaussian noise when full state
 * is not directly measurable.
 * 
 * ## Structure
 * ```
 *   u    ┌───────┐  y    ┌───────┐  x̂
 * ──────►│ Plant │──────►│Kalman │──────┐
 *        └───────┘       │Filter │      │
 *           ▲            └───────┘      │
 *           │                           │
 *           │            ┌───────┐      │
 *           └────────────│  LQR  │◄─────┘
 *                        │ u=-Kx̂│
 *                        └───────┘
 * ```
 */
class LQGController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::LQG; }
    const char* getName() const override { return "LQG Controller"; }
    const char* getDescription() const override {
        return "Linear Quadratic Gaussian. Combines LQR (optimal control) with "
               "Kalman filter (optimal estimation). Use when state is not directly "
               "measurable. Set Q/R for control, W/V for estimation.";
    }
    
    /**
     * @brief Set system matrices
     * @param A System matrix (n×n)
     * @param B Input matrix (n×m)
     * @param C Output matrix (p×n)
     * @param D Feedthrough matrix (p×m), can be nullptr
     * @param n,m,p Dimensions
     */
    void setSystemMatrices(const double* A, const double* B, 
                           const double* C, const double* D,
                           int n, int m, int p);
    
    /**
     * @brief Set LQR weight matrices
     */
    void setLQRWeights(const double* Q, const double* R);
    
    /**
     * @brief Set noise covariance matrices
     */
    void setNoiseCovariances(const double* W, const double* V);
    
    /**
     * @brief Design the complete LQG controller
     * 
     * Computes both LQR gain K and Kalman gain L.
     * 
     * @return true if design succeeded
     */
    bool design();
    
    /**
     * @brief Access the internal LQR controller
     */
    LQRController& getLQR() { return m_lqr; }
    
    /**
     * @brief Access the internal Kalman filter
     */
    KalmanFilter& getKalmanFilter() { return m_kf; }
    
    /**
     * @brief Set reference for tracking
     */
    void setReference(double ref) { m_reference = ref; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    LQRController m_lqr;
    KalmanFilter m_kf;
    
    int m_n{0}, m_m{0}, m_p{0};
    double m_reference{0.0};
    double m_lastControl{0.0};
    
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
};

// ============================================================================
// LQI Controller (LQR with Integral Action)
// ============================================================================

/**
 * @brief Linear Quadratic Integrator (LQI) Controller
 * 
 * LQR with integral action for tracking and disturbance rejection.
 * Augments the state space with integral of output error.
 * 
 * ## Augmented System
 * ```
 * [ẋ ]   [A  0][x ]   [B]     [ 0]
 * [  ] = [    ][  ] + [ ]u + [  ]r
 * [ẋᵢ]   [-C 0][xᵢ]   [0]    [I ]
 * 
 * Where xᵢ = ∫(r - y)dt
 * ```
 * 
 * Control law: u = -[Kx Ki][x; xi]
 * 
 * ## Advantages over PID
 * - Model-based (better performance if model is accurate)
 * - Systematic design via Q/R weights
 * - Handles multivariable systems naturally
 * - Guaranteed stability for controllable, observable systems
 */
class LQIController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::LQI; }
    const char* getName() const override { return "LQI Controller"; }
    const char* getDescription() const override {
        return "LQR with Integral action. Augments state with integral of error "
               "for tracking and disturbance rejection. Zero steady-state error. "
               "Systematic alternative to PID for state-space systems.";
    }
    
    /**
     * @brief Set system matrices (continuous)
     * @param A System matrix (n×n)
     * @param B Input matrix (n×m)
     * @param C Output matrix (p×n)
     * @param n,m,p Dimensions
     */
    void setSystemMatrices(const double* A, const double* B, const double* C,
                           int n, int m, int p);
    
    /**
     * @brief Set augmented weight matrices
     * 
     * @param Qa Augmented state weight ((n+p)×(n+p))
     *           [Qx   0 ]
     *           [0   Qi ]
     * @param R Control weight (m×m)
     */
    void setAugmentedWeights(const double* Qa, const double* R);
    
    /**
     * @brief Set weights separately
     * @param Qx State weight (n×n)
     * @param Qi Integral weight (p×p)
     * @param R Control weight (m×m)
     */
    void setWeights(const double* Qx, const double* Qi, const double* R);
    
    /**
     * @brief Design the LQI controller
     * @return true if design succeeded
     */
    bool design();
    
    /**
     * @brief Set integral limits (anti-windup)
     */
    void setIntegralLimits(double min, double max);
    
    /**
     * @brief Get state feedback gain Kx
     */
    void getStateGain(double* Kx) const;
    
    /**
     * @brief Get integral gain Ki
     */
    void getIntegralGain(double* Ki) const;
    
    /**
     * @brief Get current integral state
     */
    void getIntegralState(double* xi) const;
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0}, m_p{0};
    
    // Original system
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
    
    // Augmented system (n+p states)
    static constexpr int MAX_AUG = MAX_STATE_DIM + MAX_OUTPUT_DIM;
    std::array<double, MAX_AUG * MAX_AUG> m_Aa{};
    std::array<double, MAX_AUG * MAX_CONTROL_DIM> m_Ba{};
    
    // Weights
    std::array<double, MAX_AUG * MAX_AUG> m_Qa{};
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> m_R{};
    
    // Gains (split for clarity)
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> m_Kx{};
    std::array<double, MAX_CONTROL_DIM * MAX_OUTPUT_DIM> m_Ki{};
    
    // Integral state
    std::array<double, MAX_OUTPUT_DIM> m_xi{};
    double m_integralMin{-1e6};
    double m_integralMax{1e6};
    
    bool m_designed{false};
};

// ============================================================================
// Helper Functions for State-Space Operations
// ============================================================================

namespace StateSpace {

/**
 * @brief Discretize continuous system using Zero-Order Hold
 * 
 * Converts continuous ẋ = Ax + Bu to discrete x[k+1] = Ad*x[k] + Bd*u[k]
 * 
 * @param A Continuous A matrix (n×n)
 * @param B Continuous B matrix (n×m)
 * @param dt Sampling period
 * @param Ad Output discrete A (n×n)
 * @param Bd Output discrete B (n×m)
 * @param n Number of states
 * @param m Number of inputs
 */
void discretize(const double* A, const double* B, double dt,
                double* Ad, double* Bd, int n, int m);

/**
 * @brief Check if system is controllable
 * 
 * Tests rank of controllability matrix [B AB A²B ... Aⁿ⁻¹B]
 * 
 * @param A System matrix (n×n)
 * @param B Input matrix (n×m)
 * @param n Number of states
 * @param m Number of inputs
 * @return true if controllable
 */
bool isControllable(const double* A, const double* B, int n, int m);

/**
 * @brief Check if system is observable
 * 
 * Tests rank of observability matrix [C; CA; CA²; ...; CAⁿ⁻¹]
 * 
 * @param A System matrix (n×n)
 * @param C Output matrix (p×n)
 * @param n Number of states
 * @param p Number of outputs
 * @return true if observable
 */
bool isObservable(const double* A, const double* C, int n, int p);

/**
 * @brief Compute matrix exponential exp(A*t)
 * 
 * Uses Padé approximation.
 * 
 * @param A Input matrix (n×n)
 * @param t Time
 * @param expAt Output matrix (n×n)
 * @param n Matrix dimension
 */
void matrixExponential(const double* A, double t, double* expAt, int n);

} // namespace StateSpace

} // namespace tether::control
