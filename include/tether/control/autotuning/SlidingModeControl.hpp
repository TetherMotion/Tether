/**
 * @file SlidingModeControl.hpp
 * @brief Sliding Mode Control (SMC) Implementation
 * 
 * @details
 * This file implements Sliding Mode Control (SMC), a robust nonlinear control
 * technique that drives the system state to a predefined sliding surface and
 * maintains it there despite disturbances and uncertainties.
 * 
 * ## Theory Overview
 * 
 * ### Basic Concept
 * Consider a system: ẋ = f(x) + g(x)u + d(x,t)
 * 
 * SMC design involves:
 * 1. **Sliding Surface Design**: Define σ(x) = 0
 * 2. **Reaching Phase**: Drive σ → 0
 * 3. **Sliding Phase**: Maintain σ = 0
 * 
 * ### Sliding Surface
 * For tracking error e = x - x_d, typical sliding surface:
 * ```
 * σ = e^(n-1) + c_{n-2}e^(n-2) + ... + c_1ė + c_0e
 * ```
 * or more generally: σ = Ce where C is a design vector.
 * 
 * ### Control Law
 * u = u_eq + u_sw
 * 
 * - **Equivalent control** u_eq: Keeps system on surface (σ̇ = 0)
 * - **Switching control** u_sw: Drives system to surface
 * 
 * ### Chattering
 * High-frequency switching near σ = 0 causes chattering.
 * Mitigation methods:
 * - Boundary layer (saturation instead of sign)
 * - Super-twisting algorithm
 * - Higher-order SMC
 * 
 * ## Implemented Variants
 * 
 * ### Conventional SMC
 * u = u_eq - K·sign(σ)
 * 
 * ### Boundary Layer SMC
 * u = u_eq - K·sat(σ/Φ) where Φ is boundary layer width
 * 
 * ### Super-Twisting SMC (Second-Order)
 * v = -K₁|σ|^(1/2)sign(σ) + z
 * ż = -K₂sign(σ)
 * 
 * ### Integral SMC
 * Adds integral term to eliminate steady-state error
 * 
 * ### Terminal SMC
 * Guarantees finite-time convergence
 * 
 * ### Adaptive SMC
 * Adapts gains based on disturbance estimation
 * 
 * ## Usage Example
 * 
 * ```cpp
 * SlidingModeController smc;
 * 
 * // Set system model
 * smc.setSystemModel(A, B, n, m);
 * 
 * // Design sliding surface
 * smc.setSlidingSurface(C);  // σ = Cx
 * 
 * // Set control parameters
 * smc.setSwitchingGain(K);
 * smc.setBoundaryLayer(0.1);  // Reduce chattering
 * 
 * // Control loop
 * ControllerInput input;
 * input.state = currentState;
 * input.reference = targetState;
 * auto output = smc.compute(input);
 * ```
 * 
 * @see RobustControllers.hpp
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "../ControllerBase.hpp"
#include <vector>
#include <functional>
#include <cmath>
#include <complex>

namespace Control {

// ============================================================================
// SMC Configuration Types
// ============================================================================

/**
 * @brief Reaching law type for driving σ → 0
 */
enum class ReachingLaw {
    Constant,       ///< σ̇ = -K·sign(σ)
    ConstantPlusProportional, ///< σ̇ = -K·sign(σ) - λσ
    PowerRate,      ///< σ̇ = -K|σ|^α·sign(σ), 0 < α < 1
    Exponential     ///< σ̇ = -K·exp(ε|σ|)·sign(σ)
};

/**
 * @brief Chattering reduction method
 */
enum class ChatteringReduction {
    None,           ///< Pure switching (sign function)
    Saturation,     ///< Boundary layer with sat()
    Sigmoid,        ///< Smooth sigmoid approximation
    Hyperbolic,     ///< tanh() approximation
    SuperTwisting   ///< Second-order sliding mode
};

/**
 * @brief Sliding surface type
 */
enum class SurfaceType {
    Linear,         ///< σ = Cx (linear combination)
    Integral,       ///< σ = Cx + Ki∫e
    Terminal,       ///< σ = ė + βe^(p/q) (finite time)
    NonsingularTerminal, ///< Modified terminal for singularity avoidance
    FractionalOrder ///< Fractional derivative surface
};

/**
 * @brief Sliding surface parameters
 */
struct SlidingSurfaceParams {
    SurfaceType type{SurfaceType::Linear};
    std::vector<double> coefficients;  ///< Surface coefficients
    double integralGain{0.0};          ///< For integral SMC
    double terminalBeta{1.0};          ///< Terminal SMC β
    double terminalP{5.0};             ///< Terminal SMC p (odd)
    double terminalQ{3.0};             ///< Terminal SMC q (odd, p > q)
    double fractionalOrder{0.5};       ///< For fractional surface
};

/**
 * @brief SMC gain parameters
 */
struct SMCGains {
    double switchingGain{1.0};    ///< K in switching term
    double proportionalGain{0.0}; ///< λ for proportional reaching
    double powerAlpha{0.5};       ///< α for power rate law
    double boundaryWidth{0.1};    ///< Φ for boundary layer
};

// ============================================================================
// Conventional Sliding Mode Controller
// ============================================================================

/**
 * @brief Conventional Sliding Mode Controller
 * 
 * Implements basic SMC with configurable reaching law and
 * chattering reduction.
 */
class SlidingModeController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Sliding Mode Controller"; }
    const char* getDescription() const override {
        return "Robust sliding mode controller. Drives state to sliding "
               "surface and maintains it despite disturbances. "
               "Use boundary layer or super-twisting to reduce chattering.";
    }
    
    // ========================================================================
    // System Model
    // ========================================================================
    
    /**
     * @brief Set linear system model ẋ = Ax + Bu + d
     * @param A System matrix (n×n)
     * @param B Input matrix (n×m)
     * @param n State dimension
     * @param m Input dimension
     */
    void setSystemModel(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set second-order system model
     * ẍ = f(x,ẋ) + g(x,ẋ)u + d
     */
    void setSecondOrderModel(double naturalFreq, double damping,
                            double inputGain = 1.0);
    
    /**
     * @brief Set custom nonlinear dynamics
     * @param f Drift dynamics f(x)
     * @param g Input dynamics g(x)
     */
    void setNonlinearModel(
        std::function<StateVector(const StateVector&)> f,
        std::function<StateVector(const StateVector&)> g);
    
    /**
     * @brief Set disturbance bounds
     * |d| ≤ dMax
     */
    void setDisturbanceBound(double dMax) { m_dMax = dMax; }
    
    // ========================================================================
    // Sliding Surface Design
    // ========================================================================
    
    /**
     * @brief Set linear sliding surface σ = Cx
     * @param C Surface coefficient vector (1×n)
     * @param n State dimension
     */
    void setSlidingSurface(const double* C, int n);
    
    /**
     * @brief Set sliding surface for tracking
     * σ = c₁e + c₂ė + ... + e^(n-1)
     * @param lambda Surface parameter (affects convergence rate)
     * @param order System order
     */
    void setSlidingSurface(double lambda, int order);
    
    /**
     * @brief Set surface parameters
     */
    void setSurfaceParams(const SlidingSurfaceParams& params);
    
    /**
     * @brief Get current sliding variable value
     */
    double getSigma() const { return m_sigma; }
    
    // ========================================================================
    // Control Parameters
    // ========================================================================
    
    /**
     * @brief Set reaching law
     */
    void setReachingLaw(ReachingLaw law) { m_reachingLaw = law; }
    
    /**
     * @brief Set chattering reduction method
     */
    void setChatteringReduction(ChatteringReduction method) { 
        m_chatterReduction = method; 
    }
    
    /**
     * @brief Set SMC gains
     */
    void setGains(const SMCGains& gains) { m_gains = gains; }
    
    /**
     * @brief Set switching gain K
     */
    void setSwitchingGain(double K) { m_gains.switchingGain = K; }
    
    /**
     * @brief Set boundary layer width Φ
     */
    void setBoundaryLayer(double phi) { m_gains.boundaryWidth = phi; }
    
    /**
     * @brief Set proportional reaching gain λ
     */
    void setProportionalGain(double lambda) { m_gains.proportionalGain = lambda; }
    
    // ========================================================================
    // Advanced Configuration
    // ========================================================================
    
    /**
     * @brief Enable equivalent control estimation
     * 
     * If disturbance model unknown, estimate u_eq from σ dynamics
     */
    void enableEquivalentEstimation(bool enable, double filterConst = 0.01);
    
    /**
     * @brief Set control saturation limits
     */
    void setControlLimits(double uMin, double uMax) {
        m_uMin = uMin;
        m_uMax = uMax;
    }
    
    /**
     * @brief Enable adaptive gain
     * K̇ = γ|σ| when |σ| > threshold
     */
    void enableAdaptiveGain(bool enable, double gamma = 0.1, double threshold = 0.01);
    
    // ========================================================================
    // Analysis
    // ========================================================================
    
    /**
     * @brief Check reachability condition
     * σσ̇ < -η|σ| for some η > 0
     */
    bool isReachable() const;
    
    /**
     * @brief Get reaching time estimate
     */
    double estimateReachingTime(double sigma0) const;
    
    /**
     * @brief Get equivalent control (if computed)
     */
    double getEquivalentControl() const { return m_uEq; }
    
    /**
     * @brief Get switching control
     */
    double getSwitchingControl() const { return m_uSw; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    // System model
    int m_n{0}, m_m{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    bool m_hasLinearModel{false};
    
    std::function<StateVector(const StateVector&)> m_f;
    std::function<StateVector(const StateVector&)> m_g;
    bool m_hasNonlinearModel{false};
    
    double m_dMax{0.0};
    
    // Sliding surface
    std::array<double, MAX_STATE_DIM> m_C{};
    SlidingSurfaceParams m_surfaceParams;
    double m_sigma{0.0};
    double m_sigmaDot{0.0};
    double m_sigmaIntegral{0.0};
    
    // Control
    ReachingLaw m_reachingLaw{ReachingLaw::ConstantPlusProportional};
    ChatteringReduction m_chatterReduction{ChatteringReduction::Saturation};
    SMCGains m_gains;
    
    double m_uEq{0.0};
    double m_uSw{0.0};
    double m_uMin{-std::numeric_limits<double>::max()};
    double m_uMax{std::numeric_limits<double>::max()};
    
    // Equivalent control estimation
    bool m_estimateUeq{false};
    double m_filterConst{0.01};
    double m_uEqFiltered{0.0};
    
    // Adaptive gain
    bool m_adaptiveGain{false};
    double m_adaptiveGamma{0.1};
    double m_adaptiveThreshold{0.01};
    double m_adaptedK{1.0};
    
    // State
    StateVector m_lastState{};
    StateVector m_lastError{};
    double m_lastSigma{0.0};
    
    // Internal methods
    double computeSigma(const StateVector& error) const;
    double computeSigmaDot(const StateVector& error, const StateVector& errorDot) const;
    double computeEquivalentControl(const StateVector& state, 
                                   const StateVector& reference) const;
    double computeSwitchingControl(double sigma) const;
    double switchingFunction(double sigma) const;
};

// ============================================================================
// Super-Twisting Sliding Mode Controller
// ============================================================================

/**
 * @brief Super-Twisting Algorithm (STA) Controller
 * 
 * Second-order sliding mode algorithm that provides:
 * - Continuous control signal (no chattering)
 * - Finite-time convergence to σ = σ̇ = 0
 * - Robustness to Lipschitz disturbances
 * 
 * Algorithm:
 * u = -K₁|σ|^(1/2)sign(σ) + v
 * v̇ = -K₂sign(σ)
 */
class SuperTwistingController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Super-Twisting SMC"; }
    const char* getDescription() const override {
        return "Second-order sliding mode using super-twisting algorithm. "
               "Continuous control with finite-time convergence and no chattering.";
    }
    
    /**
     * @brief Set system model
     */
    void setSystemModel(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set sliding surface
     */
    void setSlidingSurface(const double* C, int n);
    
    /**
     * @brief Set super-twisting gains
     * @param K1 Gain for proportional term
     * @param K2 Gain for integral term
     * 
     * For disturbance bound D: K1 > 2√D, K2 > D
     */
    void setGains(double K1, double K2);
    
    /**
     * @brief Set gains based on disturbance bound
     * @param dMax Maximum disturbance magnitude
     * @param dDotMax Maximum disturbance rate
     */
    void setGainsFromDisturbance(double dMax, double dDotMax);
    
    /**
     * @brief Get integral state
     */
    double getIntegralState() const { return m_v; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_STATE_DIM> m_C{};
    
    double m_K1{1.0};
    double m_K2{1.0};
    double m_v{0.0};  // Integral state
    double m_sigma{0.0};
};

// ============================================================================
// Terminal Sliding Mode Controller
// ============================================================================

/**
 * @brief Terminal Sliding Mode Controller
 * 
 * Guarantees finite-time convergence using nonlinear sliding surface.
 * 
 * Sliding surface: σ = ė + β·e^(p/q)
 * where p, q are positive odd integers with p > q
 * 
 * Convergence time: t_s = (q/(β(p-q)))|e(0)|^((p-q)/q)
 */
class TerminalSlidingModeController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Terminal SMC"; }
    const char* getDescription() const override {
        return "Terminal sliding mode with finite-time convergence. "
               "Nonlinear surface σ = ė + β·e^(p/q) guarantees finite settling.";
    }
    
    /**
     * @brief Set terminal surface parameters
     * @param beta Convergence rate parameter
     * @param p, q Odd integers with p > q (typically p=5, q=3)
     */
    void setTerminalParameters(double beta, int p, int q);
    
    /**
     * @brief Set system parameters
     */
    void setSystemModel(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set switching gain
     */
    void setSwitchingGain(double K) { m_K = K; }
    
    /**
     * @brief Set boundary layer for chattering reduction
     */
    void setBoundaryLayer(double phi) { m_phi = phi; }
    
    /**
     * @brief Enable non-singular terminal SMC
     * 
     * Avoids singularity when e = 0, ė ≠ 0
     */
    void enableNonsingular(bool enable) { m_nonsingular = enable; }
    
    /**
     * @brief Estimate convergence time from current state
     */
    double estimateConvergenceTime(double error) const;
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    
    double m_beta{1.0};
    int m_p{5};
    int m_q{3};
    double m_K{1.0};
    double m_phi{0.05};
    bool m_nonsingular{true};
    
    double m_sigma{0.0};
    double m_lastError{0.0};
    double m_errorDot{0.0};
};

// ============================================================================
// Integral Sliding Mode Controller
// ============================================================================

/**
 * @brief Integral Sliding Mode Controller
 * 
 * Adds integral term to eliminate reaching phase and provide
 * invariance from initial time.
 * 
 * Surface: σ = x - x_d + ∫(f(x) + g(x)u_nom - ẋ_d)dt = 0
 */
class IntegralSlidingModeController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Integral SMC"; }
    const char* getDescription() const override {
        return "Integral sliding mode for reaching-phase-free design. "
               "Provides invariance to matched uncertainties from t=0.";
    }
    
    /**
     * @brief Set system model
     */
    void setSystemModel(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set nominal control (baseline without SMC)
     */
    void setNominalControl(std::function<double(const StateVector&, double)> uNom);
    
    /**
     * @brief Set integral gain
     */
    void setIntegralGain(const double* Ki, int n);
    
    /**
     * @brief Set discontinuous control gain
     */
    void setSwitchingGain(double K) { m_K = K; }
    
    /**
     * @brief Set boundary layer
     */
    void setBoundaryLayer(double phi) { m_phi = phi; }
    
    /**
     * @brief Initialize integral state for smooth start
     */
    void initialize(const StateVector& x0, const StateVector& xd0);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_STATE_DIM> m_Ki{};
    
    std::function<double(const StateVector&, double)> m_uNom;
    
    double m_K{1.0};
    double m_phi{0.1};
    
    StateVector m_integralState{};
    double m_sigma{0.0};
};

// ============================================================================
// Adaptive Sliding Mode Controller
// ============================================================================

/**
 * @brief Adaptive Sliding Mode Controller
 * 
 * Adapts switching gain based on disturbance estimation to:
 * - Reduce chattering during nominal operation
 * - Maintain robustness under large disturbances
 */
class AdaptiveSlidingModeController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Adaptive SMC"; }
    const char* getDescription() const override {
        return "Adaptive sliding mode with online gain adjustment. "
               "Estimates disturbance bound to minimize chattering while "
               "maintaining robustness.";
    }
    
    /**
     * @brief Set system model
     */
    void setSystemModel(const double* A, const double* B, int n, int m);
    
    /**
     * @brief Set sliding surface
     */
    void setSlidingSurface(const double* C, int n);
    
    /**
     * @brief Set adaptation parameters
     * @param gamma Adaptation rate
     * @param Kmin Minimum gain
     * @param Kmax Maximum gain
     */
    void setAdaptationParams(double gamma, double Kmin, double Kmax);
    
    /**
     * @brief Set boundary layer
     */
    void setBoundaryLayer(double phi) { m_phi = phi; }
    
    /**
     * @brief Get current adapted gain
     */
    double getAdaptedGain() const { return m_K; }
    
    /**
     * @brief Get estimated disturbance
     */
    double getDisturbanceEstimate() const { return m_dHat; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_STATE_DIM> m_C{};
    
    double m_gamma{0.1};
    double m_Kmin{0.1};
    double m_Kmax{10.0};
    double m_K{1.0};
    double m_phi{0.1};
    
    double m_sigma{0.0};
    double m_dHat{0.0};  // Disturbance estimate
};

// ============================================================================
// SMC Utilities
// ============================================================================

/**
 * @brief Utility functions for SMC design
 */
class SMCDesignUtils {
public:
    /**
     * @brief Design optimal sliding surface coefficients
     * 
     * For system ẋ = Ax + Bu, finds C such that σ = Cx = 0
     * results in desired dynamics.
     * 
     * @param A System matrix
     * @param B Input matrix
     * @param n State dimension
     * @param desiredPoles Desired poles of sliding dynamics
     * @return Surface coefficients C
     */
    static std::vector<double> designSurface(
        const double* A, const double* B, int n,
        const std::vector<std::complex<double>>& desiredPoles);
    
    /**
     * @brief Compute minimum switching gain for robustness
     * 
     * K > |d_max| + η for some η > 0
     */
    static double minimumSwitchingGain(double disturbanceBound, 
                                       double safetyMargin = 0.1);
    
    /**
     * @brief Estimate reaching time
     * 
     * For σ̇ = -K·sign(σ): t_r = |σ(0)|/K
     * For σ̇ = -K·sign(σ) - λσ: t_r = (1/λ)ln(1 + λ|σ(0)|/K)
     */
    static double reachingTime(double sigma0, double K, 
                              double lambda = 0.0);
    
    /**
     * @brief Check Lyapunov stability condition
     * V̇ = σσ̇ < 0 outside boundary layer
     */
    static bool checkLyapunovCondition(double sigma, double sigmaDot,
                                       double boundaryLayer = 0.0);
    
    /**
     * @brief Compute chattering amplitude estimate
     * @param K Switching gain
     * @param phi Boundary layer width
     * @param systemBandwidth System bandwidth (approximate)
     */
    static double chatteringAmplitude(double K, double phi, 
                                     double systemBandwidth);
};

/**
 * @brief Higher-Order SMC implementation
 */
class HigherOrderSMC {
public:
    /**
     * @brief Set sliding mode order
     * @param order 1 for conventional, 2 for super-twisting, etc.
     */
    void setOrder(int order);
    
    /**
     * @brief Compute control for r-th order SMC
     * Drives σ, σ̇, ..., σ^(r-1) to zero
     */
    double compute(double sigma, const std::vector<double>& sigmaDerivatives,
                  double dt);
    
    /**
     * @brief Set gains for each order
     */
    void setGains(const std::vector<double>& gains);
    
private:
    int m_order{2};
    std::vector<double> m_gains;
    std::vector<double> m_integralStates;
};

} // namespace Control
