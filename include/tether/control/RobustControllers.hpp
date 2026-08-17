/**
 * @file RobustControllers.hpp
 * @brief Robust Control: H2 Optimal and H∞ Control Frameworks
 * 
 * @details
 * This file provides frameworks for H2 optimal control and H∞ (H-infinity)
 * robust control design. These methods are essential when dealing with
 * model uncertainty and disturbances.
 * 
 * ## H2 Optimal Control
 * 
 * ### Problem Statement
 * Minimize the H2 norm of the closed-loop transfer function from
 * disturbance to output:
 * 
 * ```
 * ||T_zw||₂ = √(1/(2π) ∫ trace[T_zw*(jω)T_zw(jω)] dω)
 * ```
 * 
 * This equals the RMS value of z when w is unit white noise.
 * 
 * ### Generalized Plant Formulation
 * ```
 * [z]   [P₁₁ P₁₂] [w]
 * [ ] = [       ] [ ]
 * [y]   [P₂₁ P₂₂] [u]
 * 
 * Where:
 *   w = exogenous inputs (disturbances, noise, reference)
 *   z = regulated outputs (errors, control effort)
 *   y = measured outputs
 *   u = control inputs
 * ```
 * 
 * ### Solution
 * H2 optimal control is equivalent to LQG when properly formulated:
 * - State feedback: K = R⁻¹B₂'X (X solves control Riccati)
 * - Observer: L = YC₂'V⁻¹ (Y solves filter Riccati)
 * 
 * ### When to Use H2
 * - Disturbance rejection with stochastic inputs
 * - When you have good noise models
 * - Output energy minimization
 * - Equivalent to LQG for standard problems
 * 
 * ## H∞ Control
 * 
 * ### Problem Statement
 * Design controller K(s) such that:
 * ```
 * ||T_zw||∞ = sup |T_zw(jω)| < γ
 *              ω
 * ```
 * 
 * The H∞ norm is the peak gain from w to z across all frequencies.
 * 
 * ### Key Insight: Worst-Case Design
 * H∞ minimizes the worst-case amplification, making it robust to
 * model uncertainty and worst-case disturbances.
 * 
 * ### Standard Problem
 * Given generalized plant P and performance level γ:
 * ```
 * min γ subject to ||F_l(P, K)||∞ < γ
 *  K
 * ```
 * 
 * ### Solution Methods
 * 1. **γ-iteration**: Bisection on γ, solving Riccati equations
 * 2. **LMI formulation**: Convert to linear matrix inequality
 * 
 * ### Mixed Sensitivity
 * Common H∞ design approach using weighting functions:
 * ```
 * ||W₁·S  ||
 * ||W₂·KS ||  < γ
 * ||W₃·T  ||∞
 * 
 * Where:
 *   S = sensitivity (1/(1+GK))
 *   T = complementary sensitivity (GK/(1+GK))
 *   KS = control sensitivity
 *   W₁, W₂, W₃ = weighting functions
 * ```
 * 
 * ### Weighting Function Design
 * 
 * **W₁ (Sensitivity Weight)**
 * - Large at low frequencies → good tracking/rejection
 * - Roll off at high frequencies
 * - Typical: W₁(s) = (s/M + ω_B) / (s + ω_B·ε)
 * 
 * **W₂ (Control Weight)**
 * - Limits control effort
 * - Penalize high-frequency control
 * 
 * **W₃ (Complementary Sensitivity Weight)**
 * - Large at high frequencies → noise rejection
 * - Robustness to multiplicative uncertainty
 * 
 * ## Usage Examples
 * 
 * ### H2 Controller Design
 * ```cpp
 * H2Controller h2;
 * 
 * // Set generalized plant
 * h2.setGeneralizedPlant(A, B1, B2, C1, C2, D11, D12, D21, D22, dims);
 * 
 * // Design controller
 * h2.design();
 * 
 * // Control loop
 * ControllerInput input;
 * input.measured = y;
 * auto output = h2.compute(input);
 * ```
 * 
 * ### H∞ Mixed Sensitivity
 * ```cpp
 * HInfinityController hinf;
 * 
 * // Set plant model
 * hinf.setPlant(A, B, C, D, n, m, p);
 * 
 * // Configure weighting functions
 * hinf.setSensitivityWeight(M, omegaB, epsilon);  // W1
 * hinf.setControlWeight(maxControl);               // W2
 * hinf.setComplementaryWeight(omegaT);            // W3
 * 
 * // Design with specified gamma
 * hinf.design(gammaTarget);  // Or use hinf.designOptimal()
 * 
 * // Use in control loop
 * auto output = hinf.compute(input);
 * ```
 * 
 * ### Loop Shaping
 * ```cpp
 * HInfinityController hinf;
 * 
 * // Pre and post compensators for loop shaping
 * hinf.setPreCompensator(W1_num, W1_den);
 * hinf.setPostCompensator(W2_num, W2_den);
 * 
 * // Design normalized coprime factor controller
 * hinf.designLoopShaping(epsilon);
 * ```
 * 
 * ## Implementation Notes
 * 
 * ### Numerical Considerations
 * - H∞ Riccati equations require care to avoid numerical issues
 * - γ-iteration needs good initial bounds
 * - Generalized eigenvalue problems are sensitive to scaling
 * 
 * ### Computational Complexity
 * - H2: Similar to LQG (two Riccati equations)
 * - H∞: More expensive (γ-iteration or LMI)
 * 
 * ### Real-Time Implementation
 * Once designed offline, the controller is a standard state-space
 * system that can be implemented efficiently online.
 * 
 * @see LQGController
 * @see StateSpaceControllers.hpp
 */

#pragma once

#include "ControllerBase.hpp"
#include "StateSpaceControllers.hpp"
#include <array>
#include <functional>

namespace tether::control {

// ============================================================================
// Weighting Function
// ============================================================================

/**
 * @brief Transfer function for weighting
 * 
 * Represents first or second order weighting functions commonly
 * used in H∞ design.
 */
struct WeightingFunction {
    /**
     * @brief First-order weight: W(s) = (s + z) / (s + p)
     */
    static WeightingFunction firstOrder(double zero, double pole, double gain = 1.0);
    
    /**
     * @brief Integrating weight: W(s) = k/s
     */
    static WeightingFunction integrator(double gain);
    
    /**
     * @brief Sensitivity weight template
     * 
     * W₁(s) = (s/M + ωB) / (s + ωB·ε)
     * 
     * @param M Maximum sensitivity (typically 1.5-2)
     * @param omegaB Bandwidth [rad/s]
     * @param epsilon Low-frequency gain (typically 0.001)
     */
    static WeightingFunction sensitivity(double M, double omegaB, double epsilon);
    
    /**
     * @brief High-pass weight for T (complementary sensitivity)
     * 
     * W₃(s) = (s + ωT/M) / (ε·s + ωT)
     * 
     * @param M Low-frequency gain (typically 0.5-1)
     * @param omegaT Rolloff frequency [rad/s]
     * @param epsilon High-frequency gain (typically 0.01)
     */
    static WeightingFunction complementary(double M, double omegaT, double epsilon);
    
    // Transfer function coefficients (second order max)
    double num[3]{1, 0, 0};  // Numerator: num[0]*s² + num[1]*s + num[2]
    double den[3]{1, 0, 0};  // Denominator
    int order{0};
    double gain{1.0};
    
    /**
     * @brief Evaluate magnitude at frequency
     */
    double magnitude(double omega) const;
    
    /**
     * @brief Get state-space realization
     */
    void toStateSpace(double* A, double* B, double* C, double* D, int& n) const;
};

// ============================================================================
// H2 Optimal Controller
// ============================================================================

/**
 * @brief H2 Optimal Controller
 * 
 * Minimizes H2 norm of closed-loop transfer function.
 * Equivalent to LQG for standard regulation problems.
 * 
 * ## Generalized Plant Setup
 * ```
 * ẋ  = A·x  + B₁·w  + B₂·u
 * z  = C₁·x + D₁₁·w + D₁₂·u
 * y  = C₂·x + D₂₁·w + D₂₂·u
 * ```
 * 
 * ## H2 Assumptions
 * - D₁₁ = 0 (no direct feedthrough from w to z)
 * - D₂₂ = 0 (strictly proper controller)
 * - (A, B₂) stabilizable
 * - (C₂, A) detectable
 */
class H2Controller : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::H2; }
    const char* getName() const override { return "H2 Optimal Controller"; }
    const char* getDescription() const override {
        return "H2 optimal control minimizes RMS output for white noise input. "
               "Equivalent to LQG. Use for stochastic disturbance rejection. "
               "Set up generalized plant P, then design.";
    }
    
    /**
     * @brief Set generalized plant matrices
     * 
     * @param A System matrix (n×n)
     * @param B1 Disturbance input matrix (n×nw)
     * @param B2 Control input matrix (n×nu)
     * @param C1 Performance output matrix (nz×n)
     * @param C2 Measurement output matrix (ny×n)
     * @param D11 Feedthrough w→z (nz×nw), should be zero for H2
     * @param D12 Feedthrough u→z (nz×nu)
     * @param D21 Feedthrough w→y (ny×nw)
     * @param D22 Feedthrough u→y (ny×nu), should be zero
     * @param n, nw, nu, nz, ny Dimensions
     */
    void setGeneralizedPlant(const double* A, const double* B1, const double* B2,
                             const double* C1, const double* C2,
                             const double* D11, const double* D12,
                             const double* D21, const double* D22,
                             int n, int nw, int nu, int nz, int ny);
    
    /**
     * @brief Set simple plant for standard regulation
     * 
     * Automatically constructs generalized plant for tracking:
     * - z = [Q^½·e; R^½·u] (weighted error and control)
     * - w = [ref; dist; noise]
     * 
     * @param A, B, C, D Plant matrices
     * @param Q State/output weight
     * @param R Control weight
     * @param W Process noise intensity
     * @param V Measurement noise intensity
     */
    void setRegulatorProblem(const double* A, const double* B, const double* C,
                             const double* Q, const double* R,
                             const double* W, const double* V,
                             int n, int m, int p);
    
    /**
     * @brief Design the H2 optimal controller
     * @return true if design succeeded
     */
    bool design();
    
    /**
     * @brief Get achieved H2 norm
     */
    double getH2Norm() const { return m_h2Norm; }
    
    /**
     * @brief Get controller state-space matrices
     */
    void getControllerMatrices(double* Ak, double* Bk, double* Ck, double* Dk) const;
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_nw{0}, m_nu{0}, m_nz{0}, m_ny{0};
    
    // Generalized plant matrices
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_B1{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B2{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C1{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C2{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_D12{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_D21{};
    
    // Controller matrices (state-space)
    int m_nk{0};  // Controller order
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_Ak{};
    std::array<double, MAX_STATE_DIM * MAX_OUTPUT_DIM> m_Bk{};
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> m_Ck{};
    std::array<double, MAX_CONTROL_DIM * MAX_OUTPUT_DIM> m_Dk{};
    
    // Controller state
    std::array<double, MAX_STATE_DIM> m_xk{};
    
    // Performance
    double m_h2Norm{0.0};
    bool m_designed{false};
};

// ============================================================================
// H∞ Controller
// ============================================================================

/**
 * @brief H∞ (H-Infinity) Robust Controller
 * 
 * Minimizes worst-case (peak) gain from disturbance to output.
 * Provides robustness to model uncertainty.
 * 
 * ## Problem
 * Find K such that ||F_l(P, K)||∞ < γ
 * 
 * ## Solution
 * Uses γ-iteration: for given γ, check if Riccati equations
 * have stabilizing solutions with required properties.
 * 
 * ## Mixed Sensitivity
 * Common design approach:
 * ```
 * ||W₁·S  ||
 * ||W₂·KS || < γ
 * ||W₃·T  ||∞
 * ```
 */
class HInfinityController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::HInfinity; }
    const char* getName() const override { return "H∞ Controller"; }
    const char* getDescription() const override {
        return "H-infinity robust control minimizes worst-case gain. "
               "Use when uncertain about model or facing worst-case disturbances. "
               "Mixed sensitivity design via weighting functions W₁, W₂, W₃.";
    }
    
    /**
     * @brief Set plant model
     * @param A, B, C, D State-space matrices
     * @param n States, m Inputs, p Outputs
     */
    void setPlant(const double* A, const double* B, const double* C, const double* D,
                  int n, int m, int p);
    
    /**
     * @brief Set generalized plant directly
     */
    void setGeneralizedPlant(const double* A, const double* B1, const double* B2,
                             const double* C1, const double* C2,
                             const double* D11, const double* D12,
                             const double* D21, const double* D22,
                             int n, int nw, int nu, int nz, int ny);
    
    /**
     * @brief Set sensitivity weight W₁
     * 
     * W₁(s) = (s/M + ωB) / (s + ωB·ε)
     * 
     * @param M Maximum sensitivity (peak of |S|)
     * @param omegaB Bandwidth [rad/s]
     * @param epsilon Steady-state sensitivity (small)
     */
    void setSensitivityWeight(double M, double omegaB, double epsilon);
    
    /**
     * @brief Set control effort weight W₂
     * @param maxControl Maximum expected control magnitude
     */
    void setControlWeight(double maxControl);
    
    /**
     * @brief Set complementary sensitivity weight W₃
     * @param omegaT High-frequency cutoff
     */
    void setComplementaryWeight(double M, double omegaT, double epsilon);
    
    /**
     * @brief Set custom weighting functions
     */
    void setWeights(const WeightingFunction& W1, 
                    const WeightingFunction& W2,
                    const WeightingFunction& W3);
    
    /**
     * @brief Design controller for specified γ
     * @param gamma Performance level
     * @return true if design succeeded
     */
    bool design(double gamma);
    
    /**
     * @brief Find optimal (minimum) γ and design controller
     * @param tol Tolerance for γ search
     * @return Achieved γ
     */
    double designOptimal(double tol = 0.01);
    
    /**
     * @brief Check if γ is achievable
     * @param gamma Performance level to test
     * @return true if controller exists for this γ
     */
    bool isAchievable(double gamma);
    
    /**
     * @brief Get achieved H∞ norm (gamma)
     */
    double getGamma() const { return m_gamma; }
    
    /**
     * @brief Get controller state-space matrices
     */
    void getControllerMatrices(double* Ak, double* Bk, double* Ck, double* Dk) const;
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    int m_n{0}, m_m{0}, m_p{0};
    int m_nw{0}, m_nu{0}, m_nz{0}, m_ny{0};
    
    // Plant matrices
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
    std::array<double, MAX_OUTPUT_DIM * MAX_CONTROL_DIM> m_D{};
    
    // Weighting functions
    WeightingFunction m_W1, m_W2, m_W3;
    bool m_hasW1{false}, m_hasW2{false}, m_hasW3{false};
    
    // Augmented plant (with weights)
    static constexpr int MAX_AUG = 2 * MAX_STATE_DIM;
    int m_na{0};  // Augmented state dimension
    std::array<double, MAX_AUG * MAX_AUG> m_Aa{};
    std::array<double, MAX_AUG * MAX_STATE_DIM> m_B1a{};
    std::array<double, MAX_AUG * MAX_CONTROL_DIM> m_B2a{};
    std::array<double, MAX_AUG * MAX_AUG> m_C1a{};
    std::array<double, MAX_OUTPUT_DIM * MAX_AUG> m_C2a{};
    
    // Controller matrices
    int m_nk{0};
    std::array<double, MAX_AUG * MAX_AUG> m_Ak{};
    std::array<double, MAX_AUG * MAX_OUTPUT_DIM> m_Bk{};
    std::array<double, MAX_CONTROL_DIM * MAX_AUG> m_Ck{};
    std::array<double, MAX_CONTROL_DIM * MAX_OUTPUT_DIM> m_Dk{};
    
    // Controller state
    std::array<double, MAX_AUG> m_xk{};
    
    double m_gamma{0.0};
    bool m_designed{false};
    
    // Internal methods
    void buildAugmentedPlant();
    bool solveHinfRiccati(double gamma, double* X, double* Y);
    void computeController(double gamma, const double* X, const double* Y);
};

// ============================================================================
// μ-Synthesis Framework (Structured Uncertainty)
// ============================================================================

/**
 * @brief Structured Singular Value (μ) Analysis
 * 
 * μ-analysis and D-K iteration for structured uncertainty.
 * This is a framework class; full implementation would require
 * external optimization libraries.
 * 
 * ## Structured Uncertainty
 * Unlike H∞ which handles unstructured uncertainty, μ handles
 * uncertainty with known structure (diagonal, repeated blocks, etc.)
 * 
 * ## D-K Iteration
 * 1. Fix D, synthesize K (H∞ problem)
 * 2. Fix K, fit D (curve fitting)
 * 3. Iterate until convergence
 */
class MuSynthesisFramework {
public:
    /**
     * @brief Define uncertainty structure
     * @param blockSizes Sizes of uncertainty blocks
     * @param numBlocks Number of blocks
     * @param repeated Which blocks are repeated scalars
     */
    void setUncertaintyStructure(const int* blockSizes, int numBlocks,
                                  const bool* repeated = nullptr);
    
    /**
     * @brief Compute μ upper bound for a transfer function
     * @param M Transfer function matrix at frequency ω
     * @param n Matrix dimension
     * @return Upper bound on μ
     */
    double computeMuUpperBound(const double* M, int n) const;
    
    /**
     * @brief Perform one D-K iteration
     */
    bool dkIteration();
    
private:
    std::array<int, 10> m_blockSizes{};
    std::array<bool, 10> m_repeated{};
    int m_numBlocks{0};
};

} // namespace tether::control
