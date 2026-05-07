/**
 * @file FractionalPID.hpp
 * @brief Fractional-Order PID (FOPID) Controller
 * 
 * @details
 * This file implements the Fractional-Order PID controller, also known as
 * PI^λ D^μ controller, which generalizes classical PID by using fractional
 * calculus for the integral and derivative operations.
 * 
 * ## Mathematical Foundation
 * 
 * ### Fractional Calculus
 * Fractional calculus extends differentiation and integration to non-integer
 * orders. The Grünwald-Letnikov (GL) definition is commonly used:
 * 
 * ```
 *        1      n   (-1)^k (α)
 * D^α = ─── ⋅ Σ    ─────── ⋅ f(t - k⋅h)
 *       h^α  k=0      k
 * 
 * Where:
 *   α = fractional order
 *   h = sampling period
 *   (α k) = generalized binomial coefficient
 * ```
 * 
 * ### Fractional PID Control Law
 * ```
 *                   Ki
 * u(s) = Kp⋅e(s) + ──── ⋅ e(s) + Kd⋅s^μ⋅e(s)
 *                  s^λ
 * 
 * Where:
 *   λ = integral order (typically 0 < λ ≤ 1)
 *   μ = derivative order (typically 0 < μ ≤ 1)
 * ```
 * 
 * When λ = μ = 1, this reduces to standard PID.
 * 
 * ## Why Use Fractional PID?
 * 
 * ### Advantages
 * 1. **More tuning parameters**: 5 instead of 3 (Kp, Ki, Kd, λ, μ)
 * 2. **Better robustness**: Can achieve iso-damping (constant phase margin)
 * 3. **Better disturbance rejection**: Fractional integrator more effective
 * 4. **Better noise rejection**: Fractional derivative less sensitive
 * 5. **Better fit for fractional systems**: Many real systems have fractional dynamics
 * 
 * ### When to Use
 * - Systems with long memory effects (viscoelastic materials)
 * - Thermal systems (heat diffusion is fractional)
 * - Battery charge estimation
 * - Processes where classical PID doesn't achieve desired robustness
 * - When iso-damping property is desired
 * 
 * ## Implementation Methods
 * 
 * ### 1. Oustaloup Recursive Approximation
 * Approximates s^α using recursive pole-zero placement:
 * ```
 *        N   s + ω_k'
 * s^α ≈ K∏  ─────────
 *       k=1 s + ω_k
 * ```
 * 
 * ### 2. Grünwald-Letnikov Discretization
 * Direct discrete implementation using truncated GL series.
 * 
 * ### 3. Short-Memory Principle
 * Truncates GL series for real-time implementation.
 * 
 * ## Usage Examples
 * 
 * ### Basic FOPID
 * ```cpp
 * FractionalPIDController fopid;
 * fopid.setGains(1.0, 0.5, 0.2);    // Kp, Ki, Kd
 * fopid.setOrders(0.8, 0.6);         // λ (integral), μ (derivative)
 * fopid.setMemoryLength(100);        // GL truncation length
 * 
 * // Control loop
 * ControllerInput input;
 * input.reference = 100.0;
 * input.measured = actualValue;
 * input.dt = 0.01;  // 10ms
 * 
 * auto output = fopid.compute(input);
 * actuator.set(output.control);
 * ```
 * 
 * ### Robust Temperature Control
 * ```cpp
 * FractionalPIDController fopid;
 * 
 * // For thermal systems, λ < 1 often works better
 * fopid.setGains(2.0, 0.1, 0.05);
 * fopid.setOrders(0.7, 0.5);  // Fractional integral, fractional derivative
 * 
 * // Use Oustaloup approximation for frequency domain design
 * fopid.setApproximationMethod(FractionalApproximation::Oustaloup);
 * fopid.setOustaloupParams(0.001, 1000, 5);  // ωl, ωh, N
 * ```
 * 
 * ### Auto-Tuning for Iso-Damping
 * ```cpp
 * FractionalPIDController fopid;
 * 
 * // Auto-tune for constant phase margin
 * fopid.autoTuneIsoDamping(
 *     processGain,      // K
 *     processLag,       // T
 *     desiredBandwidth, // ωc
 *     desiredPhaseMargin // φm (typically 45-60°)
 * );
 * ```
 * 
 * ## Implementation Notes
 * 
 * ### Memory Requirements
 * The GL discretization requires storing L past error values:
 * ```
 * Memory ≈ L × sizeof(double) bytes
 * ```
 * For L=100 and 8-byte doubles: ~800 bytes.
 * 
 * ### Computational Cost
 * Each control cycle requires O(L) operations for each fractional term.
 * 
 * @see PIDController
 * @see ControllerBase
 */

#pragma once

#include "ControllerBase.hpp"
#include <vector>
#include <deque>

namespace Control {

/**
 * @brief Approximation method for fractional operators
 */
enum class FractionalApproximation {
    GrunwaldLetnikov,  ///< Direct GL discretization (time domain)
    Oustaloup,         ///< Oustaloup recursive filter (frequency domain)
    Matsuda,           ///< Matsuda fitting method
    ShortMemory        ///< GL with short memory principle
};

/**
 * @brief Fractional-Order PID Controller (PI^λ D^μ)
 * 
 * This controller extends classical PID with fractional-order integral
 * and derivative operators, providing additional tuning flexibility
 * and potentially better robustness characteristics.
 */
class FractionalPIDController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::FractionalPID; }
    const char* getName() const override { return "Fractional PID (PI^λ D^μ)"; }
    const char* getDescription() const override {
        return "Fractional-order PID using non-integer integral (λ) and "
               "derivative (μ) orders. Provides better robustness and "
               "5 tuning parameters. Uses Grünwald-Letnikov discretization.";
    }
    
    // ========================================================================
    // Gain and Order Configuration
    // ========================================================================
    
    /**
     * @brief Set PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(double kp, double ki, double kd);
    
    /**
     * @brief Set fractional orders
     * @param lambda Integral order λ (0 < λ ≤ 1, typically 0.7-1.0)
     * @param mu Derivative order μ (0 < μ ≤ 1, typically 0.5-1.0)
     * 
     * When λ = μ = 1, controller behaves like classical PID.
     */
    void setOrders(double lambda, double mu);
    
    /**
     * @brief Get integral order
     */
    double getLambda() const { return m_lambda; }
    
    /**
     * @brief Get derivative order  
     */
    double getMu() const { return m_mu; }
    
    // ========================================================================
    // Approximation Configuration
    // ========================================================================
    
    /**
     * @brief Set approximation method
     * @param method Approximation method to use
     */
    void setApproximationMethod(FractionalApproximation method);
    
    /**
     * @brief Set memory length for GL discretization
     * @param length Number of past samples to use (trade-off: accuracy vs memory)
     * 
     * Typical values: 50-200. Longer = more accurate but more computation.
     */
    void setMemoryLength(size_t length);
    
    /**
     * @brief Get memory length
     */
    size_t getMemoryLength() const { return m_memoryLength; }
    
    /**
     * @brief Set Oustaloup approximation parameters
     * @param omegaLow Lower frequency bound [rad/s]
     * @param omegaHigh Upper frequency bound [rad/s]
     * @param order Approximation order (number of poles/zeros)
     */
    void setOustaloupParams(double omegaLow, double omegaHigh, int order);
    
    // ========================================================================
    // Auto-Tuning
    // ========================================================================
    
    /**
     * @brief Auto-tune for iso-damping (constant phase margin)
     * 
     * Iso-damping means the closed-loop system maintains constant
     * overshoot regardless of process gain variations.
     * 
     * @param processGain Process static gain K
     * @param processLag Process time constant T [s]
     * @param bandwidth Desired crossover frequency ωc [rad/s]
     * @param phaseMargin Desired phase margin [degrees]
     */
    void autoTuneIsoDamping(double processGain, double processLag,
                            double bandwidth, double phaseMargin);
    
    /**
     * @brief Auto-tune using SIMC-like rules adapted for FOPID
     */
    void autoTuneSIMC(double processGain, double timeConstant, double deadTime);
    
    // ========================================================================
    // Anti-Windup
    // ========================================================================
    
    /**
     * @brief Set anti-windup method
     */
    void setAntiWindup(AntiWindupMethod method, double param = 0.0);
    
    /**
     * @brief Set integral limits
     */
    void setIntegralLimits(double min, double max);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    // Gains
    double m_kp{1.0};
    double m_ki{0.0};
    double m_kd{0.0};
    
    // Fractional orders
    double m_lambda{1.0};  // Integral order
    double m_mu{1.0};      // Derivative order
    
    // Approximation settings
    FractionalApproximation m_approxMethod{FractionalApproximation::GrunwaldLetnikov};
    size_t m_memoryLength{100};
    
    // Oustaloup parameters
    double m_omegaLow{0.001};
    double m_omegaHigh{1000.0};
    int m_oustaloupOrder{5};
    
    // Anti-windup
    AntiWindupMethod m_antiWindup{AntiWindupMethod::Clamping};
    double m_antiWindupParam{0.0};
    double m_integralMin{-std::numeric_limits<double>::max()};
    double m_integralMax{std::numeric_limits<double>::max()};
    
    // State for GL discretization
    std::deque<double> m_errorHistory;
    std::vector<double> m_integralCoeffs;  // GL coefficients for integral
    std::vector<double> m_derivCoeffs;     // GL coefficients for derivative
    
    // State for Oustaloup approximation
    std::vector<double> m_integralState;
    std::vector<double> m_derivState;
    
    // Accumulated values
    double m_fractionalIntegral{0.0};
    double m_fractionalDerivative{0.0};
    double m_lastOutput{0.0};
    
    // Internal methods
    void computeGLCoefficients();
    double computeFractionalIntegral(double dt);
    double computeFractionalDerivative(double dt);
    double computeOustaloupIntegral(double error, double dt);
    double computeOustaloupDerivative(double error, double dt);
    
    /**
     * @brief Compute binomial coefficient for fractional order
     * 
     * Generalized binomial coefficient:
     * (α)     α(α-1)(α-2)...(α-k+1)
     * ( ) = ─────────────────────────
     * (k)           k!
     */
    static double binomialCoeff(double alpha, int k);
};

/**
 * @brief Oustaloup Filter for fractional operator approximation
 * 
 * This class implements the Oustaloup recursive approximation for
 * fractional-order operators s^α.
 * 
 * ## Transfer Function
 * ```
 *        N   s + ω_k'
 * s^α ≈ K∏  ─────────
 *       k=1 s + ω_k
 * 
 * Where:
 *   ω_k = ωb⋅(ωh/ωb)^((k+N+(1-α)/2)/(2N+1))   (poles)
 *   ω_k'= ωb⋅(ωh/ωb)^((k+N+(1+α)/2)/(2N+1))   (zeros)
 *   K = ωh^α                                   (gain)
 * ```
 */
class OustaloupFilter {
public:
    /**
     * @brief Configure the filter
     * @param alpha Fractional order (-1 < α < 1)
     * @param omegaLow Lower frequency [rad/s]
     * @param omegaHigh Upper frequency [rad/s]
     * @param order Number of poles/zeros (typically 3-7)
     */
    void configure(double alpha, double omegaLow, double omegaHigh, int order);
    
    /**
     * @brief Process a sample through the filter
     * @param input Current input value
     * @param dt Sampling period [s]
     * @return Filtered output
     */
    double process(double input, double dt);
    
    /**
     * @brief Reset filter state
     */
    void reset();
    
private:
    double m_alpha{1.0};
    double m_omegaLow{0.001};
    double m_omegaHigh{1000.0};
    int m_order{5};
    
    std::vector<double> m_poles;
    std::vector<double> m_zeros;
    double m_gain{1.0};
    
    std::vector<double> m_state;  // Filter state
};

} // namespace Control
