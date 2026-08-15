/**
 * @file ArxLpvInverseFilter.hpp
 * @brief Time-domain LPV inverse IIR filter with ARX model interpolation.
 *
 * @details
 * Represents the LPV system as a parameter-varying ARX (Autoregressive with
 * Exogenous Input) transfer function:
 *
 *   A(z, p) y[n] = z^{-d} B(z, p) x[n]
 *
 * where:
 *   A(z, p) = 1 + a_1(p) z^{-1} + ... + a_{N_a}(p) z^{-N_a}
 *   B(z, p) = b_0(p) + b_1(p) z^{-1} + ... + b_{N_b}(p) z^{-N_b}
 *   d       = discrete transport delay (steps)
 *
 * Deconvolution (inversion) solves for x[n] given y_tgt[n]:
 *
 *   x_req[n] = (1 / b_0(p[n])) · ( y_tgt[n+d]
 *             + Σ_{i=1}^{N_a} a_i(p[n]) · y_tgt[n+d-i]
 *             − Σ_{j=1}^{N_b} b_j(p[n]) · x_req[n-j] )
 *
 * The delay d is explicitly factored out so that b_0 is the first non-zero
 * coefficient of B'(z) = z^d · B(z).  This avoids the instability trap of
 * non-minimum-phase zeros (pure transport delay).
 *
 * At runtime, the ARX coefficients are linearly interpolated from a LUT of
 * identified model points.  Ring buffers maintain past y_tgt and x_req
 * values for the recursive computation.
 *
 * This approach is best for bare-metal MCUs processing step-by-step streams
 * at high loop frequencies (e.g., 1 kHz).
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md
 */

#pragma once

#include <deque>
#include <limits>
#include <map>
#include <vector>

namespace tether::control::extrusion {

/// @brief ARX model identified at a single operating point.
struct ArxLpvModelPoint {
    double parameter = 0.0;            ///< Scheduling parameter p
    std::vector<double> aCoeffs;       ///< A(z) = 1 + a_1 z^{-1} + ... + a_{Na} z^{-Na}
    std::vector<double> bCoeffs;       ///< B'(z) = b_0 + b_1 z^{-1} + ... + b_{Nb} z^{-Nb}
    int delay = 0;                     ///< Transport delay d (steps)

    /// @brief Default constructor for container compatibility.
    ArxLpvModelPoint() = default;
    ArxLpvModelPoint(double p, std::vector<double> a, std::vector<double> b, int d)
        : parameter(p), aCoeffs(std::move(a)), bCoeffs(std::move(b)), delay(d) {}
};

/// @brief Time-domain LPV inverse IIR filter.
///
/// Processes one sample at a time using the algebraic inverse of the ARX
/// difference equation.  Coefficients are interpolated from a LUT of
/// identified model points.
class ArxLpvInverseFilter {
public:
    /// @brief Construct with polynomial orders.
    /// @param na Order of A(z) (autoregressive part).
    /// @param nb Order of B'(z) (exogenous part, excluding delay).
    ArxLpvInverseFilter(int na = 0, int nb = 0);

    /// @brief Add an identified ARX model at operating point p.
    void addModelPoint(const ArxLpvModelPoint& point);

    /// @brief Add an identified ARX model (convenience overload).
    void addModelPoint(double p, std::vector<double> a,
                       std::vector<double> b, int delay = 0);

    /// @brief Process one sample.
    /// @param yTargetCurrent y_tgt[n] (current target).
    /// @param yTargetAhead y_tgt[n+d] (lookahead target, d = current delay).
    /// @param pCurrent Current scheduling parameter p[n].
    /// @return x_req[n] (required input at step n).
    double process(double yTargetCurrent, double yTargetAhead, double pCurrent);

    /// @brief Process a full trajectory with lookahead.
    /// @param yTarget Target trajectory y_tgt[0..N-1].
    /// @param p Scheduling parameter trajectory p[0..N-1].
    /// @return Required input x_req[0..N-1].
    std::vector<double> process(const std::vector<double>& yTarget,
                                const std::vector<double>& p);

    /// @brief Reset the filter state (ring buffers).
    void reset();

    /// @return Number of model points in the LUT.
    size_t numModelPoints() const { return modelLut_.size(); }

    /// @return Current delay d (from the last interpolated model).
    int currentDelay() const { return currentDelay_; }

private:
    int na_;  ///< Order of A(z)
    int nb_;  ///< Order of B'(z)
    std::map<double, ArxLpvModelPoint> modelLut_;

    // Ring buffers for past values
    std::deque<double> yTargetHistory_;  ///< y_tgt[n-1], y_tgt[n-2], ...
    std::deque<double> xReqHistory_;     ///< x_req[n-1], x_req[n-2], ...

    // Last interpolated coefficients (cached for process() calls)
    std::vector<double> currentA_;
    std::vector<double> currentB_;
    int currentDelay_ = 0;
    double lastP_ = std::numeric_limits<double>::quiet_NaN();

    /// @brief Interpolate ARX coefficients at scheduling parameter p.
    void interpolateCoefficients(double p);

    /// @brief Ensure ring buffers have the right size.
    void ensureBufferSize();
};

} // namespace tether::control::extrusion
