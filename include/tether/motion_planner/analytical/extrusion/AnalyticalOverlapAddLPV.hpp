/**
 * @file AnalyticalOverlapAddLPV.hpp
 * @brief Analytical gain-scheduled LPV deconvolution on WSS arcs.
 *
 * @details
 * The LPV system varies with scheduling parameter p(t) = v(t) (path
 * velocity).  The gain-scheduled regularized inverse h_inv(p) is
 * interpolated from a LUT of operating points.
 *
 * Within each WSS arc, the scheduling parameter p(t) = v(t) is polynomial.
 * We approximate p as constant within the arc, using the arc's
 * time-averaged velocity:
 *
 *   p̄_arc = (1/Δt) ∫₀^Δt v(τ) dτ = Δs / Δt
 *
 * The inverse filter h_inv(p̄_arc) is then constant within the arc, and
 * the convolution reduces to the LTI case:
 *
 *   x(t) = Σ c_k(t) · M_k(p̄_arc)
 *
 * where M_k(p) = ∫ h_inv(p, τ) τ^k dτ are precomputed moments for each
 * LUT entry, linearly interpolated at p̄_arc.
 *
 * First-order correction (optional): account for p variation within the
 * arc by linearizing h_inv around p̄.
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §5
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"
#include "AnalyticalLTIDeconvolution.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for analytical overlap-add LPV deconvolution.
 */
struct AnalyticalOverlapAddLPVParams {
    /// Tikhonov regularization λ for inverse filter precomputation
    double lambda = 1e-6;

    /// Maximum polynomial degree K of the target trajectory
    int maxPolyDegree = 3;

    /// Group delay [s]
    double groupDelay = 0.0;

    /// If true, apply first-order correction for p variation within arcs
    bool firstOrderCorrection = false;
};

/**
 * @brief Analytical gain-scheduled LPV deconvolution.
 *
 * Stores a LUT of regularized inverse filters (as moment vectors) indexed
 * by scheduling parameter.  At runtime, the target trajectory is evaluated
 * per-arc using the arc-averaged scheduling parameter.
 */
template<size_t Dim, typename T = double>
class AnalyticalOverlapAddLPV {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from trajectory and parameters.
     *
     * Operating points must be added via addOperatingPoint() before use.
     */
    AnalyticalOverlapAddLPV(const Traj& traj,
                             AnalyticalOverlapAddLPVParams params)
        : traj_(&traj), params_(params) {}

    /**
     * @brief Add an operating point to the inverse-filter LUT.
     *
     * @param p Scheduling parameter value (e.g., nominal path velocity)
     * @param h Measured impulse response at this operating point
     * @param sampleRate Sampling rate of h [Hz]
     */
    void addOperatingPoint(double p, const std::vector<double>& h,
                           double sampleRate) {
        // Compute regularized inverse and moments
        AnalyticalLTIDeconvParams ltiParams;
        ltiParams.lambda = params_.lambda;
        ltiParams.maxPolyDegree = params_.maxPolyDegree;
        ltiParams.groupDelay = params_.groupDelay;

        // Use a temporary deconvolver to compute the inverse and moments
        AnalyticalLTIDeconvolution<Dim, T> deconv(*traj_, h, sampleRate,
                                                    ltiParams);
        momentLut_[p] = deconv.moments();
        inverseLut_[p] = deconv.inverseImpulseResponse();
        sampleRate_ = sampleRate;
    }

    /**
     * @brief Compute the required input x(t) at time t.
     *
     * @param t Query time [s]
     * @param usePosition If true, y(t) = extruder position; else velocity
     */
    double inputAtTime(double t, bool usePosition = false) const {
        if (momentLut_.empty()) return 0.0;

        double tEff = t - params_.groupDelay;
        if (tEff < 0.0) return 0.0;

        const auto& arcs = traj_->arcs();
        if (arcs.empty()) return 0.0;

        size_t idx = traj_->findArc(tEff);
        const auto& a = arcs[idx];
        double tau = std::clamp(tEff - a.t0, 0.0, a.duration);

        // Arc-averaged scheduling parameter (path velocity)
        double pBar = a.avgPathVelocity();

        // Interpolate moments at pBar
        std::vector<double> moments = interpolateMoments(pBar);
        if (moments.empty()) return 0.0;

        // Compute polynomial coefficients of y(t-τ) as a function of τ
        if (usePosition) {
            double eAtT = traj_->extruderPositionAtTime(tEff);
            std::vector<double> coeffs(5, 0.0);
            coeffs[0] = eAtT;
            coeffs[1] = -a.extrusionRatio * a.c0;
            coeffs[2] = a.extrusionRatio * 0.5 * a.c1;
            coeffs[3] = -a.extrusionRatio * (1.0 / 3.0) * a.c2;
            coeffs[4] = a.extrusionRatio * 0.25 * a.c3;

            double x = 0.0;
            for (int k = 0; k <= 4 && k < static_cast<int>(moments.size()); ++k)
                x += coeffs[k] * moments[k];
            return x;
        } else {
            // y(t) = extruder velocity = piecewise polynomial of degree 3
            // v_e(t-τ) = α_e · v_local(tau - τ)  where tau = local time
            double tau2 = tau * tau;
            std::vector<double> coeffs(4, 0.0);
            coeffs[0] = a.extrusionRatio * (a.c0 + a.c1 * tau
                         + a.c2 * tau2 + a.c3 * tau2 * tau);
            coeffs[1] = a.extrusionRatio * (-a.c1 - 2.0 * a.c2 * tau
                         - 3.0 * a.c3 * tau2);
            coeffs[2] = a.extrusionRatio * (a.c2 + 3.0 * a.c3 * tau);
            coeffs[3] = a.extrusionRatio * (-a.c3);

            double x = 0.0;
            for (int k = 0; k <= 3 && k < static_cast<int>(moments.size()); ++k)
                x += coeffs[k] * moments[k];

            // First-order correction (optional)
            if (params_.firstOrderCorrection && momentLut_.size() >= 2) {
                x += firstOrderCorrectionTerm(a, tau, moments, usePosition);
            }
            return x;
        }
    }

    /**
     * @brief Compute the required input at multiple time points.
     */
    std::vector<double> inputSeries(const std::vector<double>& times,
                                     bool usePosition = false) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(inputAtTime(t, usePosition));
        return result;
    }

    /**
     * @brief Compute the adjusted extruder position at time t.
     */
    double adjustedExtruderPosition(double t) const {
        return traj_->extruderPositionAtTime(t) + inputAtTime(t, true);
    }

    /**
     * @brief Compute the adjusted extruder position at multiple times.
     */
    std::vector<double> adjustedExtruderPositionSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(adjustedExtruderPosition(t));
        return result;
    }

    /// Number of operating points
    size_t numOperatingPoints() const { return momentLut_.size(); }

    /// Parameters
    const AnalyticalOverlapAddLPVParams& params() const { return params_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalOverlapAddLPVParams params_;
    std::map<double, std::vector<double>> momentLut_;
    std::map<double, std::vector<double>> inverseLut_;
    double sampleRate_ = 0.0;

    std::vector<double> interpolateMoments(double p) const {
        if (momentLut_.empty()) return {};
        if (momentLut_.size() == 1) return momentLut_.begin()->second;

        auto upper = momentLut_.lower_bound(p);
        if (upper == momentLut_.begin()) return upper->second;
        if (upper == momentLut_.end()) return std::prev(upper)->second;

        auto lower = std::prev(upper);
        double p0 = lower->first, p1 = upper->first;
        double frac = (p - p0) / (p1 - p0);

        const auto& m0 = lower->second;
        const auto& m1 = upper->second;
        std::vector<double> result(m0.size());
        for (size_t i = 0; i < m0.size(); ++i)
            result[i] = m0[i] + frac * (m1[i] - m0[i]);
        return result;
    }

    double firstOrderCorrectionTerm(const ExtrusionArc& a, double tau,
                                     const std::vector<double>& moments,
                                     bool usePosition) const {
        // ∂M_k/∂p ≈ (M_k(p1) - M_k(p0)) / (p1 - p0)
        // Correction: Σ c_k · ∂M_k/∂p · (p(t) - p̄)
        // p(t) - p̄ = v(τ) - ṽ = (c0 + c1·τ + c2·τ² + c3·τ³) - ṽ
        double pBar = a.avgPathVelocity();
        double pDev = a.pathVelocity(tau) - pBar;

        // Estimate ∂M/∂p via finite difference from the LUT
        auto upper = momentLut_.lower_bound(pBar);
        if (upper == momentLut_.begin() || upper == momentLut_.end()) return 0.0;
        auto lower = std::prev(upper);
        double dp = upper->first - lower->first;
        if (std::abs(dp) < 1e-12) return 0.0;

        const auto& m0 = lower->second;
        const auto& m1 = upper->second;

        if (usePosition) {
            std::vector<double> coeffs(5, 0.0);
            coeffs[1] = -a.extrusionRatio * a.c0;
            coeffs[2] = a.extrusionRatio * 0.5 * a.c1;
            coeffs[3] = -a.extrusionRatio * (1.0 / 3.0) * a.c2;
            coeffs[4] = a.extrusionRatio * 0.25 * a.c3;
            double corr = 0.0;
            for (int k = 0; k <= 4 && k < static_cast<int>(m0.size()); ++k)
                corr += coeffs[k] * (m1[k] - m0[k]) / dp * pDev;
            return corr;
        } else {
            double tau2 = tau * tau;
            std::vector<double> coeffs(4, 0.0);
            coeffs[0] = a.extrusionRatio * (a.c0 + a.c1 * tau
                         + a.c2 * tau2 + a.c3 * tau2 * tau);
            coeffs[1] = a.extrusionRatio * (-a.c1 - 2.0 * a.c2 * tau
                         - 3.0 * a.c3 * tau2);
            coeffs[2] = a.extrusionRatio * (a.c2 + 3.0 * a.c3 * tau);
            coeffs[3] = a.extrusionRatio * (-a.c3);
            double corr = 0.0;
            for (int k = 0; k <= 3 && k < static_cast<int>(m0.size()); ++k)
                corr += coeffs[k] * (m1[k] - m0[k]) / dp * pDev;
            return corr;
        }
    }
};

} // namespace MotionPlanner::analytical::extrusion
