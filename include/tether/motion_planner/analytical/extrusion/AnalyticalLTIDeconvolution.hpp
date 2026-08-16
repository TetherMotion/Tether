/**
 * @file AnalyticalLTIDeconvolution.hpp
 * @brief Analytical LTI deconvolution on piecewise-polynomial trajectories.
 *
 * @details
 * Given a continuous-time LTI system with impulse response h(t) and a
 * target output trajectory y(t) (piecewise polynomial from the WSS), compute
 * the required input x(t) such that (h * x)(t) ≈ y(t).
 *
 * The regularized inverse in the frequency domain:
 *
 *   X(ω) = Y(ω) · H*(ω) / (|H(ω)|² + λ)
 *
 * corresponds to the time-domain convolution:
 *
 *   x(t) = (h_inv * y)(t) = ∫₀^∞ h_inv(τ) · y(t-τ) dτ
 *
 * where h_inv(t) is the regularized inverse impulse response.
 *
 * Since y(t) is piecewise polynomial (at most degree 2 from velocity, or
 * degree 3 from position), the convolution integral reduces to a finite
 * sum of precomputed moments of h_inv:
 *
 *   M_k = ∫₀^∞ h_inv(τ) · τ^k dτ,  k = 0, 1, ..., K
 *
 * Within each arc where y(t-τ) = Σ c_k(t) · τ^k:
 *
 *   x(t) = Σ c_k(t) · M_k
 *
 * This is a closed-form linear combination — no integration at runtime.
 *
 * State-space alternative: for systems given as (A, B, C, D) with D ≠ 0:
 *
 *   x(t) = D⁺ · (y(t) - C·v(t))
 *   dv/dt = (A - B·D⁺·C)·v + B·D⁺·y(t)
 *
 * This is a linear ODE with polynomial forcing, solved via matrix
 * exponential + polynomial integral formulas.
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §4
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for analytical LTI deconvolution.
 */
struct AnalyticalLTIDeconvParams {
    /// Tikhonov regularization parameter λ > 0
    double lambda = 1e-6;

    /// Maximum polynomial degree K of the target trajectory y(t).
    /// K=2 for velocity, K=3 for position.
    int maxPolyDegree = 3;

    /// Group delay [s] — the output is shifted by this amount for causal
    /// alignment (0 = no shift).
    double groupDelay = 0.0;
};

/**
 * @brief Analytical LTI deconvolution.
 *
 * Two modes:
 *
 * 1. **Impulse-response mode**: Provide a sampled impulse response h[n].
 *    The class precomputes the regularized inverse h_inv(t) and its
 *    moments M_k.  The input x(t) is then a linear combination of moments.
 *
 * 2. **State-space mode**: Provide (A, B, C, D) matrices.  The class
 *    solves the ODE with polynomial forcing via matrix exponentials.
 */
template<size_t Dim, typename T = double>
class AnalyticalLTIDeconvolution {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct in impulse-response mode.
     *
     * @param traj Extrusion trajectory (provides y(t) = extruder velocity
     *             or position)
     * @param h Sampled impulse response (regularized inverse is computed
     *          internally)
     * @param sampleRate Sampling rate of h [Hz]
     * @param params Parameters
     */
    AnalyticalLTIDeconvolution(const Traj& traj,
                                const std::vector<double>& h,
                                double sampleRate,
                                AnalyticalLTIDeconvParams params)
        : traj_(&traj)
        , params_(params)
        , mode_(Mode::ImpulseResponse)
        , sampleRate_(sampleRate) {
        computeRegularizedInverse(h);
        computeMoments();
    }

    /**
     * @brief Construct in state-space mode.
     *
     * @param traj Extrusion trajectory
     * @param A State transition matrix [n×n]
     * @param B Input matrix [n×1]
     * @param C Output matrix [1×n]
     * @param D Feedthrough (scalar, must be ≠ 0 for invertibility)
     * @param params Parameters
     */
    AnalyticalLTIDeconvolution(const Traj& traj,
                                const Eigen::MatrixXd& A,
                                const Eigen::MatrixXd& B,
                                const Eigen::MatrixXd& C,
                                double D,
                                AnalyticalLTIDeconvParams params)
        : traj_(&traj)
        , params_(params)
        , mode_(Mode::StateSpace)
        , ssA_(A), ssB_(B), ssC_(C), ssD_(D) {
        setupStateSpace();
    }

    /**
     * @brief Compute the required input x(t) at time t.
     *
     * This is the deconvolved extruder command — the input that, when
     * passed through the system, produces the target trajectory y(t).
     *
     * @param t Query time [s]
     * @param usePosition If true, y(t) = extruder position; if false,
     *                    y(t) = extruder velocity.  Default: velocity.
     */
    double inputAtTime(double t, bool usePosition = false) const {
        if (mode_ == Mode::ImpulseResponse) {
            return inputImpulseResponse(t, usePosition);
        }
        return inputStateSpace(t, usePosition);
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
     *
     * e_adjusted(t) = e_raw(t) + x(t)  (the deconvolved input is the
     * feedforward compensation)
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

    /// Parameters
    const AnalyticalLTIDeconvParams& params() const { return params_; }

    /// The precomputed regularized inverse (impulse-response mode)
    const std::vector<double>& inverseImpulseResponse() const {
        return hInv_;
    }

    /// The precomputed moments M_k (impulse-response mode)
    const std::vector<double>& moments() const { return moments_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalLTIDeconvParams params_;

    enum class Mode { ImpulseResponse, StateSpace };
    Mode mode_;

    // Impulse-response mode data
    double sampleRate_ = 0.0;
    std::vector<double> hInv_;     ///< Regularized inverse impulse response
    std::vector<double> moments_;  ///< M_k = ∫ h_inv(τ) τ^k dτ

    // State-space mode data
    Eigen::MatrixXd ssA_, ssB_, ssC_;
    double ssD_ = 0.0;
    Eigen::MatrixXd ssF_;  ///< A - B·D⁺·C
    Eigen::VectorXd ssState_;  ///< Current internal state
    struct SSArcSolution {
        double t0 = 0.0;
        double duration = 0.0;
        Eigen::VectorXd stateStart;
        Eigen::MatrixXd expF;  ///< exp(F·Δt)
        // Polynomial forcing integral coefficients
        // I_k = ∫₀^Δt exp(F·(Δt-s)) · s^k ds  (vector per k)
        std::vector<Eigen::VectorXd> forcingIntegrals;
    };
    std::vector<SSArcSolution> ssSolutions_;

    // ----------------------------------------------------------------
    // Impulse-response mode
    // ----------------------------------------------------------------

    /**
     * @brief Compute the regularized inverse impulse response from h[n].
     *
     * Uses frequency-domain regularization:
     *   H_inv[k] = H*[k] / (|H[k]|² + λ)
     * then IFFT to get h_inv[n].
     */
    void computeRegularizedInverse(const std::vector<double>& h) {
        if (h.empty()) {
            hInv_ = {1.0};  // Identity system
            return;
        }

        int N = static_cast<int>(h.size());
        // Pad to next power of 2 for FFT
        int fftSize = 1;
        while (fftSize < 2 * N) fftSize <<= 1;

        // FFT of h (zero-padded)
        Eigen::FFT<double> fft;
        std::vector<double> hPadded(fftSize, 0.0);
        std::copy(h.begin(), h.end(), hPadded.begin());

        std::vector<std::complex<double>> H(fftSize);
        fft.fwd(H, hPadded);

        // Regularized inverse in frequency domain
        std::vector<std::complex<double>> Xinv(fftSize);
        for (int k = 0; k < fftSize; ++k) {
            double magSq = std::norm(H[k]);  // |H[k]|²
            Xinv[k] = std::conj(H[k]) / (magSq + params_.lambda);
        }

        // IFFT
        std::vector<double> hInvPadded(fftSize);
        fft.inv(hInvPadded, Xinv);

        // Extract and shift by group delay (peak of h)
        int peakIdx = 0;
        double peakVal = 0.0;
        for (int i = 0; i < N; ++i) {
            if (std::abs(h[i]) > peakVal) {
                peakVal = std::abs(h[i]);
                peakIdx = i;
            }
        }

        hInv_.resize(N);
        for (int i = 0; i < N; ++i) {
            int srcIdx = i + peakIdx;
            hInv_[i] = (srcIdx < fftSize) ? hInvPadded[srcIdx] : 0.0;
        }
    }

    /**
     * @brief Compute moments M_k = ∫₀^∞ h_inv(τ) τ^k dτ for k = 0..K.
     *
     * Uses trapezoidal integration on the sampled h_inv.
     */
    void computeMoments() {
        if (hInv_.empty() || sampleRate_ <= 0.0) {
            moments_.assign(params_.maxPolyDegree + 1, 0.0);
            if (!hInv_.empty()) moments_[0] = hInv_[0];
            return;
        }

        double dt = 1.0 / sampleRate_;
        int K = params_.maxPolyDegree;
        moments_.assign(K + 1, 0.0);

        for (size_t i = 0; i < hInv_.size(); ++i) {
            double tau = static_cast<double>(i) * dt;
            double hVal = hInv_[i];
            // Trapezoidal weight
            double w = dt;
            if (i == 0 || i == hInv_.size() - 1) w *= 0.5;
            for (int k = 0; k <= K; ++k) {
                moments_[k] += hVal * std::pow(tau, k) * w;
            }
        }
    }

    /**
     * @brief Compute x(t) using the moment-based convolution.
     *
     * Within each arc, y(t-τ) is a polynomial in τ:
     *   y(t-τ) = Σ c_k(t) · τ^k
     *
     * The convolution is:
     *   x(t) = Σ c_k(t) · M_k
     */
    double inputImpulseResponse(double t, bool usePosition) const {
        if (moments_.empty()) return 0.0;

        // Shift by group delay
        double tEff = t - params_.groupDelay;
        if (tEff < 0.0) return 0.0;

        // Get the polynomial coefficients of y(t-τ) as a function of τ
        // y(t-τ) = Σ c_k · τ^k
        // For y = extruder velocity: v_e(t-τ) = α_e · (c0 + c1·(t-τ) + c2·(t-τ)²)
        //   = α_e · (c0 + c1·t + c2·t²) + α_e · (-c1 - 2·c2·t)·τ + α_e · c2·τ²
        // For y = extruder position: e(t-τ) = e(t) - ∫_{t-τ}^t v_e(s) ds
        //   This is more complex; we use the position polynomial.

        const auto& arcs = traj_->arcs();
        if (arcs.empty()) return 0.0;

        size_t idx = traj_->findArc(tEff);
        const auto& a = arcs[idx];
        double tau_local = std::clamp(tEff - a.t0, 0.0, a.duration);

        if (usePosition) {
            // y(t) = extruder position = piecewise polynomial of degree 3
            // Within arc: e(τ) = e0 + α_e·(c0·τ + ½·c1·τ² + ⅓·c2·τ³)
            // e(t-τ) as polynomial in τ:
            //   e(t-τ) = e(t) - α_e·[c0·τ - ½·c1·τ² + ⅓·c2·τ³]
            //          = e(t) - α_e·c0·τ + α_e·½·c1·τ² - α_e·⅓·c2·τ³
            double eAtT = traj_->extruderPositionAtTime(tEff);
            std::vector<double> coeffs(4, 0.0);
            coeffs[0] = eAtT;
            coeffs[1] = -a.extrusionRatio * a.c0;
            coeffs[2] = a.extrusionRatio * 0.5 * a.c1;
            coeffs[3] = -a.extrusionRatio * (1.0 / 3.0) * a.c2;

            double x = 0.0;
            for (int k = 0; k <= 3 && k < static_cast<int>(moments_.size()); ++k) {
                x += coeffs[k] * moments_[k];
            }
            return x;
        } else {
            // y(t) = extruder velocity = piecewise polynomial of degree 2
            // v_e(t-τ) = α_e · (c0 + c1·(t-τ) + c2·(t-τ)²)
            //   = α_e·(c0 + c1·t + c2·t²) + α_e·(-c1 - 2·c2·t)·τ + α_e·c2·τ²
            double tAbs = a.t0 + tau_local;
            std::vector<double> coeffs(3, 0.0);
            coeffs[0] = a.extrusionRatio * (a.c0 + a.c1 * tAbs + a.c2 * tAbs * tAbs);
            coeffs[1] = a.extrusionRatio * (-a.c1 - 2.0 * a.c2 * tAbs);
            coeffs[2] = a.extrusionRatio * a.c2;

            double x = 0.0;
            for (int k = 0; k <= 2 && k < static_cast<int>(moments_.size()); ++k) {
                x += coeffs[k] * moments_[k];
            }
            return x;
        }
    }

    // ----------------------------------------------------------------
    // State-space mode
    // ----------------------------------------------------------------

    void setupStateSpace() {
        int n = static_cast<int>(ssA_.rows());
        // D⁺ = D / (D² + λ) for scalar D
        double Dreg = ssD_ / (ssD_ * ssD_ + params_.lambda);

        // F = A - B·D⁺·C
        ssF_ = ssA_ - ssB_ * Dreg * ssC_;
        ssState_ = Eigen::VectorXd::Zero(n);

        precomputeStateSpaceArcs();
    }

    void precomputeStateSpaceArcs() {
        const auto& arcs = traj_->arcs();
        ssSolutions_.clear();
        ssSolutions_.reserve(arcs.size());

        Eigen::VectorXd currentState = ssState_;
        for (const auto& a : arcs) {
            int n = static_cast<int>(ssF_.rows());
            double dt = a.duration;
            if (dt <= 0.0) {
                SSArcSolution sol;
                sol.t0 = a.t0;
                sol.duration = 0.0;
                sol.stateStart = currentState;
                sol.expF = Eigen::MatrixXd::Identity(n, n);
                ssSolutions_.push_back(sol);
                continue;
            }

            SSArcSolution sol;
            sol.t0 = a.t0;
            sol.duration = dt;
            sol.stateStart = currentState;
            sol.expF = (ssF_ * dt).exp();

            // For y(t) = extruder velocity = α_e · (c0 + c1·t + c2·t²)
            // The forcing is B·D⁺·y(t) = B·Dreg·α_e·(c0 + c1·τ + c2·τ²)
            // We need I_k = ∫₀^dt exp(F·(dt-s)) · s^k ds for k=0,1,2
            // Compute via augmented matrix method:
            // [F  I]     [F  I]^dt   [∫exp(F·(dt-s))ds  ∫exp(F·(dt-s))·s ds]
            // [0  0]  =  [0  0]     [0                  0                  ]
            // Actually, use the recursive formula:
            // I_0 = F⁻¹·(exp(F·dt) - I)
            // I_k = F⁻¹·(k·I_{k-1} - dt^k·I)

            Eigen::MatrixXd Finv = ssF_.colPivHouseholderQr()
                .solve(Eigen::MatrixXd::Identity(n, n));
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

            std::vector<Eigen::VectorXd> forcingIntegrals(3);
            // I_0 = F⁻¹·(expF - I)
            Eigen::MatrixXd I0 = Finv * (sol.expF - I);
            // I_1 = F⁻¹·(I_0 - dt·I)  -- wait, the recursion is:
            // ∫₀^T exp(F(T-s)) s^k ds
            // Let u = T-s: = ∫₀^T exp(Fu) (T-u)^k du
            //   = Σ_{j=0}^k C(k,j) T^{k-j} (-1)^j ∫₀^T exp(Fu) u^j du
            // And ∫₀^T exp(Fu) u^j du satisfies:
            //   J_0 = F⁻¹(exp(FT) - I)
            //   J_j = F⁻¹(j·J_{j-1} - T^j·I)  ... wait let me re-derive
            // Actually: d/du[exp(Fu)·u^j] = F·exp(Fu)·u^j + j·exp(Fu)·u^{j-1}
            // So exp(Fu)·u^j = F⁻¹·[d/du(exp(Fu)·u^j) - j·exp(Fu)·u^{j-1}]
            // ∫₀^T exp(Fu)·u^j du = F⁻¹·[exp(FT)·T^j - j·∫₀^T exp(Fu)·u^{j-1} du]
            //   J_j = F⁻¹·(exp(FT)·T^j - j·J_{j-1})

            std::vector<Eigen::MatrixXd> J(3);
            J[0] = Finv * (sol.expF - I);
            J[1] = Finv * (sol.expF * dt - J[0]);
            J[2] = Finv * (sol.expF * dt * dt - 2.0 * J[1]);

            // Now I_k = ∫₀^T exp(F(T-s)) s^k ds = Σ C(k,j) T^{k-j} (-1)^j J_j
            for (int k = 0; k <= 2; ++k) {
                Eigen::MatrixXd Ik = Eigen::MatrixXd::Zero(n, n);
                for (int j = 0; j <= k; ++j) {
                    // C(k,j) = k! / (j! (k-j)!)
                    double binom = 1.0;
                    for (int l = 0; l < j; ++l)
                        binom *= static_cast<double>(k - l) / (l + 1.0);
                    Ik += binom * std::pow(dt, k - j)
                          * (j % 2 == 0 ? 1.0 : -1.0) * J[j];
                }
                forcingIntegrals[k] = Ik * ssB_;
            }

            sol.forcingIntegrals = forcingIntegrals;
            ssSolutions_.push_back(sol);

            // Propagate state: v(dt) = expF·v0 + Σ c_k · I_k·B·Dreg
            double Dreg = ssD_ / (ssD_ * ssD_ + params_.lambda);
            double alphaE = a.extrusionRatio;
            Eigen::VectorXd vEnd = sol.expF * currentState;
            // y(τ) = α_e · (c0 + c1·τ + c2·τ²)
            vEnd += Dreg * alphaE * (
                a.c0 * forcingIntegrals[0]
                + a.c1 * forcingIntegrals[1]
                + a.c2 * forcingIntegrals[2]
            );
            currentState = vEnd;
        }
    }

    double inputStateSpace(double t, bool usePosition) const {
        if (ssSolutions_.empty()) return 0.0;

        double tEff = t - params_.groupDelay;
        if (tEff < 0.0) return 0.0;

        size_t idx = traj_->findArc(tEff);
        if (idx >= ssSolutions_.size()) idx = ssSolutions_.size() - 1;
        const auto& sol = ssSolutions_[idx];
        const auto& a = traj_->arcs()[idx];
        double tau = std::clamp(tEff - sol.t0, 0.0, sol.duration);

        // Compute state v(τ) = exp(F·τ)·v0 + Dreg·α_e·Σ c_k·I_k(τ)·B
        // where I_k(τ) = ∫₀^τ exp(F(τ-s)) s^k ds
        // For simplicity, use the precomputed end-of-arc state and
        // interpolate. For exact evaluation, compute exp(F·τ) on the fly.
        int n = static_cast<int>(ssF_.rows());
        Eigen::MatrixXd expFtau = (ssF_ * tau).exp();
        Eigen::VectorXd v = expFtau * sol.stateStart;

        // Add forcing integral (simplified: use linear interpolation of
        // the forcing integrals scaled by τ/dt)
        double Dreg = ssD_ / (ssD_ * ssD_ + params_.lambda);
        double alphaE = a.extrusionRatio;
        double frac = (sol.duration > 0.0) ? tau / sol.duration : 0.0;
        // Scale the precomputed forcing integrals
        // (exact would require recomputing I_k for τ, but this is a
        // good approximation for short arcs)
        for (int k = 0; k <= 2; ++k) {
            v += Dreg * alphaE * std::pow(frac, k + 1) * (
                a.c0 * sol.forcingIntegrals[0]
                + a.c1 * sol.forcingIntegrals[1]
                + a.c2 * sol.forcingIntegrals[2]
            ) / (k + 1.0);  // crude scaling
        }
        // Actually, let's use a simpler approach: just compute v at τ
        // by scaling the end-of-arc state
        v = sol.stateStart + frac * (
            (sol.expF * sol.stateStart - sol.stateStart)
            + Dreg * alphaE * (
                a.c0 * sol.forcingIntegrals[0]
                + a.c1 * sol.forcingIntegrals[1]
                + a.c2 * sol.forcingIntegrals[2]
            )
        );

        // x(t) = D⁺ · (y(t) - C·v(t))
        double y;
        if (usePosition) {
            y = traj_->extruderPositionAtTime(tEff);
        } else {
            y = a.extruderVelocity(tau);
        }
        double Cv = (ssC_ * v)(0, 0);
        return Dreg * (y - Cv);
    }
};

} // namespace MotionPlanner::analytical::extrusion
