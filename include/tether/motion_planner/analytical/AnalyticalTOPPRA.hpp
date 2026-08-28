/**
 * @file AnalyticalJerkLimitedTOPPRA.hpp
 * @brief Analytical TOPPRA-equivalent velocity profiler for NURBS chain inputs.
 *
 * @details
 * This profiler implements the time-optimal path-following controller
 * described in the analytical TOPPRA manual. It operates solely in the
 * analytical (arc-length) space and produces two output representations:
 *
 * - **SSR (Class A)**: Switching Structure Representation — exact,
 *   procedural. The trajectory is reconstructed by ODE integration.
 *
 * - **Hybrid (Class B)**: Hybrid Monotone Representation — practical,
 *   certifiable. Pre-computed spectral elements with error bounds.
 *
 * Both representations provide exact or certifiably-approximate sampling
 * of position, velocity, and acceleration at any time t.
 *
 * ## Interface Compatibility
 *
 * This profiler implements the standard `VelocityProfiler<Dim, T>`
 * interface. Its `computeProfile()` method:
 * 1. Solves the time-optimal problem → produces SSR (switching arcs)
 * 2. Samples the SSR at uniform arc-length intervals → produces a
 *    `VelocityProfile<T>` (tabulated, compatible with MotionPlan)
 * 3. Stores the SSR and optionally builds the Hybrid representation
 *    for later exact/certified sampling
 *
 * This means downstream consumers that expect a sampled `VelocityProfile`
 * (like the existing MotionPlan) work unchanged. Consumers that want
 * exact sampling can access the SSR or Hybrid via `analyticalSource()`.
 *
 * ## Time-Optimal Solver
 *
 * The solver uses a forward-backward integration approach in arc-length
 * space with jerk constraints:
 *
 * 1. **Forward pass**: Start from (s=0, v=v_start, a=a_start), apply
 *    eta = eta_upper (maximal jerk), integrate forward. Record the
 *    maximum velocity profile.
 *
 * 2. **Backward pass**: Start from (s=L, v=v_end, a=0), apply
 *    eta = eta_lower (maximal deceleration), integrate backward.
 *
 * 3. **Switching structure**: The final profile is the minimum of the
 *    forward, backward, and velocity-limit curves. Switching points
 *    are where the active constraint changes.
 *
 * The dynamics in arc-length space are:
 *   t' = 1/v,  v' = a/v,  a' = eta/v
 *
 * where ' = d/ds, and eta = da/dt is the jerk control input.
 *
 * ## Mathematical Foundation
 *
 * See docs/motion/AnalyticalJerkLimitedTOPPRA.md for the complete mathematical
 * derivation, including:
 * - Arc-length parameterization and dynamics
 * - Constraint transformation (per-axis → eta bounds)
 * - Pontryagin maximum principle and bang-bang structure
 * - SSR construction and sampling
 * - Hybrid LGL + Padé representation with certification
 * - Error bound proofs
 *
 * @see VelocityProfiler.hpp for the abstract interface.
 * @see SwitchingStructureRepresentation.hpp for Class A.
 * @see HybridMonotoneRepresentation.hpp for Class B.
 * @see TrajectorySampler.hpp for the unified sampling interface.
 * @see ConstraintEvaluator.hpp for eta bound computation.
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "ConstraintEvaluator.hpp"
#include "NumericalUtils.hpp"
#include "SwitchingStructureRepresentation.hpp"
#include "HybridMonotoneRepresentation.hpp"
#include "TrajectorySampler.hpp"
#include "AnalyticalSSRVelocityProfile.hpp"
#include "../BasicTOPPRA.hpp"
#include "../VelocityProfile.hpp"
#include "../VelocityProfiler.hpp"
#include "../PathAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace MotionPlanner {

/// Add the analytical profiler type to the ProfilerType enum.
/// We extend the existing enum by adding a new value. Since the existing
/// enum is in VelocityProfiler.hpp, we add a constant here for the
/// analytical profiler type identification.
namespace analytical {

/**
 * @brief Analytical 2nd-order TOPP-RA without a jerk constraint.
 *
 * This is deliberately separate from `AnalyticalJerkLimitedTOPPRA`: it uses
 * ordinary second-order TOPP-RA and never invents a finite pseudo-jerk.
 * Its acceleration is discontinuous at switches, so its profile advertises
 * acceleration, not jerk, as its highest meaningful derivative.
 */
template<size_t Dim, typename T = double>
class AnalyticalTOPPRA : public BasicTOPPRA<Dim, T> {
public:
    using Base = BasicTOPPRA<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;

    explicit AnalyticalTOPPRA(Limits limits = {})
        : Base(std::move(limits)) {}

    ProfilerType type() const override {
        return ProfilerType::AnalyticalTOPPRA;
    }

    const char* name() const override {
        return "AnalyticalTOPPRA (2nd-order, no jerk limit)";
    }
};

} // namespace analytical

namespace analytical {

/**
 * @brief Analytical TOPPRA-equivalent velocity profiler.
 *
 * Implements the VelocityProfiler interface. Produces a time-optimal
 * velocity profile with jerk constraints, operating on NURBS curve chains.
 * The profile is stored both as a tabulated VelocityProfile (for backward
 * compatibility) and as an SSR/Hybrid representation (for exact sampling).
 *
 * @tparam Dim Spatial dimension
 * @tparam T   Numeric type (default: double)
 */
template<size_t Dim, typename T = double>
class AnalyticalJerkLimitedTOPPRA : public VelocityProfiler<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;
    using Point = VelocityProfilePoint;
    using Evaluator = ConstraintEvaluator<Dim, T>;
    using SSR = SwitchingStructureRepresentation<Dim, T>;
    using Hybrid = HybridMonotoneRepresentation<Dim, T>;
    using Sampler = TrajectorySampler<Dim, T>;

    /**
     * @brief Constructor.
     * @param limits Kinematic limits (per-axis and path-level).
     * @param buildHybrid Whether to also build the Hybrid representation.
     * @param hybridTolerance Target tolerance for Hybrid representation.
     */
    explicit AnalyticalJerkLimitedTOPPRA(
        Limits limits = {},
        bool buildHybrid = true,
        double hybridTolerance = 1e-10)
        : limits_(std::move(limits))
        , buildHybrid_(buildHybrid)
        , hybridTolerance_(hybridTolerance) {}

    /**
     * @brief Compute a time-optimal velocity profile for the given path.
     *
     * This method:
     * 1. Solves the time-optimal problem in arc-length space
     * 2. Produces switching arcs (SSR)
     * 3. Samples the SSR to produce a tabulated VelocityProfile
     * 4. Optionally builds the Hybrid representation
     *
     * The returned VelocityProfile is compatible with MotionPlan and all
     * existing downstream consumers. The SSR and Hybrid representations
     * are accessible via `analyticalSource()` and `hybridSource()`.
     */
    std::unique_ptr<VelocityProfile> computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        if (path.numSegments() == 0) {
            return std::make_unique<SampledVelocityProfile>();
        }

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) {
            return std::make_unique<SampledVelocityProfile>();
        }
        if (numSamples < 2 || feedRate <= T(0) ||
            !limits_.path.jerkLimitEnabled ||
            limits_.path.maxPathJerk <= T(0) ||
            std::abs(startJerk) > T(1e-12)) {
            // This is the explicitly third-order implementation. A disabled
            // jerk limit belongs to AnalyticalTOPPRA; a nonzero initial jerk
            // cannot be satisfied because the interface has no terminal jerk
            // boundary condition, so it is rejected rather than reset.
            return std::make_unique<SampledVelocityProfile>();
        }

        // Create constraint evaluator
        Evaluator evaluator(limits_, feedRate);

        // Solve the time-optimal problem
        auto arcs = solveTimeOptimal(
            path, evaluator, feedRate,
            startVelocity, endVelocity,
            startAcceleration, startJerk,
            numSamples);

        // Build the SSR (stores a const reference to the path)
        auto ssr = std::make_shared<SSR>(
            path, std::move(arcs), std::move(evaluator));
        ssr_ = ssr;

        // Wrap the SSR in a sampler. This is the exact representation used
        // by the returned velocity profile. The Hybrid representation (if
        // requested) is kept separately for consumers that need it.
        sampler_ = std::make_shared<Sampler>(ssr);

        // Optionally build the Hybrid representation.
        if (buildHybrid_) {
            hybrid_ = std::make_shared<Hybrid>(*ssr, hybridTolerance_);
        }

        // Return an analytical profile that wraps the SSR/Hybrid sampler.
        if (ssr_ || hybrid_) {
            return std::make_unique<AnalyticalSSRVelocityProfile<Dim, T>>(sampler_);
        }

        // Fallback: empty profile.
        return std::make_unique<SampledVelocityProfile>();
    }

    /**
     * @brief Get the analytical trajectory source (for exact sampling).
     *
     * Returns a TrajectorySampler that wraps either the SSR or Hybrid
     * representation. This can be used by MotionPlan or other consumers
     * for exact/certified trajectory sampling.
     */
    std::shared_ptr<Sampler> analyticalSource() const {
        return sampler_;
    }

    /**
     * @brief Get the SSR representation.
     */
    std::shared_ptr<SSR> ssrSource() const { return ssr_; }

    /**
     * @brief Get the Hybrid representation (null if not built).
     */
    std::shared_ptr<Hybrid> hybridSource() const { return hybrid_; }

    // ========================================================================
    // VelocityProfiler interface
    // ========================================================================

    Limits limits() const override { return limits_; }

    ProfilerType type() const override {
        return ProfilerType::AnalyticalJerkLimitedTOPPRA;
    }

    const char* name() const override {
        return "AnalyticalJerkLimitedTOPPRA (analytical TOPPRA-equivalent)";
    }

    ProfileDerivativeOrder derivativeOrder() const override {
        return ProfileDerivativeOrder::Jerk;
    }

private:
    Limits limits_;
    bool buildHybrid_;
    double hybridTolerance_;

    std::shared_ptr<SSR> ssr_;
    std::shared_ptr<Hybrid> hybrid_;
    std::shared_ptr<Sampler> sampler_;

    // ========================================================================
    // Time-Optimal Solver
    // ========================================================================

    /**
     * @brief Solve the time-optimal path-following problem.
     *
     * Uses forward-backward integration in arc-length space with jerk
     * constraints. The algorithm:
     *
     * 1. Compute velocity limit curve v_lim(s) from constraints
     * 2. Forward pass: integrate from start with eta = eta_upper
     * 3. Backward pass: integrate from end with eta = eta_lower
     * 4. Merge: take min(forward, backward, v_lim) and detect switching points
     * 5. Build switching arcs from the merged profile
     *
     * @return Vector of switching arcs
     */
    std::vector<SwitchingArc> solveTimeOptimal(
        const Path& path,
        const Evaluator& evaluator,
        T feedRate,
        T startVelocity,
        T endVelocity,
        T startAcceleration,
        T startJerk,
        size_t numSamples) const {

        const T pathLength = path.totalLength();
        const T ds = pathLength / T(numSamples - 1);

        // Step 1: Compute velocity limit curve v_lim(s)
        std::vector<T> vLim(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(T(i) * ds, pathLength);
            vLim[i] = evaluator.velocityLimit(s, path);
        }

        // Step 2: Forward pass (maximal acceleration)
        // Integrate: v' = a/v, a' = eta_upper/v
        // Starting from (v_start, a_start)
        std::vector<T> fwdVel(numSamples);
        std::vector<T> fwdAcc(numSamples);
        fwdVel[0] = std::min(startVelocity, vLim[0]);
        fwdAcc[0] = startAcceleration;

        for (size_t i = 1; i < numSamples; ++i) {
            T sPrev = std::min(T(i - 1) * ds, pathLength);
            T sCurr = std::min(T(i) * ds, pathLength);
            T deltaS = sCurr - sPrev;

            auto [v, a] = integrateForward(
                evaluator, path, sPrev, sCurr,
                fwdVel[i - 1], fwdAcc[i - 1]);

            fwdVel[i] = std::min({v, vLim[i]});
            fwdAcc[i] = a;

            // If velocity hit zero, stop
            if (fwdVel[i] < T(1e-12)) {
                fwdVel[i] = T(0);
                fwdAcc[i] = T(0);
            }
        }

        // Step 3: Backward pass (maximal deceleration)
        // Integrate backward from end: v' = a/v, a' = eta_lower/v
        std::vector<T> bwdVel(numSamples);
        std::vector<T> bwdAcc(numSamples);
        bwdVel[numSamples - 1] = std::min(endVelocity, vLim[numSamples - 1]);
        bwdAcc[numSamples - 1] = T(0);

        for (size_t i = numSamples - 1; i > 0; --i) {
            T sCurr = std::min(T(i) * ds, pathLength);
            T sPrev = std::min(T(i - 1) * ds, pathLength);
            T deltaS = sCurr - sPrev;

            auto [v, a] = integrateBackward(
                evaluator, path, sCurr, sPrev,
                bwdVel[i], bwdAcc[i]);

            bwdVel[i - 1] = std::min({v, vLim[i - 1]});
            bwdAcc[i - 1] = a;

            if (bwdVel[i - 1] < T(1e-12)) {
                bwdVel[i - 1] = T(0);
                bwdAcc[i - 1] = T(0);
            }
        }

        // Step 4: Merge — take min(forward, backward, v_lim) and detect switches
        std::vector<T> finalVel(numSamples);
        std::vector<T> finalAcc(numSamples);
        std::vector<ControlMode> modes(numSamples);

        for (size_t i = 0; i < numSamples; ++i) {
            T fwd = fwdVel[i];
            T bwd = bwdVel[i];
            T lim = vLim[i];

            finalVel[i] = std::min({fwd, bwd, lim});

            // Determine mode
            if (finalVel[i] == lim && lim <= fwd && lim <= bwd) {
                modes[i] = ControlMode::CONSTRAINT_SURFACE;
                finalAcc[i] = T(0);
            } else if (finalVel[i] == fwd && fwd <= bwd) {
                modes[i] = ControlMode::ACCEL_MAX;
                finalAcc[i] = fwdAcc[i];
            } else if (finalVel[i] == bwd) {
                modes[i] = ControlMode::DECEL_MAX;
                finalAcc[i] = bwdAcc[i];
            } else {
                modes[i] = ControlMode::ZERO_JERK;
                finalAcc[i] = T(0);
            }
        }

        // Step 5: Build switching arcs from the merged profile
        return buildSwitchingArcs(
            path, evaluator, finalVel, finalAcc, modes,
            ds, pathLength, numSamples,
            startVelocity, endVelocity,
            startAcceleration, startJerk);
    }

    /**
     * @brief Forward integration step with maximal jerk (eta_upper).
     *
     * Integrates the arc-length dynamics from sPrev to sCurr:
     *   v' = a/v, a' = eta_upper/v
     *
     * @return (v, a) at sCurr
     */
    std::pair<T, T> integrateForward(
        const Evaluator& evaluator, const Path& path,
        T sPrev, T sCurr, T vPrev, T aPrev) const {

        struct State {
            double v, a;
            State operator+(const State& o) const { return {v+o.v, a+o.a}; }
            State operator*(double k) const { return {v*k, a*k}; }
            std::array<double, 2> components() const { return {v, a}; }
            double norm() const { return std::sqrt(v*v + a*a); }
        };

        const T jMax = limits_.path.maxPathJerk;
        const bool jerkEnabled = limits_.path.jerkLimitEnabled && jMax > T(0);

        State y{static_cast<double>(vPrev), static_cast<double>(aPrev)};
        double s = static_cast<double>(sPrev);
        double dsTotal = static_cast<double>(sCurr - sPrev);
        int nSteps = std::max(1, static_cast<int>(std::abs(dsTotal) / 1e-2));
        double ds = dsTotal / nSteps;
        double aCap = static_cast<double>(limits_.path.maxPathAcceleration);

        // When starting from rest (v ≈ 0, a ≈ 0), the arc-length formulation
        // dv/ds = a/v has a singularity. Use the time-domain closed-form
        // solution for the initial steps: a(t) = eta*t, v(t) = 0.5*eta*t²,
        // s(t) = eta*t³/6. Given ds, t = (6*|ds|/|eta|)^(1/3).
        auto advanceFromRest = [&](double sCur, double dsStep) -> State {
            double eta;
            if (jerkEnabled) {
                auto bounds = evaluator.etaBounds(
                    static_cast<T>(sCur), T(0), T(0), path);
                eta = bounds.eta_max;
            } else {
                eta = 1e4;  // Large jerk to reach aMax quickly
            }
            if (std::abs(eta) < 1e-12) return {0.0, 0.0};
            double t = std::cbrt(6.0 * std::abs(dsStep) / std::abs(eta));
            double a = eta * t;
            double v = 0.5 * eta * t * t;
            if (dsStep < 0) v = -v;  // Backward integration
            a = std::clamp(a, -aCap, aCap);
            return {v, a};
        };

        // If starting from rest, use closed-form for the first step
        if (std::abs(y.v) < 1e-6 && std::abs(y.a) < 1e-6) {
            auto [v, a] = advanceFromRest(s, ds);
            y.v = v;
            y.a = a;
            s += ds;
        }

        auto rhs = [&](double s, const State& y) -> State {
            double vSafe = std::max(std::abs(y.v), 1e-6);
            double eta;
            if (jerkEnabled) {
                auto bounds = evaluator.etaBounds(
                    static_cast<T>(s), static_cast<T>(y.v),
                    static_cast<T>(y.a), path);
                eta = bounds.eta_max;
            } else {
                // No jerk limit: bang-bang acceleration
                auto [aMin, aMax] = evaluator.accelerationBounds(
                    static_cast<T>(s), static_cast<T>(y.v), path);
                // Accelerate at max: set a toward aMax
                double aTarget = static_cast<double>(aMax);
                if (y.a < aTarget) {
                    // Use bounded jerk to reach aTarget
                    eta = std::min(1e4, (aTarget - y.a) * vSafe / std::max(dsTotal, 1e-12));
                } else {
                    eta = 0;
                }
            }
            return {y.a / vSafe, eta / vSafe};
        };

        for (int i = 0; i < nSteps; ++i) {
            y = rk4Step(rhs, s, y, ds);
            s += ds;
            if (std::abs(y.v) < 1e-12) y.v = (y.v < 0) ? -1e-12 : 1e-12;
            y.a = std::clamp(y.a, -aCap, aCap);
        }

        return {static_cast<T>(y.v), static_cast<T>(y.a)};
    }

    /**
     * @brief Backward integration step with minimal jerk (eta_lower).
     *
     * Integrates the arc-length dynamics backward from sCurr to sPrev:
     *   v' = a/v, a' = eta_lower/v  (integrating in negative s direction)
     *
     * @return (v, a) at sPrev
     */
    std::pair<T, T> integrateBackward(
        const Evaluator& evaluator, const Path& path,
        T sCurr, T sPrev, T vCurr, T aCurr) const {

        struct State {
            double v, a;
            State operator+(const State& o) const { return {v+o.v, a+o.a}; }
            State operator*(double k) const { return {v*k, a*k}; }
            std::array<double, 2> components() const { return {v, a}; }
            double norm() const { return std::sqrt(v*v + a*a); }
        };

        const T jMax = limits_.path.maxPathJerk;
        const bool jerkEnabled = limits_.path.jerkLimitEnabled && jMax > T(0);

        State y{static_cast<double>(vCurr), static_cast<double>(aCurr)};
        double s = static_cast<double>(sCurr);
        double dsTotal = static_cast<double>(sPrev - sCurr);  // Negative
        int nSteps = std::max(1, static_cast<int>(std::abs(dsTotal) / 1e-2));
        double ds = dsTotal / nSteps;
        double aCap = static_cast<double>(limits_.path.maxPathAcceleration);

        // If starting from rest, use closed-form for the first step.
        // For backward integration from rest at s_end, the arc decelerates
        // to v=0, a=0 at s_end. Going backward by |ds|:
        //   eta = eta_max > 0 (for backward integration dynamics)
        //   T = cbrt(6*|ds|/eta) (time from s to s_end)
        //   v = 0.5 * eta * T² > 0 (forward velocity at s)
        //   a = -eta * T < 0 (decelerating at s)
        if (std::abs(y.v) < 1e-6 && std::abs(y.a) < 1e-6) {
            double eta;
            if (jerkEnabled) {
                auto bounds = evaluator.etaBounds(
                    static_cast<T>(s), T(0), T(0), path);
                eta = bounds.eta_max;  // Backward integration uses eta_max
            } else {
                eta = 1e4;
            }
            if (std::abs(eta) > 1e-12) {
                double tStep = std::cbrt(6.0 * std::abs(ds) / eta);
                y.v = 0.5 * eta * tStep * tStep;  // Positive (forward velocity)
                y.a = std::clamp(-eta * tStep, -aCap, aCap);  // Negative (decelerating)
            }
            s += ds;
        }

        auto rhs = [&](double s, const State& y) -> State {
            double vSafe = std::max(std::abs(y.v), 1e-6);
            double eta;
            if (jerkEnabled) {
                auto bounds = evaluator.etaBounds(
                    static_cast<T>(s), static_cast<T>(y.v),
                    static_cast<T>(y.a), path);
                // Backward integration: ds < 0. To decelerate in the
                // forward direction (a < 0), we need da < 0. Since
                // da = (eta/v) * ds and ds < 0, we need eta > 0.
                // Use eta_max (positive jerk) for backward integration.
                eta = bounds.eta_max;
            } else {
                auto [aMin, aMax] = evaluator.accelerationBounds(
                    static_cast<T>(s), static_cast<T>(y.v), path);
                // Backward integration: use aMax as target (forward
                // deceleration = backward acceleration).
                double aTarget = static_cast<double>(aMax);
                if (y.a < aTarget) {
                    eta = std::min(1e4, (aTarget - y.a) * vSafe / std::max(std::abs(static_cast<double>(sCurr - sPrev)), 1e-12));
                } else {
                    eta = 0;
                }
            }
            return {y.a / vSafe, eta / vSafe};
        };

        for (int i = 0; i < nSteps; ++i) {
            y = rk4Step(rhs, s, y, ds);
            s += ds;
            if (std::abs(y.v) < 1e-12) y.v = (y.v < 0) ? -1e-12 : 1e-12;
            y.a = std::clamp(y.a, -aCap, aCap);
        }

        return {static_cast<T>(y.v), static_cast<T>(y.a)};
    }

    // ========================================================================
    // Build Switching Arcs from Merged Profile
    // ========================================================================

    /**
     * @brief Build switching arcs from the merged velocity profile.
     *
     * Walks the merged profile and groups consecutive samples with the
     * same control mode into arcs. Computes time, initial conditions,
     * and eta for each arc.
     */
    std::vector<SwitchingArc> buildSwitchingArcs(
        const Path& path,
        const Evaluator& evaluator,
        const std::vector<T>& vel,
        const std::vector<T>& acc,
        const std::vector<ControlMode>& modes,
        T ds, T pathLength, size_t numSamples,
        T startVelocity, T endVelocity,
        T startAcceleration, T startJerk) const {

        std::vector<SwitchingArc> arcs;
        if (numSamples == 0) return arcs;

        // Group consecutive samples with the same mode into arcs
        size_t arcStart = 0;
        ControlMode currentMode = modes[0];

        for (size_t i = 1; i < numSamples; ++i) {
            if (modes[i] != currentMode) {
                // End current arc
                SwitchingArc arc;
                arc.s_begin = std::min(T(arcStart) * ds, pathLength);
                arc.s_end = std::min(T(i) * ds, pathLength);
                arc.mode = currentMode;
                arc.v0 = vel[arcStart];
                arc.a0 = acc[arcStart];
                arc.t0 = 0.0;  // Will be computed below
                arc.eta = computeEtaForMode(currentMode, evaluator, path,
                                           arc.s_begin, arc.v0, arc.a0);
                if (arc.valid()) arcs.push_back(arc);

                arcStart = i;
                currentMode = modes[i];
            }
        }

        // Last arc
        {
            SwitchingArc arc;
            T lastS = std::min(T(numSamples - 1) * ds, pathLength);
            T lastVel = vel[numSamples - 1];
            T lastAcc = acc[numSamples - 1];
            // Add a final decel/accel arc if the last sample's velocity
            // doesn't match the end velocity. Use a generous distance
            // threshold to handle floating-point rounding in lastS.
            double dRemaining = static_cast<double>(pathLength - lastS);
            bool needsFinalDecel =
                std::abs(lastVel - endVelocity) > T(1e-6) &&
                dRemaining > 1e-14;

            if (needsFinalDecel) {
                // Trim the last arc to end at the last sample, then add
                // a final decel/accel arc from lastS to pathLength.
                arc.s_begin = std::min(T(arcStart) * ds, pathLength);
                arc.s_end = lastS;
            } else {
                arc.s_begin = std::min(T(arcStart) * ds, pathLength);
                arc.s_end = pathLength;
            }
            arc.mode = currentMode;
            arc.v0 = vel[arcStart];
            arc.a0 = acc[arcStart];
            arc.t0 = 0.0;
            arc.eta = computeEtaForMode(currentMode, evaluator, path,
                                       arc.s_begin, arc.v0, arc.a0);
            if (arc.valid() && arc.s_end > arc.s_begin) arcs.push_back(arc);

            // Add final decel/accel arc if needed
            if (needsFinalDecel) {
                SwitchingArc finalArc;
                finalArc.s_begin = lastS;
                finalArc.s_end = pathLength;
                double dArc = static_cast<double>(pathLength - lastS);
                if (endVelocity < lastVel) {
                    finalArc.mode = ControlMode::DECEL_MAX;
                } else {
                    finalArc.mode = ControlMode::ACCEL_MAX;
                }
                // First compute eta using the last sample's state
                double etaFinal = computeEtaForMode(
                    finalArc.mode, evaluator, path,
                    finalArc.s_begin, lastVel, lastAcc);
                finalArc.eta = etaFinal;
                // Compute the correct v0 for this arc so that the
                // trajectory exactly reaches s_end with v=endVelocity.
                // For constant-jerk from v0 to 0 with a0=0:
                //   d = (2/3)*v0*T, T = sqrt(2*v0/|eta|)
                //   => v0 = cbrt((9/8)*d²*|eta|)
                if (std::abs(etaFinal) > 1e-12 && std::abs(endVelocity) < 1e-9) {
                    double v0Corrected = std::cbrt(
                        (9.0 / 8.0) * dArc * dArc * std::abs(etaFinal));
                    finalArc.v0 = static_cast<T>(v0Corrected);
                    finalArc.a0 = T(0);
                } else {
                    finalArc.v0 = lastVel;
                    finalArc.a0 = lastAcc;
                }
                finalArc.t0 = 0.0;
                if (finalArc.valid()) arcs.push_back(finalArc);
            }
        }

        // Compute times for each arc by integrating the actual jerk dynamics
        // in arc-length space: dt/ds = 1/v, dv/ds = a/v, da/ds = eta/v.
        // The old constant-acceleration formula (v² = v0² + 2·a·ds) is wrong
        // for arcs with nonzero jerk (ACCEL_MAX/DECEL_MAX), leading to
        // duration mismatches that corrupt SSR time-domain queries.
        double currentTime = 0.0;
        for (auto& a : arcs) {
            a.t0 = currentTime;
            a.duration = computeArcDuration(a, evaluator, path);
            currentTime += a.duration;
        }

        // Compute u0 for each arc (NURBS parameter at s_begin)
        if (path.hasInner()) {
            const auto& inner = path.inner();
            for (auto& a : arcs) {
                auto loc = inner.locate(a.s_begin);
                const auto& piece = inner.piece(loc.piece);
                a.u0 = piece.invertLength(loc.localS);
            }
        }

        return arcs;
    }

    /**
     * @brief Compute the eta value for a given control mode at (s, v, a).
     */
    double computeEtaForMode(
        ControlMode mode,
        const Evaluator& evaluator,
        const Path& path,
        T s, T v, T a) const {

        switch (mode) {
            case ControlMode::ACCEL_MAX: {
                auto bounds = evaluator.etaBounds(s, v, a, path);
                return bounds.eta_max;
            }
            case ControlMode::DECEL_MAX: {
                auto bounds = evaluator.etaBounds(s, v, a, path);
                // For deceleration arcs, the jerk direction depends on the
                // current acceleration sign:
                // - If a < 0 (already decelerating): use eta_max > 0 to
                //   bring acceleration back toward 0 as velocity reaches 0.
                // - If a >= 0 (not yet decelerating): use eta_min < 0 to
                //   drive acceleration negative and start decelerating.
                if (static_cast<double>(a) < -1e-9) {
                    return bounds.eta_max;
                }
                return bounds.eta_min;
            }
            case ControlMode::ZERO_JERK:
                return 0.0;
            case ControlMode::CONSTRAINT_SURFACE:
                return 0.0;  // Velocity constraint active, a = 0
            case ControlMode::SINGULAR:
                return 0.0;
        }
        return 0.0;
    }

    /**
     * @brief Compute the time duration of a switching arc by integrating
     *        the jerk dynamics in arc-length space.
     *
     * Integrates: dt/ds = 1/v, dv/ds = a/v, da/ds = eta/v
     * from s_begin to s_end, accumulating the elapsed time.
     *
     * This replaces the old constant-acceleration formula which is wrong
     * for arcs with nonzero jerk (ACCEL_MAX/DECEL_MAX).
     */
    double computeArcDuration(
        const SwitchingArc& arc,
        const Evaluator& evaluator,
        const Path& path) const {

        const double sBegin = static_cast<double>(arc.s_begin);
        const double sEnd = static_cast<double>(arc.s_end);
        const double dsTotal = sEnd - sBegin;
        if (std::abs(dsTotal) < 1e-14) return 0.0;

        // Special case: DECEL_MAX arc with a0 < 0 and eta > 0
        // (decelerating, jerk bringing acceleration back to 0).
        // Closed-form: a(T) = a0 + eta*T = 0 => T = -a0/eta
        // v(T) = v0 + a0*T + 0.5*eta*T² = v0 - a0²/(2*eta)
        // For v(T) = 0: v0 = a0²/(2*eta)
        // d = v0*T + 0.5*a0*T² + (1/6)*eta*T³
        if (arc.mode == ControlMode::DECEL_MAX &&
            static_cast<double>(arc.a0) < -1e-6 &&
            static_cast<double>(arc.eta) > 1e-12 &&
            arc.v0 > T(1e-6)) {
            double v0 = static_cast<double>(arc.v0);
            double a0 = static_cast<double>(arc.a0);
            double eta = static_cast<double>(arc.eta);
            double tDur = -a0 / eta;  // Time for acceleration to reach 0
            double vEnd = v0 + a0 * tDur + 0.5 * eta * tDur * tDur;
            if (std::abs(vEnd) < 1e-3) {
                // Velocity reaches ~0 at the same time as acceleration
                double dExpected = v0 * tDur + 0.5 * a0 * tDur * tDur +
                                   eta * tDur * tDur * tDur / 6.0;
                if (std::abs(dExpected - dsTotal) < 0.05 * std::abs(dsTotal)) {
                    return tDur;
                }
            }
        }

        // Special case: DECEL_MAX arc with a0 ≈ 0 decelerating to v=0.
        // Use the closed-form time-domain solution:
        //   v(t) = v0 + 0.5*eta*t², v(T) = 0 => T = sqrt(2*v0/|eta|)
        //   d = (2/3)*v0*T
        // This avoids the arc-length singularity at v=0.
        if (arc.mode == ControlMode::DECEL_MAX &&
            std::abs(static_cast<double>(arc.a0)) < 1e-6 &&
            arc.v0 > T(1e-6) && std::abs(arc.eta) > 1e-12) {
            double v0 = static_cast<double>(arc.v0);
            double eta = static_cast<double>(arc.eta);
            double tDuration = std::sqrt(2.0 * v0 / std::abs(eta));
            // Verify the distance matches
            double dExpected = (2.0 / 3.0) * v0 * tDuration;
            if (std::abs(dExpected - dsTotal) < 0.01 * std::abs(dsTotal)) {
                return tDuration;
            }
        }

        // Special case: arc starts from rest (v ≈ 0, a ≈ 0) with nonzero
        // jerk. The arc-length formulation dt/ds = 1/v diverges because
        // v = 0. Use the time-domain closed-form solution instead:
        //   a(t) = eta * t,  v(t) = 0.5 * eta * t²,  s(t) = eta * t³ / 6
        //   => t = (6 * ds / eta)^(1/3)
        const double v0 = static_cast<double>(arc.v0);
        const double a0 = static_cast<double>(arc.a0);
        if (std::abs(v0) < 1e-9 && std::abs(a0) < 1e-9) {
            double eta = computeEtaForMode(
                arc.mode, evaluator, path,
                static_cast<T>(sBegin), arc.v0, arc.a0);
            if (std::abs(eta) > 1e-12) {
                return std::cbrt(6.0 * dsTotal / eta);
            }
            // Both v=0, a=0, eta=0: the arc is degenerate (no motion).
            return 0.0;
        }

        // State: [t, v, a]  (independent variable: s)
        struct State {
            double t, v, a;
            State operator+(const State& o) const { return {t+o.t, v+o.v, a+o.a}; }
            State operator*(double k) const { return {t*k, v*k, a*k}; }
            std::array<double, 3> components() const { return {t, v, a}; }
            double norm() const { return std::sqrt(t*t + v*v + a*a); }
        };

        State y{0.0, static_cast<double>(arc.v0), static_cast<double>(arc.a0)};
        double s = sBegin;
        int nSteps = std::max(1, static_cast<int>(std::abs(dsTotal) / 1e-2));
        double ds = dsTotal / nSteps;
        double aCap = static_cast<double>(limits_.path.maxPathAcceleration);

        auto rhs = [&](double s, const State& y) -> State {
            // When v is near zero (starting from rest), 1/v diverges.
            // Use a physics-aware floor: over a step ds, constant
            // acceleration a produces v ≈ sqrt(2*|a|*|ds|). This gives
            // a finite, accurate dt/ds for arcs starting from rest.
            double vFloor = std::sqrt(2.0 * std::abs(y.a) * std::abs(ds));
            double vSafe = std::max(y.v, std::max(vFloor, 1e-6));
            double eta = computeEtaForMode(
                arc.mode, evaluator, path,
                static_cast<T>(s), static_cast<T>(y.v), static_cast<T>(y.a));
            return {1.0 / vSafe, y.a / vSafe, eta / vSafe};
        };

        for (int i = 0; i < nSteps; ++i) {
            y = rk4Step(rhs, s, y, ds);
            s += ds;
            // Stop if velocity reaches zero (end of deceleration)
            if (y.v <= 1e-12) {
                y.v = 0.0;
                break;
            }
            y.a = std::clamp(y.a, -aCap, aCap);
        }

        return y.t;
    }

    /**
     * @brief Interpolate velocity at an arbitrary s from the sampled profile.
     */
    T interpolateVelocity(
        const std::vector<T>& vel, T ds, T pathLength,
        double s, size_t numSamples) const {

        if (numSamples == 0) return T(0);
        double idx = s / static_cast<double>(ds);
        size_t i0 = static_cast<size_t>(std::clamp(idx, 0.0, double(numSamples - 1)));
        size_t i1 = std::min(i0 + 1, numSamples - 1);
        double alpha = idx - static_cast<double>(i0);
        return vel[i0] * T(1 - alpha) + vel[i1] * T(alpha);
    }

    // ========================================================================
    // Sample SSR to produce tabulated VelocityProfile
    // ========================================================================

    /**
     * @brief Sample the SSR at uniform arc-length intervals to produce
     *        a tabulated VelocityProfile.
     *
     * This makes the analytical profiler compatible with all existing
     * downstream consumers that expect a sampled VelocityProfile.
     */
    SampledVelocityProfile sampleToProfile(const SSR& ssr, size_t numSamples) const {
        SampledVelocityProfile profile;
        T pathLength = ssr.totalLength();
        if (pathLength <= T(0)) return profile;

        T ds = pathLength / T(numSamples - 1);
        profile.reserve(numSamples);

        // Sample at uniform arc-length intervals
        // We need to find (t, v, a) at each s. Since the SSR stores arcs
        // with initial conditions, we integrate through the arcs.
        T currentTime = T(0);
        T currentV = T(0);
        T currentA = T(0);

        // Walk through arcs and sample
        const auto& arcs = ssr.arcs();
        size_t arcIdx = 0;
        double sInArc = 0.0;

        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(T(i) * ds, pathLength);

            // Find the arc containing s
            while (arcIdx < arcs.size() && s > T(arcs[arcIdx].s_end)) {
                // Move to next arc; update accumulated time
                if (arcIdx < arcs.size()) {
                    const auto& a = arcs[arcIdx];
                    // Compute time at end of this arc
                    // (approximate using average velocity)
                    double vAvg = 0.5 * (a.v0 + currentV);
                    if (vAvg < 1e-12) vAvg = 1e-12;
                    currentTime += T((a.s_end - a.s_begin) / vAvg);
                }
                arcIdx++;
                sInArc = 0.0;
            }

            if (arcIdx >= arcs.size()) {
                // Past the end
                Point pt;
                pt.arcLength = pathLength;
                pt.velocity = T(0);
                pt.acceleration = T(0);
                pt.jerk = T(0);
                pt.time = currentTime;
                profile.addPoint(pt);
                continue;
            }

            const auto& arc = arcs[arcIdx];

            // Integrate within the arc from s_begin to s
            // Using simple integration: v' = a/v, a' = eta/v
            double sLocal = static_cast<double>(s) - arc.s_begin;
            double v = arc.v0;
            double a = arc.a0;
            double t = 0.0;

            int nSteps = std::max(1, static_cast<int>(sLocal / 1e-2));
            double stepDs = sLocal / nSteps;

            for (int step = 0; step < nSteps; ++step) {
                double vSafe = std::max(v, 1e-12);
                double eta = arc.eta;
                // For ACCEL_MAX/DECEL_MAX, recompute eta from constraints
                if (arc.mode == ControlMode::ACCEL_MAX ||
                    arc.mode == ControlMode::DECEL_MAX) {
                    auto bounds = ssr.evaluator().etaBounds(
                        static_cast<T>(arc.s_begin + step * stepDs),
                        static_cast<T>(v), static_cast<T>(a), ssr.path());
                    eta = (arc.mode == ControlMode::ACCEL_MAX)
                        ? bounds.eta_max : bounds.eta_min;
                }
                // Clamp eta to jerk limit
                double jMax = static_cast<double>(limits_.path.maxPathJerk);
                if (limits_.path.jerkLimitEnabled && jMax > 0) {
                    eta = std::clamp(eta, -jMax, jMax);
                }
                // RK4 step in s-space
                // v' = a/v, a' = eta/v, t' = 1/v
                double k1v = a / vSafe;
                double k1a = eta / vSafe;
                double k1t = 1.0 / vSafe;

                double v2 = v + 0.5 * stepDs * k1v;
                double a2 = a + 0.5 * stepDs * k1a;
                double v2s = std::max(v2, 1e-12);
                double k2v = a2 / v2s;
                double k2a = eta / v2s;
                double k2t = 1.0 / v2s;

                double v3 = v + 0.5 * stepDs * k2v;
                double a3 = a + 0.5 * stepDs * k2a;
                double v3s = std::max(v3, 1e-12);
                double k3v = a3 / v3s;
                double k3a = eta / v3s;
                double k3t = 1.0 / v3s;

                double v4 = v + stepDs * k3v;
                double a4 = a + stepDs * k3a;
                double v4s = std::max(v4, 1e-12);
                double k4v = a4 / v4s;
                double k4a = eta / v4s;
                double k4t = 1.0 / v4s;

                v += stepDs * (k1v + 2*k2v + 2*k3v + k4v) / 6.0;
                a += stepDs * (k1a + 2*k2a + 2*k3a + k4a) / 6.0;
                t += stepDs * (k1t + 2*k2t + 2*k3t + k4t) / 6.0;
                v = std::max(v, 0.0);
                // Clamp velocity to velocity limit at this s
                double sCurr = arc.s_begin + (step + 1) * stepDs;
                double vLim = static_cast<double>(
                    ssr.evaluator().velocityLimit(
                        static_cast<T>(sCurr), ssr.path()));
                v = std::min(v, vLim);
                // Clamp acceleration to path limit
                double aCap = static_cast<double>(limits_.path.maxPathAcceleration);
                a = std::clamp(a, -aCap, aCap);
            }

            Point pt;
            pt.arcLength = s;
            pt.velocity = static_cast<T>(v);
            pt.acceleration = static_cast<T>(a);
            pt.jerk = static_cast<T>(arc.eta);
            pt.time = currentTime + static_cast<T>(t);

            // Determine limiting factor
            switch (arc.mode) {
                case ControlMode::ACCEL_MAX:
                    pt.limitedBy = VelocityProfilePoint::LimitType::ForwardAccel;
                    break;
                case ControlMode::DECEL_MAX:
                    pt.limitedBy = VelocityProfilePoint::LimitType::BackwardDecel;
                    break;
                case ControlMode::CONSTRAINT_SURFACE:
                    pt.limitedBy = VelocityProfilePoint::LimitType::Curvature;
                    break;
                case ControlMode::ZERO_JERK:
                    pt.limitedBy = VelocityProfilePoint::LimitType::Jerk;
                    break;
                default:
                    pt.limitedBy = VelocityProfilePoint::LimitType::None;
                    break;
            }

            profile.addPoint(pt);
        }

        return profile;
    }
};

} // namespace analytical
} // namespace MotionPlanner
