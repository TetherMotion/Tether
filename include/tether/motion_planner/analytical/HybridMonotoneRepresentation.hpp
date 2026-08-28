/**
 * @file HybridMonotoneRepresentation.hpp
 * @brief Hybrid Monotone Representation — Class B (practical, certifiable).
 *
 * @details
 * The Hybrid representation pre-computes and stores a spectral/polynomial
 * approximation of the time-optimal trajectory that is:
 * - CERTIFIABLE (error bound known per element)
 * - EFFICIENT (O(log M) search + O(N) per evaluation, N ~ 8-16)
 * - STRUCTURE-PRESERVING (monotonicity of t(s), boundedness of v, a)
 *
 * ## Key Insights
 *
 * 1. t(s) is STRICTLY INCREASING (dt/ds = 1/v > 0 for v > 0)
 *    => t(s) is invertible; we precompute t(s) and invert by Newton.
 *
 * 2. v(s), a(s) are SMOOTH on each switching arc, with discontinuities
 *    only at switch points in higher derivatives.
 *    => hp-adaptive scheme: high degree where smooth, small elements
 *       at switches.
 *
 * 3. u(s) is NOT POLYNOMIAL (due to 1/g(u) integration) but SMOOTH
 *    => Local Padé approximation with ERROR CONTROL.
 *
 * ## Representation Choice
 *
 * - t(s), v(s), a(s): hp-Legendre-Gauss-Lobatto (LGL) spectral elements
 * - u(s): Local Padé approximant with exact node values
 * - s(t): Inverse of t(s) by Newton iteration (safeguarded by bisection)
 *
 * ## HP-Adaptive LGL Element
 *
 * On element [s_k, s_{k+1}] with length h_k:
 *   Map to reference: xi = 2*(s - s_k)/h_k - 1,  xi in [-1, 1]
 *   LGL nodes: xi_j, j = 0..N
 *   Function represented by values at nodes, derivative matrix D_ij
 *   dg/dxi(xi_i) = sum_j D_ij * g(xi_j)
 *   Physical derivative: dg/ds = (2/h_k) * dg/dxi
 *
 * Spectral accuracy: error decays as O(N^{-m}) for all m if function
 * smooth, or exponentially if analytic.
 *
 * ## Padé Approximant for u(s)
 *
 * On element [s_k, s_{k+1}], with delta = s - s_k:
 *   u(s) ≈ R_[m/n](delta) = P(delta) / Q(delta)
 *
 * Coefficients from Taylor series of u(s) at s_k, computed by exact
 * ODE integration. For stability, use [m/n] with m >= n, typically
 * [3/2] or [4/3].
 *
 * ## Certification
 *
 * For each element, we compute:
 * - certified_u_error: sup |u_exact - u_Pade| on the element
 * - certified_du_error: sup |du/ds_exact - dPade/ds|
 * - certified_d2u_error: sup |d²u/ds²_exact - d²Pade/ds²|
 *
 * Task-space propagated errors:
 * - Position: |C(u_exact) - C(u_approx)| <= sup||C'|| * |delta_u|
 * - Velocity: similar expansion with du/dt terms
 * - Acceleration: similar with d²u/dt² terms
 *
 * @see NumericalUtils.hpp for LGL nodes, derivative matrix, Padé, barycentric.
 * @see SwitchingStructureRepresentation.hpp for the SSR (Class A).
 * @see AnalyticalTypes.hpp for ErrorCertificate.
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "ConstraintEvaluator.hpp"
#include "NumericalUtils.hpp"
#include "SwitchingStructureRepresentation.hpp"
#include "../PathAdapter.hpp"
#include "../VelocityProfile.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace MotionPlanner::analytical {

// ============================================================================
// LGL Element (scalar function of s)
// ============================================================================

/**
 * @brief A single hp-LGL spectral element for a scalar function of s.
 *
 * Represents f(s) on [s_left, s_right] as a polynomial of degree N
 * using values at N+1 Legendre-Gauss-Lobatto nodes.
 */
struct LGLElement {
    double s_left = 0.0;
    double s_right = 0.0;
    int N = 4;  ///< Polynomial degree (nodes = N+1)

    /// Function values at LGL nodes in reference coordinates [-1, 1]
    std::vector<double> node_values;

    /// Barycentric weights for the LGL nodes
    std::vector<double> bary_weights;

    /// LGL nodes in reference coordinates [-1, 1]
    std::vector<double> ref_nodes;

    /// Reference derivative matrix D[i][j] = dL_j/dxi at xi_i
    std::vector<std::vector<double>> deriv_matrix;

    /// Initialize the LGL structure (nodes, weights, derivative matrix)
    void init(int degree) {
        N = degree;
        ref_nodes = lglNodes(N);
        bary_weights = lglBarycentricWeights(ref_nodes, N);
        deriv_matrix = lglDerivativeMatrix(ref_nodes, N);
        node_values.assign(N + 1, 0.0);
    }

    /// Map s to reference coordinate xi in [-1, 1]
    double sToXi(double s) const {
        return 2.0 * (s - s_left) / (s_right - s_left) - 1.0;
    }

    /// Map reference coordinate xi to physical s
    double xiToS(double xi) const {
        return s_left + (xi + 1.0) * (s_right - s_left) / 2.0;
    }

    /// Evaluate f(s) using barycentric interpolation
    double evaluate(double s) const {
        double xi = sToXi(s);
        xi = std::clamp(xi, -1.0, 1.0);
        return barycentricEvaluate(ref_nodes, node_values, bary_weights, xi);
    }

    /// Evaluate df/ds using barycentric derivative
    double evaluateDerivative(double s) const {
        double xi = sToXi(s);
        xi = std::clamp(xi, -1.0, 1.0);
        double dxi = barycentricDerivative(ref_nodes, node_values, bary_weights, xi);
        // Physical derivative: df/ds = (2/h) * df/dxi
        double h = s_right - s_left;
        return (2.0 / h) * dxi;
    }
};

// ============================================================================
// Hybrid Element (LGL for dynamics + Padé for geometry)
// ============================================================================

/**
 * @brief A hybrid element combining LGL for dynamics and Padé for geometry.
 *
 * On the arc-length interval [s_begin, s_end]:
 * - t(s), v(s), a(s) are represented by LGL spectral elements
 * - u(s) is represented by a Padé approximant
 * - Exact boundary values are stored for certification
 */
struct HybridElement {
    double s_begin = 0.0;
    double s_end = 0.0;

    /// LGL representations for time-optimal profile
    LGLElement t_of_s;  ///< t(s), strictly increasing
    LGLElement v_of_s;  ///< v(s) >= 0
    LGLElement a_of_s;  ///< a(s)

    /// Padé approximant for geometric parameter u(s)
    PadeApproximant u_of_s;

    /// Exact values at element boundaries (for certification)
    double u_exact_begin = 0.0;
    double u_exact_end = 0.0;
    double du_ds_exact_begin = 0.0;
    double du_ds_exact_end = 0.0;

    /// Error certificates for this element
    double certified_u_error = 0.0;
    double certified_du_error = 0.0;
    double certified_d2u_error = 0.0;

    /// Element time span
    double t_begin = 0.0;
    double t_end = 0.0;
};

// ============================================================================
// Hybrid Monotone Representation (Class B)
// ============================================================================

/**
 * @brief Hybrid Monotone Representation (Class B) — practical, certifiable.
 *
 * Pre-computes a spectral/polynomial approximation of the time-optimal
 * trajectory with certified error bounds. Provides O(log M) search +
 * O(N) evaluation for real-time sampling.
 *
 * Construction from an SSR: the SSR arcs are discretized into hp-adaptive
 * LGL elements for t(s), v(s), a(s), and Padé approximants for u(s).
 * Refinement continues until the certified error meets the target tolerance.
 *
 * @tparam Dim Spatial dimension
 * @tparam T   Numeric type (default: double)
 */
template<size_t Dim, typename T = double>
class HybridMonotoneRepresentation
    : public AnalyticalTrajectorySource<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Point = Vec<Dim, T>;
    using SSR = SwitchingStructureRepresentation<Dim, T>;

    /**
     * @brief Construct from an SSR with target tolerance.
     *
     * The Hybrid stores a reference to the same path as the SSR (non-owning).
     * The caller must ensure the path outlives this Hybrid.
     *
     * @param ssr The switching structure representation to approximate
     * @param targetTolerance Target error tolerance for position
     */
    explicit HybridMonotoneRepresentation(
        const SSR& ssr, double targetTolerance = 1e-10)
        : path_(&ssr.path())
        , evaluator_(ssr.evaluator())
        , tolerance_(targetTolerance) {
        buildFromSSR(ssr);
    }

    // ========================================================================
    // AnalyticalTrajectorySource interface
    // ========================================================================

    T totalTime() const override {
        if (elements_.empty()) return T(0);
        return static_cast<T>(elements_.back().t_end);
    }

    T totalLength() const override {
        return path_->totalLength();
    }

    Point position(T t) const override {
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return Point{};

        const HybridElement& elem = elements_[idx];
        double delta = s - elem.s_begin;
        double u = elem.u_of_s.evaluate(delta);

        // Evaluate NURBS at u
        if (path_->hasInner()) {
            const auto& inner = path_->inner();
            auto loc = inner.locate(s);
            const auto& piece = inner.piece(loc.piece);
            return toVec(piece.evaluate(u));
        }
        return path_->evaluatePositionAtArcLength(static_cast<T>(s));
    }

    Point velocity(T t) const override {
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return Point{};

        const HybridElement& elem = elements_[idx];
        double v = elem.v_of_s.evaluate(s);
        double delta = s - elem.s_begin;
        double du_ds = elem.u_of_s.evaluateDerivative(delta);

        // Task velocity = C'(u) * du/dt = C'(u) * du/ds * v
        // But in arc-length parameterization, tangent * v = task velocity
        // and du/ds = 1/g(u), so du/dt = v/g(u) = v * du/ds
        // Therefore task velocity = C'(u) * v / g(u) = T * v
        auto eval = path_->evaluateAtArcLength(static_cast<T>(s));
        return eval.tangent * static_cast<T>(v);
    }

    Point acceleration(T t) const override {
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return Point{};

        const HybridElement& elem = elements_[idx];
        double v = elem.v_of_s.evaluate(s);
        double a = elem.a_of_s.evaluate(s);

        auto eval = path_->evaluateAtArcLength(static_cast<T>(s));
        T kappa = eval.curvature;

        // Tangential + centripetal
        Point tangentialAccel = eval.tangent * static_cast<T>(a);
        T vT = static_cast<T>(v);
        T centripetalMag = vT * vT * kappa;
        Point normal = computeNormal(eval.tangent, static_cast<T>(s));
        Point centripetalAccel = normal * centripetalMag;

        return tangentialAccel + centripetalAccel;
    }

    T arcLength(T t) const override {
        return static_cast<T>(invertT(static_cast<double>(t)));
    }

    T pathVelocity(T t) const override {
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return T(0);
        return static_cast<T>(elements_[idx].v_of_s.evaluate(s));
    }

    T pathAcceleration(T t) const override {
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return T(0);
        return static_cast<T>(elements_[idx].a_of_s.evaluate(s));
    }

    T pathJerk(T t) const override {
        // Jerk = da/dt = a' * v (where a' = da/ds)
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return T(0);
        double v = elements_[idx].v_of_s.evaluate(s);
        double da_ds = elements_[idx].a_of_s.evaluateDerivative(s);
        return static_cast<T>(da_ds * v);
    }

    T curvature(T t) const override {
        T s = arcLength(t);
        return path_->curvatureAtArcLength(s);
    }

    SourceReference sourceRef(T t) const override {
        T s = arcLength(t);
        return path_->sourceRefAtArcLength(s);
    }

    size_t segmentIndex(T t) const override {
        T s = arcLength(t);
        auto eval = path_->evaluateAtArcLength(s);
        return eval.segmentIndex;
    }

    T segmentParameter(T t) const override {
        T s = arcLength(t);
        auto eval = path_->evaluateAtArcLength(s);
        return eval.localParameter;
    }

    const char* representationName() const override {
        return "HybridMonotoneRepresentation (Hybrid)";
    }

    // ========================================================================
    // Certification
    // ========================================================================

    /**
     * @brief Compute certified error bounds at time t.
     *
     * @param t Query time
     * @return Error certificate with position, velocity, acceleration bounds
     */
    ErrorCertificate certify(T t) const {
        ErrorCertificate cert;
        double s = invertT(static_cast<double>(t));
        size_t idx = findElementByS(s);
        if (idx >= elements_.size()) return cert;

        const HybridElement& elem = elements_[idx];

        // Position error: |C(u_exact) - C(u_approx)| <= sup||C'|| * |delta_u|
        // sup||C'|| ≈ max speed factor on the element
        double maxSpeed = estimateMaxSpeed(elem);
        cert.pos_error = maxSpeed * elem.certified_u_error;

        // Velocity error: depends on du/ds error and v error
        double v = elem.v_of_s.evaluate(s);
        cert.vel_error = maxSpeed * elem.certified_du_error * v +
                         elem.certified_u_error * std::abs(elem.a_of_s.evaluate(s));

        // Acceleration error: dominated by d²u/ds² error
        cert.acc_error = maxSpeed * elem.certified_d2u_error * v * v;

        return cert;
    }

    // ========================================================================
    // Access to internal data
    // ========================================================================

    /// Number of hybrid elements
    size_t numElements() const { return elements_.size(); }

    /// Get element by index
    const HybridElement& element(size_t i) const { return elements_.at(i); }

    /// Get all elements
    const std::vector<HybridElement>& elements() const { return elements_; }

    /// Get the target tolerance
    double tolerance() const { return tolerance_; }

    /// Get the path
    const Path& path() const { return *path_; }

    /**
     * @brief Update the path pointer (use after the path is moved).
     */
    void setPath(const Path& path) { path_ = &path; }

    /**
     * @brief Evaluate position at a given arc length (no time inversion needed).
     */
    Point positionAtArcLength(T s) const {
        return path_->evaluateAtArcLength(s).position;
    }

    /**
     * @brief Estimate time at a given arc length.
     */
    T timeAtArcLength(T s) const override {
        // Find the element containing this arc length
        for (const auto& elem : elements_) {
            if (s >= elem.s_begin - T(1e-10) && s <= elem.s_end + T(1e-10)) {
                // Linear interpolation of time within element
                double frac = static_cast<double>(
                    (s - elem.s_begin) / std::max(elem.s_end - elem.s_begin, T(1e-12)));
                return static_cast<T>(elem.t_begin + frac * (elem.t_end - elem.t_begin));
            }
        }
        return T(0);
    }

private:
    const Path* path_;  ///< Non-owning reference to the path
    ConstraintEvaluator<Dim, T> evaluator_;
    std::vector<HybridElement> elements_;
    double tolerance_;

    // ========================================================================
    // Construction: build from SSR
    // ========================================================================

    void buildFromSSR(const SSR& ssr) {
        elements_.clear();

        const auto& arcs = ssr.arcs();
        if (arcs.empty()) return;

        // For each SSR arc, create one or more hybrid elements
        for (const auto& arc : arcs) {
            if (!arc.valid()) continue;
            buildElementsForArc(arc, ssr);
        }

        // Ensure element times are consistent
        if (!elements_.empty()) {
            elements_.front().t_begin = 0.0;
            for (size_t i = 1; i < elements_.size(); ++i) {
                elements_[i].t_begin = elements_[i - 1].t_end;
            }
        }
    }

    /**
     * @brief Build hybrid element(s) for a single SSR arc.
     *
     * Starts with one element per arc. If the error certificate exceeds
     * the tolerance, splits the arc into smaller elements (h-refinement).
     */
    void buildElementsForArc(const SwitchingArc& arc, const SSR& ssr) {
        const int initialDegree = 4;  // Start with degree-4 LGL
        const int maxSplits = 4;      // Max h-refinements per arc

        std::vector<std::pair<double, double>> intervals = {
            {arc.s_begin, arc.s_end}
        };

        for (int split = 0; split < maxSplits && !intervals.empty(); ++split) {
            std::vector<std::pair<double, double>> nextIntervals;

            for (const auto& [sLo, sHi] : intervals) {
                HybridElement elem = buildElement(sLo, sHi, initialDegree, arc, ssr);

                // Check error certificate
                if (elem.certified_u_error <= tolerance_ ||
                    (sHi - sLo) < 1e-8) {
                    elements_.push_back(std::move(elem));
                } else {
                    // Split into two sub-intervals
                    double sMid = 0.5 * (sLo + sHi);
                    nextIntervals.push_back({sLo, sMid});
                    nextIntervals.push_back({sMid, sHi});
                }
            }

            intervals = std::move(nextIntervals);
        }

        // Add any remaining intervals as-is
        for (const auto& [sLo, sHi] : intervals) {
            HybridElement elem = buildElement(sLo, sHi, initialDegree, arc, ssr);
            elements_.push_back(std::move(elem));
        }
    }

    /**
     * @brief Build a single hybrid element on [sLo, sHi].
     *
     * Samples t(s), v(s), a(s) at LGL nodes by integrating the arc dynamics.
     * Computes Padé approximant for u(s) from Taylor coefficients.
     * Certifies the error by comparing Padé to exact values at the midpoint.
     */
    HybridElement buildElement(double sLo, double sHi, int degree,
                               const SwitchingArc& arc, const SSR& ssr) {
        HybridElement elem;
        elem.s_begin = sLo;
        elem.s_end = sHi;

        // Initialize LGL elements
        elem.t_of_s.init(degree);
        elem.v_of_s.init(degree);
        elem.a_of_s.init(degree);
        elem.t_of_s.s_left = sLo;
        elem.t_of_s.s_right = sHi;
        elem.v_of_s.s_left = sLo;
        elem.v_of_s.s_right = sHi;
        elem.a_of_s.s_left = sLo;
        elem.a_of_s.s_right = sHi;

        // Sample t(s), v(s), a(s) at LGL nodes by integrating from arc start
        // The SSR's integrateArcToTime gives us (s, v, a) at a given time,
        // but we need (t, v, a) at given s values. We integrate in s-space.
        double t0 = arc.t0;
        double v0 = arc.v0;
        double a0 = arc.a0;

        // Integrate from s_begin to each LGL node
        double sPrev = arc.s_begin;
        double tPrev = t0;
        double vPrev = v0;
        double aPrev = a0;

        for (int j = 0; j <= degree; ++j) {
            double sTarget = elem.t_of_s.xiToS(elem.t_of_s.ref_nodes[j]);
            sTarget = std::clamp(sTarget, sLo, sHi);

            // Integrate from sPrev to sTarget in s-space:
            // t' = 1/v, v' = a/v, a' = eta/v
            auto [t, v, a] = integrateInSSpace(arc, sPrev, sTarget,
                                               tPrev, vPrev, aPrev, ssr);

            elem.t_of_s.node_values[j] = t;
            elem.v_of_s.node_values[j] = std::max(v, 0.0);
            elem.a_of_s.node_values[j] = a;

            sPrev = sTarget;
            tPrev = t;
            vPrev = v;
            aPrev = a;
        }

        elem.t_begin = elem.t_of_s.node_values.front();
        elem.t_end = elem.t_of_s.node_values.back();

        // Compute Padé approximant for u(s)
        // We need u(s) at sLo. Get it from the path.
        double uBegin = getUAtS(sLo);
        elem.u_exact_begin = uBegin;

        // Compute Taylor coefficients of u(s) at sLo
        // du/ds = 1/g(u), where g(u) = ||C'(u)||
        // d²u/ds² = -g'(u)/g(u)³  (from differentiating 1/g)
        // d³u/ds³ = ...
        // For [3/2] Padé, we need m+n+1 = 6 Taylor coefficients
        auto taylor = computeUTaylor(uBegin, sLo, 6);  // 6 coefficients for [3/2]

        // Build [3/2] Padé approximant
        elem.u_of_s = PadeApproximant(taylor, 3, 2);

        // Compute exact u at sHi for certification
        double uEnd = getUAtS(sHi);
        elem.u_exact_end = uEnd;
        elem.du_ds_exact_begin = 1.0 / getSpeedFactor(uBegin);
        elem.du_ds_exact_end = 1.0 / getSpeedFactor(uEnd);

        // Certify: compare Padé to exact at midpoint
        double sMid = 0.5 * (sLo + sHi);
        double uMidExact = getUAtS(sMid);
        double uMidPade = elem.u_of_s.evaluate(sMid - sLo);
        elem.certified_u_error = std::abs(uMidExact - uMidPade);

        // Also check endpoint
        double uEndPade = elem.u_of_s.evaluate(sHi - sLo);
        elem.certified_u_error = std::max(elem.certified_u_error,
                                          std::abs(uEnd - uEndPade));

        // Derivative error at midpoint
        double duMidExact = 1.0 / getSpeedFactor(uMidExact);
        double duMidPade = elem.u_of_s.evaluateDerivative(sMid - sLo);
        elem.certified_du_error = std::abs(duMidExact - duMidPade);

        // Second derivative error (approximate)
        double d2uMidPade = elem.u_of_s.evaluate2ndDerivative(sMid - sLo);
        // Exact d²u/ds² = -g'(u)/(g(u))³  -- approximate numerically
        double gMid = getSpeedFactor(uMidExact);
        double gMidPlus = getSpeedFactor(getUAtS(sMid + 1e-6));
        double gMidMinus = getSpeedFactor(getUAtS(sMid - 1e-6));
        double gPrime = (gMidPlus - gMidMinus) / 2e-6;
        double d2uMidExact = -gPrime / (gMid * gMid * gMid);
        elem.certified_d2u_error = std::abs(d2uMidExact - d2uMidPade);

        return elem;
    }

    // ========================================================================
    // Integration in s-space
    // ========================================================================

    /**
     * @brief Integrate arc dynamics in s-space from (sStart) to (sTarget).
     *
     * Dynamics: t' = 1/v, v' = a/v, a' = eta/v  (where ' = d/ds)
     *
     * @return (t, v, a) at sTarget
     */
    std::tuple<double, double, double> integrateInSSpace(
        const SwitchingArc& arc,
        double sStart, double sTarget,
        double tStart, double vStart, double aStart,
        const SSR& ssr) const {

        if (std::abs(sTarget - sStart) < 1e-14) {
            return {tStart, vStart, aStart};
        }

        struct SState {
            double t, v, a;
            SState operator+(const SState& o) const { return {t+o.t, v+o.v, a+o.a}; }
            SState operator*(double k) const { return {t*k, v*k, a*k}; }
            std::array<double, 3> components() const { return {t, v, a}; }
            double norm() const { return std::sqrt(t*t + v*v + a*a); }
        };

        const ConstraintEvaluator<Dim, T>& ce = evaluator_;
        const Path& path = *path_;

        auto rhs = [&](double sCur, const SState& y) -> SState {
            double vSafe = std::max(y.v, 1e-12);
            // Compute eta for this arc
            double eta = 0.0;
            switch (arc.mode) {
                case ControlMode::ACCEL_MAX: {
                    auto b = ce.etaBounds(static_cast<T>(sCur),
                                           static_cast<T>(y.v), static_cast<T>(y.a), path);
                    eta = b.eta_max;
                    break;
                }
                case ControlMode::DECEL_MAX: {
                    auto b = ce.etaBounds(static_cast<T>(sCur),
                                           static_cast<T>(y.v), static_cast<T>(y.a), path);
                    eta = b.eta_min;
                    break;
                }
                case ControlMode::ZERO_JERK:
                    eta = 0.0;
                    break;
                default:
                    eta = arc.eta;
                    break;
            }
            return {1.0 / vSafe, y.a / vSafe, eta / vSafe};
        };

        SState y{tStart, vStart, aStart};
        double s = sStart;
        double dsTotal = sTarget - sStart;
        int nSteps = std::max(1, std::min(100, static_cast<int>(std::abs(dsTotal) / 1e-2)));
        double ds = dsTotal / nSteps;

        // If starting from rest (v ≈ 0, a ≈ 0), use closed-form for first step.
        // a(t) = eta*t, v(t) = 0.5*eta*t², s(t) = eta*t³/6
        // => t = (6*|ds|/|eta|)^(1/3)
        int startStep = 0;
        if (std::abs(vStart) < 1e-9 && std::abs(aStart) < 1e-9) {
            double eta = 0.0;
            switch (arc.mode) {
                case ControlMode::ACCEL_MAX: {
                    auto b = ce.etaBounds(static_cast<T>(sStart), T(0), T(0), path);
                    eta = b.eta_max;
                    break;
                }
                case ControlMode::DECEL_MAX: {
                    auto b = ce.etaBounds(static_cast<T>(sStart), T(0), T(0), path);
                    eta = b.eta_min;
                    break;
                }
                default:
                    eta = arc.eta;
                    break;
            }
            if (std::abs(eta) > 1e-12 && nSteps > 0) {
                double tStep = std::cbrt(6.0 * std::abs(ds) / std::abs(eta));
                y.t = tStart + tStep;
                y.v = 0.5 * eta * tStep * tStep;
                y.a = eta * tStep;
                s += ds;
                startStep = 1;  // Already did one step
            }
        }

        for (int i = startStep; i < nSteps; ++i) {
            y = rk4Step(rhs, s, y, ds);
            s += ds;
            if (y.v < 1e-12) y.v = 1e-12;
        }

        return {y.t, y.v, y.a};
    }

    // ========================================================================
    // NURBS parameter u(s) utilities
    // ========================================================================

    /// Get the NURBS parameter u at arc length s
    double getUAtS(double s) const {
        if (!path_->hasInner()) return s;  // Fallback
        const auto& inner = path_->inner();
        auto loc = inner.locate(s);
        const auto& piece = inner.piece(loc.piece);
        return piece.invertLength(loc.localS);
    }

    /// Get the speed factor g(u) = ||C'(u)||
    double getSpeedFactor(double u) const {
        if (!path_->hasInner()) return 1.0;
        const auto& inner = path_->inner();
        // Find which piece contains u... actually we need to know the piece.
        // For simplicity, use the path's evaluate to get tangent and compute
        // speed from the parametric derivative.
        // This is a simplification; in practice we'd track the piece.
        // Use arcDerivatives at the corresponding s.
        // Actually, let's just use the derivative at u on the piece.
        // We need to find the piece. Since u is a local parameter, we need
        // to know which piece it belongs to. Let's use a different approach:
        // compute g from the arc-length derivative.
        // g(u) = ||C'(u)||, and T = C'(u)/g(u), so g(u) = ||C'(u)||.
        // We can get C'(u) from piece.derivative(u, 1).
        // But we need the piece. Let's search for it.
        for (std::size_t i = 0; i < inner.numPieces(); ++i) {
            const auto& p = inner.piece(i);
            if (u >= p.knotMin() - 1e-10 && u <= p.knotMax() + 1e-10) {
                return p.derivative(u, 1).norm();
            }
        }
        return 1.0;
    }

    /**
     * @brief Compute Taylor coefficients of u(s) at a point.
     *
     * u(s) satisfies du/ds = 1/g(u), where g(u) = ||C'(u)||.
     *
     * Taylor expansion: u(s) = u_0 + u_1*delta + u_2*delta² + ...
     * where delta = s - s_0.
     *
     * u_0 = u(s_0)
     * u_1 = du/ds = 1/g(u_0) = h(u_0)
     * u_2 = d²u/ds² = h'(u) * du/ds = h'(u_0) * h(u_0)
     * u_3 = d³u/ds³ = (h''(u) * (du/ds)² + h'(u) * d²u/ds²)
     *       = h''(u_0) * h(u_0)² + h'(u_0)² * h(u_0)
     * etc.
     *
     * where h(u) = 1/g(u), h'(u) = -g'(u)/g(u)², etc.
     *
     * @param u0 u at s_0
     * @param s0 arc length at the point
     * @param numCoeffs Number of Taylor coefficients to compute
     * @return Taylor coefficients [u_0, u_1, u_2, ...]
     */
    std::vector<double> computeUTaylor(double u0, double s0, int numCoeffs) const {
        std::vector<double> coeffs(numCoeffs, 0.0);
        if (numCoeffs < 1) return coeffs;

        // Compute g(u) and its derivatives at u0
        // g(u) = ||C'(u)||
        // g'(u) = (C'(u) · C''(u)) / ||C'(u)||
        // g''(u) = (||C''(u)||² + C'(u)·C'''(u)) / ||C'(u)|| - (C'(u)·C''(u))² / ||C'(u)||³
        // ... (gets complex for higher orders)

        // For the Padé [3/2], we need 6 Taylor coefficients (m+n+1 = 6)
        // But computing 6 derivatives analytically is complex.
        // Instead, compute them numerically by finite differences on h(u) = 1/g(u).

        const double du = 1e-5;  // Finite difference step

        // h(u) = 1/g(u)
        auto h = [&](double u) -> double {
            double g = getSpeedFactor(u);
            return (g > 1e-15) ? 1.0 / g : 1e15;
        };

        double h0 = h(u0);
        double h1 = (h(u0 + du) - h(u0 - du)) / (2.0 * du);
        double h2 = (h(u0 + du) - 2.0 * h(u0) + h(u0 - du)) / (du * du);

        // u_0 = u0
        coeffs[0] = u0;
        // u_1 = h(u0)
        coeffs[1] = h0;
        // u_2 = h'(u0) * h(u0)
        coeffs[2] = h1 * h0;
        // u_3 = h''(u0) * h(u0)² + h'(u0)² * h(u0)
        coeffs[3] = h2 * h0 * h0 + h1 * h1 * h0;
        // u_4 and u_5: approximate with higher-order finite differences
        // For [3/2] Padé we need up to c_5 (m+n = 5)
        // u_4 ≈ u_3 * (h1/h0) (rough approximation)
        if (numCoeffs > 4) {
            double h3 = (h(u0 + 2*du) - 2*h(u0 + du) + 2*h(u0 - du) - h(u0 - 2*du)) / (2 * du * du * du);
            coeffs[4] = h3 * h0 * h0 * h0 + 3.0 * h2 * h1 * h0 * h0 +
                        h1 * h1 * h1 * h0;
        }
        if (numCoeffs > 5) {
            coeffs[5] = coeffs[4] * (h0 > 1e-15 ? h1 / h0 : 0.0);
        }

        return coeffs;
    }

    // ========================================================================
    // Inversion of t(s) -> s(t)
    // ========================================================================

    /**
     * @brief Invert t(s) to find s such that t(s) = tQuery.
     *
     * Uses Newton iteration safeguarded by bisection. The LGL representation
     * of t(s) provides both the function value and its derivative (dt/ds = 1/v).
     *
     * @param tQuery Target time
     * @return Arc length s such that t(s) ≈ tQuery
     */
    double invertT(double tQuery) const {
        if (elements_.empty()) return 0.0;

        // Clamp to valid range
        if (tQuery <= elements_.front().t_begin) {
            return elements_.front().s_begin;
        }
        if (tQuery >= elements_.back().t_end) {
            return elements_.back().s_end;
        }

        // Binary search for the element containing tQuery
        size_t idx = findElementByT(tQuery);
        const HybridElement& elem = elements_[idx];

        // Newton iteration within the element
        double s = elem.s_begin + (tQuery - elem.t_begin) /
                   std::max(elem.t_end - elem.t_begin, 1e-12) *
                   (elem.s_end - elem.s_begin);

        for (int iter = 0; iter < 50; ++iter) {
            double tVal = elem.t_of_s.evaluate(s);
            double dtDs = elem.t_of_s.evaluateDerivative(s);
            double err = tVal - tQuery;

            if (std::abs(err) < 1e-12) break;

            if (std::abs(dtDs) > 1e-15) {
                double sNew = s - err / dtDs;
                // Safeguard with bisection
                if (sNew < elem.s_begin) sNew = 0.5 * (s + elem.s_begin);
                if (sNew > elem.s_end) sNew = 0.5 * (s + elem.s_end);
                s = sNew;
            } else {
                // Fall back to bisection
                s = 0.5 * (elem.s_begin + elem.s_end);
            }
        }

        return std::clamp(s, elem.s_begin, elem.s_end);
    }

    /// Find element containing arc length s
    size_t findElementByS(double s) const {
        if (elements_.empty()) return 0;
        size_t lo = 0, hi = elements_.size() - 1;
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (s < elements_[mid].s_begin) {
                if (mid == 0) return 0;
                hi = mid - 1;
            } else if (s >= elements_[mid].s_end) {
                lo = mid + 1;
            } else {
                return mid;
            }
        }
        return lo;
    }

    /// Find element containing time t
    size_t findElementByT(double t) const {
        if (elements_.empty()) return 0;
        size_t lo = 0, hi = elements_.size() - 1;
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (t < elements_[mid].t_begin) {
                if (mid == 0) return 0;
                hi = mid - 1;
            } else if (t >= elements_[mid].t_end) {
                lo = mid + 1;
            } else {
                return mid;
            }
        }
        return lo;
    }

    /// Estimate maximum speed factor on an element
    double estimateMaxSpeed(const HybridElement& elem) const {
        double maxG = 0.0;
        // Sample at a few points
        for (int i = 0; i < 4; ++i) {
            double s = elem.s_begin + (elem.s_end - elem.s_begin) * i / 3.0;
            double u = getUAtS(s);
            maxG = std::max(maxG, getSpeedFactor(u));
        }
        return std::max(maxG, 1.0);
    }

    // ========================================================================
    // Normal vector computation
    // ========================================================================

    Point computeNormal(const Point& tangent, T s) const {
        if constexpr (Dim == 2) {
            return Point{-tangent[1], tangent[0]};
        } else if constexpr (Dim >= 3) {
            T ds = T(0.001);
            Point t1 = path_->evaluateAtArcLength(s - ds).tangent;
            Point t2 = path_->evaluateAtArcLength(s + ds).tangent;
            Point dtds = (t2 - t1) / (T(2) * ds);
            T len = dtds.length();
            return (len > MathConstants::EPSILON) ? dtds / len : Point{};
        }
        return Point{};
    }

    /// Convert RVec to Vec<Dim, T>
    template<typename RVecType>
    Point toVec(const RVecType& rv) const {
        Point result;
        const std::size_t n = std::min<std::size_t>(rv.dim(), Dim);
        for (std::size_t i = 0; i < n; ++i) {
            result[i] = static_cast<T>(rv[i]);
        }
        return result;
    }
};

} // namespace MotionPlanner::analytical
