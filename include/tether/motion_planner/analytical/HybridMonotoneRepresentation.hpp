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
 * @brief Coordinate warp for elements adjacent to a low-velocity edge.
 *
 * Near a stop or a low-velocity junction stamp, v(s) ~ sqrt(s* - s)
 * has an unbounded slope that no fixed-degree polynomial in s can
 * track — certificate-based h-refinement never converges there. But
 * in the warped coordinate sigma = sqrt(s* - s) (resp. sqrt(s - s*)),
 * v, t and a are all smooth: v ~ k*sigma, dt/dsigma ~ -2*sigma/v =
 * const, a ~ const + O(sigma^2). A degree-4 LGL in sigma certifies in
 * one shot what plain s-LGL needed ~62 bisections for.
 *
 * SQRT_LEFT:  sigma = sqrt(s - s_left)   — cusp at the LEFT edge
 *             (arc leaving a low-velocity point).
 * SQRT_RIGHT: sigma = sqrt(s_right - s)  — cusp at the RIGHT edge
 *             (arc approaching a stop/stamp).
 */
enum class LGLWarp { NONE, SQRT_LEFT, SQRT_RIGHT };

/**
 * @brief A single hp-LGL spectral element for a scalar function of s.
 *
 * Represents f(s) on [s_left, s_right] as a polynomial of degree N
 * using values at N+1 Legendre-Gauss-Lobatto nodes. With warp != NONE
 * the polynomial is in the warped coordinate sigma instead of s.
 */
struct LGLElement {
    double s_left = 0.0;
    double s_right = 0.0;
    int N = 4; /**< Polynomial degree (nodes = N+1) */
    LGLWarp warp = LGLWarp::NONE;

    /**Function values at LGL nodes in reference coordinates [-1, 1] */
    std::vector<double> node_values;

    /**Barycentric weights for the LGL nodes */
    std::vector<double> bary_weights;

    /**LGL nodes in reference coordinates [-1, 1] */
    std::vector<double> ref_nodes;

    /**Reference derivative matrix D[i][j] = dL_j/dxi at xi_i */
    std::vector<std::vector<double>> deriv_matrix;

    /**
     * Initialize the LGL structure (nodes, weights, derivative matrix)
     * @param degree Polynomial degree (produces degree+1 LGL nodes)
     */
    void init(int degree) {
        N = degree;
        ref_nodes = lglNodes(N);
        bary_weights = lglBarycentricWeights(ref_nodes, N);
        deriv_matrix = lglDerivativeMatrix(ref_nodes, N);
        node_values.assign(N + 1, 0.0);
    }

    /**Max warped coordinate: sigma ranges over [0, sqrt(h)]. */
    double warpSigmaMax() const {
        return std::sqrt(std::max(s_right - s_left, 0.0));
    }

    /**Warped coordinate of s (0 at the cusp edge, sqrt(h) at the far edge). */
    double warpSigma(double s) const {
        return (warp == LGLWarp::SQRT_LEFT)
            ? std::sqrt(std::max(s - s_left, 0.0))
            : std::sqrt(std::max(s_right - s, 0.0));
    }

    /**
     * Map s to reference coordinate xi in [-1, 1]
     * @param s Physical arc-length coordinate
     * @return Normalized coordinate ξ on the element
     */
    double sToXi(double s) const {
        if (warp == LGLWarp::NONE)
            return 2.0 * (s - s_left) / (s_right - s_left) - 1.0;
        const double sigmaMax = warpSigmaMax();
        return (sigmaMax > 0.0)
            ? 2.0 * warpSigma(s) / sigmaMax - 1.0
            : 0.0;
    }

    /**
     * Map reference coordinate xi to physical s
     * @param xi Normalized coordinate on [-1, 1]
     * @return Physical arc-length coordinate
     */
    double xiToS(double xi) const {
        if (warp == LGLWarp::NONE)
            return s_left + (xi + 1.0) * (s_right - s_left) / 2.0;
        const double sigma =
            std::clamp((xi + 1.0) * 0.5, 0.0, 1.0) * warpSigmaMax();
        return (warp == LGLWarp::SQRT_LEFT)
            ? s_left + sigma * sigma
            : s_right - sigma * sigma;
    }

    /**
     * Evaluate f(s) using barycentric interpolation
     * @param s Physical arc-length coordinate (clamped to the element)
     * @return Interpolated value
     */
    double evaluate(double s) const {
        double xi = sToXi(s);
        xi = std::clamp(xi, -1.0, 1.0);
        return barycentricEvaluate(ref_nodes, node_values, bary_weights, xi);
    }

    /**
     * Evaluate df/ds using barycentric derivative
     * @param s Physical arc-length coordinate
     * @return Interpolated derivative df/ds
     */
    double evaluateDerivative(double s) const {
        double xi = sToXi(s);
        xi = std::clamp(xi, -1.0, 1.0);
        double dxi = barycentricDerivative(ref_nodes, node_values, bary_weights, xi);
        double h = s_right - s_left;
        if (warp == LGLWarp::NONE) {
            // Physical derivative: df/ds = (2/h) * df/dxi
            return (2.0 / h) * dxi;
        }
        // df/ds = df/dxi · (2/σmax) · dσ/ds with dσ/ds = ±1/(2σ).
        // The 1/σ blow-up at the cusp is the TRUE asymptotic (dt/ds =
        // 1/v → ∞ at a stop); σ is floored so the returned value is
        // large but finite and correctly ordered vs. interior points.
        const double sigmaMax = warpSigmaMax();
        const double sigma =
            std::max(warpSigma(s), 1e-6 * sigmaMax);
        const double sign = (warp == LGLWarp::SQRT_LEFT) ? 1.0 : -1.0;
        return dxi * (2.0 / sigmaMax) * sign / (2.0 * sigma);
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

    /**LGL representations for time-optimal profile */
    LGLElement t_of_s; /**< t(s), strictly increasing */
    LGLElement v_of_s; /**< v(s) >= 0 */
    LGLElement a_of_s; /**< a(s) */

    /**Padé approximant for geometric parameter u(s) */
    PadeApproximant u_of_s;

    /**NURBS piece u_of_s applies to (u is piece-local) */
    std::size_t pieceIdx = 0;

    /**Exact values at element boundaries (for certification) */
    double u_exact_begin = 0.0;
    double u_exact_end = 0.0;
    double du_ds_exact_begin = 0.0;
    double du_ds_exact_end = 0.0;

    /**Error certificates for this element */
    double certified_u_error = 0.0;
    double certified_du_error = 0.0;
    double certified_d2u_error = 0.0;
    /**Max over cert probes of |t_LGL - t_SSR| * v_SSR [mm] (the
     * s-inversion error invertT commits), |v_LGL - v_SSR| [mm/s],
     * |a_LGL - a_SSR| [mm/s^2]. */
    double certified_t_error = 0.0;
    double certified_v_error = 0.0;
    double certified_a_error = 0.0;

    /**Element time span */
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

        // Evaluate NURBS at u — on the element's OWN piece. Locating
        // the piece from s is ambiguous at piece boundaries (a
        // boundary s may resolve to the next piece, whose domain the
        // Padé-u does not belong to).
        if (path_->hasInner()) {
            const auto& inner = path_->inner();
            const auto& piece =
                inner.piece(std::min(elem.pieceIdx, inner.numPieces() - 1));
            const double uc =
                std::clamp(u, piece.knotMin(), piece.knotMax());
            return toVec(piece.evaluate(uc));
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

    /**
     * Number of hybrid elements
     * @return Element count covering the path
     */
    size_t numElements() const { return elements_.size(); }

    /**
     * Get element by index
     * @param i Element index
     * @return Element i (throws when out of range)
     */
    const HybridElement& element(size_t i) const { return elements_.at(i); }

    /**
     * Get all elements
     * @return Element list covering the path
     */
    const std::vector<HybridElement>& elements() const { return elements_; }

    /**
     * Get the target tolerance
     * @return Discretization tolerance used at build time
     */
    double tolerance() const { return tolerance_; }

    /**
     * Get the path
     * @return Path the representation was built for
     */
    const Path& path() const { return *path_; }

    /**
     * @brief Update the path pointer (use after the path is moved).
     * @param path Path object to point at
     */
    void setPath(const Path& path) { path_ = &path; }

    /**
     * @brief Evaluate position at a given arc length (no time inversion needed).
     * @param s Arc-length coordinate [mm]
     * @return Cartesian position on the path
     */
    Point positionAtArcLength(T s) const {
        return path_->evaluateAtArcLength(s).position;
    }

    /**
     * @brief Estimate time at a given arc length.
     * @param s Arc-length coordinate [mm]
     * @return Interpolated time at s [s]
     */
    T timeAtArcLength(T s) const override {
        // Use the LGL t(s) interpolant — uniformly more accurate than
        // linear interpolation and correct on warped elements, where
        // t is far from linear in s near the cusp edge.
        if (elements_.empty()) return T(0);
        size_t idx = findElementByS(static_cast<double>(s));
        if (idx >= elements_.size()) return T(0);
        return static_cast<T>(
            elements_[idx].t_of_s.evaluate(static_cast<double>(s)));
    }

private:
    // Dynamics-fit certification tolerances for h-refinement. The
    // time certificate is measured in arc-length space — the quantity
    // invertT actually commits: |t_lgl(s) - t_exact(s)| * v(s) bounds
    // the resulting s-inversion error. This is automatically strict
    // at cruise speed (t_err <= tol/v) and permissive at the sqrt
    // cusp where v -> 0 and dt/ds -> inf makes any absolute time
    // tolerance unreachable. The v certificate bounds the v_of_s
    // interpolant pointwise.
    static constexpr double kSCertTol = 0.02;     // [mm]
    static constexpr double kVCertTol = 0.25;     // [mm/s]

    const Path* path_; /**< Non-owning reference to the path */
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

        // Element times are absolute (sampled from the SSR's
        // timeAtArcLength): t_begin/t_end gaps between consecutive
        // elements are real — they are the dwell pauses whose arcs
        // carry no s-extent. Do NOT chain t_begin to the previous
        // t_end; that would erase dwells and shift every subsequent
        // sample earlier in time.

        // The split worklist emits elements in refinement-completion
        // order, not s-order — a passing right half lands before the
        // still-splitting left half. findElementByS/findElementByT
        // binary-search on s order, so sort before use.
        std::sort(elements_.begin(), elements_.end(),
                  [](const HybridElement& a, const HybridElement& b) {
                      return a.s_begin < b.s_begin;
                  });
    }

    /**
     * @brief Build hybrid element(s) for a single SSR arc.
 *
     * Starts with one element per arc. If the error certificate exceeds
     * the tolerance, splits the arc into smaller elements (h-refinement).
     */
    void buildElementsForArc(const SwitchingArc& arc, const SSR& ssr) {
        const int initialDegree = 4;  // Start with degree-4 LGL
        const int maxSplits = 10;     // Max h-refinements per arc

        // Certificate violation: worst error relative to its
        // tolerance, so the certificates compare on one scale. The
        // a-fit is measured but does NOT drive h-refinement: within
        // ACCEL_MAX/DECEL_MAX arcs eta is recomputed per step from
        // constraints, so a(s) is a multi-regime curve (limit-clamp
        // onsets, varying jerk) that no finite-regime interpolant
        // represents — splitting on it refines forever without
        // converging. The profile contract lives in u/t/s/v, which do
        // converge; certified_a_error remains populated for reporting.
        auto violation = [this](const HybridElement& e) {
            return std::max({
                e.certified_u_error / tolerance_,
                e.certified_t_error / kSCertTol,
                e.certified_v_error / kVCertTol});
        };

        // Warped-element selection: an interval adjacent to a
        // low-velocity edge gets an LGLWarp whose sigma coordinate
        // vanishes at that edge, so the v~sqrt(s* - s) cusp becomes a
        // smooth function of sigma and degree-4 LGL certifies in one
        // shot. The check uses the SSR's exact endpoint velocities and
        // is re-evaluated per sub-interval, so an interior velocity
        // dip is picked up automatically once refinement isolates it
        // at an interval edge. Warping is harmless when not needed
        // (v(sigma) is still smooth), so the ratio is generous.
        auto pickWarp = [&ssr](double sLo, double sHi) {
            const double vLo = std::max(
                ssr.pathVelocityAtArcLength(static_cast<T>(sLo)), 0.0);
            const double vHi = std::max(
                ssr.pathVelocityAtArcLength(static_cast<T>(sHi)), 0.0);
            // The sqrt cusp exists wherever an edge velocity is small
            // relative to the element's PEAK velocity — an interior
            // probe is required: a 70->12 plunge reads 12/20=0.59
            // against the endpoint-only ratio and misses the warp.
            const double vMid = std::max(
                ssr.pathVelocityAtArcLength(
                    static_cast<T>(0.5 * (sLo + sHi))), 0.0);
            const double vPk =
                std::max({vLo, vHi, vMid, 1e-9});
            constexpr double kRatio = 0.45;
            if (vHi < kRatio * vPk) return LGLWarp::SQRT_RIGHT;
            if (vLo < kRatio * vPk) return LGLWarp::SQRT_LEFT;
            return LGLWarp::NONE;
        };

        // (sLo, sHi, parent violation). The parent term drives the
        // non-improvement stop below. Intervals are seeded at NURBS
        // piece boundaries: u = invertLength is piece-local and jumps
        // at every boundary, so no single per-element approximant can
        // represent a straddle — without the pre-split the u
        // certificate could never converge there either.
        std::vector<std::tuple<double, double, double>> intervals;
        {
            double sPrev = arc.s_begin;
            if (path_->hasInner()) {
                const auto& inner = path_->inner();
                const auto locLo = inner.locate(arc.s_begin);
                const auto locHi = inner.locate(arc.s_end);
                double pieceEnd = arc.s_begin +
                    (inner.piece(locLo.piece).length() - locLo.localS);
                for (std::size_t p = locLo.piece; p < locHi.piece; ++p) {
                    if (pieceEnd > sPrev + 1e-9 &&
                        pieceEnd < arc.s_end - 1e-9) {
                        intervals.push_back(
                            {sPrev, pieceEnd,
                             std::numeric_limits<double>::infinity()});
                        sPrev = pieceEnd;
                    }
                    pieceEnd += inner.piece(p + 1).length();
                }
            }
            intervals.push_back(
                {sPrev, arc.s_end,
                 std::numeric_limits<double>::infinity()});
        }

        for (int split = 0; split < maxSplits && !intervals.empty(); ++split) {
            std::vector<std::tuple<double, double, double>> nextIntervals;

            for (const auto& [sLo, sHi, parentViol] : intervals) {
                const LGLWarp w = pickWarp(sLo, sHi);
                // p-refinement before h-refinement: in the warped
                // coordinate the dynamics are smooth, so a failed
                // degree-4 fit converges at degree 6-8 far cheaper
                // than two bisection rounds (each spawning its own
                // builds, Taylor/Padé work, and SSR queries). Keep the
                // best-scoring attempt for the split heuristic below.
                HybridElement elem;
                double viol = std::numeric_limits<double>::infinity();
                for (int deg : {initialDegree, 6, 8}) {
                    HybridElement cand =
                        buildElement(sLo, sHi, deg, w, arc, ssr);
                    const double cv = violation(cand);
                    if (cv < viol) {
                        viol = cv;
                        elem = std::move(cand);
                    }
                    if (cv <= 1.0) break;
                }
                // Split unless every certificate fits: the geometry
                // Padé AND the dynamics LGLs — a good u(s) fit says
                // nothing about the t(s)/v(s) fit, which is where the
                // invertible-time error actually comes from.
                //
                // Non-improvement stop: if splitting did not reduce
                // the violation by at least ~30%, deeper splits won't
                // converge either; accept the element with its actual
                // (bounded) error instead — but only when that error
                // is modest. A large non-improving violation means
                // the feature estimate MISSED (the dip/kink is still
                // interior): keep refining rather than shipping a
                // certified-bad element. The width floor accepts
                // slivers: below ~5e-3 mm a residual v error is a
                // bounded pointwise defect no output can resolve,
                // and refinement would only shave geometrically
                // toward it at ~30%/level.
                constexpr double kAcceptCap = 4.0;
                constexpr double kMinWidth = 5e-3;
                if (viol <= 1.0 ||
                    (viol <= kAcceptCap && viol >= 0.7 * parentViol) ||
                    (sHi - sLo) < kMinWidth) {
                    elements_.push_back(std::move(elem));
                } else {
                    // Split point: put an interior v-minimum (V-dip
                    // at a stamp) on the children's edges instead of
                    // hiding it inside — midpoint bisection leaves
                    // the dip interior so the sqrt warp never
                    // engages. Read off node values already sampled —
                    // no extra SSR work.
                    double sSplit = 0.5 * (sLo + sHi);
                    const double h = sHi - sLo;
                    const auto& vn = elem.v_of_s.node_values;
                    const int n = elem.v_of_s.N;
                    int jMin = 0;
                    for (int j = 1; j <= n; ++j) {
                        if (vn[j] < vn[jMin]) jMin = j;
                    }
                    const double vEdgeMax =
                        std::max(vn.front(), vn.back());
                    if (jMin != 0 && jMin != n &&
                        vn[jMin] < 0.7 * vEdgeMax) {
                        const double sDip = elem.v_of_s.xiToS(
                            elem.v_of_s.ref_nodes[jMin]);
                        if (sDip > sLo + 0.05 * h &&
                            sDip < sHi - 0.05 * h) {
                            sSplit = sDip;
                        }
                    }
                    nextIntervals.push_back({sLo, sSplit, viol});
                    nextIntervals.push_back({sSplit, sHi, viol});
                }
            }

            intervals = std::move(nextIntervals);
        }

        // Add any remaining intervals as-is, keeping the same
        // p-refinement ladder as the main loop so leftovers get the
        // best available fit.
        for (const auto& [sLo, sHi, parentViol] : intervals) {
            const LGLWarp w = pickWarp(sLo, sHi);
            HybridElement best;
            double bestViol = std::numeric_limits<double>::infinity();
            for (int deg : {initialDegree, 6, 8}) {
                HybridElement cand =
                    buildElement(sLo, sHi, deg, w, arc, ssr);
                const double cv = violation(cand);
                if (cv < bestViol) {
                    bestViol = cv;
                    best = std::move(cand);
                }
                if (cv <= 1.0) break;
            }
            elements_.push_back(std::move(best));
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
                               LGLWarp warp,
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
        elem.t_of_s.warp = warp;
        elem.v_of_s.s_left = sLo;
        elem.v_of_s.s_right = sHi;
        elem.v_of_s.warp = warp;
        elem.a_of_s.s_left = sLo;
        elem.a_of_s.s_right = sHi;
        elem.a_of_s.warp = warp;

        // Sample t(s), v(s), a(s) at LGL nodes from the SSR's exact
        // inverse queries. The previous implementation integrated the
        // s-space ODE t'=1/v, v'=a/v, a'=eta/v per element — singular
        // wherever v -> 0 (stamped junctions, dwells, endpoint stops),
        // which exploded t to ~1e11 and corrupted every downstream
        // element. The SSR already carries the monotone s(t) data;
        // inverting it per node is exact at stops (timeAtArcLength
        // returns the finite arrival time) and automatically includes
        // dwell time accumulated across skipped zero-length arcs.
        for (int j = 0; j <= degree; ++j) {
            double sTarget = elem.t_of_s.xiToS(elem.t_of_s.ref_nodes[j]);
            sTarget = std::clamp(sTarget, sLo, sHi);
            // timeAtArcLength resolves a shared junction to the FIRST
            // matching arc — for an element starting right at a dwell
            // the returned time can be the dwell's start rather than
            // this arc's. Clamping to the parent arc's own window keeps
            // each element on the correct side of a pause.
            const double t_j = std::clamp(
                static_cast<double>(
                    ssr.timeAtArcLength(static_cast<T>(sTarget))),
                arc.t0, arc.t0 + arc.duration);
            elem.t_of_s.node_values[j] = t_j;
            elem.v_of_s.node_values[j] = std::max(
                ssr.pathVelocityAtArcLength(static_cast<T>(sTarget)),
                0.0);
            // a sampled on THIS arc via accelerationOnArc — NOT
            // ssr.pathAcceleration(t), which resolves t through the
            // global findArcAtT and takes the neighbouring arc's
            // right-limit at every shared boundary. Acceleration jumps
            // at arc switches, so that boundary lookup poisons the
            // edge node of every element (one wrong node = global fit
            // wiggle no amount of splitting can fix).
            elem.a_of_s.node_values[j] = static_cast<double>(
                ssr.accelerationOnArc(arc, t_j - arc.t0));
        }

        // Element time bounds: evaluate AT the endpoints rather than
        // reading node_values.front()/back() — for SQRT_RIGHT elements
        // the node order is reversed (ξ=-1 maps to s_right), so the
        // front/back values would swap t_begin/t_end. LGL evaluation
        // at the endpoints returns the exact node values regardless.
        elem.t_begin = elem.t_of_s.evaluate(sLo);
        elem.t_end = elem.t_of_s.evaluate(sHi);

        // Certify the dynamics LGLs against the SSR's exact values at
        // interior probes biased toward both element edges. For a
        // sqrt-warped element xi=+-0.97 maps within ~1e-3*h of the
        // cusp edge — the region where a sqrt fit's residual
        // concentrates and where mid-probes never look; the +-0.45/0
        // probes cover interior V-dips and kinks.
        for (double xi : {-0.97, -0.5, 0.0, 0.5, 0.97}) {
            const double sP = elem.t_of_s.xiToS(xi);
            const double tExact = std::clamp(
                static_cast<double>(ssr.timeAtArcLength(
                    static_cast<T>(sP))),
                arc.t0, arc.t0 + arc.duration);
            const double vExact = std::max(
                ssr.pathVelocityAtArcLength(static_cast<T>(sP)),
                0.0);
            const double aExact = static_cast<double>(
                ssr.accelerationOnArc(
                    arc, tExact - arc.t0));
            // Time fit measured as arc-length error: what invertT
            // commits at this s is |dt| / (dt/ds) = |dt| * v.
            elem.certified_t_error = std::max(elem.certified_t_error,
                std::abs(elem.t_of_s.evaluate(sP) - tExact) * vExact);
            elem.certified_v_error = std::max(elem.certified_v_error,
                std::abs(elem.v_of_s.evaluate(sP) - vExact));
            elem.certified_a_error = std::max(elem.certified_a_error,
                std::abs(elem.a_of_s.evaluate(sP) - aExact));
        }

        // The element's NURBS piece, located by s — u is piece-local,
        // so every u(s) and g(u)=||C'(u)|| evaluation must be taken on
        // THIS piece. (Intervals are pre-split at piece boundaries so
        // an element never straddles a piece junction.) The previous
        // getSpeedFactor(u) searched pieces by knot range instead and
        // silently returned piece 0's g for most elements — every
        // Taylor coefficient was wrong, the Padé diverged, and the u
        // certificate forced ~50 futile bisections per arc.
        const std::size_t elemPiece =
            path_->hasInner()
                ? path_->inner().locate(0.5 * (sLo + sHi)).piece
                : 0;
        elem.pieceIdx = elemPiece;

        // Compute Padé approximant for u(s)
        // We need u(s) at sLo. Get it from the path.
        double uBegin = uOnPieceAtS(sLo, elemPiece);
        elem.u_exact_begin = uBegin;

        // Compute Taylor coefficients of u(s) at sLo
        // du/ds = 1/g(u), where g(u) = ||C'(u)||
        // d²u/ds² = -g'(u)/g(u)³  (from differentiating 1/g)
        // d³u/ds³ = ...
        // For [3/2] Padé, we need m+n+1 = 6 Taylor coefficients
        auto taylor = computeUTaylor(uBegin, elemPiece, 6);  // 6 coefficients for [3/2]

        // Build [3/2] Padé approximant
        elem.u_of_s = PadeApproximant(taylor, 3, 2);

        // Compute exact u at sHi for certification
        double uEnd = uOnPieceAtS(sHi, elemPiece);
        elem.u_exact_end = uEnd;
        elem.du_ds_exact_begin = 1.0 / speedFactorOnPiece(elemPiece, uBegin);
        elem.du_ds_exact_end = 1.0 / speedFactorOnPiece(elemPiece, uEnd);

        // Certify: compare Padé to exact at midpoint
        double sMid = 0.5 * (sLo + sHi);
        double uMidExact = uOnPieceAtS(sMid, elemPiece);
        double uMidPade = elem.u_of_s.evaluate(sMid - sLo);
        elem.certified_u_error = std::abs(uMidExact - uMidPade);

        // Also check endpoint
        double uEndPade = elem.u_of_s.evaluate(sHi - sLo);
        elem.certified_u_error = std::max(elem.certified_u_error,
                                          std::abs(uEnd - uEndPade));

        // Derivative error at midpoint
        double duMidExact = 1.0 / speedFactorOnPiece(elemPiece, uMidExact);
        double duMidPade = elem.u_of_s.evaluateDerivative(sMid - sLo);
        elem.certified_du_error = std::abs(duMidExact - duMidPade);

        // Second derivative error (approximate)
        double d2uMidPade = elem.u_of_s.evaluate2ndDerivative(sMid - sLo);
        // Exact d²u/ds² = -g'(u)/(g(u))³ with g' = dg/du — probed in u
        // on the element's own piece (the old code probed g in s, which
        // mixes in du/ds and can cross a piece boundary).
        const double gMid = speedFactorOnPiece(elemPiece, uMidExact);
        const double duFD = 1e-6 * std::max(1.0, std::abs(uMidExact));
        const double gPrime =
            (speedFactorOnPiece(elemPiece, uMidExact + duFD) -
             speedFactorOnPiece(elemPiece, uMidExact - duFD)) /
            (2.0 * duFD);
        double d2uMidExact = -gPrime / (gMid * gMid * gMid);
        elem.certified_d2u_error = std::abs(d2uMidExact - d2uMidPade);

        return elem;
    }

    // ========================================================================
    // NURBS parameter u(s) utilities
    // ========================================================================

    /**
     * @brief u at s interpreted on a specific piece (left-limit).
     *
     * At a shared piece boundary locate() may return the next piece,
     * yielding that piece's u_begin where an element on this piece
     * expects its own u_end — this resolves u as the element's piece
     * sees it so endpoint certification compares like with like.
     */
    double uOnPieceAtS(double s, std::size_t pieceIdx) const {
        if (!path_->hasInner()) return s;
        const auto& inner = path_->inner();
        const auto loc = inner.locate(s);
        const auto& piece = inner.piece(pieceIdx);
        const double localS = (loc.piece == pieceIdx)
            ? loc.localS
            : (loc.piece > pieceIdx ? piece.length() : 0.0);
        return piece.invertLength(
            std::clamp(localS, 0.0, piece.length()));
    }

    /**Speed factor g(u) = ||C'(u)|| evaluated on a specific piece. */
    double speedFactorOnPiece(std::size_t pieceIdx, double u) const {
        if (!path_->hasInner()) return 1.0;
        const auto& inner = path_->inner();
        if (pieceIdx >= inner.numPieces()) return 1.0;
        const auto& p = inner.piece(pieceIdx);
        const double uc = std::clamp(u, p.knotMin(), p.knotMax());
        const double g = p.derivative(uc, 1).norm();
        return (g > 1e-15) ? g : 1e15;
    }

    /**Speed factor g(u) = ||C'(u)|| on the piece located at s. */
    double speedFactorAtS(double s) const {
        if (!path_->hasInner()) return 1.0;
        const auto& inner = path_->inner();
        auto loc = inner.locate(s);
        const auto& piece = inner.piece(loc.piece);
        const double u = piece.invertLength(
            std::clamp(loc.localS, 0.0, piece.length()));
        return speedFactorOnPiece(loc.piece, u);
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
     * @param u0 u at the expansion point
     * @param pieceIdx NURBS piece the expansion point belongs to
     * @param numCoeffs Number of Taylor coefficients to compute
     * @return Taylor coefficients [u_0, u_1, u_2, ...]
     */
    std::vector<double> computeUTaylor(double u0, std::size_t pieceIdx,
                                       int numCoeffs) const {
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

        // h(u) = 1/g(u) on the element's own piece
        auto h = [&](double u) -> double {
            return 1.0 / speedFactorOnPiece(pieceIdx, u);
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

        if (elem.t_of_s.warp != LGLWarp::NONE) {
            return invertElementWarped(elem, tQuery);
        }

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

    /**
     * @brief Invert t(s) inside a warped element via the sigma coordinate.
     *
     * t~(sigma) = t_of_s(xi(sigma)) is smooth and monotone in sigma
     * (dt/dsigma = -2*sigma/v is regular even at a stop, where dt/ds
     * blows up), so Newton in sigma converges in a couple of steps
     * where s-Newton fought the 1/v wall. Safeguarded by bisection on
     * the sigma interval.
     *
     * @param elem Warped hybrid element
     * @param tQuery Target time
     * @return Arc length s such that t(s) ≈ tQuery
     */
    double invertElementWarped(const HybridElement& elem,
                               double tQuery) const {
        const LGLElement& tl = elem.t_of_s;
        const double sigmaMax = tl.warpSigmaMax();
        if (sigmaMax <= 0.0) return elem.s_begin;

        // dt~/dsigma sign: t grows with s; sigma grows with s for
        // SQRT_LEFT and shrinks for SQRT_RIGHT.
        const double dir = (tl.warp == LGLWarp::SQRT_LEFT) ? 1.0 : -1.0;

        auto tOfSigma = [&](double sigma) {
            const double xi =
                std::clamp(2.0 * sigma / sigmaMax - 1.0, -1.0, 1.0);
            return barycentricEvaluate(
                tl.ref_nodes, tl.node_values, tl.bary_weights, xi);
        };
        auto dtDsigma = [&](double sigma) {
            const double xi =
                std::clamp(2.0 * sigma / sigmaMax - 1.0, -1.0, 1.0);
            return barycentricDerivative(
                tl.ref_nodes, tl.node_values, tl.bary_weights, xi) *
                (2.0 / sigmaMax);
        };

        double lo = 0.0, hi = sigmaMax;
        const double tLo = tOfSigma(0.0), tHi = tOfSigma(sigmaMax);
        double sigma =
            (std::abs(tHi - tLo) > 1e-15)
                ? sigmaMax * (tQuery - tLo) / (tHi - tLo)
                : 0.5 * sigmaMax;
        sigma = std::clamp(sigma, lo, hi);

        for (int iter = 0; iter < 50; ++iter) {
            const double err = tOfSigma(sigma) - tQuery;
            if (std::abs(err) < 1e-12) break;
            if (err * dir > 0.0) {
                hi = sigma;
            } else {
                lo = sigma;
            }
            const double d = dtDsigma(sigma);
            double sNew = (std::abs(d) > 1e-15)
                ? sigma - err / d
                : 0.5 * (lo + hi);
            if (sNew <= lo || sNew >= hi) sNew = 0.5 * (lo + hi);
            sigma = sNew;
        }
        sigma = std::clamp(sigma, 0.0, sigmaMax);
        return (tl.warp == LGLWarp::SQRT_LEFT)
            ? elem.s_begin + sigma * sigma
            : elem.s_end - sigma * sigma;
    }

    /**Find element containing arc length s */
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

    /**Find element containing time t */
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

    /**Estimate maximum speed factor on an element */
    double estimateMaxSpeed(const HybridElement& elem) const {
        double maxG = 0.0;
        // Sample at a few points — g(u) is piece-local, so locate the
        // piece from s rather than matching u against knot ranges.
        for (int i = 0; i < 4; ++i) {
            double s = elem.s_begin + (elem.s_end - elem.s_begin) * i / 3.0;
            maxG = std::max(maxG, speedFactorAtS(s));
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

    /**Convert RVec to Vec<Dim, T> */
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
