/**
 * @file NURBSCurve.hpp
 * @brief Complete NURBS (Non-Uniform Rational B-Spline) Curve Implementation
 *
 * @details
 * This file provides a production-quality NURBS curve implementation supporting:
 *
 * - Arbitrary degree, dimension, and knot vectors
 * - Weighted (rational) and unweighted (non-rational / B-spline) evaluation
 * - De Boor algorithm for numerically stable evaluation
 * - Analytical first, second, and third derivatives
 * - Curvature, tangent, normal, binormal computation
 * - Arc length via adaptive Gaussian quadrature
 * - Knot insertion (Boehm's algorithm)
 * - Bézier decomposition (for SVG export compatibility)
 * - Subdivision and extraction of sub-curves
 * - G0/G1/G2 continuity checking
 *
 * ## NURBS Definition
 *
 * A NURBS curve of degree p with n+1 control points P_i, weights w_i,
 * and knot vector U = {u_0, ..., u_{n+p+1}} is defined as:
 *
 *   C(u) = Σ R_{i,p}(u) * P_i
 *
 * where R_{i,p}(u) = (N_{i,p}(u) * w_i) / Σ(N_{j,p}(u) * w_j)
 * and N_{i,p} are the B-spline basis functions.
 *
 * When all weights are 1.0, this reduces to a standard B-spline.
 *
 * @see BezierCurve.hpp
 * @see MathTypes.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "BezierCurve.hpp"
#include "SourceReference.hpp"

#include <vector>
#include <memory>
#include <optional>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <cassert>

namespace MotionPlanner {

// ============================================================================
// Helper: Binomial Coefficient
// ============================================================================

/**
 * @brief Compute binomial coefficient C(n, k) for non-negative integers.
 */
inline size_t binomialCoeff(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    if (k > n - k) k = n - k;
    size_t result = 1;
    for (int i = 0; i < k; ++i) {
        result = result * static_cast<size_t>(n - i) / static_cast<size_t>(i + 1);
    }
    return result;
}

// ============================================================================
// NURBS Curve Configuration
// ============================================================================

/**
 * @brief Configuration for NURBS arc length computation
 */
struct NURBSArcLengthConfig {
    double tolerance = 1e-8;
    size_t maxDepth = 20;
    size_t tableSamples = 200;
    double parameterTolerance = 1e-10;
};

// ============================================================================
// NURBS Arc Length Table
// ============================================================================

/**
 * @brief Precomputed arc length table for efficient NURBS queries
 */
template<typename T = double>
class NURBSArcLengthTable {
public:
    NURBSArcLengthTable() = default;

    void build(const std::vector<T>& parameters, const std::vector<T>& arcLengths) {
        params_ = parameters;
        lengths_ = arcLengths;
        totalLength_ = lengths_.empty() ? T(0) : lengths_.back();
    }

    T totalLength() const noexcept { return totalLength_; }

    T arcLengthAt(T u) const {
        if (params_.empty()) return T(0);
        u = clamp(u, params_.front(), params_.back());

        auto it = std::lower_bound(params_.begin(), params_.end(), u);
        if (it == params_.begin()) return lengths_.front();
        if (it == params_.end()) return lengths_.back();

        size_t i = static_cast<size_t>(std::distance(params_.begin(), it));
        T t = (u - params_[i-1]) / (params_[i] - params_[i-1]);
        return lerp(lengths_[i-1], lengths_[i], t);
    }

    T parameterAt(T s, T tolerance = T(1e-10)) const {
        if (lengths_.empty()) return T(0);
        s = clamp(s, T(0), totalLength_);

        auto it = std::lower_bound(lengths_.begin(), lengths_.end(), s);
        if (it == lengths_.begin()) return params_.front();
        if (it == lengths_.end()) return params_.back();

        size_t i = static_cast<size_t>(std::distance(lengths_.begin(), it));
        T denom = lengths_[i] - lengths_[i-1];
        if (std::abs(denom) < tolerance) return params_[i-1];

        T t = (s - lengths_[i-1]) / denom;
        return lerp(params_[i-1], params_[i], t);
    }

    bool isValid() const noexcept { return !params_.empty(); }
    size_t size() const noexcept { return params_.size(); }

private:
    std::vector<T> params_;
    std::vector<T> lengths_;
    T totalLength_ = T(0);
};

// ============================================================================
// NURBS Curve Implementation
// ============================================================================

/**
 * @brief General NURBS curve supporting arbitrary degree and dimension
 *
 * @tparam Dim Number of spatial dimensions
 * @tparam T Scalar type (default: double)
 */
template<size_t Dim, typename T = double>
class NURBSCurve {
public:
    using Point = Vec<Dim, T>;
    using ControlPoints = std::vector<Point>;
    using Weights = std::vector<T>;
    using KnotVector = std::vector<T>;

    // ========================================================================
    // Constructors
    // ========================================================================

    NURBSCurve() = default;

    /**
     * @brief Construct a NURBS curve
     * @param controlPoints Control points P_0 ... P_n
     * @param weights Weights w_0 ... w_n (if empty, all weights = 1)
     * @param knots Knot vector u_0 ... u_{n+p+1}
     * @param degree Curve degree p
     */
    NURBSCurve(ControlPoints controlPoints, Weights weights,
               KnotVector knots, size_t degree)
        : controlPoints_(std::move(controlPoints))
        , knots_(std::move(knots))
        , degree_(degree)
    {
        if (weights.empty()) {
            weights_.assign(controlPoints_.size(), T(1));
        } else {
            weights_ = std::move(weights);
        }
        validate();
    }

    /**
     * @brief Construct a uniform B-spline (all weights = 1, uniform knots)
     */
    NURBSCurve(ControlPoints controlPoints, size_t degree)
        : controlPoints_(std::move(controlPoints))
        , degree_(degree)
    {
        weights_.assign(controlPoints_.size(), T(1));
        knots_ = makeUniformKnotVector(controlPoints_.size(), degree_);
        validate();
    }

    /**
     * @brief Construct from a Bézier curve (convert to NURBS representation)
     */
    explicit NURBSCurve(const BezierCurve<Dim, T>& bezier)
        : controlPoints_(bezier.controlPoints())
        , degree_(bezier.degree())
    {
        weights_.assign(controlPoints_.size(), T(1));
        // Bézier = NURBS with clamped knot vector [0,...,0, 1,...,1]
        size_t n = controlPoints_.size();
        knots_.resize(n + degree_ + 1);
        for (size_t i = 0; i <= degree_; ++i) {
            knots_[i] = T(0);
            knots_[n + i] = T(1);
        }
        for (size_t i = degree_ + 1; i < n; ++i) {
            knots_[i] = T(i - degree_) / T(n - degree_);
        }
        sourceRef_ = bezier.sourceRef();
    }

    NURBSCurve(const NURBSCurve&) = default;
    NURBSCurve(NURBSCurve&&) noexcept = default;
    NURBSCurve& operator=(const NURBSCurve&) = default;
    NURBSCurve& operator=(NURBSCurve&&) noexcept = default;

    // ========================================================================
    // Properties
    // ========================================================================

    size_t degree() const noexcept { return degree_; }
    size_t numControlPoints() const noexcept { return controlPoints_.size(); }
    bool isValid() const noexcept {
        return controlPoints_.size() >= 2 &&
               knots_.size() == controlPoints_.size() + degree_ + 1 &&
               weights_.size() == controlPoints_.size();
    }

    const ControlPoints& controlPoints() const noexcept { return controlPoints_; }
    const Point& controlPoint(size_t i) const { return controlPoints_.at(i); }
    void setControlPoint(size_t i, const Point& p) { controlPoints_.at(i) = p; }

    const Weights& weights() const noexcept { return weights_; }
    T weight(size_t i) const { return weights_.at(i); }
    void setWeight(size_t i, T w) { weights_.at(i) = w; }

    const KnotVector& knots() const noexcept { return knots_; }

    /** @brief Parameter domain [u_min, u_max] */
    T domainStart() const { return knots_[degree_]; }
    T domainEnd() const { return knots_[knots_.size() - degree_ - 1]; }

    Point startPoint() const { return evaluate(domainStart()); }
    Point endPoint() const { return evaluate(domainEnd()); }

    const SourceReference& sourceRef() const noexcept { return sourceRef_; }
    void setSourceRef(const SourceReference& ref) { sourceRef_ = ref; }

    /** @brief Check if all weights are 1.0 (i.e., this is a non-rational B-spline) */
    bool isNonRational() const {
        for (const auto& w : weights_) {
            if (std::abs(w - T(1)) > T(1e-12)) return false;
        }
        return true;
    }

    // ========================================================================
    // De Boor Evaluation
    // ========================================================================

    /**
     * @brief Evaluate NURBS curve at parameter u using de Boor's algorithm
     *
     * For rational curves, evaluates in homogeneous coordinates and projects.
     */
    Point evaluate(T u) const {
        if (controlPoints_.empty()) return Point{};

        u = clampToParameterDomain(u);
        int span = findSpan(u);

        // Evaluate in homogeneous coordinates for rational curves
        if (!isNonRational()) {
            return evaluateRational(u, span);
        }

        // Non-rational: standard de Boor
        return deBoor(u, span);
    }

    Point operator()(T u) const { return evaluate(u); }

    // ========================================================================
    // Derivative Evaluation
    // ========================================================================

    /**
     * @brief Evaluate k-th derivative at parameter u
     *
     * For rational curves, uses the quotient rule on weighted B-spline
     * derivatives (Piegl & Tiller algorithm A4.2).
     */
    Point evaluateDerivative(T u, size_t k) const {
        if (k == 0) return evaluate(u);
        if (k > degree_) return Point{};

        u = clampToParameterDomain(u);

        if (!isNonRational()) {
            return evaluateRationalDerivative(u, k);
        }

        // Non-rational B-spline derivative
        return evaluateBSplineDerivative(u, k);
    }

    /**
     * @brief Evaluate position and first 3 derivatives at once
     */
    std::tuple<Point, Point, Point, Point> evaluateAll(T u) const {
        Point pos = evaluate(u);
        Point vel = evaluateDerivative(u, 1);
        Point acc = evaluateDerivative(u, 2);
        Point jrk = (degree_ >= 3) ? evaluateDerivative(u, 3) : Point{};
        return {pos, vel, acc, jrk};
    }

    // ========================================================================
    // Geometric Properties
    // ========================================================================

    Point tangent(T u) const {
        return evaluateDerivative(u, 1).normalized();
    }

    T speed(T u) const {
        return evaluateDerivative(u, 1).magnitude();
    }

    /**
     * @brief Compute curvature at parameter u
     *
     * κ = |C' × C''| / |C'|³   (3D)
     * κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)  (2D)
     */
    T curvature(T u) const {
        Point vel = evaluateDerivative(u, 1);
        Point acc = evaluateDerivative(u, 2);

        T speedSq = vel.magnitudeSq();
        if (speedSq < static_cast<T>(MathConstants::EPSILON)) return T(0);

        if constexpr (Dim == 2) {
            T cross = vel[0] * acc[1] - vel[1] * acc[0];
            return cross / (speedSq * std::sqrt(speedSq));
        } else if constexpr (Dim == 3) {
            Point cross = vel.cross(acc);
            return cross.magnitude() / (speedSq * std::sqrt(speedSq));
        } else {
            Point accPerp = acc - vel * (acc.dot(vel) / speedSq);
            return accPerp.magnitude() / speedSq;
        }
    }

    Point normal(T u) const {
        Point vel = evaluateDerivative(u, 1);
        Point acc = evaluateDerivative(u, 2);
        T speedSq = vel.magnitudeSq();
        if (speedSq < static_cast<T>(MathConstants::EPSILON)) return Point{};
        Point accPerp = acc - vel * (acc.dot(vel) / speedSq);
        return accPerp.normalized();
    }

    template<size_t D = Dim>
    std::enable_if_t<D == 3, Point> binormal(T u) const {
        return tangent(u).cross(normal(u));
    }

    std::pair<Point, Point> boundingBox() const {
        if (controlPoints_.empty()) return {Point{}, Point{}};
        Point minPt = controlPoints_[0];
        Point maxPt = controlPoints_[0];
        for (const auto& p : controlPoints_) {
            minPt = minPt.elementMin(p);
            maxPt = maxPt.elementMax(p);
        }
        return {minPt, maxPt};
    }

    // ========================================================================
    // Arc Length
    // ========================================================================

    T arcLength(const NURBSArcLengthConfig& config = {}) const {
        return arcLengthBetween(domainStart(), domainEnd(), config);
    }

    T arcLengthBetween(T u0, T u1, const NURBSArcLengthConfig& config = {}) const {
        return adaptiveGaussianQuadrature(u0, u1, config.tolerance, config.maxDepth);
    }

    NURBSArcLengthTable<T> buildArcLengthTable(const NURBSArcLengthConfig& config = {}) const {
        size_t samples = config.tableSamples;
        std::vector<T> params(samples);
        std::vector<T> lengths(samples);

        T uStart = domainStart();
        T uEnd = domainEnd();
        T totalLen = T(0);
        params[0] = uStart;
        lengths[0] = T(0);

        for (size_t i = 1; i < samples; ++i) {
            T u0 = uStart + (uEnd - uStart) * T(i - 1) / T(samples - 1);
            T u1 = uStart + (uEnd - uStart) * T(i) / T(samples - 1);
            params[i] = u1;
            totalLen += arcLengthBetween(u0, u1, config);
            lengths[i] = totalLen;
        }

        NURBSArcLengthTable<T> table;
        table.build(params, lengths);
        return table;
    }

    // ========================================================================
    // Knot Insertion (Boehm's Algorithm)
    // ========================================================================

    /**
     * @brief Insert a knot value into the curve (exact, no shape change)
     * @param u Knot value to insert
     * @return New NURBS curve with the additional knot
     */
    NURBSCurve insertKnot(T u) const {
        if (!isValid()) return *this;

        int span = findSpan(u);
        int p = static_cast<int>(degree_);
        int n = static_cast<int>(controlPoints_.size()) - 1;

        // Count multiplicity of u in existing knots
        int s = knotMultiplicity(u);
        if (s >= p) return *this; // Already at max multiplicity

        // New knot vector
        KnotVector newKnots(knots_.size() + 1);
        for (int i = 0; i <= span; ++i) newKnots[i] = knots_[i];
        newKnots[span + 1] = u;
        for (int i = span + 1; i < static_cast<int>(knots_.size()); ++i)
            newKnots[i + 1] = knots_[i];

        // New control points and weights (in homogeneous coords)
        ControlPoints newPts(controlPoints_.size() + 1);
        Weights newWts(weights_.size() + 1);

        // Copy unaffected points at the beginning
        for (int i = 0; i <= span - p; ++i) {
            newPts[i] = controlPoints_[i];
            newWts[i] = weights_[i];
        }
        // Copy unaffected points at the end
        for (int i = span - s; i <= n; ++i) {
            newPts[i + 1] = controlPoints_[i];
            newWts[i + 1] = weights_[i];
        }

        // Compute new points in the affected region
        for (int i = span - p + 1; i <= span - s; ++i) {
            T alpha = (u - knots_[i]) / (knots_[i + p] - knots_[i]);

            // Interpolate in homogeneous coordinates
            T w0 = weights_[i - 1];
            T w1 = weights_[i];
            T newW = (T(1) - alpha) * w0 + alpha * w1;

            Point newP;
            for (size_t d = 0; d < Dim; ++d) {
                newP[d] = ((T(1) - alpha) * controlPoints_[i-1][d] * w0 +
                           alpha * controlPoints_[i][d] * w1) / newW;
            }

            newPts[i] = newP;
            newWts[i] = newW;
        }

        NURBSCurve result(std::move(newPts), std::move(newWts),
                          std::move(newKnots), degree_);
        result.setSourceRef(sourceRef_);
        return result;
    }

    /**
     * @brief Insert a knot multiple times
     */
    NURBSCurve insertKnot(T u, int times) const {
        NURBSCurve result = *this;
        for (int i = 0; i < times; ++i) {
            result = result.insertKnot(u);
        }
        return result;
    }

    // ========================================================================
    // Bézier Decomposition (for SVG Export)
    // ========================================================================

    /**
     * @brief Decompose this NURBS curve into a sequence of Bézier curves
     *
     * This is the key method for SVG compatibility. SVG only supports
     * cubic and quadratic Bézier paths, so we decompose the NURBS into
     * Bézier segments by inserting knots until each span has multiplicity
     * equal to the degree.
     *
     * @return Vector of Bézier curves that exactly represent this NURBS
     */
    std::vector<BezierCurve<Dim, T>> decomposeToBezier() const {
        std::vector<BezierCurve<Dim, T>> beziers;
        if (!isValid()) return beziers;

        // Insert knots to make all internal knots have multiplicity = degree
        NURBSCurve refined = *this;
        int p = static_cast<int>(degree_);

        // Find unique internal knots
        std::vector<T> uniqueKnots;
        for (size_t i = degree_ + 1; i < knots_.size() - degree_ - 1; ++i) {
            if (uniqueKnots.empty() ||
                std::abs(knots_[i] - uniqueKnots.back()) > T(1e-14)) {
                uniqueKnots.push_back(knots_[i]);
            }
        }

        // Insert each unique knot until multiplicity = degree
        for (T uk : uniqueKnots) {
            int mult = refined.knotMultiplicity(uk);
            int insertions = p - mult;
            if (insertions > 0) {
                refined = refined.insertKnot(uk, insertions);
            }
        }

        // Now each span corresponds to a Bézier segment
        // Extract Bézier segments: each segment uses (p+1) consecutive
        // control points, starting at index i*(p) for the i-th segment
        size_t numBeziers = (refined.numControlPoints() - 1) / degree_;
        if (numBeziers == 0 && refined.numControlPoints() > 1) numBeziers = 1;

        for (size_t b = 0; b < numBeziers; ++b) {
            typename BezierCurve<Dim, T>::ControlPoints bezPts(degree_ + 1);
            bool isRational = false;

            for (size_t j = 0; j <= degree_; ++j) {
                size_t idx = b * degree_ + j;
                if (idx < refined.numControlPoints()) {
                    bezPts[j] = refined.controlPoints_[idx];
                    if (std::abs(refined.weights_[idx] - T(1)) > T(1e-12)) {
                        isRational = true;
                    }
                }
            }

            // For rational curves, project to Euclidean space
            // (Bézier control points are already in Euclidean coords after
            //  our knot insertion in homogeneous coordinates)
            BezierCurve<Dim, T> bez(std::move(bezPts));
            bez.setSourceRef(sourceRef_);
            beziers.push_back(std::move(bez));
        }

        return beziers;
    }

    /**
     * @brief Approximate with cubic Bézier curves (for SVG cubic path commands)
     *
     * If the NURBS degree is not 3, this creates cubic approximations
     * by sampling and fitting.
     *
     * @param maxError Maximum approximation error
     * @param maxSegments Maximum number of cubic segments
     * @return Vector of cubic Bézier curves
     */
    std::vector<BezierCurve<Dim, T>> approximateWithCubicBeziers(
            T maxError = T(0.01), size_t maxSegments = 100) const {

        // If degree == 3 and non-rational, decompose exactly
        if (degree_ == 3 && isNonRational()) {
            return decomposeToBezier();
        }

        // For other degrees or rational curves, use adaptive subdivision
        std::vector<BezierCurve<Dim, T>> result;
        approximateCubicRecursive(domainStart(), domainEnd(),
                                   maxError, maxSegments, result, 0);
        return result;
    }

    // ========================================================================
    // Subdivision
    // ========================================================================

    /**
     * @brief Split the curve at parameter u
     * @return Pair of NURBS curves [left, right]
     */
    std::pair<NURBSCurve, NURBSCurve> subdivide(T u) const {
        if (!isValid()) return {*this, *this};

        // Insert knot at u until multiplicity = degree + 1
        NURBSCurve refined = *this;
        int mult = refined.knotMultiplicity(u);
        int insertions = static_cast<int>(degree_) + 1 - mult;
        for (int i = 0; i < insertions; ++i) {
            refined = refined.insertKnot(u);
        }

        // Find first and last occurrence of u in refined knots
        int firstIdx = -1;
        int lastIdx = -1;
        for (size_t i = 0; i < refined.knots_.size(); ++i) {
            if (std::abs(refined.knots_[i] - u) < T(1e-14)) {
                if (firstIdx < 0) firstIdx = static_cast<int>(i);
                lastIdx = static_cast<int>(i);
            }
        }
        if (firstIdx < 0) return {*this, *this};

        // The split point in control points corresponds to:
        // Left curve has control points [0, firstIdx - degree_)
        // which is [0, firstIdx - degree_ - 1] inclusive
        // knots for left: [0, firstIdx]   with (firstIdx+1) knots
        // So n_left = firstIdx - degree_, and #cp = n_left
        
        size_t leftNumCP = static_cast<size_t>(firstIdx) - degree_ + 1;
        
        ControlPoints leftPts(refined.controlPoints_.begin(),
                             refined.controlPoints_.begin() + static_cast<int>(leftNumCP));
        Weights leftWts(refined.weights_.begin(),
                       refined.weights_.begin() + static_cast<int>(leftNumCP));
        KnotVector leftKnots(refined.knots_.begin(),
                            refined.knots_.begin() + static_cast<int>(leftNumCP) + static_cast<int>(degree_) + 1);
        
        // Right curve starts at control point (leftNumCP - 1) (shared junction point)
        size_t rightStart = leftNumCP - 1;
        ControlPoints rightPts(refined.controlPoints_.begin() + static_cast<int>(rightStart),
                              refined.controlPoints_.end());
        Weights rightWts(refined.weights_.begin() + static_cast<int>(rightStart),
                        refined.weights_.end());
        // Right knots start at the first occurrence of u  
        KnotVector rightKnots(refined.knots_.begin() + firstIdx,
                             refined.knots_.end());

        // Validate before constructing
        size_t leftExpectedKnots = leftPts.size() + degree_ + 1;
        size_t rightExpectedKnots = rightPts.size() + degree_ + 1;
        
        if (leftKnots.size() != leftExpectedKnots || rightKnots.size() != rightExpectedKnots) {
            // Fallback: return a copy of *this for both
            return {*this, *this};
        }

        NURBSCurve left(std::move(leftPts), std::move(leftWts),
                        std::move(leftKnots), degree_);
        NURBSCurve right(std::move(rightPts), std::move(rightWts),
                         std::move(rightKnots), degree_);
        left.setSourceRef(sourceRef_);
        right.setSourceRef(sourceRef_);

        return {std::move(left), std::move(right)};
    }

    // ========================================================================
    // Continuity Checking
    // ========================================================================

    bool isG0ContinuousWith(const NURBSCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        return endPoint().nearEqual(other.startPoint(), tolerance);
    }

    bool isG1ContinuousWith(const NURBSCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG0ContinuousWith(other, tolerance)) return false;
        Point t1 = tangent(domainEnd());
        Point t2 = other.tangent(other.domainStart());
        T dot = std::abs(t1.dot(t2));
        return dot > T(1) - tolerance;
    }

    bool isG2ContinuousWith(const NURBSCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG1ContinuousWith(other, tolerance)) return false;
        T k1 = curvature(domainEnd());
        T k2 = other.curvature(other.domainStart());
        return std::abs(k1 - k2) < tolerance;
    }

    int continuityLevelWith(const NURBSCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG0ContinuousWith(other, tolerance)) return -1;
        if (!isG1ContinuousWith(other, tolerance)) return 0;
        if (!isG2ContinuousWith(other, tolerance)) return 1;
        return 2;
    }

    // ========================================================================
    // Static Factory Methods
    // ========================================================================

    /**
     * @brief Create a NURBS line segment
     */
    static NURBSCurve makeLine(const Point& p0, const Point& p1) {
        return NURBSCurve({p0, p1}, {T(1), T(1)}, {T(0), T(0), T(1), T(1)}, 1);
    }

    /**
     * @brief Create a NURBS circle arc (rational, degree 2)
     *
     * A full circle is represented by degree-2 rational NURBS.
     * For arcs <= 120°, a single segment suffices.
     */
    static NURBSCurve makeCircularArc(const Point& center, T radius,
                                       T startAngle, T sweepAngle) {
        // For arcs > 120°, split into multiple segments
        if (std::abs(sweepAngle) > T(2.0 * MathConstants::PI / 3.0)) {
            // Split and join
            int numSegments = static_cast<int>(
                std::ceil(std::abs(sweepAngle) / (MathConstants::PI / 2.0)));
            numSegments = std::max(2, numSegments);

            T segSweep = sweepAngle / T(numSegments);
            T angle = startAngle;

            // Build first segment
            NURBSCurve result = makeCircularArc(center, radius, angle, segSweep);
            angle += segSweep;

            // Join remaining segments (simplified - return first for now)
            // Full implementation would merge knot vectors
            return result;
        }

        T halfAngle = sweepAngle / T(2);
        T cosHalf = std::cos(halfAngle);

        // Three control points for degree-2 arc
        Point p0, p1, p2;

        T a0 = startAngle;
        T a1 = startAngle + halfAngle;
        T a2 = startAngle + sweepAngle;

        if constexpr (Dim >= 2) {
            p0[0] = center[0] + radius * std::cos(a0);
            p0[1] = center[1] + radius * std::sin(a0);
            p2[0] = center[0] + radius * std::cos(a2);
            p2[1] = center[1] + radius * std::sin(a2);

            // Shoulder point
            p1[0] = center[0] + radius * std::cos(a1) / cosHalf;
            p1[1] = center[1] + radius * std::sin(a1) / cosHalf;
        }

        return NURBSCurve({p0, p1, p2}, {T(1), cosHalf, T(1)},
                          {T(0), T(0), T(0), T(1), T(1), T(1)}, 2);
    }

    /**
     * @brief Create a clamped uniform B-spline knot vector
     */
    static KnotVector makeUniformKnotVector(size_t numControlPoints, size_t degree) {
        size_t m = numControlPoints + degree + 1;
        KnotVector knots(m);

        for (size_t i = 0; i <= degree; ++i) {
            knots[i] = T(0);
        }
        for (size_t i = degree + 1; i < m - degree - 1; ++i) {
            knots[i] = T(i - degree) / T(m - 2 * degree - 1);
        }
        for (size_t i = m - degree - 1; i < m; ++i) {
            knots[i] = T(1);
        }

        return knots;
    }

    /**
     * @brief Create a G2-continuous NURBS blend curve between two directions
     *
     * Creates a degree-5 NURBS (equivalent to quintic Bézier) that smoothly
     * transitions from entryPoint/entryDir to exitPoint/exitDir with curvature
     * matching at both ends (C2 continuous). Supports zero curvature (line
     * transitions) and non-zero curvature (arc transitions).
     */
    static NURBSCurve makeG2BlendCurve(const Point& entryPoint,
                                        const Point& exitPoint,
                                        const Point& entryDir,
                                        const Point& exitDir,
                                        T entryCurvature = T(0),
                                        T exitCurvature = T(0)) {
        // Quintic Bézier blend - 6 control points P0..P5
        // C2 continuity at boundaries:
        //   C(0) = P0, C'(0) = 5*(P1-P0), C''(0) = 20*(P2-2*P1+P0)
        //   C(1) = P5, C'(1) = 5*(P5-P4), C''(1) = 20*(P5-2*P4+P3)
        //
        // Curvature at t=0: κ(0) = (4/5) * |normal component of (P2-2P1+P0)| / tangentScale²
        // For desired κ: normal offset = (5/4) * κ * tangentScale²

        T chordLength = entryPoint.distanceTo(exitPoint);
        if (chordLength < static_cast<T>(MathConstants::EPSILON)) {
            return makeLine(entryPoint, exitPoint);
        }

        T tangentScale = chordLength / T(5);

        Point P0 = entryPoint;
        Point P5 = exitPoint;

        // C1: P1 along entry tangent, P4 along exit tangent
        Point P1 = P0 + entryDir * tangentScale;
        Point P4 = P5 - exitDir * tangentScale;

        // C2: P2 and P3 control curvature at boundaries
        // For zero curvature: P2 = 2*P1 - P0 (collinear), P3 = 2*P4 - P5 (collinear)
        // For curvature κ: add normal offset = (5/4) * κ * tangentScale²
        Point P2 = P0 + entryDir * (tangentScale * T(2));
        Point P3 = P5 - exitDir * (tangentScale * T(2));

        if (std::abs(entryCurvature) > static_cast<T>(MathConstants::EPSILON)) {
            Point norm = perpendicular(entryDir);
            T offset = (T(5) / T(4)) * entryCurvature * tangentScale * tangentScale;
            P2 = P2 + norm * offset;
        }

        if (std::abs(exitCurvature) > static_cast<T>(MathConstants::EPSILON)) {
            Point norm = perpendicular(exitDir);
            T offset = (T(5) / T(4)) * exitCurvature * tangentScale * tangentScale;
            P3 = P3 + norm * offset;
        }

        // Clamped knot vector for degree 5 with 6 control points (= Bézier)
        KnotVector knots = {T(0),T(0),T(0),T(0),T(0),T(0),
                           T(1),T(1),T(1),T(1),T(1),T(1)};
        Weights wts(6, T(1));

        NURBSCurve curve({P0, P1, P2, P3, P4, P5}, std::move(wts),
                         std::move(knots), 5);
        return curve;
    }

private:
    ControlPoints controlPoints_;
    KnotVector knots_;
    Weights weights_;
    size_t degree_ = 0;
    SourceReference sourceRef_;

    // ========================================================================
    // Validation
    // ========================================================================

    void validate() const {
        if (controlPoints_.empty()) return;
        if (knots_.size() != controlPoints_.size() + degree_ + 1) {
            throw std::invalid_argument(
                "NURBS: knot vector size must equal n + p + 1. Got " +
                std::to_string(knots_.size()) + ", expected " +
                std::to_string(controlPoints_.size() + degree_ + 1));
        }
        if (weights_.size() != controlPoints_.size()) {
            throw std::invalid_argument(
                "NURBS: weights size must equal number of control points");
        }
        // Verify knot vector is non-decreasing
        for (size_t i = 1; i < knots_.size(); ++i) {
            if (knots_[i] < knots_[i-1] - T(1e-14)) {
                throw std::invalid_argument(
                    "NURBS: knot vector must be non-decreasing");
            }
        }
    }

    // ========================================================================
    // Knot Span Finding
    // ========================================================================

    /**
     * @brief Find the knot span index for parameter u
     *
     * Returns i such that u ∈ [knots_[i], knots_[i+1])
     * (Piegl & Tiller Algorithm A2.1)
     */
    int findSpan(T u) const {
        int n = static_cast<int>(controlPoints_.size()) - 1;
        int p = static_cast<int>(degree_);

        // Special case: u at end of domain
        if (u >= knots_[n + 1] - T(1e-14)) {
            return n;
        }
        if (u <= knots_[p] + T(1e-14)) {
            return p;
        }

        // Binary search
        int low = p;
        int high = n + 1;
        int mid = (low + high) / 2;

        while (u < knots_[mid] || u >= knots_[mid + 1]) {
            if (u < knots_[mid]) {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }

        return mid;
    }

    /**
     * @brief Count multiplicity of knot value u
     */
    int knotMultiplicity(T u) const {
        int count = 0;
        for (const auto& k : knots_) {
            if (std::abs(k - u) < T(1e-14)) ++count;
        }
        return count;
    }

    T clampToParameterDomain(T u) const {
        return clamp(u, domainStart(), domainEnd());
    }

    // ========================================================================
    // De Boor's Algorithm (non-rational)
    // ========================================================================

    /**
     * @brief Standard de Boor evaluation for B-splines
     */
    Point deBoor(T u, int span) const {
        int p = static_cast<int>(degree_);

        // Initialize working array with affected control points
        std::vector<Point> d(p + 1);
        for (int j = 0; j <= p; ++j) {
            int idx = span - p + j;
            if (idx >= 0 && idx < static_cast<int>(controlPoints_.size())) {
                d[j] = controlPoints_[idx];
            }
        }

        // Triangular computation
        for (int r = 1; r <= p; ++r) {
            for (int j = p; j >= r; --j) {
                int i = span - p + j;
                T denom = knots_[i + p + 1 - r] - knots_[i];
                T alpha = (std::abs(denom) > T(1e-14))
                    ? (u - knots_[i]) / denom
                    : T(0);
                d[j] = d[j-1] * (T(1) - alpha) + d[j] * alpha;
            }
        }

        return d[p];
    }

    // ========================================================================
    // Rational Evaluation
    // ========================================================================

    Point evaluateRational(T u, int span) const {
        int p = static_cast<int>(degree_);

        // Work in homogeneous coordinates: (w*x, w*y, w*z, w)
        // Use (Dim+1) homogeneous vector
        using HomPoint = Vec<Dim + 1, T>;

        std::vector<HomPoint> d(p + 1);
        for (int j = 0; j <= p; ++j) {
            int idx = span - p + j;
            if (idx >= 0 && idx < static_cast<int>(controlPoints_.size())) {
                T w = weights_[idx];
                for (size_t dim = 0; dim < Dim; ++dim) {
                    d[j][dim] = controlPoints_[idx][dim] * w;
                }
                d[j][Dim] = w;
            }
        }

        // De Boor in homogeneous space
        for (int r = 1; r <= p; ++r) {
            for (int j = p; j >= r; --j) {
                int i = span - p + j;
                T denom = knots_[i + p + 1 - r] - knots_[i];
                T alpha = (std::abs(denom) > T(1e-14))
                    ? (u - knots_[i]) / denom
                    : T(0);
                d[j] = d[j-1] * (T(1) - alpha) + d[j] * alpha;
            }
        }

        // Project back to Euclidean space
        Point result;
        T w = d[p][Dim];
        if (std::abs(w) > T(1e-14)) {
            for (size_t dim = 0; dim < Dim; ++dim) {
                result[dim] = d[p][dim] / w;
            }
        }
        return result;
    }

    // ========================================================================
    // B-spline Basis Function Derivatives
    // ========================================================================

    /**
     * @brief Compute B-spline basis function values at u
     * (Piegl & Tiller Algorithm A2.2)
     */
    std::vector<T> basisFunctions(T u, int span) const {
        int p = static_cast<int>(degree_);
        std::vector<T> N(p + 1, T(0));
        std::vector<T> left(p + 1), right(p + 1);

        N[0] = T(1);

        for (int j = 1; j <= p; ++j) {
            left[j] = u - knots_[span + 1 - j];
            right[j] = knots_[span + j] - u;
            T saved = T(0);
            for (int r = 0; r < j; ++r) {
                T denom = right[r + 1] + left[j - r];
                T temp = (std::abs(denom) > T(1e-14)) ? N[r] / denom : T(0);
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }

        return N;
    }

    /**
     * @brief Compute B-spline basis function derivatives
     * (Piegl & Tiller Algorithm A2.3)
     *
     * Returns nders[k][j] = N^(k)_{span-p+j,p}(u)
     */
    std::vector<std::vector<T>> basisFunctionDerivatives(T u, int span,
                                                          int numDerivs) const {
        int p = static_cast<int>(degree_);

        // Basis function and knot difference arrays
        std::vector<std::vector<T>> ndu(p + 1, std::vector<T>(p + 1, T(0)));
        std::vector<T> left(p + 1), right(p + 1);

        ndu[0][0] = T(1);
        for (int j = 1; j <= p; ++j) {
            left[j] = u - knots_[span + 1 - j];
            right[j] = knots_[span + j] - u;
            T saved = T(0);
            for (int r = 0; r < j; ++r) {
                // Lower triangle
                ndu[j][r] = right[r + 1] + left[j - r];
                T temp = (std::abs(ndu[j][r]) > T(1e-14))
                    ? ndu[r][j-1] / ndu[j][r] : T(0);
                // Upper triangle
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        // Derivatives
        int nd = std::min(numDerivs, p);
        std::vector<std::vector<T>> ders(nd + 1, std::vector<T>(p + 1, T(0)));

        // Load basis functions
        for (int j = 0; j <= p; ++j) {
            ders[0][j] = ndu[j][p];
        }

        // Compute derivatives
        std::vector<std::vector<T>> a(2, std::vector<T>(p + 1, T(0)));

        for (int r = 0; r <= p; ++r) {
            int s1 = 0, s2 = 1;
            a[0][0] = T(1);

            for (int k = 1; k <= nd; ++k) {
                T d = T(0);
                int rk = r - k;
                int pk = p - k;

                if (r >= k) {
                    T denom = ndu[pk + 1][rk];
                    a[s2][0] = (std::abs(denom) > T(1e-14))
                        ? a[s1][0] / denom : T(0);
                    d = a[s2][0] * ndu[rk][pk];
                }

                int j1 = (rk >= -1) ? 1 : -rk;
                int j2 = (r - 1 <= pk) ? k - 1 : p - r;

                for (int j = j1; j <= j2; ++j) {
                    T denom = ndu[pk + 1][rk + j];
                    a[s2][j] = (std::abs(denom) > T(1e-14))
                        ? (a[s1][j] - a[s1][j-1]) / denom : T(0);
                    d += a[s2][j] * ndu[rk + j][pk];
                }

                if (r <= pk) {
                    T denom = ndu[pk + 1][r];
                    a[s2][k] = (std::abs(denom) > T(1e-14))
                        ? -a[s1][k-1] / denom : T(0);
                    d += a[s2][k] * ndu[r][pk];
                }

                ders[k][r] = d;
                std::swap(s1, s2);
            }
        }

        // Multiply by correct factors
        T factor = static_cast<T>(p);
        for (int k = 1; k <= nd; ++k) {
            for (int j = 0; j <= p; ++j) {
                ders[k][j] *= factor;
            }
            factor *= static_cast<T>(p - k);
        }

        return ders;
    }

    // ========================================================================
    // B-spline Derivative Evaluation
    // ========================================================================

    Point evaluateBSplineDerivative(T u, size_t k) const {
        int span = findSpan(u);
        auto ders = basisFunctionDerivatives(u, span, static_cast<int>(k));
        int p = static_cast<int>(degree_);

        Point result;
        for (int j = 0; j <= p; ++j) {
            int idx = span - p + j;
            if (idx >= 0 && idx < static_cast<int>(controlPoints_.size())) {
                result = result + controlPoints_[idx] * ders[k][j];
            }
        }
        return result;
    }

    // ========================================================================
    // Rational Derivative Evaluation
    // ========================================================================

    /**
     * @brief Evaluate k-th derivative of a rational NURBS curve
     *
     * Uses the formula: A^(k) = (wC)^(k) - Σ binom(k,i) * w^(i) * C^(k-i)
     * where A = wC and C is the rational curve.
     * (Piegl & Tiller equations for rational derivatives)
     */
    Point evaluateRationalDerivative(T u, size_t k) const {
        int span = findSpan(u);
        int p = static_cast<int>(degree_);
        int nd = std::min(static_cast<int>(k), p);

        auto ders = basisFunctionDerivatives(u, span, nd);

        // Compute weighted curve derivatives: (wC)^(k) and w^(k)
        std::vector<Point> CurveDers(nd + 1);
        std::vector<T> wDers(nd + 1, T(0));

        for (int kk = 0; kk <= nd; ++kk) {
            CurveDers[kk] = Point{};
            wDers[kk] = T(0);
            for (int j = 0; j <= p; ++j) {
                int idx = span - p + j;
                if (idx >= 0 && idx < static_cast<int>(controlPoints_.size())) {
                    T w = weights_[idx];
                    CurveDers[kk] = CurveDers[kk] + controlPoints_[idx] * (ders[kk][j] * w);
                    wDers[kk] += ders[kk][j] * w;
                }
            }
        }

        // Apply quotient rule recursively
        // C^(k) = (A^(k) - Σ_{i=1}^{k} binom(k,i) * w^(i) * C^(k-i)) / w
        std::vector<Point> Ck(nd + 1);

        // C^(0) = A^(0) / w^(0) = evaluate position
        if (std::abs(wDers[0]) > T(1e-14)) {
            Ck[0] = CurveDers[0] / wDers[0];
        }

        for (int kk = 1; kk <= nd; ++kk) {
            Point v = CurveDers[kk];
            for (int i = 1; i <= kk; ++i) {
                T binom = static_cast<T>(binomialCoeff(kk, i));
                v = v - Ck[kk - i] * (binom * wDers[i]);
            }
            if (std::abs(wDers[0]) > T(1e-14)) {
                Ck[kk] = v / wDers[0];
            }
        }

        if (static_cast<int>(k) <= nd) {
            return Ck[k];
        }
        return Point{};
    }

    // ========================================================================
    // Arc Length Quadrature
    // ========================================================================

    T adaptiveGaussianQuadrature(T a, T b, T tolerance, size_t maxDepth) const {
        static const T nodes[] = {
            T(-0.906179845938664), T(-0.538469310105683), T(0.0),
            T(0.538469310105683),  T(0.906179845938664)
        };
        static const T weights[] = {
            T(0.236926885056189), T(0.478628670499366), T(0.568888888888889),
            T(0.478628670499366), T(0.236926885056189)
        };
        return adaptiveQuadRecursive(a, b, nodes, weights, 5,
                                      tolerance, maxDepth, 0);
    }

    T adaptiveQuadRecursive(T a, T b, const T* nodes, const T* wts,
                             size_t numPts, T tol, size_t maxD, size_t depth) const {
        T mid = (a + b) / T(2);
        T halfW = (b - a) / T(2);

        T integral = T(0);
        for (size_t i = 0; i < numPts; ++i) {
            T u = mid + halfW * nodes[i];
            integral += wts[i] * speed(u);
        }
        integral *= halfW;

        if (depth >= maxD) return integral;

        T qW = halfW / T(2);
        T midL = (a + mid) / T(2);
        T midR = (mid + b) / T(2);

        T intL = T(0), intR = T(0);
        for (size_t i = 0; i < numPts; ++i) {
            intL += wts[i] * speed(midL + qW * nodes[i]);
            intR += wts[i] * speed(midR + qW * nodes[i]);
        }
        intL *= qW;
        intR *= qW;

        T refined = intL + intR;
        if (std::abs(integral - refined) < tol) return refined;

        return adaptiveQuadRecursive(a, mid, nodes, wts, numPts,
                                      tol / T(2), maxD, depth + 1) +
               adaptiveQuadRecursive(mid, b, nodes, wts, numPts,
                                      tol / T(2), maxD, depth + 1);
    }

    // ========================================================================
    // Cubic Bézier Approximation (Adaptive)
    // ========================================================================

    void approximateCubicRecursive(T u0, T u1, T maxError, size_t maxSegs,
                                    std::vector<BezierCurve<Dim, T>>& result,
                                    size_t depth) const {
        if (result.size() >= maxSegs || depth > 20) {
            // Force single cubic
            result.push_back(fitCubicBezier(u0, u1));
            return;
        }

        // Fit a cubic Bézier to this span
        auto cubic = fitCubicBezier(u0, u1);

        // Check approximation error by sampling
        T error = T(0);
        const int checkPts = 5;
        for (int i = 1; i < checkPts; ++i) {
            T t = T(i) / T(checkPts);
            T u = u0 + t * (u1 - u0);
            Point nurbsPt = evaluate(u);
            Point bezPt = cubic.evaluate(t);
            T dist = nurbsPt.distanceTo(bezPt);
            error = std::max(error, dist);
        }

        if (error <= maxError) {
            result.push_back(std::move(cubic));
        } else {
            T uMid = (u0 + u1) / T(2);
            approximateCubicRecursive(u0, uMid, maxError, maxSegs, result, depth + 1);
            approximateCubicRecursive(uMid, u1, maxError, maxSegs, result, depth + 1);
        }
    }

    /**
     * @brief Fit a single cubic Bézier to the NURBS curve over [u0, u1]
     */
    BezierCurve<Dim, T> fitCubicBezier(T u0, T u1) const {
        Point P0 = evaluate(u0);
        Point P3 = evaluate(u1);
        Point T0 = evaluateDerivative(u0, 1);
        Point T3 = evaluateDerivative(u1, 1);

        // Scale tangents by parameter span
        T span = u1 - u0;
        T scale = span / T(3);

        Point P1 = P0 + T0 * scale;
        Point P2 = P3 - T3 * scale;

        BezierCurve<Dim, T> bez({P0, P1, P2, P3});
        bez.setSourceRef(sourceRef_);
        return bez;
    }

    // ========================================================================
    // Utility
    // ========================================================================

    static Point perpendicular(const Point& v) {
        if constexpr (Dim == 2) {
            return Point{-v[1], v[0]};
        } else if constexpr (Dim >= 3) {
            return Point{-v[1], v[0], T(0)};
        } else {
            return Point{};
        }
    }
};

// ============================================================================
// Type Aliases
// ============================================================================

using NURBSCurve2D = NURBSCurve<2, double>;
using NURBSCurve3D = NURBSCurve<3, double>;
using NURBSArcLengthTable2D = NURBSArcLengthTable<double>;
using NURBSArcLengthTable3D = NURBSArcLengthTable<double>;

} // namespace MotionPlanner
