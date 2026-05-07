/**
 * @file BezierCurve.hpp
 * @brief Bézier Curve Implementation with De Casteljau Algorithm
 *
 * @details
 * This file provides a complete Bézier curve implementation supporting:
 *
 * - Arbitrary degree curves (linear, quadratic, cubic, quintic, etc.)
 * - Arbitrary dimensions (2D, 3D, or higher)
 * - De Casteljau algorithm for numerically stable evaluation
 * - Analytical derivatives up to third order
 * - Subdivision and extraction of sub-curves
 * - Arc length computation with adaptive Gaussian quadrature
 * - Continuity checking (G0, G1, G2)
 *
 * ## De Casteljau Algorithm
 *
 * The de Casteljau algorithm recursively computes:
 *   P_i^(r) = (1-u) * P_i^(r-1) + u * P_{i+1}^(r-1)
 *
 * This is more numerically stable than direct polynomial evaluation.
 *
 * ## Derivative Computation
 *
 * The derivative of a degree-n Bézier is a degree-(n-1) Bézier with:
 *   Q_i = n * (P_{i+1} - P_i)
 *
 * @see MathTypes.hpp
 * @see PiecewisePath.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "SourceReference.hpp"

#include <vector>
#include <memory>
#include <optional>
#include <stdexcept>

namespace MotionPlanner {

// ============================================================================
// Forward Declarations
// ============================================================================

template<size_t Dim, typename T = double>
class BezierCurve;

// ============================================================================
// Bézier Curve Configuration
// ============================================================================

/**
 * @brief Configuration for arc length computation
 */
struct ArcLengthConfig {
    /// Tolerance for adaptive quadrature
    double tolerance = 1e-8;
    
    /// Maximum recursion depth
    size_t maxDepth = 20;
    
    /// Number of samples for lookup table
    size_t tableSamples = 100;
    
    /// Tolerance for parameter lookup
    double parameterTolerance = 1e-10;
};

// ============================================================================
// Arc Length Lookup Table
// ============================================================================

/**
 * @brief Precomputed arc length table for efficient queries
 */
template<typename T = double>
class ArcLengthTable {
public:
    ArcLengthTable() = default;

    // Backwards-compatible constructor used by some tests: build table directly
    // from a curve and a number of samples
    template<size_t Dim>
    ArcLengthTable(const BezierCurve<Dim, T>& curve, size_t samples) {
        ArcLengthConfig cfg;
        cfg.tableSamples = samples;
        *this = curve.buildArcLengthTable(cfg);
    }

    /**
     * @brief Build table from parameter-arc length samples
     */
    void build(const std::vector<T>& parameters,
               const std::vector<T>& arcLengths) {
        params_ = parameters;
        lengths_ = arcLengths;
        if (!lengths_.empty()) {
            totalLength_ = lengths_.back();
        }
    }

    /**
     * @brief Get total arc length
     */
    T totalLength() const noexcept { return totalLength_; }

    /**
     * @brief Get arc length at parameter u
     */
    T arcLengthAt(T u) const {
        if (params_.empty()) return T(0);
        
        u = clamp(u, T(0), T(1));
        
        // Binary search for bracketing interval
        auto it = std::lower_bound(params_.begin(), params_.end(), u);
        if (it == params_.begin()) return lengths_.front();
        if (it == params_.end()) return lengths_.back();
        
        size_t i = std::distance(params_.begin(), it);
        
        // Linear interpolation
        T t = (u - params_[i-1]) / (params_[i] - params_[i-1]);
        return lerp(lengths_[i-1], lengths_[i], t);
    }

    /**
     * @brief Get parameter u at arc length s (inverse lookup)
     */
    T parameterAt(T s, T tolerance = T(1e-10)) const {
        if (lengths_.empty()) return T(0);
        
        s = clamp(s, T(0), totalLength_);
        
        // Binary search for bracketing interval
        auto it = std::lower_bound(lengths_.begin(), lengths_.end(), s);
        if (it == lengths_.begin()) return params_.front();
        if (it == lengths_.end()) return params_.back();
        
        size_t i = std::distance(lengths_.begin(), it);
        
        // Linear interpolation
        T denom = lengths_[i] - lengths_[i-1];
        if (std::abs(denom) < tolerance) {
            return params_[i-1];
        }
        
        T t = (s - lengths_[i-1]) / denom;
        return lerp(params_[i-1], params_[i], t);
    }

    /**
     * @brief Check if table is built
     */
    bool isValid() const noexcept { return !params_.empty(); }

    /**
     * @brief Get number of samples
     */
    size_t size() const noexcept { return params_.size(); }

private:
    std::vector<T> params_;
    std::vector<T> lengths_;
    T totalLength_ = T(0);
};

// ============================================================================
// Bézier Curve Implementation
// ============================================================================

/**
 * @brief General Bézier curve supporting arbitrary degree and dimension
 *
 * @tparam Dim Number of spatial dimensions
 * @tparam T Scalar type (default: double)
 *
 * ## Example Usage
 *
 * ```cpp
 * // Create cubic Bézier in 3D
 * std::vector<Vec3> controlPoints = {
 *     {0, 0, 0}, {1, 2, 0}, {3, 2, 0}, {4, 0, 0}
 * };
 * BezierCurve<3> curve(controlPoints);
 *
 * // Evaluate at u = 0.5
 * Vec3 pos = curve.evaluate(0.5);
 * Vec3 vel = curve.evaluateDerivative(0.5, 1);
 * Vec3 acc = curve.evaluateDerivative(0.5, 2);
 *
 * // Get arc length
 * double length = curve.arcLength();
 *
 * // Subdivide at u = 0.5
 * auto [left, right] = curve.subdivide(0.5);
 * ```
 */
template<size_t Dim, typename T>
class BezierCurve {
public:
    /// Point type
    using Point = Vec<Dim, T>;
    
    /// Control point storage
    using ControlPoints = std::vector<Point>;

    // ========================================================================
    // Constructors
    // ========================================================================

    /**
     * @brief Default constructor - creates invalid curve
     */
    BezierCurve() = default;

    /**
     * @brief Construct from control points
     *
     * @param controlPoints Control points P_0, P_1, ..., P_n
     */
    explicit BezierCurve(ControlPoints controlPoints)
        : controlPoints_(std::move(controlPoints)) {}

    /**
     * @brief Construct from initializer list
     */
    BezierCurve(std::initializer_list<Point> points)
        : controlPoints_(points) {}

    /**
     * @brief Copy constructor
     */
    BezierCurve(const BezierCurve&) = default;

    /**
     * @brief Move constructor
     */
    BezierCurve(BezierCurve&&) noexcept = default;

    /**
     * @brief Copy assignment
     */
    BezierCurve& operator=(const BezierCurve&) = default;

    /**
     * @brief Move assignment
     */
    BezierCurve& operator=(BezierCurve&&) noexcept = default;

    // ========================================================================
    // Properties
    // ========================================================================

    /**
     * @brief Get curve degree (n for n+1 control points)
     */
    size_t degree() const noexcept {
        return controlPoints_.empty() ? 0 : controlPoints_.size() - 1;
    }

    /**
     * @brief Get number of control points
     */
    size_t numControlPoints() const noexcept {
        return controlPoints_.size();
    }

    /**
     * @brief Check if curve is valid (has at least 2 control points)
     */
    bool isValid() const noexcept {
        return controlPoints_.size() >= 2;
    }

    /**
     * @brief Get control points
     */
    const ControlPoints& controlPoints() const noexcept {
        return controlPoints_;
    }

    /**
     * @brief Get control point at index
     */
    const Point& controlPoint(size_t i) const {
        return controlPoints_.at(i);
    }

    /**
     * @brief Set control point at index
     */
    void setControlPoint(size_t i, const Point& p) {
        controlPoints_.at(i) = p;
        invalidateCache();
    }

    /**
     * @brief Get start point
     */
    const Point& startPoint() const {
        return controlPoints_.front();
    }

    /**
     * @brief Get end point
     */
    const Point& endPoint() const {
        return controlPoints_.back();
    }

    /**
     * @brief Get/set source reference for traceability
     */
    const SourceReference& sourceRef() const noexcept { return sourceRef_; }
    void setSourceRef(const SourceReference& ref) { sourceRef_ = ref; }

    // ========================================================================
    // De Casteljau Evaluation
    // ========================================================================

    /**
     * @brief Evaluate curve position at parameter u using de Casteljau
     *
     * @param u Parameter in [0, 1]
     * @return Position on curve
     */
    Point evaluate(T u) const {
        if (controlPoints_.empty()) {
            return Point{};
        }
        return deCasteljau(u).back()[0];
    }

    /**
     * @brief Evaluate position at parameter u (operator form)
     */
    Point operator()(T u) const {
        return evaluate(u);
    }

    /**
     * @brief Full de Casteljau pyramid computation
     *
     * Returns intermediate points at all levels, useful for subdivision.
     *
     * @param u Parameter in [0, 1]
     * @return Vector of vectors: pyramid[level][index]
     */
    std::vector<ControlPoints> deCasteljau(T u) const {
        size_t n = controlPoints_.size();
        std::vector<ControlPoints> pyramid(n);
        
        // Level 0 is the original control points
        pyramid[0] = controlPoints_;
        
        T oneMinusU = T(1) - u;
        
        // Compute each level
        for (size_t r = 1; r < n; ++r) {
            pyramid[r].resize(n - r);
            for (size_t i = 0; i < n - r; ++i) {
                pyramid[r][i] = pyramid[r-1][i] * oneMinusU + pyramid[r-1][i+1] * u;
            }
        }
        
        return pyramid;
    }

    // ========================================================================
    // Derivative Evaluation
    // ========================================================================

    /**
     * @brief Evaluate k-th derivative at parameter u
     *
     * @param u Parameter in [0, 1]
     * @param k Derivative order (0 = position, 1 = velocity, etc.)
     * @return k-th derivative vector
     */
    Point evaluateDerivative(T u, size_t k) const {
        if (k == 0) {
            return evaluate(u);
        }
        
        if (k > degree()) {
            return Point{};  // Derivative of order > degree is zero
        }
        
        // Get derivative control points
        auto derivCurve = derivativeCurve();
        for (size_t i = 1; i < k; ++i) {
            derivCurve = derivCurve.derivativeCurve();
        }
        
        return derivCurve.evaluate(u);
    }

    /**
     * @brief Create the derivative curve
     *
     * The derivative of a degree-n Bézier is a degree-(n-1) Bézier
     * with control points: Q_i = n * (P_{i+1} - P_i)
     */
    BezierCurve derivativeCurve() const {
        if (degree() == 0) {
            return BezierCurve{};
        }
        
        size_t n = degree();
        ControlPoints derivPoints(n);
        
        for (size_t i = 0; i < n; ++i) {
            derivPoints[i] = (controlPoints_[i+1] - controlPoints_[i]) * static_cast<T>(n);
        }
        
        BezierCurve result(std::move(derivPoints));
        result.setSourceRef(sourceRef_);
        return result;
    }

    /**
     * @brief Evaluate position and first 3 derivatives at once
     *
     * More efficient than calling evaluateDerivative multiple times.
     *
     * @param u Parameter in [0, 1]
     * @return Tuple of (position, velocity, acceleration, jerk)
     */
    std::tuple<Point, Point, Point, Point> evaluateAll(T u) const {
        Point pos = evaluate(u);
        
        auto d1 = derivativeCurve();
        Point vel = d1.isValid() ? d1.evaluate(u) : Point{};
        
        auto d2 = d1.derivativeCurve();
        Point acc = d2.isValid() ? d2.evaluate(u) : Point{};
        
        auto d3 = d2.derivativeCurve();
        Point jrk = d3.isValid() ? d3.evaluate(u) : Point{};
        
        return {pos, vel, acc, jrk};
    }

    // ========================================================================
    // Geometric Properties
    // ========================================================================

    /**
     * @brief Compute unit tangent vector at parameter u
     */
    Point tangent(T u) const {
        Point vel = evaluateDerivative(u, 1);
        return vel.normalized();
    }

    /**
     * @brief Compute speed (magnitude of velocity) at parameter u
     */
    T speed(T u) const {
        return evaluateDerivative(u, 1).magnitude();
    }

    /**
     * @brief Compute curvature at parameter u
     *
     * For 2D: κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
     * For 3D: κ = |γ' × γ''| / |γ'|³
     */
    T curvature(T u) const {
        Point vel = evaluateDerivative(u, 1);
        Point acc = evaluateDerivative(u, 2);
        
        T speedSq = vel.magnitudeSq();
        if (speedSq < static_cast<T>(MathConstants::EPSILON)) {
            return T(0);
        }
        
        if constexpr (Dim == 2) {
            // 2D curvature formula
            T cross = vel[0] * acc[1] - vel[1] * acc[0];
            return cross / (speedSq * std::sqrt(speedSq));
        } else if constexpr (Dim == 3) {
            // 3D curvature formula
            Point cross = vel.cross(acc);
            return cross.magnitude() / (speedSq * std::sqrt(speedSq));
        } else {
            // General case: use Frenet formula approximation
            T accPerp = (acc - vel * (acc.dot(vel) / speedSq)).magnitude();
            return accPerp / speedSq;
        }
    }

    /**
     * @brief Compute principal normal at parameter u (unit vector)
     */
    Point normal(T u) const {
        Point vel = evaluateDerivative(u, 1);
        Point acc = evaluateDerivative(u, 2);
        
        T speedSq = vel.magnitudeSq();
        if (speedSq < static_cast<T>(MathConstants::EPSILON)) {
            return Point{};
        }
        
        // Normal = (acc - (acc·vel/|vel|²)*vel) normalized
        Point accPerp = acc - vel * (acc.dot(vel) / speedSq);
        return accPerp.normalized();
    }

    /**
     * @brief Compute binormal at parameter u (3D only)
     */
    template<size_t D = Dim>
    std::enable_if_t<D == 3, Point> binormal(T u) const {
        return tangent(u).cross(normal(u));
    }

    /**
     * @brief Compute bounding box of the curve
     *
     * Uses control point hull (which bounds the curve)
     */
    std::pair<Point, Point> boundingBox() const {
        if (controlPoints_.empty()) {
            return {Point{}, Point{}};
        }
        
        Point minPt = controlPoints_[0];
        Point maxPt = controlPoints_[0];
        
        for (const auto& p : controlPoints_) {
            minPt = minPt.elementMin(p);
            maxPt = maxPt.elementMax(p);
        }
        
        return {minPt, maxPt};
    }

    // ========================================================================
    // Subdivision and Extraction
    // ========================================================================

    /**
     * @brief Subdivide curve at parameter u
     *
     * @param u Split parameter in [0, 1]
     * @return Pair of curves (left from 0 to u, right from u to 1)
     */
    std::pair<BezierCurve, BezierCurve> subdivide(T u) const {
        auto pyramid = deCasteljau(u);
        size_t n = controlPoints_.size();
        
        // Left curve: first point from each level
        ControlPoints leftPoints(n);
        for (size_t i = 0; i < n; ++i) {
            leftPoints[i] = pyramid[i][0];
        }
        
        // Right curve: last point from each level (reversed)
        ControlPoints rightPoints(n);
        for (size_t i = 0; i < n; ++i) {
            rightPoints[i] = pyramid[n - 1 - i][i];
        }
        
        BezierCurve left(std::move(leftPoints));
        BezierCurve right(std::move(rightPoints));
        
        left.setSourceRef(sourceRef_);
        right.setSourceRef(sourceRef_);
        
        return {std::move(left), std::move(right)};
    }

    /**
     * @brief Extract sub-curve between parameters u0 and u1
     *
     * @param u0 Start parameter
     * @param u1 End parameter
     * @return Sub-curve
     */
    BezierCurve extract(T u0, T u1) const {
        if (u0 >= u1) {
            return BezierCurve{};
        }
        
        // First split at u0, take right part
        auto [_, right0] = subdivide(u0);
        
        // Remap u1 to new parameter space
        T u1New = (u1 - u0) / (T(1) - u0);
        
        // Split right part at u1New, take left part
        auto [result, __] = right0.subdivide(u1New);
        
        return result;
    }

    // ========================================================================
    // Degree Elevation
    // ========================================================================

    /**
     * @brief Elevate curve to higher degree
     *
     * The curve shape is preserved exactly, but represented with more
     * control points.
     *
     * @param targetDegree Target degree (must be >= current degree)
     * @return Elevated curve
     */
    BezierCurve elevate(size_t targetDegree) const {
        size_t n = degree();
        if (targetDegree <= n) {
            return *this;
        }
        
        BezierCurve result = *this;
        while (result.degree() < targetDegree) {
            result = result.elevateDegreeOnce();
        }
        
        return result;
    }

    /**
     * @brief Elevate degree by one
     *
     * New control points:
     *   Q_i = (i/(n+1)) * P_{i-1} + ((n+1-i)/(n+1)) * P_i
     */
    BezierCurve elevateDegreeOnce() const {
        size_t n = degree();
        size_t newN = n + 1;
        ControlPoints newPoints(newN + 1);
        
        newPoints[0] = controlPoints_[0];
        newPoints[newN] = controlPoints_[n];
        
        for (size_t i = 1; i < newN; ++i) {
            T t = static_cast<T>(i) / static_cast<T>(newN);
            newPoints[i] = controlPoints_[i-1] * t + controlPoints_[i] * (T(1) - t);
        }
        
        BezierCurve result(std::move(newPoints));
        result.setSourceRef(sourceRef_);
        return result;
    }

    // ========================================================================
    // Arc Length Computation
    // ========================================================================

    /**
     * @brief Compute total arc length using adaptive Gaussian quadrature
     *
     * @param config Configuration for accuracy and performance
     * @return Total arc length
     */
    T arcLength(const ArcLengthConfig& config = {}) const {
        return arcLengthBetween(T(0), T(1), config);
    }

    /**
     * @brief Compute arc length between two parameters
     */
    T arcLengthBetween(T u0, T u1, const ArcLengthConfig& config = {}) const {
        return adaptiveGaussianQuadrature(u0, u1, config.tolerance, config.maxDepth);
    }

    /**
     * @brief Build arc length lookup table
     */
    ArcLengthTable<T> buildArcLengthTable(const ArcLengthConfig& config = {}) const {
        size_t samples = config.tableSamples;
        std::vector<T> params(samples);
        std::vector<T> lengths(samples);
        
        T totalLen = T(0);
        params[0] = T(0);
        lengths[0] = T(0);
        
        for (size_t i = 1; i < samples; ++i) {
            T u0 = static_cast<T>(i - 1) / static_cast<T>(samples - 1);
            T u1 = static_cast<T>(i) / static_cast<T>(samples - 1);
            
            params[i] = u1;
            totalLen += arcLengthBetween(u0, u1, config);
            lengths[i] = totalLen;
        }
        
        ArcLengthTable<T> table;
        table.build(params, lengths);
        return table;
    }

    /**
     * @brief Find parameter u at a given arc length s
     *
     * Uses Newton's method with arc length table for initial guess.
     */
    T parameterAtArcLength(T s, const ArcLengthTable<T>& table,
                           T tolerance = T(1e-10), size_t maxIter = 20) const {
        if (s <= T(0)) return T(0);
        if (s >= table.totalLength()) return T(1);
        
        // Initial guess from table
        T u = table.parameterAt(s);
        
        // Newton's method: find u where arcLength(0, u) = s
        for (size_t iter = 0; iter < maxIter; ++iter) {
            T currentLen = table.arcLengthAt(u);  // Approximate
            T error = currentLen - s;
            
            if (std::abs(error) < tolerance) {
                break;
            }
            
            // Derivative is speed at u
            T spd = speed(u);
            if (spd < static_cast<T>(MathConstants::EPSILON)) {
                break;
            }
            
            u -= error / spd;
            u = clamp(u, T(0), T(1));
        }
        
        return u;
    }

    // ========================================================================
    // Continuity Checking
    // ========================================================================

    /**
     * @brief Check G0 continuity with another curve (position matching)
     */
    bool isG0ContinuousWith(const BezierCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        return endPoint().nearEqual(other.startPoint(), tolerance);
    }

    /**
     * @brief Check G1 continuity with another curve (tangent direction matching)
     */
    bool isG1ContinuousWith(const BezierCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG0ContinuousWith(other, tolerance)) {
            return false;
        }
        
        Point t1 = tangent(T(1));
        Point t2 = other.tangent(T(0));
        
        // Tangent directions should be parallel (or anti-parallel for cusps)
        T dot = std::abs(t1.dot(t2));
        return dot > T(1) - tolerance;
    }

    /**
     * @brief Check G2 continuity with another curve (curvature matching)
     */
    bool isG2ContinuousWith(const BezierCurve& other,
                            T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG1ContinuousWith(other, tolerance)) {
            return false;
        }
        
        T k1 = curvature(T(1));
        T k2 = other.curvature(T(0));
        
        return std::abs(k1 - k2) < tolerance;
    }

    /**
     * @brief Get continuity level with another curve
     *
     * @return 0, 1, or 2 for G0, G1, G2 continuity (-1 if not even G0)
     */
    int continuityLevelWith(const BezierCurve& other,
                           T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (!isG0ContinuousWith(other, tolerance)) return -1;
        if (!isG1ContinuousWith(other, tolerance)) return 0;
        if (!isG2ContinuousWith(other, tolerance)) return 1;
        return 2;
    }

private:
    ControlPoints controlPoints_;
    SourceReference sourceRef_;
    
    // Cached data (mutable for const evaluation)
    mutable std::optional<ArcLengthTable<T>> arcLengthTable_;

    /**
     * @brief Invalidate cached computations
     */
    void invalidateCache() {
        arcLengthTable_.reset();
    }

    /**
     * @brief Adaptive Gaussian quadrature for arc length
     */
    T adaptiveGaussianQuadrature(T a, T b, T tolerance, size_t maxDepth) const {
        // 5-point Gauss-Legendre nodes and weights
        static const T nodes[] = {
            T(-0.906179845938664),
            T(-0.538469310105683),
            T(0.0),
            T(0.538469310105683),
            T(0.906179845938664)
        };
        static const T weights[] = {
            T(0.236926885056189),
            T(0.478628670499366),
            T(0.568888888888889),
            T(0.478628670499366),
            T(0.236926885056189)
        };
        
        return adaptiveQuadratureRecursive(a, b, nodes, weights, 5, tolerance, maxDepth, 0);
    }

    T adaptiveQuadratureRecursive(T a, T b, const T* nodes, const T* weights,
                                   size_t numPoints, T tolerance, size_t maxDepth,
                                   size_t depth) const {
        T mid = (a + b) / T(2);
        T halfWidth = (b - a) / T(2);
        
        // Compute integral over [a, b]
        T integral = T(0);
        for (size_t i = 0; i < numPoints; ++i) {
            T u = mid + halfWidth * nodes[i];
            integral += weights[i] * speed(u);
        }
        integral *= halfWidth;
        
        if (depth >= maxDepth) {
            return integral;
        }
        
        // Compute integrals over [a, mid] and [mid, b]
        T integralLeft = T(0);
        T integralRight = T(0);
        T quarterWidth = halfWidth / T(2);
        T midLeft = (a + mid) / T(2);
        T midRight = (mid + b) / T(2);
        
        for (size_t i = 0; i < numPoints; ++i) {
            T uLeft = midLeft + quarterWidth * nodes[i];
            T uRight = midRight + quarterWidth * nodes[i];
            integralLeft += weights[i] * speed(uLeft);
            integralRight += weights[i] * speed(uRight);
        }
        integralLeft *= quarterWidth;
        integralRight *= quarterWidth;
        
        T refinedIntegral = integralLeft + integralRight;
        
        // Check error estimate
        T error = std::abs(integral - refinedIntegral);
        if (error < tolerance) {
            return refinedIntegral;
        }
        
        // Recurse
        return adaptiveQuadratureRecursive(a, mid, nodes, weights, numPoints,
                                           tolerance / T(2), maxDepth, depth + 1) +
               adaptiveQuadratureRecursive(mid, b, nodes, weights, numPoints,
                                           tolerance / T(2), maxDepth, depth + 1);
    }
};

// ============================================================================
// Type Aliases
// ============================================================================

using BezierCurve2D = BezierCurve<2, double>;
using BezierCurve3D = BezierCurve<3, double>;

// ============================================================================
// Bézier Curve Factory Functions
// ============================================================================

/**
 * @brief Create linear Bézier (degree 1)
 */
template<size_t Dim, typename T = double>
BezierCurve<Dim, T> makeLinearBezier(const Vec<Dim, T>& p0, const Vec<Dim, T>& p1) {
    return BezierCurve<Dim, T>({p0, p1});
}

/**
 * @brief Create quadratic Bézier (degree 2)
 */
template<size_t Dim, typename T = double>
BezierCurve<Dim, T> makeQuadraticBezier(const Vec<Dim, T>& p0,
                                         const Vec<Dim, T>& p1,
                                         const Vec<Dim, T>& p2) {
    return BezierCurve<Dim, T>({p0, p1, p2});
}

/**
 * @brief Create cubic Bézier (degree 3)
 */
template<size_t Dim, typename T = double>
BezierCurve<Dim, T> makeCubicBezier(const Vec<Dim, T>& p0,
                                     const Vec<Dim, T>& p1,
                                     const Vec<Dim, T>& p2,
                                     const Vec<Dim, T>& p3) {
    return BezierCurve<Dim, T>({p0, p1, p2, p3});
}

/**
 * @brief Create quintic Bézier (degree 5)
 */
template<size_t Dim, typename T = double>
BezierCurve<Dim, T> makeQuinticBezier(const Vec<Dim, T>& p0,
                                       const Vec<Dim, T>& p1,
                                       const Vec<Dim, T>& p2,
                                       const Vec<Dim, T>& p3,
                                       const Vec<Dim, T>& p4,
                                       const Vec<Dim, T>& p5) {
    return BezierCurve<Dim, T>({p0, p1, p2, p3, p4, p5});
}

// ---------------------------------------------------------------------------
// Legacy compatibility shims (keeps old test/API names working)
// ---------------------------------------------------------------------------

// Legacy "create*" factory names used in older tests. These simply forward to
// the modern `make*` factory functions and also accept an optional
// `SourceReference` to set traceability information directly.

template<size_t Dim, typename T = double>
BezierCurve<Dim, T> createLinearBezier(const Vec<Dim, T>& p0,
                                       const Vec<Dim, T>& p1,
                                       const SourceReference& src = {}) {
    auto c = makeLinearBezier<Dim, T>(p0, p1);
    if (src.isValid()) c.setSourceRef(src);
    return c;
}

template<size_t Dim, typename T = double>
BezierCurve<Dim, T> createQuadraticBezier(const Vec<Dim, T>& p0,
                                           const Vec<Dim, T>& p1,
                                           const Vec<Dim, T>& p2,
                                           const SourceReference& src = {}) {
    auto c = makeQuadraticBezier<Dim, T>(p0, p1, p2);
    if (src.isValid()) c.setSourceRef(src);
    return c;
}

template<size_t Dim, typename T = double>
BezierCurve<Dim, T> createCubicBezier(const Vec<Dim, T>& p0,
                                       const Vec<Dim, T>& p1,
                                       const Vec<Dim, T>& p2,
                                       const Vec<Dim, T>& p3,
                                       const SourceReference& src = {}) {
    auto c = makeCubicBezier<Dim, T>(p0, p1, p2, p3);
    if (src.isValid()) c.setSourceRef(src);
    return c;
}

template<size_t Dim, typename T = double>
BezierCurve<Dim, T> createQuinticBezier(const Vec<Dim, T>& p0,
                                         const Vec<Dim, T>& p1,
                                         const Vec<Dim, T>& p2,
                                         const Vec<Dim, T>& p3,
                                         const Vec<Dim, T>& p4,
                                         const Vec<Dim, T>& p5,
                                         const SourceReference& src = {}) {
    auto c = makeQuinticBezier<Dim, T>(p0, p1, p2, p3, p4, p5);
    if (src.isValid()) c.setSourceRef(src);
    return c;
}

// Alias for 2D arc-length table used in tests
using ArcLengthTable2D = ArcLengthTable<double>;

// Simple free functions for continuity checks used by tests
template<size_t Dim, typename T = double>
bool checkG0Continuity(const BezierCurve<Dim, T>& a,
                       const BezierCurve<Dim, T>& b,
                       T tol = static_cast<T>(MathConstants::EPSILON)) {
    return a.isG0ContinuousWith(b, tol);
}

template<size_t Dim, typename T = double>
bool checkG1Continuity(const BezierCurve<Dim, T>& a,
                       const BezierCurve<Dim, T>& b,
                       T tol = static_cast<T>(MathConstants::EPSILON)) {
    return a.isG1ContinuousWith(b, tol);
}

/**
 * @brief Create circular arc approximation using cubic Bézier
 *
 * This creates a cubic Bézier that approximates a circular arc.
 * For arcs <= 90°, the approximation is very accurate.
 *
 * @param center Arc center
 * @param radius Arc radius
 * @param startAngle Start angle in radians
 * @param sweepAngle Sweep angle in radians (positive = CCW)
 */
template<typename T = double>
BezierCurve<2, T> makeCircularArcBezier(const Vec<2, T>& center,
                                         T radius,
                                         T startAngle,
                                         T sweepAngle) {
    // For arcs > 90°, split into multiple segments
    if (std::abs(sweepAngle) > MathConstants::HALF_PI) {
        throw std::invalid_argument("Arc sweep must be <= 90° for single cubic approximation");
    }
    
    // Magic number for circular arc approximation
    // k = (4/3) * tan(θ/4) where θ is the sweep angle
    T k = T(4.0/3.0) * std::tan(sweepAngle / T(4));
    
    T cosStart = std::cos(startAngle);
    T sinStart = std::sin(startAngle);
    T endAngle = startAngle + sweepAngle;
    T cosEnd = std::cos(endAngle);
    T sinEnd = std::sin(endAngle);
    
    // Start and end points on arc
    Vec<2, T> p0 = center + Vec<2, T>{cosStart, sinStart} * radius;
    Vec<2, T> p3 = center + Vec<2, T>{cosEnd, sinEnd} * radius;
    
    // Control points tangent to arc
    Vec<2, T> p1 = p0 + Vec<2, T>{-sinStart, cosStart} * (k * radius);
    Vec<2, T> p2 = p3 - Vec<2, T>{-sinEnd, cosEnd} * (k * radius);
    
    return makeCubicBezier(p0, p1, p2, p3);
}

/**
 * @brief Create G2 continuous blend between two line segments
 *
 * Creates a quintic Bézier that smoothly connects two line segments
 * with G2 continuity at both ends.
 *
 * @param entryPoint Point on incoming segment where blend starts
 * @param cornerPoint Corner vertex
 * @param exitPoint Point on outgoing segment where blend ends
 * @param entryDir Unit direction of incoming segment
 * @param exitDir Unit direction of outgoing segment
 * @param entryCurvature Curvature at entry (0 for lines)
 * @param exitCurvature Curvature at exit (0 for lines)
 */
template<size_t Dim, typename T = double>
BezierCurve<Dim, T> makeG2BlendCurve(const Vec<Dim, T>& entryPoint,
                                      const Vec<Dim, T>& cornerPoint,
                                      const Vec<Dim, T>& exitPoint,
                                      const Vec<Dim, T>& entryDir,
                                      const Vec<Dim, T>& exitDir,
                                      T entryCurvature = T(0),
                                      T exitCurvature = T(0)) {
    // Quintic Bézier has 6 control points
    // P0 = entryPoint, P5 = exitPoint
    // P1 determined by entry tangent
    // P4 determined by exit tangent
    // P2, P3 determined by curvature constraints
    
    Vec<Dim, T> P0 = entryPoint;
    Vec<Dim, T> P5 = exitPoint;
    
    // Distances for tangent control points
    T entryLen = entryPoint.distanceTo(cornerPoint);
    T exitLen = exitPoint.distanceTo(cornerPoint);
    
    // First-order continuity: P1 = P0 + t1 * entryDir
    T t1 = entryLen / T(3);
    Vec<Dim, T> P1 = P0 + entryDir * t1;
    
    // P4 = P5 - t4 * exitDir
    T t4 = exitLen / T(3);
    Vec<Dim, T> P4 = P5 - exitDir * t4;
    
    // Second-order continuity determines P2 and P3
    // For zero curvature at endpoints (line-to-line blend):
    // P2 should be collinear with P0-P1
    // P3 should be collinear with P5-P4
    
    T t2 = entryLen * T(2) / T(3);
    Vec<Dim, T> P2 = P0 + entryDir * t2;
    
    T t3 = exitLen * T(2) / T(3);
    Vec<Dim, T> P3 = P5 - exitDir * t3;
    
    // If curvatures are non-zero, adjust P2 and P3
    // (This is a simplified version; full G2 matching requires solving constraints)
    
    return makeQuinticBezier(P0, P1, P2, P3, P4, P5);
}

}  // namespace MotionPlanner
