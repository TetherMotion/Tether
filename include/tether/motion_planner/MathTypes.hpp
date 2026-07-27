/**
 * @file MathTypes.hpp
 * @brief Core Mathematical Types for Motion Planning
 *
 * @details
 * This file provides foundational mathematical types used throughout the
 * advanced motion planning system:
 *
 * - **Vec<N,T>**: Fixed-size vector type with full arithmetic operations
 * - **Polynomial<N,T>**: Univariate polynomial with analytical derivatives
 *
 * ## Design Philosophy
 *
 * 1. **Zero Heap Allocation**: Fixed-size vectors avoid dynamic memory
 * 2. **Numerical Stability**: Horner's method for polynomial evaluation
 * 3. **Type Safety**: Template-based with compile-time dimension checking
 * 4. **Performance**: Inline operations, cache-friendly layout
 *
 * @see NurbsCurve.hpp
 * @see MotionPlan.hpp
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <algorithm>
#include <stdexcept>
#include <initializer_list>
#include <optional>
#include <vector>

namespace MotionPlanner {

// ============================================================================
// Constants
// ============================================================================

namespace MathConstants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double HALF_PI = PI / 2.0;
    constexpr double EPSILON = 1e-12;
    constexpr double SQRT2 = 1.41421356237309504880;
    constexpr double SQRT3 = 1.73205080756887729353;
    constexpr double INV_SQRT2 = 0.70710678118654752440;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Safe floating-point comparison with tolerance
 */
template<typename T>
constexpr bool nearEqual(T a, T b, T tolerance = static_cast<T>(MathConstants::EPSILON)) {
    return std::abs(a - b) <= tolerance;
}

/**
 * @brief Clamp value to range
 */
template<typename T>
constexpr T clamp(T value, T minVal, T maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

/**
 * @brief Linear interpolation
 */
template<typename T>
constexpr T lerp(T a, T b, T t) {
    return a + t * (b - a);
}

/**
 * @brief Square of a value
 */
template<typename T>
constexpr T sqr(T x) {
    return x * x;
}

/**
 * @brief Cube of a value
 */
template<typename T>
constexpr T cube(T x) {
    return x * x * x;
}

/**
 * @brief Binomial coefficient (n choose k) for non-negative integers
 *
 * Implemented with an iterative multiplicative algorithm to avoid overflow
 * where possible and to keep the function constexpr-friendly.
 */
constexpr int binomial(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    k = std::min(k, n - k);
    long long result = 1;
    for (int i = 1; i <= k; ++i) {
        result = result * (n - k + i) / i;
    }
    return static_cast<int>(result);
}

/**
 * @brief Sign of a value (-1, 0, or 1)
 */
template<typename T>
constexpr int sign(T x) {
    return (T(0) < x) - (x < T(0));
}

// ============================================================================
// Fixed-Size Vector Type
// ============================================================================

/**
 * @brief Fixed-size mathematical vector
 *
 * A template-based vector type supporting arbitrary dimensions and scalar types.
 * Designed for zero-heap allocation and efficient computation.
 *
 * @tparam N Number of dimensions
 * @tparam T Scalar type (default: double)
 *
 * ## Example Usage
 *
 * ```cpp
 * Vec<3> v1{1.0, 2.0, 3.0};
 * Vec<3> v2{4.0, 5.0, 6.0};
 *
 * auto sum = v1 + v2;
 * auto scaled = v1 * 2.0;
 * double dot = v1.dot(v2);
 * auto cross = v1.cross(v2);  // Only for 3D vectors
 * double mag = v1.magnitude();
 * auto normalized = v1.normalized();
 * ```
 */
template<size_t N, typename T = double>
class Vec {
public:
    static_assert(N > 0, "Vector dimension must be positive");
    static_assert(std::is_arithmetic_v<T>, "Vector element type must be arithmetic");

    /// Number of dimensions
    static constexpr size_t Dimensions = N;

    /// Element type
    using value_type = T;

    /// Storage type
    using storage_type = std::array<T, N>;

    // ========================================================================
    // Constructors
    // ========================================================================

    /**
     * @brief Default constructor - initializes to zero
     */
    constexpr Vec() noexcept : data_{} {}

    /**
     * @brief Fill constructor - all elements set to same value
     */
    constexpr explicit Vec(T value) noexcept : data_{} {
        for (size_t i = 0; i < N; ++i) {
            data_[i] = value;
        }
    }

    /**
     * @brief Initializer list constructor
     */
    constexpr Vec(std::initializer_list<T> init) noexcept : data_{} {
        size_t i = 0;
        for (auto it = init.begin(); it != init.end() && i < N; ++it, ++i) {
            data_[i] = *it;
        }
    }

    /**
     * @brief Array constructor
     */
    constexpr explicit Vec(const std::array<T, N>& arr) noexcept : data_(arr) {}

    /**
     * @brief Variadic constructor for exact dimension match
     */
    template<typename... Args,
             typename = std::enable_if_t<sizeof...(Args) == N &&
                                         (std::is_convertible_v<Args, T> && ...)>>
    constexpr Vec(Args... args) noexcept : data_{static_cast<T>(args)...} {}

    /**
     * @brief Copy from different scalar type
     */
    template<typename U>
    constexpr explicit Vec(const Vec<N, U>& other) noexcept : data_{} {
        for (size_t i = 0; i < N; ++i) {
            data_[i] = static_cast<T>(other[i]);
        }
    }

    // ========================================================================
    // Element Access
    // ========================================================================

    constexpr T& operator[](size_t i) noexcept { return data_[i]; }
    constexpr const T& operator[](size_t i) const noexcept { return data_[i]; }

    constexpr T& at(size_t i) {
        if (i >= N) throw std::out_of_range("Vec index out of range");
        return data_[i];
    }
    constexpr const T& at(size_t i) const {
        if (i >= N) throw std::out_of_range("Vec index out of range");
        return data_[i];
    }

    /// Named accessors for common dimensions
    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 1), T&> x() noexcept { return data_[0]; }
    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 1), const T&> x() const noexcept { return data_[0]; }

    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 2), T&> y() noexcept { return data_[1]; }
    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 2), const T&> y() const noexcept { return data_[1]; }

    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 3), T&> z() noexcept { return data_[2]; }
    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 3), const T&> z() const noexcept { return data_[2]; }

    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 4), T&> w() noexcept { return data_[3]; }
    template<size_t M = N>
    constexpr std::enable_if_t<(M >= 4), const T&> w() const noexcept { return data_[3]; }

    /// Size and data access
    constexpr size_t size() const noexcept { return N; }
    constexpr T* data() noexcept { return data_.data(); }
    constexpr const T* data() const noexcept { return data_.data(); }
    constexpr const storage_type& array() const noexcept { return data_; }

    /// Iterator support
    constexpr auto begin() noexcept { return data_.begin(); }
    constexpr auto end() noexcept { return data_.end(); }
    constexpr auto begin() const noexcept { return data_.begin(); }
    constexpr auto end() const noexcept { return data_.end(); }
    constexpr auto cbegin() const noexcept { return data_.cbegin(); }
    constexpr auto cend() const noexcept { return data_.cend(); }

    // ========================================================================
    // Arithmetic Operations
    // ========================================================================

    /// Negation
    constexpr Vec operator-() const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = -data_[i];
        }
        return result;
    }

    /// Addition
    constexpr Vec operator+(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] + other.data_[i];
        }
        return result;
    }

    constexpr Vec& operator+=(const Vec& other) noexcept {
        for (size_t i = 0; i < N; ++i) {
            data_[i] += other.data_[i];
        }
        return *this;
    }

    /// Subtraction
    constexpr Vec operator-(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] - other.data_[i];
        }
        return result;
    }

    constexpr Vec& operator-=(const Vec& other) noexcept {
        for (size_t i = 0; i < N; ++i) {
            data_[i] -= other.data_[i];
        }
        return *this;
    }

    /// Scalar multiplication
    constexpr Vec operator*(T scalar) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] * scalar;
        }
        return result;
    }

    constexpr Vec& operator*=(T scalar) noexcept {
        for (size_t i = 0; i < N; ++i) {
            data_[i] *= scalar;
        }
        return *this;
    }

    /// Scalar division
    constexpr Vec operator/(T scalar) const noexcept {
        Vec result;
        T inv = T(1) / scalar;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] * inv;
        }
        return result;
    }

    constexpr Vec& operator/=(T scalar) noexcept {
        T inv = T(1) / scalar;
        for (size_t i = 0; i < N; ++i) {
            data_[i] *= inv;
        }
        return *this;
    }

    /// Element-wise multiplication (Hadamard product)
    constexpr Vec elementMul(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] * other.data_[i];
        }
        return result;
    }

    /// Element-wise division
    constexpr Vec elementDiv(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = data_[i] / other.data_[i];
        }
        return result;
    }

    /// Element-wise minimum
    constexpr Vec elementMin(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = std::min(data_[i], other.data_[i]);
        }
        return result;
    }

    /// Element-wise maximum
    constexpr Vec elementMax(const Vec& other) const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = std::max(data_[i], other.data_[i]);
        }
        return result;
    }

    /// Element-wise absolute value
    constexpr Vec abs() const noexcept {
        Vec result;
        for (size_t i = 0; i < N; ++i) {
            result.data_[i] = std::abs(data_[i]);
        }
        return result;
    }

    // ========================================================================
    // Vector Operations
    // ========================================================================

    /**
     * @brief Dot product
     */
    constexpr T dot(const Vec& other) const noexcept {
        T result = T(0);
        for (size_t i = 0; i < N; ++i) {
            result += data_[i] * other.data_[i];
        }
        return result;
    }

    /**
     * @brief Cross product (3D vectors only)
     */
    template<size_t M = N>
    constexpr std::enable_if_t<M == 3, Vec> cross(const Vec& other) const noexcept {
        return Vec{
            data_[1] * other.data_[2] - data_[2] * other.data_[1],
            data_[2] * other.data_[0] - data_[0] * other.data_[2],
            data_[0] * other.data_[1] - data_[1] * other.data_[0]
        };
    }

    /**
     * @brief 2D cross product (returns scalar z-component)
     */
    template<size_t M = N>
    constexpr std::enable_if_t<M == 2, T> cross(const Vec& other) const noexcept {
        return data_[0] * other.data_[1] - data_[1] * other.data_[0];
    }

    /**
     * @brief Squared magnitude (avoids sqrt)
     */
    constexpr T magnitudeSq() const noexcept {
        return dot(*this);
    }

    /**
     * @brief Magnitude (Euclidean length)
     */
    T magnitude() const noexcept {
        return std::sqrt(magnitudeSq());
    }

    // --------------------------------------------------------------------
    // Legacy compatibility
    // Older code/tests used `length()` and `lengthSquared()` names; provide
    // lightweight wrappers to keep those callers working.
    // --------------------------------------------------------------------
    constexpr T lengthSquared() const noexcept { return magnitudeSq(); }
    T length() const noexcept { return magnitude(); }

    /**
     * @brief Normalize to unit length
     *
     * @return Normalized vector (zero vector if magnitude is zero)
     */
    Vec normalized() const noexcept {
        T mag = magnitude();
        if (mag < static_cast<T>(MathConstants::EPSILON)) {
            return Vec{};
        }
        return *this / mag;
    }

    /**
     * @brief Normalize in place
     *
     * @return Reference to this vector
     */
    Vec& normalize() noexcept {
        T mag = magnitude();
        if (mag >= static_cast<T>(MathConstants::EPSILON)) {
            *this /= mag;
        }
        return *this;
    }

    /**
     * @brief Check if this is a zero vector
     */
    constexpr bool isZero(T tolerance = static_cast<T>(MathConstants::EPSILON)) const noexcept {
        for (size_t i = 0; i < N; ++i) {
            if (std::abs(data_[i]) > tolerance) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Check if this is a unit vector
     */
    bool isUnit(T tolerance = static_cast<T>(MathConstants::EPSILON)) const noexcept {
        return nearEqual(magnitudeSq(), T(1), tolerance);
    }

    /**
     * @brief Distance to another vector
     */
    T distanceTo(const Vec& other) const noexcept {
        return (*this - other).magnitude();
    }

    /**
     * @brief Squared distance to another vector
     */
    constexpr T distanceSqTo(const Vec& other) const noexcept {
        return (*this - other).magnitudeSq();
    }

    /**
     * @brief Linear interpolation between vectors
     */
    constexpr Vec lerp(const Vec& other, T t) const noexcept {
        return *this + (other - *this) * t;
    }

    /**
     * @brief Project this vector onto another vector
     */
    Vec projectOnto(const Vec& other) const noexcept {
        T otherMagSq = other.magnitudeSq();
        if (otherMagSq < static_cast<T>(MathConstants::EPSILON)) {
            return Vec{};
        }
        return other * (dot(other) / otherMagSq);
    }

    /**
     * @brief Reject this vector from another (perpendicular component)
     */
    Vec rejectFrom(const Vec& other) const noexcept {
        return *this - projectOnto(other);
    }

    /**
     * @brief Reflect this vector about a normal
     */
    constexpr Vec reflect(const Vec& normal) const noexcept {
        return *this - normal * (T(2) * dot(normal));
    }

    /**
     * @brief Angle between this vector and another (radians)
     */
    T angleTo(const Vec& other) const noexcept {
        T magProduct = magnitude() * other.magnitude();
        if (magProduct < static_cast<T>(MathConstants::EPSILON)) {
            return T(0);
        }
        T cosAngle = clamp(dot(other) / magProduct, T(-1), T(1));
        return std::acos(cosAngle);
    }

    /**
     * @brief Minimum component value
     */
    constexpr T minComponent() const noexcept {
        T result = data_[0];
        for (size_t i = 1; i < N; ++i) {
            result = std::min(result, data_[i]);
        }
        return result;
    }

    /**
     * @brief Maximum component value
     */
    constexpr T maxComponent() const noexcept {
        T result = data_[0];
        for (size_t i = 1; i < N; ++i) {
            result = std::max(result, data_[i]);
        }
        return result;
    }

    /**
     * @brief Sum of all components
     */
    constexpr T sum() const noexcept {
        T result = T(0);
        for (size_t i = 0; i < N; ++i) {
            result += data_[i];
        }
        return result;
    }

    /**
     * @brief Product of all components
     */
    constexpr T product() const noexcept {
        T result = T(1);
        for (size_t i = 0; i < N; ++i) {
            result *= data_[i];
        }
        return result;
    }

    // ========================================================================
    // Comparison Operations
    // ========================================================================

    constexpr bool operator==(const Vec& other) const noexcept {
        for (size_t i = 0; i < N; ++i) {
            if (data_[i] != other.data_[i]) {
                return false;
            }
        }
        return true;
    }

    constexpr bool operator!=(const Vec& other) const noexcept {
        return !(*this == other);
    }

    /**
     * @brief Near-equality comparison with tolerance
     */
    bool nearEqual(const Vec& other, T tolerance = static_cast<T>(MathConstants::EPSILON)) const noexcept {
        for (size_t i = 0; i < N; ++i) {
            if (std::abs(data_[i] - other.data_[i]) > tolerance) {
                return false;
            }
        }
        return true;
    }

    // ========================================================================
    // Static Factory Methods
    // ========================================================================

    /**
     * @brief Create zero vector
     */
    static constexpr Vec zero() noexcept {
        return Vec{};
    }

    /**
     * @brief Create vector with all components set to one
     */
    static constexpr Vec one() noexcept {
        return Vec{T(1)};
    }

    /**
     * @brief Create unit vector along axis i
     */
    static constexpr Vec unit(size_t i) noexcept {
        Vec result;
        if (i < N) {
            result.data_[i] = T(1);
        }
        return result;
    }

    /// Common unit vectors for 2D/3D
    template<size_t M = N>
    static constexpr std::enable_if_t<(M >= 1), Vec> unitX() noexcept { return unit(0); }
    template<size_t M = N>
    static constexpr std::enable_if_t<(M >= 2), Vec> unitY() noexcept { return unit(1); }
    template<size_t M = N>
    static constexpr std::enable_if_t<(M >= 3), Vec> unitZ() noexcept { return unit(2); }

private:
    storage_type data_;
};

// ============================================================================
// Non-member operators
// ============================================================================

/// Scalar * Vector
template<size_t N, typename T>
constexpr Vec<N, T> operator*(T scalar, const Vec<N, T>& v) noexcept {
    return v * scalar;
}

// ============================================================================
// Type Aliases
// ============================================================================

using Vec2 = Vec<2, double>;
using Vec3 = Vec<3, double>;
using Vec4 = Vec<4, double>;

using Vec2f = Vec<2, float>;
using Vec3f = Vec<3, float>;
using Vec4f = Vec<4, float>;

using Vec2i = Vec<2, int>;
using Vec3i = Vec<3, int>;

// ============================================================================
// Univariate Polynomial Type
// ============================================================================

/**
 * @brief Univariate polynomial with analytical derivatives
 *
 * Represents a polynomial of the form:
 *   p(x) = a_0 + a_1*x + a_2*x^2 + ... + a_n*x^n
 *
 * @tparam MaxDegree Maximum supported degree
 * @tparam T Coefficient type (default: double)
 *
 * ## Features
 *
 * - Horner's method for numerically stable evaluation
 * - Analytical derivative and integral computation
 * - Polynomial arithmetic (addition, multiplication)
 * - Real root finding for polynomials up to cubic
 *
 * ## Example Usage
 *
 * ```cpp
 * // Create polynomial: 3x^2 + 2x + 1
 * Polynomial<3> p({1.0, 2.0, 3.0});
 *
 * double y = p.evaluate(2.0);  // = 3*4 + 2*2 + 1 = 17
 * auto dp = p.derivative();    // = 6x + 2
 * auto ip = p.integral();      // = x^3 + x^2 + x + C
 * ```
 */
template<size_t MaxDegree, typename T = double>
class Polynomial {
public:
    static_assert(MaxDegree > 0, "Maximum polynomial degree must be positive");
    static_assert(std::is_floating_point_v<T>, "Polynomial coefficients must be floating point");

    /// Maximum degree this polynomial can represent
    static constexpr size_t MaxCoefficients = MaxDegree + 1;

    /// Coefficient type
    using value_type = T;

    // ========================================================================
    // Constructors
    // ========================================================================

    /**
     * @brief Default constructor - creates zero polynomial
     */
    constexpr Polynomial() noexcept : coeffs_{}, degree_(0) {}

    /**
     * @brief Construct from initializer list of coefficients
     *
     * Coefficients are in order [a_0, a_1, a_2, ...] (constant term first)
     */
    Polynomial(std::initializer_list<T> init) : coeffs_{}, degree_(0) {
        size_t i = 0;
        for (auto it = init.begin(); it != init.end() && i < MaxCoefficients; ++it, ++i) {
            coeffs_[i] = *it;
        }
        updateDegree();
    }

    /**
     * @brief Construct from coefficient array
     *
     * @param coeffs Coefficients [a_0, a_1, ..., a_n]
     * @param n Number of coefficients (degree = n-1)
     */
    template<size_t N>
    explicit Polynomial(const std::array<T, N>& coeffs) : coeffs_{}, degree_(0) {
        static_assert(N <= MaxCoefficients, "Too many coefficients for polynomial");
        for (size_t i = 0; i < N; ++i) {
            coeffs_[i] = coeffs[i];
        }
        updateDegree();
    }

    /**
     * @brief Construct constant polynomial
     */
    explicit Polynomial(T constant) : coeffs_{}, degree_(0) {
        coeffs_[0] = constant;
        updateDegree();
    }

    // ========================================================================
    // Coefficient Access
    // ========================================================================

    /**
     * @brief Get coefficient at index
     */
    constexpr T operator[](size_t i) const noexcept {
        return (i < MaxCoefficients) ? coeffs_[i] : T(0);
    }

    /**
     * @brief Set coefficient at index
     */
    void setCoefficient(size_t i, T value) {
        if (i < MaxCoefficients) {
            coeffs_[i] = value;
            updateDegree();
        }
    }

    /**
     * @brief Get current degree
     */
    constexpr size_t degree() const noexcept { return degree_; }

    /**
     * @brief Check if polynomial is zero
     */
    constexpr bool isZero() const noexcept {
        return degree_ == 0 && std::abs(coeffs_[0]) < static_cast<T>(MathConstants::EPSILON);
    }

    /**
     * @brief Check if polynomial is constant
     */
    constexpr bool isConstant() const noexcept { return degree_ == 0; }

    // ========================================================================
    // Evaluation
    // ========================================================================

    /**
     * @brief Evaluate polynomial using Horner's method
     *
     * Horner's method is numerically stable and efficient:
     * p(x) = a_0 + x(a_1 + x(a_2 + ... + x*a_n))
     *
     * @param x Point at which to evaluate
     * @return p(x)
     */
    T evaluate(T x) const noexcept {
        if (degree_ == 0) {
            return coeffs_[0];
        }

        T result = coeffs_[degree_];
        for (size_t i = degree_; i > 0; --i) {
            result = result * x + coeffs_[i - 1];
        }
        return result;
    }

    /**
     * @brief Evaluate polynomial (operator form)
     */
    T operator()(T x) const noexcept {
        return evaluate(x);
    }

    /**
     * @brief Evaluate polynomial and its first derivative
     *
     * @param x Point at which to evaluate
     * @return Pair of (p(x), p'(x))
     */
    std::pair<T, T> evaluateWithDerivative(T x) const noexcept {
        if (degree_ == 0) {
            return {coeffs_[0], T(0)};
        }

        T p = coeffs_[degree_];
        T dp = T(0);

        for (size_t i = degree_; i > 0; --i) {
            dp = dp * x + p;
            p = p * x + coeffs_[i - 1];
        }

        return {p, dp};
    }

    // ========================================================================
    // Calculus Operations
    // ========================================================================

    /**
     * @brief Compute derivative polynomial
     *
     * If p(x) = sum(a_i * x^i), then p'(x) = sum(i * a_i * x^(i-1))
     */
    Polynomial derivative() const noexcept {
        Polynomial result;
        if (degree_ == 0) {
            return result;  // Derivative of constant is zero
        }

        for (size_t i = 1; i <= degree_; ++i) {
            result.coeffs_[i - 1] = static_cast<T>(i) * coeffs_[i];
        }
        result.updateDegree();
        return result;
    }

    /**
     * @brief Compute n-th derivative polynomial
     */
    Polynomial derivative(size_t n) const noexcept {
        Polynomial result = *this;
        for (size_t i = 0; i < n && !result.isZero(); ++i) {
            result = result.derivative();
        }
        return result;
    }

    /**
     * @brief Compute antiderivative (indefinite integral)
     *
     * If p(x) = sum(a_i * x^i), then ∫p(x)dx = sum(a_i/(i+1) * x^(i+1)) + C
     *
     * @param constant Integration constant (default: 0)
     */
    Polynomial<MaxDegree + 1, T> integral(T constant = T(0)) const noexcept {
        Polynomial<MaxDegree + 1, T> result;
        result.setCoefficient(0, constant);

        for (size_t i = 0; i <= degree_; ++i) {
            result.setCoefficient(i + 1, coeffs_[i] / static_cast<T>(i + 1));
        }
        return result;
    }

    /**
     * @brief Compute definite integral from a to b
     */
    T integrate(T a, T b) const noexcept {
        auto F = integral();
        return F.evaluate(b) - F.evaluate(a);
    }

    // ========================================================================
    // Arithmetic Operations
    // ========================================================================

    /**
     * @brief Negate polynomial
     */
    Polynomial operator-() const noexcept {
        Polynomial result;
        for (size_t i = 0; i <= degree_; ++i) {
            result.coeffs_[i] = -coeffs_[i];
        }
        result.degree_ = degree_;
        return result;
    }

    /**
     * @brief Add polynomials
     */
    Polynomial operator+(const Polynomial& other) const noexcept {
        Polynomial result;
        size_t maxDeg = std::max(degree_, other.degree_);
        for (size_t i = 0; i <= maxDeg && i < MaxCoefficients; ++i) {
            result.coeffs_[i] = coeffs_[i] + other.coeffs_[i];
        }
        result.updateDegree();
        return result;
    }

    Polynomial& operator+=(const Polynomial& other) noexcept {
        *this = *this + other;
        return *this;
    }

    /**
     * @brief Subtract polynomials
     */
    Polynomial operator-(const Polynomial& other) const noexcept {
        Polynomial result;
        size_t maxDeg = std::max(degree_, other.degree_);
        for (size_t i = 0; i <= maxDeg && i < MaxCoefficients; ++i) {
            result.coeffs_[i] = coeffs_[i] - other.coeffs_[i];
        }
        result.updateDegree();
        return result;
    }

    Polynomial& operator-=(const Polynomial& other) noexcept {
        *this = *this - other;
        return *this;
    }

    /**
     * @brief Multiply by scalar
     */
    Polynomial operator*(T scalar) const noexcept {
        Polynomial result;
        for (size_t i = 0; i <= degree_; ++i) {
            result.coeffs_[i] = coeffs_[i] * scalar;
        }
        result.updateDegree();
        return result;
    }

    Polynomial& operator*=(T scalar) noexcept {
        for (size_t i = 0; i <= degree_; ++i) {
            coeffs_[i] *= scalar;
        }
        updateDegree();
        return *this;
    }

    /**
     * @brief Divide by scalar
     */
    Polynomial operator/(T scalar) const noexcept {
        return *this * (T(1) / scalar);
    }

    Polynomial& operator/=(T scalar) noexcept {
        return *this *= (T(1) / scalar);
    }

    /**
     * @brief Multiply polynomials
     *
     * Note: Result degree is sum of input degrees. If this exceeds MaxDegree,
     * higher-order terms are truncated.
     */
    Polynomial operator*(const Polynomial& other) const noexcept {
        Polynomial result;

        for (size_t i = 0; i <= degree_; ++i) {
            for (size_t j = 0; j <= other.degree_; ++j) {
                if (i + j < MaxCoefficients) {
                    result.coeffs_[i + j] += coeffs_[i] * other.coeffs_[j];
                }
            }
        }

        result.updateDegree();
        return result;
    }

    Polynomial& operator*=(const Polynomial& other) noexcept {
        *this = *this * other;
        return *this;
    }

    // ========================================================================
    // Root Finding
    // ========================================================================

    /**
     * @brief Find real roots of the polynomial
     *
     * Uses analytical solutions for degree <= 3, otherwise returns empty.
     *
     * @param tolerance Tolerance for considering a value as zero
     * @return Vector of real roots (may be empty)
     */
    std::vector<T> realRoots(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        std::vector<T> roots;

        switch (degree_) {
            case 0:
                // Constant polynomial - no roots (or infinite if zero)
                break;

            case 1:
                // Linear: ax + b = 0 => x = -b/a
                if (std::abs(coeffs_[1]) > tolerance) {
                    roots.push_back(-coeffs_[0] / coeffs_[1]);
                }
                break;

            case 2: {
                // Quadratic
                std::vector<T> r;
                solveQuadratic(coeffs_[2], coeffs_[1], coeffs_[0], r, tolerance);
                roots.insert(roots.end(), r.begin(), r.end());
                break;
            }

            case 3: {
                // Cubic
                std::vector<T> r;
                solveCubic(coeffs_[3], coeffs_[2], coeffs_[1], coeffs_[0], r, tolerance);
                roots.insert(roots.end(), r.begin(), r.end());
                break;
            }

            default:
                // Higher-degree root finding not implemented
                break;
        }

        return roots;
    }

    /**
     * @brief Compatibility: quadraticRoots() -> returns pair of roots if real
     */
    std::optional<std::pair<T, T>> quadraticRoots() const {
        if (degree_ < 2) return std::nullopt;
        std::vector<T> r;
        solveQuadratic(coeffs_[2], coeffs_[1], coeffs_[0], r, static_cast<T>(MathConstants::EPSILON));
        if (r.size() == 2) {
            return std::make_optional(std::make_pair(r[0], r[1]));
        } else if (r.size() == 1) {
            return std::make_optional(std::make_pair(r[0], r[0]));
        }
        return std::nullopt;
    }

    /**
     * @brief Compatibility: cubicRoots() -> returns all real roots for cubic
     */
    std::vector<T> cubicRoots() const {
        if (degree_ < 3) return {};
        return realRoots(static_cast<T>(MathConstants::EPSILON));
    }

    /**
     * @brief Find roots in an interval using bisection
     *
     * @param a Lower bound
     * @param b Upper bound
     * @param tolerance Convergence tolerance
     * @param maxIterations Maximum iterations
     * @return Root if found, nullopt otherwise
     */
    std::optional<T> findRootBisection(T a, T b, T tolerance = static_cast<T>(1e-10),
                                        size_t maxIterations = 100) const {
        T fa = evaluate(a);
        T fb = evaluate(b);

        // Check if signs are different (intermediate value theorem)
        if (fa * fb > 0) {
            return std::nullopt;
        }

        for (size_t i = 0; i < maxIterations; ++i) {
            T mid = (a + b) / T(2);
            T fmid = evaluate(mid);

            if (std::abs(fmid) < tolerance || (b - a) / T(2) < tolerance) {
                return mid;
            }

            if (fa * fmid < 0) {
                b = mid;
                fb = fmid;
            } else {
                a = mid;
                fa = fmid;
            }
        }

        return (a + b) / T(2);
    }

    /**
     * @brief Find root using Newton-Raphson method
     *
     * @param x0 Initial guess
     * @param tolerance Convergence tolerance
     * @param maxIterations Maximum iterations
     * @return Root if found, nullopt otherwise
     */
    std::optional<T> findRootNewton(T x0, T tolerance = static_cast<T>(1e-10),
                                     size_t maxIterations = 50) const {
        if (degree_ == 0) {
            return std::nullopt;
        }

        T x = x0;
        for (size_t i = 0; i < maxIterations; ++i) {
            auto [fx, dfx] = evaluateWithDerivative(x);

            if (std::abs(fx) < tolerance) {
                return x;
            }

            if (std::abs(dfx) < static_cast<T>(MathConstants::EPSILON)) {
                return std::nullopt;  // Derivative too small
            }

            T xNew = x - fx / dfx;

            if (std::abs(xNew - x) < tolerance) {
                return xNew;
            }

            x = xNew;
        }

        return std::nullopt;  // Did not converge
    }

private:
    std::array<T, MaxCoefficients> coeffs_;
    size_t degree_;

    /**
     * @brief Update degree based on leading non-zero coefficient
     */
    void updateDegree() noexcept {
        degree_ = 0;
        for (size_t i = MaxCoefficients; i > 0; --i) {
            if (std::abs(coeffs_[i - 1]) > static_cast<T>(MathConstants::EPSILON)) {
                degree_ = i - 1;
                break;
            }
        }
    }

    /**
     * @brief Solve quadratic equation ax^2 + bx + c = 0
     */
    static void solveQuadratic(T a, T b, T c, std::vector<T>& roots, T tol) {
        if (std::abs(a) < tol) {
            // Degenerate to linear
            if (std::abs(b) > tol) {
                roots.push_back(-c / b);
            }
            return;
        }

        T discriminant = b * b - T(4) * a * c;

        if (discriminant < -tol) {
            // No real roots
            return;
        }

        if (std::abs(discriminant) < tol) {
            // One repeated root
            roots.push_back(-b / (T(2) * a));
            return;
        }

        // Two distinct roots
        T sqrtD = std::sqrt(discriminant);
        roots.push_back((-b - sqrtD) / (T(2) * a));
        roots.push_back((-b + sqrtD) / (T(2) * a));
    }

    /**
     * @brief Solve cubic equation ax^3 + bx^2 + cx + d = 0 using Cardano's formula
     */
    static void solveCubic(T a, T b, T c, T d, std::vector<T>& roots, T tol) {
        if (std::abs(a) < tol) {
            // Degenerate to quadratic
            solveQuadratic(b, c, d, roots, tol);
            return;
        }

        // Convert to depressed cubic: t^3 + pt + q = 0
        // where x = t - b/(3a)
        T invA = T(1) / a;
        T bOverA = b * invA;
        T cOverA = c * invA;
        T dOverA = d * invA;

        T p = cOverA - bOverA * bOverA / T(3);
        T q = (T(2) * bOverA * bOverA * bOverA - T(9) * bOverA * cOverA + T(27) * dOverA) / T(27);

        T discriminant = q * q / T(4) + p * p * p / T(27);
        T shift = -bOverA / T(3);

        if (discriminant > tol) {
            // One real root
            T sqrtD = std::sqrt(discriminant);
            T u = std::cbrt(-q / T(2) + sqrtD);
            T v = std::cbrt(-q / T(2) - sqrtD);
            roots.push_back(u + v + shift);
        } else if (std::abs(discriminant) < tol) {
            // Three real roots, at least two equal
            if (std::abs(q) < tol) {
                roots.push_back(shift);  // Triple root
            } else {
                T u = std::cbrt(-q / T(2));
                roots.push_back(T(2) * u + shift);
                roots.push_back(-u + shift);
            }
        } else {
            // Three distinct real roots (use trigonometric method)
            T r = std::sqrt(-p * p * p / T(27));
            T theta = std::acos(-q / (T(2) * r));
            T m = T(2) * std::cbrt(r);

            roots.push_back(m * std::cos(theta / T(3)) + shift);
            roots.push_back(m * std::cos((theta + T(2) * MathConstants::PI) / T(3)) + shift);
            roots.push_back(m * std::cos((theta + T(4) * MathConstants::PI) / T(3)) + shift);
        }
    }
};

// ============================================================================
// Non-member operators
// ============================================================================

/// Scalar * Polynomial
template<size_t N, typename T>
Polynomial<N, T> operator*(T scalar, const Polynomial<N, T>& p) noexcept {
    return p * scalar;
}

// ============================================================================
// Type Aliases
// ============================================================================

using Poly2 = Polynomial<2, double>;   // Up to quadratic
using Poly3 = Polynomial<3, double>;   // Up to cubic
using Poly5 = Polynomial<5, double>;   // Up to quintic
using Poly7 = Polynomial<7, double>;   // Up to septic

}  // namespace MotionPlanner
