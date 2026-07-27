/**
 * @file Vector.hpp
 * @brief Fixed-capacity runtime-dimension vector (RVec) for the new motion stack
 *
 * @details
 * `RVec` is the coordinate type of the rewritten path-blending stack
 * (namespace `tether::motion`). Unlike the legacy templated `Vec<N>` in
 * `MathTypes.hpp`, the dimension is a *runtime* value (1..5) with a fixed
 * compile-time capacity of 5 doubles. This lets one non-templated code path
 * handle 2-, 3-, 4- and 5-axis geometry without instantiating templates per
 * dimension, so all non-template code can live in compiled .cpp files.
 *
 * Properties:
 * - No heap allocation, no templates: `std::array<double, 5>` + `uint8_t` dim.
 * - All arithmetic validates that both operands have the same dimension and
 *   throws `std::invalid_argument` otherwise (dimension mismatch is a
 *   programming error and must never silently compute garbage).
 * - `operator==` is *exact* (bitwise per-component) by design: G0 connectivity
 *   of blended paths is asserted via bitwise endpoint equality (see plan
 *   §4.6). Use `nearEqual()` for tolerant comparisons.
 *
 * All math conventions used together with this type are derived in
 * `docs/motion/GeometryFoundations.md`.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace tether::motion {

class RVec {
public:
    /// Maximum supported dimension (number of axes).
    static constexpr std::size_t kMaxDim = 5;

    // ========================================================================
    // Construction
    // ========================================================================

    /// Default: dimension 0 (invalid for geometry; assign before use).
    RVec() noexcept : data_{}, dim_(0) {}

    /// Zero vector of the given dimension (1..kMaxDim).
    static RVec zero(std::size_t dim) {
        checkDim(dim);
        RVec v;
        v.dim_ = static_cast<std::uint8_t>(dim);
        return v;
    }

    /// Construct from an initializer list; dimension = number of elements.
    /// Example: `RVec p{1.0, 2.0, 3.0};` is a 3-D point.
    RVec(std::initializer_list<double> init) : data_{}, dim_(0) {
        if (init.size() < 1 || init.size() > kMaxDim) {
            throw std::invalid_argument(
                "RVec: initializer list size must be in 1..5, got " +
                std::to_string(init.size()));
        }
        std::size_t i = 0;
        for (double x : init) {
            data_[i++] = x;
        }
        dim_ = static_cast<std::uint8_t>(init.size());
    }

    // ========================================================================
    // Access
    // ========================================================================

    std::size_t dim() const noexcept { return dim_; }

    double& operator[](std::size_t i) {
        if (i >= dim_) {
            throw std::out_of_range("RVec: index " + std::to_string(i) +
                                    " out of range for dim " + std::to_string(dim_));
        }
        return data_[i];
    }
    double operator[](std::size_t i) const {
        if (i >= dim_) {
            throw std::out_of_range("RVec: index " + std::to_string(i) +
                                    " out of range for dim " + std::to_string(dim_));
        }
        return data_[i];
    }

    /// Raw access without bounds check (i must be < dim()).
    double unchecked(std::size_t i) const noexcept { return data_[i]; }

    // ========================================================================
    // Arithmetic (all dimension-checked)
    // ========================================================================

    RVec operator+(const RVec& o) const {
        checkSameDim(o);
        RVec r = *this;
        for (std::size_t i = 0; i < dim_; ++i) r.data_[i] += o.data_[i];
        return r;
    }

    RVec operator-(const RVec& o) const {
        checkSameDim(o);
        RVec r = *this;
        for (std::size_t i = 0; i < dim_; ++i) r.data_[i] -= o.data_[i];
        return r;
    }

    RVec operator-() const {
        RVec r = *this;
        for (std::size_t i = 0; i < dim_; ++i) r.data_[i] = -r.data_[i];
        return r;
    }

    RVec operator*(double s) const {
        RVec r = *this;
        for (std::size_t i = 0; i < dim_; ++i) r.data_[i] *= s;
        return r;
    }

    RVec operator/(double s) const {
        RVec r = *this;
        double inv = 1.0 / s;
        for (std::size_t i = 0; i < dim_; ++i) r.data_[i] *= inv;
        return r;
    }

    RVec& operator+=(const RVec& o) {
        checkSameDim(o);
        for (std::size_t i = 0; i < dim_; ++i) data_[i] += o.data_[i];
        return *this;
    }

    RVec& operator-=(const RVec& o) {
        checkSameDim(o);
        for (std::size_t i = 0; i < dim_; ++i) data_[i] -= o.data_[i];
        return *this;
    }

    RVec& operator*=(double s) {
        for (std::size_t i = 0; i < dim_; ++i) data_[i] *= s;
        return *this;
    }

    // ========================================================================
    // Vector operations
    // ========================================================================

    /// Euclidean dot product.
    double dot(const RVec& o) const {
        checkSameDim(o);
        double r = 0.0;
        for (std::size_t i = 0; i < dim_; ++i) r += data_[i] * o.data_[i];
        return r;
    }

    /// Squared Euclidean norm (avoids sqrt).
    double normSq() const {
        double r = 0.0;
        for (std::size_t i = 0; i < dim_; ++i) r += data_[i] * data_[i];
        return r;
    }

    /// Euclidean norm.
    double norm() const { return std::sqrt(normSq()); }

    /**
     * @brief Unit vector in the same direction.
     * @throws std::domain_error if the norm is below `tol` (degenerate
     *         direction — callers must handle this diagnostic, never UB).
     */
    RVec normalized(double tol = 1e-300) const {
        double n = norm();
        if (n <= tol) {
            throw std::domain_error("RVec::normalized: zero-norm vector");
        }
        return *this / n;
    }

    /// Euclidean distance to another vector.
    double distanceTo(const RVec& o) const { return (*this - o).norm(); }

    // ========================================================================
    // Comparison
    // ========================================================================

    /// Exact (bitwise per-component) equality, including dimension.
    bool operator==(const RVec& o) const noexcept {
        if (dim_ != o.dim_) return false;
        for (std::size_t i = 0; i < dim_; ++i) {
            if (data_[i] != o.data_[i]) return false;
        }
        return true;
    }
    bool operator!=(const RVec& o) const noexcept { return !(*this == o); }

    /// Tolerant per-component comparison (dimensions must match).
    bool nearEqual(const RVec& o, double tol) const {
        checkSameDim(o);
        for (std::size_t i = 0; i < dim_; ++i) {
            if (std::abs(data_[i] - o.data_[i]) > tol) return false;
        }
        return true;
    }

private:
    static void checkDim(std::size_t dim) {
        if (dim < 1 || dim > kMaxDim) {
            throw std::invalid_argument("RVec: dimension must be in 1..5, got " +
                                        std::to_string(dim));
        }
    }

    void checkSameDim(const RVec& o) const {
        if (dim_ != o.dim_) {
            throw std::invalid_argument(
                "RVec: dimension mismatch (" + std::to_string(dim_) + " vs " +
                std::to_string(o.dim_) + ")");
        }
    }

    std::array<double, kMaxDim> data_;
    std::uint8_t dim_;
};

/// Scalar * vector (convenience non-member).
inline RVec operator*(double s, const RVec& v) { return v * s; }

} // namespace tether::motion
