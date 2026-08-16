/**
 * @file TrajectorySampler.hpp
 * @brief Unified trajectory sampling interface wrapping SSR or Hybrid.
 *
 * @details
 * The TrajectorySampler provides a single interface for sampling
 * position, velocity, and acceleration at any time t, regardless of
 * whether the underlying representation is:
 * - SSR (Switching Structure Representation, Class A) — exact, procedural
 * - Hybrid (Hybrid Monotone Representation, Class B) — fast, certifiable
 *
 * It implements the AnalyticalTrajectorySource interface by delegating
 * to the wrapped representation.
 *
 * ## Usage
 *
 * ```cpp
 * auto ssr = std::make_shared<SwitchingStructureRepresentation<3>>(path, arcs, evaluator);
 * TrajectorySampler<3> sampler(ssr);
 *
 * for (double t = 0; t < sampler.totalTime(); t += 0.001) {
 *     auto [pos, vel, acc] = sampler.state(t);
 *     // Use pos, vel, acc for control
 * }
 * ```
 *
 * @see SwitchingStructureRepresentation.hpp for Class A.
 * @see HybridMonotoneRepresentation.hpp for Class B.
 * @see AnalyticalTypes.hpp for AnalyticalTrajectorySource.
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "SwitchingStructureRepresentation.hpp"
#include "HybridMonotoneRepresentation.hpp"

#include <memory>
#include <array>

namespace MotionPlanner::analytical {

/**
 * @brief Unified trajectory sampler wrapping either SSR or Hybrid.
 *
 * @tparam Dim Spatial dimension
 * @tparam T   Numeric type (default: double)
 */
template<size_t Dim, typename T = double>
class TrajectorySampler : public AnalyticalTrajectorySource<Dim, T> {
public:
    using Point = Vec<Dim, T>;
    using SSR = SwitchingStructureRepresentation<Dim, T>;
    using Hybrid = HybridMonotoneRepresentation<Dim, T>;

    /// Representation type enum
    enum class RepresentationType : uint8_t {
        SSR,     ///< Switching Structure Representation (exact)
        Hybrid,  ///< Hybrid Monotone Representation (certified)
    };

    /// Construct from SSR
    explicit TrajectorySampler(std::shared_ptr<SSR> ssr)
        : type_(RepresentationType::SSR)
        , ssr_(std::move(ssr)) {}

    /// Construct from Hybrid
    explicit TrajectorySampler(std::shared_ptr<Hybrid> hybrid)
        : type_(RepresentationType::Hybrid)
        , hybrid_(std::move(hybrid)) {}

    // ========================================================================
    // AnalyticalTrajectorySource interface
    // ========================================================================

    T totalTime() const override {
        return type_ == RepresentationType::SSR
            ? ssr_->totalTime() : hybrid_->totalTime();
    }

    T totalLength() const override {
        return type_ == RepresentationType::SSR
            ? ssr_->totalLength() : hybrid_->totalLength();
    }

    Point position(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->position(t) : hybrid_->position(t);
    }

    Point velocity(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->velocity(t) : hybrid_->velocity(t);
    }

    Point acceleration(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->acceleration(t) : hybrid_->acceleration(t);
    }

    T arcLength(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->arcLength(t) : hybrid_->arcLength(t);
    }

    T pathVelocity(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->pathVelocity(t) : hybrid_->pathVelocity(t);
    }

    T pathAcceleration(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->pathAcceleration(t) : hybrid_->pathAcceleration(t);
    }

    T pathJerk(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->pathJerk(t) : hybrid_->pathJerk(t);
    }

    T curvature(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->curvature(t) : hybrid_->curvature(t);
    }

    SourceReference sourceRef(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->sourceRef(t) : hybrid_->sourceRef(t);
    }

    size_t segmentIndex(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->segmentIndex(t) : hybrid_->segmentIndex(t);
    }

    T segmentParameter(T t) const override {
        return type_ == RepresentationType::SSR
            ? ssr_->segmentParameter(t) : hybrid_->segmentParameter(t);
    }

    const char* representationName() const override {
        if (type_ == RepresentationType::SSR) {
            return ssr_->representationName();
        }
        return hybrid_->representationName();
    }

    // ========================================================================
    // Convenience: get full state at time t
    // ========================================================================

    /**
     * @brief Get complete state [position, velocity, acceleration] at time t.
     * @return Array of 3 points: {position, velocity, acceleration}
     */
    std::array<Point, 3> state(T t) const {
        return {position(t), velocity(t), acceleration(t)};
    }

    // ========================================================================
    // Access to underlying representation
    // ========================================================================

    RepresentationType representationType() const { return type_; }

    const SSR& ssr() const { return *ssr_; }
    const Hybrid& hybrid() const { return *hybrid_; }

    /**
     * @brief Update the path pointer in the underlying representation.
     * Call this after the path has been moved to a new location.
     */
    void setPath(const typename SSR::Path& path) {
        if (type_ == RepresentationType::SSR && ssr_) {
            ssr_->setPath(path);
        } else if (type_ == RepresentationType::Hybrid && hybrid_) {
            hybrid_->setPath(path);
        }
    }

    /**
     * @brief Get error certificate (only meaningful for Hybrid representation).
     * @return Error certificate; zeros for SSR (which is exact)
     */
    ErrorCertificate certify(T t) const {
        if (type_ == RepresentationType::Hybrid) {
            return hybrid_->certify(t);
        }
        return {};  // SSR is exact (to integration tolerance)
    }

private:
    RepresentationType type_;
    std::shared_ptr<SSR> ssr_;
    std::shared_ptr<Hybrid> hybrid_;
};

} // namespace MotionPlanner::analytical
