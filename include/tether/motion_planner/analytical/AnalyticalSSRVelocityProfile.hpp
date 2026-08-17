/**
 * @file AnalyticalSSRVelocityProfile.hpp
 * @brief VelocityProfile adapter for an analytical trajectory source.
 *
 * @details
 * Wraps any `AnalyticalTrajectorySource` (SSR, Hybrid, WSS, or the
 * unified `TrajectorySampler`) and exposes the `VelocityProfile` query
 * API by delegating to the source's exact path-kinematic methods.
 */

#pragma once

#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/analytical/AnalyticalTypes.hpp"

#include <memory>

namespace MotionPlanner::analytical {

/**
 * @brief Velocity profile backed by an analytical trajectory source.
 *
 * @tparam Dim Spatial dimension.
 * @tparam T   Numeric type (default: double).
 */
template<size_t Dim, typename T = double>
class AnalyticalSSRVelocityProfile : public VelocityProfile {
public:
    using Source = AnalyticalTrajectorySource<Dim, T>;

    explicit AnalyticalSSRVelocityProfile(std::shared_ptr<Source> source)
        : source_(std::move(source)) {}

    double velocityAt(double arcLength) const override {
        if (!source_) return 0.0;
        T t = source_->timeAtArcLength(static_cast<T>(arcLength));
        return static_cast<double>(source_->pathVelocity(t));
    }

    double accelerationAt(double arcLength) const override {
        if (!source_) return 0.0;
        T t = source_->timeAtArcLength(static_cast<T>(arcLength));
        return static_cast<double>(source_->pathAcceleration(t));
    }

    double jerkAt(double arcLength) const override {
        if (!source_) return 0.0;
        T t = source_->timeAtArcLength(static_cast<T>(arcLength));
        return static_cast<double>(source_->pathJerk(t));
    }

    double timeAt(double arcLength) const override {
        if (!source_) return 0.0;
        return static_cast<double>(source_->timeAtArcLength(static_cast<T>(arcLength)));
    }

    double arcLengthAt(double time) const override {
        if (!source_) return 0.0;
        return static_cast<double>(source_->arcLength(static_cast<T>(time)));
    }

    double totalTime() const override {
        if (!source_) return 0.0;
        return static_cast<double>(source_->totalTime());
    }

    double totalLength() const override {
        if (!source_) return 0.0;
        return static_cast<double>(source_->totalLength());
    }

    /// Access the underlying analytical source.
    const std::shared_ptr<Source>& source() const { return source_; }

    /// Tabulated points sampled from the analytical source (lazy, cached).
    const std::vector<VelocityProfilePoint>& points() const override {
        if (sampledCache_.points().empty() && source_ && source_->totalLength() > T(0)) {
            sampleSourceTo(sampledCache_, 200);
        }
        return sampledCache_.points();
    }

private:
    std::shared_ptr<Source> source_;
    mutable SampledVelocityProfile sampledCache_;

    void sampleSourceTo(SampledVelocityProfile& out, size_t numSamples) const {
        double L = static_cast<double>(source_->totalLength());
        if (L <= 0.0 || numSamples < 2) return;

        out.reserve(numSamples);
        double ds = L / static_cast<double>(numSamples - 1);
        for (size_t i = 0; i < numSamples; ++i) {
            double s = std::min(i * ds, L);
            T t = source_->timeAtArcLength(static_cast<T>(s));
            VelocityProfilePoint pt;
            pt.arcLength = s;
            pt.velocity = static_cast<double>(source_->pathVelocity(t));
            pt.acceleration = static_cast<double>(source_->pathAcceleration(t));
            pt.jerk = static_cast<double>(source_->pathJerk(t));
            pt.time = static_cast<double>(t);
            out.addPoint(pt);
        }
    }
};

} // namespace MotionPlanner::analytical
