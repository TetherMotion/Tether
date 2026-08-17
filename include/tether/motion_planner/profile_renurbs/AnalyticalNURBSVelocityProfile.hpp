/**
 * @file AnalyticalNURBSVelocityProfile.hpp
 * @brief VelocityProfile adapter for a ReNURBSProfile.
 *
 * @details
 * Wraps a `ReNURBSProfile` (per-segment NURBS curves for v(s), a(s),
 * j(s), t(s)) and exposes the `MotionPlanner::VelocityProfile` query
 * API by evaluating the underlying B-splines.
 *
 * The per-segment NURBS curves are defined over the normalized parameter
 * u ∈ [0,1]. Arc length s is mapped to u via the segment bounds, and the
 * corresponding quantity is evaluated using the scalar `NurbsCurve`. The
 * time curve is inverted with bisection to implement `arcLengthAt(t)`.
 */

#pragma once

#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cmath>
#include <limits>
#include <memory>

namespace tether::motion::profile_renurbs {

/**
 * @brief Velocity profile backed by a ReNURBS representation.
 */
class AnalyticalNURBSVelocityProfile : public MotionPlanner::VelocityProfile {
public:
    explicit AnalyticalNURBSVelocityProfile(
        std::shared_ptr<const ReNURBSProfile> profile)
        : profile_(std::move(profile)) {
        if (profile_ && !profile_->perSegment.empty()) {
            const auto& last = profile_->perSegment.back();
            totalLength_ = last.sEnd;
            totalTime_ = evaluateQuantity(last.time, 1.0);
        }
    }

    double velocityAt(double arcLength) const override {
        return evaluateAt(arcLength, 0);
    }

    double accelerationAt(double arcLength) const override {
        return evaluateAt(arcLength, 1);
    }

    double jerkAt(double arcLength) const override {
        return evaluateAt(arcLength, 2);
    }

    double timeAt(double arcLength) const override {
        return evaluateAt(arcLength, 3);
    }

    double arcLengthAt(double time) const override {
        if (!profile_ || profile_->perSegment.empty()) return 0.0;
        if (time <= 0.0) return 0.0;
        if (time >= totalTime_) return totalLength_;

        for (const auto& seg : profile_->perSegment) {
            if (!seg.time.curve) continue;
            double t0 = evaluateQuantity(seg.time, 0.0);
            double t1 = evaluateQuantity(seg.time, 1.0);
            if (time < t0 || time > t1) continue;

            // Bisection on u to find time(u) == target.
            double uLow = 0.0;
            double uHigh = 1.0;
            double u = 0.5;
            for (int iter = 0; iter < 60; ++iter) {
                double t = evaluateQuantity(seg.time, u);
                if (std::abs(t - time) < 1e-15) break;
                if (t < time) {
                    uLow = u;
                } else {
                    uHigh = u;
                }
                u = 0.5 * (uLow + uHigh);
                if (uHigh - uLow < 1e-15) break;
            }

            double segLen = seg.sEnd - seg.sStart;
            return seg.sStart + u * segLen;
        }

        // Fallback: linear in time.
        return (time / totalTime_) * totalLength_;
    }

    double totalTime() const override { return totalTime_; }
    double totalLength() const override { return totalLength_; }

    /// Access the underlying ReNURBS profile.
    const std::shared_ptr<const ReNURBSProfile>& profile() const { return profile_; }

private:
    std::shared_ptr<const ReNURBSProfile> profile_;
    double totalTime_ = 0.0;
    double totalLength_ = 0.0;

    /// Evaluate one of the four quantities at arc length s.
    /// quantityIndex: 0=velocity, 1=acceleration, 2=jerk, 3=time.
    double evaluateAt(double arcLength, int quantityIndex) const {
        if (!profile_ || profile_->perSegment.empty()) return 0.0;

        const ReNURBSSegmentProfile* seg = findSegment(arcLength);
        if (!seg) return 0.0;

        const ReNURBSQuantityCurves* qc = quantityByIndex(*seg, quantityIndex);
        if (!qc || !qc->curve) return 0.0;

        double u = segmentU(*seg, arcLength);
        return evaluateQuantity(*qc, u);
    }

    static double evaluateQuantity(const ReNURBSQuantityCurves& qc, double u) {
        if (!qc.curve) return 0.0;
        // Clamp to the B-spline domain [0,1].
        u = std::clamp(u, 0.0, 1.0);
        const auto& value = qc.curve->evaluate(u);
        return value.dim() > 0 ? value[0] : 0.0;
    }

    const ReNURBSSegmentProfile* findSegment(double s) const {
        for (const auto& seg : profile_->perSegment) {
            if (s >= seg.sStart - 1e-12 && s <= seg.sEnd + 1e-12) {
                return &seg;
            }
        }
        if (s <= profile_->perSegment.front().sStart) {
            return &profile_->perSegment.front();
        }
        if (s >= profile_->perSegment.back().sEnd) {
            return &profile_->perSegment.back();
        }
        return nullptr;
    }

    static double segmentU(const ReNURBSSegmentProfile& seg, double s) {
        double segLen = seg.sEnd - seg.sStart;
        if (segLen <= 1e-15) return 0.0;
        double u = (s - seg.sStart) / segLen;
        return std::clamp(u, 0.0, 1.0);
    }

    static const ReNURBSQuantityCurves* quantityByIndex(
        const ReNURBSSegmentProfile& seg, int index) {
        switch (index) {
            case 0: return &seg.velocity;
            case 1: return &seg.acceleration;
            case 2: return &seg.jerk;
            case 3: return &seg.time;
        }
        return nullptr;
    }
};

} // namespace tether::motion::profile_renurbs
