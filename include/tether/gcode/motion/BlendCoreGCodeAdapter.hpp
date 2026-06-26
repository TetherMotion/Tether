/**
 * @file BlendCoreGCodeAdapter.hpp
 * @brief SegmentTraits specialization for GCode::PlanningSegment
 *
 * Allows the G64 blender to use the shared tether::blend core.
 */

#pragma once

#include "tether/motion_planner/BlendCore.hpp"
#include "InterpolationStrategy.hpp"

namespace tether::blend {

template<>
struct SegmentTraits<GCode::PlanningSegment> {
    using Seg = GCode::PlanningSegment;

    static BlendVec position(const Seg& s, bool isEnd) {
        const auto& p = isEnd ? s.end : s.start;
        return {p[0], p[1], p[2]};
    }

    static BlendVec tangent(const Seg& s, bool isEnd) {
        BlendVec start = position(s, false);
        BlendVec end = position(s, true);
        return (end - start).normalized();
    }

    static double curvature(const Seg& s, bool /*isEnd*/) {
        if (!s.isArc() || s.arcRadius < 1e-15) return 0.0;
        return static_cast<double>(s.arcDirection()) / s.arcRadius;
    }

    static double length(const Seg& s) { return s.segmentLength; }
    static bool isArc(const Seg& s) { return s.isArc(); }
    static int arcDirection(const Seg& s) { return s.arcDirection(); }
};

} // namespace tether::blend
