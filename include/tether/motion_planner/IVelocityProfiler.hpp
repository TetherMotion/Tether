/**
 * @file IVelocityProfiler.hpp
 * @brief Abstract interface for velocity profilers.
 *
 * @details
 * This interface allows MotionPlanBuilder and other consumers to choose
 * between different velocity profiling strategies without being coupled
 * to a specific implementation:
 *
 * - **VelocityProfiler** (basic TOPP-RA): 2nd-order time-optimal profile.
 *   Bang-bang acceleration; no jerk limiting. Fastest possible trajectory
 *   but acceleration is discontinuous at constraint switching points.
 *
 * - **JerkLimitedVelocityProfiler** (jerk-integrated TOPP-RA): 3rd-order
 *   time-optimal profile with jerk as a first-class constraint inside the
 *   optimizer. Acceleration is continuous; jerk is bounded by j_max.
 *   Slightly slower than basic TOPP-RA (jerk limiting costs time) but
 *   produces smoother trajectories without post-hoc filtering.
 *
 * - **SCurveVelocityProfiler** (basic S-curve): Per-piece S-curve profiles
 *   with jerk-limited transitions. Simpler than TOPP-RA; does not produce
 *   a time-optimal profile but is well-understood and easy to reason about.
 *   Useful for applications where simplicity matters more than optimality.
 *
 * All three produce a `VelocityProfile<T>` — a tabulated v(s) profile with
 * per-point acceleration, jerk, and time. MotionPlan consumes this profile
 * directly; it does not need to know which profiler produced it.
 *
 * ## Design Rationale
 *
 * The expert guidance on jerk handling is clear: jerk must be a constraint
 * *inside* the optimizer, not a post-filter on its output. Post-hoc S-curve
 * smoothing of a TOPP-RA profile breaks both optimality and feasibility —
 * the smoothed trajectory was never checked against the curvature/centripetal/
 * per-axis constraints that TOPP-RA verified. By making jerk a constraint
 * in the JerkLimitedVelocityProfiler, we get smooth trajectories that are
 * still guaranteed feasible.
 *
 * The SCurveVelocityProfiler is provided as an alternative for users who
 * prefer the simpler S-curve approach and don't need time-optimality. It
 * respects curvature and velocity limits but is not time-optimal.
 *
 * @see VelocityProfile.hpp for the profile data structure.
 * @see SCurveProfile.hpp for the 7-phase S-curve and jerk-limited distance
 *      functions used by the profilers.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace MotionPlanner {

// Forward declarations — the full definitions are in VelocityProfile.hpp
// and PathAdapter.hpp. They are needed at template instantiation time,
// not at the point of class definition.
template<typename T> class VelocityProfile;
template<size_t Dim, typename T> class PathAdapter;
template<size_t NumAxes, typename T> struct KinematicLimits;

/// Enum identifying the profiler type (for logging/debugging).
enum class ProfilerType : uint8_t {
    /// Basic 2nd-order TOPP-RA (bang-bang acceleration, no jerk limit)
    ToppraBasic,
    /// 3rd-order TOPP-RA with jerk constraints inside the optimizer
    ToppraJerkLimited,
    /// Basic per-piece S-curve (jerk-limited but not time-optimal)
    SCurve,
};

/**
 * @brief Abstract interface for velocity profilers.
 *
 * All velocity profilers in Tether implement this interface, allowing
 * MotionPlanBuilder and other consumers to choose a profiling strategy
 * at the C++ API level.
 *
 * @tparam Dim Spatial dimension (2 for 2D, 3 for 3D)
 * @tparam T   Numeric type (default: double)
 */
template<size_t Dim, typename T = double>
class IVelocityProfiler {
public:
    using Profile = VelocityProfile<T>;
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;

    virtual ~IVelocityProfiler() = default;

    /**
     * @brief Compute a velocity profile for the given path.
     *
     * @param path The piecewise path to profile.
     * @param feedRate Commanded feed rate (units/second). The profile
     *                 velocity will not exceed this.
     * @param startVelocity Initial velocity (default: 0, start at rest).
     * @param endVelocity Target final velocity (default: 0, end at rest).
     * @param numSamples Number of sample points along the path.
     * @param startAcceleration Initial acceleration (for replanning from
     *                          a moving state; default: 0).
     * @param startJerk Initial jerk (for replanning; default: 0).
     * @return The computed velocity profile.
     */
    virtual Profile computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) = 0;

    /**
     * @brief Get the kinematic limits used by this profiler.
     */
    virtual Limits limits() const = 0;

    /**
     * @brief Get the profiler type (for logging/debugging).
     */
    virtual ProfilerType type() const = 0;

    /**
     * @brief Human-readable name of the profiler.
     */
    virtual const char* name() const = 0;
};

} // namespace MotionPlanner
