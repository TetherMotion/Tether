/**
 * @file AnalyticalTypes.hpp
 * @brief Core data structures for the analytical TOPPRA-equivalent profiler.
 *
 * @details
 * This file defines the fundamental types used by the analytical velocity
 * profiler that operates in the arc-length parameterization space:
 *
 * - **ControlMode**: Enum identifying the control mode of a switching arc
 *   (max acceleration, max deceleration, zero jerk, singular, constraint
 *   surface following).
 * - **SwitchingArc**: A single arc in the switching structure, storing
 *   the domain, mode, initial conditions, and integration parameters.
 * - **KinematicCoefficients**: Computed coefficients that transform
 *   arc-length dynamics to task-space derivatives.
 * - **EtaBounds**: The feasible interval for the jerk control input eta
 *   at a given state (u, v, a).
 * - **AnalyticalTrajectorySource**: Abstract interface for exact trajectory
 *   sampling, implemented by both SSR and Hybrid representations. This
 *   is the bridge to MotionPlan for backward-compatible consumption.
 *
 * ## Arc-Length Dynamics
 *
 * The path C(u) is parameterized by arc length s, with state:
 *   x(s) = [t(s), v(s), a(s)]  where v = ds/dt, a = dv/dt
 *
 * Dynamics (using ' for d/ds, dot for d/dt):
 *   t' = 1/v
 *   v' = a/v
 *   a' = eta/v    where eta = jerk = da/dt (control input)
 *
 * The control eta appears linearly in the task-space jerk:
 *   qddd = j⃗ * v³ + 3 * κ⃗ * v * a + T * eta
 *
 * where T = tangent, κ⃗ = curvature vector, j⃗ = jounce vector (all
 * arc-length derivatives from NurbsCurve::arcDerivatives).
 *
 * @see NumericalUtils.hpp for the numerical building blocks.
 * @see ConstraintEvaluator.hpp for eta bound computation.
 * @see SwitchingStructureRepresentation.hpp for Class A.
 * @see HybridMonotoneRepresentation.hpp for Class B.
 */

#pragma once

#include "../MathTypes.hpp"
#include "../VelocityProfile.hpp"
#include "../SourceReference.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace MotionPlanner::analytical {

// ============================================================================
// Control Mode
// ============================================================================

/**
 * @brief Control mode for a switching arc in the time-optimal solution.
 *
 * The time-optimal controller is bang-bang or singular in the jerk
 * control input eta = da/dt:
 *
 * - **ACCEL_MAX**: eta = eta_upper (maximal acceleration increase)
 * - **DECEL_MAX**: eta = eta_lower (maximal deceleration)
 * - **ZERO_JERK**: eta = 0 (coasting with constant acceleration)
 * - **SINGULAR**: singular arc, determined by Pontryagin optimality
 *   conditions (switching function phi = 0 over an interval)
 * - **CONSTRAINT_SURFACE**: following an active constraint boundary
 *   (e.g., velocity constraint active with a = 0)
 */
enum class ControlMode : uint8_t {
    ACCEL_MAX,            ///< eta = eta_upper
    DECEL_MAX,            ///< eta = eta_lower
    ZERO_JERK,            ///< eta = 0, a = const
    SINGULAR,             ///< singular arc (phi = 0)
    CONSTRAINT_SURFACE,   ///< following active constraint boundary
};

/**
 * @brief Get human-readable name for a control mode.
 */
inline const char* controlModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::ACCEL_MAX:          return "ACCEL_MAX";
        case ControlMode::DECEL_MAX:          return "DECEL_MAX";
        case ControlMode::ZERO_JERK:          return "ZERO_JERK";
        case ControlMode::SINGULAR:           return "SINGULAR";
        case ControlMode::CONSTRAINT_SURFACE: return "CONSTRAINT_SURFACE";
    }
    return "UNKNOWN";
}

// ============================================================================
// Kinematic Coefficients
// ============================================================================

/**
 * @brief Computed coefficients at a state (s, v, a) for constraint evaluation.
 *
 * These coefficients express the task-space kinematic quantities as
 * functions of the arc-length state (v, a) and control (eta):
 *
 *   qdot  = T * v
 *   qddot = κ⃗ * v² + T * a
 *   qddd  = j⃗ * v³ + 3 * κ⃗ * v * a + T * eta
 *
 * where T, κ⃗, j⃗ are the arc-length derivatives of the path
 * (tangent, curvature vector, jounce vector).
 *
 * For the jerk constraint |qddd_i| <= qddd_max_i, we write:
 *   qddd_i = alpha_i * eta + beta_i
 *
 * where:
 *   alpha_i = T_i  (tangent component i)
 *   beta_i  = j⃗_i * v³ + 3 * κ⃗_i * v * a  (jounce and curvature terms)
 */
struct KinematicCoefficients {
    /// Tangent vector T = dp/ds (unit, from arcDerivatives order 1)
    std::vector<double> tangent;      // size = dim

    /// Curvature vector κ⃗ = d²p/ds² (from arcDerivatives order 2)
    std::vector<double> curvature;    // size = dim

    /// Jounce vector j⃗ = d³p/ds³ (from arcDerivatives order 3)
    std::vector<double> jounce;       // size = dim

    /// Per-axis alpha coefficients for jerk: alpha_i = T_i
    std::vector<double> alpha_jerk;   // size = dim

    /// Per-axis beta coefficients for jerk: beta_i = j⃗_i*v³ + 3*κ⃗_i*v*a
    std::vector<double> beta_jerk;    // size = dim

    /// Speed factor g(u) = ||C'(u)|| = ||T|| * ... actually for arc-length
    /// parameterization, ds/du = g(u), and T = C'(u)/g(u). So g = ||C'(u)||.
    /// But since we work in s-space, T is already the unit tangent.
    double speedFactor = 0.0;  // g(u) = ||C'(u)||

    /// Path curvature magnitude kappa = ||κ⃗||
    double kappa = 0.0;
};

// ============================================================================
// Eta Bounds
// ============================================================================

/**
 * @brief Feasible interval [eta_min, eta_max] for the jerk control input.
 *
 * At a given state (s, v, a), the constraints on velocity, acceleration,
 * and jerk define a feasible interval for eta = da/dt.
 *
 * If the interval is empty (eta_min > eta_max), the state is infeasible.
 */
struct EtaBounds {
    double eta_min = -std::numeric_limits<double>::infinity();
    double eta_max =  std::numeric_limits<double>::infinity();

    /// Check if the interval is non-empty.
    bool feasible() const { return eta_min <= eta_max; }

    /// Check if a given eta value is within the bounds.
    bool contains(double eta) const {
        return eta >= eta_min && eta <= eta_max;
    }

    /// Clamp eta to the feasible interval.
    double clamp(double eta) const {
        return std::max(eta_min, std::min(eta, eta_max));
    }

    /// Intersect with another interval (take the tighter bounds).
    void intersect(const EtaBounds& other) {
        eta_min = std::max(eta_min, other.eta_min);
        eta_max = std::min(eta_max, other.eta_max);
    }
};

// ============================================================================
// Switching Arc
// ============================================================================

/**
 * @brief A single arc in the switching structure representation.
 *
 * An arc covers the arc-length interval [s_begin, s_end] with a fixed
 * control mode. The trajectory within the arc is reconstructed by
 * integrating the arc-length dynamics ODEs from the initial conditions.
 *
 * For ACCEL_MAX / DECEL_MAX modes:
 *   eta(s) = eta_bound(s)  [upper or lower, from active constraints]
 *   Dynamics: v' = a/v, a' = eta/v, t' = 1/v
 *
 * For ZERO_JERK mode:
 *   eta = 0, so a = const = a0 throughout the arc
 *
 * For SINGULAR / CONSTRAINT_SURFACE:
 *   eta is determined by the constraint/optimality condition
 */
struct SwitchingArc {
    /// Arc-length domain
    double s_begin = 0.0;
    double s_end = 0.0;

    /// Control mode
    ControlMode mode = ControlMode::ACCEL_MAX;

    /// Constant eta value for this arc.
    /// For ACCEL_MAX: eta = eta_upper at s_begin (may vary along arc
    ///   as constraints change, but we store the initial value and
    ///   recompute during integration)
    /// For DECEL_MAX: eta = eta_lower
    /// For ZERO_JERK: eta = 0
    /// For SINGULAR/CONSTRAINT_SURFACE: the determined value
    double eta = 0.0;

    /// Initial conditions at s_begin (stored for independent reconstruction)
    double v0 = 0.0;   ///< v(s_begin)
    double a0 = 0.0;   ///< a(s_begin)
    double t0 = 0.0;   ///< t(s_begin)
    double u0 = 0.0;   ///< u(s_begin) — NURBS parameter

    /// For CONSTRAINT_SURFACE: which constraint is active
    int active_constraint_idx = -1;

    /// Integration tolerance for ODE reconstruction
    double integrator_abs_tol = 1e-12;
    double integrator_rel_tol = 1e-10;

    /// Arc duration (time span), computed during solve
    double duration = 0.0;

    /// Check if this arc is valid (non-empty domain)
    bool valid() const { return s_end > s_begin; }

    /// Arc length span
    double length() const { return s_end - s_begin; }
};

// ============================================================================
// Analytical Trajectory Source (Interface for MotionPlan compatibility)
// ============================================================================

/**
 * @brief Abstract interface for analytical trajectory sampling.
 *
 * Both the Switching Structure Representation (SSR) and the Hybrid
 * Monotone Representation implement this interface, allowing MotionPlan
 * to consume exact trajectory data without knowing which representation
 * is used.
 *
 * This interface is designed to be backward-compatible with MotionPlan's
 * existing sampled-profile consumption: MotionPlan checks if an analytical
 * source is available, and if so, uses it for exact sampling; otherwise
 * it falls back to the tabulated VelocityProfile.
 *
 * The methods return task-space quantities (position, velocity, acceleration)
 * in R^Dim, as well as path-space quantities (arc length, path velocity,
 * path acceleration, path jerk, curvature) needed by MotionPlan.
 */
template<size_t Dim, typename T = double>
class AnalyticalTrajectorySource {
public:
    virtual ~AnalyticalTrajectorySource() = default;

    /// Total traversal time (seconds)
    virtual T totalTime() const = 0;

    /// Total path length (arc length)
    virtual T totalLength() const = 0;

    /// Position in task space at time t
    virtual Vec<Dim, T> position(T t) const = 0;

    /// Velocity vector in task space at time t
    virtual Vec<Dim, T> velocity(T t) const = 0;

    /// Acceleration vector in task space at time t
    virtual Vec<Dim, T> acceleration(T t) const = 0;

    /// Arc length position at time t
    virtual T arcLength(T t) const = 0;

    /// Time at which the trajectory reaches arc length s
    virtual T timeAtArcLength(T s) const = 0;

    /// Path (tangential) velocity magnitude at time t
    virtual T pathVelocity(T t) const = 0;

    /// Path (tangential) acceleration magnitude at time t
    virtual T pathAcceleration(T t) const = 0;

    /// Path jerk magnitude at time t
    virtual T pathJerk(T t) const = 0;

    /// Curvature at time t
    virtual T curvature(T t) const = 0;

    /// Source reference at time t (G-code traceability)
    virtual SourceReference sourceRef(T t) const = 0;

    /// Segment index at time t
    virtual size_t segmentIndex(T t) const = 0;

    /// Segment parameter [0,1] at time t
    virtual T segmentParameter(T t) const = 0;

    /// Representation type name (for logging/debugging)
    virtual const char* representationName() const = 0;
};

// ============================================================================
// Error Certificate (for Hybrid representation)
// ============================================================================

/**
 * @brief Certified error bounds for trajectory samples.
 *
 * The Hybrid Monotone Representation provides guaranteed error bounds
 * for position, velocity, and acceleration samples based on the
 * precomputed element-wise certificates.
 */
struct ErrorCertificate {
    /// Position error bound (task-space units)
    double pos_error = 0.0;

    /// Velocity error bound (task-space units/second)
    double vel_error = 0.0;

    /// Acceleration error bound (task-space units/second²)
    double acc_error = 0.0;
};

} // namespace MotionPlanner::analytical
