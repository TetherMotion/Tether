/**
 * @file MotionPlanner.hpp
 * @brief Main Header for the Advanced Motion Planner Framework
 *
 * @details
 * Include this single header to access the complete motion planning system.
 *
 * ## Overview
 *
 * This motion planner converts G-code motion segments into smooth, jerk-limited
 * trajectories represented as piecewise quintic Bézier curves with analytical
 * S-curve velocity profiles.
 *
 * ## Key Features
 *
 * - **Analytical Evaluation**: Position, velocity, acceleration, and jerk at
 *   any time t without numerical integration
 *
 * - **Full Lookahead/Lookbehind**: Support for negative feed rate override
 *   and reverse motion
 *
 * - **Complete Traceability**: Every data structure traces back to original
 *   G-code line numbers
 *
 * - **G2-Continuous Blending**: Smooth corner transitions matching position,
 *   tangent, and curvature
 *
 * - **Per-Axis and Cartesian Limits**: Velocity, acceleration, and jerk limits
 *   on both axis and path levels
 *
 * ## Quick Start
 *
 * ```cpp
 * #include <tether/motion_planner/MotionPlanner.hpp>
 *
 * using namespace MotionPlanner;
 *
 * // Create motion segments (from G-code or manually)
 * MotionSegmentList segments;
 * segments.append(MotionSegment::linear({0,0,0}, {100,0,0}, 50.0));
 * segments.append(MotionSegment::linear({100,0,0}, {100,100,0}, 50.0));
 *
 * // Build motion plan
 * KinematicLimits3D limits;
 * limits.path.maxPathVelocity = 100.0;
 * limits.path.maxPathAcceleration = 500.0;
 * limits.path.maxPathJerk = 5000.0;
 *
 * MotionPlanBuilder3D builder(limits);
 * auto plan = builder.build(segments, 50.0);
 *
 * // Evaluate at any time
 * auto state = plan.evaluateAt(1.5);  // State at t=1.5 seconds
 * auto pos = state.position;
 * auto vel = state.velocity;
 * auto accel = state.acceleration;
 * auto lineRef = state.sourceRef;  // Traceability to G-code
 *
 * // Feed rate override
 * plan.setFeedOverride(0.5);  // 50% speed
 *
 * // Reverse motion
 * plan.setReverse(true);
 * ```
 *
 * ## Architecture
 *
 * ```
 *                    G-Code Input
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │   GCodeAdapter      │  (Step 10)
 *              │   (Parser Bridge)   │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │  MotionSegmentList  │  (Step 4)
 *              │  (Linked List)      │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │  PathBuilderAdapter │  (Step 6)
 *              │  + SegmentConverter │  (Step 5)
 *              │  + PathBlender      │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │ PiecewiseNurbsPath  │  (Step 3)
 *              │   (Arc-Length)      │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │  BasicTOPPRA        │  (Step 7)
 *              │  (TOPP-RA Style)    │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │   SCurveProfile     │  (Step 8)
 *              │   (7-Phase)         │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *              ┌─────────────────────┐
 *              │    MotionPlan       │  (Step 9)
 *              │  (Unified Query)    │
 *              └─────────────────────┘
 *                         │
 *                         ▼
 *                State(t): p, v, a, j
 * ```
 *
 * ## Mathematical Foundation
 *
 * ### Bézier Curves (Step 2)
 *
 * Quintic Bézier with 6 control points:
 * $$ B(u) = \sum_{i=0}^{5} \binom{5}{i} (1-u)^{5-i} u^i P_i $$
 *
 * Evaluated using de Casteljau's algorithm for numerical stability.
 *
 * ### Curvature Computation
 *
 * For a parametric curve γ(u):
 * $$ \kappa = \frac{|\gamma' \times \gamma''|}{|\gamma'|^3} $$
 *
 * ### S-Curve Velocity Profile (Step 8)
 *
 * Position during jerk phase:
 * $$ p(t) = p_0 + v_0 t + \frac{1}{2} a_0 t^2 + \frac{1}{6} j t^3 $$
 *
 * ### Arc Length Computation
 *
 * Adaptive Gaussian quadrature:
 * $$ L = \int_0^1 |\gamma'(u)| du \approx \sum_{i=1}^{n} w_i |\gamma'(u_i)| $$
 *
 * ## Files
 *
 * | File | Description |
 * |------|-------------|
 * | MathTypes.hpp | Vec<N,T>, Polynomial<D,T>, constants |
 * | SourceReference.hpp | G-code traceability infrastructure |
 * | NurbsCurve.hpp | NURBS curves with rational de Casteljau |
 * | PiecewiseNurbsPath.hpp | Arc-length parameterized piecewise path |
 * | MotionSegment.hpp | Motion segment data structures |
 * | PathAdapter.hpp | Segment→path conversion (SegmentConverter+PathBlender) |
 * | VelocityProfile.hpp | TOPP-RA velocity profile data structure |
 * | BasicTOPPRA.hpp | Basic 2nd-order TOPP-RA profiler |
 * | VelocityProfiler.hpp | Abstract velocity profiler interface |
 * | JerkConstrainedTOPPRA.hpp | Jerk-constrained 3rd-order TOPP-RA profiler |
 * | SCurveProfile.hpp | 7-phase jerk-limited profiles |
 * | MotionPlan.hpp | Unified query interface |
 * | GCodeAdapter.hpp | G-code parser integration |
 *
 * ## No External Dependencies
 *
 * All algorithms are implemented from scratch. No external libraries are
 * required except the C++ standard library.
 *
 * @author Advanced Motion Generator Implementation
 * @version 1.0.0
 */

#pragma once

// Core mathematical types
#include "MathTypes.hpp"

// Traceability infrastructure
#include "SourceReference.hpp"

// Motion segment data structures
#include "MotionSegment.hpp"

// Path adapter (bridges new geometry core to old template API)
#include "PathAdapter.hpp"

// Velocity profiling
#include "VelocityProfile.hpp"
#include "VelocityProfiler.hpp"
#include "BasicTOPPRA.hpp"
#include "JerkConstrainedTOPPRA.hpp"

// Analytical TOPPRA-equivalent profiler (arc-length space, SSR + Hybrid)
#include "analytical/NumericalUtils.hpp"
#include "analytical/AnalyticalTypes.hpp"
#include "analytical/ConstraintEvaluator.hpp"
#include "analytical/SwitchingStructureRepresentation.hpp"
#include "analytical/HybridMonotoneRepresentation.hpp"
#include "analytical/TrajectorySampler.hpp"
#include "analytical/AnalyticalTOPPRA.hpp"
#include "analytical/AnalyticalJerkLimitedTOPPRA.hpp"
#include "analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp"

// S-curve jerk-limited profiles
#include "SCurveProfile.hpp"

// Unified motion plan
#include "MotionPlan.hpp"

// G-code parser integration
#include "GCodeAdapter.hpp"

namespace MotionPlanner {

/**
 * @brief Library version
 */
constexpr const char* VERSION = "1.0.0";

/**
 * @brief Get library version string
 */
inline const char* version() { return VERSION; }

}  // namespace MotionPlanner
