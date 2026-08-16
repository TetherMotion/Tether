# Velocity Profiler Selection Guide

This document explains the velocity profilers available in Tether,
their trade-offs, and when to choose each one.

---

## Overview

Tether provides four velocity profilers, all implementing the
`VelocityProfiler<Dim, T>` interface:

| Profiler | `ProfilerType` | Jerk-limited | Time-optimal | Accel continuity |
|---|---|---|---|---|
| `ParetoTimeEnergyOptimalVelocityPlanner` | `ParetoTimeEnergy` (default) | Yes | Configurable | Yes |
| `BasicTOPPRA` | `ToppraBasic` | No | Yes | No (bang-bang) |
| `JerkConstrainedTOPPRA` | `ToppraJerkConstrained` | Yes | Approx. (subject to jerk + grid) | Yes |
| `SCurveVelocityProfiler` | `SCurve` | Yes | No | Yes |

> **Default:** `ParetoTimeEnergy` is the default profiler used by
> `MotionPlanBuilder` and `MotionDispatcher` when no profiler type is
> specified. It recovers time-optimal behavior when `w_a = 0` and smoothly
> trades time for energy as `w_a` increases. See
> `docs/motion/ParetoTimeEnergyOptimal.md` for the full derivation.

> **Note (WI-8):** `JerkConstrainedTOPPRA` was rewritten to carry
> acceleration as state in both passes (Option B). The total time is now
> approximately independent of `numSamples` (previously it grew with the
> sample count because each sample paid a full jerk-ramp-up + ramp-down
> cost). See `docs/motion/ToppraDerivation.md` (T.5b, T.6) for the
> derivation.

All three produce a `VelocityProfile<T>` — a tabulated v(s) profile with
per-point velocity, acceleration, jerk, and time. `MotionPlan` consumes
this profile directly; it does not perform post-hoc smoothing or
finite-difference estimation.

---

## 1. `BasicTOPPRA` (Standard TOPP-RA)

**`ProfilerType::ToppraBasic`**

### Algorithm

Second-order Time-Optimal Path Parameterization considering Acceleration
constraints (TOPP-RA). The algorithm computes the time-optimal velocity
profile along a path subject to:

- Maximum velocity (feed rate, per-axis limits, curvature-based limits)
- Maximum acceleration (path-level and per-axis)

The profile is computed in two sweeps:

1. **Forward pass:** $v^2 = v_0^2 + 2 \cdot a_{\max} \cdot \Delta s$ — maximum velocity reachable
   from the start, accelerating at $a_{\max}$.
2. **Backward pass:** $v^2 = v_1^2 + 2 \cdot a_{\max} \cdot \Delta s$ — maximum velocity that
   allows stopping by the end, decelerating at $a_{\max}$.
3. **Final profile:** $v(s) = \min(v_{\text{fwd}}, v_{\text{bwd}}, v_{\text{lim}}(s))$

The velocity limit curve $v_{\text{lim}}(s)$ incorporates:
- Feed rate
- Curvature: $v \leq \sqrt{a_{\text{cent}} / \kappa}$
- Per-axis velocity limits projected onto the path tangent

### Properties

- **Time-optimal:** Produces the fastest possible trajectory for the
  given constraints. No other profiler can be faster.
- **Bang-bang acceleration:** Acceleration switches instantaneously
  between $+a_{\max}$ and $-a_{\max}$ at constraint switching points. This
  means **jerk is theoretically infinite** at these switching points.
- **Acceleration discontinuity:** The acceleration profile has step
  changes. This can cause mechanical vibration, ringing, and excitation
  of resonances in the printer/CNC machine.
- **No jerk field:** The `jerk` field in the velocity profile points is
  always zero (jerk is not constrained).

### When to Choose

- **Rapid prototyping / simulation:** When you need the fastest possible
  trajectory and don't care about mechanical smoothness.
- **Stiff machines:** CNC mills with heavy frames and ball screws can
  tolerate bang-bang acceleration better than lightweight 3D printers.
- **Benchmarking:** Use as the time-optimal baseline to measure how much
  time the jerk-limited profiler costs.
- **When jerk is handled downstream:** If the MCU firmware applies its
  own jerk limiting (e.g., Klipper's `max_jerk` config), the host-side
  profile doesn't need to be jerk-limited.

### When NOT to Choose

- **3D printers with lightweight frames:** Bang-bang acceleration causes
  ringing, ghosting, and layer shifting.
- **Extrusion systems:** Discontinuous acceleration causes pressure
  spikes in the extruder, leading to inconsistent extrusion.
- **When the downstream consumer expects continuous acceleration:** Some
  step generators and kinematics transforms assume smooth acceleration.

### Example

```cpp
using namespace MotionPlanner;

KinematicLimits<3, double> limits;
limits.path.maxPathVelocity = 100.0;       // mm/s
limits.path.maxPathAcceleration = 500.0;   // mm/s²

// Option A: via MotionPlanBuilder
MotionPlanBuilder3D builder(limits, {}, ProfilerType::ToppraBasic);
auto plan = builder.build(segments, feedRate);

// Option B: direct
BasicTOPPRA<3, double> profiler(limits);
auto profile = profiler.computeProfile(path, feedRate);
```

---

## 2. `JerkConstrainedTOPPRA` (Jerk-Integrated TOPP-RA)

**`ProfilerType::ToppraJerkConstrained`**

### Algorithm

Third-order TOPP-RA with jerk as a first-class constraint inside the
optimizer. This is the **recommended profiler for most applications**.

The algorithm extends the basic TOPP-RA by replacing the 2nd-order
kinematic equation $v^2 = v_0^2 + 2 \cdot a \cdot \Delta s$ with the
**state-aware** jerk-limited distance function from `SCurveProfile`
(WI-8 Option B):

$$
\Delta s = \text{computeAccelDistanceWithState}(v_0, a_0, v_1, a_{\max}, j_{\max})
$$

This function computes the exact arc length needed to change velocity
from $v_0$ to $v_1$ starting from acceleration $a_0$, accounting for
the finite time required to ramp acceleration up and down (jerk-bang-bang
control). The acceleration is carried as state in both the forward and
backward passes, so the profile does NOT force $a = 0$ at every sample
point (unlike the pre-WI-8 implementation).

The forward and backward passes use Newton's method with bisection
fallback (`maxVelocityAfterDistance`, WI-P1) to find the maximum
velocity reachable within the available distance, subject to jerk
limits. Newton converges in 2–4 iterations (vs. 60 for the previous
binary search).

### Properties

- **Approximately time-optimal subject to jerk constraint:** Produces a
  fast, feasible trajectory that respects both acceleration AND jerk
  limits. The state-carrying implementation (WI-8 Option B) makes the
  total time approximately independent of `numSamples` (the pre-WI-8
  implementation's time grew with `numSamples` because each sample
  paid a full jerk-ramp-up + jerk-ramp-down cost).
- **Continuous acceleration:** Acceleration ramps smoothly between
  values. No step changes at switching points (jerk-limited smoothing
  pass enforces $|\Delta a / \Delta t| \leq j_{\max}$).
- **Bounded jerk:** $|\text{jerk}(t)| \leq j_{\max}$ everywhere, by
  construction. The jerk is computed from the acceleration change over
  time and reported truthfully (WI-3: not clamped).
- **All constraints verified:** Velocity, acceleration, curvature,
  per-axis velocity/acceleration/jerk limits (WI-2), and junction
  velocity at tangent discontinuities (WI-4) are all checked. The
  output is guaranteed feasible.
- **No post-hoc smoothing:** Jerk is constrained *inside* the optimizer,
  not filtered afterward. This preserves both feasibility and optimality.

### When to Choose

- **3D printers:** Smooth acceleration reduces ringing, ghosting, and
  mechanical stress. Use `ParetoTimeEnergy` (the default) with a small
  `w_a` for a near-time-optimal but smoother profile, or
  `ToppraJerkConstrained` for strict jerk-limited time-optimality.
- **Extrusion systems:** Continuous acceleration means smooth pressure
  changes in the extruder, leading to consistent extrusion.
- **When mechanical smoothness matters:** Any application where
  vibration, resonance, or mechanical wear is a concern.
- **When the downstream consumer needs jerk data:** The profile's `jerk`
  field is populated and can be used for feedforward control or
  pressure advance compensation.

### When NOT to Choose

- **When maximum speed is the only priority:** Use `ToppraBasic` instead,
  or `ParetoTimeEnergy` with `w_a = 0` (which degenerates toward the
  time-optimal solution while still producing jerk-bounded arcs).
- **When jerk limits are not configured:** If `jerkLimitEnabled` is false
  or `maxPathJerk` is zero, the profiler automatically falls back to
  basic TOPP-RA behavior.
- **When simplicity is more important than optimality:** The
  `SCurveVelocityProfiler` is simpler to reason about (though not
  time-optimal).

### Performance Characteristics

- **Compute time:** $O(N \times \log(V_{\max}/\varepsilon))$ per pass, where $N$ is the number
  of samples and the log factor comes from binary search in
  `maxVelocityAfterDistance`. Typically 2-5× slower than basic TOPP-RA
  due to the binary search, but still fast (< 10ms for 1000 samples).
- **Memory:** $O(N)$ — same as basic TOPP-RA.
- **Trajectory time:** 5-15% slower than basic TOPP-RA, depending on
  path geometry and jerk/acceleration ratio.

### Example

```cpp
using namespace MotionPlanner;

KinematicLimits<3, double> limits;
limits.path.maxPathVelocity = 100.0;       // mm/s
limits.path.maxPathAcceleration = 500.0;   // mm/s²
limits.path.maxPathJerk = 5000.0;          // mm/s³
limits.path.jerkLimitEnabled = true;

// Option A: via MotionPlanBuilder (recommended)
MotionPlanBuilder3D builder(limits, {}, ProfilerType::ToppraJerkConstrained);
auto plan = builder.build(segments, feedRate);

// Option B: direct
JerkConstrainedTOPPRA<3, double> profiler(limits);
auto profile = profiler.computeProfile(path, feedRate);

// Option C: custom profiler instance
auto custom = std::make_unique<JerkConstrainedTOPPRA<3, double>>(limits);
MotionPlanBuilder3D builder2(std::move(custom), limits);
auto plan2 = builder2.build(segments, feedRate);
```

---

## 3. `SCurveVelocityProfiler` (Basic S-curve)

**`ProfilerType::SCurve`**

### Algorithm

Per-piece 7-phase S-curve profiles. The path is divided into pieces
(one per segment), and each piece gets an independent S-curve profile
with jerk-limited transitions.

1. For each path piece, compute a cruise velocity limited by curvature
   ($v = \sqrt{a_{\text{cent}} / \kappa}$) and the path-level max velocity.
2. Build a sequence of S-curve profiles with velocity continuity:
   - Piece 0 starts at velocity 0 (rest).
   - Each piece's exit velocity is $\min(v_{\text{cruise,this}}, v_{\text{cruise,next}})$.
   - The last piece ends at velocity 0 (rest).
3. Sample the S-curve profiles at uniform arc length intervals.

### Properties

- **Jerk-limited:** Each piece uses a 7-phase S-curve (jerk ramp-up,
  constant accel, jerk ramp-down, cruise, jerk ramp-down, constant
  decel, jerk ramp-up). Jerk is bounded by $j_{\max}$ by construction.
- **Continuous acceleration:** Within each piece, acceleration is
  continuous. At piece boundaries, velocity is continuous but
  acceleration may have small discontinuities.
- **NOT time-optimal:** The S-curve approach does not consider the
  global velocity limit curve. It uses per-piece cruise velocities
  based on midpoint curvature, which is conservative. The resulting
  trajectory is typically 10-30% slower than jerk-limited TOPP-RA.
- **Simple:** The algorithm is easy to understand and verify. Each
  piece is an independent S-curve; there's no global optimization.

### When to Choose

- **When simplicity matters:** The S-curve approach is well-understood
  and easy to reason about. If you need to manually verify the profile
  or debug issues, this is the easiest to inspect.
- **When time-optimality is not important:** For applications where
  smoothness matters but speed doesn't (e.g., teaching robots, demo
  trajectories).
- **For testing and validation:** Use as a reference to validate the
  jerk-limited TOPP-RA profiler. Both should produce feasible
  trajectories; the TOPP-RA version should be faster.
- **When you need per-piece S-curve phase information:** The S-curve
  profiler exposes the 7-phase structure (jerk ramp, constant accel,
  cruise, etc.) which can be useful for diagnostics.

### When NOT to Choose

- **When time-optimality matters:** Use `ToppraJerkConstrained` instead.
  It's 10-30% faster for the same constraints.
- **For production 3D printing:** The TOPP-RA profilers are better
  optimized for complex paths with varying curvature.
- **When you need global constraint verification:** The S-curve
  profiler only checks curvature at piece midpoints, not at every
  sample point.

### Limitations

- **Per-axis jerk limits at blend boundaries are not enforced.** Only
  the path-level jerk limit is respected. Multi-axis coordinated jerk
  limiting is a future enhancement.
- **Curvature is sampled at piece midpoints only.** This is
  conservative but may miss curvature peaks within a piece.
- **Does not use the certified curvature sampler.** The TOPP-RA
  profilers use `CertifiedCurvatureSampler` for guaranteed curvature
  bounds; the S-curve profiler uses pointwise curvature.

### Example

```cpp
using namespace MotionPlanner;

KinematicLimits<3, double> limits;
limits.path.maxPathVelocity = 100.0;
limits.path.maxPathAcceleration = 500.0;
limits.path.maxPathJerk = 5000.0;
limits.path.jerkLimitEnabled = true;

MotionPlanBuilder3D builder(limits, {}, ProfilerType::SCurve);
auto plan = builder.build(segments, feedRate);
```

---

## Decision Matrix

| Criterion | ToppraBasic | ToppraJerkConstrained | SCurve |
|---|---|---|---|
| **Time-optimality** | Best (baseline) | Good (5-15% slower) | Poor (10-30% slower) |
| **Jerk bounded** | No | Yes | Yes |
| **Acceleration continuous** | No | Yes | Mostly (per-piece) |
| **Compute time** | Fastest | Moderate (2-5× slower) | Fast |
| **Global constraint verification** | Yes | Yes | No (midpoint only) |
| **Extrusion smoothness** | Poor | Best | Good |
| **Mechanical vibration** | High | Low | Low |
| **Simplicity** | Moderate | Complex | Simple |
| **Jerk field populated** | No | Yes | Yes |
| **Recommended for 3D printing** | No | **Yes** | Acceptable |
| **Recommended for CNC milling** | Yes (stiff machines) | Yes | Acceptable |
| **Recommended for simulation** | Yes | No (unnecessary) | No |

---

## Configuration

All three profilers use the same `KinematicLimits<Dim, T>` structure:

```cpp
template<size_t NumAxes, typename T>
struct KinematicLimits {
    PathLimits<T> path;
    AxisLimits<NumAxes, T> axis;
};

struct PathLimits<T> {
    T maxPathVelocity = 100.0;              // mm/s
    T maxPathAcceleration = 500.0;          // mm/s²
    T maxPathJerk = 5000.0;                 // mm/s³
    T maxCentripetalAcceleration = 500.0;   // mm/s²
    bool jerkLimitEnabled = true;
};

struct AxisLimits<NumAxes, T> {
    std::array<T, NumAxes> maxVelocity;
    std::array<T, NumAxes> maxAcceleration;
    std::array<T, NumAxes> maxJerk;
    bool jerkLimitEnabled = true;
};
```

### Jerk Limit Configuration

- If `jerkLimitEnabled` is `false` or `maxPathJerk` is `0`, the
  `JerkConstrainedTOPPRA` automatically falls back to basic
  TOPP-RA behavior.
- The `SCurveVelocityProfiler` requires a valid jerk limit; if not
  provided, it returns an empty profile.
- The `BasicTOPPRA` ignores jerk limits entirely.

### Typical Values for 3D Printers

| Parameter | Typical Range | Notes |
|---|---|---|
| `maxPathVelocity` | 50-200 mm/s | Feed rate; often set per-move by G-code |
| `maxPathAcceleration` | 500-5000 mm/s² | Higher = faster but more vibration |
| `maxPathJerk` | 1000-10000 mm/s³ | Lower = smoother but slower |
| `maxCentripetalAcceleration` | 500-2000 mm/s² | Limits cornering speed |

### Tuning Guidance

1. **Start with conservative values** and increase until mechanical
   issues appear (ringing, skipping steps, layer shifting).
2. **Jerk/acceleration ratio matters:** $j_{\max} / a_{\max}$ determines the
   S-curve transition time. A ratio of 10 (e.g., $a=500$, $j=5000$) gives
   a 0.1s transition; a ratio of 5 gives a 0.2s transition.
3. **Higher jerk = faster but less smooth.** Lower jerk = smoother but
   slower. The optimal value depends on the machine's mechanical
   resonance characteristics.
4. **Measure the time cost:** Compare `plan.totalDuration()` between
   `ToppraBasic` and `ToppraJerkConstrained` to quantify the cost of jerk
   limiting for your specific paths.

---

## API Reference

### Selecting a Profiler via MotionPlanBuilder

```cpp
// Via enum (most common)
MotionPlanBuilder3D builder(limits, config, ProfilerType::ToppraJerkConstrained);
auto plan = builder.build(segments, feedRate);

// Via custom instance (for advanced configuration)
auto profiler = std::make_unique<JerkConstrainedTOPPRA<3, double>>(limits);
MotionPlanBuilder3D builder(std::move(profiler), limits, config);
auto plan = builder.build(segments, feedRate);
```

### Using a Profiler Directly

```cpp
// All profilers implement VelocityProfiler
std::unique_ptr<VelocityProfiler<3, double>> profiler =
    std::make_unique<JerkConstrainedTOPPRA<3, double>>(limits);

auto profile = profiler->computeProfile(path, feedRate);

// Query the profile
for (const auto& pt : profile.points()) {
    std::println("s={:.1f} v={:.1f} a={:.1f} j={:.1f} t={:.3f}",
                 pt.arcLength, pt.velocity, pt.acceleration,
                 pt.jerk, pt.time);
}

// Check profiler metadata
std::println("Profiler: {} (type={})",
             profiler->name(),
             static_cast<int>(profiler->type()));
```

### Querying a MotionPlan

```cpp
auto plan = builder.build(segments, feedRate);

// Evaluate complete state at time t
auto state = plan.evaluateAt(1.5);  // t=1.5s
std::println("Position: ({:.2f}, {:.2f}, {:.2f})",
             state.position[0], state.position[1], state.position[2]);
std::println("Path velocity: {:.2f} mm/s", state.pathVelocity);
std::println("Path acceleration: {:.2f} mm/s²", state.pathAcceleration);
std::println("Path jerk: {:.2f} mm/s³", state.pathJerk);

// Position-only query (faster)
auto pos = plan.positionAt(1.5);

// Total duration
std::println("Total time: {:.3f}s", plan.totalDuration());
```

---

## Internal Architecture

### Class Hierarchy

```
VelocityProfiler<Dim, T>              (abstract interface)
    ├── BasicTOPPRA<Dim, T>               (ToppraBasic)
    ├── JerkConstrainedTOPPRA<Dim, T>     (ToppraJerkConstrained)
    └── SCurveVelocityProfiler<Dim, T>    (SCurve)
```

### Key Difference: 2nd-order vs 3rd-order

The fundamental difference between the profilers is the order of the
optimization:

- **2nd-order (ToppraBasic):** State = $(s, \dot{s})$. Control = $\ddot{s}$ (unbounded
  jerk). The optimizer can switch acceleration instantaneously.
- **3rd-order (ToppraJerkConstrained):** State = $(s, \dot{s}, \ddot{s})$. Control = $\dddot{s}$
  (bounded jerk). The optimizer must ramp acceleration smoothly.

The 3rd-order formulation is what produces continuous acceleration.
The `SCurveProfile::computeAccelDistance()` function is the key
building block — it computes the exact distance needed for a
jerk-limited velocity change, replacing the 2nd-order
$v^2 = v_0^2 + 2 \cdot a \cdot \Delta s$.

### Why Not Post-Hoc Smoothing?

An earlier version of Tether applied S-curve smoothing *after* the
TOPP-RA optimization. This was removed because:

1. **Breaks feasibility:** The smoothed trajectory was never checked
   against the curvature/centripetal/per-axis constraints that TOPP-RA
   verified. The smoothed profile could violate these constraints.
2. **Breaks optimality:** The smoothed trajectory is neither
   time-optimal (TOPP-RA was, but smoothing added time) nor
   jerk-optimal (the smoothing was applied after the fact).
3. **Desynchronizes extrusion:** Any downstream consumer (extrusion
   compensation, step generation) that derived timing from the TOPP-RA
   profile would be desynchronized by the post-hoc smoothing.

The correct approach — implemented in `JerkConstrainedTOPPRA` —
is to integrate jerk as a constraint *inside* the optimizer.

---

## See Also

- [Motion Chain](MotionChain.md) — Top-level pipeline from G-code to step execution
- [Architecture](Architecture.md) — Motion planner layer architecture
- [BlendingAlgorithm.md](BlendingAlgorithm.md) — Corner blending math
- `VelocityProfiler.hpp` — Abstract interface definition
- `BasicTOPPRA.hpp` — Basic 2nd-order TOPP-RA implementation
- `JerkConstrainedTOPPRA.hpp` — Jerk-constrained 3rd-order TOPP-RA implementation
- `SCurveProfile.hpp` — 7-phase S-curve and jerk-limited distance functions
