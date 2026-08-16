# Motion Chain

This document describes the complete motion pipeline in Tether: from
G-code text input through to real-time step execution on the MCU. It is
the top-level entry point for understanding how motion flows through the
system.

---

## Pipeline Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        G-code Text                                  │
│  "G1 X10 Y20 F1500"                                                 │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  1. G-code Parsing                                          (host)   │
│  GCode::Parser → GCode::Block → GCode::Interpreter                  │
│  → MotionSegmentList                                                │
│  Output: list of motion commands (linear, arc, dwell, NURBS)        │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  2. Path Construction                                       (host)   │
│  SegmentConverter → PiecewiseNurbsPath                               │
│  Output: piecewise NURBS path with arc-length parameterization       │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  3. Corner Blending                                         (host)   │
│  PathBlender → BlendSolver → BlendedPath                             │
│  Output: smooth path with G²/G³ blends at corners                    │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  4. Velocity Profiling                                      (host)   │
│  VelocityProfiler::computeProfile() → VelocityProfile               │
│  Output: time-parameterized v(s) profile with accel + jerk           │
│  Choice: ToppraBasic | ToppraJerkConstrained | SCurve | AnalyticalTOPPRA │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  5. Motion Plan                                             (host)   │
│  MotionPlanBuilder::build() → MotionPlan                             │
│  Output: unified plan with evaluateAt(t) query interface             │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  6. Motion Translation                                      (host)   │
│  MotionTranslator::translate() → AxisStepSequence[]                  │
│  Applies: kinematics transform, step rounding                        │
│           + pressure advance / extrusion compensation (if applicable) │
│  Output: per-axis queue_step command sequences                       │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  7. Step Scheduling                                         (MCU)    │
│  StepScheduler::tick() → GPIO step callbacks                         │
│  Output: real-time step pulses to stepper drivers                    │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Step 1: G-code Parsing

**Stage:** Host-side, before motion planning begins.

**Input:** Raw G-code text (from file, serial, or network).

**Output:** `MotionSegmentList` — a list of `MotionSegment` objects,
each representing one motion command (G0/G1 linear, G2/G3 arc, G5
NURBS, G4 dwell).

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `GCode::Parser` | `include/tether/gcode/GCodeParser.hpp` | Tokenizes and parses G-code into `Block` structures |
| `GCode::Interpreter` | `include/tether/gcode/GCodeInterpreter.hpp` | Executes blocks and emits `MotionSegment` objects |
| `GCode::PlanningSegmentBuilder` | `include/tether/gcode/PlanningSegmentBuilder.hpp` | High-level utility: G-code text → segment array |

**What Happens:**

1. The **Lexer** tokenizes the G-code text into words (letter + number
   pairs like `X10`, `F1500`).
2. The **Parser** groups tokens into `Block` structures (one per
   G-code line).
3. The **Interpreter** executes each block:
   - Modal state tracking (current position, feed rate, units, plane)
   - Motion commands become `MotionSegment` objects
   - Non-motion commands (M-codes, settings) are handled inline
4. Each `MotionSegment` carries:
   - Start/end positions (up to 9 axes: X, Y, Z, A, B, C, U, V, W)
   - Geometry type (linear, arc CW/CCW, NURBS, dwell)
   - Feed rate
   - Path mode (exact stop, exact path, blending)
   - Source reference (G-code line number for traceability)

**Traceability:** Every `MotionSegment` retains a `SourceReference`
pointing back to the original G-code line. This reference propagates
through the entire pipeline, so at any point during execution you can
query which G-code line generated the current motion.

---

## Step 2: Path Construction

**Stage:** Host-side, part of motion planning.

**Input:** `MotionSegmentList` (from Step 1).

**Output:** `PiecewiseNurbsPath` — a piecewise NURBS curve with
arc-length parameterization.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `tether::motion::SegmentConverter` | `include/tether/motion_planner/blend/SegmentConverter.hpp` | Converts `MotionSegment` → `NurbsCurve` |
| `tether::motion::PiecewiseNurbsPath` | `include/tether/motion_planner/geometry/PiecewiseNurbsPath.hpp` | Assembles NURBS curves into a piecewise path |
| `MotionPlanner::PathAdapter<Dim,T>` | `include/tether/motion_planner/PathAdapter.hpp` | Wraps the path with the templated planner API |

**What Happens:**

1. **SegmentConverter** converts each `MotionSegment` into a
   `NurbsCurve`:
   - Linear moves → degree-1 NURBS (Bézier line)
   - Arc moves → degree-2 NURBS (exact circular arc representation)
   - NURBS moves → passed through directly
   - Only active axes are extracted (e.g., a move that changes only X
     and Y produces a 2D curve, not a 9D curve)
2. The `NurbsCurve` objects are assembled into a `PiecewiseNurbsPath`,
   which provides:
   - **Arc-length parameterization:** `s → (piece index, local
     parameter)`, computed lazily via a watermark scheme
   - **Geometric evaluation:** position, tangent, curvature at any
     arc length `s`
   - **Certified curvature bounds:** via `CertifiedCurvatureSampler`
     (Lipschitz-bound, lazy per-span)
3. `PathAdapter` wraps the path for the templated motion planner API,
   adding source reference tracking and PH quintic fast-path support.

**Why NURBS?** NURBS provide a unified representation for all geometry
types (lines, arcs, free-form curves) with exact arc-length computation
and certified curvature bounds. This is essential for velocity profiling,
which needs accurate curvature to compute centripetal acceleration limits.

---

## Step 3: Corner Blending

**Stage:** Host-side, part of motion planning.

**Input:** `PiecewiseNurbsPath` (from Step 2) + `BlendSpec` (tolerance,
continuity, curve type).

**Output:** `BlendedPath` — a new piece sequence with smooth blend
curves inserted at corners, plus an audit trail.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `tether::motion::PathBlender` | `include/tether/motion_planner/blend/PathBlender.hpp` | Orchestrates whole-path blending |
| `tether::motion::BlendSolver` | `include/tether/motion_planner/blend/BlendSolver.hpp` | Per-corner blend solving |
| `tether::motion::BlendSpec` | `include/tether/motion_planner/blend/BlendSpec.hpp` | Blend configuration (tolerance, continuity) |
| `tether::motion::BlendCurveBuilder` | `include/tether/motion_planner/blend/BlendCurveBuilder.hpp` | Constructs Bézier blend curves |
| `tether::motion::PHQuinticBlendBuilder` | `include/tether/motion_planner/blend/PHQuinticBlendBuilder.hpp` | Constructs PH quintic blends (fast path) |
| `tether::motion::DeviationCertifier` | `include/tether/motion_planner/blend/DeviationCertifier.hpp` | Certifies blend deviation within tolerance |

**What Happens:**

1. **PathBlender** iterates over all junctions between path pieces.
2. At each junction, **CornerAnalyzer** determines the corner geometry
   (angle, tangent directions, curvature).
3. **BlendSolver** solves for a blend curve that:
   - Connects the two path pieces with the requested continuity
     (G¹, G², or G³)
   - Stays within the specified tolerance (the "cut" at the corner)
   - Is certified by **DeviationCertifier** to not exceed the tolerance
4. The blend curve is inserted between the two path pieces, replacing
   the sharp corner with a smooth transition.
5. **Overlap resolution** (L1/L2) handles cases where adjacent blends
   would overlap, ensuring the path remains valid.
6. The result is a `BlendedPath` with:
   - A new sequence of NURBS pieces (original pieces trimmed + blend
     curves inserted)
   - An audit trail recording every blend decision (which construction,
     what tolerance, why fallback if any)

**Blend Constructions:**

| Construction | Continuity | When Used |
|---|---|---|
| Exact Bézier G² | G² (curvature continuous) | Standard corners |
| Exact Bézier G³ | G³ (curvature derivative continuous) | High-precision corners |
| PH Quintic | G² + closed-form arc length | Performance-critical paths |
| Exact Stop | G⁰ (full stop) | When no blend fits the tolerance |

**Configuration:** The `BlendSpec` controls blending behavior:

```cpp
tether::motion::BlendSpec spec;
spec.mode = tether::motion::PathMode::Blend;
spec.tolerance = 0.1;                    // 0.1 mm corner cut
spec.continuity = tether::motion::Continuity::G2;
spec.maxBlendFraction = 0.25;            // max 25% of segment length
spec.curveType = tether::motion::BlendCurveType::BezierGk;
```

---

## Step 4: Velocity Profiling

**Stage:** Host-side, part of motion planning.

**Input:** `PathAdapter<Dim, T>` (the blended path) + `KinematicLimits`
+ feed rate.

**Output:** `VelocityProfile<T>` — a tabulated v(s) profile with
per-point velocity, acceleration, jerk, and time.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `VelocityProfiler<Dim,T>` | `include/tether/motion_planner/VelocityProfiler.hpp` | Abstract interface |
| `BasicTOPPRA<Dim,T>` | `include/tether/motion_planner/BasicTOPPRA.hpp` | Standard TOPP-RA (no jerk limit) |
| `JerkConstrainedTOPPRA<Dim,T>` | `include/tether/motion_planner/JerkConstrainedTOPPRA.hpp` | Jerk-integrated TOPP-RA |
| `SCurveVelocityProfiler<Dim,T>` | `include/tether/motion_planner/SCurveVelocityProfiler.hpp` | Basic per-piece S-curve |
| `analytical::AnalyticalTOPPRA<Dim,T>` | `include/tether/motion_planner/analytical/AnalyticalTOPPRA.hpp` | Analytical TOPPRA with SSR/Hybrid representations |
| `VelocityProfile<T>` | `include/tether/motion_planner/VelocityProfile.hpp` | Profile data structure + queries |

**What Happens:**

The velocity profiler computes a time-optimal (or near-optimal) velocity
profile along the path, subject to:

- **Feed rate:** `v ≤ feedRate`
- **Curvature (centripetal):** `v ≤ √(a_centripetal / κ)`
- **Per-axis velocity:** `v · |tangent_axis| ≤ maxAxisVelocity`
- **Acceleration:** `|a| ≤ maxPathAcceleration`
- **Jerk** (if jerk-limited profiler): `|j| ≤ maxPathJerk`

The profile is computed by sweeping forward and backward along the path:

1. **Velocity limit curve:** `v_lim(s)` = minimum of all velocity
   constraints at each arc length.
2. **Forward pass:** Maximum velocity reachable from the start,
   respecting acceleration (and jerk, if applicable) limits.
3. **Backward pass:** Maximum velocity that allows stopping by the end.
4. **Final profile:** `v(s) = min(forward, backward, v_lim)`
5. **Time integration:** `t(s) = ∫ ds / v(s)`

**Profiler Choice:** See [Velocity Profiler Selection
Guide](VelocityProfilerSelection.md) for a detailed comparison of the
three profilers and when to choose each.

**Key Decision:**

| Profiler | Use When |
|---|---|
| `ToppraBasic` | Maximum speed, stiff machine, jerk handled downstream |
| `ToppraJerkConstrained` | **Default for 3D printing** — smooth + time-optimal |
| `SCurve` | Simplicity, testing, when time-optimality doesn't matter |
| `AnalyticalTOPPRA` | Certified trajectory with SSR/Hybrid representations, exact sampling |

See [Analytical TOPPRA](AnalyticalTOPPRA.md) for details on the
analytical profiler and its Switching Structure Representation (SSR)
and Hybrid Monotone + Exact Composition representations.

---

## Step 5: Motion Plan

**Stage:** Host-side, final output of the motion planner.

**Input:** `PathAdapter` (geometry) + `VelocityProfile` (timing) +
`KinematicLimits` (constraints).

**Output:** `MotionPlan<Dim, T>` — a unified plan with a query
interface for motion state at any time `t`.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `MotionPlan<Dim,T>` | `include/tether/motion_planner/MotionPlan.hpp` | Unified plan + query interface |
| `MotionPlanBuilder<Dim,T>` | same file | Constructs plan from segments |
| `MotionState<Dim,T>` | same file | Complete state at a point in time |

**What Happens:**

`MotionPlanBuilder::build()` orchestrates Steps 2-4 and assembles the
result into a `MotionPlan`:

```cpp
MotionPlanBuilder3D builder(limits, config, ProfilerType::ToppraJerkConstrained);
auto plan = builder.build(segments, feedRate);
```

The `MotionPlan` provides:

- **`evaluateAt(t)`** → `MotionState`: Complete state at time `t`:
  - Position, velocity, acceleration, jerk (all axes)
  - Path velocity, acceleration, jerk (tangential)
  - Curvature, arc length
  - Source reference (G-code line)
  - Segment index and parameter
- **`positionAt(t)`** → `Vec<Dim,T>`: Faster position-only query
- **`totalDuration()`**: Total trajectory time
- **Feed override:** `setFeedOverride()`, `rampFeedOverride()` for
  real-time speed adjustment
- **Pause/resume:** `pause(t)`, `resume(t)` with state preservation
- **Reverse motion:** `setReverse(true)` for bidirectional traversal

**Design Principle:** `MotionPlan` consumes the velocity profile as-is.
It does NOT perform post-hoc smoothing or finite-difference estimation.
The acceleration and jerk values come directly from the profiler, so
the constraints verified during profiling are preserved exactly.

> **WI-3 (truthful outputs):** For `JerkConstrainedTOPPRA` (post-WI-8),
> the acceleration is the analytic value from the carried (v, a) state,
> and the jerk is computed from the acceleration change over time —
> **not clamped**. The jerk-limited smoothing of the acceleration
> profile ensures $|j| \leq j_{\max}$ by construction; any residual
> (e.g. from numerical noise at switching points) is reported truthfully
> so violations surface in audit tests. For `BasicTOPPRA`, jerk is zero
> (not constrained — theoretically infinite at switching points).

---

## Step 6: Motion Translation

**Stage:** Host-side, bridges motion planner to MCU.

**Input:** `MotionPlan` + per-axis configuration (steps/mm, direction
invert) + kinematics transform + clock parameters.

**Output:** `std::vector<AxisStepSequence>` — per-axis `queue_step`
command sequences ready for the MCU.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `MotionTranslator<Dim,T>` | `include/tether/klipper/motion/MotionTranslator.hpp` | Plan → step sequences |
| `MotionDispatcher` | `include/tether/klipper/motion/MotionDispatcher.hpp` | High-level move dispatch |
| `ExtrusionFlowTracker` | `include/tether/klipper/motion/ExtrusionFlowTracker.hpp` | Extruder flow estimation |

**What Happens:**

1. **Sample the MotionPlan** at regular intervals (typically every
   1-5ms, configurable).
2. **Apply kinematics transform** to convert Cartesian/path-space
   positions to stepper-motor-space positions:
   - Cartesian: direct mapping (X→X-motor, Y→Y-motor, etc.)
   - CoreXY: `A = X+Y`, `B = X-Y`
   - Delta: trigonometric transform to tower positions
   - Rotary delta: different trigonometric transform
   - Polar winch: cable-length transform
3. **Convert positions to step counts:** `steps = position × steps_per_mm`
4. **Group consecutive steps** into `queue_step` commands:
   - `interval`: clock ticks per step
   - `count`: number of steps in this group
   - `add`: interval change per step (for acceleration)
5. **Apply pressure advance / extrusion compensation** to the extruder
   axis, *if applicable* (see below).
6. **Emit per-axis sequences** tagged with the start clock.

**Pressure Advance / Extrusion Compensation (if applicable):**

Pressure advance (PA) is an *optional* extrusion-compensation stage
applied to the extruder (E) axis during step generation. It is **not
part of the core motion pipeline** — it is applied only when all of the
following conditions are met:

1. **Compile-time gate:** Tether is built with
   `-DTETHER_ENABLE_PRESSURE_ADVANCE=ON` (default: ON). When compiled
   out, the `PressureAdvanceConfig` struct and all PA code paths are
   absent from the binary, with zero overhead.
2. **Runtime enable:** `PressureAdvanceConfig::enabled` is `true`
   (default: `false`). This can be toggled at runtime via G-code
   (`M900` / `SET_PRESSURE_ADVANCE`) or the
   `MotionDispatcher::setPressureAdvanceConfig()` API.
3. **Non-zero gain:** The selected model's effective gain is non-zero
   (e.g. `pressureAdvance > 0` for Linear, `powerLawBaseGain > 0` for
   PowerLaw, etc.). With zero gain, PA is a no-op even if enabled.
4. **Extruder axis present:** The move involves the configured extruder
   axis (default: axis index 3 = E). Non-extrusion moves (travel moves)
   are never compensated.

When all conditions are satisfied, the `MotionTranslator` applies a
position offset `δe` to the extruder axis, advancing or retracting the
extruder to compensate for pressure lag in the Bowden tube or hotend.
Three compensation models are supported:

| Model | Formula | When to Use |
|---|---|---|
| Linear (classic) | `δe = PA · v_e` | Standard PA, works for most filaments |
| PowerLaw | `δe = K_base · (v_e · A_f)^n` | Non-Newtonian filaments (PETG, TPU) |
| Cross-WLF | `δe = (βV_m/A_f) · P_LUT(Q, T)` | Temperature-dependent rheology |

**Configuration:**

```cpp
#if TETHER_ENABLE_PRESSURE_ADVANCE
PressureAdvanceConfig pa;
pa.enabled = true;
pa.pressureAdvance = 0.045;       // 45 ms linear PA
pa.smoothTime = 0.02;             // 20 ms smoothing window
pa.extruderAxis = 3;              // E axis
pa.model = ExtrusionCompensationModel::Linear;  // or PowerLaw, CrossWlf

translator.setPressureAdvanceConfig(pa);
#endif
```

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `PressureAdvanceConfig` | `include/tether/klipper/motion/MotionTranslator.hpp` | PA configuration (model, gain, smoothing) |
| `ExtrusionCompensationModel` | same file | Enum: `Linear`, `PowerLaw`, `CrossWlf` |
| `MotionTranslator` | same file | Applies PA offset during step generation |
| `MotionDispatcher` | `include/tether/klipper/motion/MotionDispatcher.hpp` | Runtime PA config updates from G-code |

**Smoothing:** When `smoothTime > 0`, the extruder velocity is
EWMA-smoothed before computing the PA offset. This reduces discontinuities
at the start/end of extrusion moves. The smoothing window should be
≤ the `MotionTranslator` sample interval × number of samples.

**Safety clamp:** `maxCompensation` (default: 0.5 mm) limits the absolute
PA offset to prevent runaway compensation on non-Newtonian models.

---

## Step 7: Step Scheduling

**Stage:** MCU-side, real-time execution.

**Input:** `AxisStepSequence[]` (from Step 6) + clock frequency + GPIO
callbacks.

**Output:** Real-time step pulses to stepper motor drivers.

**Key Classes:**

| Class | File | Role |
|---|---|---|
| `StepScheduler` | `include/tether/klipper/motion/StepScheduler.hpp` | Real-time step scheduling |
| `KlipperDevice` | `include/tether/klipper/device/KlipperDevice.hpp` | Device-side protocol handler |
| `KlippyHost` | `include/tether/klipper/klippy/KlippyHost.hpp` | Host-side MCU communication |

**What Happens:**

1. **KlippyHost** sends `queue_step` commands to the MCU via the
   transport layer (loopback, pipe, or TCP).
2. **KlipperDevice** receives the commands and enqueues them in the
   `StepScheduler`.
3. **StepScheduler** anchors the MCU clock to wall time and schedules
   each step sequence.
4. A periodic timer calls `StepScheduler::tick()`:
   - Converts wall time to MCU clock ticks
   - Fires due steps by calling the registered GPIO callback
   - Each callback toggles a stepper driver pin (step + direction)
5. The scheduler handles:
   - **Clock synchronization:** Periodic clock sync between host and MCU
   - **Buffer management:** Ensures the step buffer doesn't underflow
     (the host must stay ahead of real-time)
   - **Wait/blocking:** `wait()` blocks until all queued steps complete

**Real-Time Constraints:**

- The host must generate step sequences faster than the MCU consumes
  them. If the host falls behind, the step buffer underflows and the
  print fails.
- Typical buffer depth: 100ms of steps (configurable).
- The `MotionTranslator` can pre-generate steps for the entire plan,
  or generate them incrementally during execution.

---

## Extrusion Compensation (Cross-Cutting, if applicable)

Extrusion compensation is not a sequential step in the pipeline but
rather a cross-cutting concern that touches multiple stages. All
extrusion-compensation features are **opt-in** — they apply only when
explicitly enabled and configured. When disabled (the default), the
motion pipeline behaves as a pure Cartesian/kinematics step generator
with no extrusion-specific processing.

```
                    ┌─────────────────────────┐
                    │  ExtrusionFlowTracker    │
                    │  (flow estimation)       │
                    └────────┬────────────────┘
                             │
    ┌────────────────────────┼────────────────────────┐
    │                        │                        │
    ▼                        ▼                        ▼
Step 4: Profiling      Step 6: Translation     Flow-Adaptive Heater
(curvature limits      (pressure advance       (feed-forward temperature
 affect extruder       offset on E axis,       based on instantaneous
 velocity)             if applicable)          flow rate, if enabled)
```

**ExtrusionFlowTracker** (`include/tether/klipper/motion/ExtrusionFlowTracker.hpp`):
- Updated with per-move extruder velocity
- Provides instantaneous and EWMA-smoothed flow rate
- Used by the flow-adaptive heater controller for feed-forward

**Pressure Advance** (in `MotionTranslator`, if applicable):
- Three models: Linear, PowerLaw, Cross-WLF
- Applied as a position offset on the extruder axis
- Compensates for pressure lag in the Bowden tube / hotend
- See [Step 6: Pressure Advance / Extrusion Compensation](#step-6-motion-translation)
  for the full conditional application rules

**Flow-Adaptive Heater** (`include/tether/control/extrusion/`, if enabled):
- Three-state thermal model (heater block → sensor → melt zone)
- Luenberger observer corrects state estimate from thermistor reading
- Feed-forward uses flow rate from `ExtrusionFlowTracker`

**Deconvolution Controllers** (`include/tether/control/extrusion/`, if enabled):
- Four controllers for extrusion feedforward compensation:
  - `LTIFrequencyDomainDeconvolver` — Baseline spectral deconvolution
  - `OverlapAddLPVDeconvolver` — Gain-scheduled overlap-add
  - `ARXLPVInverseFilter` — Time-domain IIR inverse filter
  - `StateSpaceLPVInputEstimator` — State-space input estimation

See `docs/extrusion/` for full documentation.

---

## Complete Example

```cpp
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/gcode/PlanningSegmentBuilder.hpp>
#include <tether/klipper/motion/MotionTranslator.hpp>

using namespace MotionPlanner;
using namespace tether::klipper::motion;

// ── Step 1: G-code Parsing ──
auto segments = GCode::PlanningSegmentBuilder::fromText(
    "G1 X0 Y0 F1500\n"
    "G1 X100 Y0\n"
    "G2 X100 Y100 I0 J50\n"   // arc
    "G1 X0 Y100\n"
    "G1 X0 Y0\n"
);

// ── Steps 2-5: Path + Blending + Profiling + Plan ──
KinematicLimits<2, double> limits;
limits.path.maxPathVelocity = 100.0;       // mm/s
limits.path.maxPathAcceleration = 500.0;   // mm/s²
limits.path.maxPathJerk = 5000.0;          // mm/s³
limits.path.jerkLimitEnabled = true;

MotionPlanConfig<double> config;
MotionPlanBuilder2D builder(limits, config, ProfilerType::ToppraJerkConstrained);
auto plan = builder.build(segments, 100.0);  // 100 mm/s feed rate

// ── Query the plan ──
std::println("Total duration: {:.3f}s", plan.totalDuration());
std::println("Total length: {:.1f}mm", plan.totalLength());

for (double t = 0.0; t <= plan.totalDuration(); t += 0.5) {
    auto state = plan.evaluateAt(t);
    std::println("t={:.2f}s pos=({:.1f},{:.1f}) v={:.1f} a={:.1f} j={:.1f}",
                 t, state.position[0], state.position[1],
                 state.pathVelocity, state.pathAcceleration,
                 state.pathJerk);
}

// ── Step 6: Motion Translation ──
MotionTranslator<2, double>::Config translatorConfig;
translatorConfig.stepsPerMm = {100, 100, 100, 100, 100};  // X, Y, Z, E, ...
translatorConfig.clockFrequency = 32000000;  // 32 MHz MCU clock
translatorConfig.sampleInterval = 0.001;     // 1ms sampling

MotionTranslator<2, double> translator(translatorConfig);

// Pressure advance (if applicable — opt-in, disabled by default)
#if TETHER_ENABLE_PRESSURE_ADVANCE
PressureAdvanceConfig pa;
pa.enabled = true;                // Runtime enable
pa.pressureAdvance = 0.045;       // 45 ms linear PA
pa.smoothTime = 0.02;             // 20 ms smoothing window
pa.extruderAxis = 3;              // E axis
pa.model = ExtrusionCompensationModel::Linear;
translator.setPressureAdvanceConfig(pa);
#endif

auto stepSequences = translator.translate(plan);

std::println("Generated {} step sequences", stepSequences.size());

// ── Step 7: Send to MCU (via KlippyHost) ──
// KlippyHost sends queue_step commands to the device...
// StepScheduler on the device fires step pulses in real-time.
```

---

## Thread Model

The motion chain spans two execution contexts:

```
┌─────────────────────────────────────────────────────────────┐
│  Host (Linux, non-real-time)                                │
│                                                             │
│  Steps 1-6: G-code parsing, path construction, blending,    │
│  velocity profiling, motion plan, motion translation        │
│                                                             │
│  These steps run on the host CPU. They are NOT real-time    │
│  but must stay ahead of the MCU's step buffer.              │
│                                                             │
│  Typical latency: 1-50ms per move (depending on path        │
│  complexity and profiler choice).                           │
└──────────────────────────┬──────────────────────────────────┘
                           │  Transport (pipe/TCP/loopback)
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  MCU (EtherCAT slave / real-time thread)                    │
│                                                             │
│  Step 7: Step scheduling                                    │
│                                                             │
│  Runs in a real-time thread (or on a dedicated MCU).        │
│  Must fire step pulses with microsecond precision.          │
│                                                             │
│  Buffer depth: typically 100ms of pre-generated steps.      │
└─────────────────────────────────────────────────────────────┘
```

**Key constraint:** The host must generate steps faster than the MCU
consumes them. If the host falls behind, the step buffer underflows
and the print fails. The `MotionTranslator` can pre-generate all steps
for a plan, or generate them incrementally.

---

## Replanning

The motion chain supports replanning during execution via the
`motion_replanner` component:

| Class | File | Role |
|---|---|---|
| `ProfileReplanner` | `include/tether/motion_replanner/ProfileReplanner.hpp` | Re-compute velocity profile for a path |
| `OnlineReblender` | `include/tether/motion_replanner/OnlineReblender.hpp` | Re-blend path corners online |

Replanning is used when:
- The feed rate override changes significantly (requiring a new profile)
- A new segment is appended to an in-progress plan
- Corner tolerances are adjusted mid-print

The replanner uses the same `VelocityProfiler` interface, so the
profiler choice applies to replanning as well.

See `docs/MotionReplanner.md` for details.

---

## See Also

| Document | Scope |
|---|---|
| [Velocity Profiler Selection](VelocityProfilerSelection.md) | Choosing between ToppraBasic, ToppraJerkConstrained, and SCurve |
| [Analytical TOPPRA](AnalyticalTOPPRA.md) | Analytical TOPPRA-equivalent profiler with SSR/Hybrid representations |
| [Architecture](Architecture.md) | Motion planner layer architecture and class hierarchy |
| [BlendingAlgorithm.md](BlendingAlgorithm.md) | Corner blending math (M11-M20, T1-T3) |
| [GeometryFoundations.md](GeometryFoundations.md) | NURBS geometry core math |
| [AlgorithmComparison.md](AlgorithmComparison.md) | Design decisions and rejected alternatives |
| [ImplementationGuide.md](ImplementationGuide.md) | How to read and extend the motion planner code |
| [KlipperArchitecture.md](../KlipperArchitecture.md) | Klipper module architecture and threading |
| [MotionReplanner.md](../MotionReplanner.md) | Online replanning |
| [Extrusion docs](../extrusion/) | Pressure advance and non-Newtonian compensation |
