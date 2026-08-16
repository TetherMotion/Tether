# ReNURBS — Analytical NURBS Representation of TOPP-RA Velocity Profiles

## 0. Status

| Field | Value |
|---|---|
| **Status** | PLANNING — not yet implemented |
| **Scope** | `tether::motion::profile_renurbs` (new component), `MotionPlanner::VelocityProfile` extension, `MotionReplanner::SvgExporter` consumer |
| **Depends on** | `tether::motion::NurbsCurve`, `MotionPlanner::VelocityProfile<T>`, `MotionPlanner::KinematicLimits`, existing `DeviationCertifier` / `CertifiedCurvatureSampler` Lipschitz patterns |
| **Opt-in** | API optional; default OFF. Enabled per-build via `TETHER_ENABLE_RENURBS` and per-plan via `MotionPlanConfig::enableReNURBS` |
| **Target continuity** | v(s): C³/G³, a(s): C²/G², j(s): C¹/G¹, t(s): C³/G³ — *where the source profile permits* (see §5 Edge Cases for the basic-TOPP-RA caveat) |
| **Minimum continuity** | v(s): C¹/G¹, a(s): C⁰/G⁰, j(s): C⁰/G⁰, t(s): C¹/G¹ |

---

## 1. Problem Statement

The current pipeline (see `docs/motion/MotionChain.md` Step 4) produces a
`VelocityProfile<T>` — a tabulated `v(s)` (plus `a(s)`, `j(s)`, `t(s)`) sampled
at `numSamples` points (default 100, often 1000+ for long paths). The
visualization layer (`MotionReplanner::SvgExporter::renderVelocityProfile` /
`renderAccelerationProfile`) renders these as SVG `<polyline>` elements with one
vertex per sample <ref_snippet file="/home/uli/dev/Tether/src/replanner/SvgExporter.cpp" lines="568-606" />.

For shaped velocity curves (jerk-limited TOPP-RA on a path with many curvature
transitions), this means:

- **Thousands of polyline vertices** to represent a single smooth curve → large
  SVG files, slow rendering, no zoom-friendly resolution.
- **No analytical derivative** for the visualization layer — acceleration plots
  are computed by finite differencing the velocity polyline, amplifying noise.
- **No segment-aware structure** — the polyline is a flat list; the
  visualization cannot, e.g., highlight the NURBS piece corresponding to G-code
  line N.

The motion *execution* path (`MotionPlan::evaluateAt`) already works fine from
the sampled profile (linear interpolation between points,
`VelocityProfile::velocityAt` <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/VelocityProfile.hpp" lines="225-243" />).
**This plan does not change execution.** It adds an *optional, parallel*
analytical NURBS representation of the profile, consumed only by visualization
and audit tooling.

---

## 2. Goals & Non-Goals

### Goals

1. **Per-segment NURBS curves** for `v(s)`, `a(s)`, `j(s)`, `t(s)`, one set per
   `MotionSegment` (mapped via the `PathAdapter` segment index → profile sample
   range).
2. **Adaptive construction** — knot density increases where the profile has high
   curvature / rapid constraint switching, so a tight fit is achieved with
   *few* control points on smooth regions.
3. **Constraint preservation** — the NURBS curves never violate the
   `KinematicLimits` that were fed into TOPP-RA: `v_NURBS(s) ≤ v_lim(s)`,
   `|a_NURBS(s)| ≤ a_max(s)`, `|j_NURBS(s)| ≤ j_max(s)` (when jerk-limited).
4. **Interpolation within epsilon** — `|v_NURBS(s_i) − v_sample(s_i)| ≤ ε_v`
   for every sample `s_i` (and analogously for a, j, t). ε is configurable.
5. **Certified (optional)** — a `ProfileConstraintCertifier` produces a
   Lipschitz-based certificate (analogous to `DeviationCertifier` / 
   `CertifiedCurvatureSampler`) proving the constraint-preservation and
   interpolation claims.
6. **Continuity** — within a segment, v/a/j/t NURBS are at least C¹; preferred
   C³. Across segment boundaries, continuity is enforced *up to what the source
   profile supports* (see §5).
7. **API optional** — zero cost when disabled; the sampled `VelocityProfile` is
   unchanged and remains the execution-time source of truth.

### Non-Goals

- Replacing the sampled `VelocityProfile` for execution. `MotionPlan::evaluateAt`
  continues to read the sampled profile.
- Re-running TOPP-RA. The NURBS is a *post-hoc representation* of an already-
  computed profile, not a re-optimization.
- Handling `Dwell` segments (no geometry, no profile samples — out of scope, per
  user direction).
- 2D/3D *path* NURBS — that already exists (`PiecewiseNurbsPath`). This plan is
  strictly about the *scalar* profile curves v(s), a(s), j(s), t(s).

---

## 3. Data Flow & Integration Points

```
                     existing pipeline (unchanged)
   MotionSegment ──▶ SegmentConverter ──▶ PiecewiseNurbsPath ──▶ PathAdapter
          │                                                        │
          ▼                                                        ▼
   IVelocityProfiler::computeProfile(path, feed, ...) ──▶ VelocityProfile<T>  (sampled)
          │                                                        │
          │   ┌──────────── ReNURBS (NEW, optional) ────────────┐ │
          │   │                                                  │ │
          └──▶│  ReNURBSProfileBuilder::build(                  │ │
                  profile, path, limits, config)                 │ │
                  │                                              │ │
                  ▼                                              │ │
              ReNURBSProfile {                                   │ │
                perSegment: vector<ReNURBSSegmentProfile>        │ │
                certificate?: ProfileConstraintCertificate       │ │
              }                                                  │ │
                  │                                              │ │
                  ▼                                              │ │
              SvgExporter / audit tooling (NEW consumers)        │ │
              └──────────────────────────────────────────────────┘ │
                                                                   ▼
                                                          MotionPlan (execution,
                                                           uses sampled profile)
```

### New files

| Path | Purpose |
|---|---|
| `include/tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp` | Data structures: `ReNURBSSegmentProfile`, `ReNURBSProfile` |
| `include/tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp` | The builder class (adaptive B-spline fit + constraint shrink) |
| `include/tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.hpp` | Optional Lipschitz certifier (mirrors `DeviationCertifier`) |
| `include/tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp` | Low-level adaptive B-spline interpolation through samples |
| `src/motion_planner/profile_renurbs/ReNURBSProfileBuilder.cpp` | Implementation |
| `src/motion_planner/profile_renurbs/ProfileConstraintCertifier.cpp` | Implementation |
| `src/motion_planner/profile_renurbs/ProfileSplineFitter.cpp` | Implementation |
| `tests/motion_planner/profile_renurbs/test_ReNURBSProfileBuilder.cpp` | Unit tests (see §7) |
| `tests/motion_planner/profile_renurbs/test_ProfileSplineFitter.cpp` | Unit tests for the fitter |
| `tests/motion_planner/profile_renurbs/test_ProfileConstraintCertifier.cpp` | Unit tests for the certifier |
| `tests/motion_planner/profile_renurbs/test_ReNURBSEdgeCases.cpp` | Edge-case + property tests |
| `tests/motion_planner/profile_renurbs/ReNURBSTestHelpers.hpp` | Shared fixtures: synthetic profiles, limit configs |

### Modified files

| Path | Change |
|---|---|
| `include/tether/motion_planner/MotionPlan.hpp` | Add `std::optional<ReNURBSProfile> renurbsProfile_` + accessor; `MotionPlanConfig::enableReNURBS` flag |
| `include/tether/motion_planner/MotionPlan.hpp` (`MotionPlanBuilder::build`) | After computing the sampled profile, if `config.enableReNURBS`, call `ReNURBSProfileBuilder::build` and attach |
| `include/tether/motion_replanner/SvgExporter.hpp` / `.cpp` | New `SvgPlotType::NurbsVelocityProfile` etc. that render the NURBS curves via adaptive sampling of the *curve* (not the original samples) — see §6 |
| `tests/CMakeLists.txt` | Add `profile_renurbs` test executable (GLOB, mirroring `motion_planner/geometry/`) |
| `CMakeLists.txt` (root) | `option(TETHER_ENABLE_RENURBS "Enable ReNURBS profile representation" ON)` — on by default but inert unless `MotionPlanConfig::enableReNURBS` is set |
| `AGENTS.md` | Append build/test commands for the new target |

---

## 4. Mathematical Construction

### 4.1 Notation

- `s` — arc length along the path (the profile's abscissa).
- `v(s), a(s), j(s), t(s)` — the sampled profile quantities.
- `v_lim(s)` — the TOPP-RA velocity limit curve (min of feed, curvature, axis
  limits). Reconstructed from `VelocityProfilePoint::limitedBy` + the path's
  `CertifiedCurvatureSampler`.
- `a_max(s)` — tangential acceleration limit (path + axis).
- `j_max(s)` — jerk limit (only meaningful for `JerkConstrainedVelocityProfiler`).
- Segment `k` covers `s ∈ [s_k^start, s_k^end]`, mapped from `PathAdapter::getSegment(k).cumulativeArcLength`.

### 4.2 Per-segment B-spline interpolation

For each profile quantity `q ∈ {v, a, j, t}` and each segment `k`:

1. **Collect samples** falling in `[s_k^start, s_k^end]` (include the boundary
   samples from neighbors to ensure inter-segment continuity).

2. **Parameterize** by chord-length: `u_i = (s_i − s_k^start) / (s_k^end − s_k^start) ∈ [0,1]`.

3. **Initial knot vector**: clamped uniform B-spline of degree `p_q`:
   - `p_v = 4` (cubic → C² inside; we want C³ so use `p_v = 5`, quintic → C⁴ inside, more than enough)
   - `p_a = 4` (quintic → C⁴ inside; a(s) from jerk-limited TOPP-RA is C⁰ at switches, so the spline will smooth it — see §5)
   - `p_j = 3` (cubic → C² inside; j(s) from jerk-limited TOPP-RA is piecewise-constant, so the spline approximates — see §5)
   - `p_t = 5` (t(s) is the integral of 1/v, smooth wherever v > 0)
   - Knots: `p_q+1` repetitions of 0, `p_q+1` of 1, interior knots at the
     sample `u_i` (averaged per de Boor / Piegl & Tiller §9.3.1 to avoid
     singular systems).

4. **Solve for control points** via the interpolation system
   `B(u_i) · P = q_i` (banded linear solve, O(n·p²)). This is the standard
   global B-spline interpolation — *not* a local fit — so it passes through
   every sample exactly.

5. **Adaptive refinement loop** (the "adaptive" requirement):
   - Evaluate the spline at a dense test grid (e.g. 10× the sample density).
   - Compute the residual `r(u) = |B(u) − q_interpolated(u)|` where
     `q_interpolated` is the linear interpolation of the *original* samples
     (the ground truth the spline should match between samples).
   - Where `r(u) > ε_q`, insert a knot at `u` (Boehm insertion) and re-solve.
   - Cap total control points at `maxCpPerSegment` (default 64) to bound cost.
   - This mirrors the adaptive refinement in `CertifiedCurvatureSampler`'s grid
     refinement <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp" lines="85-100" />.

### 4.3 Constraint preservation (the "never violated" requirement)

The interpolation above may *overshoot* between samples (Runge phenomenon,
Gibbs-like ringing near discontinuities). For a velocity profile, overshoot
above `v_lim` is a constraint violation. We handle this with a **conservative
shrink** step:

1. After the spline is fit, evaluate `v_NURBS(u)` on the dense test grid.
2. Compute the *local* limit `v_lim(s(u))` (reconstructed — see §4.4).
3. Wherever `v_NURBS(u) > v_lim(s(u)) − safetyMargin`, **project the nearest
   control points downward** so the spline lies at or below
   `v_lim − safetyMargin`. This is a constrained projection:
   - For a B-spline, the spline is a convex combination of control points
     (partition-of-unity property when weights are uniform). So clamping each
     control point `P_i ← min(P_i, v_lim_at_knot_i − safetyMargin)` guarantees
     `v_NURBS(u) ≤ max_i(v_lim_at_knot_i − safetyMargin)` — but this is too
     conservative.
   - **Better**: use the convex-hull property locally. For each span
     `[u_i, u_{i+1})`, the spline lies in the convex hull of `p_q+1` control
     points. Clamp those control points so their convex hull is below the
     *minimum* of `v_lim` over that span (which we bound via the
     `CertifiedCurvatureSampler` Lipschitz bound on `v_lim`). This is tight and
     still certified.
4. Repeat for `a(s)` against `a_max(s)` and `j(s)` against `j_max(s)`.
5. Re-run the interpolation residual check after shrinking — if shrinking
   pushed the spline off a sample by more than `ε_q`, insert a knot near that
   sample and re-fit. This converges because shrinking only ever *lowers* the
   spline, and the samples themselves are feasible (TOPP-RA guarantees
   `v_sample ≤ v_lim`), so the samples are always reachable.

**Key invariant**: the samples are feasible by construction (TOPP-RA enforces
this). The spline passes through the samples (within ε). Between samples, the
convex-hull clamp guarantees the spline never exceeds the limit envelope. So
the constraint is preserved *everywhere*, not just at samples.

### 4.4 Reconstructing the limit curves

The TOPP-RA limit curves are not stored in `VelocityProfile` — only the
`limitedBy` tag per point. We reconstruct them:

- `v_lim(s)` = `min(feedRate, √(a_cent / κ_cert(s)), per-axis limits)` —
  recompute from `PathAdapter::curvatureAtArcLength` (or the
  `CertifiedCurvatureSampler` for the certified upper bound on κ) and the
  `KinematicLimits`. This is exactly what `VelocityProfiler::computeVelocityLimit`
  does <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/VelocityProfile.hpp" lines="508-512" />; we factor it out into a reusable `VelocityLimitCurve` helper.
- `a_max(s)` = `KinematicLimits::maxAccelerationForDirection` evaluated at the
  path tangent at `s`.
- `j_max(s)` = `KinematicLimits::axis.maxJerk` (uniform for now; per-axis
  directional projection is a future extension).

This reconstruction is *deterministic* and matches what the profiler used, so
the constraint-preservation guarantee is against the *same* limits TOPP-RA
checked.

### 4.5 Inter-segment continuity

At a segment boundary `s = s_k^end = s_{k+1}^start`:

- **C⁰**: guaranteed — both segments interpolate the same boundary sample.
- **C¹** (and higher): enforce by sharing derivative boundary conditions. We
  compute the analytical derivative of segment `k`'s spline at its end and use
  it as the clamped derivative condition for segment `k+1`'s spline at its
  start (and vice versa for the backward pass). This is the standard
  "not-a-knot"-adjacent clamped B-spline construction.
- **For v(s) and t(s)**: aim for C³ across boundaries (quintic splines with
  shared position, tangent, and curvature at the boundary).
- **For a(s)**: C¹ across boundaries if the source profile has continuous
  acceleration (jerk-limited TOPP-RA). For basic TOPP-RA, a(s) is discontinuous
  at switching points — we *cannot* make it C¹ without violating the sample
  interpolation. In that case we document C⁰ and optionally split the segment
  at the discontinuity (detected via `limitedBy` tag change) so each sub-piece
  is smooth. See §5.
- **For j(s)**: C⁰ across boundaries (jerk-limited TOPP-RA has piecewise-
  constant jerk with jumps at phase transitions). Same split-at-discontinuity
  strategy.

### 4.6 Why NURBS and not just B-splines

The profile quantities are scalar and non-rational, so plain B-splines suffice
mathematically. We still store them as `NurbsCurve` (degree-p, all-weights-1)
for these reasons:

1. **Reuse the existing infrastructure** — `NurbsCurve::evaluate`,
   `derivative`, `split`, `bezierDecompose`, arc-length machinery all work
   for the weights-all-ones case.
2. **Future rational fits** — if a profile segment is exactly a circular arc
   in (s, v) space (e.g. a pure centripetal-limited region where
   `v = √(a_cent/κ)` with constant κ), a rational quadratic is exact and far
   cheaper than a high-degree polynomial fit. Storing as `NurbsCurve` keeps
   that option open.
3. **Consistency with the path representation** — the whole stack speaks
   `NurbsCurve`; the visualization layer already has `NurbsCurve` rendering
   paths.

The `NurbsCurve` constructor requires `dim ≥ 1` and accepts 1-D curves
(`RVec` of length 1), so scalar profiles fit directly.

---

## 5. Edge Cases

| # | Case | Detection | Handling |
|---|---|---|---|
| E1 | **Empty profile** (0 samples) | `profile.points().empty()` | Return empty `ReNURBSProfile`; log warning. |
| E2 | **Single sample** (1 point) | `points().size() == 1` | Degenerate: a constant NURBS (degree-1, 2 control points, both = the sample value). No derivative info. |
| E3 | **Zero-length segment** (`s_k^end == s_k^start`) | `PathAdapter::getSegment(k).arcLength == 0` | Skip; emit no curve for that segment. |
| E4 | **Segment with 1–2 samples** (very short segment) | sample count < `p_q + 1` | Reduce degree to `min(p_q, n−1)`; if `n == 1`, see E2. |
| E5 | **v(s) = 0 region** (start/end at rest, or a stall) | `velocity == 0` over a range | t(s) has a singularity (`dt/ds = 1/v → ∞`). Split the segment at the zero-crossing; on zero-velocity sub-segments, t(s) is linear in the *dwell-like* pause duration (read from the sample `time` deltas), not `∫ ds/v`. Fit t(s) as a line there. v(s) is fit normally (passes through 0). |
| E6 | **Discontinuous acceleration** (basic TOPP-RA, `ProfilerType::ToppraBasic`) | `limitedBy` tag changes from `ForwardAccel` to `BackwardDecel` (or to `Curvature`) between adjacent samples with a jump in `acceleration` > tol | a(s) cannot be C¹. **Option A (default)**: fit a(s) as C⁰ piecewise — split the segment at the detected discontinuity, fit each sub-piece as a smooth spline, join at C⁰. **Option B (opt-in via config)**: smooth a(s) to C¹ by least-squares relaxation *away from the samples* (no longer interpolates a exactly, only within ε). Document the continuity downgrade in the `ReNURBSSegmentProfile::achievedContinuity` field. |
| E7 | **Discontinuous jerk** (jerk-limited TOPP-RA, `ProfilerType::ToppraJerkConstrained`) | `jerk` field jumps between ±`j_max` and 0 | Same as E6 but for j(s): C⁰ piecewise by default. |
| E8 | **Reversal** (`MotionPlan::isReverse`) | n/a (runtime state, not in profile) | ReNURBS is built from the *forward* profile; reversal is a display-time flip (`s → totalLength − s`). No special build-time handling. |
| E9 | **Feed override active during build** | n/a | ReNURBS is built from the *nominal* profile (override = 1.0). The visualization layer applies the override as a scalar multiply at render time. |
| E10 | **Sample not monotonic in s** (shouldn't happen, but defensive) | `points_[i].arcLength > points_[i+1].arcLength` | Sort a copy; log error. TOPP-RA guarantees monotonicity, so this is a corruption guard. |
| E11 | **Sample not monotonic in t** | `points_[i].time > points_[i+1].time` | Same as E10. |
| E12 | **Adaptive refinement hits `maxCpPerSegment`** | control point count cap reached | Stop refining; report the worst residual in `ReNURBSSegmentProfile::maxResidual`. The certifier (§4.6 of the *certifier*, not this section) will flag the segment as "uncertified — residual budget exhausted" rather than falsely claiming compliance. |
| E13 | **Constraint shrink pushes a control point below 0** (velocity can't be negative) | `P_i < 0` after shrink | Clamp at 0; if this breaks interpolation (a sample is at 0 and the spline must pass through it, fine; if a sample is > 0 and the clamp makes the spline miss it by > ε), insert a knot and re-fit. v ≥ 0 is itself a constraint (no reverse in the profile). |
| E14 | **Segment spans a blend curve** (the path piece is a blend, not an original G-code segment) | `PathAdapter::getSegment(k).sourceRef` is the blend's ref, or the piece came from `PathBlender` | Treat identically — ReNURBS is per *path piece*, not per original G-code line. The `sourceRef` is propagated so the visualization can label it. |
| E15 | **Rational exact fit** (a span where `v(s) = √(a_cent/κ)` with constant κ over the span) | detect by checking if the samples on the span fit a `√`-shape within ε | Optional optimization: emit a degree-2 rational NURBS with the exact `√` weight. Off by default; on via `ReNURBSConfig::allowRationalExactFit`. |

---

## 6. Visualization Integration

### 6.1 New SvgPlotTypes

Add to `SvgPlotType` <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_replanner/SvgExporter.hpp" lines="134-152" />:

- `NurbsVelocityProfile` — render `v_NURBS(s)` vs `t` (or vs `s`) by sampling the
  NURBS curve adaptively (e.g. 32 points per NURBS piece — far fewer than the
  original 1000+ samples, and resolution-independent).
- `NurbsAccelerationProfile`, `NurbsJerkProfile` — analogous.
- `NurbsProfileDashboard` — all four on one SVG with shared time axis.

### 6.2 Rendering

`SvgExporter` gains an overload accepting `const ReNURBSProfile&`. The render
functions:

1. Iterate `perSegment`.
2. For each segment's v-NURBS, call `NurbsCurve::bezierDecompose()` to get
   Bézier pieces, then render each as an SVG `<path>` with cubic Bézier
   commands (`C` / `S`) — *exact* SVG representation of the NURBS piece (since
   a Bézier is a single-span NURBS). This is the key win: the SVG is now
   vector-accurate and tiny.
3. Overlay the original sample points as small circles (optional, for
   verification visual).

### 6.3 Backward compatibility

The existing `SvgPlotType::VelocityProfile` (polyline) is kept. The NURBS
variants are additive. `exportAllPlots` gains the NURBS variants when a
`ReNURBSProfile` is available.

---

## 7. Testing Plan

All tests are Google Test, in `tests/motion_planner/profile_renurbs/`, collected
by `file(GLOB)` (matching the convention in `tests/CMakeLists.txt` for
`motion_planner/geometry/` <ref_snippet file="/home/uli/dev/Tether/tests/CMakeLists.txt" lines="491-496" />). New
test executable: `tether_motion_planner_renurbs_tests`.

### 7.1 `test_ProfileSplineFitter.cpp` — the math core

| Test | What it verifies |
|---|---|
| `InterpolatesConstantSamples` | All samples equal → spline is exactly constant. |
| `InterpolatesLinearSamples` | Samples on a line → spline is exactly that line (degree-1 special case + degree-p both recover it). |
| `InterpolatesAllSamplesWithinEpsilon` | Random smooth samples (sin, polynomial) → `|B(u_i) − q_i| ≤ ε` for all i. |
| `AdaptiveRefinementReducesResidual` | A high-curvature sample set → initial fit residual > ε; after refinement, residual ≤ ε; final CP count < 2× initial. |
| `AdaptiveRefinementRespectsMaxCp` | Cap at 16 CPs on a nasty profile → stops at 16, reports `maxResidual`. |
| `DegreeReductionForFewSamples` | 3 samples, degree 5 requested → degree reduced to 2, still interpolates. |
| `KnotInsertionPreservesInterpolation` | After Boehm insertion + re-solve, all original samples still hit. |
| `ConvexHullClampPreservesSamples` | Samples feasible, limit envelope below some inter-sample region → clamp lowers the spline between samples but samples remain interpolated. |
| `ConvexHullClampNeverExceedsLimit` | Dense grid check: `B(u) ≤ limit(u) − safetyMargin` everywhere after clamp. |
| `ClampedBoundaryDerivatives` | Clamped C¹ at both ends → derivative matches the imposed value to 1e-9. |
| `ThrowsOnNonMonotonicParameter` | `u_i` not increasing → throws `std::invalid_argument`. |
| `ThrowsOnEmptySamples` | Empty input → throws. |
| `ThrowsOnDimensionMismatch` | (N/A for scalar, but test the 1-D RVec path.) |

### 7.2 `test_ReNURBSProfileBuilder.cpp` — end-to-end

| Test | What it verifies |
|---|---|
| `BuildsFromBasicToppraLinearPath` | 1-segment linear path, `ToppraBasic` → 1 segment profile, v-NURBS is C¹, a-NURBS is C⁰ (bang-bang), interpolation within ε. |
| `BuildsFromJerkConstrainedProfiler` | `ToppraJerkConstrained` on a curved path → v C³, a C², j C¹ within each smooth sub-piece. |
| `BuildsFromSCurveProfiler` | `SCurve` → all four curves smooth, C³ achievable. |
| `PerSegmentCountMatchesPathPieces` | 5-segment path → `perSegment.size() == 5`. |
| `SegmentBoundariesAreC0` | `v_NURBS_k(end) == v_NURBS_{k+1}(start)` to 1e-9. |
| `SegmentBoundariesAreC1ForV` | derivative match at boundaries for v (when source profile is smooth). |
| `ConstraintPreservationVelocity` | Dense grid: `v_NURBS(s) ≤ v_lim(s)` everywhere, for a path with a sharp curvature peak. |
| `ConstraintPreservationAcceleration` | Same for `|a_NURBS(s)| ≤ a_max(s)`. |
| `ConstraintPreservationJerk` | Same for `|j_NURBS(s)| ≤ j_max(s)` (jerk-limited profiler only). |
| `ZeroVelocityStartEnd` | v(0) = v(L) = 0 exactly (clamped). |
| `ZeroVelocityMidProfile` | Synthetic profile with v=0 in the middle → t(s) split and linear on the zero region. |
| `TimeCurveMonotonic` | `t_NURBS(s)` strictly increasing (where v > 0). |
| `TimeCurveMatchesIntegral` | `t_NURBS(s_end) − t_NURBS(s_start) ≈ ∫ ds/v` to within ε on a smooth region. |
| `DisabledByDefault` | `MotionPlanConfig` default → `plan.renurbsProfile() == std::nullopt`. |
| `EnabledByConfig` | `config.enableReNURBS = true` → profile is populated. |
| `SourceRefPropagated` | `perSegment[k].sourceRef` matches `PathAdapter::getSegment(k).sourceRef`. |
| `EmptyProfile` | Empty `VelocityProfile` → empty `ReNURBSProfile`, no throw. |
| `SingleSampleProfile` | 1 point → constant curves, no throw. |

### 7.3 `test_ProfileConstraintCertifier.cpp`

| Test | What it verifies |
|---|---|
| `CertifiesCompliantProfile` | A profile built within limits → certificate `compliant == true`, interval width ≤ ε_cert. |
| `RejectsViolatingProfile` | Manually perturb a control point above `v_lim` → certificate `compliant == false`, reports the violating span and the overshoot magnitude. |
| `LipschitzBoundTight` | Certificate upper bound − lower bound ≤ `epsilon` requested. |
| `AccelerationCertification` | Same for a(s) limits. |
| `JerkCertification` | Same for j(s) limits. |
| `InterpolationCertificate` | Certifies `|B(u_i) − q_i| ≤ ε` at all samples (separate from constraint cert). |
| `MaxCpExhaustedFlag` | E12 case → certificate reports `residualBudgetExhausted == true` instead of false-positive compliance. |

### 7.4 `test_ReNURBSEdgeCases.cpp` — edge cases & property tests

Every edge case in §5 gets at least one test. Plus property-based / fuzz tests:

| Test | What it verifies |
|---|---|
| `EdgeCase_E1_EmptyProfile` | E1. |
| `EdgeCase_E2_SingleSample` | E2. |
| `EdgeCase_E3_ZeroLengthSegment` | E3. |
| `EdgeCase_E4_ShortSegmentDegreeReduction` | E4. |
| `EdgeCase_E5_ZeroVelocityRegion` | E5 — t(s) split + linear. |
| `EdgeCase_E6_DiscontinuousAccelBasicToppra` | E6 — a(s) is C⁰, segment split at the switch. |
| `EdgeCase_E7_DiscontinuousJerk` | E7. |
| `EdgeCase_E12_MaxCpExhausted` | E12 — graceful degradation. |
| `EdgeCase_E13_NegativeVelocityClamp` | E13. |
| `EdgeCase_E14_BlendSegment` | E14 — blend piece works. |
| `EdgeCase_E15_RationalExactFit` | E15 — `√`-shape → rational quadratic, off by default. |
| `Fuzz_RandomProfilesConstraintPreserved` | 1000 random profiles (random path, random limits, random feed) → constraint preservation never violated. Property test via `--gtest_filter='*Fuzz*'` (matches the convention in `AGENTS.md`). |
| `Property_MonotonicTime` | For any profile with v > 0 throughout, t(s) is strictly monotonic. |
| `Property_ConvexHullImpliesConstraint` | If all control points ≤ limit, spline ≤ limit (convex-hull property check on random data). |
| `Property_SampleInterpolationHolds` | For any profile, all samples interpolated within ε after refinement. |
| `Regression_RealWorldKlipperPath` | A 200-segment Klipper G-code path → builds without crash, constraint-preserved, CP count per segment < 64. (Use the `test_helpers.hpp` fixture loader.) |

### 7.5 Test helpers (`ReNURBSTestHelpers.hpp`)

- `makeLinearProfile(numSamples, vMax)` — a trapezoidal profile.
- `makeSCurveProfile(numSamples, vMax, jMax)` — a 7-phase S-curve.
- `makeCurvedPathProfile(...)` — profile over a path with a circular arc.
- `makeBangBangProfile(...)` — basic TOPP-RA with a known acceleration discontinuity.
- `assertInterpolatesWithinEpsilon(spline, samples, eps)` — shared assertion.
- `assertConstraintPreserved(spline, limitFn, grid, eps)` — shared assertion.
- `loadKlipperFixture(name)` — loads a real G-code fixture from
  `tests/motion_planner/profile_renurbs/fixtures/`.

### 7.6 Build & run

```bash
cmake -B build -DTETHER_ENABLE_KLIPPER=1 -DTETHER_ENABLE_RENURBS=ON
cmake --build build --target tether_motion_planner_renurbs_tests -j$(nproc)
./build/bin/tests/tether_motion_planner_renurbs_tests
# Fuzz only:
./build/bin/tests/tether_motion_planner_renurbs_tests --gtest_filter='*Fuzz*:*Property*'
```

---

## 8. Configuration API

```cpp
// in MotionPlanConfig<T>
struct ReNURBSConfig {
    bool enabled = false;                 // master switch (also gated by TETHER_ENABLE_RENURBS)

    // Interpolation tolerances (per quantity)
    double epsilonVelocity     = 1e-4;    // mm/s
    double epsilonAcceleration = 1e-3;    // mm/s²
    double epsilonJerk         = 1e-2;    // mm/s³
    double epsilonTime         = 1e-6;    // s

    // Safety margin below the limit envelope (constraint preservation)
    double safetyMarginVelocity     = 1e-4;  // mm/s
    double safetyMarginAcceleration = 1e-3;  // mm/s²
    double safetyMarginJerk         = 1e-2;  // mm/s³

    // Degree (continuity target). p gives C^(p-1) inside a segment.
    int degreeVelocity = 5;   // C⁴ inside
    int degreeAcceleration = 4;
    int degreeJerk = 3;
    int degreeTime = 5;

    // Adaptive refinement caps
    std::size_t maxControlPointsPerSegment = 64;
    std::size_t refinementGridMultiplier = 10;  // test grid = multiplier × samples

    // Discontinuity handling
    bool splitAtDiscontinuities = true;   // E6/E7: split segment into smooth sub-pieces
    bool smoothAccelBasicToppra = false;  // E6 option B: least-squares relax to C¹

    // Optional features
    bool allowRationalExactFit = false;   // E15
    bool certify = true;                  // run ProfileConstraintCertifier
    double certificationEpsilon = 1e-5;   // Lipschitz certificate width
};

// in MotionPlanConfig<T>
ReNURBSConfig renurbs;
```

When `renurbs.enabled == false` (default), `MotionPlanBuilder::build` skips the
ReNURBS step entirely — zero overhead.

---

## 9. Certification (optional, on by default)

`ProfileConstraintCertifier` mirrors the existing Lipschitz-certification
pattern (`DeviationCertifier` <ref_file file="/home/uli/dev/Tether/include/tether/motion_planner/blend/DeviationCertifier.hpp" />, `CertifiedCurvatureSampler` <ref_file file="/home/uli/dev/Tether/include/tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp" />):

### 9.1 What it certifies

For each segment and each quantity `q ∈ {v, a, j, t}`:

1. **Interpolation**: `|q_NURBS(s_i) − q_sample(s_i)| ≤ ε_q` for every sample
   `s_i` in the segment. (Pointwise, exact — no Lipschitz needed.)
2. **Constraint preservation**: `q_NURBS(s) ≤ q_lim(s)` for *all* `s` in the
   segment, not just samples. Uses a Lipschitz bound on `q_NURBS` (B-spline
   derivative bound via control polygon — the same M1-style bound used in
   `CertifiedCurvatureSampler` <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp" lines="26-43" />) and a Lipschitz bound on `q_lim` to
   certify on a finite grid that the inequality holds between grid points.
3. **Continuity**: reports the achieved continuity class (Cᵏ) per segment and
   per boundary, verified by checking derivative jumps at knot boundaries.

### 9.2 Output

```cpp
struct ProfileConstraintCertificate {
    bool compliant = true;
    std::vector<SegmentViolation> violations;  // empty if compliant
    double lipschitzWidth = 0.0;                // achieved certificate width
    bool residualBudgetExhausted = false;       // E12
    std::vector<ContinuityReport> continuity;   // per segment + per boundary
};

struct SegmentViolation {
    std::size_t segmentIndex;
    enum class Quantity { Velocity, Acceleration, Jerk, Time } quantity;
    double arcLength;       // where the violation was detected
    double value;           // the offending NURBS value
    double limit;           // the limit at that s
    double overshoot;       // value − limit
};
```

### 9.3 Failure policy

If `certify == true` and the certificate is non-compliant, `ReNURBSProfileBuilder::build`:
- By default: **throws `ReNURBSCertificationError`** (a `std::runtime_error`
  subclass) with the violation summary. This is the safe default — a
  non-compliant visualization profile is a bug.
- If `ReNURBSConfig::certifyThrowOnFailure == false`: logs the violations via
  `KLIPPER_LOG_ERROR` (or the motion-planner equivalent) and returns the
  profile with the certificate attached, so the caller can inspect it. Intended
  for debugging.

---

## 10. Implementation Phasing

| Phase | Scope | Deliverable |
|---|---|---|
| **P1** | `ProfileSplineFitter` (interpolation + adaptive refinement + convex-hull clamp), unit-tested in isolation. | `test_ProfileSplineFitter.cpp` passing. No `NurbsCurve` dependency yet — pure B-spline math on `std::vector<double>`. |
| **P2** | Wrap as `NurbsCurve` (1-D, weights all 1); `ReNURBSSegmentProfile` + `ReNURBSProfile` structs; `ReNURBSProfileBuilder` skeleton that builds per-segment curves from a `VelocityProfile` + `PathAdapter` (no constraint shrink yet). | `test_ReNURBSProfileBuilder.cpp` interpolation + continuity tests passing. |
| **P3** | Constraint shrink (§4.3) + limit-curve reconstruction (§4.4). | Constraint-preservation tests passing. |
| **P4** | `ProfileConstraintCertifier` (§9). | Certifier tests passing. |
| **P5** | Edge cases E1–E15 (§5) + fuzz/property tests. | `test_ReNURBSEdgeCases.cpp` passing. |
| **P6** | `MotionPlan` / `MotionPlanConfig` integration; `MotionPlanBuilder` wiring; CMake option. | End-to-end: `builder.build(segments, feed)` with `config.renurbs.enabled = true` produces a plan with `renurbsProfile()` populated. |
| **P7** | `SvgExporter` NURBS rendering (§6). | Visual regression: NURBS-rendered velocity profile matches the polyline version within rendering tolerance, at <10% of the SVG file size on a 1000-sample profile. |

Each phase is independently testable and committable. P1–P5 are pure library
work; P6 is the integration; P7 is the visualization payoff.

---

## 11. Open Questions (to resolve before P1)

1. **Limit-curve reconstruction fidelity** — `VelocityProfiler::computeVelocityLimit`
   uses the `CertifiedCurvatureSampler`'s *upper bound* on κ (conservative).
   Reconstructing `v_lim` the same way guarantees the ReNURBS constraint check
   is against the *same or tighter* envelope. Confirm there's no path through
   `VelocityProfiler` that uses pointwise κ instead (the code shows both
   branches <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/VelocityProfile.hpp" lines="495-505" />); the reconstruction must match the branch the profiler actually took. **Resolution**: store the `v_lim` array in `VelocityProfile` at profile-build time (a small additive change to `VelocityProfilePoint` or a parallel vector), so ReNURBS reads the *exact* limits the profiler used rather than reconstructing. This is cleaner than reconstruction and removes the ambiguity. **Action**: add `std::vector<T> velocityLimitCurve` to `VelocityProfile<T>` (populated by all three profilers). ReNURBS reads it directly. (This is a minor, backward-compatible addition — the field is ignored by existing consumers.)

2. **`a_max(s)` storage** — same argument: store the per-sample acceleration
   limit used by the profiler, so ReNURBS doesn't reconstruct. Add
   `accelerationLimit` to `VelocityProfilePoint` (default `+inf` for
   backward compat with hand-built profiles in tests).

3. **Jerk limit storage** — `j_max` is currently uniform per profiler config;
   no per-sample storage needed. Read from `KinematicLimits`.

4. **Should ReNURBS curves be cached on `MotionPlan` across feed-override
   changes?** Yes — the curves are built from the nominal profile and are
   override-invariant. The override is a render-time scalar. Cache them once
   on the plan.

5. **Thread safety** — `NurbsCurve` arc-length caches are not thread-safe
   ("one path per thread" convention <ref_snippet file="/home/uli/dev/Tether/include/tether/motion_planner/geometry/NurbsCurve.hpp" lines="22-24" />). ReNURBS curves inherit this. Document; no mutex needed if we follow the same convention.

---

## 12. Glossary

| Term | Meaning |
|---|---|
| TOPP-RA | Time-Optimal Path Parameterization based on Reachability Analysis — the algorithm family used by `VelocityProfiler` and `JerkConstrainedVelocityProfiler`. |
| `v_lim(s)` | Velocity limit curve: the minimum of feed, centripetal, and per-axis velocity constraints at arc length `s`. |
| Bang-bang | Acceleration switches between `+a_max` and `−a_max` (basic TOPP-RA); acceleration is discontinuous at switching points. |
| Convex-hull property | A B-spline segment lies within the convex hull of its `p+1` defining control points (for uniform weights). Used by the constraint-shrink step. |
| Lipschitz certificate | A bound of the form `|f(x) − f(y)| ≤ L·|x − y|` that lets us certify a property on a finite grid and bound the gap between grid points. The pattern used by `DeviationCertifier` and `CertifiedCurvatureSampler`. |
| Cᵏ / Gᵏ | Parametric (C) vs geometric (G) continuity of order k. For scalar profiles v(s), C and G coincide (1-D, no reparameterization freedom). |
