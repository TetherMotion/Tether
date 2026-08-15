# Algorithm Comparison

This document analyzes the alternatives that were considered for the
corner-blending problem and explains why each was rejected (or adopted
as an opt-in fast path). References [R1]-[R4] are in `MotionPlan.md`.

---

## 1. The problem, restated

Given two path pieces meeting at a vertex $V$ with turning angle
$\theta \in (0, \pi)$, replace a neighborhood of $V$ with a smooth
blend curve that:
- Matches the neighbors with $G^k$ continuity ($k \ge 2$).
- Stays within a signed tolerance $\delta$ of the original path.
- Works in N dimensions (2, 3, 5 axes).
- Has a closed-form or certified construction (no iterative solves
  in the hot path).

---

## 2. Rejected alternatives

### 2.1 Circular-arc blend

**What:** replace the corner with a circular arc tangent to both
neighbors. The arc is determined by the tolerance $\delta$ and the
angle $\theta$.

**Why not the default:**
- **$G^1$ only.** A circular arc has constant curvature; the neighbors
  (lines) have zero curvature. At the junction, curvature jumps from
  0 to $1/R$ — a $G^1$ discontinuity that causes a centripetal
  acceleration step $v^2/R$.
- **N-D awkward.** The arc lives in the corner plane; lifting to 5
  axes requires separate handling of the extra axes.
- **No deviation control for ears.** Negative tolerance (outside
  bulge) is not expressible as a single tangent arc.

**Where it appears:** the old `CornerBlending.hpp` used circular arcs
as the default. It was rejected because G64 P tolerances are tight
enough that $G^1$ steps cause visible marks at high feedrates.

### 2.2 Biarc

**What:** two circular arcs joined with $G^2$ continuity at their
internal junction.

**Why not:**
- **Still $G^1$ at the neighbors.** The biarc's endpoints are tangent
  to the neighbors but curvature-mismatched (same problem as 2.1).
- **Two solutions.** The biarc has a free parameter (the junction
  point), giving a family of solutions with no clear selection
  criterion.
- **No closed-form deviation.** The Hausdorff distance from the biarc
  to the corner requires iterative computation.

### 2.3 Clothoid (Euler spiral)

**What:** a curve whose curvature varies linearly with arc length,
giving $G^2$ continuity with straight neighbors.

**Why not:**
- **No closed form.** The clothoid is defined by Fresnel integrals
  $\int \cos(t^2)\,dt$, $\int \sin(t^2)\,dt$ — no closed-form
  evaluation. Real-time CNC requires closed-form or certified
  evaluation.
- **Curved neighbors.** The clothoid matches straight neighbors
  exactly, but for arc/spline neighbors with nonzero curvature, the
  curvature profile must be generalized (a "generalized clothoid"),
  losing the closed form entirely.
- **N-D.** The clothoid is inherently 2D; lifting to 5 axes requires
  separate extra-axis interpolation.

**Reference:** [R1] discusses clothoid blends for highway design; the
CNC adaptation is in [R2] but notes the evaluation cost.

### 2.4 PH curves as the DEFAULT (not opt-in)

**What:** use Pythagorean-hodograph quintic curves as the default
blend, exploiting their closed-form arc length and curvature.

**Why not the default (but adopted as D6 opt-in fast path):**
- **$G^1$-only boundaries.** A PH quintic constrained by endpoint
  positions and tangents (Hermite data) has no remaining DOF to match
  boundary curvature. The blend is $G^1$ with the neighbors, not
  $G^2$ — a centripetal acceleration step $v^2 \Delta\kappa$ appears
  at the blend boundaries.
- **Four-solution ambiguity.** The PH Hermite construction produces 8
  candidates (4 sign pairs × 2 for $\omega_1$); selecting the "right"
  one by sign convention is a common bug in the literature. The
  correct approach is to certify all non-degenerate candidates and
  keep the best — but this is more work than the exact Bézier path.
- **No jounce matching.** $G^3$ continuity is not achievable with PH
  quintics at all (the degree is too low for 8 boundary conditions).

**Why adopted as opt-in (D6):** for high-feedrate roughing passes
where $G^1$ steps are acceptable, the closed-form arc length and
curvature enable real-time interpolation without quadrature. The PH
path goes through the same `DeviationCertifier` acceptance loop
(T2/T3), so the geometric tolerance guarantee is NOT traded away.
See `BlendingAlgorithm.md` §9 for the full construction.

### 2.5 Global B-spline fairing

**What:** instead of per-corner blends, re-fit the entire path as a
single high-degree B-spline that approximates the original within
tolerance.

**Why not:**
- **Global coupling.** A change at one corner affects the entire
  spline, making interactive editing impossible.
- **No certified deviation.** The approximation error is
  global and hard to certify tightly; local blends give per-corner
  certificates.
- **Memory.** A 100k-segment path becomes a single B-spline with
  ~100k control points — unwieldy.
- **Source traceability.** The one-to-one mapping from G-code lines
  to path pieces is lost; debugging is harder.

### 2.6 Time-domain FIR blending (LinuxCNC style)

**What:** blend in the time domain by low-pass filtering the
velocity command, rather than in the geometric domain.

**Why not:**
- **No geometric tolerance.** The blending happens in velocity space;
  the geometric deviation is a side effect, not a controlled
  quantity. G64 P is a geometric tolerance, not a time constant.
- **Path departure.** The filtered trajectory can depart arbitrarily
  far from the original path at high speeds — the opposite of the
  certified-deviation guarantee.
- **No G64 Q.** The naive CAM tolerance has no time-domain analog.

### 2.7 Extended-tangent "ears" (teardrop overtravel)

**What:** for negative tolerance (outside mode), extend the tangents
past the vertex so the blend loops outward, forming a teardrop.

**Why not (rejected in favor of M20):**
- **Overtravel.** The extension requires the machine to move past
  the vertex, which may violate machine limits or collide with
  fixturing.
- **No closed form.** The extension length is found iteratively.
- **(M20) is better.** The augmented-curvature construction
  (`BlendingAlgorithm.md` §8) achieves the same ear shape without
  overtravel — the blend stays within the trim region, bulging
  outward via the curvature augmentation term.

### 2.8 The old scoring search

**What:** the old `PathBuilder` scored candidate blends by a weighted
sum of deviation, continuity error, and length, then picked the
best-scoring candidate.

**Why not:**
- **No guarantee.** The scoring weights are magic numbers; the
  "best" candidate may violate the tolerance.
- **Two inconsistent radius formulas.** The old code had two
  different formulas for the blend radius (one in `CornerBlending`,
  one in `PathBuilder`), giving different results for the same
  input — a case study in why ad-hoc scoring is fragile.
- **(M15) is better.** The bisection solver finds the speed that
  gives certified deviation $\le |\text{tol}|$ directly, with no
  scoring and no magic weights.

### 2.9 Parameter-fraction trimming

**What:** instead of trimming by arc length, trim by parameter
fraction (e.g. remove the first 10% of each piece's parameter
range).

**Why not:**
- **Parameter ≠ arc length.** For rational NURBS, the parameter
  spacing is non-uniform; a 10% parameter trim removes an unknown
  arc length. The deviation depends on arc length, not parameter.
- **Non-transferable.** The trim fraction depends on the piece's
  parameterization, not its geometry. Re-parameterizing a piece
  changes the trim — fragile.
- **(M15) trims by arc length** directly, giving a geometrically
  meaningful trim that is invariant under re-parameterization.

---

## 3. The chosen approach: exact Bézier + certified deviation

The default blend is an exact Bézier quintic (M11) or septic (M12)
with:
- **Closed-form control points** from the boundary conditions.
- **$G^2$ or $G^3$ continuity** with the neighbors (T1).
- **Certified Hausdorff deviation** via Lipschitz sampling (M10/M14).
- **Bisection on speed** to hit the tolerance (M15).
- **N-D via corner-plane projection** (M13).

This is the simplest construction that satisfies all the requirements.
The PH quintic is offered as an opt-in fast path for cases where
$G^1$ continuity is acceptable and closed-form arc length is needed
for real-time interpolation.

---

## 4. Summary table

| Alternative | Continuity | N-D | Deviation bound | Closed form | Status |
|---|---|---|---|---|---|
| Circular arc | $G^1$ | awkward | iterative | yes | rejected |
| Biarc | $G^1$ | awkward | iterative | yes | rejected |
| Clothoid | $G^2$ (straight only) | 2D | iterative | no | rejected |
| PH quintic (default) | $G^1$ | yes | certified | yes | rejected as default |
| PH quintic (opt-in) | $G^1$ | yes | certified | yes | **adopted (D6)** |
| B-spline fairing | $G^\infty$ | yes | global | yes | rejected |
| Time-domain FIR | — | yes | none | yes | rejected |
| Extended-tangent ear | $G^2$ | yes | iterative | no | rejected (M20 better) |
| Scoring search | varies | yes | no | yes | rejected |
| Parameter-fraction | varies | yes | no | yes | rejected |
| **Exact Bézier (M11/M12)** | $G^2$/$G^3$ | yes | **certified** | **yes** | **chosen** |

---

## 5. PH fast-path benchmark (Phase 5.4)

The PH quintic fast path (D6) trades $G^2$/$G^3$ boundary continuity for
closed-form arc length and curvature (M16–M19). The benchmark
(`examples/PHBenchmark.cpp`) measures the two phases of planning
separately — blend construction (certified deviation solving) and
velocity planning (curvature sampling + profile computation) — on a
1-corner 90° zigzag path with 200 velocity-profile samples.

**Results (single run, 1 corner, 200 profile samples):**

| Phase | Bezier $G^2$ | PH quintic | Ratio (PH/Bezier) |
|---|---|---|---|
| Blend construction | 13 435 ms | 19 760 ms | 1.47× slower |
| Velocity planning | 74.3 ms | 5.8 ms | **12.8× faster** |
| Total | 13 510 ms | 19 766 ms | 1.46× slower |

**Interpretation:**

- The PH fast path delivers a **12.8× speedup in velocity planning**
  because the closed-form curvature $\kappa(\xi) = 2(uv' - u'v)/\sigma^2(\xi)$ (M16)
  replaces the certified per-span Lipschitz-bound sampler, which must
  adaptively subdivide each span to bound the curvature.
- However, **blend construction is 1.47× slower** with PH because the
  Hermite construction produces 8 candidates (4 sign choices × 2
  curve types) that must each be certified by the DeviationCertifier,
  vs. a single candidate for the exact Bézier builder.
- The blend construction phase dominates total planning time, so PH is
  currently **1.46× slower overall**.

**When PH is beneficial:**

- **Pre-computed blends:** if the blend curves are constructed once
  and the velocity profile is recomputed many times (e.g. different
  feed rates, acceleration limits, or look-ahead windows), the 12.8×
  velocity-planning speedup amortizes the blend construction cost
  after ~2 replanning passes.
- **Real-time interpolation:** the closed-form arc length $s(\xi)$ and its
  polynomial Newton inversion (M19) eliminate adaptive quadrature
  during real-time setpoint generation, which is the primary use case
  for PH curves in CNC (Farouki & Shah 1996).

**Note:** The benchmark is informational, not a hard CI gate. The PH
fast path is opt-in (`BlendSpec::curveType = BlendCurveType::PHQuintic`)
and never the default. Run with:

```bash
build/bin/ph_benchmark <num_corners> <num_runs>
```
