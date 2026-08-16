# TOPPRA-TODO — Audit findings and fix plan for BasicTOPPRA / JerkConstrainedTOPPRA

**Date:** 2026-08-15
**Scope:** `include/tether/motion_planner/BasicTOPPRA.hpp`,
`include/tether/motion_planner/JerkConstrainedTOPPRA.hpp`, plus the code they
depend on (`VelocityProfile.hpp`, `SCurveProfile.hpp`) and the documentation
(`docs/motion/ToppraDerivation.md`, `docs/motion/VelocityProfilerSelection.md`,
`VelocityProfiler.hpp`, `VelocityProfile.hpp`, `docs/motion/MotionChain.md`).

> **Note:** Both profilers are **header-only**. The files
> `BasicTOPPRA.cpp` / `JerkConstrainedTOPPRA.cpp` do not exist; all code lives
> in the `.hpp` files above.

---

## 1. Verdict

- **BasicTOPPRA** is mathematically correct for its stated problem (2nd-order
  bang-bang TOPP-RA on a smooth path). Measured total time on a straight
  100 mm line: 2.1011 s vs. theoretical optimum 2.1000 s. It has minor bugs
  (WI-6, WI-7) and two shared feasibility gaps (WI-4, WI-5).
- **JerkConstrainedTOPPRA** produces a *feasible, jerk-bounded* velocity
  profile, but it is **not a true 3rd-order TOPP-RA and is NOT time-optimal
  subject to the jerk constraint** — contradicting its header, the interface
  docs, and both markdown docs. It ignores per-axis acceleration limits, and
  its stored acceleration/jerk fields are finite-difference artifacts whose
  clamping actively falsifies data consumed downstream by `MotionPlan`
  (`MotionPlan.hpp:563-564` feeds them to feedforward / extrusion consumers).

All findings below were verified empirically with a scratch harness compiled
against `build/lib` (harness deleted after the audit; every number can be
reproduced with the tests specified in §6).

### Measured evidence (straight 100 mm line, feed 50 mm/s, a_max = 500, j_max = 5000)

| Quantity | Theory | Implementation | Verdict |
|---|---|---|---|
| BasicTOPPRA total time (N=100) | 2.1000 s | 2.1011 s | OK |
| JerkConstrained total time (N=25) | 2.2000 s | 2.2199 s | ~OK |
| JerkConstrained total time (N=100) | 2.2000 s | 2.6738 s | **+21%** |
| JerkConstrained total time (N=1000) | 2.2000 s | 5.6469 s | **+157%** |
| JerkConstrained accel with axis limit 100, path limit 500 | ≤ 100 | 146.7 stored / ~293 implied | **Violated** |
| JerkConstrained stored peak accel vs true implied S-curve peak | equal | 146.7 vs 293.4 | **Understated 2×** |
| `limitedBy` histogram (N=100) | mixed | `ForwardAccel`: 100/100 | **Broken** |
| aMax=0 degenerate config, start from rest | stop / error | jumps to 50 mm/s instantly | **Infinite accel** |
| Exact-path 90° corner (L-shape, no blend) | v→0 at corner | v = 50 mm/s through corner | **Infinite centripetal accel** |

---

## 2. Root cause of the main problem (read first)

`SCurveProfile::computeAccelDistance(v0, v1, aMax, jMax)`
(`SCurveProfile.hpp:398-423`) computes the distance of a **symmetric S-curve
that starts AND ends at zero acceleration**. Both passes of
`JerkConstrainedTOPPRA` call it once per grid interval, so the optimizer
implicitly enforces the extra constraint **a = 0 at every sample point** —
a constraint that does not exist in the problem statement.

Consequences:
1. The profile is much slower than the true jerk-limited optimum, and the
   suboptimality **grows with `numSamples`** (more samples = more forced
   zero-acceleration points). See the table above.
2. Between two samples the implied trajectory ramps acceleration
   0 → a_peak → 0, so the **true peak acceleration is ≈ 2× the stored
   finite-difference value** (stored a is the interval average).
3. The doc claims "jerk is ±j_max or 0 by construction" refers to the implied
   per-interval S-curve, not to the stored `jerk` field, which is a clamped
   finite difference (see WI-3).

`BasicTOPPRA` does not have this problem because 2nd-order reachability
(`v² = v₀² + 2aΔs`) is exact regardless of the acceleration state.

---

## 3. Work items (ordered by priority)

Each item lists: files/lines, what to change, why, and acceptance criteria.
Items WI-1…WI-7 are small and independent; do them first. WI-8 is the large
algorithmic item. WI-9/WI-10 are documentation-only but **mandatory** even if
WI-8 is postponed.

---

### WI-1 — Validate inputs; fix degenerate-limit and grid-size holes (both profilers)

**Files:** `BasicTOPPRA.hpp` (`computeProfile`, line 68), `JerkConstrainedTOPPRA.hpp` (line 113), `SCurveProfile.hpp:398-401,459-462`.

**Problem:**
- `numSamples < 2` → `ds = pathLength / (numSamples - 1)` divides by zero or
  wraps around (`size_t`). No guard.
- If `maxPathAcceleration <= 0` while jerk is enabled,
  `computeAccelDistance` returns 0 for **any** v1
  (`SCurveProfile.hpp:400`), so `maxVelocityAfterDistance`'s binary search
  concludes "everything fits" and returns `vMax`. Measured: profile jumps
  from standstill to 50 mm/s in one sample — infinite acceleration.
- `maxCentripetalAcceleration < 0` with κ > 0 → `sqrt` of a negative number
  → NaN velocity limit (`BasicTOPPRA.hpp:278-280`, `JerkConstrainedTOPPRA.hpp:327-329`).

**Change:**
1. At the top of both `computeProfile` implementations, add:
   ```cpp
   if (numSamples < 2) return profile;   // empty
   if (feedRate <= T(0)) return profile;
   if (limits_.path.maxPathAcceleration <= T(0)) {
       // log via KLIPPER_LOG_ERROR-style logger used in this module,
       // or std::abort in debug — pick the project convention
       return profile;
   }
   if (limits_.path.maxCentripetalAcceleration <= T(0)) { /* same */ }
   ```
   In `JerkConstrainedTOPPRA`, extend the existing jerk fallback
   (`JerkConstrainedTOPPRA.hpp:135-143`): fall back to `BasicTOPPRA` also
   when `jMax <= 0`, and never enter the jerk passes with `aMax <= 0`.
2. In `SCurveProfile::computeAccelDistance`, replace the silent `return T(0)`
   for `aMax <= 0 || jMax <= 0` with `return std::numeric_limits<T>::infinity()`
   (infinite distance = infeasible), so the binary search degrades to `v0`
   instead of `vMax`. Add a comment explaining why.

**Acceptance:** new unit tests (§6, T5): `numSamples=0/1`, `aMax=0`,
`maxCentripetalAcceleration=-1` all return empty or rest profiles, never NaN,
never a velocity jump.

---

### WI-2 — Enforce per-axis acceleration (and jerk) limits in JerkConstrainedTOPPRA

**File:** `JerkConstrainedTOPPRA.hpp:131-135, 197-208, 220-230`.

**Problem:** The passes use only `limits_.path.maxPathAcceleration` /
`maxPathJerk`. `BasicTOPPRA` routes through
`KinematicLimits::maxAccelerationForDirection()` (which folds in per-axis
limits); the jerk profiler never calls it. Measured: axis limit 100 mm/s²,
path limit 500 mm/s² → profile accelerates at ~147 mm/s² (stored),
~293 mm/s² implied peak. Per-axis jerk (`axis.maxJerk`,
`axis.jerkLimitEnabled`) is likewise ignored.

**Change:**
1. In the sampling loop, also store a per-sample scalar
   `aMax[i] = limits_.maxAccelerationForDirection(samples[i].tangent,
   samples[i].curvature, /*v*/ forwardVel/backwardVel estimate or 0)`.
2. In the backward pass (line 203) and forward pass (line 225) replace the
   scalar `aMax` with `std::min(aMax[i-1], aMax[i])` for that interval
   (conservative over the interval; note the signature of
   `maxVelocityAfterDistance` already takes `aMax` per call — just pass the
   per-interval value).
3. Per-axis jerk: compute
   `jMax_dir = min_i(axis.maxJerk[i] / |t_i|)` over axes with
   `|t_i| > EPSILON` (same projection pattern as
   `maxVelocityForDirection`, `VelocityProfile.hpp:123-134`) when
   `axis.jerkLimitEnabled`, and use `jMax_eff = min(path.maxPathJerk, jMax_dir)`.
   If this projection is considered out of scope, document the omission in
   `VelocityProfilerSelection.md` §2 (currently claims all limits respected).

**Acceptance:** §6 test T3: straight X move, axis accel 100 / path 500 →
stored and implied acceleration ≤ 100 + ε; total time ≈ the jerk-limited
optimum for a_max=100. Existing behavior with axis ≥ path limits unchanged.

---

### WI-3 — Stop falsifying acceleration/jerk outputs; fix their semantics

**Files:** `JerkConstrainedTOPPRA.hpp:269-300`, `VelocityProfile.hpp:244-251,271-276`, `BasicTOPPRA.hpp:227-237`.

**Problem:**
- Stored `acceleration` is a backward finite difference of the min()-combined
  profile; for the jerk profiler this equals the *interval average* of the
  implied S-curve, understating the true peak by ≈ 2× (measured 146.7 vs 293.4).
- Stored `jerk` is a finite difference of that finite difference, then
  **clamped to ±jMax** (line 288-290), which hides real overshoots and
  guarantees the field never reports a violation. The doc claim "jerk is
  ±j_max or 0 (by construction)" (`VelocityProfile.hpp:274`,
  `ToppraDerivation.md` T.9) is false for the stored values.
- These values flow into `MotionPlan::evaluateAt()` → `state.pathAcceleration`
  / `state.pathJerk` (`MotionPlan.hpp:563-564`) and from there to feedforward
  and extrusion compensation.

**Change (minimal, honest):**
1. Remove the jerk clamp at `JerkConstrainedTOPPRA.hpp:288-290`. Replace with
   a debug-only check (`assert` or `KLIPPER_LOG_WARN` when
   `|jerk| > 1.5*jMax`) so real violations surface instead of being hidden.
2. Document the actual semantics in `VelocityProfilePoint`
   (`VelocityProfile.hpp:183-189`): `acceleration`/`jerk` are *interval-average
   finite-difference approximations*; for the jerk profiler the true
   per-interval peak acceleration is up to 2× the stored value while the
   forced a=0-at-samples construction is in place. Fix the wrong docstrings
   on `accelerationAt()` ("analytic acceleration computed during the
   forward/backward passes" — it is not) and `jerkAt()` ("±j_max or 0 by
   construction" — it is not).
3. If WI-8 is implemented, replace the finite differences with the analytic
   acceleration/jerk of the carried state and delete this paragraph from the
   docs again.

**Acceptance:** no clamping code remains; docstrings match implementation;
§6 test T4 asserts `|stored jerk|` is reported truthfully.

---

### WI-4 — Handle tangent discontinuities (exact/unblended corners)

**Files:** `BasicTOPPRA.hpp:107-152` (sampling loop), `JerkConstrainedTOPPRA.hpp:151-180`, docs.

**Problem:** With `PathMode::ExactPath` (no blending), two line pieces meet
at a corner with a tangent discontinuity. Curvature is 0 inside each piece,
so the velocity limit curve never sees the corner. Measured: 90° corner
traversed at the full 50 mm/s — physically an instantaneous direction change
= infinite centripetal acceleration. The certified curvature sampler cannot
help: there is no span with κ > 0.

**Change:**
1. In both profilers' sampling loops, detect piece-boundary crossings:
   `path.evaluateAtArcLength(s)` already returns `segmentIndex`
   (`PathAdapter.hpp:207`). When `segmentIndex` changes between sample i-1
   and i, compare the tangents of the two samples. If
   `angle = acos(clamp(t_prev·t_cur, -1, 1)) > 1e-6`, force
   `velocityLimit` at the boundary sample to a junction velocity:
   - simplest correct choice: `v_junction = 0` (exact-stop semantics, matches
     LinuxCNC "exact path" mode);
   - optionally later: GRBL/Klipper-style junction deviation velocity.
   Put the junction detection behind a small helper in the profiler, e.g.
   `computeJunctionVelocity(tPrev, tCur, limits)`.
2. Document in `ToppraDerivation.md` (§"Velocity Limit Curve") and
   `VelocityProfilerSelection.md` that profilers assume C¹ paths *unless*
   junction handling inserts zero-velocity points, and state which behavior
   is implemented.

**Acceptance:** §6 test T6: L-shaped exact path → velocity at the corner
sample = 0 (or ≤ junction velocity); blended path behavior unchanged.

---

### WI-5 — Close the between-sample curvature gap in v_lim

**Files:** `BasicTOPPRA.hpp:107-152`, `JerkConstrainedTOPPRA.hpp:151-180`,
`CertifiedCurvatureSampler` usage; docs `ToppraDerivation.md` (T.3).

**Problem (two parts):**
1. Uniform arc-length sampling can skip a narrow span entirely; its certified
   max curvature is then never reflected in `v_lim`, and linear interpolation
   of velocity between samples can exceed the true limit inside that span.
2. The PH fast path (`BasicTOPPRA.hpp:124-139`) deliberately uses **pointwise**
   κ, not the certified per-span max — the code comment admits this, while
   `ToppraDerivation.md:107-114` claims certification is "critical" and
   implies it is always used. Doc contradicts code.

**Change:**
1. Extend the sampling loop: for each interval `[s_{i-1}, s_i]`, query the
   certified sampler for the max curvature over the *interval* (add an
   interval-max method to `CertifiedCurvatureSampler` that iterates the spans
   intersecting the interval and takes the max certificate — the sampler
   already certifies per span, so this is a max-fold, not new math). Use
   `κ_interval_max` for `velocityLimit[i]`.
2. For PH pieces: either route `v_lim` through the same interval certificate
   (keep the closed-form κ only for the acceleration-budget code path), or
   compute the analytic max of the PH curvature κ(ξ) = 2(uv'−u'v)/σ² on the
   span (rational function; bound it by sampling + Lipschitz constant of the
   numerator/denominator — same certificate pattern as the existing sampler).
   If (2) is postponed, fix the docs to state the PH fast path is uncertified
   and bounded by sample density.

**Acceptance:** §6 test T7: path with a narrow high-curvature span placed
between uniform samples → peak `v²·κ` along the executed profile ≤
`maxCentripetalAcceleration + ε`.

---

### WI-6 — Fix BasicTOPPRA small bugs

**File:** `BasicTOPPRA.hpp`.

1. **Line 180:** backward pass evaluates the deceleration limit at
   `samples[i]` / `backwardVelocity[i]` — the *destination* point. For
   symmetry with the forward pass (which uses `samples[i-1]`), use
   `samples[i-1]` / `backwardVelocity[i-1]`... no: use the constraint of the
   interval being traversed; pick `samples[i-1]` with the velocity at i-1 is
   wrong too since backwardVelocity[i-1] is not yet known. The clean choice:
   use `computeMaxAcceleration(samples[i], backwardVelocity[i])` but document
   it, or evaluate at both endpoints and take the min. **Recommended:** take
   `min` over both endpoints (conservative, matches the interval semantics
   used in WI-2/WI-5).
2. **Line 232:** dead variable `nextVel` — delete (it also reads
   `forwardVelocity[i+1]`, which is misleading because that is not the final
   velocity).
3. **Lines 228-237:** the last profile point never gets an acceleration value
   (stays 0 even during the final deceleration; measured: last point a=0
   while decelerating at −500). Set it with the same backward difference as
   interior points.
4. **`startJerk` parameter** (line 73) is accepted and silently ignored —
   correct for a 2nd-order profiler, but say so in the docstring
   ("ignored; basic TOPP-RA has unbounded jerk").

**Acceptance:** build with `-Wall` shows no unused-variable warning for this
file; §6 test T2 checks last-point acceleration.

---

### WI-7 — Fix `limitedBy` diagnostics in JerkConstrainedTOPPRA

**File:** `JerkConstrainedTOPPRA.hpp:220-230, 245-256`.

**Problem:** The forward pass already caps by `vLim[i]` and `backwardVel[i]`
(line 229), so `forwardVel[i] ≤ backwardVel[i]` and `≤ vLim[i]` for all i.
The final `min({fwd, bwd, lim})` therefore always returns `fwd`, and the
first `limitedBy` branch is always taken. Measured: 100/100 points report
`ForwardAccel`; `LimitType::Jerk` is unreachable anywhere in the codebase.

**Change:**
1. In the forward pass, record which of the three caps was active:
   ```cpp
   T vAccel = SCurve::maxVelocityAfterDistance(...);
   T v = vAccel; auto cause = Point::LimitType::ForwardAccel;
   if (vLim[i] < v)          { v = vLim[i];        cause = Point::LimitType::Curvature; }
   if (backwardVel[i] < v)   { v = backwardVel[i]; cause = Point::LimitType::BackwardDecel; }
   forwardVel[i] = v; forwardCause[i] = cause;
   ```
   (Distinguish `FeedRate`/`AxisVelocity`/`Curvature` by re-checking which
   term inside `computeVelocityLimit` was minimal — store that from
   `computeVelocityLimit` via an out-parameter or a small struct.)
2. In the final loop use the recorded cause instead of re-deriving it with
   float equality. Drop the unreachable `Jerk` branch or repurpose it for
   intervals where the jerk cap (not the accel cap) was active in the
   distance function — that requires `computeAccelDistance` to report its
   active case; simplest is to remove `Jerk` from the enum usage here and
   leave the enum value for future use.
3. Also remove the redundant final `min({fwd, bwd, lim})` (keep a
   `DCHECK`-style assertion that `fwd <= min(bwd, lim)`), or drop the in-pass
   capping and keep the final min — pick one scheme, document it. (Analysis
   during the audit showed both schemes coincide for feasible inputs because
   backward-pass dips coincide with `vLim` dips; the in-pass capping is fine.)

**Acceptance:** §6 test T8: on a path with a tight blended corner, the
histogram contains `Curvature` (or `FeedRate` on straight cruise), and no
point reports an impossible cause.

---

### WI-8 — (Large) Restore time-optimality of JerkConstrainedTOPPRA

**Files:** `JerkConstrainedTOPPRA.hpp`, possibly `SCurveProfile.hpp`;
docs `ToppraDerivation.md` (T.6, T.11), `VelocityProfilerSelection.md`.

**Problem:** see §2. The profiler forces a = 0 at every sample point; it is
feasible but far from time-optimal, and gets worse as `numSamples` grows
(measured: +21% at N=100, +157% at N=1000 on a straight line).

**Two options — decide before starting:**

**Option A (documentation + mitigation, ~1 day).** Keep the algorithm.
- Reword all "time-optimal" claims (WI-9 list) to "feasible and jerk-bounded,
  conservative; suboptimality grows with sample count; typically within
  x–y% of the jerk-limited optimum at the recommended N≈100–200 samples
  per move" (measure the real numbers and state them).
- Add a note recommending `AnalyticalTOPPRA`
  (`include/tether/motion_planner/analytical/AnalyticalTOPPRA.hpp`, see
  `docs/motion/MotionChain.md:241,276`) for applications needing true
  optimality — first verify whether it supports jerk constraints; if it
  does, say so explicitly in `VelocityProfilerSelection.md`.

**Option B (proper fix, ~1–2 weeks, needs design review).** Carry the
acceleration as state in both passes — i.e., turn the profiler into a real
3rd-order scheme:
1. Forward pass state: `(v_i, a_i)`. Per interval, integrate the
   time-optimal jerk-bang-bang control from `(v_{i-1}, a_{i-1})`:
   `j = +jMax` until `a = aMax`, then hold; solve the cubic
   `Δs = v₀τ + ½a₀τ² + (1/6)jτ³` for the phase time τ (closed form via
   Cardano or Newton from the previous τ — monotone in τ, 2–3 iterations).
   This yields exact max `(v_i, a_i)` without forcing `a_i = 0`.
2. Backward pass symmetric, ending at `(v_end, 0)`.
3. Switching points where `fwd` and `bwd` cross need a jerk-limited
   transition (acceleration cannot jump from +a to −a); the classic
   approach inserts a jerk-limited connecting arc around each crossing
   (this is where the extra distance goes — see e.g. the TOPP-RA3 /
   Reflexxes OTG literature). Implement crossing detection + local
   re-solve.
4. Store the *analytic* `a_i`/`j_i` from the carried state (fixes WI-3 at
   the root).
5. Re-run the evidence table: N=100 and N=1000 must both land within a few
   % of the theoretical optimum (2.2 s on the reference line) and be
   N-independent.

**Recommendation:** do Option A now (it is mostly WI-9), schedule Option B
as a separate design-reviewed task. Do **not** let Option B block the
feasibility fixes WI-1…WI-7.

**Acceptance (Option B):** §6 test T1 passes with tight tolerances at
multiple N; T9 (grid independence) passes.

---

## 4. Numerical / performance items (do after WI-1…WI-7)

### WI-P1 — Replace the 60-iteration binary search with a direct inversion

**File:** `SCurveProfile.hpp:459-477` (`maxVelocityAfterDistance`).

The 60-iteration bisection per sample is exact but ~10–20× slower than
necessary (`ToppraDerivation.md` T.7 even brags about the over-precision).
`computeAccelDistance` is piecewise polynomial and strictly monotone in v1:
- Case 1 (`Δv ≤ a²/j`): solve `2v₀t + jt³ = d` for t (depressed cubic,
  closed form), then `v1 = v0 + jt²`.
- Case 2: solve the quadratic/cubic of the full 3-phase distance directly.
Implement a closed-form/Newton inversion (Newton from
`v1⁰ = v0 + d·aMax/(2v0+1)`-style guess converges in 2–4 iterations); keep
the bisection as a fallback if Newton fails to converge. Update
`ToppraDerivation.md` T.7 accordingly.

### WI-P2 — Centripetal component ignored in the acceleration budget

**File:** `VelocityProfile.hpp:141-162` (`maxAccelerationForDirection`).

- The tangential budget is never reduced by the centripetal load. If
  `maxPathAcceleration` is a magnitude limit, the available tangential part
  is `sqrt(a_path² − (v²κ)²)`. State the chosen semantics in the docstring
  and implement accordingly (today the centripetal term only triggers the
  dead `return 0` branch — dead because `v_lim` already enforces
  `v²κ ≤ a_cent`; remove or repurpose that branch).
- Per-axis limits ignore the normal component: the true per-axis
  acceleration is `a_i = a_tan·T_i + v²κ·N_i`. `PathAdapter` already
  returns `normal` (`PathAdapter.hpp:241-242`); pass it into
  `maxAccelerationForDirection` and solve the per-axis budget with both
  components. On curves this currently exceeds per-axis acceleration limits
  in both profilers. Update `ToppraDerivation.md` T.10 to match the
  implemented formula.

### WI-P3 — `startAcceleration`/`startJerk` are stored but not honored

Both profilers accept `startAcceleration` (for replanning from a moving
state) but the passes assume zero initial acceleration. Either document
"stored on the first point only; the optimization assumes a(0)=0" (one-line
doc change), or honor it in the first interval (requires the Option-B
machinery of WI-8). Decide and document.

---

## 5. Documentation fixes (WI-9) — mandatory corrections

All of these are wrong or misleading **today**, independent of WI-8:

| File:line | Current claim | Reality / required change |
|---|---|---|
| `docs/motion/ToppraDerivation.md` T.11 (line 510-523) | jerk variant is time-optimal "same argument applies" | False — a=0 forced at every sample. Rewrite: feasible, jerk-bounded, conservative; suboptimality ∝ grid density; reference measured numbers. |
| `docs/motion/ToppraDerivation.md` T.6 (line 367-399) | describes passes as if they were 3rd-order TOPP | Add the zero-endpoint-acceleration assumption of (T.5) and its consequence (§2 above). |
| `docs/motion/ToppraDerivation.md` T.9 (line 456-483) | "jerk is ±j_max or 0 by construction", clamp "removes numerical noise" | Stored values are finite differences; clamp falsifies. Rewrite after WI-3. |
| `docs/motion/ToppraDerivation.md` T.3 (line 96-114) | certified curvature always used | PH fast path uses pointwise κ (see WI-5). |
| `docs/motion/ToppraDerivation.md` T.7 (line 407-439) | 60-iteration binary search | Update if WI-P1 lands. |
| `docs/motion/ToppraDerivation.md` summary table (line 550-561) | "Time-optimal: Yes (subject to jerk)", "Global constraints: Yes" | Correct per above; jerk variant does not enforce per-axis accel (WI-2). |
| `docs/motion/VelocityProfilerSelection.md` line 13-17, 131-146, 294-309 | "Time-optimal subject to jerk", "All constraints verified... guaranteed feasible", "5-15% slower" | Correct: not time-optimal (grid-dependent, up to >2× slower), per-axis accel not enforced pre-WI-2, stored jerk not ±j_max. The "5-15%" figure is unsubstantiated — replace with measured numbers. |
| `docs/motion/VelocityProfilerSelection.md` line 340-347 | jerk fallback behavior | Also falls back on `jMax ≤ 0` etc. after WI-1. |
| `include/tether/motion_planner/VelocityProfiler.hpp:14-18, 29-37` | "3rd-order time-optimal profile", "still guaranteed feasible" | Reword: feasible and jerk-bounded; not time-optimal until WI-8. |
| `include/tether/motion_planner/JerkConstrainedTOPPRA.hpp:1-62, 81-113` | "time-optimal *subject to the jerk constraint*", "Per-axis velocity and acceleration limits" respected | Fix both claims. |
| `include/tether/motion_planner/VelocityProfile.hpp:188, 246-250, 273-275` | semantics of `jerk`/`accelerationAt`/`jerkAt` | Fix per WI-3. |
| `docs/motion/MotionChain.md:274` | "Default for 3D printing — smooth + time-optimal" | Drop "time-optimal" (or qualify). |
| `docs/motion/ToppraDerivation.md` (new) | — | Add a section on the C¹/tangent-discontinuity requirement and junction handling (WI-4). |

---

## 6. Test plan (WI-10)

**Current state:** the only dedicated test file,
`tests/motion_planner/VelocityProfilerTest.cpp` (68 lines, two trivial
passthrough tests), is **excluded from the build** — see the removal NOTE in
`tests/CMakeLists.txt:244-249` — and would not even compile against the new
`PathAdapter` API. Effective dedicated coverage of both profilers is zero.

**Steps:**
1. Delete or rewrite `tests/motion_planner/VelocityProfilerTest.cpp`.
2. Create `tests/motion_planner/ToppraAuditTest.cpp` and register it in
   `MOTION_PLANNER_INTEGRATION_SOURCES` (`tests/CMakeLists.txt:252-258`).
   Build/run:
   ```bash
   cmake --build build --target tether_motion_planner_integration_tests -j$(nproc)
   ./build/bin/tests/tether_motion_planner_integration_tests --gtest_filter='ToppraAudit*'
   ```
3. Required tests (helper: build paths with `PathBuilderAdapter` from
   `MotionSegment::linear`, exactly as in the audit harness):
   - **T1 Optimality bound (jerk):** straight 100 mm line, feed 50, a=500,
     j=5000 → theoretical optimum 2.2 s. Today: expect ≈2.67 s at N=100 —
     encode as a documented `EXPECT_NEAR(..., 2.67, 0.05)` with a TODO, or
     `EXPECT_LE(T, 2.2*1.05)` marked `DISABLED_` until WI-8.
   - **T2 Basic correctness:** straight line → total time ≈ 2.1 s ± 1%;
     last-point acceleration ≈ −a_max (fails today, WI-6.3); max stored
     |a| ≈ a_max.
   - **T3 Per-axis accel (jerk):** axis 100 / path 500 → stored AND implied
     ((vᵢ²−vᵢ₋₁²)/(2Δs)) accel ≤ 100+ε. Fails today (WI-2).
   - **T4 No jerk clamping:** construct a case where the fd jerk exceeds
     jMax before WI-3; after WI-3 assert the stored value is reported, not
     clamped.
   - **T5 Degenerate inputs:** `numSamples ∈ {0,1}`, `aMax=0`,
     `maxCentripetalAcceleration=-1`, `feedRate=0` → empty/rest profile, no
     NaN (WI-1).
   - **T6 Exact corner:** L-path in `ExactPath` mode → v(corner) ≈ 0
     (WI-4).
   - **T7 Curvature gap:** narrow high-κ span between uniform samples →
     max v²κ along profile ≤ a_cent+ε (WI-5).
   - **T8 limitedBy:** blended-corner path produces non-`ForwardAccel`
     causes (WI-7).
   - **T9 Grid independence:** T(N=100) ≈ T(N=400) within a few %
     (`DISABLED_` until WI-8 Option B).
   - **T10 Regression:** symmetric S-curve distance function unit tests for
     `computeAccelDistance` against the analytic formulas in
     `ToppraDerivation.md` T.5 (these pass today — pin the behavior before
     WI-P1 changes the inversion).

---

## 7. Suggested execution order

1. WI-1 (guards) → WI-2 (per-axis) → WI-4 (corners): feasibility, small diffs.
2. WI-3 (stop falsifying outputs) → WI-6 → WI-7 (diagnostics).
3. WI-10 tests (each test lands with its WI).
4. WI-9 docs (or in parallel; mandatory regardless of WI-8).
5. WI-P1/WI-P2/WI-P3 (performance/numerics).
6. WI-8 decision: Option A immediately; Option B as a separate,
   design-reviewed task.

## 8. Explicitly out of scope

- `analytical::AnalyticalTOPPRA` (`include/tether/motion_planner/analytical/`)
  — not audited here; verify its jerk support before citing it as the
  optimal alternative in docs (WI-8 Option A).
- `SCurveVelocityProfiler` — separate known limitations already documented
  in `VelocityProfilerSelection.md:266-275`.
- `ProfileReplanner` / `CurvatureAwareLimiter` interaction with these
  profilers (`src/replanner/ProfileReplanner.cpp`).
