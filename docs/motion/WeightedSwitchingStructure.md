# Weighted Switching Structure (WSS) Reference

**Manual for interpreting and consuming the output of
[`ParetoTimeEnergyOptimalVelocityPlanner`](ParetoTimeEnergyOptimal.md)**

## Overview

The `WeightedSwitchingStructure<Dim, T>` (WSS) is the exact, analytic output
representation of the Pareto time-energy-optimal velocity planner. It replaces
the tabulated `VelocityProfile` with a piecewise-analytic trajectory source:
every segment of the motion is stored as one of a small number of primitive arc
types, and the state at any time is reconstructed from closed-form formulas
rather than interpolated from a sampled table.

The WSS is the source of truth for:

- Exact position, velocity, acceleration, and jerk at arbitrary time $t$
- Arc-length-to-time mapping without numerical ODE integration (except on
  constraint-surface arcs)
- Time-energy Pareto analysis
- Analytical extrusion compensation (pressure advance, flow adaptive heater,
  deconvolution)
- Step generation and motion translation in the Klipper layer

**Key properties:**

| Property | Value |
|---|---|
| Arc types | `BANG_PLUS`, `BANG_MINUS`, `SINGULAR`, `WALL` |
| Domain | Arc length $s \in [0, s_f]$ |
| Time | Absolute time $t \in [0, T]$ |
| Sampling cost | $O(\log N_{\text{arcs}})$ locate + $O(1)$ polynomial eval |
| Constraint checks | Enforced by construction, with hard guards at sample time |
| Downstream formats | `VelocityProfile` (sampled), `ExtrusionTrajectory`, `MotionPlan` |

If you are only consuming trajectories, you normally use the WSS through the
`AnalyticalTrajectorySource` interface or the `VelocityProfile` wrapper.
If you are writing a new consumer (extrusion model, step generator, certified
checker), you need to understand the arc list itself.

---

## Table of Contents

1. [Arc-List View vs. Sampled View](#1-arc-list-view-vs-sampled-view)
2. [The Four Arc Types](#2-the-four-arc-types)
3. [The `WeightedArc` Record](#3-the-weightedarc-record)
4. [How the Arc List Is Built](#4-how-the-arc-list-is-built)
5. [Coalescing and Why It Matters](#5-coalescing-and-why-it-matters)
6. [Interpreting an Arc List](#6-interpreting-an-arc-list)
7. [State Reconstruction at Sample Time](#7-state-reconstruction-at-sample-time)
8. [WALL Arcs and Quadrature](#8-wall-arcs-and-quadrature)
9. [Time-at-Arc-Length Inversion](#9-time-at-arc-length-inversion)
10. [Lifetime, Ownership, and Thread Safety](#10-lifetime-ownership-and-thread-safety)
11. [Downstream Consumers](#11-downstream-consumers)
12. [Common Pitfalls and Interpretation Rules](#12-common-pitfalls-and-interpretation-rules)
13. [API Reference](#13-api-reference)
14. [Implementation Files](#14-implementation-files)

---

## 1. Arc-List View vs. Sampled View

Most older motion-planning code consumes a `VelocityProfile`:

```cpp
for (const auto& pt : profile.points()) {
    double s = pt.arcLength;
    double v = pt.velocity;
    double a = pt.acceleration;
    double j = pt.jerk;
    double t = pt.time;
}
```

This is a **sampled view**: finite-difference derivatives, fixed grid, and no
guarantee that the trajectory is feasible between sample points.

The WSS is an **arc-list view**:

```cpp
for (const auto& arc : wss->arcs()) {
    // arc.type, arc.s0, arc.s1, arc.t0, arc.v0, arc.a0, arc.eta, ...
}
```

Each arc is a small piece of the trajectory with a known closed-form
parametrization. The sampled view is derived from the arc-list view, not the
other way around. Any consumer that needs exact timing, exact derivatives, or
certified constraint bounds should work with the WSS or with the
`AnalyticalTrajectorySource` interface that it implements.

---

## 2. The Four Arc Types

Every arc in a WSS has one of these types:

### 2.1 `BANG_PLUS`

- **Control:** jerk $\eta = \eta_{\max} > 0$ (or the largest feasible $\eta$)
- **Time-domain shape:** acceleration is linear in time, velocity is quadratic,
  arc length is cubic
- **Purpose:** raise acceleration from a lower value toward the singular level
  $a_*$
- **Closed form ($\tau = t - t_0$):**

$$
\begin{aligned}
a(\tau) &= a_0 + \eta \tau \\
v(\tau) &= v_0 + a_0 \tau + \tfrac{1}{2} \eta \tau^2 \\
s(\tau) &= s_0 + v_0 \tau + \tfrac{1}{2} a_0 \tau^2 + \tfrac{1}{6} \eta \tau^3
\end{aligned}
$$

### 2.2 `BANG_MINUS`

- **Control:** jerk $\eta = \eta_{\min} < 0$ (or the smallest feasible $\eta$)
- **Time-domain shape:** same polynomial family as `BANG_PLUS`, with negative $\eta$
- **Purpose:** lower acceleration from a higher value toward $a_*$, or initiate
  braking

### 2.3 `SINGULAR`

- **Control:** jerk $\eta = 0$
- **Time-domain shape:** constant acceleration $a \equiv a_*$
- **Purpose:** cruise at the optimal singular acceleration level selected by
  the cost weights
- **Closed form:**

$$
\begin{aligned}
a(\tau) &= a_* \\
v(\tau) &= v_0 + a_* \tau \\
s(\tau) &= s_0 + v_0 \tau + \tfrac{1}{2} a_* \tau^2
\end{aligned}
$$

When $a_* \to 0$ this degenerates to constant-velocity cruise:

$$
\tau = \frac{\Delta s}{v_0}, \quad v(\tau) = v_0
$$

### 2.4 `WALL`

- **Control:** velocity follows the constraint wall $v(s) = v_{\text{wall}}(s)$;
  acceleration is slaved to the wall slope
- **Time-domain shape:** not a polynomial; time is computed by quadrature
- **Purpose:** follow an active velocity constraint (feed rate, curvature limit,
  per-axis limit)
- **Kinematics on the wall:**

$$
\begin{aligned}
v(s) &= v_{\text{wall}}(s) \\
a(s) &= v \frac{dv_{\text{wall}}}{ds} \\
\eta(s) &= 0
\end{aligned}
$$

The WSS precomputes the wall duration by 8-point Gauss-Legendre quadrature and
inverts it by bisection at sample time.

### 2.5 Summary

| Type | $\eta$ | $a(t)$ | $v(t)$ | $s(t)$ | Inversion |
|---|---|---|---|---|---|
| `BANG_PLUS` | $>0$ | linear | quadratic | cubic | cubic root (Newton) |
| `BANG_MINUS` | $<0$ | linear | quadratic | cubic | cubic root (Newton) |
| `SINGULAR` | $0$ | constant | linear | quadratic | quadratic root |
| `WALL` | $0$ | from wall slope | from wall | quadrature | bisection on quadrature |

---

## 3. The `WeightedArc` Record

Each arc is a `WeightedArc` structure. The fields have precise meanings and
must be read together; no single field is meaningful in isolation.

```cpp
struct WeightedArc {
    WeightedArcType type;   // BANG_PLUS, BANG_MINUS, SINGULAR, WALL
    double s0, s1;          // arc-length span [s0, s1]
    double t0;              // absolute time at s0
    double v0, a0;          // state at (s0, t0)
    double u0;              // NURBS parameter at s0
    double eta;             // constant jerk (BANG only)
    double a_star;          // constant acceleration (SINGULAR only)
    double duration;        // time span Δt = t1 - t0
};
```

### Field-by-field interpretation

- `type`
  - Determines which closed-form formulas apply.
  - **Do not ignore this field.** The other fields are only valid in the
    context of the arc type.

- `s0`, `s1`
  - Domain of the arc in path arc length. `s1 > s0` for a valid arc.
  - Arcs are ordered, contiguous, and non-overlapping:
    `arcs[i].s1 == arcs[i+1].s0` (within floating-point tolerance).

- `t0`
  - Absolute time at the start of the arc. In a coalesced WSS this equals the
    cumulative time of all previous arcs.

- `v0`
  - Path velocity at the start of the arc. Must be non-negative.

- `a0`
  - Path acceleration at the start of the arc.
  - For `BANG` arcs this is the initial acceleration for the polynomial family.
  - For `SINGULAR` arcs this field is unused; use `a_star` instead.
  - For `WALL` arcs this field is unused; acceleration is computed from the wall.

- `u0`
  - NURBS parameter at `s0`. Used only for traceability and wall evaluation;
  most consumers do not need it.

- `eta`
  - Constant jerk for `BANG_PLUS` and `BANG_MINUS`.
  - Zero for `SINGULAR` and `WALL`.
  - **Important:** this is the jerk **after clamping** to feasible bounds
    `[eta_min, eta_max]`. It may be smaller in magnitude than `eta_max` if the
    guidance law was clamped.

- `a_star`
  - The singular acceleration level $a_*$ for `SINGULAR` arcs.
  - For `BANG` arcs this stores the **target** $a_*$ that the bang arc is trying
    to reach (the same value for the whole trajectory). It is not the actual
    acceleration inside the arc.
  - For `WALL` arcs this field is unused.

- `duration`
  - $\Delta t = t_1 - t_0$, the time needed to traverse the arc length
    `s1 - s0` under the arc's control law.
  - For `BANG` and `SINGULAR` arcs this is a closed-form root solve.
  - For `WALL` arcs this is a quadrature result.

### Invariants that must hold

For a valid, coalesced WSS:

1. `arcs` is ordered by `s0` (and therefore by `t0`).
2. `arcs[i].s1 == arcs[i+1].s0` (contiguous in arc length).
3. `arcs[i].t0 + arcs[i].duration == arcs[i+1].t0` (contiguous in time).
4. The final state of arc $i$ matches the initial state of arc $i+1$:
   - `arcs[i+1].v0 == v(arcs[i].duration)`
   - `arcs[i+1].a0 == a(arcs[i].duration)` (for `BANG`)
   - `arcs[i+1].a_star == arcs[i].a_star` (for `SINGULAR`)
5. `v0 >= 0` everywhere.
6. `s1 > s0` for every arc.

A consumer can rely on these invariants **after** it has verified that the
profiler returned a non-empty WSS. If the solve failed, `weightedSource()`
returns `nullptr`.

---

## 4. How the Arc List Is Built

The `WeightedTimeEnergySolver` produces the arc list by a forward state-machine
simulation over a cached constraint grid. The pseudo-code for one fixed $a_*$
(golden-section candidate) is:

```text
s = 0, t = 0, v = v0, a = 0
while s < s_end:
    (eta_min, eta_max, v_lim, a_min, a_max) = evaluateConstraints(s, v, a)

    if remaining_distance is within terminal braking distance:
        solve terminal BANG arc that lands at (s_end, v=0)
        append arc and break

    if v >= v_lim:
        type = WALL, eta = 0
    else if a < a* - tol:
        type = BANG_PLUS, eta = eta_max
    else if a > a* + tol:
        type = BANG_MINUS, eta = eta_min
    else:
        type = SINGULAR, eta = 0

    eta = clamp(eta, eta_min, eta_max)

    ds_arc = min(grid_step, s_end - s, distance_to_v_lim, distance_to_a*)
    tau = solveTimeForDistance(type, v, a, eta, a*, ds_arc)
    append arc
    s += ds_arc; t += tau; v = v_new; a = a_new
```

### Event detection

The solver steps forward until one of three events occurs:

1. **End of grid cell:** the default step size `ds`.
2. **Velocity limit hit:** for a `BANG_PLUS` arc, solve for the $\tau$ where
   $v(\tau) = v_{\text{lim}}$ and truncate the step there.
3. **Target acceleration reached:** for a `BANG` arc, solve for the $\tau$ where
   $a(\tau) = a_*$ and truncate the step there.

The smallest of these three is chosen as the arc length `ds_arc`. This keeps
arcs aligned with the true switching structure and prevents overshoot.

### Terminal braking

When the remaining distance $s_{\text{end}} - s$ is within the shortest feasible
stopping distance, the solver attempts to solve a single constant-$\eta$ arc
that lands exactly at $v = 0$. If no feasible arc exists, the candidate $a_*$ is
marked infeasible and receives a large cost penalty. The terminal arc is
usually recorded as `BANG_MINUS`, but if the guidance law happens to produce
$\eta = 0$ it may be recorded as `SINGULAR`.

### Infeasibility and guards

The state machine has several guards:

- If `eta_bounds` is infeasible, the candidate $a_*$ is rejected.
- If `a` would need to stop before the end of the path, a large penalty is added.
- If the final velocity is not within tolerance of the requested end velocity,
  a large penalty is added.
- All states are clamped to non-negative velocity and feasible acceleration/jerk
  bounds at record time.

---

## 5. Coalescing and Why It Matters

The raw output of the simulation is one arc per grid step. For a short path
with `constraintCacheSize = 200`, the raw list may contain hundreds of tiny
arcs. Before the WSS is returned, consecutive arcs with identical type and
control are coalesced into a single arc:

```cpp
if (coalesced.back().type == arc.type &&
    coalesced.back().eta == arc.eta &&
    coalesced.back().a_star == arc.a_star &&
    |coalesced.back().s1 - arc.s0| < 1e-12) {
    coalesced.back().s1 = arc.s1;
    coalesced.back().duration += arc.duration;
}
```

### Why coalescing is semantically required

The bang-singular-bang structure theorem predicts a small number of switching
events, not one event per grid cell. Coalescing:

- Removes grid-discretization artifacts.
- Produces the true analytic switching structure.
- Reduces `N_arcs` from $O(N_{\text{samples}})$ to $O(N_{\text{switches}})$.
- Makes binary search and polynomial evaluation cheaper.
- Is the step that turns a finite-difference simulation into an exact
  representation.

### What coalescing preserves

- Total arc length: `sum(arcs[i].s1 - arcs[i].s0) == s_f`.
- Total time: `sum(arcs[i].duration) == T`.
- Continuity: the merged arc's `(v0, a0)` are the start state of the first raw
  arc, and the polynomial family is unchanged across the merge.

### What a consumer should not do

Do **not** attempt to finite-difference across arc boundaries to get velocity
or acceleration. The arc list already contains the exact transition formulas.
Finite differencing will produce noise and may violate jerk/acceleration
bounds.

---

## 6. Interpreting an Arc List

A typical WSS for a rest-to-rest straight-line move with $w_a > 0$ looks like
this after coalescing:

```
[ BANG_PLUS | SINGULAR | BANG_MINUS | SINGULAR | BANG_PLUS ]
```

That is the canonical double-S profile:

1. **BANG_PLUS:** raise acceleration from $0$ to $+a_*$.
2. **SINGULAR:** cruise at $+a_*$ (velocity rises linearly).
3. **BANG_MINUS:** lower acceleration from $+a_*$ through $0$ to $-a_*$.
4. **SINGULAR:** cruise at $-a_*$ (velocity falls linearly).
5. **BANG_PLUS:** raise acceleration from $-a_*$ to $0$ at the final stop.

For $w_a = 0$ (pure time-optimal), $a_* \to a_{\max}$ and the `SINGULAR` arcs
shrink to points; the structure degenerates to bang-bang with jerk bounds,
similar to the time-optimal `SwitchingStructureRepresentation`.

For paths with curvature, a `WALL` arc appears wherever the velocity limit
$v_{\text{wall}}(s)$ is active:

```
[ BANG_PLUS | SINGULAR | WALL | SINGULAR | BANG_MINUS | SINGULAR | BANG_PLUS ]
```

### Visual inspection checklist

When you print an arc list, verify:

- The first arc starts at `s0 = 0` and the last arc ends at `s1 = path_length`.
- Arc types alternate in a physically meaningful way.
- `v0` is non-negative everywhere.
- For `BANG` arcs, `eta` has the correct sign (positive for `BANG_PLUS`,
  negative for `BANG_MINUS`).
- For `SINGULAR` arcs, `a_star` equals the planner's `optimalAStar()`.
- `t0` and `duration` are monotonic and contiguous.

---

## 7. State Reconstruction at Sample Time

When a consumer queries the WSS at time $t$, the implementation:

1. **Locates the arc:** binary search on `t0 + duration` to find `i` such that
   `arcs[i].t0 <= t <= arcs[i].t0 + arcs[i].duration`.
2. **Computes local time:** $\tau = t - \text{arcs}[i].t0$, clamped to
   `[0, duration]`.
3. **Reconstructs path state:**
   - `BANG_PLUS/BANG_MINUS:` cubic formulas
   - `SINGULAR:` quadratic formulas
   - `WALL:` quadrature inversion and wall derivative
4. **Maps to task space:**
   - `position = path.evaluateAtArcLength(s).position`
   - `velocity = tangent * v`
   - `acceleration = normal * curvature * v^2 + tangent * a`
   - `path_jerk = eta`
   - `jerk = jounce_vec * v^3 + 3 * curvature_vec * v * a + tangent * eta`
     (task-space jerk vector, where `jounce_vec` and `curvature_vec` are the
     arc-length derivative vectors from the path evaluation)
5. **Hard clamps:** velocity is clamped to the wall, acceleration to
   `[a_min, a_max]`, jerk to `[eta_min, eta_max]` at the current state.

### Clamps and guards

The sampling functions in `WeightedSwitchingStructure::locateAndState` apply
final hard clamps:

- `v <= v_lim(s)`
- `a in [a_min(s, v), a_max(s, v)]`
- `eta in [eta_min(s, v, a), eta_max(s, v, a)]`

These protect downstream consumers from tiny overshoots caused by:

- Discrete grid stepping in the solver
- Approximate braking distance on very short paths
- Wall-arc quadrature residual

**Implication:** the WSS sampled values are always **feasible**, even if the
underlying arc polynomials would slightly exceed a constraint at a particular
time. The arc list is the best approximation; the clamps are the safety net.

---

## 8. WALL Arcs and Quadrature

WALL arcs are the only arcs that are not polynomial in time. They follow the
velocity-limit curve $v_{\text{wall}}(s)$, which is computed by
`ConstraintEvaluator::velocityLimit` from:

- the feed rate
- the path-level maximum velocity
- the curvature-based centripetal limit $v \le \sqrt{a_{\text{cent}} / \kappa}$
- per-axis velocity limits projected onto the path tangent

The wall velocity is sampled pointwise at arc length; the WSS does not store a
parametric formula for $v_{\text{wall}}(s)$. Instead it stores the arc-length
span `[s0, s1]` and recomputes the wall on demand.

### Duration by quadrature

The time to traverse a wall arc is:

$$
\Delta t = \int_{s_0}^{s_1} \frac{1}{v_{\text{wall}}(s)} \, ds
$$

The WSS uses 8-point Gauss-Legendre quadrature with symmetric evaluation over
`[s0, s1]`.

### Inversion at sample time

To find $s$ for a given local time $\tau \in [0, \Delta t]$, the WSS solves:

$$
\int_{s_0}^{s} \frac{1}{v_{\text{wall}}(\sigma)} \, d\sigma = \tau
$$

by bisection on $s \in [s_0, s_1]$. The wall velocity and acceleration are then:

$$
\begin{aligned}
v(s) &= v_{\text{wall}}(s) \\
a(s) &= v(s) \frac{dv_{\text{wall}}}{ds}
\end{aligned}
$$

with the derivative estimated by finite difference and then clamped to feasible
acceleration bounds.

### Why this is acceptable

Wall arcs are typically short (curvature-constrained regions or feed-rate
plateaus). Quadrature over a short interval with a smooth $v_{\text{wall}}(s)$
gives high accuracy, and the bisection is only needed once per sample. For
real-time use the WSS can be sampled sparsely and the results interpolated by
the downstream `MotionPlan`.

---

## 9. Time-at-Arc-Length Inversion

The `timeAtArcLength(s)` method inverts the mapping $s(t)$ for any arc length
$s \in [0, s_f]$. This is the inverse of the usual `arcLength(t)` query and is
used by `AnalyticalSSRVelocityProfile` to convert the WSS into a sampled
`VelocityProfile` at uniform arc-length points.

### Inversion per arc type

- **BANG:** solve the cubic $s(\tau) = s_0 + v_0 \tau + \tfrac{1}{2} a_0 \tau^2
  + \tfrac{1}{6} \eta \tau^3 = s$ for $\tau$.
- **SINGULAR:** solve the quadratic
  $s_0 + v_0 \tau + \tfrac{1}{2} a_* \tau^2 = s$ for $\tau$.
- **WALL:** solve the quadrature equation by bisection, as in §8.

The result is `t = t0 + tau`. The function is monotone because $v > 0$ inside
every arc (enforced by construction and by the hard guards).

---

## 10. Lifetime, Ownership, and Thread Safety

### Path ownership

The WSS keeps the path alive via `std::shared_ptr<const Path>`:

```cpp
WeightedSwitchingStructure(
    std::shared_ptr<const Path> path,
    std::vector<Arc> arcs,
    CostWeights w,
    ConstraintEvaluator<Dim, T> evaluator,
    double optimalAStar);
```

This eliminates the lifetime hazard of the earlier const-reference design.
As long as a `std::shared_ptr<WSS>` exists, the path it references is valid.

### Profiler ownership

`ParetoTimeEnergyOptimalVelocityPlanner` holds a `std::shared_ptr<WSS>` after a
successful `computeProfile()`:

```cpp
auto wss = profiler.weightedSource();
```

The WSS remains valid until the planner is destroyed or `computeProfile()` is
called again. If you need to keep the WSS beyond the planner's lifetime, copy
the `std::shared_ptr`.

### Thread safety

The WSS is **read-only after construction**. Sampling methods (`position`,
`velocity`, `pathVelocity`, etc.) do not mutate state, so a single WSS instance
can be queried concurrently from multiple threads without locking. Any code
that iterates `arcs()` also only reads.

The WSS is **not** safe to modify while being sampled. Do not call
`setCostValue` or mutate arc data from one thread while another thread queries
it.

---

## 11. Downstream Consumers

### 11.1 `AnalyticalSSRVelocityProfile`

`ParetoTimeEnergyOptimalVelocityPlanner::computeProfile` returns a
`VelocityProfile` that wraps the WSS:

```cpp
auto profile = profiler.computeProfile(path, feedRate);
auto* ssr = dynamic_cast<AnalyticalSSRVelocityProfile<3, double>*>(profile.get());
if (ssr) {
    auto wss = std::dynamic_pointer_cast<WeightedSwitchingStructure<3, double>>(
        ssr->source());
}
```

This is how `MotionPlanBuilder` consumes the WSS transparently.

### 11.2 `MotionPlan`

`MotionPlan` stores the `AnalyticalTrajectorySource` and uses it for exact
sampling if available, falling back to the tabulated `VelocityProfile`
otherwise. This means the rest of the pipeline (step generation, extrusion,
Klipper translation) can work with exact trajectories without knowing that the
WSS exists.

### 11.3 `ExtrusionTrajectory`

All analytical extrusion compensation algorithms consume the WSS through
`ExtrusionTrajectory<Dim, T>`:

```cpp
ExtrusionTrajectory<3, double> etraj(*wss, extrusionRatio);
```

`ExtrusionTrajectory` converts each `WeightedArc` into polynomial coefficients
for $v(t)$ and annotates it with an extrusion ratio $\alpha_e$ (E-distance per
unit path distance). The resulting arcs are:

| WSS type | $v(\tau)$ polynomial | $c_0$ | $c_1$ | $c_2$ |
|---|---|---|---|---|
| `BANG` | $v_0 + a_0 \tau + \tfrac{1}{2}\eta \tau^2$ | `v0` | `a0` | `eta / 2` |
| `SINGULAR` | $v_0 + a_* \tau$ | `v0` | `a_star` | `0` |
| `WALL` | constant $v_{\text{wall}}$ | `v0` | `0` | `0` |

These coefficients are the basis for closed-form pressure advance, flow
adaptive heater control, and deconvolution.

### 11.4 `MotionTranslator` and Klipper

`MotionTranslator` in `tether_klipper` checks whether the `MotionPlan` carries
a WSS and, if so, uses the analytical extrusion path for pressure advance:

```cpp
auto wss = std::dynamic_pointer_cast<WeightedSwitchingStructure<Dim, T>>(
    plan.analyticalSource());
if (wss) {
    ExtrusionTrajectory<Dim, T> traj(*wss, extrusionRatio);
    // apply analytical pressure advance
}
```

This avoids the sampling errors of the legacy sampled-space pressure advance.

---

## 12. Common Pitfalls and Interpretation Rules

### 12.1 Do not treat `a0` as the arc acceleration for `SINGULAR` or `WALL` arcs

- `SINGULAR`: use `a_star`.
- `WALL`: acceleration is computed from the wall slope at sample time; the arc
  record's `a0` is not the constant acceleration.
- `BANG`: `a0` is the initial acceleration, and the acceleration varies
  linearly with time.

### 12.2 Do not assume `eta` is exactly `±eta_max`

The guidance law sets $\eta$ to the bang direction, but it is then clamped to
the feasible interval `[eta_min, eta_max]`. A `BANG_PLUS` arc may therefore have
a smaller positive `eta` than the global jerk limit if per-axis or curvature
constraints tighten the bound at that state.

### 12.3 Do not ignore the `WALL` type

A `WALL` arc has `eta = 0` and `a0` may also be `0`, but the arc is **not** a
constant-velocity cruise. The velocity and acceleration are determined by the
wall geometry and must be sampled through `pathVelocity(t)` and
`pathAcceleration(t)`, not from the arc record alone.

### 12.4 Do not finite-difference across arcs

The arc list is exact. Finite differencing between sample points or across
arc boundaries introduces noise, violates constraints, and defeats the purpose
of the analytical representation. Use the `pathJerk(t)` and `acceleration(t)`
methods directly.

### 12.5 Check for `nullptr` before using the WSS

```cpp
auto wss = profiler.weightedSource();
if (!wss) {
    // solve failed; handle gracefully
}
```

A `nullptr` indicates infeasible boundary conditions, zero feed rate, zero
acceleration limit, or an empty path.

### 12.6 Be careful with `arcs().back()`

The last arc may end at a non-zero acceleration on very short paths. The
terminal braking arc guarantees $v = 0$ at $s_f$, but the acceleration may not
be exactly $0$ because the solver solves a single constant-$\eta$ arc to stop.
For long paths the final acceleration is numerically close to zero; for short
paths it may be noticeably negative. Do not assume the trajectory ends with
`a = 0`.

### 12.7 Arc count is not the grid count

After coalescing the number of arcs is much smaller than
`constraintCacheSize`. Do not use the arc count to estimate solver resolution
or accuracy. Use `totalTime()`, `costValue()`, and sampling checks instead.

### 12.8 The WSS is tied to a specific set of cost weights

`optimalAStar()` and `costValue()` are only valid for the `CostWeights` stored
in the WSS (`weights()`). If you call `setWeights()` on the planner and
recompute, the old WSS is no longer valid.

---

## 13. API Reference

### `WeightedSwitchingStructure<Dim, T>`

Implements `AnalyticalTrajectorySource<Dim, T>`.

```cpp
// Time/length
T totalTime() const;
T totalLength() const;

// Task-space sampling
Vec<Dim, T> position(T t) const;
Vec<Dim, T> velocity(T t) const;
Vec<Dim, T> acceleration(T t) const;

// Path-space sampling
T arcLength(T t) const;
T timeAtArcLength(T s) const;
T pathVelocity(T t) const;
T pathAcceleration(T t) const;
T pathJerk(T t) const;
T curvature(T t) const;

// Traceability
SourceReference sourceRef(T t) const;
size_t segmentIndex(T t) const;
T segmentParameter(T t) const;

// Arc-list access
const std::vector<WeightedArc>& arcs() const;
WeightedArcType arcTypeAt(T t) const;

// Solver metadata
const CostWeights& weights() const;
double costValue() const;
double optimalAStar() const;

// Conversion
SampledVelocityProfile toVelocityProfile(size_t numSamples,
                                          T startAcceleration = T(0)) const;
```

### `WeightedArc`

```cpp
struct WeightedArc {
    WeightedArcType type;   // BANG_PLUS, BANG_MINUS, SINGULAR, WALL
    double s0, s1;          // arc-length span
    double t0;              // start time
    double v0, a0;          // start state
    double u0;              // NURBS parameter at s0
    double eta;             // constant jerk (BANG)
    double a_star;          // constant acceleration (SINGULAR)
    double duration;        // time span

    bool valid() const { return s1 > s0; }
    double length() const { return s1 - s0; }
};
```

### `ParetoTimeEnergyOptimalVelocityPlanner` accessors

```cpp
std::shared_ptr<WeightedSwitchingStructure<Dim, T>> weightedSource() const;
double costValue() const;
double optimalAStar() const;
CostWeights weights() const;
```

---

## 14. Implementation Files

| File | Purpose |
|---|---|
| `include/tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp` | `WeightedArc`, `WeightedSwitchingStructure`, solver |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalExtrusionTypes.hpp` | `ExtrusionTrajectory` — WSS to extrusion arc conversion |
| `include/tether/motion_planner/analytical/AnalyticalSSRVelocityProfile.hpp` | `VelocityProfile` adapter for `AnalyticalTrajectorySource` |
| `include/tether/motion_planner/analytical/AnalyticalTypes.hpp` | Shared interfaces (`AnalyticalTrajectorySource`, `EtaBounds`) |
| `include/tether/motion_planner/analytical/ConstraintEvaluator.hpp` | Wall velocity and constraint-bound evaluation |
| `docs/motion/ParetoTimeEnergyOptimal.md` | Theory and tuning of the Pareto planner |
| `docs/motion/WeightedSwitchingStructure.md` | This document |
