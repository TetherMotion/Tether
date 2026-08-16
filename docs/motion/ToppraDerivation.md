# TOPP-RA Derivation

This document derives the mathematics behind Tether's velocity
profilers: the basic 2nd-order TOPP-RA, the jerk-limited 3rd-order
variant, and the 7-phase S-curve. It is organized in three parts:
**goals** (what we want to achieve), **how it is achieved** (the
algorithmic approach), and **mathematical derivation** (the formulas
and proofs).

Equation numbers `(T.x)` are new to this document. References to
`(G.x)` and `(M.x)` point to `GeometryFoundations.md` and
`BlendingAlgorithm.md` respectively.

---

## Part I: Goals

### Goal 1: Time-Optimality

Given a path $P(s)$ parameterized by arc length $s \in [0, L]$ and a
set of kinematic constraints, find the velocity profile $v(s)$ that
minimizes total traversal time:

$$
T = \int_0^L \frac{ds}{v(s)}
$$

subject to all constraints being satisfied at every point.

**Why it matters:** Minimizing traversal time maximizes throughput
without violating machine limits. A non-optimal profile wastes time by
being unnecessarily conservative.

### Goal 2: Constraint Satisfaction

The profile must satisfy, at every point $s$:

1. **Velocity limit:** $v(s) \leq v_{\text{lim}}(s)$
   - Feed rate: $v \leq v_{\text{feed}}$
   - Curvature (centripetal): $v^2 \kappa(s) \leq a_{\text{cent}}$
   - Per-axis: $v |t_i(s)| \leq v_{\text{axis},i}$ for each axis $i$

2. **Acceleration limit:** $|a(s)| \leq a_{\text{max}}(s)$
   - Path-level: $|a| \leq a_{\text{path}}$
   - Per-axis: $|a_i| \leq a_{\text{axis},i}$

3. **Jerk limit** (if jerk-limited profiler): $|j(s)| \leq j_{\text{max}}$

4. **Boundary conditions:** $v(0) = v_{\text{start}}$, $v(L) = v_{\text{end}}$

**Why it matters:** Violating velocity limits causes step loss or
excessive mechanical stress. Violating acceleration limits causes
vibration and missed steps. Violating jerk limits causes discontinuous
acceleration, which excites resonances.

### Goal 3: Smoothness (for jerk-limited variants)

The acceleration profile $a(t)$ should be continuous (no step changes).
This requires jerk to be bounded, not infinite.

**Why it matters:** Discontinuous acceleration (bang-bang) causes
mechanical ringing in lightweight machines and pressure spikes in
extrusion systems. Continuous acceleration produces smoother motion
and better print quality.

---

## Part II: How It Is Achieved

### Overview

```
┌─────────────────────────────────────────────────────────────┐
│  All Profilers (VelocityProfiler)                          │
│                                                             │
│  1. Sample path at uniform arc-length intervals             │
│  2. Compute velocity limit curve v_lim(s)                   │
│  3. Forward pass (acceleration from start)                  │
│  4. Backward pass (deceleration to end)                     │
│  5. Final profile: v(s) = min(forward, backward, v_lim)     │
│  6. Time integration: t(s) = ∫ ds / v(s)                   │
└──────────────────────────┬──────────────────────────────────┘
                           │
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
┌──────────────────┐ ┌─────────────────┐ ┌──────────────────┐
│  ToppraBasic     │ │ ToppraJerkLimit │ │  SCurve          │
│                  │ │                 │ │                  │
│  v² = v₀² + 2aΔs │ │ Δs = S-curve   │ │  Per-piece       │
│  (2nd-order)     │ │   distance      │ │  7-phase S-curve │
│  Bang-bang accel │ │ (3rd-order)     │ │  Not optimal     │
│  Infinite jerk   │ │ Bounded jerk    │ │  Bounded jerk    │
└──────────────────┘ └─────────────────┘ └──────────────────┘
```

### The Velocity Limit Curve $v_{\text{lim}}(s)$

All profilers compute the same velocity limit curve, which is the
maximum allowed velocity at each arc length:

$$
v_{\text{lim}}(s) = \min\left(v_{\text{feed}}, \; \sqrt{\frac{a_{\text{cent}}}{\kappa(s)}}, \; \min_i \frac{v_{\text{axis},i}}{|t_i(s)|}, \; v_{\text{path}}\right)
$$

where:
- $v_{\text{feed}}$ is the commanded feed rate
- $\kappa(s)$ is the **certified** maximum curvature (from
  `CertifiedCurvatureSampler` — see [CertificationPath.md](CertificationPath.md))
- $t_i(s)$ is the $i$-th component of the unit tangent vector
- $v_{\text{path}}$ is the global path velocity limit

**Curvature certification is critical here:** using an uncertified
curvature sample could miss a curvature peak between samples, leading
to excessive velocity and centripetal acceleration violation.

### The Forward-Backward Pass Structure

All TOPP-RA variants share the same structure:

1. **Forward pass:** Sweep from $s=0$ to $s=L$. At each sample,
   compute the maximum velocity reachable from the previous sample,
   respecting acceleration (and jerk, if applicable) limits.

2. **Backward pass:** Sweep from $s=L$ to $s=0$. At each sample,
   compute the maximum velocity that allows decelerating to the next
   sample's velocity.

3. **Final profile:** $v(s) = \min(v_{\text{fwd}}(s), v_{\text{bwd}}(s), v_{\text{lim}}(s))$

The forward pass ensures we can accelerate from the start; the
backward pass ensures we can decelerate to the end; the minimum of
both ensures both constraints are satisfied simultaneously.

### The Key Difference: 2nd-Order vs 3rd-Order

The difference between `ToppraBasic` and `ToppraJerkConstrained` is the
equation used in the forward/backward passes:

| Profiler | State | Control | Distance Equation |
|---|---|---|---|
| ToppraBasic | $(s, \dot{s})$ | $\ddot{s}$ (unbounded jerk) | $v^2 = v_0^2 + 2a\Delta s$ |
| ToppraJerkConstrained | $(s, \dot{s}, \ddot{s})$ | $\dddot{s}$ (bounded jerk) | $\Delta s = d_{\text{sc}}(v_0, v_1, a, j)$ |

where $d_{\text{sc}}$ is the jerk-limited S-curve distance function
(derived below).

---

## Part III: Mathematical Derivation

### (T.1) The Path Parameterization

The path $P(s)$ is parameterized by arc length $s \in [0, L]$, where
$L$ is the total path length. At each point:

- **Position:** $P(s) \in \mathbb{R}^n$
- **Tangent:** $T(s) = dP/ds$ (unit vector)
- **Curvature:** $\kappa(s) = \|dT/ds\|$

The velocity profile $v(s) = \dot{s}(s)$ gives the speed at each arc
length. The time to traverse the path is:

$$
T = \int_0^L \frac{ds}{v(s)}
$$

### (T.2) The 2nd-Order Kinematic Equation (ToppraBasic)

**Derivation:** For constant acceleration $a$ over distance $\Delta s$,
the kinematic equation is:

$$
v_1^2 = v_0^2 + 2a \cdot \Delta s
$$

Solving for $v_1$:

$$
v_1 = \sqrt{v_0^2 + 2a \cdot \Delta s}
$$

**Forward pass:** Starting from $v_{\text{fwd}}(0) = v_{\text{start}}$:

$$
v_{\text{fwd}}(s_i) = \min\left(\sqrt{v_{\text{fwd}}(s_{i-1})^2 + 2 a_{\text{max}} \Delta s_i}, \; v_{\text{lim}}(s_i)\right)
$$

**Backward pass:** Starting from $v_{\text{bwd}}(L) = v_{\text{end}}$:

$$
v_{\text{bwd}}(s_{i-1}) = \min\left(\sqrt{v_{\text{bwd}}(s_i)^2 + 2 a_{\text{max}} \Delta s_i}, \; v_{\text{lim}}(s_{i-1})\right)
$$

**Final profile:**

$$
v(s) = \min(v_{\text{fwd}}(s), v_{\text{bwd}}(s), v_{\text{lim}}(s))
$$

**Properties:**
- **Time-optimal:** This is the classic TOPP-RA result. The profile is
  the time-optimal solution subject to velocity and acceleration
  constraints.
- **Bang-bang acceleration:** The acceleration switches between
  $+a_{\text{max}}$ and $-a_{\text{max}}$ at the points where
  $v_{\text{fwd}}$ and $v_{\text{bwd}}$ cross. Jerk is infinite at
  these switching points.
- **Acceleration discontinuity:** $a(s)$ has step changes.

### (T.3) The Velocity Limit Curve

The velocity limit at arc length $s$ is:

$$
v_{\text{lim}}(s) = \min\left(v_{\text{feed}}, \; v_{\text{curv}}(s), \; v_{\text{axis}}(s), \; v_{\text{path}}\right)
$$

where:

**Curvature limit (centripetal acceleration):**

$$
v_{\text{curv}}(s) = \sqrt{\frac{a_{\text{cent}}}{\kappa(s)}} \qquad \text{if } \kappa(s) > 0
$$

This ensures $v^2 \kappa \leq a_{\text{cent}}$, i.e., the centripetal
acceleration does not exceed the limit.

**Per-axis velocity limit:**

$$
v_{\text{axis}}(s) = \min_i \frac{v_{\text{axis},i}}{|T_i(s)|} \qquad \text{if } |T_i(s)| > 0
$$

This ensures the velocity component along each axis does not exceed
that axis's maximum velocity. $T_i(s)$ is the $i$-th component of the
unit tangent vector.

**Per-axis acceleration limit:**

$$
a_{\text{max}}(s, v) = \min\left(a_{\text{path}}, \; \min_i \frac{a_{\text{axis},i}}{|T_i(s)|}\right)
$$

subject to the centripetal acceleration not exceeding its limit:
$v^2 \kappa(s) \leq a_{\text{cent}}$.

### (T.4) The 7-Phase S-Curve Profile

The S-curve is a jerk-limited acceleration profile consisting of 7
phases:

| Phase | Jerk $j$ | Accel $a$ | Velocity $v$ | Duration |
|---|---|---|---|---|
| 1: Jerk+ | $+j_{\max}$ | $0 \to +a_{\max}$ | increasing | $t_j = a_{\max}/j_{\max}$ |
| 2: Const Accel | $0$ | $+a_{\max}$ | increasing | $t_c$ (variable) |
| 3: Jerk- | $-j_{\max}$ | $+a_{\max} \to 0$ | increasing | $t_j$ |
| 4: Cruise | $0$ | $0$ | $v_{\max}$ | $t_v$ (variable) |
| 5: Jerk- | $-j_{\max}$ | $0 \to -a_{\max}$ | decreasing | $t_j$ |
| 6: Const Decel | $0$ | $-a_{\max}$ | decreasing | $t_c$ (variable) |
| 7: Jerk+ | $+j_{\max}$ | $-a_{\max} \to 0$ | decreasing | $t_j$ |

**Phase equations** (for a phase with constant jerk $j$, starting at
$(p_0, v_0, a_0)$):

$$
\begin{aligned}
p(t) &= p_0 + v_0 t + \frac{1}{2} a_0 t^2 + \frac{1}{6} j t^3 \\
v(t) &= v_0 + a_0 t + \frac{1}{2} j t^2 \\
a(t) &= a_0 + j t \\
j(t) &= j \quad (\text{constant})
\end{aligned}
$$

**Jerk time:** $t_j = a_{\max} / j_{\max}$ — the time to ramp
acceleration from 0 to $a_{\max}$ (or vice versa) at jerk $j_{\max}$.

**Velocity change during jerk phase:**

$$
\Delta v_{\text{jerk}} = \frac{1}{2} j_{\max} t_j^2 = \frac{a_{\max}^2}{2 j_{\max}}
$$

### (T.5) Jerk-Limited Distance Function

**Problem:** Compute the distance $\Delta s$ required to change velocity
from $v_0$ to $v_1$ (where $v_1 > v_0$), with maximum acceleration
$a_{\max}$ and maximum jerk $j_{\max}$.

**Case 1: Small velocity change** ($\Delta v \leq 2 \Delta v_{\text{jerk}}$)

The acceleration never reaches $a_{\max}$. The profile consists of two
symmetric jerk phases (phase 1 + phase 3, no phase 2).

The jerk time for each phase is:

$$
t = \sqrt{\frac{\Delta v}{j_{\max}}}
$$

The distance is:

$$
\Delta s = 2 v_0 t + j_{\max} t^3
$$

**Derivation:** During the first jerk phase ($j = +j_{\max}$, starting
at $a=0, v=v_0$):

$$
v(t) = v_0 + \frac{1}{2} j_{\max} t^2, \quad p(t) = v_0 t + \frac{1}{6} j_{\max} t^3
$$

After time $t$, velocity is $v_0 + \frac{1}{2} j_{\max} t^2 = v_0 +
\Delta v / 2$. The second jerk phase ($j = -j_{\max}$, starting at
$a = j_{\max} t$, $v = v_0 + \Delta v / 2$) brings acceleration back
to 0 and velocity to $v_1 = v_0 + \Delta v$. By symmetry, the distance
for the second phase equals the first plus the additional velocity
contribution. The total is $2 v_0 t + j_{\max} t^3$.

**Case 2: Full acceleration profile** ($\Delta v > 2 \Delta v_{\text{jerk}}$)

The acceleration reaches $a_{\max}$. The profile has all three
acceleration phases (1, 2, 3).

**Constant-acceleration phase duration:**

$$
t_c = \frac{\Delta v - 2 \Delta v_{\text{jerk}}}{a_{\max}}
$$

**Phase 1 distance** (jerk+, $j = +j_{\max}$, $t = t_j$):

$$
d_1 = v_0 t_j + \frac{1}{6} j_{\max} t_j^3
$$

Velocity at end of phase 1: $v_1' = v_0 + \Delta v_{\text{jerk}}$

**Phase 2 distance** (const accel, $a = a_{\max}$, $t = t_c$):

$$
d_2 = v_1' t_c + \frac{1}{2} a_{\max} t_c^2
$$

Velocity at end of phase 2: $v_2' = v_1' + a_{\max} t_c$

**Phase 3 distance** (jerk-, $j = -j_{\max}$, $t = t_j$):

$$
d_3 = v_2' t_j + \frac{1}{2} a_{\max} t_j^2 - \frac{1}{6} j_{\max} t_j^3
$$

**Total distance:**

$$
\boxed{\Delta s = d_1 + d_2 + d_3}
$$

**Deceleration distance:** By symmetry, the distance to decelerate
from $v_0$ to $v_1$ (where $v_1 < v_0$) is:

$$
\Delta s_{\text{decel}}(v_0, v_1) = \Delta s_{\text{accel}}(v_1, v_0)
$$

### (T.6) The 3rd-Order Forward/Backward Pass (ToppraJerkConstrained)

The jerk-limited profiler replaces the 2nd-order kinematic equation
with the jerk-limited distance function (T.5).

#### Pre-WI-8 (stateless, suboptimal)

The original implementation used the symmetric S-curve distance
function (T.5), which implicitly forces $a = 0$ at every sample point.
This makes the profile increasingly suboptimal as `numSamples` grows:
each sample pays a full jerk-ramp-up + jerk-ramp-down cost, even when
the optimal trajectory would hold $a = a_{\max}$ across many samples.

#### Post-WI-8b (state-carrying, approximately time-optimal, no post-hoc smoothing)

The state-carrying implementation carries the acceleration as state in
both passes, using the generalized distance function (T.5b):

$$
\Delta s = d_{\text{sc,state}}(v_0, a_0, v_1, a_{\max}, j_{\max})
$$

which plans a jerk-bang-bang trajectory from $(v_0, a_0)$ to $(v_1, 0)$
without forcing $a_0 = 0$.

**Forward pass:** Starting from $(v_{\text{fwd}}(0), a_{\text{fwd}}(0)) =
(v_{\text{start}}, a_{\text{start}})$:

$$
(v_{\text{fwd}}(s_i), a_{\text{fwd}}(s_i)) = \text{maxVelocityWithState}(v_{\text{fwd}}(s_{i-1}), a_{\text{fwd}}(s_{i-1}), \Delta s_i, v_{\text{cap}}(s_i), a_{\max}, j_{\max})
$$

where $v_{\text{cap}}(s_i) = \min(v_{\text{lim}}(s_i), v_{\text{bwd}}(s_i))$
is the velocity ceiling (WI-8b.3). Using the backward velocity as part
of the ceiling makes the forward pass join the backward curve at the
velocity level, eliminating the need for post-hoc acceleration smoothing.

**Backward pass:** Starting from $(v_{\text{bwd}}(L), a_{\text{bwd}}(L)) =
(v_{\text{end}}, 0)$:

$$
(v_{\text{bwd}}(s_{i-1}), a_{\text{bwd}}(s_{i-1})) = \text{maxEntryVelocityWithState}(v_{\text{bwd}}(s_i), a_{\text{bwd}}(s_i), \Delta s_i, v_{\text{lim}}(s_{i-1}), a_{\max}, j_{\max})
$$

**Shed-acceleration ceiling constraint (WI-8b.2):** A state $(v_1, a_1)$
with $a_1 > 0$ is only feasible w.r.t. the ceiling if the acceleration can
be shed before $v$ exceeds $v_{\text{cap}}$:

$$
\frac{a_1^2}{2 j_{\max}} \leq v_{\text{cap}} - v_1
$$

When the uncapped trajectory would violate this, `maxVelocityWithState`
finds the point where the trajectory crosses the shed boundary
$v + a^2/(2j_{\max}) = v_{\text{cap}}$ and follows the boundary
($j = -j_{\max}$) for the remaining distance. This makes ceiling arrival
tangent ($a \to 0$ as $v \to v_{\text{cap}}$) and carries a non-zero
acceleration right up to the shed boundary.

**Feasibility domain (WI-8b.1):** `computeAccelDistanceWithState` returns
$\infty$ when $a_0 > 0$ and the required velocity change $\Delta v$ is less
than the velocity shed by bringing acceleration to zero ($a_0^2/(2j_{\max})$).
For feasible cases with $a_0 > 0$, the function sheds $a_0$ first
($j = -j_{\max}$) before solving the $a_0 = 0$ problem.

**No switching-point smoothing (WI-8b.3):** The velocity-level joining
($v_{\text{cap}} = \min(v_{\text{lim}}, v_{\text{bwd}})$ + shed constraint)
makes the forward state shed acceleration to 0 before hitting $v_{\text{cap}}$,
so it joins the backward curve smoothly at switching points. The old
post-hoc forward-backward acceleration smoothing pass (which smoothed the
entire profile) has been replaced with a **light jerk-limited smoothing**
that only enforces $|\Delta a / \Delta t| \leq j_{\max}$ on the acceleration
profile without changing the velocity. This is needed because the shed
constraint is per-sample and can cause a jerk spike at the sample where it
first kicks in. The velocity profile remains feasible (from the
velocity-level joining); only the acceleration is smoothed for jerk
feasibility.

**Final profile:**

$$
v(s) = v_{\text{fwd}}(s) \leq \min(v_{\text{lim}}(s), v_{\text{bwd}}(s))
$$

The acceleration at each point is the carried analytic state from the
binding pass: $a_{\text{fwd}}$ in the accel region, $a_{\text{bwd}}$ in
the decel region, $0$ at velocity-limited cruise. The jerk is computed
from the acceleration change over time and reported truthfully (WI-3:
not clamped).

### (T.5b) State-Aware Jerk-Limited Distance Function

Generalizes (T.5) to carry the acceleration as state. The trajectory
from $(v_0, a_0)$ to $(v_1, 0)$ consists of up to 3 phases:

1. **Jerk ramp:** $j = +j_{\max}$, $a$ goes from $a_0$ to $a_{\max}$ (or $a_{\text{peak}}$)
2. **Constant accel:** $a = a_{\max}$ (trapezoidal case only)
3. **Jerk ramp:** $j = -j_{\max}$, $a$ goes from $a_{\max}$ (or $a_{\text{peak}}$) to $0$

**Trapezoidal case** ($\Delta v \geq (a_{\max}^2 - a_0^2)/(2j_{\max}) + a_{\max}^2/(2j_{\max})$):

$$
t_1 = \frac{a_{\max} - a_0}{j_{\max}}, \quad t_3 = \frac{a_{\max}}{j_{\max}}, \quad t_2 = \frac{\Delta v - \Delta v_1 - \Delta v_3}{a_{\max}}
$$

**Triangular case** ($\Delta v < $ threshold):

$$
a_{\text{peak}} = \sqrt{j_{\max} \Delta v + a_0^2 / 2}
$$

The distance is the sum of the phase distances. See
`SCurveProfile::computeAccelDistanceWithState` for the full formula.

### (T.7) Newton Iteration for Maximum Velocity After Distance (WI-P1)

**Problem:** Given starting velocity $v_0$, available distance $d$,
velocity ceiling $v_{\max}$, acceleration limit $a_{\max}$, and jerk
limit $j_{\max}$, find the maximum $v_1$ such that:

$$
\Delta s_{\text{accel}}(v_0, v_1, a_{\max}, j_{\max}) \leq d
$$

**Algorithm (WI-P1):** Newton's method with bisection fallback. The
derivative of $\Delta s_{\text{accel}}$ with respect to $v_1$ is
analytically known (piecewise: triangular vs. trapezoidal), so Newton
converges in 2–4 iterations. Bisection is used as a fallback if Newton
diverges or goes out of bounds.

```
v_low = v_0, v_high = v_max
v_1 = initial_guess (linear approximation)
for up to 20 iterations:
    f = computeAccelDistance(v_0, v_1, a, j) - d
    f' = computeAccelDistanceDerivative(v_0, v_1, a, j)
    update bisection bracket from sign of f
    if |f| < tolerance: break
    if f' > 0 and v_low < v_1 - f/f' < v_high:
        v_1 = v_1 - f/f'    // Newton step
    else:
        v_1 = (v_low + v_high) / 2    // bisection fallback
return v_1
```

**Convergence:** Newton converges quadratically (2–4 iterations for
$10^{-10}$ relative precision). Bisection converges linearly (20
iterations for the same precision). The hybrid approach is both fast
and robust.

**Monotonicity:** $\Delta s_{\text{accel}}(v_0, v_1, a, j)$ is
strictly increasing in $v_1$ (more velocity change requires more
distance), so both Newton and bisection are valid.

### (T.8) Time Integration

Given the final velocity and acceleration profile, the time at each
sample is computed using the constant-jerk formula (WI-8b.3):

$$
\Delta t_i = \frac{2 (v_i - v_{i-1})}{a_{i-1} + a_i}
$$

This is exact when jerk is constant over the interval (derived from
$v_1 - v_0 = a_{\text{avg}} \cdot \Delta t$ where $a_{\text{avg}} =
(a_0 + a_1)/2$ for constant jerk). The trapezoidal rule
($\Delta t = 2 \Delta s / (v_0 + v_1)$) is only exact for constant
acceleration and underestimates $\Delta t$ by up to 30% in the ramp-up
phase ($v_0 \approx 0$), making $T$ appear below the theoretical optimum.

**Fallbacks:**
- When $a_0 + a_1 \approx 0$ (cruise or sign-change): trapezoidal rule.
- When starting from rest ($v_0 \approx 0$, $a_0 \approx 0$): jerk-only
  ramp formula $\Delta t = \sqrt{6 \Delta s / |a_1|}$.

**Inverse mapping** (time → arc length): Binary search on the time
array, then linear interpolation for arc length.

### (T.9) Acceleration and Jerk Computation

**Basic TOPP-RA:** Acceleration is computed post-hoc from the velocity
profile:

$$
a(s_i) = \frac{v(s_i) - v(s_{i-1})}{t(s_i) - t(s_{i-1})}
$$

This gives a piecewise-constant acceleration approximation. Jerk is
not computed (theoretically infinite at switching points).

**Jerk-limited TOPP-RA (post-WI-8b):** Acceleration is the carried
analytic state from the binding pass (forward or backward), not computed
post-hoc from velocity differences:

$$
a(s_i) = \begin{cases}
a_{\text{fwd}}(s_i) & \text{if forward pass binds} \\
a_{\text{bwd}}(s_i) & \text{if backward pass binds} \\
0 & \text{if velocity limit binds (cruise)}
\end{cases}
$$

Jerk is computed from the acceleration change over time:

$$
j(s_i) = \frac{a(s_i) - a(s_{i-1})}{\Delta t_i}
$$

and reported truthfully (WI-3: not clamped). By construction of the
jerk-limited distance function and the shed-acceleration constraint
(WI-8b.2), the true jerk is bounded by $j_{\max}$.

**S-curve profiler:** Acceleration and jerk come directly from the
7-phase S-curve's closed-form equations (T.4).

### (T.10) Per-Axis Constraint Projection

The path velocity $v$ and path acceleration $a$ relate to per-axis
quantities via the tangent vector $T(s)$:

$$
v_i(s) = v(s) \cdot T_i(s), \qquad a_i(s) = a(s) \cdot T_i(s) + v(s)^2 \cdot \kappa_i(s)
$$

The per-axis velocity constraint $|v_i| \leq v_{\text{axis},i}$ becomes:

$$
v \leq \frac{v_{\text{axis},i}}{|T_i|} \qquad \text{for each axis } i
$$

The per-axis acceleration constraint is more complex because it
includes both tangential and centripetal components. The profiler
handles this by:
1. Computing the centripetal acceleration: $a_{\text{cent}} = v^2 \kappa$
2. Checking $a_{\text{cent}} \leq a_{\text{cent,max}}$
3. Computing available tangential acceleration: $a_{\text{tan}} = \min(a_{\text{path}}, \min_i a_{\text{axis},i} / |T_i|)$

### (T.11) Time-Optimality of TOPP-RA

**Theorem:** The profile $v(s) = \min(v_{\text{fwd}}, v_{\text{bwd}}, v_{\text{lim}})$
is time-optimal subject to the velocity and acceleration constraints.

**Proof sketch:** The forward pass gives the maximum velocity reachable
from the start at each point. The backward pass gives the maximum
velocity that allows stopping by the end. The minimum of both is the
maximum velocity that satisfies both boundary conditions. Any faster
profile would violate either the forward or backward constraint.

For the jerk-limited variant, the same argument applies with the
jerk-limited distance function replacing the 2nd-order kinematic
equation. The profile is time-optimal *subject to the jerk constraint*.

### (T.12) Why Not Post-Hoc Smoothing?

An earlier version of Tether applied S-curve smoothing *after* the
TOPP-RA optimization. This was removed because:

1. **Breaks feasibility:** The smoothed trajectory was never checked
   against $v_{\text{lim}}(s)$. The S-curve transitions could push
   velocity above the curvature limit at points between the original
   TOPP-RA samples.

2. **Breaks optimality:** The smoothed trajectory is neither
   time-optimal (smoothing added time) nor jerk-optimal (the smoothing
   was applied after the fact, not as a constraint).

3. **Desynchronizes extrusion:** Any downstream consumer (extrusion
   compensation, step generation) that derived timing from the TOPP-RA
   profile would be desynchronized by the post-hoc smoothing.

**The correct approach** — implemented in `JerkConstrainedTOPPRA`
— is to integrate jerk as a constraint *inside* the optimizer, using
the jerk-limited distance function (T.5) in place of the 2nd-order
kinematic equation (T.2).

---

## Summary of Profiler Properties

| Property | ToppraBasic | ToppraJerkConstrained (post-WI-8b) | SCurve |
|---|---|---|---|
| **State** | $(s, \dot{s})$ | $(s, \dot{s}, \ddot{s})$ carried | per-piece |
| **Control** | $\ddot{s}$ (unbounded) | $\dddot{s}$ (bounded) | 7-phase |
| **Distance eq.** | $v^2 = v_0^2 + 2a\Delta s$ | State-aware S-curve (T.5b) | S-curve (T.4) |
| **Time-optimal** | Yes | Approx. (subject to jerk + grid) | No |
| **numSamples-dep.** | No | No (post-WI-8b) | N/A |
| **Jerk bounded** | No | Yes ($\leq j_{\max}$) | Yes ($\leq j_{\max}$) |
| **Accel continuous** | No | Yes (velocity-level joining + light jerk smoothing, WI-8b.3) | Mostly (per-piece) |
| **Post-hoc smoothing** | No | Light (jerk-only on accel, WI-8b.3) | N/A |
| **Global constraints** | Yes | Yes | No (midpoint only) |
| **Compute complexity** | $O(N)$ | $O(N)$ (Newton, WI-P1) | $O(N)$ |

---

## See Also

| Document | Scope |
|---|---|
| [VelocityProfilerSelection.md](VelocityProfilerSelection.md) | When to choose each profiler |
| [CertificationPath.md](CertificationPath.md) | Certified curvature bounds used in $v_{\text{lim}}$ |
| [MotionChain.md](MotionChain.md) | Full motion pipeline |
| [Architecture.md](Architecture.md) | Motion planner architecture |
| `SCurveProfile.hpp` | 7-phase S-curve implementation |
| `JerkConstrainedTOPPRA.hpp` | Jerk-integrated TOPP-RA implementation |
