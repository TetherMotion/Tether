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
│  All Profilers (IVelocityProfiler)                          │
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

**Forward pass:** Starting from $v_{\text{fwd}}(0) = v_{\text{start}}$:

$$
v_{\text{fwd}}(s_i) = \min\left(v_{\text{max-accel}}, \; v_{\text{lim}}(s_i)\right)
$$

where $v_{\text{max-accel}}$ is the maximum velocity reachable from
$v_{\text{fwd}}(s_{i-1})$ over distance $\Delta s_i$ with jerk-limited
acceleration:

$$
v_{\text{max-accel}} = \max\left\{ v_1 : \Delta s_{\text{accel}}(v_{\text{fwd}}(s_{i-1}), v_1, a_{\max}, j_{\max}) \leq \Delta s_i \right\}
$$

This is solved by binary search (see T.7).

**Backward pass:** Starting from $v_{\text{bwd}}(L) = v_{\text{end}}$:

$$
v_{\text{bwd}}(s_{i-1}) = \min\left(v_{\text{max-decel}}, \; v_{\text{lim}}(s_{i-1})\right)
$$

where $v_{\text{max-decel}}$ is the maximum velocity that allows
decelerating to $v_{\text{bwd}}(s_i)$ over distance $\Delta s_i$:

$$
v_{\text{max-decel}} = \max\left\{ v_0 : \Delta s_{\text{decel}}(v_0, v_{\text{bwd}}(s_i), a_{\max}, j_{\max}) \leq \Delta s_i \right\}
$$

**Final profile:**

$$
v(s) = \min(v_{\text{fwd}}(s), v_{\text{bwd}}(s), v_{\text{lim}}(s))
$$

### (T.7) Binary Search for Maximum Velocity After Distance

**Problem:** Given starting velocity $v_0$, available distance $d$,
velocity ceiling $v_{\max}$, acceleration limit $a_{\max}$, and jerk
limit $j_{\max}$, find the maximum $v_1$ such that:

$$
\Delta s_{\text{accel}}(v_0, v_1, a_{\max}, j_{\max}) \leq d
$$

**Algorithm:** Binary search on $v_1 \in [v_0, v_{\max}]$:

```
v_low = v_0
v_high = v_max
for 60 iterations:
    v_mid = (v_low + v_high) / 2
    needed = computeAccelDistance(v_0, v_mid, a_max, j_max)
    if needed ≤ d:
        v_low = v_mid      // v_mid is achievable, try higher
    else:
        v_high = v_mid     // v_mid needs too much distance, try lower
return v_low
```

**Convergence:** After $k$ iterations, the uncertainty is
$(v_{\max} - v_0) / 2^k$. With 60 iterations and $v_{\max} - v_0 \leq
1000$ mm/s, the precision is $\sim 10^{-15}$ mm/s — far beyond
machine precision.

**Monotonicity:** $\Delta s_{\text{accel}}(v_0, v_1, a, j)$ is
strictly increasing in $v_1$ (more velocity change requires more
distance), so binary search is valid.

### (T.8) Time Integration

Given the final velocity profile $v(s)$, the time at each sample is
computed by trapezoidal integration:

$$
t(s_i) = t(s_{i-1}) + \frac{2 \Delta s_i}{v(s_{i-1}) + v(s_i)}
$$

using the average velocity over each interval. This is the trapezoidal
rule for $\int ds / v(s)$.

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

**Jerk-limited TOPP-RA:** Acceleration is computed from the velocity
change over time:

$$
a(s_i) = \frac{v(s_i) - v(s_{i-1})}{\Delta t_i}
$$

Jerk is computed from the acceleration change:

$$
j(s_i) = \frac{a(s_i) - a(s_{i-1})}{\Delta t_i}
$$

and clamped to $[-j_{\max}, j_{\max}]$ to remove numerical noise. By
construction of the jerk-limited distance function, the true jerk is
bounded by $j_{\max}$.

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

**The correct approach** — implemented in `JerkConstrainedVelocityProfiler`
— is to integrate jerk as a constraint *inside* the optimizer, using
the jerk-limited distance function (T.5) in place of the 2nd-order
kinematic equation (T.2).

---

## Summary of Profiler Properties

| Property | ToppraBasic | ToppraJerkConstrained | SCurve |
|---|---|---|---|
| **State** | $(s, \dot{s})$ | $(s, \dot{s}, \ddot{s})$ | per-piece |
| **Control** | $\ddot{s}$ (unbounded) | $\dddot{s}$ (bounded) | 7-phase |
| **Distance eq.** | $v^2 = v_0^2 + 2a\Delta s$ | S-curve distance (T.5) | S-curve (T.4) |
| **Time-optimal** | Yes | Yes (subject to jerk) | No |
| **Jerk bounded** | No | Yes ($\leq j_{\max}$) | Yes ($\leq j_{\max}$) |
| **Accel continuous** | No | Yes | Mostly (per-piece) |
| **Global constraints** | Yes | Yes | No (midpoint only) |
| **Compute complexity** | $O(N)$ | $O(N \log(v_{\max}/\varepsilon))$ | $O(N)$ |

---

## See Also

| Document | Scope |
|---|---|
| [VelocityProfilerSelection.md](VelocityProfilerSelection.md) | When to choose each profiler |
| [CertificationPath.md](CertificationPath.md) | Certified curvature bounds used in $v_{\text{lim}}$ |
| [MotionChain.md](MotionChain.md) | Full motion pipeline |
| [Architecture.md](Architecture.md) | Motion planner architecture |
| `SCurveProfile.hpp` | 7-phase S-curve implementation |
| `JerkConstrainedVelocityProfiler.hpp` | Jerk-integrated TOPP-RA implementation |
