# Configurable Energy/Time-Optimal Path Tracking for NURBS Chains

**Manual for `ParetoTimeEnergyOptimalVelocityPlanner`**

## Overview

This document describes the analytical TOPPRA-family solver with a tunable
cost:

$$
J = \int_0^T \left[ w_t + w_a \cdot a(t)^2 \right] dt
$$

- $w_t \to \text{large}$: time-optimal (recovers bang-bang TOPPRA)
- $w_a \to \text{large}$: energy-optimal (min-squared-acceleration, smooth)
- Intermediate: configurable compromise

The key insight from Pontryagin's maximum principle is that the optimal
solution consists of only **two primitive arc types**:

1. **BANG arcs** ($\eta = \pm\eta_{\max}$): cubic-in-time transitions
2. **SINGULAR arcs** ($\eta = 0$, $a = a_* = \text{const}$): constant-acceleration cruising

The singular acceleration level $a_*$ is the single optimization parameter,
selected by minimizing the closed-form scalar cost $J(a_*)$ via golden-section
search.

## Table of Contents

1. [The Weighted Cost Problem](#1-the-weighted-cost-problem)
2. [Pontryagin Analysis](#2-pontryagin-analysis)
3. [Structure Theorem](#3-structure-theorem)
4. [Constraint Handling](#4-constraint-handling)
5. [Data Structures](#5-data-structures)
6. [Solver Algorithm](#6-solver-algorithm)
7. [Output Representation (WSS)](#7-output-representation-wss)
8. [Sampling](#8-sampling)
9. [Certification and Error Bounds](#9-certification-and-error-bounds)
10. [Tuning Guide](#10-tuning-guide)

---

## 1. The Weighted Cost Problem

### 1.1 Problem Statement

Given a fixed path $C(u)$ and traversal $s(t)$ (arc length over time):

$$
J = \int_0^T \left[ w_t + w_a \cdot a(t)^2 \right] dt, \quad a(t) = \frac{d^2s}{dt^2}
$$

- $w_t \geq 0$: weight on time (dimensionless)
- $w_a \geq 0$: weight on acceleration energy (units: time$^3$)

**Extremes:**

| Weight setting | Behavior |
|---|---|
| $w_a = 0$ | Pure time-optimal (TOPPRA; bang-bang jerk) |
| $w_t = 0$ | Ill-posed (infinite time crawling) — **always keep $w_t > 0$** |
| Both > 0 | Configurable compromise (the regime this planner targets) |

### 1.2 Why Not Lowpass-Filter a Time-Optimal Solution?

Filtering a bang-bang profile after the fact:

1. **Violates constraints** — convolution is nonlocal; peaks shift and overshoot
2. **Not optimal for any cost** — it is an ad-hoc heuristic
3. **Loses certification** — the smoothed trajectory was never checked against constraints

The correct approach puts the energy term **into** the optimization, which
changes the optimal **structure**, not just the numbers.

### 1.3 Dynamics (Arc-Length Domain)

$$
t' = \frac{1}{v}, \quad v' = \frac{a}{v}, \quad a' = \frac{\eta}{v}
$$

where $'$ = $d/ds$ and $\eta = da/dt$ is the jerk control input.

Running cost per unit arc length: $dt = ds/v$, so

$$
J = \int_0^{s_f} \frac{w_t + w_a \cdot a^2}{v} \, ds
$$

---

## 2. Pontryagin Analysis

### 2.1 Hamiltonian

Costates: $\lambda_t, \lambda_v, \lambda_a$ (functions of $s$). Running cost
$L = (w_t + w_a a^2)/v$:

$$
H = \frac{\lambda_t}{v} + \frac{\lambda_v \cdot a}{v} + \frac{\lambda_a \cdot \eta}{v} + \frac{w_t + w_a a^2}{v}
$$

### 2.2 Costate Equations

$$
\dot{\lambda}_t = -\frac{\partial H}{\partial t} = 0 \quad \Rightarrow \quad \lambda_t = \text{const} =: c
$$

$$
\dot{\lambda}_v = -\frac{\partial H}{\partial v} = \frac{\lambda_t + \lambda_v a + \lambda_a \eta + w_t + w_a a^2}{v^2}
$$

$$
\dot{\lambda}_a = -\frac{\partial H}{\partial a} = -\frac{\lambda_v + 2 w_a a}{v}
$$

### 2.3 Optimal Control: $\eta$ Enters Linearly

$$
\frac{\partial H}{\partial \eta} = \frac{\lambda_a}{v}
$$

Since $H$ is **linear** in $\eta$ (the $a^2$ penalizes the state, not the
control), the minimum principle gives:

| Condition | Control |
|---|---|
| $\lambda_a > 0$ | $\eta = \eta_{\min}$ (bang, decelerating) |
| $\lambda_a < 0$ | $\eta = \eta_{\max}$ (bang, accelerating) |
| $\lambda_a \equiv 0$ on interval | **Singular arc** ($\eta$ interior, smooth) |

**The whole difference from pure TOPPRA is the singular arc analysis.**

### 2.4 Singular Arc: Solving $\lambda_a \equiv 0$

On a singular arc, $\lambda_a(s) = 0$ for an interval, so all derivatives vanish.

**Step 1:** $\lambda_a = 0$

**Step 2:** $\dot{\lambda}_a = 0 \Rightarrow \lambda_v = -2 w_a a$

**Step 3:** $\dot{\lambda}_v = 0$. Substituting $\lambda_a = 0$, $\lambda_v = -2 w_a a$:

$$
0 = \frac{c + (-2 w_a a) \cdot a + 0 + w_t + w_a a^2}{v^2}
= \frac{c - w_a a^2 + w_t}{v^2}
$$

$$
\Rightarrow \quad a^2 = \frac{c + w_t}{w_a} =: a_*^2 \quad \text{(CONSTANT on the arc!)}
$$

**Key result:** On a singular arc, acceleration is constant:

$$
a(s) \equiv a_* = \pm\sqrt{\frac{c + w_t}{w_a}}
$$

and the singular control is simply $\eta_{\text{sing}} = 0$ (jerk is zero on
the singular arc).

### 2.5 Interpretation

The compromise solution is built from only **two** primitive arc types:

| Arc type | Control | Time-domain shape |
|---|---|---|
| **BANG** | $\eta = \eta_{\max}$ or $\eta_{\min}$ | Cubic: $a$ linear in $t$, $v$ quadratic in $t$ |
| **SINGULAR** | $\eta = 0$, $a = a_* = \text{const}$ | Constant-accel: $v$ linear in $t$ |

The weight ratio $w_a/w_t$ selects $a_*$ (through costate $c$) and the
number/duration of bang arcs:

- $w_a \to 0$: singular arcs shrink to points → pure bang-bang (TOPPRA)
- $w_a \to \infty$: bang arcs shrink, $a_* \to$ minimal feasible → smooth
  trapezoidal-ish profile with maximal constant-accel cruising

### 2.6 Transversality: Costate Value $c$

Free terminal time $\Rightarrow$ $H \equiv 0$ along the optimal trajectory.
Evaluating $H$ on a singular arc ($\lambda_a = 0$, $\lambda_v = -2 w_a a_*$):

$$
H = \frac{c + w_t - w_a a_*^2}{v} = 0 \quad \checkmark
$$

So $c = w_a a_*^2 - w_t$. Given a target $a_*$, everything is determined.
The solver finds $c$ (equivalently $a_*$) by minimizing $J(a_*)$.

---

## 3. Structure Theorem

**Theorem (informal):** For the weighted problem with $w_t > 0$, $w_a \geq 0$,
rest-to-rest boundary conditions, and regular constraint walls, the optimal
trajectory consists of a **finite** sequence of arcs of only these types:

| Arc type | Description |
|---|---|
| `BANG_PLUS` | $\eta = +\eta_{\max}$ |
| `BANG_MINUS` | $\eta = -\eta_{\min}$ |
| `SINGULAR` | $\eta = 0$, $a = a_*$ (constant) |
| `WALL` | $v = v_{\text{wall}}(u(s))$; $a$ slaved to geometry |

Between arcs, $v$ and $a$ are continuous (state); $\eta$ may jump (control).
Jerk is piecewise continuous and bounded by construction.

**Canonical rest-to-rest sequence (unconstrained):**

```
[+η][S:a=+a*][-η][S:a=0,v=v_wall][-η][S:a=-a*][+η]
```

This is the classic "double-S" / 7-segment-like profile, but where the
flat-top acceleration level $a_*$ is chosen **optimally** by $w_a/w_t$,
not by a hardware limit.

- $a_* = a_{\max}$ (hardware) → standard time-optimal S-curves
- $a_* < a_{\max}$ → buying smoothness

---

## 4. Constraint Handling

### 4.1 Chain-Rule Factorization

With $u(s)$, $du/ds = 1/g(u)$, $h(u) := 1/g(u)$:

$$
\dot{u} = v \cdot h, \quad \ddot{u} = a \cdot h + v^2 \cdot h', \quad \dddot{u} = \eta \cdot h + 3va \cdot h' + v^3 \cdot h''
$$

Per axis $i$:

$$
\dddot{q}_i = \alpha_i(u) \cdot \eta + \beta_i(u, v, a)
$$

where $\alpha_i = C_i'/g$ and $\beta_i$ = rest.

### 4.2 Velocity Walls

Axis velocity limit $|\dot{q}_i| \leq \dot{q}_i^{\max}$ gives:

$$
v \leq v_{\text{wall},i}(u) = \frac{\dot{q}_i^{\max} \cdot g(u)}{|C_i'(u)|}
$$

The effective wall is the minimum over all constraints. While riding a wall,
acceleration is slaved: $a = v_{\text{wall}}'(u) \cdot v^2 / g(u)$.

### 4.3 Acceleration Walls → Control Bounds

Axis jerk limits $|\dddot{q}_i| \leq \dddot{q}_i^{\max}$ become:

$$
\eta \in \left[ \frac{-\dddot{q}^{\max} - \beta_i}{\alpha_i}, \frac{\dddot{q}^{\max} - \beta_i}{\alpha_i} \right]
$$

(watch $\alpha_i$ sign). The aggregate:

$$
\eta_{\min}(u,v,a) = \max \text{ over constraints of lower bounds}
$$
$$
\eta_{\max}(u,v,a) = \min \text{ over constraints of upper bounds}
$$

### 4.4 The Only Difference vs. TOPPRA

Constraint **algebra** is identical. What changes is the **control law**:
instead of always banging $\eta$ to a bound, we select between bang,
singular ($\eta=0$), and wall-following, using the $a_*$ guidance.

---

## 5. Data Structures

### `CostWeights`

```cpp
struct CostWeights {
    double w_t = 1.0;  // > 0 ALWAYS
    double w_a = 0.0;  // ≥ 0; 0 recovers TOPPRA
    double a_star(double c) const;  // singular accel from costate
};
```

### `WeightedArc`

```cpp
struct WeightedArc {
    WeightedArcType type;  // BANG_PLUS, BANG_MINUS, SINGULAR, WALL
    double s0, s1;         // arc-length span
    double t0;             // absolute time at s0
    double v0, a0;         // state at s0
    double u0;             // NURBS parameter at s0
    double eta;            // BANG: constant jerk value
    double a_star;         // SINGULAR: constant acceleration level
    double duration;       // time span
};
```

### `WeightedSwitchingStructure<Dim, T>`

Implements `AnalyticalTrajectorySource<Dim, T>` for exact sampling.
Stores the arc list and provides closed-form position/velocity/acceleration/jerk
at any time $t$.

---

## 6. Solver Algorithm

### Overview

The solver is a **single-parameter shooting** method over $a_*$ (equivalently
over the costate $c$), exploiting the structure theorem. For fixed $a_*$, the
entire control law is a deterministic state machine. The cost $J(a_*)$ is then
a closed-form scalar function, minimized by golden-section search.

### Step 0: Estimate Max Reachable Acceleration

Compute $a_{\max}$ from path-level and per-axis acceleration limits at the
path midpoint.

### Step 1: Analytic Arc Propagation (Closed Forms)

**BANG arc** ($\eta = \eta_b = \text{const}$), from $(t_0, v_0, a_0)$:

$$
a(t) = a_0 + \eta_b \cdot \tau
$$
$$
v(t) = v_0 + a_0 \cdot \tau + \frac{1}{2} \eta_b \cdot \tau^2
$$
$$
s(t) = s_0 + v_0 \cdot \tau + \frac{1}{2} a_0 \cdot \tau^2 + \frac{1}{6} \eta_b \cdot \tau^3
$$

Inverse (given $\Delta s$, find $\tau$): solve cubic via Newton's method
(monotone in the region of interest since $ds/d\tau = v > 0$).

**SINGULAR arc** ($\eta = 0$, $a = a_* = \text{const}$):

$$
a(t) = a_*, \quad v(t) = v_0 + a_* \cdot \tau, \quad s(t) = s_0 + v_0 \cdot \tau + \frac{1}{2} a_* \cdot \tau^2
$$

Inverse: quadratic; $\tau = (-v_0 + \sqrt{v_0^2 + 2 a_* \Delta s}) / a_*$.
(Limit $a_* \to 0$: $\tau = \Delta s / v_0$ — constant-velocity cruise.)

**No ODE solver is needed** away from walls — everything is polynomial
root-finding.

### Step 2: Control Law as a State Machine

The practical rule (derived from §2, $H \equiv 0$ + $\lambda_a$ sign):

| Condition | Control | Arc type |
|---|---|---|
| $v \geq v_{\text{wall}}$ | Follow wall | `WALL` |
| $a < a_*$ | $\eta = +\eta_{\max}$ | `BANG_PLUS` |
| $a > a_*$ | $\eta = -\eta_{\min}$ | `BANG_MINUS` |
| $a \approx a_*$ | $\eta = 0$ | `SINGULAR` |

During braking phase: use $-a_*$ as the target.

**Braking initiation:** The transition to braking occurs when the remaining
distance equals the closed-form braking distance $S_{\text{brake}}(v)$, which
is a fixed analytic function of $v$ and $a_*$.

### Step 3: Golden-Section Search over $a_*$

With free final time, **every** $a_*$ gives a valid bang-singular-bang
trajectory satisfying boundary conditions. The cost $J$ is then a scalar
function of $a_*$:

$$
J(a_*) = w_t \cdot T(a_*) + w_a \cdot \int a^2 \, dt(a_*)
$$

Both terms are in closed form (sum of analytic arc contributions). The solver
reduces to **minimizing a scalar closed-form function** $J(a_*)$ over
$a_* \in (0, a_{\max}]$ via golden-section search.

This is dramatically simpler than generic optimal control and is the payoff
of the structure theorem.

### Step 4: Jerk-Feasibility Repair

If during simulation the clamp activates ($\eta_{\text{desired}}$ outside
$[\eta_{\min}, \eta_{\max}]$), the arc is recorded with the clamped $\eta$.
The structure theorem still holds; $J(a_*)$ remains piecewise closed-form.

---

## 7. Output Representation (WSS)

The `WeightedSwitchingStructure` stores the arc list and provides exact
sampling. Because every arc is analytic, position/velocity/acceleration/and
jerk are all sampled in closed form.

Sampling cost: $O(\log \#\text{arcs})$ for arc location + $O(1)$ polynomial
evaluation + one NURBS evaluation. Real-time capable (kHz+).

---

## 8. Sampling

### Position

$$
\mathbf{q}(t) = C(u(s(t)))
$$

where $u(s)$ is obtained from the path's arc-length-to-parameter mapping.

### Velocity

$$
\dot{\mathbf{q}} = \mathbf{T} \cdot v
$$

where $\mathbf{T}$ is the unit tangent.

### Acceleration

$$
\ddot{\mathbf{q}} = \boldsymbol{\kappa} \cdot v^2 + \mathbf{T} \cdot a
$$

### Jerk

$$
\dddot{\mathbf{q}} = \mathbf{j} \cdot v^3 + 3 \boldsymbol{\kappa} \cdot v \cdot a + \mathbf{T} \cdot \eta
$$

where $\mathbf{j}$ is the jounce vector ($d^3p/ds^3$).

---

## 9. Certification and Error Bounds

### Error Sources

1. **Root solves** (quadratic/cubic): residual controllable to $\sim 10^{-15}$
2. **Arc-length to parameter mapping**: uses the path's built-in certified
   inversion (from `PiecewiseNurbsPath::locate` + `invertLength`)

### Feasibility Certificate

Because every arc is analytic, all constraint functions can be evaluated at
the Chebyshev extrema of their per-arc closed forms, with interval-arithmetic
enclosure between extrema. This guarantees constraint satisfaction for all $t$,
not just at samples.

### Optimality Certificate (Optional)

Reconstruct costates backward from terminal transversality, integrating the
linear time-varying costate ODEs (1D per arc, cheap). Check:
- $H \equiv 0$ within tolerance
- $\text{sign}(\lambda_a)$ matches stored bang signs
- $\lambda_a \approx 0$ on singular arcs
- $\lambda$ continuous at switches

---

## 10. Tuning Guide

### Dimensional Consistency

$[w_t] = 1$, $[w_a] = \text{time}^3$. Use the scaled weight:

$$
\lambda := \frac{w_a}{w_t \cdot T_{\text{toppra}}^2 \cdot a_{\text{typ}}^2}
$$

where $T_{\text{toppra}}$ = time-optimal duration (solve once with $w_a = 0$),
$a_{\text{typ}}$ = typical acceleration scale (e.g., axis accel limit).

| $\lambda$ | Behavior |
|---|---|
| 0 | Pure time-optimal |
| 0.01–0.1 | Mild smoothing, usually <5% time penalty |
| 0.5 | Strong smoothing, ~15–40% time penalty (path dependent) |
| $\to \infty$ | Approaches constant-accel / minimum-energy motion |

### Sweep & Pareto (Recommended Workflow)

1. Solve for $\lambda \in \{0, 10^{-3}, 3 \times 10^{-3}, 10^{-2}, 3 \times 10^{-2}, 0.1, 0.3, 1\}$
2. Each solve is cheap (golden section over closed-form $J$)
3. Plot $(T, \int a^2 \, dt)$ pairs → Pareto front
4. Pick operating point
5. Store chosen $(w_t, w_a)$ with the trajectory for reproducibility

### Monotonicity

$T(\lambda)$ is non-decreasing, $\int a^2 \, dt(\lambda)$ is non-increasing.
The knob behaves predictably.

### Relation to Pure TOPPRA

Set $w_a = 0$ $\Rightarrow$ singular level $a_*$ hits $a_{\max}$ $\Rightarrow$
bang arcs vanish except at constraint events $\Rightarrow$ the WSS degenerates
to the SSR of the time-optimal manual. Same constraint algebra, same sampling
code — one framework, two regimes, continuously connected.

---

## API Reference

### `ParetoTimeEnergyOptimalVelocityPlanner<Dim, T>`

```cpp
// Constructor
explicit ParetoTimeEnergyOptimalVelocityPlanner(
    KinematicLimits<Dim, T> limits = {},
    CostWeights w = {});

// VelocityProfiler interface
VelocityProfile<T> computeProfile(
    const PathAdapter<Dim, T>& path,
    T feedRate,
    T startVelocity = T(0),
    T endVelocity = T(0),
    size_t numSamples = 100,
    T startAcceleration = T(0),
    T startJerk = T(0)) override;

KinematicLimits<Dim, T> limits() const override;
ProfilerType type() const override;  // ProfilerType::ParetoTimeEnergy
const char* name() const override;

// Extended interface
std::shared_ptr<WeightedSwitchingStructure<Dim, T>> weightedSource() const;
double costValue() const;        // achieved J
double optimalAStar() const;     // optimal singular acceleration
CostWeights weights() const;
void setWeights(CostWeights w);
```

### Usage Example

```cpp
using namespace MotionPlanner;

// Setup
KinematicLimits<3, double> limits;
limits.path.maxPathVelocity = 100.0;
limits.path.maxPathAcceleration = 500.0;
limits.path.maxPathJerk = 5000.0;
limits.path.jerkLimitEnabled = true;

CostWeights w;
w.w_t = 1.0;
w.w_a = 0.05;  // ← THE KNOB. 0 → TOPPRA. large → smooth/slow.

analytical::ParetoTimeEnergyOptimalVelocityPlanner<3> profiler(limits, w);

// Compute
auto profile = profiler.computeProfile(path, feedRate, 0, 0, 200);

// Access exact sampling
auto wss = profiler.weightedSource();
for (double t = 0; t < wss->totalTime(); t += 0.001) {
    auto q   = wss->position(t);
    auto qd  = wss->velocity(t);
    auto qdd = wss->acceleration(t);
    auto qddd = wss->pathJerk(t);
    // ...
}
```

### Integration with MotionPlanBuilder

```cpp
MotionPlanBuilder<3> builder(limits, config, ProfilerType::ParetoTimeEnergy);
auto plan = builder.build(segments, feedRate);
```

---

## Implementation Files

| File | Purpose |
|---|---|
| `include/tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp` | Main implementation |
| `include/tether/motion_planner/analytical/ConstraintEvaluator.hpp` | Constraint algebra (shared with AnalyticalTOPPRA) |
| `include/tether/motion_planner/analytical/AnalyticalTypes.hpp` | Shared types (EtaBounds, etc.) |
| `docs/motion/ParetoTimeEnergyOptimal.md` | This document |
| `tests/motion_planner/ParetoTimeEnergyTest.cpp` | Regression tests |
