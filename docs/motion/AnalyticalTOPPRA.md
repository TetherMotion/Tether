# Analytical TOPPRA-Equivalent for NURBS Chain Inputs

This document provides the complete mathematical specification, algorithmic
description, and design rationale for the analytical time-optimal
path-following controller with jerk constraints, operating on NURBS curve
chains.

The implementation lives in `include/tether/motion_planner/analytical/`
and consists of two output representations:

- **Class A — Switching Structure Representation (SSR)**: exact, procedural
- **Class B — Hybrid Monotone Representation**: practical, certifiable

Equation numbers `(A.x)` are new to this document. References to `(T.x)`,
`(G.x)`, and `(M.x)` point to `ToppraDerivation.md`, `GeometryFoundations.md`,
and `BlendingAlgorithm.md` respectively.

---

## Part I: Problem Setup

### 1.1 The Time-Optimal Path-Following Problem

We are given a path $C(u) : [u_0, u_m] \to \mathbb{R}^d$, a NURBS curve
chain (represented in Tether as `tether::motion::PiecewiseNurbsPath`). We
seek the time-optimal traversal $u(t)$ subject to:

- **Axis limits**: $|q_i^{(k)}(u(t))| \leq q_{i,k}^{\max}$ for each axis $i$
  and derivative order $k \in \{1, 2, 3\}$ (velocity, acceleration, jerk)
- **Path-level limits**: $|v| \leq v_{\text{path}}^{\max}$,
  $|a| \leq a_{\text{path}}^{\max}$, $|\eta| \leq j_{\text{path}}^{\max}$
- **Centripetal acceleration**: $v^2 \kappa(s) \leq a_{\text{cent}}^{\max}$
- **Feed rate**: $v \leq v_{\text{feed}}$
- **Boundary conditions**: $v(0) = v_{\text{start}}$, $v(L) = v_{\text{end}}$

The objective is to minimize total traversal time:

$$
T = \int_0^L \frac{ds}{v(s)}
$$

### 1.2 The TOPPRA Reduction

Since the path is fixed, all task-space quantities factor through $u(t)$.
The key insight (shared with classical TOPP-RA) is to work in **arc-length
parameterization** $s$:

$$
s(u) = \int_{u_0}^{u} g(\tau)\, d\tau, \quad g(u) := \|C'(u)\|
$$

where $g(u)$ is the **speed factor** (the arc-length derivative of the
NURBS curve). The state in arc-length space is:

$$
x(s) := \begin{bmatrix} t(s) \\ v(s) \\ a(s) \end{bmatrix}, \quad
v = \frac{ds}{dt}, \quad a = \frac{dv}{dt}
$$

The dynamics in arc-length (using $'$ for $d/ds$ and $\dot{}$ for $d/dt$):

$$
t' = \frac{1}{v}, \quad v' = \frac{a}{v}, \quad a' = \frac{\eta}{v}
\tag{A.1}
$$

where $\eta = \dot{a} = d^2s/dt^2$ is the **jerk** — our control input.

These dynamics are **singular at $v = 0$**; the optimal solution never
rests (except possibly at endpoints if boundary conditions require).

---

## Part II: Constraint Transformation

### 2.1 Arc-Length Derivatives and Task-Space Quantities

The `NurbsCurve::arcDerivatives(u, order)` method provides the arc-length
derivatives of the path:

- **Order 0**: position $p(s)$
- **Order 1**: tangent $T(s) = dp/ds$ (unit vector)
- **Order 2**: curvature vector $\vec{\kappa}(s) = d^2p/ds^2$
- **Order 3**: jounce vector $\vec{j}(s) = d^3p/ds^3$

Using the chain rule $p = p(s(t))$, the task-space quantities are:

$$
\dot{q} = T \cdot v
\tag{A.2}
$$

$$
\ddot{q} = \vec{\kappa} \cdot v^2 + T \cdot a
\tag{A.3}
$$

$$
\dddot{q} = \vec{j} \cdot v^3 + 3\vec{\kappa} \cdot v \cdot a + T \cdot \eta
\tag{A.4}
$$

This is the **key simplification** of the arc-length formulation: the
task-space jerk is **linear in the control input** $\eta$:

$$
\dddot{q}_i = \alpha_i \cdot \eta + \beta_i
\tag{A.5}
$$

where:

$$
\alpha_i = T_i, \quad \beta_i = \vec{j}_i \cdot v^3 + 3\vec{\kappa}_i \cdot v \cdot a
\tag{A.6}
$$

### 2.2 Velocity Constraints

Per-axis velocity: $|T_i \cdot v| \leq v_{\max,i}$ gives:

$$
v \leq \frac{v_{\max,i}}{|T_i|}
\tag{A.7}
$$

Centripetal: $v^2 \kappa \leq a_{\text{cent}}^{\max}$ gives:

$$
v \leq \sqrt{\frac{a_{\text{cent}}^{\max}}{\kappa}}
\tag{A.8}
$$

The overall velocity limit is:

$$
v_{\text{lim}}(s) = \min\left(v_{\text{feed}},\; v_{\text{path}}^{\max},\;
\min_i \frac{v_{\max,i}}{|T_i|},\; \sqrt{\frac{a_{\text{cent}}^{\max}}{\kappa(s)}}\right)
\tag{A.9}
$$

This is independent of $(a, \eta)$ and is precomputed at each sample point.

### 2.3 Acceleration Constraints

Per-axis: $|\vec{\kappa}_i \cdot v^2 + T_i \cdot a| \leq a_{\max,i}$

If $|T_i| > \epsilon$:

$$
a \in \left[\frac{-a_{\max,i} - \vec{\kappa}_i v^2}{T_i},\;
\frac{a_{\max,i} - \vec{\kappa}_i v^2}{T_i}\right]
\tag{A.10}
$$

If $|T_i| < \epsilon$: the constraint becomes $|\vec{\kappa}_i v^2| \leq a_{\max,i}$,
which is a constraint on $v$ only (no $a$ dependence).

Path-level: $|a| \leq a_{\text{path}}^{\max}$

The overall acceleration bounds are:

$$
a_{\min}(s, v) \leq a \leq a_{\max}(s, v)
\tag{A.11}
$$

### 2.4 Jerk Constraints

Per-axis: $|\alpha_i \eta + \beta_i| \leq j_{\max,i}$

If $\alpha_i > \epsilon$:

$$
\eta \in \left[\frac{-j_{\max,i} - \beta_i}{\alpha_i},\;
\frac{j_{\max,i} - \beta_i}{\alpha_i}\right]
\tag{A.12}
$$

If $\alpha_i < -\epsilon$: the interval is flipped.

If $|\alpha_i| < \epsilon$: the constraint becomes $|\beta_i| \leq j_{\max,i}$,
which is a constraint on $(v, a)$ only. If violated, the state is infeasible.

Path-level: $|\eta| \leq j_{\text{path}}^{\max}$

The overall eta bounds are the intersection:

$$
\eta_{\min}(s, v, a) \leq \eta \leq \eta_{\max}(s, v, a)
\tag{A.13}
$$

This is computed by the `ConstraintEvaluator::etaBounds()` method.

---

## Part III: Time-Optimal Control (Pontryagin)

### 3.1 Hamiltonian and Costates

The Hamiltonian for time minimization (cost = $\int dt$) is:

$$
H = \lambda_t \cdot \frac{1}{v} + \lambda_v \cdot \frac{a}{v} + \lambda_a \cdot \frac{\eta}{v} + 1
\tag{A.14}
$$

Costates evolve by $d\lambda/ds = -\partial H / \partial x$.

The **switching function** is:

$$
\phi(s) = \frac{\lambda_a(s)}{v(s)}
\tag{A.15}
$$

### 3.2 Bang-Bang Structure

The optimal control is determined by the sign of $\phi$:

- **$\phi > 0$**: $\eta = \eta_{\max}$ (maximal acceleration increase)
- **$\phi < 0$**: $\eta = \eta_{\min}$ (maximal deceleration)
- **$\phi = 0$ over an interval**: **singular arc** (requires further analysis)

For most practical cases with simple bounds, singular arcs do not appear;
the solution is **bang-bang** with finite switches.

### 3.3 Why No Global NURBS $u(t)$?

**Claim**: No global NURBS $u(t)$ can exactly represent time-optimal motion
for generic NURBS $C(u)$ with jerk constraints.

**Proof sketch**:

1. The time-optimal control has piecewise analytic structure with
   non-analytic switching points (finite jumps in higher derivatives).
2. A NURBS is piecewise rational, hence piecewise analytic with analytic
   joints (matching derivatives to order $p-k$ at knots with multiplicity $k$).
3. To represent a non-analytic switch, the NURBS would need a knot with
   multiplicity $p+1$ ($C^{-1}$ discontinuity), but this only allows
   position discontinuity, not the required derivative-profile shape.
4. Alternatively, approximation with smooth NURBS converges slowly
   (Gibbs phenomenon near switches, algebraic decay of coefficients).

**Consequence**: Any global NURBS $u(t)$ is necessarily approximate. The
representations here are exact (SSR) or certifiably approximate (Hybrid).

### 3.4 Why Arc-Length Parameterization?

The mapping $u \leftrightarrow s$ involves:

$$
\frac{ds}{du} = g(u) = \|C'(u)\|
\tag{A.16}
$$

For NURBS, $g(u)^2 = C'(u) \cdot C'(u)$ is **rational** (quotient of
polynomials). Thus $g(u)$ is **algebraic**, not rational in general.

The integral $s(u) = \int g(\tau)\, d\tau$ is therefore:
- Not expressible in elementary functions for generic NURBS
- An elliptic-type integral (reducible to elliptic only for special cases)

Hence $u(s)$ is **transcendental**. This is why:
- We cannot write closed-form $u(t)$
- We integrate numerically with high precision
- Local approximations (Padé, Taylor) are the best we can do

The arc-length formulation **decouples** this difficulty:
- **Geometry**: $u(s)$ handled locally (Padé per element)
- **Dynamics**: $v(s)$, $a(s)$ solved optimally (bang-bang in $\eta$)
- **Composition**: exact at evaluation time

---

## Part IV: Switching Structure Representation (SSR) — Class A

### 4.1 Design Philosophy

The SSR stores the **exact computational recipe**, not a fitted approximation.
The trajectory is reconstructed by following the stored logic: integrating
the arc-length dynamics ODEs within each switching arc.

### 4.2 Arc Structure

Each arc $k$ covers $[s_k, s_{k+1}]$ with mode $\sigma_k$:

| Mode | Control | Description |
|------|---------|-------------|
| `ACCEL_MAX` | $\eta = \eta_{\max}(s,v,a)$ | Maximal acceleration increase |
| `DECEL_MAX` | $\eta = \eta_{\min}(s,v,a)$ | Maximal deceleration |
| `ZERO_JERK` | $\eta = 0$ | Coasting, $a = \text{const}$ |
| `SINGULAR` | $\phi = 0$ condition | Singular arc (Pontryagin) |
| `CONSTRAINT_SURFACE` | Active constraint | Following constraint boundary |

Each arc stores:
- Domain: $s_{\text{begin}}$, $s_{\text{end}}$
- Initial conditions: $v_0$, $a_0$, $t_0$, $u_0$
- Mode and eta value
- Integration tolerances

### 4.3 Sampling Algorithm

To sample at time $t$:

1. **Locate arc**: Binary search on precomputed $t_k$ array — $O(\log K)$
2. **Solve for $s$**: Find $s$ such that $\int_{s_k}^{s} 1/v(\sigma)\, d\sigma = t - t_k$
   - Newton iteration with $v(s)$ from ODE integration
3. **Solve for $u$**: Integrate $du/ds = 1/g(u(s))$ from $(s_k, u_k)$ to $s$
4. **Evaluate NURBS**: $C(u)$ for position, $C'(u)$ for velocity, etc.

**Exactness**: Steps 2–3 solved to integration tolerance (machine epsilon ×
scale). Step 4 is exact NURBS mathematics (De Boor algorithm).

**Cost**: $O(\log K)$ for search + $O(M)$ for ODE integration per sample,
where $M$ depends on stiffness and tolerance.

### 4.4 Task-Space Velocity and Acceleration

Using the arc-length derivatives (A.2)–(A.4):

$$
\dot{q} = T \cdot v
\tag{A.17}
$$

$$
\ddot{q} = \vec{\kappa} \cdot v^2 + T \cdot a
\tag{A.18}
$$

where $T$ and $\vec{\kappa}$ are obtained from `NurbsCurve::arcDerivatives()`.

---

## Part V: Hybrid Monotone Representation — Class B

### 5.1 Design Philosophy

The Hybrid representation pre-computes and stores a **spectral/polynomial
approximation** that is:
- **Certifiable** (error bound known per element)
- **Efficient** ($O(\log M)$ search + $O(N)$ per evaluation, $N \sim 8$–$16$)
- **Structure-preserving** (monotonicity of $t(s)$, boundedness of $v$, $a$)

### 5.2 Key Insights

1. **$t(s)$ is strictly increasing** ($dt/ds = 1/v > 0$ for $v > 0$)
   → $t(s)$ is invertible; we precompute $t(s)$ and invert by Newton.

2. **$v(s)$, $a(s)$ are smooth** on each switching arc, with discontinuities
   only at switch points in higher derivatives.
   → hp-adaptive scheme: high degree where smooth, small elements at switches.

3. **$u(s)$ is not polynomial** (due to $1/g(u)$ integration) but smooth
   → Local Padé approximation with error control.

### 5.3 Representation Choice

| Quantity | Representation | Method |
|----------|---------------|--------|
| $t(s)$, $v(s)$, $a(s)$ | hp-LGL spectral elements | Legendre-Gauss-Lobatto collocation |
| $u(s)$ | Local Padé approximant | Rational approximation from Taylor series |
| $s(t)$ | Inverse of $t(s)$ | Newton iteration safeguarded by bisection |

### 5.4 HP-Adaptive LGL Element

On element $[s_k, s_{k+1}]$ with length $h_k = s_{k+1} - s_k$:

**Reference mapping**:
$$
\xi = \frac{2(s - s_k)}{h_k} - 1, \quad \xi \in [-1, 1]
\tag{A.19}
$$

**LGL nodes**: $\xi_j = \cos(j\pi/N)$, $j = 0, \ldots, N$ (the endpoints
and the roots of $L_N'(\xi)$, where $L_N$ is the Legendre polynomial of
degree $N$).

**Derivative matrix**: $D[i][j] = L_j'(\xi_i)$, computed via the barycentric
formula:

$$
D[i][j] = \frac{\lambda_j}{\lambda_i(\xi_i - \xi_j)}, \quad i \neq j
\tag{A.20}
$$

$$
D[i][i] = -\sum_{j \neq i} D[i][j]
\tag{A.21}
$$

where $\lambda_j$ are the barycentric weights:

$$
\lambda_j = \frac{1}{N(N+1) L_N(\xi_j)^2}
\tag{A.22}
$$

**Physical derivative**: $df/ds = (2/h_k) \cdot df/d\xi$

**Spectral accuracy**: Error decays as $O(N^{-m})$ for all $m$ if the
function is smooth, or exponentially if analytic in a complex neighborhood.

### 5.5 Padé Approximant for $u(s)$

On element $[s_k, s_{k+1}]$, with $\delta = s - s_k$:

$$
u(s) \approx R_{[m/n]}(\delta) = \frac{a_0 + a_1 \delta + \cdots + a_m \delta^m}
{1 + b_1 \delta + \cdots + b_n \delta^n}
\tag{A.23}
$$

**Taylor coefficients** of $u(s)$ at $s_k$:

$u(s)$ satisfies $du/ds = h(u) = 1/g(u)$. The Taylor expansion
$u(s) = \sum c_k \delta^k$ has:

$$
c_0 = u_0, \quad c_1 = h(u_0), \quad c_2 = h'(u_0) \cdot h(u_0)
\tag{A.24}
$$

$$
c_3 = h''(u_0) \cdot h(u_0)^2 + h'(u_0)^2 \cdot h(u_0)
\tag{A.25}
$$

and so on, where $h'(u) = -g'(u)/g(u)^2$, etc.

**Padé system**: The denominator coefficients $b_1, \ldots, b_n$ satisfy:

$$
\sum_{j=0}^{n} b_j \cdot c_{i-j} = 0, \quad i = m+1, \ldots, m+n
\tag{A.26}
$$

(with $b_0 = 1$, $c_k = 0$ for $k < 0$). This is an $n \times n$ linear
system solved by Gaussian elimination with partial pivoting.

The numerator coefficients are then:

$$
a_k = \sum_{j=0}^{\min(k,n)} b_j \cdot c_{k-j}, \quad k = 0, \ldots, m
\tag{A.27}
$$

**Stability**: Use $[m/n]$ with $m \geq n$, typically $[3/2]$ or $[4/3]$.

### 5.6 Certification

For each element, we compute certified error bounds:

**$u(s)$ Padé error**: For $[m/n]$ Padé with exact match of first $m+n$
derivatives:

$$
|u - R| = O(\delta^{m+n+1}) \quad \text{as } \delta \to 0
\tag{A.28}
$$

**Certificate**: Evaluate at element midpoint, compare to exact ODE
integration. Bound sup error by:

$$
|u - R| \leq \frac{1}{(m+n+1)!} \sup|u^{(m+n+1)}| \cdot h^{m+n+1}
\tag{A.29}
$$

**Task-space propagated errors**:

$$
|C(u_{\text{exact}}) - C(u_{\text{approx}})| \leq \sup\|C'\| \cdot |\delta u|
\tag{A.30}
$$

$$
|\dot{q}_{\text{exact}} - \dot{q}_{\text{approx}}| \leq
\|C''\| \cdot |\dot{u}| \cdot |\delta u| + \|C'\| \cdot |\delta \dot{u}|
\tag{A.31}
$$

$$
|\ddot{q}_{\text{exact}} - \ddot{q}_{\text{approx}}| \leq
\text{(similar expansion, with } |\delta \ddot{u}| \text{ dominating)}
\tag{A.32}
$$

### 5.7 Adaptive Refinement Strategy

- **h-refinement**: Split element where error > tolerance
- **p-refinement**: Increase $N$ where function is smooth
- **Choice guided by smoothness indicator**: decay of LGL coefficient magnitudes

The current implementation uses h-refinement with a fixed initial degree
($N = 6$), splitting elements until the certified $u$ error meets the
target tolerance.

---

## Part VI: Time-Optimal Solver

### 6.1 Forward-Backward Integration

The solver uses a forward-backward integration approach in arc-length space:

**Phase 1 — Forward pass** (maximal acceleration):
- Start from $(s=0, v=v_{\text{start}}, a=a_{\text{start}})$
- Apply $\eta = \eta_{\max}(s, v, a)$ at each step
- Integrate dynamics (A.1) using RK4
- Record maximum velocity profile $v_{\text{fwd}}(s)$

**Phase 2 — Backward pass** (maximal deceleration):
- Start from $(s=L, v=v_{\text{end}}, a=0)$
- Apply $\eta = \eta_{\min}(s, v, a)$ at each step
- Integrate backward
- Record $v_{\text{bwd}}(s)$

**Phase 3 — Merge**:
$$
v(s) = \min(v_{\text{fwd}}(s),\; v_{\text{bwd}}(s),\; v_{\text{lim}}(s))
\tag{A.33}
$$

**Phase 4 — Switching structure**: Detect where the active constraint
changes and build switching arcs.

### 6.2 Integration Details

The dynamics (A.1) are integrated in $s$-space using classical RK4:

$$
\frac{dv}{ds} = \frac{a}{v}, \quad \frac{da}{ds} = \frac{\eta}{v}
\tag{A.34}
$$

At each step, $\eta$ is recomputed from the constraint evaluator:

- Forward pass: $\eta = \eta_{\max}(s, v, a)$
- Backward pass: $\eta = \eta_{\min}(s, v, a)$

The step size is adaptively chosen to ensure numerical stability (typically
$\Delta s \sim 10^{-4}$ units).

### 6.3 Switching Point Detection

Switching points are where the active constraint changes:
- Forward → backward (acceleration → deceleration)
- Forward → velocity limit (acceleration → cruise)
- Backward → velocity limit (deceleration → cruise)

These are detected by comparing which of $\{v_{\text{fwd}}, v_{\text{bwd}},
v_{\text{lim}}\}$ is the minimum at each sample.

---

## Part VII: Interface Compatibility

### 7.1 VelocityProfiler Interface

The `AnalyticalTOPPRA` class implements the standard
`VelocityProfiler<Dim, T>` interface. Its `computeProfile()` method:

1. Solves the time-optimal problem → produces SSR (switching arcs)
2. Samples the SSR at uniform arc-length intervals → produces a
   `VelocityProfile<T>` (tabulated, with linear interpolation)
3. Stores the SSR and optionally builds the Hybrid representation

This means **all existing downstream consumers** that expect a sampled
`VelocityProfile` (like `MotionPlan`) work **unchanged**.

### 7.2 MotionPlan Compatibility

`MotionPlan` has been extended to optionally hold an
`AnalyticalTrajectorySource`:

- When present, `evaluateAt(t)` uses the analytical source for exact/
  certified sampling of position, velocity, and acceleration.
- When absent, it falls back to the tabulated profile (original behavior).

The `MotionPlanBuilder` automatically extracts the analytical source from
an `AnalyticalTOPPRA` profiler and passes it to the `MotionPlan`.

### 7.3 Sampling Old vs New Data Structures

Consumers that use sampled data (the tabulated `VelocityProfile`) work
unchanged — the analytical profiler produces a compatible tabulated profile
by sampling the SSR.

Consumers that want exact sampling can access the `TrajectorySampler` via
`AnalyticalTOPPRA::analyticalSource()`, which wraps either the SSR or
Hybrid representation and provides `position(t)`, `velocity(t)`,
`acceleration(t)`.

### 7.4 ProfilerType Enum

The `ProfilerType` enum has been extended with `AnalyticalTOPPRA`. The
`MotionPlanBuilder` can be configured to use the analytical profiler:

```cpp
MotionPlanBuilder3D builder(limits, config, ProfilerType::AnalyticalTOPPRA);
auto plan = builder.build(segments, feedRate);
```

Or with a custom-configured instance:

```cpp
auto profiler = std::make_unique<analytical::AnalyticalTOPPRA<3, double>>(
    limits, /*buildHybrid=*/true, /*hybridTolerance=*/1e-10);
MotionPlanBuilder3D builder(std::move(profiler), limits);
auto plan = builder.build(segments, feedRate);
```

---

## Part VIII: File Structure

```
include/tether/motion_planner/analytical/
├── NumericalUtils.hpp                    # LGL, Padé, barycentric, RK4, Newton
├── AnalyticalTypes.hpp                   # ControlMode, SwitchingArc, EtaBounds,
│                                         # KinematicCoefficients, ErrorCertificate,
│                                         # AnalyticalTrajectorySource interface
├── ConstraintEvaluator.hpp               # eta bounds from NURBS arc-derivatives
├── SwitchingStructureRepresentation.hpp  # Class A (SSR) — exact, procedural
├── HybridMonotoneRepresentation.hpp      # Class B (Hybrid) — LGL + Padé + cert
├── TrajectorySampler.hpp                 # Unified interface (wraps SSR or Hybrid)
└── AnalyticalTOPPRA.hpp                  # Profiler + solver (implements VelocityProfiler)
```

### Class Relationships

```
VelocityProfiler<Dim,T>  (abstract interface)
       ▲
       │
AnalyticalTOPPRA<Dim,T>  (solver + sampler)
       │
       ├──► SwitchingStructureRepresentation<Dim,T>  (Class A, SSR)
       │         │
       │         └──► AnalyticalTrajectorySource<Dim,T>  (interface)
       │
       ├──► HybridMonotoneRepresentation<Dim,T>  (Class B, Hybrid)
       │         │
       │         └──► AnalyticalTrajectorySource<Dim,T>  (interface)
       │
       └──► TrajectorySampler<Dim,T>  (unified wrapper)
                 │
                 └──► AnalyticalTrajectorySource<Dim,T>  (interface)
                          │
                          ▼
                    MotionPlan<Dim,T>  (consumer, backward compatible)
```

---

## Part IX: Usage Example

```cpp
#include <tether/motion_planner/MotionPlanner.hpp>

using namespace MotionPlanner;

// 1. Setup limits with jerk constraints
KinematicLimits3D limits;
limits.path.maxPathVelocity = 100.0;
limits.path.maxPathAcceleration = 500.0;
limits.path.maxPathJerk = 5000.0;
limits.path.jerkLimitEnabled = true;
limits.path.maxCentripetalAcceleration = 500.0;
for (int i = 0; i < 3; ++i) {
    limits.axis.maxVelocity[i] = 100.0;
    limits.axis.maxAcceleration[i] = 500.0;
    limits.axis.maxJerk[i] = 5000.0;
}
limits.axis.jerkLimitEnabled = true;

// 2. Build motion plan with analytical profiler
MotionPlanBuilder3D builder(limits, {}, ProfilerType::AnalyticalTOPPRA);
auto plan = builder.build(segments, 50.0);

// 3. Evaluate at any time (uses analytical source automatically)
for (double t = 0; t < plan.totalDuration(); t += 0.001) {
    auto state = plan.evaluateAt(t);
    // state.position, state.velocity, state.acceleration are exact
}

// 4. Access analytical source directly for certification
auto source = plan.analyticalSource();
if (source) {
    auto cert = source->certify(1.5);
    // cert.pos_error, cert.vel_error, cert.acc_error are guaranteed bounds
}
```

---

## Part X: Comparison with Existing Profilers

| Feature | BasicTOPPRA | JerkConstrainedTOPPRA | AnalyticalTOPPRA |
|---------|------------|----------------------|-----------------|
| Order | 2nd (bang-bang) | 3rd (jerk-limited) | 3rd (jerk-optimal) |
| Time-optimal | Yes | Yes (subject to jerk) | Yes (subject to jerk) |
| Jerk bounded | No | Yes | Yes |
| Accel continuous | No | Yes | Yes |
| Representation | Tabulated | Tabulated | SSR (exact) + Hybrid (certified) |
| Sampling | Linear interp | Linear interp | ODE integration / spectral |
| Error bounds | No | No | Yes (Hybrid) |
| Arc-length dynamics | Simplified | Simplified | Full ODE integration |
| Constraint eval | Per-sample | Per-sample | Per-step (recomputed) |
| NURBS derivatives | Order 2 | Order 2 | Order 3 (jounce) |

The AnalyticalTOPPRA profiler is the most accurate and complete
implementation, at the cost of higher computational complexity. It is
recommended for applications requiring:
- Certified error bounds
- Exact trajectory reconstruction
- Full jerk-constrained optimality
- High-precision sampling at arbitrary time points

---

## References

- Piegl, L. & Tiller, W. *The NURBS Book*, 2nd ed. Springer, 1997.
- Canuto, C. et al. *Spectral Methods*. Springer, 2006.
- Pham, H. & Pham, Q.-C. "A new approach to time-optimal path parameterization
  based on reachability analysis." *IEEE T-RO*, 2018.
- Verscheure, D. et al. "Time-optimal path tracking for robots: a convex
  optimization approach." *IEEE T-AC*, 2009.
- Bobrow, J.E. et al. "Time-optimal control of robotic manipulators along
  specified paths." *IJRR*, 1985.
