# SnapSpace Velocity Profiler — Design Document

## 1. Overview

The SnapSpace Velocity Profiler computes time-energy-optimal velocity profiles
for NURBS toolpaths in a **4th-order dynamics** framework (snapspace). It
replaces the previous jerkspace (3rd-order) profiler with one that limits
**snap** σ = d³s/dt³ = dj/dt as the control input, producing trajectories that
are C³-continuous (jerk is continuous, snap is piecewise-constant).

### 1.1 Problem Statement

Given:
- A path P(s) parameterized by arc length s ∈ [0, L]
- Kinematic limits: v_max, a_max, j_max, σ_max
- A velocity-limit function v_lim(s) from curvature, feed rate, axis limits
- Boundary conditions: v(0)=v₀, a(0)=0, j(0)=0, v(L)=v_f, a(L)=0, j(L)=0
- Cost weights: w_t (time), w_j (jerk energy), w_a (acceleration energy)

Find the trajectory that minimizes:

```
J = ∫₀ᵀ [ w_t + w_j · j(t)² + w_a · a(t)² ] dt
```

subject to:
- |v(t)| ≤ v_lim(s(t))
- |a(t)| ≤ a_max(s, v)
- |j(t)| ≤ j_max
- |σ(t)| ≤ σ_max
- v(t) ≥ 0

### 1.2 State Vector

The state is 4-dimensional:

```
x = (s, v, a, j)
```

with dynamics:

```
ṡ = v
v̇ = a
ȧ = j
j̇ = σ  (control: snap)
```

### 1.3 Arc Types (Pontryagin's Maximum Principle)

The optimal control structure has three arc types:

| Arc Type | Control | Duration Formula | Use |
|----------|---------|-----------------|-----|
| SNAP_PLUS | σ = +σ_max | Quartic in τ | Ramp jerk up toward j* |
| SNAP_MINUS | σ = -σ_max | Quartic in τ | Ramp jerk down |
| SINGULAR | σ = 0, j = j* | Cubic in τ | Constant-jerk cruising |
| WALL | σ = 0, j = 0, a = 0 | v = v_lim(s) | Cruise at velocity limit |
| DWELL | (no motion) | Fixed duration | G4 pause at v=0 |

**SNAP arcs** propagate the state as:
```
j(τ) = j₀ + σ·τ
a(τ) = a₀ + j₀·τ + ½·σ·τ²
v(τ) = v₀ + a₀·τ + ½·j₀·τ² + (1/6)·σ·τ³
Δs(τ) = v₀·τ + ½·a₀·τ² + (1/6)·j₀·τ³ + (1/24)·σ·τ⁴
```

**SINGULAR arcs** (σ=0, j=j*):
```
j(τ) = j*
a(τ) = a₀ + j*·τ
v(τ) = v₀ + a₀·τ + ½·j*·τ²
Δs(τ) = v₀·τ + ½·a₀·τ² + (1/6)·j*·τ³
```

The inverse (given Δs, find τ) requires solving a quartic (SNAP) or cubic
(SINGULAR), done via safeguarded Newton-bisection.

### 1.4 Optimization Parameter: j*

The **singular jerk level** j* is the single scalar optimization parameter.
By Pontryagin analysis:

```
j* = √((c + w_t) / w_j)
```

where c is the costate (energy constant). The solver searches over j* ∈
(0, j_max] to minimize J(j*).

Weight extremes:
- w_j = 0, w_a = 0 → pure time-optimal (j* → j_max)
- w_t = 0 → ill-posed (infinite time); always keep w_t > 0
- Both > 0 → configurable compromise (smooth, energy-optimal)


## 2. Algorithm Architecture

### 2.1 High-Level Flow

```
computeProfile(path, feedRate, v0, vf)
  ├── Split path at dwell points (v=0 breaking points)
  ├── For each sub-path:
  │     ├── buildConstraintCache()     — geometry, v_lim grid
  │     ├── buildFineVelocityGrid()    — fine v_lim with per-segment limits
  │     ├── Backward pass (analytical) — v_lim(s) from corners
  │     ├── solve()                    — optimize over j*
  │     │     ├── Coarse grid scan over j*
  │     │     ├── Golden-section refinement
  │     │     └── simulateAndCost(j*)  — forward pass, produce arcs
  │     └── Coalesce arcs
  └── Build WSS + VelocityProfile
```

### 2.2 Breaking Points (Zero-Velocity Conditions)

The path is split at **breaking points** — positions where v must be exactly
zero. Each sub-path is solved independently with v=0 boundary conditions.

Breaking points include:
- **Dwell commands** (G4): tool must stop for a specified duration
- **Rest-to-rest boundaries**: v₀=0 or v_f=0
- **Explicit exact-stop commands**

At each breaking point, the state is fully reset: (v, a, j) = (0, 0, 0).

### 2.3 Constraint Cache

A coarse grid (default 200 points) caches:
- Geometric coefficients (tangent, curvature) from NURBS evaluation
- Velocity limit from curvature, feed rate, per-axis limits

A fine grid (10× coarse resolution) caches:
- Per-segment feed rates
- Corner velocities (from path geometry)
- Backward/forward propagated velocity limits

### 2.4 Backward Pass (Analytical TOPP-RA)

The backward pass computes the maximum velocity v_lim(s) such that the tool
can decelerate to every upcoming corner velocity.

**Key insight**: The braking distance from state (v, a, j) is computed
analytically using the exact snapspace stopping formula `sStopFromState(v, a, j)`.

#### 2.4.1 sStopFromState(v, a, j)

The optimal braking sequence from state (v, a, j) to v=0:

```
Phase 1: σ = -σ_max, from (v, a, j) until j = -j_max
  τ₁ = (j + j_max) / σ_max
  v₁ = v + a·τ₁ + ½·j·τ₁² - (1/6)·σ_max·τ₁³
  s₁ = v·τ₁ + ½·a·τ₁² + (1/6)·j·τ₁³ - (1/24)·σ_max·τ₁⁴

Phase 2: j = -j_max, from (v₁, a₁, -j_max) until a = -a_max
  τ₂ = (a₁ + a_max) / j_max
  v₂ = v₁ + a₁·τ₂ - ½·j_max·τ₂²
  s₂ = v₁·τ₂ + ½·a₁·τ₂² - (1/6)·j_max·τ₂³

Phase 3: a = -a_max, from (v₂, -a_max, 0) until v = 0
  s₃ = v₂² / (2·a_max)

sStop = s₁ + s₂ + s₃
```

If v reaches 0 during any phase, the formula is truncated at that point.

#### 2.4.2 Backward Pass Algorithm

For each corner (s_c, v_c) ahead of position s:

```
d = s_c - s
sStopCorner = sStopFromRest(v_c)
decelDist = sStopFromState(v_eff, a_worst, j_worst) - sStopCorner

if decelDist > d:
    # v_eff is too high — reduce it
    # Bisection: find max v such that sStopFromState(v, a_worst, j_worst) - sStopCorner ≤ d
    v_eff = bisection_result
```

where (a_worst, j_worst) is the worst-case state at the switching point.

**The worst-case state problem**: The backward pass doesn't know the actual
(a, j) at each point because that depends on the forward pass (which hasn't
run yet). See §3 for the iterative solution.

#### 2.4.3 Fine Grid Backward/Forward Propagation

On the fine grid, a bang-bang formula is used for geometric vLim propagation:

```
Backward: vLim[i] = min(vLim[i], √(vLim[i+1]² + 2·a_max·ds))
Forward:  vLim[i] = min(vLim[i], √(vLim[i-1]² + 2·a_max·ds))
```

This handles geometric vLim dips (curvature, feed rate changes) at grid
resolution. The analytical backward pass from corners (§2.4.2) handles the
snapspace braking constraint at exact corner positions.

### 2.5 Forward Pass (simulateAndCost)

The forward pass walks the path from s=0 to s=L, producing arcs.

At each step:
1. Query v_lim(s) from the backward pass
2. Compute acceleration bounds [a_min, a_max] at (s, v)
3. Choose desired acceleration a_des:
   - v > v_lim → a_des = a_min (decelerate)
   - v ≈ v_lim → a_des = track v_lim slope
   - v < v_lim → a_des = ramp using j* as target jerk
4. Choose desired jerk j_des:
   - Accelerating: j_des = j*
   - Otherwise: j_des = (a_des - a) / dt
5. Choose snap σ:
   - |j_des - j| > threshold → σ = ±σ_max (bang-bang in snap)
   - |j_des - j| ≈ 0 → σ = 0 (SINGULAR)
6. Compute arc length (limited by switching points, corners, bounds)
7. Propagate state (s, v, a, j) by the arc
8. Accumulate cost J

The cost is computed in closed form for each arc type:
- SNAP: ∫(w_t + w_j·j² + w_a·a²)dt — polynomial integration
- SINGULAR: ∫(w_t + w_j·j*² + w_a·a²)dt — polynomial integration
- WALL: ∫ w_t dt = w_t · Δs / v

### 2.6 Arc Limiting

The arc length dsArc is limited by:
1. **Jerk switching**: when jerk reaches j_des (SNAP → SINGULAR transition)
2. **Acceleration bounds**: when a reaches a_max or a_min
3. **Corner velocities**: when v would exceed v_corner at an upcoming corner
4. **Grid step**: dsArc ≤ dsStep (safety)

This ensures each arc ends at a switching point, producing a minimal arc
sequence.

### 2.7 Arc Coalescing

Consecutive arcs with identical type, sigma, and j_star are merged into a
single arc. For SNAP arcs, jerk continuity is verified (j₁ of previous = j₀
of current).

### 2.8 Dwell Handling

Dwell points (G4 commands) split the path into sub-paths. Each sub-path is
solved independently with v=0 boundary conditions. DWELL arcs are inserted
between sub-path solutions.

This avoids the 1/v integral divergence that would occur if dwells were
handled within a single solve.


## 3. The Core Issue: Backward Pass State Dependency

### 3.1 Problem Statement

The backward pass needs `sStopFromState(v, a, j)` to compute the exact braking
distance. But the actual (a, j) at each point depends on the forward pass,
which depends on the backward pass. This is a **circular dependency**.

### 3.2 Previous Approaches (and why they failed)

#### 3.2.1 Assume rest state (a=0, j=0)

The simplest approach: use `sStopFromRest(v)` in the backward pass.

**Problem**: When the solver is accelerating at (a=a_max, j=j_max) and needs
to brake, it must first reverse a and j. During this reversal, velocity is
still increasing (because a > 0). The actual braking distance is much larger
than `sStopFromRest(v)`, so the solver overshoots and cannot stop in time.

#### 3.2.2 Worst-case state (a=a_max, j=j_max)

Use `sStopFromState(v, a_max, j_max)` for all points.

**Problem**: This is overly conservative. After braking begins, a and j are
negative, so the worst-case assumption is wrong. The v_lim profile is too
low, producing unnecessarily slow trajectories.

#### 3.2.3 Safety factor

Apply a factor (e.g., 0.5) to the distance: `decelDist ≤ 0.5 · d`.

**Problem**: This is a hack with no theoretical basis. The factor must be
tuned per problem, and it either overshoots (too aggressive) or is too slow
(too conservative).

### 3.3 Solution: Iterative Forward-Backward Pass (Future Work)

The circular dependency is resolved by **iteration**:

```
Iteration 0:
  1. Backward pass: assume (a=0, j=0) everywhere → v_lim⁰(s)
  2. Forward pass: simulate using v_lim⁰(s) → record (a(s), j(s)) profile

Iteration k (k = 1, 2, ...):
  1. Backward pass: use (a(s), j(s)) from iteration k-1 → v_lim^k(s)
  2. Forward pass: simulate using v_lim^k(s) → update (a(s), j(s))
  3. Check convergence: |v_lim^k - v_lim^(k-1)| < ε
```

**This is future work.** Before implementing iteration, the single forward
pass must be perfected. See §3.4 below.

### 3.4 Current Strategy: Perfect the Single Forward Pass First

**Before implementing any iteration, the single forward pass must work 100%
correctly under all conditions.** This is the foundation — if one forward
pass is unreliable, iterating it will not fix the problem.

#### 3.4.1 The Forward Pass as a Reusable Method

The forward pass is codified as a standalone, reusable method:

```cpp
struct ForwardPassResult {
    std::vector<Arc> arcs;        // produced arc sequence
    double cost;                  // total cost J
    double finalS;                // final arc length reached
    double finalV;                // final velocity
    double finalA;                // final acceleration
    double finalJ;                // final jerk
    bool feasible;                // reached sEnd with v ≈ vf
    std::string failureReason;    // if not feasible, why
};

ForwardPassResult forwardPass(
    double jStar,                 // singular jerk level
    const std::vector<double>& vLimProfile,  // velocity limit profile
    double v0, double vf,         // boundary velocities
    double sTotal                 // path length
);
```

This method:
- Takes a **pre-computed v_lim profile** (from any backward pass)
- Produces a deterministic arc sequence and final state
- Reports feasibility and failure reason
- Does NOT depend on any backward-pass internals
- Is independently testable

#### 3.4.2 Testing Strategy: Extreme Conditions

The forward pass is tested under a comprehensive set of adverse conditions:

**Category A: Boundary Conditions**
- A1: Rest-to-rest (v0=0, vf=0) on a long path
- A2: Rest-to-rest on a very short path (barely enough room to stop)
- A3: Flying start (v0 > 0) with rest end (vf=0)
- A4: Rest start (v0=0) with flying end (vf > 0)
- A5: Flying start and flying end (v0 > 0, vf > 0)
- A6: v0 = v_max (start at full speed)

**Category B: Path Geometry**
- B1: Straight line (no curvature)
- B2: L-shaped path (one 90° corner)
- B3: Path with multiple corners
- B4: Very long path (v_lim plateau in the middle)
- B5: Very short path (accel + brake only, no cruise)

**Category C: Kinematic Limits**
- C1: Very high snap (σ_max → ∞, approaches jerkspace)
- C2: Very low snap (σ_max → 0, very smooth)
- C3: Very high jerk (j_max → ∞, approaches bang-bang in accel)
- C4: Very low jerk (j_max → 0, very smooth)
- C5: Very high accel (a_max → ∞)
- C6: Very low accel (a_max → 0, very slow)

**Category D: v_lim Profiles**
- D1: Flat v_lim (constant, no corners)
- D2: v_lim with a sharp dip (narrow corner)
- D3: v_lim with a wide dip (long corner zone)
- D4: v_lim = 0 at the end (terminal stop)
- D5: v_lim ramping down linearly
- D6: v_lim with multiple dips

**Category E: j* Values**
- E1: j* = 0 (degenerate, no jerk)
- E2: j* very small (very smooth, slow)
- E3: j* = j_max (time-optimal)
- E4: j* > j_max (should be clamped)

**Category F: Stress / Edge Cases**
- F1: Zero-length path (sTotal = 0)
- F2: v0 > v_lim(0) (start above limit)
- F3: vf > v_lim(L) (end above limit)
- F4: v_lim drops faster than possible deceleration
- F5: Extremely fine grid (dsStep → 0)
- F6: Extremely coarse grid (dsStep → large)

**For each test, we verify:**
1. **No crash**: the forward pass completes without exception
2. **State continuity**: (s, v, a, j) are continuous across arc boundaries
3. **Constraint satisfaction**: v ≤ v_lim, |a| ≤ a_max, |j| ≤ j_max, |σ| ≤ σ_max
4. **Monotonic s**: s strictly increases (no backward motion)
5. **Non-negative v**: v ≥ 0 everywhere
6. **Final state**: s_final ≈ sTotal, v_final ≈ vf (within tolerance)
7. **Arc count reasonable**: < 10000 arcs (no explosion)
8. **Cost finite and non-negative**: J is a valid number ≥ 0

#### 3.4.3 Implementation Order

1. **Extract** the forward pass from `simulateAndCost` into a clean, reusable
   method with a well-defined interface
2. **Write** the extreme test suite (§3.4.2) as a new test file
3. **Fix** all failures, one by one, until every test passes
4. **Only then** implement the iterative approach (§3.3)


## 4. Control Law Details

### 4.1 Acceleration Selection

```
if v > v_lim + ε:
    a_des = a_min              # overshooting — brake hard
elif v ≥ v_lim - margin:
    a_des = clamp(a_track)     # tracking v_lim
else:
    a_des = min(a + j*·dt, a_max)  # accelerate using j*
```

where:
- `a_track = (v_lim_next² - v²) / (2·ds)` — acceleration to reach v_lim_next
- `margin = a_max · dt + ε` — tolerance for v_lim tracking

### 4.2 Jerk Selection

```
if accelerating (v < v_lim, a < a_max, a_des > 0):
    j_des = j*                 # target jerk is the singular level
else:
    j_des = (a_des - a) / dt   # compute from acceleration target
```

The dtEst is clamped to 1.0s to avoid the v→0 singularity where dtEst→∞.

### 4.3 Snap Selection (Bang-Bang in Snap)

```
jErr = j_des - j
if |jErr| < threshold:
    σ = 0       # SINGULAR — jerk is at target
else:
    σ = ±σ_max  # SNAP — ramp jerk at max rate
```

This is the snapspace analog of bang-bang in acceleration. The arc limiting
logic ensures each SNAP arc ends exactly when jerk reaches j_des, so the
transition to SINGULAR is precise.


## 5. Data Structures

### 5.1 WeightedArc

```cpp
struct WeightedArc {
    WeightedArcType type;   // SNAP_PLUS, SNAP_MINUS, SINGULAR, WALL, DWELL
    double s0, s1;          // arc-length span
    double t0;              // absolute time at s0
    double v0, a0, j0;      // state at s0 (4D)
    double u0;              // NURBS parameter at s0
    double sigma;           // SNAP: constant snap value
    double j_star;          // SINGULAR: constant jerk level
    double duration;        // time span
};
```

### 5.2 WeightedSwitchingStructure (WSS)

The WSS wraps the arc list and provides:
- `locateAndState(t)` → (arcIdx, τ, s, v, a, j) — exact state at time t
- `computeArcDuration(arc)` — duration from arc-length
- `timeAtArcLength(s)` — time at arc-length s

### 5.3 SolverConfig (new)

```cpp
struct SolverConfig {
    int maxIterations = 3;
    double convergenceThreshold = 0.01;
    bool conservativeFallback = true;
};
```


## 6. Known Issues and Solutions

### 6.1 v→0 Singularity in Control Law

**Issue**: When v≈0, `dtEst = ds/v → ∞`, making `j_des = (a_des - a)/dt → 0`
and `σ_des = (j_des - j)/dt → 0`. The solver gets stuck at v=0.

**Solution**: Clamp `dtEst` to 1.0s. Use j* directly as the target jerk
during acceleration (not derived from a_des/dt). Use bang-bang snap (±σ_max)
instead of computing σ from dt.

### 6.2 Terminal Velocity Residual

**Issue**: Snapspace discretization makes exact v=0 at the terminal point
difficult. The solver may end with a small residual velocity.

**Solution**: Allow a tolerance (0.5 m/s) on terminal velocity. Add a
penalty (1e6 × |v - v_f|) to the cost for large residuals, and a smaller
penalty (1e3 × |v - v_f|) for small residuals. The penalty is consistent
between record and non-record runs.

### 6.3 Backward Pass State Dependency

**Issue**: The backward pass needs (a, j) from the forward pass, which needs
the backward pass. See §3.

**Solution**: Iterative forward-backward pass with conservative fallback.

### 6.4 Arc Explosion

**Issue**: When the solver gets stuck (e.g., v≈0, tiny arcs), the number of
arcs can explode to tens of thousands, causing slow performance.

**Solution**: Limit maxIter to `constraintCacheSize × 200`. If v drops to ~0
after starting, mark as infeasible. The arc coalescing step merges consecutive
arcs with identical parameters.

### 6.5 Corner Detection

**Issue**: Corners (velocity dips from path geometry) may not be explicitly
provided. The solver must detect them from the v_lim grid.

**Solution**: Scan the fine grid for local minima and significant drops in
v_lim. Add these as corners for the backward pass. Always add the terminal
point (sEnd, v_f) as a corner.


## 7. Implementation Plan

### Phase 1: Extract and Perfect the Forward Pass (CURRENT)

1. Extract the forward pass from `simulateAndCost` into a clean, reusable
   method `forwardPass()` with a well-defined `ForwardPassResult` interface
2. Write the extreme test suite (§3.4.2) as `ForwardPassTest.cpp`
3. Fix all failures, one by one, until every test passes
4. The forward pass must handle all adverse conditions without crashing,
   exploding, or producing invalid states

### Phase 2: Breaking Points (NEXT)

1. Identify breaking points: dwell positions, v=0 boundaries
2. Split path at breaking points
3. Solve each sub-path independently
4. Insert DWELL arcs between sub-paths

### Phase 3: Iterative Forward-Backward Pass (FUTURE)

1. Add `SolverConfig` struct with `maxIterations`, `convergenceThreshold`
2. Store (a(s), j(s)) profile from the forward pass (sampled on the grid)
3. Modify `lookAheadVLimit` to accept (a, j) arrays instead of worst-case
4. Implement the iteration loop in `solve()`:
   - Iteration 0: backward pass with (a=0, j=0)
   - Iteration k: backward pass with (a, j) from iteration k-1
   - Check convergence
5. Implement conservative fallback (worst-case state)

### Phase 4: Validation

1. All ParetoTimeEnergyTest tests pass
2. All ForwardPassTest tests pass
3. Terminal velocity within tolerance
4. No arc explosion (arc count < 1000 for typical paths)
5. Profile time within 3× theoretical minimum


## 8. Test Plan

| Test | Description | Key Check |
|------|-------------|-----------|
| P5 | Basic profile computation | points > 0, time > 0 |
| P6 | Rest-to-rest boundaries | v(0)≈0, v(L)≈0 |
| P7 | Velocity limit satisfied | v ≤ v_lim everywhere |
| P8 | Acceleration limit satisfied | |a| ≤ a_max |
| P9 | Jerk limit satisfied | |j| ≤ j_max |
| P10 | w_a=0 → time-optimal | time > 0, j* > 0 |
| P25 | Physical sanity | T < 3× t_bangbang |
| P26 | Jerk limit disabled | valid profile |
