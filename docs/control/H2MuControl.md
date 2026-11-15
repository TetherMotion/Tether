# H2 Optimal and μ-Synthesis (`H2Controller`, `MuSynthesisFramework`)

Companion to `HInfinityControl.md`. This document covers the two remaining
robust-control classes in `include/tether/control/RobustControllers.hpp`:

- `H2Controller` — minimum-expected-cost (LQG-equivalent) output-feedback
  synthesis for the generalized plant.
- `MuSynthesisFramework` — structured-singular-value (μ) upper bounds and
  D-K iteration for block-structured uncertainty.

Both are **offline synthesis** tools: CAREs / eigensolvers run at design
time; the runtime path is a cached Tustin-discretized matrix recurrence.

---

## H2Controller

### Problem

For the generalized plant

```
xdot = A x + B1 w + B2 u
z    = C1 x + D11 w + D12 u
y    = C2 x + D21 w + D22 u
```

find a stabilizing controller `K` minimizing the closed-loop **H2 norm**
`||F_l(P,K)||₂` — the RMS of `z` under unit-intensity white `w`. For the
standard LQG regulator this is exactly the separation-principle optimum.

### Two ways to describe the problem

**`setRegulatorProblem(A, B, C, Q, R, W, V, n, m, p)`** — the convenient
front-end. It builds the generalized plant internally using symmetric
PSD square roots (`sqrtPSD`, eigenvalue-clamped) of the weight matrices:

```
plant:   x' = Ax + Bu + w0 ,  w0 ~ (0, W)
weights: z  = [sqrt(Q)·x ; sqrt(R)·u]
measure: y  = Cx + v     ,  v  ~ (0, V)

w = [w0 (n) ; v (p)]  ->  nw = n + p
B1 = [sqrt(W) | 0]        C1 = [sqrt(Q) ; 0]
D12 = [0 ; sqrt(R)]       D21 = [0 | sqrt(V)]
D11 = D22 = 0
```

`Q` penalizes **states** (not just the measured output) — this is the
regulator form. Passing `nullptr` for `Q/R/W/V` selects defaults
(`Q=R=I`, `W=V=0.01·I`). Dimension limits: `n ≤ 12`, `m ≤ 6`, `p ≤ 6`,
and the induced `nz = n+m ≤ 6`, `nw = n+p ≤ 12`; violations set an
internal invalid flag and `design()` returns `false`.

**`setGeneralizedPlant(...)`** — full LFT form, same signature as the
H∞ class. Requirements: `D11 = D22 = 0` (checked via stored max-norms),
`rank(D12) = nu`, `rank(D21) = ny`, `(A,B2)` stabilizable, `(C2,A)`
detectable. All failures return `false` cleanly.

### Algorithm

H2 is the `γ → ∞` limit of the DGKF solution, so `design()` solves the
two standard CAREs (the γ-dependent terms of the H∞ AREs vanish):

```
control: (A − B2R⁻¹D12'C1)'X + X(A − B2R⁻¹D12'C1)
           + C1'(I − D12R⁻¹D12')C1 − X B2R⁻¹B2' X = 0
filter:   (A − B1D21'Rt⁻¹C2)'Y + Y(A − B1D21'Rt⁻¹C2)'
           + B1(I − D21'Rt⁻¹D21)B1' − Y C2'Rt⁻¹C2 Y = 0
```

Both are solved by the same Hamiltonian invariant-subspace CARE solver
used for H∞ (`solveGameCARE`), with residual / Hurwitz / PSD checks.
The separation controller (order = plant order `n`) is

```
F  = −R⁻¹(B2'X + D12'C1)
L  = −(Y C2' + B1 D21') Rt⁻¹
Ak = A + B2F + L C2 ,   Bk = L ,   Ck = −F ,   Dk = 0
```

— identical to the H∞ central controller with `Z → I` and all `γ⁻²`
terms dropped.

### Achieved H2 norm

`getH2Norm()` is **computed, not assumed**: the closed loop
`[Acl Bcl; Cz Dzw]` is assembled via the same LFT helper used for
verification, the controllability Gramian `P` solves
`Acl·P + P·Acl' + Bcl·Bcl' = 0` (again via `solveGameCARE`), and

```
||Tzw||₂² = tr(Cz·P·Cz')        (Dzw = 0 under the D11 = D22 = 0 assumption)
```

If the closed loop is not stabilizable the norm stays `0` and
`design()` has already failed earlier on the CARE checks.

### Runtime

`computeImpl()` uses the measurement `y` **directly** — unlike the H∞
mixed-sensitivity controller there is no reference input in the
regulator problem, so SISO callers pass the raw sensor value in
`input.measured` (MIMO: first `ny` entries of `input.state`). The
continuous controller is Tustin-discretized once per `dt` (cached);
`reset()` clears the controller state and the discretization cache.

> **Tracking note:** the regulator minimizes disturbance rejection, not
> setpoint tracking — there is no integrator and no reference channel.
> For trajectory tracking, augment the plant with an integrator/reference
> model yourself and use `setGeneralizedPlant()`.

---

## MuSynthesisFramework

### What it computes

For a matrix `M` (real or complex `M(jω)`) and a block-structured
uncertainty set `Δ`, the structured singular value satisfies

```
μ_Δ(M) ≤ inf_{D ∈ 𝒟} σ̄(D M D⁻¹)
```

`computeMuUpperBound(M, n)` evaluates this upper bound by optimizing
the free D-scaling parameters with cyclic coordinate descent (golden
section per parameter, ~5 sweeps, mean pinned to zero since the
objective is shift-invariant).

**Commutant constraint (important):**

| Block type | `repeated` flag | Allowed D-scaling |
|---|---|---|
| size-1 scalar | either | one positive scalar |
| full k×k block | `false` | only `d·I_k` commutes with *all* k×k Δ → all k positions share **one** parameter |
| repeated scalar `δ·I_k` | `true` | any PD matrix commutes → each diagonal position gets its **own** parameter (diagonal restriction of the general PD scaling — conservative but valid) |

If the structure is unset or the block sizes don't tile `n`, the method
falls back to the unstructured bound `σ̄(M)` — still a valid (looser)
upper bound since `μ ≤ σ̄` always. The returned value is therefore
**always ≥ true μ**; it is never optimistic.

Both real (`const double*`) and complex (`const std::complex<double>*`)
row-major overloads exist; the complex path uses real positive D
(restriction of the general complex-D set → conservative, valid).

### D-K iteration

`dkIteration()` performs one full D-K step:

1. **K-step** — the current static D is folded into the generalized
   plant: the trailing `nUncW` input channels of `w` are right-scaled by
   `D⁻¹` and the trailing `nUncZ` output channels of `z` by `D`
   (applied consistently to `B1`, `C1`, `D11`, `D12`, `D21`). An
   `HInfinityController` then synthesizes the controller by optimal-γ
   bisection on the scaled plant — this is the standard "D-scaled H∞"
   subproblem.
2. **D-step** — with K fixed, the closed loop
   `[Acl Bcl; Cz Dzw]` is built once, and `M(jω)` (the `z_Δ × w_Δ`
   block of `F_u(P,K)`) is evaluated on a log-spaced frequency grid
   (`setFrequencyGrid`, default 0.1–1000 rad/s, 33 points). The static
   log-scales are refit by coordinate descent to minimize
   `max_ω σ̄(D·M(jω)·D⁻¹)`; `getMuBound()` reports that maximum.

`dkIterate(maxIter, tol)` loops until the relative bound improvement
drops below `tol` or `maxIter` is hit; `isConverged()` reports which.

**Honest limitations:**

- The D-scale is **static** (constant across frequency). A dynamic
  `D(s)` fit — the classical D-K iteration's second half — is *not*
  implemented; the static restriction gives a valid but more
  conservative bound.
- The K-step inherits all H∞ assumptions: `D11 = D22 = 0`,
  `rank(D12) = nu`, `rank(D21) = ny`, stabilizability/detectability.
  Violations make `dkIteration()` return `false` — it never claims
  progress it didn't make.
- Dimension limits: `n ≤ 12`, `nw ≤ 12`, `nu,ny ≤ 6`, `nz ≤ 6`,
  `nUncW = nUncZ ≤ 6`, ≤ 10 uncertainty blocks, ≤ 64 grid points.
- The uncertainty channels must be the **trailing** channels of `w`
  and `z` — order your generalized plant accordingly.
- Each iteration costs one H∞ synthesis (γ-bisection of two CARE
  solves) plus a grid of complex linear solves and SVDs — design-time
  only, not a runtime component.

### Verified behavior

- Known μ values reproduced exactly: `diag(2, 0.5)` → 2.0,
  nilpotent `[[0,1],[0,0]]` → ~0, swap matrix → 1.0 under scalar,
  full-block, and repeated-scalar structures.
- `[[1,2],[0,1]]` with two scalar blocks: bound 1.0 vs `σ̄ = 2.56` —
  D-scaling recovers the true `μ = 1`.
- End-to-end D-K on `x' = −x + u + w_d`, `y = x + w_n`: synthesized
  controller yields closed-loop bound 0.537, matching an independent
  frequency sweep of `|T_{w_d→x}(jω)|` (0.5375).

---

## When to use which

| Need | Class |
|---|---|
| Minimum RMS state/control effort under known noise covariances | `H2Controller` |
| Worst-case bounded disturbance rejection | `HInfinityController` |
| Robustness margin under *structured* parametric uncertainty | `MuSynthesisFramework` (analysis + D-K synthesis) |
| Runtime trajectory tracking for ovens | `OvenController` (see `OvenThermalControl.md`) |
