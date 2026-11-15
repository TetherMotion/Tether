# H∞ Robust Control (`HInfinityController`)

`HInfinityController` implements the DGKF state-space solution to the
standard H∞ problem: given a generalized plant

```
xdot = A x + B1 w + B2 u
z    = C1 x + D11 w + D12 u
y    = C2 x + D21 w + D22 u
```

find a controller `K` with `||F_l(P, K)||∞ < γ`. Synthesis is offline;
the resulting LTI controller runs in `compute()` with no per-cycle
optimization.

## Algorithm

For a trial `γ`, two game-type algebraic Riccati equations are solved:

```
(A − B2 R⁻¹ D12'C1)'X + X(A − B2 R⁻¹ D12'C1)
    + C1'(I − D12 R⁻¹ D12')C1
    + X(γ⁻² B1B1' − B2 R⁻¹ B2')X = 0          (control ARE → X)

(A − B1 D21' Rt⁻¹ C2)Y + Y(A − B1 D21' Rt⁻¹ C2)'
    + B1(I − D21' Rt⁻¹ D21)B1'
    + Y(γ⁻² C1'C1 − C2' Rt⁻¹ C2)Y = 0        (filter ARE → Y)
```

with `R = D12'D12`, `Rt = D21 D21'`. Each CARE is solved by Hamiltonian
invariant-subspace iteration (stable eigenvectors → `X = Z2 Z1⁻¹`),
verified by residual, stabilizing-dynamics (`M + SX` Hurwitz), and
positive-semidefiniteness checks. Feasibility additionally requires the
coupling condition `ρ(XY) < γ²`, evaluated as a true spectral radius.
`designOptimal()` brackets `γ_min` and bisects.

The central controller (order = augmented plant order) is

```
F  = −R⁻¹(B2'X + D12'C1)
L  = −(Y C2' + B1 D21') Rt⁻¹
Z  = (I − γ⁻² YX)⁻¹
Ak = A + γ⁻² B1B1'X + B2F + Z L C2
Bk = Z L ,   Ck = −F ,   Dk = 0
```

`compute()` discretizes `(Ak, Bk)` once per sample period with the
Tustin (bilinear) transform — cached, so the runtime path is a plain
matrix recurrence.

## Two ways to describe the plant

### 1. `setPlant()` + weighting functions (mixed sensitivity)

```cpp
HInfinityController hinf;
hinf.setPlant(A, B, C, D, n, m, p);          // strictly proper plant (D = 0)
hinf.setSensitivityWeight(M, omegaB, eps);   // W1 on tracking error
hinf.setControlWeight(maxControl);           // W2 on control effort
hinf.setComplementaryWeight(M, omegaT, eps); // W3 on output
hinf.designOptimal();                        // or hinf.design(gamma)
```

The builder constructs the generalized plant

```
w = [r ; d ; n]          (reference, input disturbance, sensor noise)
z = [W1·e ; W2·u ; W3·y] (e = r − y)
y = r − Cx + n           (the error seen by the controller)
```

with each weight appended as dynamic states (order-0/1/2
`WeightingFunction` realizations; SISO weights replicated diagonally for
MIMO). If **no W2 is configured**, a small static control penalty
(`0.1·u`) is added automatically — `D12` must have full column rank or
the DGKF solution does not exist.

### 2. `setGeneralizedPlant()`

For problems that don't fit the mixed-sensitivity template, supply the
full generalized plant including `D11…D22`. The two-Riccati solution
requires the standard assumptions:

- `D11 = 0`, `D22 = 0` (no direct w→z or u→y feedthrough)
- `rank(D12) = nu`, `rank(D21) = ny`
- weights/augmented dynamics stable where applicable

Violations make `design()`/`isAchievable()` return `false` rather than
producing a meaningless controller. Feedthrough matrices are preserved
and used (previously they were silently dropped).

`getAugmentedDimensions()` / `getAugmentedPlant()` expose the assembled
generalized plant for inspection and verification.

## Weight design guidance

The weights are the entire tuning interface — there are no gain knobs.

| Weight | Shape | Role |
|---|---|---|
| `W1 = (s/M + ωB)/(s + ωBε)` | high at low freq (~1/ε), ~1/M at high freq | bounds sensitivity `S = (I+GK)⁻¹` — tracking + disturbance rejection |
| `W2 = firstOrder(0, 1, 1/umax)` | `s/(s+1)/umax`, high-pass | bounds control effort `KS` (penalizes HF control action) |
| `W3 = complementary(M, ωT, ε)` | ~1 at low freq → 1/ε at high freq | bounds complementary sensitivity `T = GK(I+GK)⁻¹` — noise/robustness |

**`ε` in W1 is the critical knob and it is counter-intuitive.** It sets
W1's DC asymptote `|W1(0)| ≈ 1/ε`, and `γ_min` scales roughly with it:
ε = 0.01 → γ ~ 80, ε = 0.1 → γ ~ 8, ε = 0.5 → γ ~ 2 on the test plant.
Tightening ε does **not** improve tracking — verified numerically: the
H∞ optimum trades `S(0)` against `T(0)` because W1 penalizes the error
from *all* exogenous channels (reference, disturbance, noise). Forcing
`S→0` (integral action) amplifies the noise path; the optimum is
interior. On a stable plant the central controller has no integrator,
so **steady-state tracking error is expected** — H∞ here is a
robustness/constraint layer, not a trajectory tracker. Use it where
worst-case disturbance rejection matters more than nominal tracking;
for tight tracking pair it with feedforward or integral action.

Weights must be **stable** (poles in the open LHP). `WeightingFunction::
integrator()` has a pole at the origin and will make synthesis fail —
use a small ε in `sensitivity()` instead.

## Limitations

- **Continuous-time synthesis.** The plant model is in continuous time;
  discretization happens only inside `compute()` via Tustin. For plants
  with sample-time-scale dynamics, design in discrete time yourself and
  use `setGeneralizedPlant()`.
- **No integrator on stable plants.** As above — expect steady-state
  error on setpoint/trajectory tracking unless W1's low-frequency
  asymptote forces near-integral action at the cost of a large γ.
- **`setPlant()` requires strictly proper plant** (`D ≈ 0`); biproper
  plants must go through `setGeneralizedPlant()`.
- **MIMO is supported structurally** (weights replicate diagonally;
  `w = [r;d;n]`, `z = [W1e;W2u;W3y]` per channel) but untested beyond
  SISO — cross-channel weighting is not implemented.
- **Runtime I/O is SISO-centric.** For `ny = 1` the controller input is
  `reference − measured`. For `ny > 1`, measurements must be supplied
  through `ControllerInput::state` (first `ny` entries).
- **Numerical robustness.** The Hamiltonian eigensolver is dense
  (`O(n³)`, `n ≤ MAX_AUG = 24`); fine for commissioning-time synthesis,
  not for online retuning inside a fast loop. Near-imaginary-axis
  Hamiltonian eigenvalues (marginally stable weights, tiny ε) can make
  `design()` fail — that is a genuine infeasibility signal, not a bug.
- **`MuSynthesisFramework` is implemented separately** — see
  `H2MuControl.md` for the μ upper bound and D-K iteration.

## Verification performed

- Closed-loop `||Tzw||∞` computed by explicit LFT + frequency sweep
  matches the design γ (e.g. γ = 81.2 → achieved 80.8) and `Acl` is
  Hurwitz.
- `design()`/`isAchievable()` correctly reject γ below γ_min and accept
  γ above; `designOptimal()` brackets and bisects γ_min.
- Generalized-plant feedthrough matrices (`D12`, `D21`) are retained and
  used; rank-deficient `D12` fails cleanly.
- Runtime `compute()` produces finite, bounded, stable closed-loop
  output (Tustin-discretized controller).
