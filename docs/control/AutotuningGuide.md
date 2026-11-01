# Autotuning Guide

How to go from *"I have a plant and no idea what gains to use"* to a
reproducible controller baseline.

Tether ships three complementary tuning paths:

| Path | Entry point | Needs | Best for |
|---|---|---|---|
| Model-based rules | `PIDController::autoTune(K, τ, θ, method)` | A FOPDT model of the plant | Production tuning, repeatable across machines |
| Ultimate-cycle rules | `PIDController::autoTuneFromUltimate(Ku, Tu, method)` | A relay or manual oscillation experiment | Quick baseline, legacy machines |
| Online relay autotuner | `RelayAutotuner` | Closed loop + a safe operating point | Automatic commissioning runs |

---

## 1. Identify the plant (FOPDT)

Most classical rules assume a **first-order-plus-dead-time** model:

```
        K · e^(-θs)
G(s) = ────────────
          τ·s + 1
```

- `K` — steady-state gain (output change / input change)
- `τ` — time constant (time to reach 63 % of the final value)
- `θ` — dead time (transport/integration delay before the output reacts)

You can obtain the model three ways:

1. **Step response** (simplest): apply a known step, log the response, then
   use `ProcessIdentification::tangentMethod`, `twoPointMethod`,
   `areaMethod` or `leastSquaresFit` from
   `tether/control/autotuning/classical/Common.hpp`.
2. **Relay experiment**: `RelayAutotuner` (below) yields `Ku`/`Tu`; convert
   to FOPDT if a rule needs it.
3. **Existing vendor data**: drive manuals usually list `K`, `τ`, `θ` per
   operating mode — start there, verify with a step.

> **Sanity check:** `θ/τ` is the controllability ratio. Above ~0.5 the
> plant is delay-dominated; prefer conservative rules (Lambda/SIMC/AMIGO)
> and expect slow loops. Ziegler-Nichols will oscillate.

---

## 2. Pick a rule

`PIDController::TuningMethod` selects the rule applied by
`autoTune(K, τ, θ, method)`:

| Method | Character | When to choose |
|---|---|---|
| `Lambda` | Smooth, monotone; closed-loop time constant `λ` sets speed | Default for servo axes; `λ ≈ 1–3·θ` is a good start |
| `AMIGO` | Robust (max sensitivity constrained) | Production machines, unknown disturbance spectrum |
| `SIMC` | Skogestad IMC, robust and simple | Similar to AMIGO, slightly faster |
| `CohenCoon` | Faster than Z-N, better for dead time | Processes with `θ/τ` in 0.1–1.0 |
| `ZieglerNichols` | Aggressive, overshoots | Only as a starting point for manual refinement |
| `TyreusLuyben` | Conservative, damped | Oscillation-prone loops, mechanical systems with backlash |
| `ZieglerNicholsPI` / `ZieglerNicholsPID` | Variants for `autoTuneFromUltimate` | Ultimate-cycle experiments |

```cpp
PIDController pid;
pid.autoTune(/*K=*/2.0, /*tau=*/0.15, /*theta=*/0.01,
             PIDController::TuningMethod::Lambda,
             /*lambda=*/0.03);   // 0 → rule default
```

---

## 3. Relay autotuning (Åström–Hägglund)

The relay tuner induces a controlled oscillation and measures the ultimate
gain `Ku` and period `Tu` without driving the plant to instability.

```cpp
#include "tether/control/PIDAutotuning.hpp"

RelayAutotuner::Config cfg;
cfg.relayAmplitude = 0.5;    // half the actuator range you dare to use
cfg.hysteresis     = 0.01;   // > noise amplitude, else you get chatter
cfg.minCycles      = 3;
cfg.maxCycles      = 20;

RelayAutotuner tuner(cfg);
tuner.start(/*setpoint=*/100.0);

for (;;) {
    const double u = tuner.update(measured, dt);   // feed to the plant
    if (tuner.isComplete()) break;
}

const auto r = tuner.getResults();
if (r.valid)
    pid.autoTuneFromUltimate(r.Ku, r.Tu,
                             PIDController::TuningMethod::ZieglerNicholsPID);
```

Practical notes:

- **Hysteresis must exceed measurement noise**, otherwise the relay
  switches on noise and the estimated `Tu` is garbage.
- The tuner needs the loop to be stable at the operating point — run it
  where the machine is safe to oscillate (typically mid-travel, no load).
- If `isComplete()` never fires, check `getState()`: `State::Idle` after
  `start()` means the setpoint was never reached.
- `r.valid == false` means the oscillation never stabilised within
  `maxCycles`; raise `relayAmplitude` or lower `stabilityMargin`.

---

## 4. Pattern-search autotuning

`PatternSearchAutotuner` optimises gains directly against a cost function
(settling time, overshoot, ISE — see `StandardCostFunctions` in
`tether/control/autotuning/AutotuningFramework.hpp`). Use it when:

- the plant is not well approximated by FOPDT, or
- you have a measurable cost and a safe way to run experiments.

It is slower than a rule-based tune (tens of experiments) but produces a
result tailored to your actual cost, not a textbook compromise.

---

## 5. The generic autotuning framework

`tether/control/autotuning/AutotuningFramework.hpp` provides the
building blocks for custom tuning pipelines:

- `ProcessModel` — plant abstraction (`evaluate(ω)`, `stepResponse`,
  `toFOPDT`, `getUltimateParams`)
- `TunableController` — anything exposing `ParameterDescriptors` /
  `ParameterVector`
- `CostFunction` — `StandardCostFunctions::{ISE,IAE,ITAE,ITSE,combinedCost}`
  or `SimulationCostFunction` (closed-loop simulation of a candidate)
- `OptimizationAlgorithm` — 14 optimisers (`createOptimizer(name)`,
  `getAvailableOptimizers()`): gradient descent variants, Nelder-Mead,
  PSO, simulated annealing, differential evolution, Bayesian optimisation,
  …

Minimal custom pipeline:

```cpp
using namespace tether::control::Autotuning;

auto opt = createOptimizer("nelder-mead");
opt->setTerminationCriteria({.maxIterations = 500});
opt->setTrackHistory(true);

SimulationCostFunction cost(controller, model, reference, dt);
auto result = opt->optimize(cost, initialParams, bounds);
controller->setParameters(result.bestParameters);
```

Rules of thumb:

- **Gradient methods** need a differentiable cost — use
  `SimulationCostFunction` only with smooth plants; finite differences on
  a discontinuous cost stall.
- **Population methods** (PSO, DE) handle non-smooth costs but need a
  budget: 20–50 individuals × 100+ iterations.
- **Bayesian optimisation** pays off when one evaluation is expensive
  (minutes), not for cheap simulations.
- Always set **bounds** — unconstrained optimisers find gain sets that
  saturate instantly.

---

## 6. Validating a tune

Do not trust a tune until you have seen:

1. **Step response** at several amplitudes (small and large — saturation
   changes everything).
2. **Disturbance rejection** (load step or torque impulse).
3. **Setpoint tracking** at the speeds you will actually run.
4. **Sensitivity margin**: with `AMIGO`/`SIMC` this is guaranteed; with
   Z-N it is usually poor. Increase `λ`/decrease `kp` until the loop stops
   ringing at the highest expected load.

Record the identified model, the chosen rule, the resulting gains and the
verification plots alongside the machine configuration — the next person
to touch the machine will need all four.

---

## 7. Common failure modes

| Symptom | Likely cause | Fix |
|---|---|---|
| Integrator saturates, huge overshoot | No/ineffective anti-windup | `setAntiWindup(BackCalculation, Tt)`; check `saturated` |
| Derivative kick on every setpoint change | Derivative on error | `setDerivativeOnMeasurement(true)` |
| Loop oscillates at `Tu/2` | Too much `kp` for the dead time | Use `AMIGO`/`Lambda`, increase `λ` |
| Autotune result "valid" but unusable | Relay hysteresis below noise floor | Raise `hysteresis`, re-run |
| Gains work at 1 kHz but not at 2 kHz | Discrete-time effects, filter `tf` too large | Reduce `tf`, re-check sampling |
| Different result every run | Non-deterministic experiment (load, temperature) | Fix operating conditions; use `setRandomSeed` |
