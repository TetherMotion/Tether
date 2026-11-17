# Oven Thermal Control

`OvenController` (`include/tether/control/thermal/`) is a generic
profile-following controller for heated chambers with **one-sided
actuation** (heat-only; cooling is passive). Every user-facing parameter
is physical — thermal coefficients, dead time, disturbance — never a raw
PID gain.

The feedback law is a **pluggable backend**: the default SIMC-PI for
collocated sensing, or an LQI state-feedback backend with a built-in
Kalman observer for **non-collocated sensing** (sensor measures chamber
air, the controlled variable is the product/load temperature). Both
backends share the same feedforward, governor, estimator, dither and
safety layers, and both support online retuning from the bounded physical
parameter estimator.

## Overview

- **Profile-following control** for heat-only chambers — ramps, soaks,
  and arbitrary trajectories via `TemperatureProfile` (see also the
  `ThermalCurve` preset library for standard process curves).
- **Physical-parameter design** — every knob is a thermal coefficient,
  dead time, or disturbance estimate. Controller gains are *derived*
  (SIMC rules / Riccati), never hand-set.
- **Model-inverse feedforward** previewed by the dead time, plus an
  **actuation governor** that pre-shapes references into what passive
  cooling can actually achieve.
- **Two feedback backends**: SIMC-PI (collocated sensor) or
  LQI + Kalman observer (non-collocated — e.g. air sensor controlling
  product temperature), swappable at runtime.
- **Online adaptation**: a bounded physical-parameter estimator
  identifies `{α, β, d}` from routine I/O and retunes feedforward every
  cycle and feedback gains at a gated supervisory cadence.
- **Safety**: output clamps, slew rate limit, over-temperature
  interlock, conditional-integration anti-windup.
- **Multi-zone**: `OvenGradientController` supervises one
  `OvenController` per zone with gradient references and coupling
  feedforward.

## Quick start

```cpp
// 1. Commission the thermal model (see "Commissioning"):
auto tune = autoTuneOven(stepResponse);   // step test -> {alpha,beta,delay,...}

// 2. Configure:
OvenControllerConfig cfg;
cfg.model = tune.params;                  // physical FOPDT params
// cfg.feedbackBackend = OvenFeedbackBackendKind::LQI;  // non-collocated only
OvenController ctrl(cfg);
ctrl.setProfile(myProfile);               // or leave unset for setpoint mode

// 3. Run the control loop:
ControllerInput in;
in.measured = readThermistor();
in.dt = actualElapsedSeconds();           // real dt, not nominal
in.reference = 80.0;                      // setpoint mode only
auto u = ctrl.compute(in).control;        // write u to the heater
```

`ctrl.status()` exposes the governed reference, filtered measurement,
feedforward/feedback split, live `{alpha,beta,disturbance}` estimates,
and saturation/over-temp flags for monitoring.

## Which setup do I need?

| Your situation | Use |
|---|---|
| Sensor measures the mass you want to control (collocated) | `PI` backend (default) |
| Sensor is chamber air; product/load is the controlled mass | `LQI` backend |
| Step response clearly shows **two** lags on the sensor path | `modelOrder = Sopdt` |
| Several zones with a spatial or conveyor-staggered gradient | `OvenGradientController` |
| Standard process curves (reflow, wave, selective, brazing, anneal) | `ThermalCurve` presets |
| Step test possible | `autoTuneOven()` / `autoTuneOvenSOPDT()` → `applyAutotune()` |
| No step test | Set `cfg.model`/`cfg.modelSopdt` to physical estimates; gains auto-derive |

Details on model order (FOPDT vs. SOPDT vs. alternatives) and the
method-selection knobs are in
[Model support](#model-support-fopdt-sopdt-and-the-alternatives) below.

## Plant model

`cfg.modelOrder` selects the plant model — **user-selected, never
inferred**. Both are heat-loss (FOPDT-class) models: constant heater
power reaches a finite temperature, so passive cooling and a finite DC
gain are always preserved.

### FOPDT (default) — single lossy mass

```
dT/dt = α·u(t−θ) − β·(T − T_amb) + d(t)
```

| Param | Meaning | Units | Physical origin |
|---|---|---|---|
| `alpha` | Heating coefficient | °C/s per unit u | η·P_max / C_th |
| `beta` | Loss coefficient | 1/s | 1 / (R_th·C_th) |
| `delay` | Input dead time | s | heater→air→sensor transport + sensor lag |
| `disturbance` | Additive heat-flow estimate | °C/s | Q_load / C_th (door, airflow, load) |
| `ambient` | Ambient temperature | °C | — |
| `uMin/uMax` | Actuator bounds | — | typically [0, 1] (PWM fraction) |

Derived quantities: DC gain `K = α/β` [°C/u], time constant `τ = 1/β` [s].
If the real power-per-unit-u is known, an effective heat capacity is
`C_eff = P_scale / α` [J/K].

**Identifiability note:** from input/output data alone, `C_th` and `R_th`
cannot be separated — only the combinations `α`, `β` (equivalently `K`,
`τ`) are identifiable. That is what the estimator produces.

### SOPDT — cascade of two masses

When the step response shows **two** lags on the sensor path, select
`modelOrder = OvenModelOrder::Sopdt`. The model is a physical cascade —
not an anonymous transfer function:

```
dT1/dt = α·u(t−θ) − β·(T1 − T_amb) + d     ← stage 1: lossy heater/air mass
dT2/dt = (T1 − T2) / τ2                      ← stage 2: pure-lag product/sensor mass
y      = T2                                  ← controlled output
```

Stage 1 is exactly the FOPDT mass (heater + chamber air, with heat loss
to ambient); stage 2 is a second thermal mass with **no direct ambient
loss** (product, load, or a secondary sensor mass) — it only exchanges
heat with stage 1. Input→output transfer:

```
G(s) = K·e^(−Ls) / ((τ1·s + 1)(τ2·s + 1)),   K = α/β,  τ1 = 1/β
```

| Param | Meaning | Units |
|---|---|---|
| `alpha`, `beta` | Stage-1 heating / loss coefficients | °C/s/u, 1/s |
| `tau2` | Stage-2 lag time constant | s (`0` → recovers FOPDT exactly) |
| `delay`, `disturbance`, `ambient`, `uMin/uMax` | Same semantics as FOPDT | |

`OvenModelParamsSOPDT::fromFopdtParams(model, tau2)` projects an existing
FOPDT set onto the cascade (only `tau2` is genuinely new — stage-1
parameters are solved so the *effective* FOPDT dynamics `{K, τ_eff}` are
preserved); `toFopdtParams()` folds the cascade back to an equivalent
FOPDT for the governor. `effectiveAlphaBeta()` gives the quasi-static
output-stage parameters `{α_eff = α/(1+βτ2), β_eff = β/(1+βτ2)}` used
for feasibility rates — DC gain is unchanged (`α_eff/β_eff = α/β`).

## Model support: FOPDT, SOPDT, and the alternatives

`OvenController` supports **two** model orders, and **you choose every
method** — nothing is inferred or silently swapped:

| Knob | Options | Default |
|---|---|---|
| `modelOrder` | `Fopdt`, `Sopdt` | `Fopdt` |
| `ffMethod` | `ModelInverse`, `DominantLag`, `None` | `ModelInverse` |
| `estimateMethod` | `FopdtIntegral`, `SopdtOe`, `Off` | `FopdtIntegral` |
| `tuneMethod` | `Simc`, `ImcPi`, `ImcPid`, `Manual` | `Simc` |
| `feedbackBackend` | `PI`, `LQI` | `PI` |

The active selections are echoed in `status()` (`modelOrder`, `ffMethod`,
`estimateMethod`, `tuneMethod`, `backendName`) so you can always verify
what is running.

### FOPDT vs. SOPDT — which to pick

- **FOPDT** — one lossy lag describes the sensor path. Right for fast
  ovens, thin loads, or when the sensor sees essentially the controlled
  mass. Simplest, best-identified, cheapest to run.
- **SOPDT** — the step response clearly shows **two** lags (a fast rise
  then a slow tail). The cascade model puts the loss on stage 1
  (heater/air mass) and a pure lag on stage 2 (product/load/sensor mass).
  `τ2 = 0` recovers FOPDT exactly, so there is no modeling penalty for
  trying it — but see the identifiability caveat below.

To enable SOPDT:

```cpp
cfg.modelOrder = OvenModelOrder::Sopdt;
cfg.modelSopdt = OvenModelParamsSOPDT::fromFopdtParams(cfg.model, 120.0);
// or autoTuneOvenSOPDT() on a step test → applyAutotuneSOPDT()
cfg.estimateMethod = OvenEstimateMethod::SopdtOe;   // optional online fit
cfg.ffMethod = OvenFeedforwardMethod::ModelInverse; // exact two-stage inverse
```

### Feedforward methods (`ffMethod`)

- **`ModelInverse`** — exact inverse of the *active* model order.
  FOPDT: `u = [ṙ + β(r−T_amb) − d]/α`. SOPDT: inverts both stages —
  stage 2 tracking `r` requires stage 1 to follow `r + τ2·ṙ`, giving
  `u = [ṙ + τ2·r̈ + β(r + τ2·ṙ − T_amb) − d]/α`. Exact on ramp/soak
  profiles; `r̈` at profile knots is finite-differenced over
  `sopdtFFAccelWindow`.
- **`DominantLag`** — quasi-static FOPDT-equivalent inverse that ignores
  the `τ2` compensation term. Simpler and cheaper, at the cost of extra
  lag the feedback must trim. Identical to `ModelInverse` for FOPDT.
- **`None`** — feedback only.

### Estimator methods (`estimateMethod`)

- **`FopdtIntegral`** (default) — windowed integral regression →
  `{α̂, β̂, d̂}`. Noise-robust (integrates over the window). With
  `modelOrder=Sopdt` it updates **stage-1** `{α,β,d}` and holds `τ2`
  fixed — a projection, not a full SOPDT fit.
- **`SopdtOe`** — trust-region **output-error** fit → `{α̂, β̂, τ2̂, d̂}`.
  Re-simulates the exact cascade over a sliding window for a grid of
  `{τ1, τ2}` lag candidates and solves `{α, d}` in closed form (the
  cascade is linear in `v = αu + d`) for each — only the two lags need
  a nonlinear search. Bounded to a multiplicative trust region around
  the prior, excitation-gated, EMA-smoothed. Needs richer excitation
  (dither or setpoint changes) and a scored window of several
  slowest-lag time constants (auto-floored at
  `sopdtEstimator.windowLagFactor·(τ1+τ2)`).
- **`Off`** — no online estimation (same as `adaptParams=false`).

> **SOPDT identifiability caveat.** The input→output transfer
> `K/((τ1s+1)(τ2s+1))` is **symmetric in the two lags** — from a single
> sensor you can identify the DC gain `K` and the *set* `{τ1, τ2}` but
> not which lag belongs to which physical stage. The OE fit therefore
> recovers `K` and the lag sum reliably while the stage labeling stays
> near the prior (bounded by the trust region). This is fine for
> control — the model self-consistently inverts whatever split it
> holds — but do not read `tau1`/`tau2` as "heater lag" vs "product
> lag" without independent evidence (e.g. a second sensor or a
> cool-down test). If you need the *physical* split, the **LQI backend**
> is the structured answer (see below).

### Tuning methods (`tuneMethod`)

SIMC / IMC-PID rules are applied to the **active** model order:

- **FOPDT** — SIMC → PI; IMC-PI → PI; IMC-PID → PID.
- **SOPDT** — SIMC → PID with `Td = τ_secondary`; IMC-PI → PI on
  `Στ = τ1+τ2`; IMC-PID → PID with `Td = τ1·τ2/Στ`. The derivative term
  is what compensates the second lag, so prefer `ImcPid`/`Simc` (both
  produce `kd`) over `ImcPi` when `τ2` is significant.
- **`Manual`** — use `cfg.kp/ki/kd` verbatim.

`autoTuneOvenSOPDT(StepResponseData, method, tauC)` is the SOPDT
commissioning counterpart to `autoTuneOven()`: it fits the symmetric
two-pole step response (grid + pattern refinement over `{τa, τb, θ}`),
maps the **fast** pole to stage-1 `β` and the **slow** pole to `τ2`
(product mass is typically the slower lag), and returns
`OvenAutotuneResultSOPDT{params, kp, ki, kd}` → `applyAutotuneSOPDT()`.

### Where the alternatives fit

The generic autotuning library also exposes SOPDT directly:
`IMCDesign::designForSOPDT()`, `SIMCMethod`, and `AutotuningFramework`
accept a `SOPDTModel{K, τ1, τ2, L}` for **non-oven loops** — separate
from the oven path above.

**Why SOPDT-in-`OvenController` is a cascade, not a bare transfer
function:** a bare `K/((τ1s+1)(τ2s+1))` has no restoring heat loss — it
would drift without bound under constant input. The cascade keeps the
loss on stage 1, preserving finite DC gain, ambient coupling, and the
`{α, β, d}` physical semantics the estimator and governor rely on.

**LQI backend vs. SOPDT model order** — they solve different problems:

- `modelOrder=Sopdt` models **two lags on the single sensor path**
  (input→sensor has two time constants). Best when the *sensor itself*
  is slow (e.g. a thermistor in a heavy housing) but you still control
  to the sensor.
- `LQI` backend models **two physical zones** — sensor-side air and an
  unmeasured product mass — with a Kalman observer reconstructing the
  product temperature and integral action on the *product* error. This
  is the answer when the controlled mass is **not** what the sensor
  sees (non-collocated), and it attaches physics to the second state
  rather than leaving it an anonymous lag.

For the common cases:

- **Collocated sensor, one effective lag** → `PI` backend + `Fopdt`.
- **Collocated sensor, two visible lags** → `PI` backend + `Sopdt`.
- **Non-collocated sensor** (air sensor, product/load controlled) →
  `LQI` backend — the structured two-zone answer.
- **MIMO / multi-zone** → `OvenGradientController` (see below).

**IPDT** (pure integrator) is **never** correct for an oven — heat loss
to ambient gives a finite DC gain, so constant power settles.

**More than two lags / distributed parameters** (thick product with
internal gradients, vaporization fronts) are not modeled. The honest
options are the LQI coupled model, per-zone SISO loops, or an external
robust-design workflow (`HInfinityController`/`MuSynthesisController`)
for plants outside the oven thermal class.

If tracking shows a persistent lag that grows with profile rate beyond
what feedback covers, the likely cause is unmodeled higher-order
dynamics, sensor placement, or the SOPDT lag-split ambiguity — revisit
the backend/model class before raising gains.

## Control structure

```
TemperatureProfile ──► ActuationGovernor ──► r̂(t)   (achievable reference)
                              │                    │
                              │              model-inverse FF (ffMethod)
                              │              u_ff = requiredInput(r̂, ṙ̂, r̈̂, …)
                              │                    │
                        PI feedback ◄── T_filt      │
                        (tuneMethod gains)          ▼
                              u = clamp(u_ff + u_PI + dither, uMin, uMax)
                                        │
              estimator (estimateMethod) ──► {α̂, β̂, [τ2̂], d̂}
              FopdtIntegral | SopdtOe            │
                                          updates model + retunes PI
```

- **Feedforward** inverts the *active* model order along the *governed*
  reference, previewed by `delay` seconds so power arrives when needed.
  For SOPDT the exact inverse also needs `r̈` — finite-differenced over
  `sopdtFFAccelWindow` at profile knots.
- **Governor** rate-limits the reference into the achievable band —
  on heat-only actuation a descent faster than passive cooling allows
  is pre-shaped to the passive rate instead of being left to feedback.
  For SOPDT it uses the quasi-static effective parameters
  (`effectiveAlphaBeta()`), so feasibility accounts for the second lag.
- **PI feedback** is tuned by the selected `tuneMethod` rules *from the
  physical parameters* — never hand-set unless `Manual`. Its saturation
  limits are `[uMin−u_ff, uMax−u_ff]` so back-calculation anti-windup
  sees the true remaining actuator authority. SOPDT tuning produces a
  derivative term (`kd`) that compensates the second lag.
- **Estimator** is selected by `estimateMethod` (see "Estimator
  methods"): the FOPDT integral regression, or the SOPDT output-error
  fit. Both are bounded, excitation-gated (skip steady soaks where
  parameters are unidentifiable), and EMA-smoothed.
- **Dither**: a small sinusoidal perturbation on `u` while adaptation
  is enabled — *required* for meaningful closed-loop identification,
  especially `SopdtOe`. In closed loop `u ≈ f(T)` otherwise, leaving the
  parameters identifiable only up to a low-dimensional ambiguity; the
  dither decorrelates the regressors (~0.1–0.3 °C measurement ripple for
  typical parameters).

## Feedback backends

`cfg.feedbackBackend` selects the feedback block; `setFeedbackBackend()`
can also swap it at runtime (`nullptr` restores the configured default).
The swap is one virtual `ControllerBase::compute()` call per cycle —
negligible at oven loop rates — and the FF/governor/estimator layers are
untouched.

### `PI` (default) — collocated sensor

SIMC PI on the filtered measurement, as described above. Correct when
the sensor sees essentially the thermal mass being controlled.

### `LQI` — non-collocated sensor

Use when the sensor does **not** measure what should follow the profile
(chamber-air thermistor vs. product core). `OvenLQIBackend`
(`OvenLQIBackend.hpp`) implements:

```
measured y (zone 1) ──► OvenThermalObserver ──► x̂ = [T̂_s, T̂_p]
                             ▲                        │
             total applied u ─┘ (delay line)          ▼
                                       LQIController, C = [0,1]
                                       u_fb on deviation states z = x̂ − x_ss(r)
```

- **Two-state plant** (`OvenCoupledParams`): zone 1 (sensor side,
  heated directly) coupled to zone 2 (product, controlled). Parameters
  `alpha, betaSensor, betaProduct, gammaSP, gammaPS, delay,
  disturbance, ambient, uMin/uMax` — physical RC rates, see
  `OvenCoupledModel.hpp` for the equations and `makeCoupledParams()` to
  derive a nominal set from the FOPDT model.
- **Observer**: steady-state Kalman filter (2×2 predictor Riccati solved
  once per redesign). It predicts with the *total applied* input —
  feedforward + feedback + dither, after clamping and rate limiting —
  reported via `IOvenAdaptiveFeedback::notifyAppliedInput()`, so FF is
  not misread as process noise. Corrects with the raw measurement.
- **LQI**: `LQIController` on the augmented (state + integral) system,
  `C = [0,1]` so the integrator drives the *product* temperature to the
  reference. The regulator is designed **discrete-time**: the backend
  Euler-discretizes the coupled plant at `backendDt` and solves the
  discrete algebraic Riccati equation on the augmented pair
  (`xi[k+1] = xi[k] + dt·(r − y)` — the same update `computeImpl()`
  runs), so the gains match the sampled-data loop exactly. It is fed
  **deviation states** `z = x̂ − x_ss(r)` with
  `reference = 0`: Tether's LQI penalizes absolute state, and feeding
  ~100 °C absolutes would create an uncorrectable ~10⁴ bias. The analytic
  steady-state map `x1_ss = r + (βP·(r−T_amb) − d)/γPS` also gives
  implicit feedforward during ramps.
- **Weights** `lqiQxSensor/QxProduct/Qi/R` are performance weights, not
  physical — defaults are verified non-oscillatory for typical oven time
  constants; raise `lqiR` for softer action.
- **Online retune**: the same gated `maybeRetune()` path pushes bounded
  `{α̂, β̂}` into `updateThermalParameters()`, which re-derives observer
  gain + LQI gains. `d̂` is deliberately *not* mapped — the FOPDT
  disturbance estimate conflates true loads with single-vs-two-state
  structural mismatch; real load disturbances are rejected by the LQI
  integral instead.

**Limitations:** the feedforward still uses the SISO FOPDT model
(approximate for the coupled plant — the LQI trims the residual, and the
governor's feasibility band is sensor-path based); coupled-model
parameters beyond `{α, β}` (coupling rates, loss split) are
commissioning constants — there is no coupled-model autotune yet.

## Commissioning (autotune)

`autoTuneOven(StepResponseData)` consumes a recorded step response —
heater 0 → `uStep` held constant, temperature sampled at uniform `dt`:

```cpp
StepResponseData d;
d.uStep = 0.4; d.tAmb = 25.0; d.uMin = 0; d.uMax = 1;
// fill d.time / d.temp from the recording (>= ~5τ duration)

auto r = autoTuneOven(d);          // FOPDT via 10%/90% two-point method
if (r.success) ctrl.applyAutotune(r);
```

The result carries `params` (`OvenModelParams`) and SIMC `kp/ki`.
Requirements: step amplitude large enough for a clear response
(`minDeltaT`, default 1 °C), duration ≥ ~5τ for steady state, and the
pre-step input must be 0/off (the delay-line assumes heater off before
t=0). `identifyFopdt`/`simcPiGains` are also exposed separately.

For SOPDT, `autoTuneOvenSOPDT(StepResponseData, method, tauC)` fits the
symmetric two-pole step response and returns
`OvenAutotuneResultSOPDT` — apply it with `applyAutotuneSOPDT()` (sets
`modelOrder=Sopdt`, `modelSopdt`, and the tuned gains in one shot):

```cpp
auto r = autoTuneOvenSOPDT(d, OvenTuneMethod::Simc);
if (r.success) ctrl.applyAutotuneSOPDT(r);
```

The SOPDT fit maps the **fast** pole to stage-1 `β` and the **slow**
pole to `τ2` (product mass is typically the slower lag). Because the
two-pole response is symmetric, the pole *set* is well-identified but
the stage labeling is a convention — see the identifiability caveat
under "Model support".

If no step test is possible, set `cfg.model` (FOPDT) or
`cfg.modelSopdt` (SOPDT, via `fromFopdtParams(model, tau2)`) to
best-estimate physical parameters and leave `kp = ki = kd = 0` — gains
derive automatically.

## Runtime usage

```cpp
OvenControllerConfig cfg;
cfg.model = nominalParams;        // or applyAutotune() result
// tune knobs as needed (see below)
OvenController ctrl(cfg);
ctrl.setProfile(profile);          // or leave unset for setpoint mode

ControllerInput in;
in.measured = readThermistor();
in.dt = actualElapsedSeconds();   // real dt, not nominal
in.reference = 80.0;               // setpoint mode only (ignored w/ profile)
auto out = ctrl.compute(in);
writeHeater(out.control);
```

`ctrl.status()` exposes `reference` (governed), `measuredFiltered`,
`feedforward`/`feedback` split, live `{alpha,beta,disturbance}`,
`windowsUsed`, `saturated`, `overTemp` for monitoring.

## Online retuning — what it does and does not do

Every control cycle (when `adaptParams` and `estimateMethod != Off`):

1. The estimator integrates I/O data and updates the model parameters —
   but only for windows showing real excitation (input or temperature
   variation above noise). Which parameters depends on
   `estimateMethod`/`modelOrder`:
   - `FopdtIntegral` → `{α̂, β̂, d̂}` (with `modelOrder=Sopdt` this updates
     stage-1 `{α,β,d}` and **holds `τ2` fixed** — a projection, not a
     full SOPDT fit).
   - `SopdtOe` → `{α̂, β̂, τ2̂, d̂}` — the OE fit does move `τ2`, but only
     within its trust region and subject to the lag-split ambiguity
     (see "Model support").
2. The estimates are written into the feedforward model immediately, so
   load changes (heavier product → larger effective `C_th` → smaller
   `α`; door open → `d` excursion) are absorbed within seconds.
3. Feedback gains are **re-derived via `tuneMethod`** at most every
   `retuneInterval`, only after `retuneMinWindows` new excited updates
   and only if they changed by > `retuneMinDelta`. For SOPDT the
   re-derivation uses the *effective* parameters (the cascade's
   quasi-static `{α_eff, β_eff}`), so `τ2` enters the tuning through
   `Στ`/`Td` rather than double-counting. Gain changes are bumpless
   (the integrator stores an output-units contribution).

**Deliberate limits:**

- `delay` (θ) is **not** adapted — dead time is not identifiable from
  routine closed-loop data. Re-run autotune if the mechanics change.
- `beta` is a single global value — it is *not* temperature-scheduled.
  `R_th` drifts with temperature (radiation ∝ T⁴); the estimator tracks
  the average, and feedback covers the residual. If a wide range
  (e.g. 25→400 °C) shows systematic tracking bias vs temperature,
  consider scheduling `β(T)` externally via `setModelParams()`.
- During long soaks the estimator holds (correctly — nothing is
  identifiable); the PI integral absorbs steady offsets.

## Tuning per backend: commissioning and online retune

The two backends share the *estimation* layer but differ in what a
retune means. This section is the practical checklist for each.

### PI backend — initial tuning

1. Commission with `autoTuneOven()` on a step test → `applyAutotune()`
   sets `{α, β, θ, d}` *and* SIMC gains in one shot.
2. Or set `cfg.model` to physical estimates and leave `kp = ki = 0` —
   gains derive automatically. Never hand-tune `kp/ki` unless you also
   set `adaptParams=false` (otherwise retune overwrites them).
3. `simcTauC` is the single performance knob: `0` → `τc = θ` (aggressive
   but delay-robust); raise toward `τ/10` for softer response.
4. `sensorTau` trades noise rejection vs. loop lag — start at 3–8 s for
   thermistors.

### PI backend — online retune

- `{α̂, β̂, d̂}` write into the FF model **every cycle** (immediate).
- SIMC gains re-derive at most every `retuneInterval`, gated by
  `retuneMinWindows`/`retuneMinDelta`. **Bumpless**: the PI integrator
  stores an output-units contribution, so a gain swap does not move `u`.
- `d̂` enters the FF numerator directly — endothermic/exothermic loads
  are absorbed as feedforward, not just feedback.

### LQI backend — initial tuning

The LQI backend still needs the **FOPDT autotune for the SISO layer**
(`cfg.model` feeds the estimator, FF model and governor). On top of that:

1. `cfg.coupled = makeCoupledParams(cfg.model)` gives a nominal
   two-state plant. The split parameters (`betaProduct`, `gammaSP`,
   `gammaPS`) are *not* identified by the SISO autotune — refine them
   from dedicated experiments:
   - **Empty vs. loaded step response** → capacitance split (product
     mass raises zone-2 share → lowers `gammaPS` relative to `gammaSP`).
   - **Cool-down curve** (heater off, record both zones if possible, or
     sensor only) → loss rates `betaSensor`/`betaProduct`.
   - Rule of thumb: `gammaSP·C_s = gammaPS·C_p = G_couple` — the two
     coupling rates share one conductance; their ratio is the inverse
     capacitance ratio.
2. **LQI weights are performance knobs, not physics.** Defaults
   (`qxSensor=0.01, qxProduct=0.004, qi=0.02, R=1000`) are verified
   smooth for typical oven time constants. Direction: raise `lqiR` for
   softer/slower, lower for more aggressive — too low produces a
   limit cycle (u banging 0↔1 around setpoint). `qi` sets integral
   authority vs. disturbance rejection speed.
3. **Observer bandwidth**: `observerQProc/observerRMeas` ratio is the
   knob. `observerRMeas` ≈ sensor variance (σ=0.3 °C → 0.09). Larger
   ratio → faster state convergence, more measurement-noise feedthrough
   into `x̂_p` (and thence into `u`).
4. `backendDt` ≈ actual loop `dt` — the observer/LQI are designed at
   this step.

### LQI backend — online retune

- Same gated `maybeRetune()` cadence (`retuneInterval`,
  `retuneMinWindows`, `retuneMinDelta` — evaluated on `{α̂, β̂}`).
- On retune, `updateThermalParameters(α̂, β̂)` re-derives the observer
  gain *and* re-solves the augmented LQI Riccati — small (3-state DARE +
  2×2 iteration), but it runs inline at supervisory cadence, never per
  cycle budget.
- **`d̂` is not mapped** into the coupled plant — the FOPDT disturbance
  estimate conflates true loads with single-vs-two-state structural
  mismatch. Real load disturbances are rejected by the LQI integrator
  (verified: −0.03 °C/s endothermic load → <3 °C soak error).
- **Not perfectly bumpless**: unlike the PI (output-units integrator),
  the LQI integrator `xᵢ` is a *state* in °C·s — a gain swap rescales
  `Ki·xᵢ` instantly, producing a small `u` step. Gating keeps changes
  small and the thermal plant low-passes the kick; if bumps are visible,
  increase `retuneMinDelta`/`retuneInterval`.
- The observer gain also changes on retune → `x̂` shifts slightly;
  same gating covers it.

### What online retuning cannot fix (either backend)

- **Wrong model class**: if the step response shows two lags but you run
  FOPDT, no amount of `{α,β}` adaptation fixes the *structure* — but
  this is now a **configuration fix, not a limitation**: set
  `modelOrder=Sopdt` (and `estimateMethod=SopdtOe` if you want `τ2`
  tracked online). The residual case is >2 lags or non-collocated
  sensing — use the LQI backend's structured two-zone model.
- **SOPDT lag-split ambiguity**: `SopdtOe` identifies `K` and the lag
  *set* reliably but cannot prove which lag is stage-1 vs stage-2 —
  the labeling stays near the prior. If the physical split matters
  (e.g. you need to know the product mass lag), get independent
  evidence or use the LQI backend.
- **Delay changes**: θ is fixed online; re-run autotune.
- **Strong β(T)**: radiation losses grow ~T⁴; the estimator tracks the
  average. Persistent bias vs. temperature → schedule `β(T)` via
  `setModelParams()` or accept the feedback trim.
- **Coupling/loss-split drift** (LQI): `gammaSP/gammaPS/betaProduct`
  ratio are commissioning constants — product mass changes that alter
  the *split* (not just overall gain) need re-commissioning or a
  coupled-model estimator (not implemented).

### Monitoring during adaptation

`status()` exposes the health signals: `alpha/beta/disturbance` (should
settle near commissioning values; large drift = load change or wrong
bounds), `tau2` (SOPDT stage-2 lag; `0` in FOPDT mode), `windowsUsed`
(stalls during soaks are *expected* — gating), `saturated` (sustained
true = actuator-limited, not a tuning problem), `backendName`, and the
active method selections (`modelOrder`, `ffMethod`, `estimateMethod`,
`tuneMethod`). Estimator bounds (`estimator.{alpha,beta,d}{Min,Max}` and
`sopdtEstimator.*`) are the real safety envelope on adaptation —
tighten them to the physically possible range for your oven.

## Tuning knobs — when to change what

Knobs marked † are PI-backend only; ‡ are LQI-backend only; § apply
only when `modelOrder=Sopdt`. Everything else applies to both model
orders.

| Knob | Default | Change when… |
|---|---|---|
| `modelOrder` § | `Fopdt` | Step response shows two lags → `Sopdt`. Never inferred — set explicitly. |
| `modelSopdt` § | — | SOPDT physical params. Build via `fromFopdtParams(model, tau2)` or `autoTuneOvenSOPDT()`. |
| `ffMethod` | `ModelInverse` | `DominantLag` for a cheaper/softer FF (more lag); `None` for feedback-only. `ModelInverse` is exact for the active model order. |
| `estimateMethod` | `FopdtIntegral` | `SopdtOe` to track `τ2` online (needs richer excitation + longer window). `Off` disables estimation. |
| `tuneMethod` | `Simc` | `ImcPi`/`ImcPid` for IMC tuning; `Manual` to use `kp/ki/kd` verbatim. SOPDT prefers a method that yields `kd` (`Simc`/`ImcPid`). |
| `sopdtFFAccelWindow` § | 4 s | Half-width for finite-differencing `r̈` at profile knots in the exact SOPDT FF. Larger = softer FF kicks. |
| `sopdtEstimator.windowLagFactor` § | 4.0 | Scored-window floor = this × (τ1+τ2). Smaller → faster but the lag split becomes unidentifiable (fit drifts); larger → slower, tighter split. |
| `sopdtEstimator.windowTime` § | 300 s | Absolute scored-window minimum [s]; the lag-factor floor usually dominates for slow ovens. |
| `sopdtEstimator.spinupFactor` § | 4.0 | Extra history (× lag sum) before the scored window so the assumed initial state decays. |
| `sopdtEstimator.simStride` § | 4.0 | OE sim decimation (fit runs at `dt·simStride`). Higher = cheaper; too high aliases the input. |
| `sopdtEstimator.{lagSpanLo,lagSpanHi,wLag,wAD,smoothing}` § | 0.35/2.8/0.15/0.05/0.3 | Trust-region + regularization around the prior — keep tight for bounded online refinement. |
| `model.*` | required | After autotune, or manual physical estimates. Bounds `[uMin,uMax]` are hard safety limits — set before tuning. |
| `simcTauC` † | 0 → τc=θ | Want slower/more robust response: raise (e.g. `2*delay` or `tau/10`). Faster: lower, but keep ≥ θ for delay robustness. |
| `kp/ki/kd` † | 0 → auto | Only to override auto-tuning entirely (then retune is effectively bypassed — set `adaptParams=false` to be explicit). `kd` matters for SOPDT (`Simc`/`ImcPid` produce it). |
| `sensorTau` | 3 s | Noisier sensor → raise (5–15 s). Clean sensor → lower (1–2 s). The FOPDT integral estimator uses the raw signal; the SOPDT OE fit uses the *filtered* measurement, so `sensorTau` also steadies `SopdtOe`. |
| `estimator.windowTime` | 15 s | Faster adaptation → shorter (≥ ~3–5 s). Noisier data → longer. Should be ≪ profile feature times. |
| `estimator.memTime` | 300 s | How fast old data is forgotten. Slow load drift → longer; abrupt load changes → shorter. |
| `estimator.{alpha,beta,d}{Min,Max}` | wide | Tighten to the physically possible envelope for your oven — this is the real safety bound on adaptation. |
| `ditherAmp` | 0.02 | 0 disables (estimator will be biased in closed loop — only do this if `adaptParams=false`). Larger → better ID, more ripple. |
| `retuneInterval` | 30 s | Minimum seconds between gain retunes. |
| `retuneMinWindows` | 60 | Excited updates (~30 s at dt=0.5) required between retunes. |
| `retuneMinDelta` | 0.02 | Ignore retunes smaller than 2% gain change (avoids churn). |
| `uRateLimit` | 0.2 /s | Max output slew. Heater/SSR lifetime or EMI → lower; aggressive profiles → raise or 0. |
| `integralLimit` † | 10 | |I-term| bound in u units. |
| `antiWindupTt` † | 0 → Ti | Back-calc tracking time; default = PI integral time is fine. |
| `maxTempC` / `overTempHysteresis` | 400 / 5 °C | **Set these for your process.** Hard interlock: output → uMin above the limit, resumes below limit−hysteresis. Residual overshoot is physical coast-down (no active cooling). |
| `feedbackBackend` | PI | `LQI` for non-collocated sensing (sensor ≠ controlled mass). See "Feedback backends". |
| `coupled.*` | defaults | LQI backend's two-state plant params. Start from `makeCoupledParams(cfg.model)`; refine coupling/loss-split from commissioning data (empty vs loaded step response). |
| `lqiQxSensor/QxProduct/Qi/R` | 0.01/0.004/0.02/1000 | LQI performance weights (not physical). Larger `lqiR` → softer/slower; smaller → more aggressive (watch for limit cycle). `qi` is scaled for the discrete `dt`-integrator — retune if `backendDt` changes a lot. |
| `observerQProc` / `observerRMeas` | 1e-4 / 0.09 | Observer bandwidth: larger QProc/RMeas ratio → trusts sensor more (faster state convergence, more noise feedthrough). `observerRMeas` ≈ sensor variance (σ=0.3 °C → 0.09). |
| `backendDt` | 0.5 s | Observer/LQI design step; set ≈ actual loop dt. |

## Multi-zone gradient control (`OvenGradientController`)

For ovens with several spatially ordered thermal zones (reflow /
conveyor ovens, multi-chamber systems), `OvenGradientController`
(`OvenGradientController.hpp`) supervises one `OvenController` per
zone and adds the gradient layer:

- **Gradient references** — `GradientSpec{base, tempOffsetC[],
  timeShiftS[], maxZoneDeltaC}` builds each zone's profile from one
  master `TemperatureProfile` via `transformed()`: zone *i* follows
  `base(t − shift_i) + offset_i`. Static offsets give a spatial
  gradient; time shifts give conveyor-style stagger. Independent
  per-zone profiles (`setZoneProfile`) and constant setpoints
  (`setZoneSetpoints`) are also supported.
- **Gradient constraint** — `maxZoneDeltaC` clamps adjacent-zone
  reference offsets (profile build) and setpoints (forward pass from
  zone 0), enforcing a maximum inter-zone temperature difference.
  `maxSetpointDelta()`/`maxMeasuredDelta()` report the live deltas;
  `setGradientAlarmLimit()` raises `gradientAlarm()` when the measured
  delta exceeds the threshold.
- **Coupling feedforward** — zones exchange heat through thermal
  resistances. `couplingLeft/couplingRight` [1/s] (= 1/(R·C_i) in lumped
  RC terms) inject the instantaneous coupling heat flow
  `γ·(T_n − T_i)` [°C/s] into each zone's feedforward via
  `OvenController::setCouplingDisturbance()`. The zone estimator still
  owns the residual disturbance, so steady-state behavior is exact and
  transients get immediate decoupling.

```cpp
OvenGradientController gc;
for (auto& zc : zoneConfigs) gc.addZone(zc);   // spatial order!

GradientSpec spec;
spec.base = masterProfile;
spec.tempOffsetC = {0, 9, 18};     // +9 °C per zone
spec.timeShiftS   = {0, 25, 50};   // conveyor stagger [s]
spec.maxZoneDeltaC = 15.0;
gc.setGradientProfile(spec);

auto u = gc.compute(measuredTemps, dt);   // u[i] per zone
// or the allocation-free computeInto(measured, dt, outSpan)
```

`zone(i)` exposes each `OvenController` for per-zone tuning/autotune;
`zoneStatus()` reports per-zone governed reference, output, coupling
injection, and adjacent deltas. An interactive demo runs the
controller on the `MultiZoneOven` benchmark plant:
`examples/oven_gradient_tui.cpp` (`oven_gradient_tui` — space run,
`1`/`2`/`3` switch profile presets, `c` cycles the master curve,
`r` reset).

## Industrial temperature curves (`ThermalCurve`)

`include/tether/control/thermal/ThermalCurve.hpp` provides the
standard process-curve layer on top of `TemperatureProfile`:

- **Preset library** — `presetProfile(ThermalCurvePreset)` returns
  ready-made `TemperatureProfile`s for `ReflowSac305`, `ReflowSnPb`,
  `WaveSolder`, `SelectiveSolder`, `Brazing`, and `Anneal`;
  `thermalCurvePresetName()` labels them. Each preset is a normal
  `TemperatureProfile`, so it feeds `OvenController`,
  `OvenGradientController::GradientSpec::base`, or any other consumer
  unchanged.
- **Process windows** — `specForPreset()` returns a
  `ThermalCurveSpec` describing the acceptance window: liquidus
  temperature (enables TAL + soak-band metrics), soak band
  `[soakMinC, soakMaxC]` and minimum soak time, peak range
  `[peakMinC, peakMaxC]`, TAL range `[talMinS, talMaxS]`, and maximum
  heating/cooling ramp rates. A field `<= 0` disables that check;
  `liquidusC == 0` disables TAL/soak-band analysis entirely.
- **Analysis** — `analyzeProfile(profile, spec)` computes exact
  piecewise-linear metrics from the profile's segment geometry (no
  sampling error): peak temperature, total duration, time above
  liquidus (TAL), soak-band residence time, and max heat/cool ramp
  rates. `analyzeSamples(times, temps, spec)` computes the same
  metrics from a measured trajectory using trapezoidal
  interpolation — use it for run qualification against the spec.
- **Verdict** — `ThermalCurveReport` carries the metrics, a
  `violations` list (human-readable strings), and `pass()`. Both
  analyzers share the same check logic, so a profile and a measured
  run are judged identically.

The TUI plots the master curve (past solid / future dotted, current
time marked) and shows the live spec verdict — `oven_gradient_tui`
presets `1`/`3` use the selected curve preset as the gradient base,
`c` cycles through all six.

```cpp
auto prof = presetProfile(ThermalCurvePreset::ReflowSac305);
auto spec = specForPreset(ThermalCurvePreset::ReflowSac305);
auto rep  = analyzeProfile(prof, spec);        // reference check
// ... run the oven, record measured (t, T) ...
auto meas = analyzeSamples(tHist, THist, spec); // run qualification
if (!meas.pass()) log(meas.violations.front());
```

## Real-time notes

`compute()` allocates nothing and takes no locks: fixed-size ring
buffers (input history ≤ `maxSamples`, estimator window bounded), a 3×3
LDLT solve per excited cycle, linear scans over short segment lists.
The per-cycle cost is O(1); estimation and retuning run inline but are
O(window samples):

- `FopdtIntegral` — a handful of accumulators + one small solve per
  cycle; cheap at any oven rate.
- `SopdtOe` — runs a grid + pattern-search OE fit every
  `sopdtEstimator.fitInterval` seconds (default 5 s), each evaluation
  re-simulating the cascade over the scored window at
  `dt·simStride`. Cost ≈ `gridSize² × window/dtFit` cascade steps per
  fit — keep `simStride`/`gridSize` moderate on slow loops, and note the
  scored window is `max(windowTime, windowLagFactor·(τ1+τ2))` so slow
  ovens fit less often but over more data. It is a bounded refinement,
  not a per-cycle burden.
Configure everything before the real-time thread starts; pass the
**actual** elapsed `dt` each cycle.
