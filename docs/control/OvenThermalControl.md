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

## Plant model

The controller assumes a lumped first-order-plus-dead-time (FOPDT) model:

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

## Control structure

```
TemperatureProfile ──► ActuationGovernor ──► r̂(t)   (achievable reference)
                              │                    │
                              │              model-inverse FF
                              │              u_ff = [ṙ̂(t+θ) + β(r̂−T_amb) − d̂]/α
                              │                    │
                        PI feedback ◄── T_filt      │
                        (SIMC gains)               ▼
                              u = clamp(u_ff + u_PI + dither, uMin, uMax)
                                        │
                        ThermalParameterEstimator ──► {α̂, β̂, d̂}
                        (bounded, excitation-gated)      │
                                          updates model + retunes PI
```

- **Feedforward** inverts the thermal model along the *governed*
  reference, previewed by `delay` seconds so power arrives when needed.
- **Governor** rate-limits the reference into the achievable band —
  on heat-only actuation a descent faster than passive cooling allows
  is pre-shaped to the passive rate instead of being left to feedback.
- **PI feedback** is tuned by SIMC rules *from the physical parameters*
  (`Kp = τ/(K·(τc+θ))`, `Ti = min(τ, 4(τc+θ))`), never hand-set. Its
  saturation limits are `[uMin−u_ff, uMax−u_ff]` so back-calculation
  anti-windup sees the true remaining actuator authority.
- **Estimator** runs a sliding-window integral regression
  (`ΔT/Tw = α·ū_d − β·mean(T−T_amb) + d`) every cycle: bounded
  projection onto physical ranges, excitation gating (skips steady
  soaks where parameters are unidentifiable), forgetting, EMA smoothing.
- **Dither**: a small sinusoidal perturbation on `u` while adaptation
  is enabled. In closed loop `u ≈ f(T)` otherwise, leaving `{α,β,d}`
  identifiable only up to a 1-D ambiguity; the dither decorrelates the
  regressors (~0.1–0.3 °C measurement ripple for typical parameters).

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
  reference. It is fed **deviation states** `z = x̂ − x_ss(r)` with
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

If no step test is possible, set `cfg.model` to best-estimate physical
parameters and leave `cfg.kp = cfg.ki = 0` — gains derive automatically.

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

Every control cycle (when `adaptParams`):

1. The estimator integrates I/O data over a trailing `windowTime`
   window and updates `{α̂, β̂, d̂}` — but only for windows showing real
   excitation (input or temperature variation above noise).
2. `α̂, β̂, d̂` are written into the feedforward model immediately, so
   load changes (heavier product → larger effective `C_th` → smaller
   `α`; door open → `d` excursion) are absorbed within seconds.
3. PI gains are **re-derived via SIMC** at most every
   `retuneInterval`, only after `retuneMinWindows` new excited updates
   and only if they changed by > `retuneMinDelta`. Gain changes are
   bumpless (the integrator stores an output-units contribution).

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

## Tuning knobs — when to change what

| Knob | Default | Change when… |
|---|---|---|
| `model.*` | required | After autotune, or manual physical estimates. Bounds `[uMin,uMax]` are hard safety limits — set before tuning. |
| `simcTauC` | 0 → τc=θ | Want slower/more robust response: raise (e.g. `2*delay` or `tau/10`). Faster: lower, but keep ≥ θ for delay robustness. |
| `kp/ki` | 0 → auto | Only to override auto-tuning entirely (then retune is effectively bypassed — set `adaptParams=false` to be explicit). |
| `sensorTau` | 3 s | Noisier sensor → raise (5–15 s). Clean sensor → lower (1–2 s). Does **not** affect estimation (estimator uses raw signal). |
| `estimator.windowTime` | 15 s | Faster adaptation → shorter (≥ ~3–5 s). Noisier data → longer. Should be ≪ profile feature times. |
| `estimator.memTime` | 300 s | How fast old data is forgotten. Slow load drift → longer; abrupt load changes → shorter. |
| `estimator.{alpha,beta,d}{Min,Max}` | wide | Tighten to the physically possible envelope for your oven — this is the real safety bound on adaptation. |
| `ditherAmp` | 0.02 | 0 disables (estimator will be biased in closed loop — only do this if `adaptParams=false`). Larger → better ID, more ripple. |
| `retuneInterval` | 30 s | Minimum seconds between gain retunes. |
| `retuneMinWindows` | 60 | Excited updates (~30 s at dt=0.5) required between retunes. |
| `retuneMinDelta` | 0.02 | Ignore retunes smaller than 2% gain change (avoids churn). |
| `uRateLimit` | 0.2 /s | Max output slew. Heater/SSR lifetime or EMI → lower; aggressive profiles → raise or 0. |
| `integralLimit` | 10 | |I-term| bound in u units. |
| `antiWindupTt` | 0 → Ti | Back-calc tracking time; default = PI integral time is fine. |
| `maxTempC` / `overTempHysteresis` | 400 / 5 °C | **Set these for your process.** Hard interlock: output → uMin above the limit, resumes below limit−hysteresis. Residual overshoot is physical coast-down (no active cooling). |
| `feedbackBackend` | PI | `LQI` for non-collocated sensing (sensor ≠ controlled mass). See "Feedback backends". |
| `coupled.*` | defaults | LQI backend's two-state plant params. Start from `makeCoupledParams(cfg.model)`; refine coupling/loss-split from commissioning data (empty vs loaded step response). |
| `lqiQxSensor/QxProduct/Qi/R` | 0.01/0.004/0.004/1000 | LQI performance weights (not physical). Larger `lqiR` → softer/slower; smaller → more aggressive (watch for limit cycle). |
| `observerQProc` / `observerRMeas` | 1e-4 / 0.09 | Observer bandwidth: larger QProc/RMeas ratio → trusts sensor more (faster state convergence, more noise feedthrough). `observerRMeas` ≈ sensor variance (σ=0.3 °C → 0.09). |
| `backendDt` | 0.5 s | Observer/LQI design step; set ≈ actual loop dt. |

## Model selection: FOPDT vs. alternatives

The estimator and feedforward always assume **FOPDT**
(`K·e^(−Ls)/(τs+1)`) for the input→sensor path. What changes with plant
topology is the *feedback backend*:

- **Collocated sensor** (sensor sees the controlled mass) → `PI` backend.
  Correct whenever a single effective lag + dead time describes the
  plant.
- **Non-collocated sensor** (air sensor, product/load is the controlled
  mass) → `LQI` backend on the two-state coupled model. This is the
  answer to the "two visible lags" case: instead of fitting SOPDT
  parameters, the second state *is* the unmeasured product temperature,
  reconstructed by the observer. It also gives integral action on the
  product-temperature error, rejecting load disturbances the SISO loop
  cannot see.
- **MIMO / multi-zone** → not yet implemented (LQI supports it in
  principle; the coupled model and estimator are SISO).

IPDT (integrating) is **never** correct for an oven — heat loss to
ambient gives a finite DC gain.

If tracking shows a persistent lag that grows with profile rate beyond
what feedback covers, the likely cause is unmodeled higher-order
dynamics or sensor placement — revisit the backend/model class before
raising gains.

## Real-time notes

`compute()` allocates nothing and takes no locks: fixed-size ring
buffers (input history ≤ 8192 samples, estimator window bounded by
`maxSamples`), a 3×3 LDLT solve per excited cycle, linear scans over
short segment lists. Retuning and estimation run inline but are
O(window samples) — cheap at oven sample rates (≥ 1 Hz typical).
Configure everything before the real-time thread starts; pass the
**actual** elapsed `dt` each cycle.
