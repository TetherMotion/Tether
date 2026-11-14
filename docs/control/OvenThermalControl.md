# Oven Thermal Control

`OvenController` (`include/tether/control/thermal/`) is a generic
profile-following controller for heated chambers with **one-sided
actuation** (heat-only; cooling is passive). Every user-facing parameter
is physical — thermal coefficients, dead time, disturbance — never a raw
PID gain.

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

## Model selection: FOPDT vs. alternatives

Only the **FOPDT** model is implemented. It is the right choice when a
single effective lag + dead time describes the plant — i.e. the sensor
sees roughly the mass being controlled and heater→sensor dynamics are
dominated by one time constant.

Consider a richer model (not yet implemented) when the step response
shows **two distinct lags** — e.g. a fast initial rise (heater element)
followed by a slow tail (chamber air → product), or when the sensor is
far upstream of the controlled mass. That is the SOPDT case
(`K·e^(−Ls)/((τ₁s+1)(τ₂s+2))`); the autotuning stack elsewhere in Tether
(`ProcessIdentification::identifySOPDT`) can fit it offline, but this
controller's estimator and feedforward assume first order. IPDT
(integrating) is **never** correct for an oven — heat loss to ambient
gives a finite DC gain.

If FOPDT tracking shows a persistent lag that grows with profile rate
beyond what PI trim covers, the likely cause is unmodeled second-order
dynamics or sensor placement — revisit the model class before raising
gains.

## Real-time notes

`compute()` allocates nothing and takes no locks: fixed-size ring
buffers (input history ≤ 8192 samples, estimator window bounded by
`maxSamples`), a 3×3 LDLT solve per excited cycle, linear scans over
short segment lists. Retuning and estimation run inline but are
O(window samples) — cheap at oven sample rates (≥ 1 Hz typical).
Configure everything before the real-time thread starts; pass the
**actual** elapsed `dt` each cycle.
