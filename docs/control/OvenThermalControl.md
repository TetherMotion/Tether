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
   (`qxSensor=0.01, qxProduct=0.004, qi=0.004, R=1000`) are verified
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
  PI on a FOPDT fit, no amount of `{α,β}` adaptation fixes the structure
  — switch to the LQI backend (its second state *is* the extra lag).
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
bounds), `windowsUsed` (stalls during soaks are *expected* — gating),
`saturated` (sustained true = actuator-limited, not a tuning problem),
`backendName`. Estimator bounds (`estimator.{alpha,beta,d}{Min,Max}`)
are the real safety envelope on adaptation — tighten them to the
physically possible range for your oven.

## Tuning knobs — when to change what

Knobs marked † are PI-backend only; knobs marked ‡ are LQI-backend only.
Everything else applies to both.

| Knob | Default | Change when… |
|---|---|---|
| `model.*` | required | After autotune, or manual physical estimates. Bounds `[uMin,uMax]` are hard safety limits — set before tuning. |
| `simcTauC` † | 0 → τc=θ | Want slower/more robust response: raise (e.g. `2*delay` or `tau/10`). Faster: lower, but keep ≥ θ for delay robustness. |
| `kp/ki` † | 0 → auto | Only to override auto-tuning entirely (then retune is effectively bypassed — set `adaptParams=false` to be explicit). |
| `sensorTau` | 3 s | Noisier sensor → raise (5–15 s). Clean sensor → lower (1–2 s). Does **not** affect estimation (estimator uses raw signal). |
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
- **MIMO / multi-zone** → `OvenGradientController` supervises one
  `OvenController` per zone with gradient references, inter-zone delta
  constraints, and coupling feedforward (see "Multi-zone gradient
  control"). A true MIMO LQI (joint Riccati over all zones) is not
  implemented — per-zone SISO loops + decoupling covers the common
  reflow/conveyor cases.

IPDT (integrating) is **never** correct for an oven — heat loss to
ambient gives a finite DC gain.

If tracking shows a persistent lag that grows with profile rate beyond
what feedback covers, the likely cause is unmodeled higher-order
dynamics or sensor placement — revisit the backend/model class before
raising gains.

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
buffers (input history ≤ 8192 samples, estimator window bounded by
`maxSamples`), a 3×3 LDLT solve per excited cycle, linear scans over
short segment lists. Retuning and estimation run inline but are
O(window samples) — cheap at oven sample rates (≥ 1 Hz typical).
Configure everything before the real-time thread starts; pass the
**actual** elapsed `dt` each cycle.
