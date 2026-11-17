# Servo-Axis LQI + Disturbance Observer Backend

`ServoAxisLQIBackend` (`include/tether/control/motion/`) is a
position-servo feedback backend for a **torque-mode axis** — the inner
current/torque loop is closed by the drive, and this backend issues the
torque command. It is the motion analogue of `OvenLQIBackend`: every
tuning parameter is **physical** — inertia, viscous/Coulomb friction,
torque constant, dead time, sensor variance — never a raw PID gain.

## Plant model

```
J·θ̈ = Kt·u_d − b·θ̇ − τ_c·sgn(θ̇) − τ_d
```

| Param | Meaning | Units (rotary) |
|---|---|---|
| `inertia` | Reflected inertia J (`linear=true` → mass) | kg·m² |
| `damping` | Viscous friction b | N·m·s |
| `torqueConstant` | Torque per command unit Kt | N·m/u |
| `coulombFriction` | Coulomb level τ_c (feedforward only) | N·m |
| `delay` | Input dead time θ_d | s |
| `uMin`/`uMax` | Torque command limits | u |

## Structure

```
encoder y = θ + ν ──► [θ, ω, τ_d] steady-state Kalman observer
                          ▲                     │
       total applied u ───┘ (delay line)         ▼
                     u = u_ff + u_dob + u_fb
```

- **Observer** — fixed-gain steady-state Kalman over `[θ, ω, τ_d]` with
  `τ_d` a random-walk lumped disturbance (load torque, cable drag,
  friction misfit). Predicts with the *total applied* command
  (`notifyAppliedInput`, or its own output standalone) so feedforward is
  not mistaken for process noise. Noise knobs are physical: `rMeas` =
  encoder quantization (`res²/12`), `qAccel` = unmodelled acceleration
  density, `qDist` = disturbance random-walk intensity.
- **Feedback** — `LQIController` on deviation states `z = [θ̂−r, ω̂−ω_ref]`
  plus an integrator on position error → zero steady-state offset under
  constant load.
- **Physics feedforward** — `u_ff = (J·α_ref + b·ω_ref)/Kt +
  τ_c·tanh(ω_ref/vε)/Kt` from the *current* physical parameters, plus
  `ControllerInput::feedforward` for external terms (e.g. gravity).
- **Disturbance feedforward** — `u_dob = +τ̂_d/Kt` through a first-order
  Q-filter (`setDisturbanceBandwidth`, default 15 rad/s). Cancels load
  torque without waiting for the integrator. The Q-filter is essential:
  a raw estimate injects observer convergence transients into the
  actuator.
- **Anti-windup** — three selectable mechanisms via
  `setAntiWindup(ServoAxisAWMode)`:
  - `kConditional` (**default**) — conditional integration: the
    integrator freezes while the loop is saturated in the error's
    direction. Simplest robust scheme, no extra knobs.
  - `kBackCalculation` — tracking anti-windup: the integrator is pulled
    toward the value consistent with the saturated output, with time
    constant `Tt` (`setAntiWindupTrackingTime`, default 4 ms — rule of
    thumb: `Tt ≈ 1/bandwidth`). The classic scheme in industrial servo
    drives; implemented natively in `LQIController` so the correction is
    dimensionally consistent (`xi -= (u_sat−u_desat)·dt/(Ki·Tt)`).
  - `kNone` — only the authority-bounded integral state (below).
    Diagnostic baseline: under deep saturation the integrator parks at
    its bound and the loop settles at a windup equilibrium with a
    permanent offset.
  - Always on: the integral state is bounded so `|Ki·xi| ≤ ~1.2·uMax`
    — prevents *unbounded* windup, orthogonal to the mode.
- **Remaining-authority clamp** — feedback is clamped to
  `[uMin−u_ff, uMax−u_ff]` before summing, so the total command never
  exceeds the torque limits.
- **Output slew limit (optional)** — `setOutputRateLimit(u/s)` bounds
  the total command's rate of change. An actuator-protection layer, not
  a shaping tool: a limit tighter than the closed loop's demand acts as
  saturation (it does flag for anti-windup) and adds lag that can
  destabilize a fast loop — keep it comfortably above the loop's
  typical command rates.

## Tuning

Two honest ways to set the LQI weights:

```cpp
// (a) Single physical-ish knob: desired bandwidth + damping.
//     Maps the rigid-plant LQR equivalence (k1=ω_n², k2=2ζω_n) onto
//     weights, normalized by (J/Kt)² — exact for a pure double
//     integrator, approximate with damping/delay.
auto t = ServoAxisTuning::fromBandwidth(plant, 60.0 /*rad/s*/, 1.0 /*ζ*/);

// (b) Explicit Q/R with physical units: qPos [1/rad²], qVel [s²/rad²],
//     qInt [1/(rad·s)²], r [1/u²].
ServoAxisTuning t2; t2.qPos = 1e4; t2.qInt = 40; t2.r = 1;
```

## Online retuning (LiveTune seam)

`IServoAxisAdaptiveFeedback::updateAxisParameters(J, b, Kt, τ_c)` is the
push interface — designed to consume `SystemIdentifier` output
(`FrictionIdentificationResult::viscousCoeff` → `damping`,
`coulombForce` → `coulombFriction`, plus an inertia estimate). Invalid
pushes (J ≤ 0, Kt = 0, b < 0) are rejected; a valid push re-solves the
observer Riccati and the LQI DARE — off the hard real-time path, same
supervisory cadence as the oven's retune.

## Usage

```cpp
ServoAxisParams p;
p.inertia = 2e-4;  p.damping = 5e-4;  p.torqueConstant = 0.05;
p.uMin = -2; p.uMax = 2;
ServoAxisLQIBackend axis(p, ServoAxisTuning::fromBandwidth(p, 60, 1), 1e-3);

// per servo cycle:
ControllerInput in;
in.reference           = posRef;
in.referenceDerivative = velRef;   // → physics FF
in.referenceAccel      = accRef;   // → physics FF
in.measured            = encoderPos;
in.dt                  = 1e-3;
auto out = axis.compute(in);        // out.control = torque command
drive.setTorque(out.control);
axis.notifyAppliedInput(out.control); // post-clamp, if the drive clamps
```

## Honest scope

- The DOB cannot shrink the *instantaneous* dip below what the feedback
  already arrests — the excursion happens inside one observer
  convergence time. Its payoff is recovery speed, integrated error and
  integrator offload.
- The input dead time is honored in the *observer's* input history;
  the LQI synthesis treats the plant as delay-free (same convention as
  the oven backend). Keep bandwidth ≪ 1/θ_d.
- Friction compensation is Coulomb+viscous only; Stribeck/LuGre fits
  from `SystemIdentifier` map onto `coulombFriction`/`damping` —
  residual misfit is absorbed by τ̂_d and the integrator.

Tests: `tests/control/test_servo_axis_lqi_backend.cpp` (11 closed-loop
scenarios against an in-test rigid-body plant).
