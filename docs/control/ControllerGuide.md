# Controller Guide

How to choose, configure and compose controllers from `tether/control`.
This guide is task-oriented: it answers *"which controller should I use,
how do I wire it up, and what will bite me in production?"*

The API reference for each class lives in the Doxygen pages; this page
explains the decisions around them.

---

## 1. Choosing a controller

All controllers implement the same `ControllerBase` interface, so you can
swap them without touching the call site.

| Situation | Start with | Why |
|---|---|---|
| Speed loop on a servo drive | `PIDController` | Familiar, tunable, well-behaved with anti-windup |
| Position loop driving a velocity loop | `CascadeController` + two PIDs | Separates stiffness (outer) from damping (inner) |
| Constant-velocity tracking with known dynamics | `FeedforwardController` | Removes the lag the feedback loop would otherwise fight |
| Setpoint steps cause overshoot | `PIDController` with derivative-on-measurement | No derivative kick on reference changes |
| Actuator has hard on/off behaviour | `BangBangController` | Hysteresis-based switching |
| Plant is unstable or highly nonlinear | `SlidingModeController` family | Robust to matched disturbances |
| You have a state-space model | `LQRController` / `LQGController` / `LQIController` | Optimal state feedback, optional observer + integral action |
| Multiple loops must act on one output | `ParallelController` | Sums several controller outputs |
| Repetitive task (same path every cycle) | `RepetitiveController` / ILC family | Learns the repeating error |

> **Rule of thumb:** start with PID. Move to state-space or robust control
> only when you can point at a concrete failure mode (overshoot you cannot
> tune out, disturbance rejection gap, model-based feedforward need).

---

## 2. The call contract

Every controller consumes a `ControllerInput` and produces a
`ControllerOutput`:

```cpp
struct ControllerInput {
    double reference{0.0};   // setpoint
    double measured{0.0};    // process variable
    double dt{0.001};        // sample period [s]

    // optional, controller-specific
    double referenceDerivative{0.0};  // 2DOF feedforward
    double referenceAccel{0.0};
    double feedforward{0.0};          // additive FF term
    double disturbance{0.0};

    StateVector state{};     // state-space controllers
    size_t stateDim{0};

    bool reset{false};       // clear integrators/filters this call
    bool enable{true};
};
```

`ControllerOutput::control` is the signal you send to the actuator. The
remaining fields (`proportional`, `integral`, `derivative`,
`feedforward`, `saturated`, `error`) are diagnostics — log them, they make
tuning dramatically faster.

### Minimal loop

```cpp
#include "tether/control/PIDControllers.hpp"

using namespace tether::control;

PIDController pid;
pid.setGains(/*kp=*/1.2, /*ki=*/0.8, /*kd=*/0.05);
pid.setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
pid.setDerivativeOnMeasurement(true);   // no derivative kick

ControllerInput in;
in.dt = 0.001;                          // 1 kHz
for (;;) {
    in.reference = target;
    in.measured  = encoder.read();
    const ControllerOutput out = pid.compute(in);
    drive.setVelocity(out.control);
}
```

**`dt` must be the real elapsed time**, not the nominal period. If your
loop jitters, measure the interval; an integrator fed a wrong `dt` drifts
silently.

---

## 3. Gain forms

`PIDController` accepts gains in two forms. Pick one and stay consistent —
mixing them is the most common source of "why did the loop change
character when I only retuned Ki?".

| Form | Setter | Meaning |
|---|---|---|
| Parallel | `setGains(kp, ki, kd)` | `u = kp·e + ki·∫e + kd·de/dt` |
| ISA / standard | `setGainsFromTimeConstants(kp, ti, td)` | `ki = kp/ti`, `kd = kp·td` |

Time constants are easier to reason about physically: `ti` is roughly the
time the integrator needs to repeat the proportional action, `td` the
derivative look-ahead. Most vendor drive manuals use this form.

---

## 4. Anti-windup, saturation and rate limiting

Integrators wind up whenever the actuator saturates: the error stays
positive, the integral keeps growing, and the loop overshoots badly on the
way back.

```cpp
SaturationLimits lim;
lim.outputMin = -10.0;      // [V] or [A] — actuator range
lim.outputMax =  10.0;
lim.rateLimit = 500.0;      // max change per second
pid.setSaturationLimits(lim);
```

| `AntiWindupMethod` | Behaviour | Use when |
|---|---|---|
| `Clamping` | Stops integrating at the limit | Simple, no tuning parameter |
| `BackCalculation` | Feeds `(u_sat - u)` back through `1/Tt` | Best general default; `Tt ≈ sqrt(ti·td)` |
| `Conditional` | Integrates only when it reduces the error | Cheap, slightly less smooth |
| `Tracking` | External reset signal | You have a supervisory controller |
| `ObserverBased` | Model-based | State-space designs |

`rateLimit` is enforced by `ControllerBase::rateLimit()` and reported via
`ControllerOutput::saturated` — check that flag when debugging overshoot.

---

## 5. Modes

```cpp
pid.setEnabled(false);            // → ControllerMode::Disabled
pid.setMode(ControllerMode::Manual);
pid.setManualOutput(0.5);         // fixed output while in Manual
```

Use `Manual` for commissioning (bumpless transfer: set the manual output to
the current feedback value *before* switching to Automatic) and `Disabled`
for safe-state handling. `isEnabled()` is false only in `Disabled`.

---

## 6. Composition

### 6.1 Cascade (position → velocity)

```cpp
PIDController posLoop, velLoop;
CascadeController cascade;
cascade.setOuterController(&posLoop);
cascade.setInnerController(&velLoop);
cascade.setInnerReferenceLimits(-1000.0, 1000.0);  // limit velocity command
```

- Tune **inner first** (velocity loop on the drive, or the velocity PID),
  then close the outer loop.
- The inner loop must be at least ~5× faster than the outer loop.
- `setInnerReferenceLimits` is what protects the mechanics when the outer
  loop demands impossible velocities.

### 6.2 Parallel (sum of controllers)

`ParallelController` adds the outputs of several controllers — useful for
e.g. a PID plus a disturbance observer or a gravity-compensation term.

### 6.3 Feedforward

`FeedforwardController` passes a model-based term through with an optional
feedback controller as backend. Feedforward cancels *predictable* error;
feedback handles the rest. If you are fighting lag on a known trajectory,
add feedforward before increasing `kp`.

### 6.4 Backend chaining

Any controller can carry another as a backend:

```cpp
pid.setBackendController(other);
```

`compute()` on the outer controller calls the backend as part of its law.
This is how cascade, feedforward and dual-loop structures are assembled
internally — you rarely need to build your own chain, but the hook exists
for custom laws.

---

## 7. Diagnostics

`getDiagnostics()` returns counters maintained by `compute()`; `getLastOutput()`
returns the full last `ControllerOutput`. Log the per-term breakdown during
commissioning:

```
t=1.234  ref=100.0 meas=98.2  err=1.8  P=1.44 I=0.31 D=0.02  sat=0
```

If `I` keeps growing while `sat=1`, your anti-windup is not working. If `D`
spikes when the setpoint steps, enable derivative-on-measurement.

---

## 8. Autotuning

See [Autotuning Guide](AutotuningGuide.md) for the full workflow. The
short version:

```cpp
// From an identified first-order-plus-dead-time model:
pid.autoTune(/*K=*/2.0, /*tau=*/0.15, /*theta=*/0.01,
             PIDController::TuningMethod::AMIGO);

// Or from a Ku/Tu experiment:
pid.autoTuneFromUltimate(/*Ku=*/45.0, /*Tu=*/0.08,
                         PIDController::TuningMethod::ZieglerNicholsPID);
```

Never ship the Ziegler-Nichols result untuned — it is a *starting point*
chosen for aggressive response, not a final answer.

---

## 9. Real-time considerations

- **No allocation in `compute()`.** All controllers are designed for
  steady-state allocation-free operation; do not add containers to
  `ControllerInput`/`ControllerOutput` copies in the hot path.
- **Fixed `dt` or measured `dt`** — decide once, document it, and use the
  same convention everywhere (filters included).
- **Construct before the RT thread starts.** `setGains`, `setAntiWindup`,
  `setSaturationLimits` are not synchronised; configure, then start.
- **Determinism:** `compute()` has no locks and no syscalls; any logging
  inside it will destroy your jitter budget — log from the diagnostics
  fields instead.

---

## 10. Commissioning checklist

1. Verify the sign of the feedback before closing the loop (open-loop step
   test).
2. Set `SaturationLimits` to the real actuator range *first*.
3. Choose an anti-windup method (`BackCalculation` unless you have a reason).
4. Enable derivative-on-measurement if the setpoint is stepped.
5. Tune P until the response is fast but not ringing; add I for offset;
   add D only if needed.
6. Add feedforward for known trajectories.
7. Record `Ku`, `Tu` or a FOPDT model, then autotune for a reproducible
   baseline.
8. Log per-term diagnostics for one full cycle and archive it with the
   machine configuration.
