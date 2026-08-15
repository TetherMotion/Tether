# Flow-Adaptive Temperature Control

## Overview

When filament begins flowing through the hotend, it absorbs heat from the melt
zone. The heater block's thermal mass buffers the initial transient, but the
melt-zone temperature can drop several degrees before the PID controller catches
up. This causes under-extrusion at the start of a print move and over-extrusion
when flow stops.

The key physical insight is that the **thermistor is not at the melt zone** —
it is physically between the heater block and the melt zone, with its own
thermal mass and coupling resistances. This means:

1. **T_m drops first** (enthalpy drain is at the melt zone)
2. **T_s drops second** (sensor is coupled to melt zone through the barrel, with some thermal resistance and delay)
3. **T_h drops last** (heater block has the largest thermal mass, and the PID reacts to T_s, not T_m)

The PID reacts to T_s, so it *partially* compensates on its own — but with a
lag and a gain error, because the sensor doesn't see the full magnitude of the
melt-zone drop. After flow stops, the sensor overshoots because the heater
block is still hot while the melt zone has recovered — the PID sees a falsely
high temperature and reduces power prematurely.

Tether's **flow-adaptive heater controller** addresses this with a three-state
thermal model and model-based feed-forward:

1. **Three-state thermal observer** (`MeltZoneThermalObserver`): Models the
   heater block (T_h), sensor point (T_s), and melt zone (T_m) as three coupled
   thermal masses. A Luenberger observer uses the actual thermistor reading to
   correct the state estimate, compensating for unmodelled losses (radiation,
   convection, fan cooling).
2. **Pre-emphasis**: When flow starts, a feed-forward power boost is applied
   *before* the melt zone cools. The boost is scaled by (1 - α), where α is the
   sensor coupling factor — the fraction of the disturbance the PID will see
   and react to on its own. This avoids double-compensating.
3. **Post-emphasis**: When flow stops, a decaying power offset compensates the
   thermal debt left in the melt zone, counteracting the PID's premature power
   reduction caused by the sensor's thermal lag. Also scaled by (1 - α).
4. **Closed-loop PID**: A PID controller drives the sensor temperature to the
   target, with the pre/post-emphasis added as feed-forward.

## Configuration

### `[extruder]` config keys

#### Enable and filament properties

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `heater_flow_pre_emphasis` | bool   | `false` | Enable flow-adaptive heater control  |
| `filament_heat_capacity`   | float  | `2.1`   | ρ·c_p [J/(mm³·K)]                    |

#### Three-state thermal model — capacitances

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `heater_block_capacitance` | float  | `8.0`   | C_h [J/K] — heater block mass        |
| `sensor_capacitance`       | float  | `1.0`   | C_s [J/K] — thermistor + surrounding |
| `melt_zone_capacitance`    | float  | `2.0`   | C_m [J/K] — melt zone thermal mass   |

#### Three-state thermal model — conductances

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `heater_sensor_conductance`| float  | `2.0`   | G_hs [W/K] — heater block → sensor   |
| `sensor_melt_conductance`  | float  | `1.5`   | G_sm [W/K] — sensor → melt zone      |

#### Luenberger observer gains

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `luenberger_gain_heater`   | float  | `0.5`   | L_h [1/s] — correction to T_h        |
| `luenberger_gain_sensor`   | float  | `2.0`   | L_s [1/s] — correction to T_s        |
| `luenberger_gain_melt`     | float  | `0.3`   | L_m [1/s] — correction to T_m        |

#### Feed-forward limits

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `debt_time_constant`       | float  | `2.0`   | τ [s] — thermal debt decay           |
| `max_pre_emphasis_power`   | float  | `0.4`   | Max pre-emphasis PWM boost [0–1]     |
| `max_post_emphasis_power`  | float  | `0.2`   | Max post-emphasis PWM cut [0–1]      |
| `max_heater_overshoot`     | float  | `10.0`  | Max allowed overshoot above target [°C] |

### Example

```ini
[extruder]
heater_flow_pre_emphasis: true
filament_heat_capacity: 2.1
heater_block_capacitance: 8.0
sensor_capacitance: 1.0
melt_zone_capacitance: 2.0
heater_sensor_conductance: 2.0
sensor_melt_conductance: 1.5
luenberger_gain_heater: 0.5
luenberger_gain_sensor: 2.0
luenberger_gain_melt: 0.3
debt_time_constant: 1.5
max_pre_emphasis_power: 0.35
max_post_emphasis_power: 0.15
max_heater_overshoot: 8.0
```

## G-code commands

### `SET_HEATER_FLOW_COMPENSATION`

```
SET_HEATER_FLOW_COMPENSATION [ENABLE=0|1]
    [FILAMENT_HEAT_CAPACITY=<J/(mm³·K)>]
    [HEATER_BLOCK_CAPACITANCE=<J/K>]
    [SENSOR_CAPACITANCE=<J/K>]
    [MELT_ZONE_CAPACITANCE=<J/K>]
    [HEATER_SENSOR_CONDUCTANCE=<W/K>]
    [SENSOR_MELT_CONDUCTANCE=<W/K>]
    [LUENBERGER_GAIN_HEATER=<1/s>]
    [LUENBERGER_GAIN_SENSOR=<1/s>]
    [LUENBERGER_GAIN_MELT=<1/s>]
    [DEBT_TIME_CONSTANT=<s>]
    [MAX_PRE_EMPHASIS_POWER=<0-1>]
    [MAX_POST_EMPHASIS_POWER=<0-1>]
    [MAX_HEATER_OVERSHOOT=<°C>]
```

## Status objects

The `extruder` printer object now exposes flow-compensation diagnostics:

| Field                  | Type  | Description                              |
|------------------------|-------|------------------------------------------|
| `melt_temp_estimate`   | float | Estimated melt-zone temperature [°C]     |
| `pre_emphasis_power`   | float | Last pre-emphasis PWM contribution [0–1] |
| `post_emphasis_power`  | float | Last post-emphasis PWM contribution [0–1]|

## Architecture

### `MeltZoneThermalObserver` (three-state)

A three-state lumped-capacitance thermal model with Luenberger correction:

```
C_h · dT_h/dt = P_heater - G_hs·(T_h - T_s)
C_s · dT_s/dt = G_hs·(T_h - T_s) - G_sm·(T_s - T_m)
C_m · dT_m/dt = G_sm·(T_s - T_m) - ρ·c_p·Q·(T_m - T_in)
```

States:
- `T_h` — heater block temperature [°C] (directly heated by cartridge)
- `T_s` — sensor point temperature [°C] (where the thermistor sits)
- `T_m` — melt-zone temperature estimate [°C] (where filament melts)

Physical layout:

```
Heater cartridge
    ↓ (G_hs)
Heater block (T_h)  — largest thermal mass
    ↓ (G_hs)
Sensor point (T_s)  — thermistor location
    ↓ (G_sm)
Melt zone (T_m)     — cold plastic enters here
```

Inputs:
- `P_heater` — heater power [W] (PWM × power scale)
- `Q` — volumetric flow [mm³/s] (from `ExtrusionFlowTracker`)
- `T_s_measured` — actual thermistor reading [°C] (for Luenberger correction)
- `dt` — time step [s]

#### Luenberger correction

The observer uses the real sensor measurement to correct all three state
estimates:

```
innovation = T_s_measured - T_s_estimated
T_h += L_h · innovation · dt
T_s += L_s · innovation · dt
T_m += L_m · innovation · dt
```

This is critical because:
- Without correction, the open-loop model drifts due to unmodelled losses.
- The sensor carries information about the melt-zone state that the heater-block
  temperature alone cannot provide.
- The gains determine how aggressively the observer trusts the measurement vs.
  the model. L_s is largest (direct measurement), L_h and L_m are moderate
  (indirect).

### `FlowAdaptiveHeaterController`

Wraps a PID controller and adds model-based feed-forward:

```
PWM_total = clamp(PWM_pid + PWM_pre + PWM_post, 0, 1)
```

#### Sensor coupling factor α

The key innovation is the **sensor coupling factor**:

```
α = G_sm / (G_hs + G_sm)
```

This represents the fraction of the melt-zone thermal disturbance that the PID
(which reacts to T_s) will compensate on its own. The feed-forward only needs
to cover the remaining (1 - α) fraction, avoiding double-compensation.

- **High α** (G_sm >> G_hs): Sensor is tightly coupled to melt zone → PID sees
  most of the disturbance → small feed-forward needed.
- **Low α** (G_hs >> G_sm): Sensor is tightly coupled to heater block → PID
  barely sees the melt-zone disturbance → large feed-forward needed.

#### Pre-emphasis

At flow onset, the feed-forward covers the uncompensated enthalpy power:

```
P_pre = (1 - α) · ρ·c_p·Q·(T_target - T_inlet)
```

Plus a gradient boost to establish the thermal gradient across the heater→melt
path ahead of the thermal lag. Both are bounded by `maxPreEmphasisPower` and
`maxHeaterOvershoot`.

Pre-emphasis is suppressed when the measured temperature is far from target
(during warmup) to avoid fighting the PID.

#### Post-emphasis

After flow stops, the melt zone is thermally depleted but the sensor is still
lagging. A "thermal debt" D relaxes toward zero with time constant τ:

```
Ḋ = (D_target(Q) - D) / τ
P_post = (1 - α) · max(-D, 0) / P_scale
```

When flow stops, D_target → 0 but D is still positive, so a decaying positive
power is added to compensate the dip. Bounded by `maxPostEmphasisPower`.

The controller inherits PID gains from the extruder heater's existing PID
configuration.

### `ExtrusionFlowTracker`

A shared, thread-safe (atomic-based) flow tap between the motion dispatcher and
the heater compensation. The dispatcher updates it on every move with the
per-move E-axis velocity; the heater reads the smoothed (EWMA) flow.

## Integration with `Heater::control()`

When `setFlowCompensation()` is called on a `Heater` object, its `control()`
method delegates to the `FlowAdaptiveHeaterController` instead of running the
raw PID. The flow tracker's smoothed flow is fed automatically, and the
measured temperature is used for both PID feedback and Luenberger correction:

```cpp
// Wired by KlippyInstance::applyFlowAdaptiveHeaterSettings()
heater->setFlowCompensation(flowAdaptiveHeater, extrusionFlowTracker);
```

If no compensation is wired, `Heater::control()` runs the classic PID path
unchanged.

## Tuning guide

### Thermal model parameters

The three-state model parameters (C_h, C_s, C_m, G_hs, G_sm) should be
identified from step-response data. A simple procedure:

1. Heat the hotend to a stable temperature with no flow.
2. Apply a step in heater power and record the thermistor response.
3. Fit the three capacitances and two conductances to match the response.
4. Alternatively, use physical estimates:
   - C_h ≈ mass_heater_block × c_aluminum ≈ 8 J/K for a typical hotend
   - C_s ≈ mass_thermistor_mount × c_steel ≈ 1 J/K
   - C_m ≈ mass_melt_zone × c_polymer ≈ 2 J/K
   - G_hs, G_sm: estimate from the thermal resistance of the barrel/throat

### Luenberger gains

Start with the defaults (L_h=0.5, L_s=2.0, L_m=0.3). If the observer diverges
or oscillates:

- **Oscillation**: Reduce all gains by 2×.
- **Slow convergence**: Increase L_s (the direct measurement gain).
- **T_m estimate drifts**: Increase L_m, but watch for oscillation.

### Feed-forward limits

| Parameter              | Too small | Too large |
|------------------------|-----------|-----------|
| `max_pre_emphasis_power` | Under-extrusion at flow onset | Overshoot, heater stress |
| `max_post_emphasis_power` | Temperature dip after flow stops | Over-shoot after stop |
| `max_heater_overshoot` | Slow gradient establishment | Risk of burning filament |
| `debt_time_constant` | Post-emphasis decays too fast | Post-emphasis lingers too long |

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/MeltZoneThermalObserver.hpp` | Three-state thermal observer with Luenberger correction |
| `include/tether/control/extrusion/FlowAdaptiveHeaterController.hpp` | Model-based flow-adaptive heater controller |
| `include/tether/klipper/motion/ExtrusionFlowTracker.hpp` | Shared flow tap |
| `include/tether/klipper/objects/Thermal.hpp` | Heater hook |
| `tests/control/test_flow_adaptive_heater.cpp` | Unit tests (15 tests) |
| `tests/klipper/test_klipper_extrusion_compensation.cpp` | Integration tests |
