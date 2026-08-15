# Flow-Adaptive Temperature Control

## Overview

When filament begins flowing through the hotend, it absorbs heat from the melt
zone.  The heater block's thermal mass buffers the initial transient, but the
melt-zone temperature can drop several degrees before the PID controller catches
up.  This causes under-extrusion at the start of a print move and over-extrusion
when flow stops.

Tether's **flow-adaptive heater controller** addresses this by:

1. **Pre-emphasis**: Adding a feed-forward power boost when flow starts, to
   pre-heat the melt zone before the enthalpy of the incoming filament cools it.
2. **Post-emphasis**: Reducing power when flow stops, to bleed off the excess
   thermal energy that accumulated in the melt zone.
3. **Closed-loop PID**: A PID controller drives the heater block temperature to
   the target, with the pre/post-emphasis added as feed-forward.

A two-state thermal observer (`MeltZoneThermalObserver`) estimates the melt-zone
temperature from the heater block temperature and the current flow rate,
providing a virtual sensor for the controller.

## Configuration

### `[extruder]` config keys

| Key                        | Type   | Default | Description                          |
|----------------------------|--------|---------|--------------------------------------|
| `heater_flow_pre_emphasis` | bool   | `false` | Enable flow-adaptive heater control  |
| `filament_heat_capacity`   | float  | `2.1`   | ρ·c_p [J/(mm³·K)]                    |
| `melt_zone_capacitance`    | float  | `2.0`   | C_m [J/K]                            |
| `heater_melt_conductance`  | float  | `0.8`   | G_hm [W/K]                           |
| `debt_time_constant`       | float  | `2.0`   | τ [s] — thermal debt decay           |
| `max_pre_emphasis_power`   | float  | `0.4`   | Max pre-emphasis PWM boost [0–1]     |
| `max_post_emphasis_power`  | float  | `0.2`   | Max post-emphasis PWM cut [0–1]      |
| `max_heater_overshoot`     | float  | `10.0`  | Max allowed overshoot above target [°C] |

### Example

```ini
[extruder]
heater_flow_pre_emphasis: true
filament_heat_capacity: 2.1
melt_zone_capacitance: 2.0
heater_melt_conductance: 0.8
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
    [MELT_ZONE_CAPACITANCE=<J/K>]
    [HEATER_MELT_CONDUCTANCE=<W/K>]
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

### `MeltZoneThermalObserver`

A two-state lumped-capacitance thermal model:

```
C_h · dT_h/dt = P_heater - G_hm·(T_h - T_m)
C_m · dT_m/dt = G_hm·(T_h - T_m) - ρ·c_p·Q·(T_m - T_in)
```

States:
- `T_h` — heater block temperature [°C] (driven by heater power)
- `T_m` — melt-zone temperature estimate [°C]

Inputs:
- `P_heater` — heater power [W] (PWM × power scale)
- `Q` — volumetric flow [mm³/s] (from `ExtrusionFlowTracker`)
- `dt` — time step [s]

### `FlowAdaptiveHeaterController`

Wraps a PID controller and adds feed-forward:

```
PWM_total = clamp(PWM_pid + PWM_pre_emphasis - PWM_post_emphasis, 0, 1)
```

- **Pre-emphasis**: When flow increases, a power boost proportional to the
  enthalpy flux (`ρ·c_p·Q·ΔT`) is applied, decaying with time constant τ.
- **Post-emphasis**: When flow stops, the accumulated "thermal debt" is bled
  off by reducing power, preventing overshoot.

The controller inherits PID gains from the extruder heater's existing PID
configuration.

### `ExtrusionFlowTracker`

A shared, thread-safe (atomic-based) flow tap between the motion dispatcher and
the heater compensation.  The dispatcher updates it on every move with the
per-move E-axis velocity; the heater reads the smoothed (EWMA) flow.

## Integration with `Heater::control()`

When `setFlowCompensation()` is called on a `Heater` object, its `control()`
method delegates to the `FlowAdaptiveHeaterController` instead of running the
raw PID.  The flow tracker's smoothed flow is fed automatically:

```cpp
// Wired by KlippyInstance::applyFlowAdaptiveHeaterSettings()
heater->setFlowCompensation(flowAdaptiveHeater, extrusionFlowTracker);
```

If no compensation is wired, `Heater::control()` runs the classic PID path
unchanged.

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/MeltZoneThermalObserver.hpp` | Two-state thermal observer |
| `include/tether/control/extrusion/FlowAdaptiveHeaterController.hpp` | Flow-adaptive heater controller |
| `include/tether/klipper/motion/ExtrusionFlowTracker.hpp` | Shared flow tap |
| `include/tether/klipper/objects/Thermal.hpp` | Heater hook |
| `tests/control/test_flow_adaptive_heater.cpp` | Unit tests |
| `tests/klipper/test_klipper_extrusion_compensation.cpp` | Integration tests |
