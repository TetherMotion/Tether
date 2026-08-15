# Non-Newtonian Pressure Advance

## Overview

Classic Klipper pressure advance (PA) assumes a **linear** (Newtonian)
relationship between extrusion velocity and the compensating position offset:

$$ \delta e = \text{PA} \cdot v_e $$

This works well for Newtonian fluids but underestimates the pressure build-up
in shear-thinning polymer melts (PLA, ABS, PETG, …), where the apparent
viscosity drops with increasing shear rate.  Tether extends the PA system with
two non-Newtonian models that can be selected at runtime via config or G-code:

| Model     | Formula                                              | When to use                         |
|-----------|------------------------------------------------------|-------------------------------------|
| `linear`  | $\delta e = \text{PA} \cdot v_e$                    | Newtonian fluids, classic Klipper PA |
| `power_law` | $\delta e = K_{\text{base}} \cdot (v_e \cdot A_f)^n$ | Shear-thinning melts with known flow index |
| `cross_wlf` | $\delta e = \frac{\beta V_m}{A_f} \cdot P_{\text{LUT}}(Q, T)$ | Full temperature-dependent Cross-WLF rheology |

All three models share the same **position-offset** implementation in
`MotionTranslator`, so the step-generation pipeline is unchanged.

## Compile-time gate

The entire feature is gated behind `TETHER_ENABLE_PRESSURE_ADVANCE` (default
`ON`).  When compiled out, none of the code in this document is included.

## Configuration

### `[extruder]` config keys

| Key                               | Type   | Default   | Description                              |
|-----------------------------------|--------|-----------|------------------------------------------|
| `pressure_advance`                | float  | `0.0`     | Classic linear PA amount (seconds)       |
| `smooth_time`                     | float  | `0.040`   | PA smoothing window (seconds)            |
| `pressure_advance_model`          | string | `"linear"`| `linear` / `power_law` / `cross_wlf`     |
| `pa_flow_index`                   | float  | `1.0`     | Power-law flow index $n$ (1 = Newtonian) |
| `pa_consistency`                  | float  | `0.0`     | $K_{\text{base}}$ [filament-mm / (mm³/s)^n] |
| `pa_max_compensation`             | float  | `0.5`     | Safety clamp on $|\delta e|$ [mm]        |
| `cross_wlf_tau_star`              | float  | `1e5`     | Cross $\tau^*$ [Pa]                      |
| `cross_wlf_flow_index`            | float  | `0.4`     | Cross $n$                                |
| `cross_wlf_c1`                    | float  | `17.44`   | WLF C1                                   |
| `cross_wlf_c2`                    | float  | `51.6`    | WLF C2 [K]                               |
| `cross_wlf_ref_temp`              | float  | `200.0`   | WLF $T_{\text{ref}}$ [°C]                |
| `cross_wlf_zero_shear_viscosity`  | float  | `1000.0`  | $\eta_{\text{ref}}$ [Pa·s]               |
| `cross_wlf_compressibility_over_area` | float | `0.0`  | $\beta V_m / A_f$ [mm/Pa]                |
| `cross_wlf_lut_path`              | string | `""`      | Optional serialized LUT path             |

### Example: power-law model

```ini
[extruder]
nozzle_diameter: 0.4
filament_diameter: 1.75
pressure_advance_model: power_law
pa_flow_index: 0.5
pa_consistency: 0.012
pa_max_compensation: 0.3
smooth_time: 0.040
```

### Example: Cross-WLF model

```ini
[extruder]
nozzle_diameter: 0.4
filament_diameter: 1.75
pressure_advance_model: cross_wlf
cross_wlf_tau_star: 100000.0
cross_wlf_flow_index: 0.4
cross_wlf_c1: 17.44
cross_wlf_c2: 51.6
cross_wlf_ref_temp: 200.0
cross_wlf_zero_shear_viscosity: 1500.0
cross_wlf_compressibility_over_area: 0.00001
```

## G-code commands

### `SET_PRESSURE_ADVANCE`

Extends the classic command with `MODEL`, `FLOW_INDEX`, and `CONSISTENCY`
parameters:

```
SET_PRESSURE_ADVANCE [ADVANCE=<pa>] [SMOOTH_TIME=<s>]
                      [MODEL=linear|power_law|cross_wlf]
                      [FLOW_INDEX=<n>] [CONSISTENCY=<K_base>]
```

### Status objects

The `pressure_advance` printer object now exposes a `model` field:

```json
{"pressure_advance": {"pressure_advance": 0.045, "smooth_time": 0.04, "model": "power_law"}}
```

## Implementation notes

### Position-offset form

All three models use the **position-offset** form rather than the
velocity-derivative form:

$$ e_{\text{adjusted}} = e_{\text{raw}} + \delta e(v_e) $$

This avoids numerical differentiation and the Q→0 singularity.  The smoothed
extruder velocity is computed once (centered moving average over `smooth_time`)
and reused across all models.

### Power-law model

The power-law (Ostwald–de Waele) model relates shear stress to shear rate:

$$ \tau = K \cdot \dot{\gamma}^n $$

The pressure inside the melt chamber is proportional to the wall shear stress,
and the compensating filament offset is:

$$ \delta e = K_{\text{base}} \cdot Q^n = K_{\text{base}} \cdot (v_e \cdot A_f)^n $$

where:
- $K_{\text{base}} = \beta V_m \cdot C_n / A_f$ [filament-mm / (mm³/s)^n]
- $Q = v_e \cdot A_f$ is the volumetric flow [mm³/s]
- $n$ is the flow index ($n < 1$ = shear-thinning, $n = 1$ = Newtonian)

When $n = 1$ and $K_{\text{base}} = \text{PA}$, the power-law model reduces exactly to the
linear model.

### Cross-WLF model

The Cross-WLF model captures the full temperature and shear-rate dependence of
the melt viscosity:

$$ \eta(T, \dot{\gamma}) = \frac{\eta_0(T)}{1 + (\eta_0(T) \cdot \dot{\gamma} / \tau^*)^{1-n}} $$

$$ \eta_0(T) = \eta_{\text{ref}} \cdot \frac{C_1 (T - T_{\text{ref}})}{C_2 + T - T_{\text{ref}}} \quad \text{[WLF shift]} $$

The pressure is obtained by integrating the wall shear stress over the nozzle
length, and a {Q,T}→P lookup table (`PressureFlowLut`) is built at startup for
fast online interpolation.  The compensating offset is:

$$ \delta e = \frac{\beta V_m}{A_f} \cdot P_{\text{LUT}}(Q, T_{\text{melt}}) $$

The melt temperature `T_melt` is typically fed from a `MeltZoneThermalObserver`
at runtime (see [Flow-Adaptive Temperature Control](FlowAdaptiveTemperatureControl.md)).

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/PowerLawRheology.hpp` | Power-law rheology model |
| `include/tether/control/extrusion/CrossWlfRheology.hpp` | Cross-WLF rheology model |
| `include/tether/control/extrusion/PressureFlowLut.hpp` | {Q,T}→P lookup table |
| `include/tether/control/extrusion/ExtrusionPressureModels.hpp` | Position-offset PA models |
| `include/tether/klipper/motion/MotionTranslator.hpp` | Step-generation with PA offset |
| `tests/control/test_power_law_rheology.cpp` | Power-law unit tests |
| `tests/control/test_extrusion_pressure_models.cpp` | PA model unit tests |
| `tests/klipper/test_klipper_extrusion_compensation.cpp` | Klipper integration tests |
