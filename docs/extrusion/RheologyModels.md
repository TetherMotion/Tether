# Rheology Models

## Overview

Tether provides standalone, framework-agnostic rheology models for polymer
melts used in FFF/FDM 3D printing.  These models relate **volumetric flow rate**
$(Q)$ and **melt temperature** $(T)$ to the **pressure** $(P)$ inside the nozzle,
which is the physical basis for non-Newtonian pressure advance and flow-adaptive
temperature control.

All models live in `tether::control::extrusion` and have no dependencies on the
Klipper or motion-planning layers.

## Power-Law (Ostwald–de Waele) Model

### Physics

The simplest non-Newtonian model.  The apparent viscosity is a power law in
shear rate:

$$ \eta(\dot{\gamma}) = K \cdot \dot{\gamma}^{n-1} $$

$$ \tau = K \cdot \dot{\gamma}^n \quad \text{(shear stress)} $$

For pressure-driven flow in a capillary (nozzle):

$$ P = \frac{2 K L}{R} \cdot \left(\frac{n+3}{4}\right)^{1/n} \cdot \left(\frac{Q}{\pi R^3}\right)^{1/n} $$

Wait — this is the **forward** direction ($Q \to P$).  The PA compensation needs
the **inverse**: given the flow $Q$, compute the pressure $P$.  The forward form is
already what we need:

$$ P(Q) = C_n \cdot K \cdot Q^n $$

where `C_n` collects the nozzle geometry constants.

### API

```cpp
#include "tether/control/extrusion/PowerLawRheology.hpp"

tether::control::extrusion::PowerLawParams p;
p.consistency = 1000.0;   // K [Pa·s^n]
p.flowIndex = 0.5;        // n

double P = tether::control::extrusion::PowerLawRheology::pressure(
    p, Q_mm3_per_s, nozzle);
```

### Parameters

| Parameter     | Symbol | Unit         | Description                    |
|---------------|--------|--------------|--------------------------------|
| `consistency` | $K$      | Pa·s^n       | Melt consistency               |
| `flowIndex`   | $n$      | —            | Flow index ($n<1$ = thinning)  |

### Nozzle geometry

| Parameter     | Symbol | Unit | Description            |
|---------------|--------|------|------------------------|
| `radius`      | $R$      | mm   | Nozzle bore radius     |
| `length`      | $L$      | mm   | Melt channel length    |

## Cross-WLF Model

### Physics

The Cross model with WLF (Williams–Landel–Ferry) temperature shift captures the
full shear-rate and temperature dependence of amorphous polymer melts:

$$ \eta(T, \dot{\gamma}) = \frac{\eta_0(T)}{1 + \left(\frac{\eta_0(T) \cdot \dot{\gamma}}{\tau^*}\right)^{(1-n)/n}} $$

$$ \eta_0(T) = \eta_{\text{ref}} \cdot \exp\left(-\frac{C_1 (T - T_{\text{ref}})}{C_2 + T - T_{\text{ref}}}\right) \quad \text{[WLF shift]} $$

For capillary flow, the wall shear rate is:

$$ \dot{\gamma}_w = \frac{4Q}{\pi R^3} $$

and the pressure is obtained by integrating the wall shear stress over the
nozzle length:

$$ P = \frac{2 L \tau_w}{R} = \frac{2 L \eta(T, \dot{\gamma}_w) \dot{\gamma}_w}{R} $$

### API

```cpp
#include "tether/control/extrusion/CrossWlfRheology.hpp"

tether::control::extrusion::CrossWlfParams p;
p.tauStar = 1e5;         // τ* [Pa]
p.flowIndex = 0.4;       // n
p.c1 = 17.44;            // WLF C1
p.c2 = 51.6;             // WLF C2 [K]
p.refTempC = 200.0;      // T_ref [°C]
p.zeroShearViscosityRef = 1000.0;  // η_ref [Pa·s]

double P = tether::control::extrusion::CrossWlfRheology::pressure(
    p, Q_mm3_per_s, T_melt_C, nozzle);
```

### Parameters

| Parameter                 | Symbol  | Unit   | Description              |
|---------------------------|---------|--------|--------------------------|
| `tauStar`                 | $\tau^*$      | Pa     | Critical stress          |
| `flowIndex`               | $n$       | —      | Cross flow index         |
| `c1`                      | $C_1$      | —      | WLF $C_1$                   |
| `c2`                      | $C_2$      | K      | WLF $C_2$                   |
| `refTempC`                | $T_{\text{ref}}$   | °C     | WLF reference temperature|
| `zeroShearViscosityRef`   | $\eta_{\text{ref}}$   | Pa·s   | Zero-shear viscosity at $T_{\text{ref}}$ |

## Pressure-Flow LUT

### Overview

For online use in the motion pipeline, calling the full Cross-WLF equation
per sample is too expensive.  A $\{Q, T\} \to P$ lookup table (`PressureFlowLut`)
is built at startup and queried with bilinear interpolation at runtime.

### API

```cpp
#include "tether/control/extrusion/PressureFlowLut.hpp"

auto lut = std::make_shared<tether::control::extrusion::PressureFlowLut>();
lut->build(crossWlfParams, nozzleGeometry,
           {1.0, 2.0, 4.0, 8.0},    // Q grid [mm³/s]
           {200.0, 220.0, 240.0});  // T grid [°C]

double P = lut->pressure(Q, T);  // bilinear interpolation
```

### Grid selection

- **Q grid**: Should span the expected print speed range.  For a 1.75 mm
  filament at 20–100 mm/s, Q ranges from ~48 to ~240 mm³/s.  A typical grid:
  `{1, 2, 5, 10, 20, 50, 100, 200, 400}` mm³/s.
- **T grid**: Should span the expected melt temperature range.  For PLA:
  `{190, 200, 210, 220, 230}` °C.

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/PowerLawRheology.hpp` | Power-law model |
| `include/tether/control/extrusion/CrossWlfRheology.hpp` | Cross-WLF model |
| `include/tether/control/extrusion/PressureFlowLut.hpp` | $\{Q,T\} \to P$ LUT |
| `src/control/extrusion/CrossWlfRheology.cpp` | Cross-WLF implementation |
| `src/control/extrusion/PressureFlowLut.cpp` | LUT implementation |
| `tests/control/test_power_law_rheology.cpp` | Power-law unit tests |
