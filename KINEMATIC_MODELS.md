# Kinematic Models in the Tether Codebase

This document catalogs every forward and inverse (backwards) kinematics model
found in the codebase. There are two distinct lineages of kinematics code with
**different naming conventions**, which is important to be aware of:

| Lineage | Location | Convention |
|---|---|---|
| **Generic robotics** | `include/tether/kinematics/` | Standard robotics: **Forward = joint → Cartesian**, **Inverse = Cartesian → joint** |
| **Klipper / 3D-printer** | `include/tether/klipper/` | 3D-printer: **Forward = Cartesian → actuator**, **Inverse = actuator → Cartesian** |

These conventions are opposite. In the Klipper/printer code, "forward" means
the direction normally computed when commanding a move (Cartesian target →
stepper/actuator positions), while in the generic robotics code "forward"
follows the textbook definition (joint values → end-effector pose).

---

## 1. Generic Robotics Kinematics

Located in `include/tether/kinematics/`. All implementations use SI units
(meters, radians) and live in `namespace Kinematics` / `namespace Dynamics`.

### 1.1 Forward Kinematics — `ForwardKinematics.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/ForwardKinematics.hpp" />

Abstract base `ForwardKinematicsBase` (line 494) defines the interface:
`compute(const float* joint_positions) -> Pose6D` and
`getTransform(const float* joint_positions) -> Transform4x4`.

The following concrete forward-kinematics models are provided (joint →
end-effector pose). Several also include a closed-form inverse direction.

| Model | Class | DOF | Forward (joint→Cart) | Inverse (Cart→joint) | Notes |
|---|---|---|---|---|---|
| 2-DOF planar arm (RR) | `Planar2DOF` | 2 | `compute` / `computePosition` (line 532) | — (analytic reachable via `getMaxReach`/`getMinReach`) | Base + elbow rotation about Z |
| 3-DOF articulated arm (RRR) | `Articulated3DOF` | 3 | `compute` / `getTransform` (line 611) | — | Base yaw + shoulder/elbow pitch |
| 6-DOF serial manipulator (DH) | `Serial6DOF` | 6 | `compute` / `getTransform` (line 675) | — | Configurable DH params; presets for UR5 (`setUR5Parameters`) and KUKA KR6 (`setKukaKR6Parameters`) |
| 7-DOF redundant manipulator | `Serial7DOF` | 7 | `compute` / `getTransform` (line 769) | — | Franka Emika Panda preset (`setPandaParameters`) |
| SCARA (RRPR) | `SCARA` | 4 | `compute` / `getTransform` (line 835) | — | 2 rotary + Z slide + tool rotation |
| Delta (parallel) | `DeltaRobot` | 3 | `compute` / `computePosition` (line 910) | — | Trilateration of 3 sphere centers (`solvTrilateration`) |
| Cartesian / gantry (PPP) | `CartesianRobot` | 3 | `compute` (line 1029) | trivial identity | X, Y, Z linear |
| 5-DOF gantry | `Gantry5DOF` | 5 | `compute` / `getTransform` (line 1052) | — | XYZ + pitch + roll |
| Stewart platform (hexapod) | `StewartPlatform` | 6 | `compute` (line 1304, Newton-Raphson) | `computeLegLengths` (line 1403, pose → leg lengths) | Iterative FK; closed-form IK |

#### Mobile-robot kinematics (also in `ForwardKinematics.hpp`)

These provide both forward (wheel → body) and inverse (body → wheel) velocity
kinematics, plus pose integration.

| Model | Class | Forward (wheel→body) | Inverse (body→wheel) | Pose update |
|---|---|---|---|---|
| Differential drive | `DifferentialDrive` (line 1083) | `wheelToBody` (line 1099) | `bodyToWheel` (line 1140) | `updatePose` (line 1117) |
| 3-wheel omnidirectional | `OmniDrive3Wheel` (line 1160) | `wheelToBody` (line 1175) | `bodyToWheel` (line 1197) | `updatePose` (line 1212) |
| 4-wheel Mecanum | `MecanumDrive` (line 1237) | `wheelToBody` (line 1253) | `bodyToWheel` (line 1267) | `updatePose` (line 1278) |

### 1.2 Forward & Inverse Dynamics — `ForwardDynamics.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/ForwardDynamics.hpp" />

This file lives in `namespace Dynamics` and provides **dynamics** models
(torque ↔ acceleration), not pose kinematics. It is included here because
dynamics is the natural counterpart to kinematics and the file ships
side-by-side with `ForwardKinematics.hpp`. Each model implements both
`forwardDynamics` (torque → acceleration) and `inverseDynamics`
(acceleration → torque).

| Model | Class | Forward dynamics | Inverse dynamics |
|---|---|---|---|
| 2-DOF planar arm | `Planar2DOFDynamics` (line 175) | `forwardDynamics` (line 264) | `inverseDynamics` (line 303) |
| 3-DOF articulated arm | `Articulated3DOFDynamics` (line 377) | `forwardDynamics` (line 431) | `inverseDynamics` (line 462) |
| Single joint / motor | `SingleJointDynamics` (line 572) | `forwardDynamics` (line 591) | `inverseDynamics` (line 601) |
| SCARA (RRPR) | `SCARADynamics` (line 662) | `forwardDynamics` (line 678) | — (delegates to planar + single-joint) |
| Differential drive | `DifferentialDriveDynamics` (line 717) | `forwardDynamics` (line 747) | — |
| Generic serial (Newton-Euler) | `NewtonEulerDynamics<N>` (line 843) | — | `inverseDynamics` (line 876) | O(n) recursive; revolute & prismatic joints |

Supporting infrastructure: `FrictionModel` (viscous + Coulomb + Stribeck,
line 100), `LinkProperties` (line 54), spatial types
(`SpatialVelocity`/`SpatialAcceleration`/`SpatialForce`/`SpatialInertia`),
and `ComputedTorqueController<N>` (line 975) which uses inverse dynamics for
feedforward + PD feedback trajectory tracking.

---

## 2. Klipper / 3D-Printer Kinematics

Located in `include/tether/klipper/`. Uses the 3D-printer convention
(**Forward = Cartesian → actuator**, **Inverse = actuator → Cartesian**).
Units are millimeters and degrees/radians as noted.

### 2.1 Linear Delta Printer — `DeltaPrinter.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/klipper/klippy/DeltaPrinter.hpp" />

Class `tether::klipper::klippy::DeltaPrinter` models a 3-tower linear delta.

| Direction | Method | Line | Description |
|---|---|---|---|
| Forward (Cart → tower) | `cartesianToTower` | 44 | Closed-form: tower height = `z + sqrt(armLength² - dist²) + endstopAdj` for each of 3 towers at 120° spacing |
| Inverse (tower → Cart) | `towerToCartesian` | 81 | Trilateration: subtract sphere equations to get linear system in x,y (in terms of z), then solve quadratic in z; picks the root with smallest total sphere error |

Geometry via `DeltaGeometry` (M665: arm length, delta radius, tower angle
offsets) and `DeltaEndstopAdjust` (M666).

### 2.2 Rotary Delta Printer — `RotaryDeltaPrinter.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/klipper/klippy/RotaryDeltaPrinter.hpp" />

Class `tether::klipper::klippy::RotaryDeltaPrinter` models a rotary delta
with upper arms (L1) + forearms (L2) pivoting at 120°-spaced shoulders.

| Direction | Method | Line | Description |
|---|---|---|---|
| Forward (Cart → shoulder angles) | `cartesianToTower` | 94 | Per tower: solve `P·cos θ + Q·sin θ = K` via `θ = φ ± atan2(denom, K)`; picks the more-horizontal solution |
| Inverse (shoulder angles → Cart) | `towerToCartesian` | 170 | Compute upper-arm-end sphere centers, then trilaterate with forearm length L2 (quadratic in z, picks lower root) |

Geometry via `RotaryDeltaGeometry` (upper/forearm lengths, base/effector
radius, base height, tower angles) and `RotaryDeltaEndstopAdjust`.

### 2.3 Printer Kinematics Enum — `KlippyInstanceConfig.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/klipper/klippy/KlippyInstanceConfig.hpp" />

`enum class Kinematics` (line 19) enumerates all supported printer
kinematics types and their string mappings (line 35+):

`Cartesian`, `CoreXY`, `CoreXZ`, `CoreYZ`, `HybridCoreXY`, `HybridCoreXZ`,
`Delta`, `RotaryDelta`, `Polar`, `Winch`, `None`.

Per-kinematics config structs: `DeltaGeometry`/`DeltaEndstopAdjust`,
`RotaryDeltaGeometry`/`RotaryDeltaEndstopAdjust`, `PolarConfig` (line 367),
`WinchConfig` (line 375).

### 2.4 Kinematics Transform Dispatcher — `MotionTranslator.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/klipper/motion/MotionTranslator.hpp" />

Class `tether::klipper::motion::KinematicsTransform` (line 63) is the
central dispatcher that converts between Cartesian and stepper-space for
all printer kinematics. It holds the forward/inverse pair:

| Direction | Method | Line | Coverage |
|---|---|---|---|
| Forward (Cart → stepper) | `transform` | 89 | Cartesian, CoreXY, CoreXZ, CoreYZ, HybridCoreXY, HybridCoreXZ, Delta, RotaryDelta, Polar, Winch |
| Inverse (stepper → Cart) | `inverseTransform` | 153 | Same set |

Inline forward formulas (line 89–147):
- **CoreXY**: `A = X+Y, B = X-Y, C = Z`
- **CoreXZ**: `A = X+Z, B = X-Z, C = Y`
- **CoreYZ**: `A = Y+Z, B = Y-Z, C = X`
- **HybridCoreXY / HybridCoreXZ**: same as CoreXY / CoreXZ respectively
- **Delta / RotaryDelta**: delegates to `DeltaPrinter`/`RotaryDeltaPrinter`
- **Polar**: `A = sqrt(x²+y²), B = atan2(y,x)·180/π, C = Z`
- **Winch**: cable lengths from 3 anchors at equilateral triangle
  (`la,lb,lc = sqrt((x-a)² + (y-a)² + (z-h)²)`)

Inline inverse formulas (line 153–208) mirror these; the Winch inverse uses
a simplified 2-anchor trilateration.

---

## 3. Examples, Tests & Bindings

These consume the models above rather than defining new ones, but are useful
references for usage.

| File | Purpose |
|---|---|
| `examples/klipper_delta_kinematics.cpp` | Demonstrates `DeltaPrinter` forward/inverse round-trip, M665 geometry, M666 endstop adjustment |
| `tests/klipper/test_klipper_rotary_delta.cpp` | `RotaryDeltaPrinter` forward/inverse round-trip and edge cases |
| `tests/klipper/test_klipper_polar_winch.cpp` | Polar & Winch `KinematicsTransform` correctness |
| `tests/klipper/test_klipper_missing_features.cpp` | `DeltaPrinter` cartesianToTower / towerToCartesian |
| `python_bindings/bindings/klipper_bindings.cpp` | Python bindings exposing `cartesian_to_tower` / `tower_to_cartesian` (line 466) |

---

## 4. Related but Not Robot-Kinematics Models

These files mention "kinematic" but implement motion-planning physics or
dynamical-system simulation rather than robot pose kinematics:

- `include/tether/gcode/motion/GCodeMath.hpp` — point-mass kinematic
  equations (v² = v₀² + 2as, etc.) for velocity/acceleration planning.
- `include/tether/gcode/motion/InterpolationStrategy.hpp` —
  `KinematicLimits` struct (max velocity/accel/jerk) for trajectory planning.
- `include/tether/simulation/systems/rotational/ControlMomentGyroscope.hpp`
  and `include/tether/simulation/systems/aerospace/BicycleLean.hpp` —
  dynamical-system benchmarks (gyroscopic coupling, lean-steer dynamics),
  not pose kinematics.
