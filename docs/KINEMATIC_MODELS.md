# Kinematic Models in the Tether Codebase

This document catalogs every forward and inverse (backwards) kinematics model
found in the codebase. All kinematics models now live in the
**`tether_kinematics`** module (`include/tether/kinematics/`, namespace
`tether::kinematics`), with a unified naming scheme:

- **Robotics lineage**: `forwardKinematics` (joint → Cartesian),
  `inverseKinematics` (Cartesian → joint)
- **Printer lineage**: `forwardActuatorKinematics` (Cartesian → actuator),
  `inverseActuatorKinematics` (actuator → Cartesian)

The "forward" direction is qualified by domain so the two lineages — which
have opposite definitions of "forward" — are never confused.

---

## 1. Generic Robotics Kinematics

Located in `include/tether/kinematics/`. All implementations use SI units
(meters, radians) and live in `namespace tether::kinematics`.

### 1.1 Forward Kinematics — `ForwardKinematics.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/ForwardKinematics.hpp" />

Abstract base `ForwardKinematicsBase` (line 494) defines the interface:
`forwardKinematics(const float* joint_positions) -> Pose6D` and
`getTransform(const float* joint_positions) -> Transform4x4`.

The following concrete forward-kinematics models are provided (joint →
end-effector pose). Several also include a closed-form inverse direction.

| Model | Class | DOF | Forward (joint→Cart) | Inverse (Cart→joint) | Notes |
|---|---|---|---|---|---|
| 2-DOF planar arm (RR) | `Planar2DOF` | 2 | `forwardKinematics` / `computePosition` (line 532) | — (analytic reachable via `getMaxReach`/`getMinReach`) | Base + elbow rotation about Z |
| 3-DOF articulated arm (RRR) | `Articulated3DOF` | 3 | `forwardKinematics` / `getTransform` (line 611) | — | Base yaw + shoulder/elbow pitch |
| 6-DOF serial manipulator (DH) | `Serial6DOF` | 6 | `forwardKinematics` / `getTransform` (line 675) | — | Configurable DH params; presets for UR5 (`setUR5Parameters`) and KUKA KR6 (`setKukaKR6Parameters`) |
| 7-DOF redundant manipulator | `Serial7DOF` | 7 | `forwardKinematics` / `getTransform` (line 769) | — | Franka Emika Panda preset (`setPandaParameters`) |
| SCARA (RRPR) | `SCARA` | 4 | `forwardKinematics` / `getTransform` (line 835) | — | 2 rotary + Z slide + tool rotation |
| Delta (parallel) | `DeltaRobot` | 3 | `forwardKinematics` / `computePosition` (line 910) | — | Trilateration of 3 sphere centers (`solvTrilateration`) |
| Cartesian / gantry (PPP) | `CartesianRobot` | 3 | `forwardKinematics` (line 1029) | trivial identity | X, Y, Z linear |
| 5-DOF gantry | `Gantry5DOF` | 5 | `forwardKinematics` / `getTransform` (line 1052) | — | XYZ + pitch + roll |
| Stewart platform (hexapod) | `StewartPlatform` | 6 | `forwardKinematics` (line 1304, Newton-Raphson) | `inverseKinematics` (line 1403, pose → leg lengths) | Iterative FK; closed-form IK |

#### Mobile-robot kinematics (also in `ForwardKinematics.hpp`)

These provide both forward (wheel → body) and inverse (body → wheel) velocity
kinematics, plus pose integration. The `wheelToBody`/`bodyToWheel` method
names are kept as-is since they are unambiguous (not "forward/inverse").

| Model | Class | Forward (wheel→body) | Inverse (body→wheel) | Pose update |
|---|---|---|---|---|
| Differential drive | `DifferentialDrive` (line 1083) | `wheelToBody` (line 1099) | `bodyToWheel` (line 1140) | `updatePose` (line 1117) |
| 3-wheel omnidirectional | `OmniDrive3Wheel` (line 1160) | `wheelToBody` (line 1175) | `bodyToWheel` (line 1197) | `updatePose` (line 1212) |
| 4-wheel Mecanum | `MecanumDrive` (line 1237) | `wheelToBody` (line 1253) | `bodyToWheel` (line 1267) | `updatePose` (line 1278) |

### 1.2 Forward & Inverse Dynamics — `ForwardDynamics.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/ForwardDynamics.hpp" />

This file provides **dynamics** models (torque ↔ acceleration), not pose
kinematics. It is included here because dynamics is the natural counterpart
to kinematics and the file ships side-by-side with `ForwardKinematics.hpp`.
Each model implements both `forwardDynamics` (torque → acceleration) and
`inverseDynamics` (acceleration → torque). These method names are standard,
unambiguous robotics terms and are kept as-is.

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

## 2. Printer Kinematics

Located in `include/tether/kinematics/`. Uses the 3D-printer convention
(**Forward = Cartesian → actuator**, **Inverse = actuator → Cartesian**),
now expressed via the qualified method names
`forwardActuatorKinematics` / `inverseActuatorKinematics`.
Units are millimeters and degrees/radians as noted.

### 2.1 Linear Delta Printer — `DeltaPrinter.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/DeltaPrinter.hpp" />

Class `tether::kinematics::DeltaPrinter` models a 3-tower linear delta.

| Direction | Method | Description |
|---|---|---|
| Forward (Cart → tower) | `forwardActuatorKinematics` | Closed-form: tower height = `z + sqrt(armLength² - dist²) + endstopAdj` for each of 3 towers at 120° spacing |
| Inverse (tower → Cart) | `inverseActuatorKinematics` | Trilateration: subtract sphere equations to get linear system in x,y (in terms of z), then solve quadratic in z; picks the root with smallest total sphere error |

Geometry via `DeltaGeometry` (M665: arm length, delta radius, tower angle
offsets) and `DeltaEndstopAdjust` (M666).

### 2.2 Rotary Delta Printer — `RotaryDeltaPrinter.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/RotaryDeltaPrinter.hpp" />

Class `tether::kinematics::RotaryDeltaPrinter` models a rotary delta
with upper arms (L1) + forearms (L2) pivoting at 120°-spaced shoulders.

| Direction | Method | Description |
|---|---|---|
| Forward (Cart → shoulder angles) | `forwardActuatorKinematics` | Per tower: solve `P·cos θ + Q·sin θ = K` via `θ = φ ± atan2(denom, K)`; picks the more-horizontal solution |
| Inverse (shoulder angles → Cart) | `inverseActuatorKinematics` | Compute upper-arm-end sphere centers, then trilaterate with forearm length L2 (quadratic in z, picks lower root) |

Geometry via `RotaryDeltaGeometry` (upper/forearm lengths, base/effector
radius, base height, tower angles) and `RotaryDeltaEndstopAdjust`.

### 2.3 Printer Kinematics Enum — `PrinterKinematics.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/PrinterKinematics.hpp" />

`enum class PrinterKinematics` enumerates all supported printer
kinematics types and their string mappings
(`printerKinematicsFromString` / `printerKinematicsToString`):

`Cartesian`, `CoreXY`, `CoreXZ`, `CoreYZ`, `HybridCoreXY`, `HybridCoreXZ`,
`Delta`, `RotaryDelta`, `Polar`, `Winch`, `None`.

Per-kinematics config structs: `DeltaGeometry`/`DeltaEndstopAdjust` (in
`DeltaPrinter.hpp`), `RotaryDeltaGeometry`/`RotaryDeltaEndstopAdjust` (in
`RotaryDeltaPrinter.hpp`), `PolarConfig` and `WinchConfig` (in
`PrinterKinematics.hpp`).

The Klipper module retains backward-compatible aliases in
`KlippyInstanceConfig.hpp` (`using Kinematics = PrinterKinematics;`,
`kinematicsFromString`, `kinematicsToString`) so existing klippy code
continues to compile.

### 2.4 Kinematics Transform Dispatcher — `KinematicsTransform.hpp`

<ref_file file="/home/uli/dev/Tether/include/tether/kinematics/KinematicsTransform.hpp" />

Class `tether::kinematics::KinematicsTransform` is the central dispatcher
that converts between Cartesian and stepper-space for all printer
kinematics. It holds the forward/inverse pair:

| Direction | Method | Coverage |
|---|---|---|
| Forward (Cart → stepper) | `forwardActuatorKinematics` | Cartesian, CoreXY, CoreXZ, CoreYZ, HybridCoreXY, HybridCoreXZ, Delta, RotaryDelta, Polar, Winch |
| Inverse (stepper → Cart) | `inverseActuatorKinematics` | Same set |

Inline forward formulas:
- **CoreXY**: `A = X+Y, B = X-Y, C = Z`
- **CoreXZ**: `A = X+Z, B = X-Z, C = Y`
- **CoreYZ**: `A = Y+Z, B = Y-Z, C = X`
- **HybridCoreXY / HybridCoreXZ**: same as CoreXY / CoreXZ respectively
- **Delta / RotaryDelta**: delegates to `DeltaPrinter`/`RotaryDeltaPrinter`
- **Polar**: `A = sqrt(x²+y²), B = atan2(y,x)·180/π, C = Z`
- **Winch**: cable lengths from 3 anchors at equilateral triangle
  (`la,lb,lc = sqrt((x-a)² + (y-a)² + (z-h)²)`)

Inline inverse formulas mirror these; the Winch inverse uses
a simplified 2-anchor trilateration.

The Klipper module retains a backward-compatible alias in
`MotionTranslator.hpp` (`using KinematicsTransform = ::tether::kinematics::KinematicsTransform;`)
so existing code referencing `tether::klipper::motion::KinematicsTransform`
continues to compile.

---

## 3. CMake Module Structure

The `tether_kinematics` component is a **header-only INTERFACE library**:

- **CMake component**: `cmake/components/kinematics.cmake`
- **CMake target**: `tether_kinematics` (INTERFACE), alias `tether::kinematics`
- **Build option**: `TETHER_BUILD_KINEMATICS` (default ON)
- **Dependencies**: `tether_common`
- **Consumers**: `tether_motion_control` (PUBLIC link), `tether_klipper` (PUBLIC link)
- **ESP-IDF**: `TETHER_ENABLE_KINEMATICS` Kconfig option (no `depends on`)

---

## 4. Examples, Tests & Bindings

These consume the models above rather than defining new ones, but are useful
references for usage.

| File | Purpose |
|---|---|
| `examples/klipper_delta_kinematics.cpp` | Demonstrates `DeltaPrinter` forward/inverse round-trip, M665 geometry, M666 endstop adjustment |
| `tests/klipper/test_klipper_rotary_delta.cpp` | `RotaryDeltaPrinter` forward/inverse round-trip and edge cases |
| `tests/klipper/test_klipper_polar_winch.cpp` | Polar & Winch `KinematicsTransform` correctness |
| `tests/klipper/test_klipper_missing_features.cpp` | `DeltaPrinter` forwardActuatorKinematics / inverseActuatorKinematics |
| `tests/klipper/test_klipper_e3_features.cpp` | `KinematicsTransform` round-trip, `PrinterKinematics` enum/string helpers |
| `tests/klipper/test_klipper_e4_wiring.cpp` | `KinematicsTransform` round-trip for multiple kinematics types |
| `python_bindings/bindings/klipper_bindings.cpp` | Python bindings exposing `forward_actuator_kinematics` / `inverse_actuator_kinematics` |

---

## 5. Related but Not Robot-Kinematics Models

These files mention "kinematic" but implement motion-planning physics or
dynamical-system simulation rather than robot pose kinematics:

- `include/tether/gcode/motion/GCodeMath.hpp` — point-mass kinematic
  equations (v² = v₀² + 2as, etc.) for velocity/acceleration planning.
- `include/tether/gcode/motion/InterpolationStrategy.hpp` —
  `KinematicLimits` struct (max velocity/accel/jerk) for trajectory planning.
- `include/tether/simulation/systems/rotational/ControlMomentGyroscope.hpp`
  and `include/tether/simulation/systems/aerospace/BicycleLean.hpp` —
  dynamical-system benchmarks (gyroscopic coupling, lean-steer dynamics),
  not pose kinematics. These remain in the `tether_simulation` module.
