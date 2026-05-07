# Rotational Systems

Rotational systems cover servo drives, compliant couplings, and spacecraft-style attitude actuators. For identification, `FlexibleShaft` is the main benchmark because it gives a clean torsional resonance with physically meaningful motor-side and load-side inertia splits.

## Recommended Benchmarks

- Use `DCMotorSpeed` or `DCMotorPosition` for fast low-order electromechanical identification.
- Use `FlexibleShaft` when you need a dominant torsional mode plus load-side measurement dynamics.
- Use `DiskDriveHead` when you want a compact high-bandwidth resonant plant.

## Identification Notes

The rotational family is useful when you want to test whether an estimator separates rigid-body motion from shaft or structural flexibility. A good excitation signal should include low-frequency content for inertia estimation and enough mid-band energy to reveal shaft or head resonances.

## Tradeoffs

- Advantage: Strong connection to common servo-drive commissioning tasks.
- Advantage: Flexible-shaft and disk-drive models produce spectral peaks that are easy to check against ETFE results.
- Disadvantage: Some plants include coupled electrical and mechanical time scales, which can widen the identification bandwidth requirement.
- Disadvantage: Load-side-only measurements can make parameter recovery less direct than in translational benchmarks.
