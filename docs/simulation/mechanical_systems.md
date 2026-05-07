# Mechanical Systems

Mechanical systems are the default starting point for system-identification work in Tether because they expose clean force-to-motion dynamics, interpretable parameters, and controllable resonance. The most useful entry points are `MassSpringDamper` for a single dominant mode, `CoupledMassSpringDamper` for multi-mode translational coupling, and the suspension models when you want road-like disturbances and lightly damped poles.

## When To Use Them

- Use `MassSpringDamper` when you need a benchmark with analytically predictable natural frequency and damping ratio.
- Use `CoupledMassSpringDamper` or `VibrationIsolationPlatform` when you need more than one resonant mode in a compact model.
- Use `QuarterCarSuspension` or `HalfCarSuspension` when disturbance rejection matters more than clean modal interpretation.

## Identification Notes

These systems are usually well matched to ARX, OE, ETFE, and subspace identification because the output is dominated by a small number of modes. If you inject actuator saturation or Coulomb-like effects externally, the same plants also become useful Hammerstein-Wiener benchmarks.

## Tradeoffs

- Advantage: Parameters map directly to physical concepts like mass, stiffness, and damping.
- Advantage: Resonant frequencies are easy to predict and validate against estimated spectra.
- Disadvantage: Very stiff parameter choices can require smaller integration steps for repeatable results.
- Disadvantage: Highly idealized models may overstate achievable fit compared with friction-heavy real mechanisms.
