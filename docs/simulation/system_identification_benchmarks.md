# System Identification Benchmarks

This note maps the simulation library to the identification stack so tests and experiments can pick the right plant quickly.

## Fast Choices

- `MassSpringDamper`: best default for ARX, OE, ETFE, and resonance checks.
- `FlexibleShaft`: best default for coupled-mode rotational identification and torsional resonance tests.
- `DuffingOscillator`: best default for nonlinear spectral tests where a dominant forced peak should remain visible.
- `CSTR`: best default for nonlinear process identification around an operating point.

## Disturbance Patterns Covered By The Integration Tests

- Additive white measurement noise.
- Colored measurement noise from deterministic sinusoidal contamination.
- Input-side process disturbance added to the commanded actuation.
- Constant sensor bias.
- Actuator saturation represented as a Hammerstein input nonlinearity.

## Practical Guidance

Use mechanical and rotational plants when you need short experiments and crisp spectral assertions. Use chemical plants when disturbance interpretation matters more than raw fit percentage. Use chaotic plants when the main question is whether the identification result still preserves dominant peaks or regime changes under nonlinearity.