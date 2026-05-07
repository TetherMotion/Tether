# Chaotic Systems

Chaotic systems are not the first choice for low-order parametric identification, but they are valuable for stress-testing frequency estimators and nonlinear model structures. `DuffingOscillator` is the most practical entry point because it retains a dominant forced response while still introducing amplitude-dependent resonance.

## Recommended Benchmarks

- Use `DuffingOscillator` for nonlinear resonance and amplitude-dependent peak shifts.
- Use `LorenzSystem` or `RosslerSystem` when you want sensitivity to initial conditions more than spectral interpretability.
- Use `KapitzaPendulum` when you want a parametrically excited system rather than a purely forced one.

## Identification Notes

For this family, success is usually defined by recovering dominant peaks, tracking operating-regime changes, or demonstrating robustness to colored measurement noise, not by perfectly fitting a low-order linear model across the whole state space.

## Tradeoffs

- Advantage: Useful for checking that estimators do not collapse in the presence of strong nonlinearity.
- Advantage: Duffing gives a controlled way to test nonlinear frequency response without needing a large state dimension.
- Disadvantage: Fit percentages can be misleading when the trajectory changes regime with small perturbations.
- Disadvantage: Repeatability depends strongly on consistent initial conditions and forcing choices.
