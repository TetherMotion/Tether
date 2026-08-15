# LTI Frequency-Domain Deconvolution

## Overview

The `LTIFrequencyDomainDeconvolver` is the baseline deconvolution controller.
Given a measured LTI impulse response $h[n]$ and a target output trajectory
$y_{\text{tgt}}[n]$, it computes the required input $x_{\text{req}}[n]$ such that the convolution
$(x_{\text{req}} * h)[n]$ approximates $y_{\text{tgt}}[n]$.

In the frequency domain, convolution becomes multiplication:

$$ Y[k] = H[k] \cdot X[k] $$

Naive inversion $X[k] = Y[k] / H[k]$ is ill-posed because $|H[k]| \to 0$ at
high frequencies (the system is low-pass).  Tikhonov regularization
stabilises the inversion:

$$ X_{\text{req}}[k] = \frac{Y_{\text{tgt}}[k] \cdot H^*[k]}{|H[k]|^2 + \lambda} $$

where $\lambda > 0$ is the regularization parameter.  This is equivalent to the
Wiener filter when $\lambda$ represents the noise-to-signal ratio.

## Algorithm

1. **Zero-padding**: Pad $h$ and $y_{\text{tgt}}$ to length $N \geq L_y + L_h - 1$
   (next power of 2 for FFT efficiency) to prevent circular-convolution
   artifacts (time-aliasing).
2. **Forward FFT**: Compute $H[k] = \text{FFT}(h_{\text{padded}})$ and
   $Y_{\text{tgt}}[k] = \text{FFT}(y_{\text{tgt,padded}})$.
3. **Regularized division**:
   $X_{\text{req}}[k] = Y_{\text{tgt}}[k] \cdot \text{conj}(H[k]) / (|H[k]|^2 + \lambda)$.
4. **Inverse FFT**: $x_{\text{req,padded}}[n] = \text{IFFT}(X_{\text{req}}[k])$.
5. **Truncation & shift**: Extract the first $L_y$ real values, shifted left
   by the group delay (peak index of $h[n]$) for causal alignment.

## API

```cpp
#include "tether/control/extrusion/LTIFrequencyDomainDeconvolver.hpp"

using namespace tether::control::extrusion;

// Configure
LTIDeconvolutionParams params;
params.lambda = 1e-6;
params.padToPowerOfTwo = true;
LTIFrequencyDomainDeconvolver deconv(params);

// Two usage patterns:

// Pattern A: Precompute inverse, then deconvolve multiple targets
deconv.setImpulseResponse(h);
deconv.precomputeInverseFilter();
auto x1 = deconv.deconvolve(y_tgt_1);
auto x2 = deconv.deconvolve(y_tgt_2);  // reuses precomputed inverse

// Pattern B: One-shot
auto x = deconv.deconvolve(y_tgt, h);
```

### Parameters

| Parameter          | Type   | Default | Description                              |
|--------------------|--------|---------|------------------------------------------|
| `lambda`           | double | `1e-6`  | Tikhonov regularization parameter $\lambda > 0$  |
| `padToPowerOfTwo`  | bool   | `true`  | Pad to next power of 2 for FFT efficiency|

### Key methods

| Method                      | Description                              |
|-----------------------------|------------------------------------------|
| `setImpulseResponse(h)`     | Set the measured LTI impulse response    |
| `precomputeInverseFilter()` | Build the regularized inverse $h_{\text{inv}}[n]$ |
| `deconvolve(y_tgt)`         | Deconvolve using precomputed inverse     |
| `deconvolve(y_tgt, h)`      | One-shot: set h, precompute, deconvolve  |
| `setLambda($\lambda$)`      | Update $\lambda$ and recompute the inverse       |
| `groupDelay()`              | Peak index of $h[n]$ (causal alignment)  |
| `inverseFilter()`           | Read-only access to $h_{\text{inv}}[n]$           |
| `reset()`                   | Clear all state                          |

## Tuning Guide

### Choosing $\lambda$

The regularization parameter $\lambda$ is the single most important knob.  It
balances **tracking accuracy** (small $\lambda$) against **noise rejection**
(large $\lambda$).

| $\lambda$ range | Behaviour                                   | When to use          |
|-------------|---------------------------------------------|----------------------|
| `1e-10`     | Near-exact inversion; amplifies noise       | Clean signals, identity-like `h` |
| `1e-8`–`1e-6` | Good tracking with moderate noise rejection | Default starting point |
| `1e-4`–`1e-2` | Smooth, conservative input; poor tracking   | Noisy measurements, aggressive low-pass `h` |
| `> 1e-1`    | Over-smoothed; input barely tracks target   | Rarely useful        |

**Tuning procedure:**

1. Start with $\lambda = 1\text{e-}6$.
2. Record a known input `x`, convolve with `h` to get `y`, then deconvolve.
3. Compare `x_req` to `x`:
   - If `x_req` is noisy/oscillatory → increase $\lambda$ by 10×.
   - If `x_req` is too smooth (lags `x` at edges) → decrease $\lambda$ by 10×.
4. Iterate until the tracking error in the region of interest is acceptable.

### Group delay alignment

The deconvolver automatically shifts the output by the peak index of $h[n]$
to align the command causally.  If your impulse response has a non-obvious
peak (e.g., a delayed resonance), verify that `groupDelay()` returns the
expected value.  You can override the shift by post-processing the output.

### FFT size

When `padToPowerOfTwo = true` (default), the FFT size is the next power of 2
$\geq L_y + L_h - 1$.  This is optimal for most FFT backends.  Disable it only
if you need exact-length convolution and are using a backend that handles
non-power-of-2 sizes efficiently.

### Precomputation vs. one-shot

For real-time use where the same system impulse response is reused across
many target trajectories (e.g., a print job with many G-code segments):

```cpp
deconv.setImpulseResponse(h);
deconv.precomputeInverseFilter();  // O(N log N), done once
// Each deconvolve() is now a single FFT-multiply-IFFT
for (const auto& segment : segments) {
    auto x = deconv.deconvolve(segment.target);
}
```

The one-shot `deconvolve(y_tgt, h)` is convenient for testing but
recomputes the inverse filter every call.

## Limitations

- **LTI assumption**: The impulse response must be constant.  If the system
  varies with a scheduling parameter (e.g., speed-dependent viscosity), use
  one of the LPV variants instead.
- **Causality**: The shift by group delay assumes the peak of $h[n]$
  represents the dominant delay.  Systems with significant pre-ring (non-
  minimum phase) may require additional delay compensation.
- **Edge effects**: The first and last $L_h$ samples of `x_req` are less
  accurate due to the zero-padding boundary.  In practice, discard or
  weight these samples less.

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/LTIFrequencyDomainDeconvolver.hpp` | Header |
| `src/control/extrusion/LTIFrequencyDomainDeconvolver.cpp` | Implementation (uses `Eigen::FFT`) |
| `tests/control/test_deconvolution_controllers.cpp` | Unit tests |
