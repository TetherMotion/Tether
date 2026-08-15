# LPV Deconvolution Controllers

## Overview

When the system impulse response varies with a scheduling parameter `p[n]`
(e.g., toolhead speed affecting polymer viscosity), the LTI assumption
breaks.  Tether provides three LPV (Linear Parameter-Varying) deconvolution
variants, each suited to a different compute platform and causality
requirement:

| Class                          | Domain    | Best for                  | Lookahead? |
|--------------------------------|-----------|---------------------------|------------|
| `OverlapAddLPVDeconvolver`     | Frequency | Host-side planning        | Non-causal |
| `ARXLPVInverseFilter`          | Time      | Bare-metal MCU streaming  | d steps    |
| `StateSpaceLPVInputEstimator`  | Time      | Embedded with Eigen       | 1 step     |

All three share the same pattern: a **LUT of models** identified at M
operating points, with **linear interpolation** of coefficients at runtime.

---

## 1. Overlap-Add LPV Deconvolver (Gain-Scheduled, Pseudo-Frequency Domain)

### When to use

- Host-side trajectory planning (Klipper host process)
- Non-causal lookahead is available (full trajectory known in advance)
- Moderate compute budget (FFT per block is acceptable)

### Algorithm

1. **LUT generation**: At M operating points `p_m`, measure `h_m[n]` and
   precompute the regularized inverse `h_inv_m[n]` via
   `LTIFrequencyDomainDeconvolver`.
2. **Windowing**: Segment `y_tgt[n]` into overlapping blocks of length `B`
   (default 50% overlap), multiplied by a Hann window.
3. **Block processing**: For each block `i`:
   - Compute the average scheduling parameter `p̄_i`.
   - Linearly interpolate `h_inv(p̄_i)` from the two closest LUT entries.
   - Time-domain convolution of the windowed block with `h_inv(p̄_i)`.
4. **Overlap-add**: Sum the block outputs at their time offsets.

### API

```cpp
#include "tether/control/extrusion/OverlapAddLPVDeconvolver.hpp"

using namespace tether::control::extrusion;

OverlapAddLPVParams params;
params.blockSize = 256;
params.overlapRatio = 0.5;
params.lambda = 1e-6;
OverlapAddLPVDeconvolver lpv(params);

// Build the LUT
lpv.addOperatingPoint(20.0, h_slow);   // h measured at speed=20
lpv.addOperatingPoint(50.0, h_mid);
lpv.addOperatingPoint(100.0, h_fast);

// Deconvolve with a scheduling-parameter trajectory
auto x_req = lpv.deconvolve(y_tgt, p_trajectory);
```

### Parameters

| Parameter        | Type   | Default | Description                              |
|------------------|--------|---------|------------------------------------------|
| `blockSize`      | int    | `256`   | Block length B (samples)                 |
| `overlapRatio`   | double | `0.5`   | Overlap fraction in [0, 1)               |
| `lambda`         | double | `1e-6`  | Tikhonov λ for inverse filter precompute |

### Tuning guide

#### Block size B

| B range     | Effect                                        |
|-------------|-----------------------------------------------|
| 64–128      | Fast parameter tracking; more windowing artifacts |
| 256 (default) | Good balance for most print trajectories    |
| 512–1024    | Smooth parameter interpolation; less edge artifacts; slower |

**Rule of thumb**: Choose B such that the scheduling parameter changes by
less than ~10% within a single block.  For a 1 kHz loop and a print speed
ramp from 20 to 100 mm/s over 2 seconds, B = 256 (0.256 s) gives a
~10 mm/s change per block — acceptable.

#### Overlap ratio

- **0.5 (default)**: Hann window with 50% overlap gives perfect
  reconstruction (constant overlap-add gain = 1.0).  Recommended.
- **0.75**: More overlap → smoother transitions but 2× the compute.
- **0.0**: No overlap → block boundary artifacts (not recommended).

#### Lambda

Same tuning as the LTI deconvolver (see
[DeconvolutionControllers.md](DeconvolutionControllers.md)).  The same `λ`
is used for all LUT entries.  If different operating points need different
regularization, precompute inverse filters externally with per-point `λ`
and add them via `addInverseFilter()`.

#### Number of operating points M

| M  | When                                          |
|----|-----------------------------------------------|
| 1  | System is nearly LTI (use LTI deconvolver instead) |
| 2  | Linear parameter dependence                   |
| 3–5 | Typical for speed-dependent viscosity        |
| >10 | Highly nonlinear parameter dependence        |

Operating points should span the full range of `p` with roughly equal
spacing.  Extrapolation outside the LUT range clamps to the nearest entry.

---

## 2. ARX LPV Inverse Filter (Time-Domain IIR)

### When to use

- Bare-metal MCU processing step-by-step streams
- High loop frequency (e.g., 1 kHz)
- Minimal compute budget (no FFT, no matrix library)
- Causal with known transport delay

### Algorithm

The system is modelled as a parameter-varying ARX transfer function:

```
A(z, p) y[n] = z^{-d} B'(z, p) x[n]
```

where:
- `A(z, p) = 1 + a_1(p) z^{-1} + ... + a_{Na}(p) z^{-Na}`
- `B'(z, p) = b_0(p) + b_1(p) z^{-1} + ... + b_{Nb}(p) z^{-Nb}`
- `d` = discrete transport delay (steps)

The inverse (deconvolution) solves for `x[n]` algebraically:

```
x_req[n] = (1 / b_0(p[n])) · ( y_tgt[n+d]
          + Σ_{i=1}^{Na} a_i(p[n]) · y_tgt[n+d-i]
          − Σ_{j=1}^{Nb} b_j(p[n]) · x_req[n-j] )
```

The delay `d` is explicitly factored out so that `b_0` is the first non-zero
coefficient.  This avoids the instability trap of non-minimum-phase zeros.

### API

```cpp
#include "tether/control/extrusion/ARXLPVInverseFilter.hpp"

using namespace tether::control::extrusion;

// Construct with polynomial orders
ARXLPVInverseFilter filter(/*na=*/1, /*nb=*/1);

// Add identified ARX models at operating points
// A(z) = 1 + a1*z^-1, B'(z) = b0 + b1*z^-1, delay = 1
filter.addModelPoint(20.0, {-0.3}, {0.7, 0.1}, 1);
filter.addModelPoint(100.0, {-0.5}, {0.9, 0.05}, 1);

// Process a full trajectory
auto x_req = filter.process(y_tgt, p_trajectory);

// Or process one sample at a time (streaming)
double x = filter.process(y_tgt[n], y_tgt[n+d], p[n]);
```

### ARXLPVModelPoint fields

| Field       | Type                 | Description                    |
|-------------|----------------------|--------------------------------|
| `parameter` | double               | Scheduling parameter `p`       |
| `aCoeffs`   | `std::vector<double>`| AR coefficients `[a1, a2, ...]`|
| `bCoeffs`   | `std::vector<double>`| B' coefficients `[b0, b1, ...]`|
| `delay`     | int                  | Transport delay `d` (steps)    |

### Tuning guide

#### System identification (offline)

Use ARX least-squares identification on step-response data collected at
each operating point:

```
% MATLAB / Octave example
data = iddata(y, x, Ts);
model = arx(data, [na, nb, nk]);  % nk = delay+1
a = model.a(2:end);  % drop leading 1
b = model.b(nk+1:end);
d = nk - 1;
```

Or use Python's `statsmodels.tsa.ar_model` or `scipy.signal.dlsim`.

#### Delay estimation

The transport delay `d` is critical for stability:

| Symptom                        | Cause                    | Fix                |
|--------------------------------|--------------------------|--------------------|
| Explosive oscillation in `x_req` | `d` too small (b_0 ≈ 0) | Increase `d`       |
| Excessive lag in tracking       | `d` too large           | Decrease `d`       |
| Stable but poor tracking        | `d` correct, λ tuning   | Tune model orders  |

**Procedure**: Start with `d = round(measured_delay_seconds × loop_rate_Hz)`.
Verify that `b_0` is not near zero.  If `|b_0| < 0.01`, increase `d` by 1
and re-identify.

#### Model orders (Na, Nb)

| Order | Effect                                        |
|-------|-----------------------------------------------|
| Na=1, Nb=0 | First-order system, minimal compute    |
| Na=2, Nb=1 | Second-order with one zero (typical)   |
| Na=3, Nb=2 | Higher-order dynamics, resonance       |

**Rule of thumb**: Start with Na=2, Nb=1.  Increase if the residual
`(y_tgt - simulated_y)` is too large.  Decrease if the filter is unstable
or noisy.

#### Coefficient interpolation

Coefficients are linearly interpolated between LUT entries.  This works
well when the parameter dependence is smooth.  If the system has a sharp
transition (e.g., onset of melt flow), add more operating points near the
transition.

#### Stability check

After building the LUT, verify stability at several interpolated points:

```cpp
for (double p = p_min; p <= p_max; p += step) {
    filter.reset();
    auto x = filter.process(unit_step, constant_p);
    // Check that x does not diverge
    ASSERT_TRUE(std::all_of(x.begin(), x.end(),
        [](double v) { return std::abs(v) < 1e3; }));
}
```

---

## 3. State-Space LPV Input Estimator

### When to use

- Embedded systems with matrix algebra support (Eigen)
- Most mathematically rigorous time-domain approach
- Handles MIMO systems (multiple inputs/outputs)
- One-step-ahead lookahead available

### Algorithm

The LPV system is represented in state-space form:

```
v[n+1] = A(p[n]) v[n] + B(p[n]) x[n]
y[n]   = C(p[n]) v[n] + D(p[n]) x[n]
```

For strictly proper systems (D = 0, relative degree d = 1), the input is
recovered by looking one step ahead:

```
x_req[n] = [C(p[n+1]) B(p[n])]^{+} (y_tgt[n+1] - C(p[n+1]) A(p[n]) v[n])
```

The pseudo-inverse `[C·B]^{+}` is computed with **Tikhonov regularization**:

```
x = (M^T M + λI)^{-1} M^T b
```

For SISO systems (scalar M), this reduces to `x = M·b / (M² + λ)`.

The internal state `v[n]` is propagated in a feedforward simulation:

```
v[n+1] = A(p[n]) v[n] + B(p[n]) x_req[n]
```

### API

```cpp
#include "tether/control/extrusion/StateSpaceLPVInputEstimator.hpp"

using namespace tether::control::extrusion;

// Construct with system dimensions
StateSpaceLPVParams params;
params.lambda = 1e-8;
StateSpaceLPVInputEstimator estimator(/*stateDim=*/2, /*inputDim=*/1,
                                       /*outputDim=*/1, params);

// Add state-space models at operating points
Eigen::MatrixXd A(2,2), B(2,1), C(1,2);
A << 0.8, 0.1, 0.0, 0.9;
B << 1.0, 0.0;
C << 1.0, 0.0;
estimator.addModelPoint({20.0, A, B, C});

// Process a full trajectory
auto x_req = estimator.process(y_tgt, p_trajectory);

// Or process one step at a time
double x = estimator.process(y_tgt[n+1], p[n], p[n+1]);
```

### StateSpaceLPVModelPoint fields

| Field       | Type              | Description                    |
|-------------|-------------------|--------------------------------|
| `parameter` | double            | Scheduling parameter `p`       |
| `A`         | `Eigen::MatrixXd` | State transition [stateDim × stateDim] |
| `B`         | `Eigen::MatrixXd` | Input matrix [stateDim × inputDim] |
| `C`         | `Eigen::MatrixXd` | Output matrix [outputDim × stateDim] |
| `D`         | `Eigen::MatrixXd` | Feedthrough (usually 0)        |

### Parameters

| Parameter | Type   | Default | Description                              |
|-----------|--------|---------|------------------------------------------|
| `lambda`  | double | `1e-8`  | Tikhonov λ for matrix pseudo-inverse     |

### Tuning guide

#### State dimension

The state dimension determines how many internal dynamics the model can
capture:

| stateDim | Models                          |
|----------|---------------------------------|
| 1        | First-order (single time constant) |
| 2        | Second-order (resonance, two time constants) |
| 3–4      | Higher-order dynamics            |

**Rule of thumb**: Start with `stateDim = 2`.  Use ERA (Eigensystem
Realization Algorithm) or subspace identification to determine the minimal
realization order from impulse-response data.

#### Lambda (Tikhonov regularization)

The `λ` parameter regularizes the matrix pseudo-inverse `[C·B]^{+}`:

| λ range     | Effect                                        |
|-------------|-----------------------------------------------|
| `1e-12`     | Near-exact inversion; may amplify noise       |
| `1e-8` (default) | Good balance for well-conditioned systems |
| `1e-4`      | Conservative; smooths input at cost of tracking |
| `1e-2`      | Very conservative; use only for ill-conditioned systems |

**When to increase λ**:
- `C·B` is near-singular (small `|C·B|` relative to other terms)
- The input `x_req` shows high-frequency oscillation
- The system has a near-non-minimum-phase zero

**Procedure**:
1. Start with `λ = 1e-8`.
2. Process a known target and check the tracking error.
3. If `x_req` oscillates, increase `λ` by 100× and retry.
4. If tracking error is too large, decrease `λ` by 100×.

#### Matrix interpolation

Matrices `A`, `B`, `C`, `D` are linearly interpolated element-wise between
LUT entries.  This preserves the state-space structure as long as the
interpolated matrices remain stable (eigenvalues of `A` inside the unit
circle).

**Stability check after interpolation**:

```cpp
auto model = estimator.interpolateModel(p);
Eigen::EigenSolver<Eigen::MatrixXd> es(model.A);
for (int i = 0; i < es.eigenvalues().size(); ++i) {
    assert(std::abs(es.eigenvalues()[i]) < 1.0);
}
```

If interpolated matrices become unstable, add more operating points or
switch to a log-interpolation scheme for the eigenvalues.

#### Relative degree > 1

The current implementation assumes relative degree `d = 1` (input affects
output on the next step).  For higher relative degree, chain multiple
lookahead steps:

```cpp
// For d = 2: look two steps ahead
double x = estimator.process(y_tgt[n+2], p[n], p[n+2]);
```

This requires modifying the internal state propagation to account for the
extra delay.  Future versions may support arbitrary relative degree
natively.

#### MIMO systems

For multi-input, multi-output systems, set `inputDim > 1` and
`outputDim > 1`.  The Tikhonov-regularized solve handles the general
rectangular case:

```
x = (M^T M + λI)^{-1} M^T b
```

where `M = C·B` is `[outputDim × inputDim]`.  The regularization term `λI`
ensures invertibility even when `M` is not square.

---

## Comparison Summary

| Feature                | OverlapAddLPV      | ARXLPV             | StateSpaceLPV      |
|------------------------|--------------------|--------------------|--------------------|
| Domain                 | Frequency (FFT)    | Time (IIR)         | Time (state-space) |
| Compute per sample     | O(B log B) per block | O(Na + Nb)       | O(stateDim²)       |
| Lookahead              | Full block         | d steps            | 1 step             |
| Memory                 | O(M·B)             | O(M·(Na+Nb))      | O(M·stateDim²)     |
| Matrix library needed  | No (Eigen::FFT)    | No                 | Yes (Eigen)        |
| MIMO support           | No                 | No                 | Yes                |
| Delay handling         | Implicit (in h)    | Explicit (d param) | Implicit (in A,B)  |
| Regularization         | Tikhonov (λ)       | N/A (algebraic)    | Tikhonov (λ)       |
| Best platform          | Host CPU           | Bare-metal MCU     | Embedded with Eigen|

## Source files

| File | Description |
|------|-------------|
| `include/tether/control/extrusion/OverlapAddLPVDeconvolver.hpp` | Overlap-add LPV header |
| `src/control/extrusion/OverlapAddLPVDeconvolver.cpp` | Overlap-add LPV implementation |
| `include/tether/control/extrusion/ARXLPVInverseFilter.hpp` | ARX LPV header |
| `src/control/extrusion/ARXLPVInverseFilter.cpp` | ARX LPV implementation |
| `include/tether/control/extrusion/StateSpaceLPVInputEstimator.hpp` | State-space LPV header |
| `src/control/extrusion/StateSpaceLPVInputEstimator.cpp` | State-space LPV implementation |
| `tests/control/test_deconvolution_controllers.cpp` | Unit tests for all four deconvolvers |
