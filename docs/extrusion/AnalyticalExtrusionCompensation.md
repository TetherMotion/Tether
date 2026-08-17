# Analytical Extrusion Compensation

## Overview

This document describes the **analytical** (continuous-time, closed-form)
formulations of every pressure-advance and extrusion-compensation algorithm in
Tether.  Unlike the existing *sampled-space* implementations in
`include/tether/control/extrusion/`, which operate on discrete per-sample
velocity/temperature arrays, the analytical variants operate directly on the
**Weighted Switching Structure (WSS)** data structures produced by the
[`ParetoTimeEnergyOptimalVelocityPlanner`](../motion/ParetoTimeEnergyOptimal.md).
For the full WSS data-structure reference and consumer guide, see
[`docs/motion/WeightedSwitchingStructure.md`](../motion/WeightedSwitchingStructure.md).

The WSS represents the trajectory as a sequence of **analytically integrable
arcs**, each with a closed-form expression for the path velocity $v(t)$:

| Arc type   | $v(\tau)$                                         | $a(\tau)$           | $\eta(\tau)$ |
|------------|---------------------------------------------------|---------------------|--------------|
| BANG       | $v_0 + a_0\tau + \tfrac{1}{2}\eta\tau^2$          | $a_0 + \eta\tau$    | $\eta$       |
| SINGULAR   | $v_0 + a^*\tau$                                    | $a^*$               | $0$          |
| WALL       | $v_{\text{wall}}$                                  | $0$                 | $0$          |

where $\tau = t - t_0$ is the local time within the arc.

The extruder-axis velocity is proportional to the path velocity:

$$ v_e(t) = \alpha_e \cdot v(t), \qquad \alpha_e = \frac{\Delta e}{\Delta s} $$

where $\alpha_e$ is the extrusion ratio (E-distance per unit path distance),
determined from the G-code move's E and XYZ components.

Because $v(t)$ is **piecewise polynomial** (at most quadratic), every quantity
derived from it — volumetric flow $Q(t)$, pressure $P(t)$, feed-forward power
$P_{ff}(t)$, deconvolution input $x(t)$ — is also piecewise polynomial or
piecewise smooth, and can be evaluated in **closed form** at any time $t$
without sampling.

## Algorithms

The following nine analytical formulations are provided, one for each
sampled-space algorithm:

| # | Analytical class                        | Sampled-space original                  |
|---|----------------------------------------|-----------------------------------------|
| 1 | `AnalyticalLinearPressureAdvance`                   | Classic Klipper linear PressureAdvance               |
| 2 | `AnalyticalPowerLawPressureAdvance`                 | `PowerLawPressureAdvance`               |
| 3 | `AnalyticalCrossWLFPressureAdvance`                 | `CrossWlfPressureAdvance`               |
| 4 | `AnalyticalLTIDeconvolution`           | `LTIFrequencyDomainDeconvolver`         |
| 5 | `AnalyticalOverlapAddLPV`              | `OverlapAddLPVDeconvolver`              |
| 6 | `AnalyticalARXLPVInverse`              | `ARXLPVInverseFilter`                   |
| 7 | `AnalyticalStateSpaceLPV`              | `StateSpaceLPVInputEstimator`           |
| 8 | `AnalyticalFlowAdaptiveHeater`         | `FlowAdaptiveHeaterController`          |
| 9 | `AnalyticalMeltZoneThermalObserver`    | `MeltZoneThermalObserver`               |

All analytical classes live in
`include/tether/motion_planner/analytical/extrusion/` and share the namespace
`MotionPlanner::analytical::extrusion`.

---

## 1. Analytical Linear Pressure Advance

### Sampled-space form

$$ \delta e[i] = \text{PressureAdvance} \cdot v_e[i] $$

### Analytical form

$$ \delta e(t) = \text{PressureAdvance} \cdot \alpha_e \cdot v(t) $$

Since $v(t)$ is piecewise polynomial, $\delta e(t)$ is piecewise polynomial of
the same degree:

- **BANG arc:** $\delta e(\tau) = \text{PressureAdvance}\,\alpha_e\,(v_0 + a_0\tau + \tfrac{1}{2}\eta\tau^2)$
- **SINGULAR arc:** $\delta e(\tau) = \text{PressureAdvance}\,\alpha_e\,(v_0 + a^*\tau)$
- **WALL arc:** $\delta e(\tau) = \text{PressureAdvance}\,\alpha_e\,v_{\text{wall}}$

The **integrated** extruder position offset (cumulative) is:

$$ \Delta e(t) = \int_0^t \delta e(t')\,dt' $$

which is piecewise polynomial of degree one higher (cubic for BANG, quadratic
for SINGULAR, linear for WALL).

### Smoothing

The classic Klipper PressureAdvance uses a centered moving average on $v_e$.  The
analytical equivalent is a **continuous-time rectangular-window convolution**:

$$ v_{\text{smooth}}(t) = \frac{1}{T_s}\int_{t-T_s/2}^{t+T_s/2} v(\tau)\,d\tau $$

Since $v(\tau)$ is piecewise polynomial, this integral is evaluated in closed
form by splitting at arc boundaries within the window.  The result is a
piecewise polynomial of degree one higher than $v$ (cubic for BANG, quadratic
for SINGULAR).

---

## 2. Analytical Power-Law Pressure Advance

### Sampled-space form

$$ \delta e[i] = K_{\text{base}} \cdot (v_e[i] \cdot A_f)^n $$

### Analytical form

$$ \delta e(t) = K_{\text{base}} \cdot (\alpha_e \cdot A_f)^n \cdot v_{\text{smooth}}(t)^n $$

where $v_{\text{smooth}}(t)$ is the continuous-time smoothed velocity (see
§1 Smoothing) and $A_f$ is the filament cross-sectional area.

Within each arc, $v_{\text{smooth}}(\tau)$ is a known polynomial $p(\tau)$, so
$\delta e(\tau) = K' \cdot p(\tau)^n$ where $K' = K_{\text{base}} \cdot
(\alpha_e A_f)^n$.  This is a **closed-form expression** — it is evaluated
directly (no integration needed for the offset itself).

For the **cumulative** offset $\Delta e(t) = \int_0^t \delta e\,dt'$, the
integral $\int p(\tau)^n\,d\tau$ is computed analytically:

- **SINGULAR arc** ($p = v_0 + a^*\tau$, linear): $\int (v_0 + a^*\tau)^n\,d\tau$
  has the closed form $\frac{(v_0+a^*\tau)^{n+1}}{a^*(n+1)}$ for $a^* \neq 0$,
  or $v_0^n \cdot \tau$ for $a^* = 0$.

- **BANG arc** ($p = v_0 + a_0\tau + \tfrac{1}{2}\eta\tau^2$, quadratic): for
  integer $n$, $\int p^n\,d\tau$ is a polynomial of degree $2n+1$ computed by
  expansion.  For non-integer $n$, a Gauss-Jacobi quadrature of order 8
  suffices to machine precision (the integrand is smooth on each arc since
  $v > 0$).

- **WALL arc** ($p = v_{\text{wall}}$, constant): $\int v_{\text{wall}}^n\,d\tau
  = v_{\text{wall}}^n \cdot \tau$.

### Newtonian limit

When $n = 1$ and $K_{\text{base}} = \text{PressureAdvance} / A_f$, the power-law model
reduces exactly to the linear model (§1).

---

## 3. Analytical Cross-WLF Pressure Advance

### Sampled-space form

$$ \delta e[i] = \frac{\beta V_m}{A_f} \cdot P_{\text{LUT}}(Q[i], T[i]) $$

### Analytical form

$$ \delta e(t) = \frac{\beta V_m}{A_f} \cdot P_{\text{LUT}}\!\bigl(Q(t),\, T(t)\bigr) $$

where:
- $Q(t) = \alpha_e \cdot A_f \cdot v(t)$ is the **piecewise polynomial**
  volumetric flow.
- $T(t)$ is the melt temperature from the
  `AnalyticalMeltZoneThermalObserver` (§9), which is piecewise exponential
  (matrix exponential solution of the thermal ODEs).

The LUT $P_{\text{LUT}}(Q, T)$ is a **bilinear interpolation** on a 2-D grid.
Within each grid cell $(Q_j, Q_{j+1}) \times (T_k, T_{k+1})$:

$$ P = c_{00} + c_{10}\,\Delta Q + c_{01}\,\Delta T + c_{11}\,\Delta Q\,\Delta T $$

where $\Delta Q = Q - Q_j$, $\Delta T = T - T_k$, and the $c$ coefficients are
precomputed from the four corner values.

Since $Q(t)$ is piecewise polynomial and $T(t)$ is piecewise exponential,
identifying which LUT cell $(Q(t), T(t))$ falls into at time $t$ is a matter of
evaluating $Q(t)$ and $T(t)$ (both closed-form) and looking up the cell.  The
bilinear expression is then evaluated analytically.

When $Q(t)$ or $T(t)$ crosses a cell boundary within an arc, the arc is
**subdivided** at the crossing time (found by root-finding on the polynomial
$Q(t) - Q_j = 0$ or the exponential $T(t) - T_k = 0$).  Each sub-arc then has
a single bilinear expression.

---

## 4. Analytical LTI Deconvolution

### Sampled-space form

$$ X_{\text{req}}[k] = \frac{Y_{\text{tgt}}[k] \cdot H^*[k]}{|H[k]|^2 + \lambda} $$

(FFT-based regularized spectral division)

### Analytical form

The continuous-time LTI system is:

$$ y(t) = (h * x)(t) = \int_0^\infty h(\tau)\, x(t-\tau)\,d\tau $$

The **regularized inverse** in the frequency domain is:

$$ X(\omega) = \frac{Y(\omega) \cdot H^*(\omega)}{|H(\omega)|^2 + \lambda} $$

The corresponding time-domain regularized inverse impulse response is:

$$ h_{\text{inv}}(t) = \mathcal{F}^{-1}\!\left\{ \frac{H^*(\omega)}{|H(\omega)|^2 + \lambda} \right\} $$

The required input is the convolution:

$$ x(t) = (h_{\text{inv}} * y)(t) = \int_0^\infty h_{\text{inv}}(\tau)\, y(t-\tau)\,d\tau $$

Since $y(t)$ is **piecewise polynomial** (at most degree 2 from the WSS
velocity, or degree 3 from the position), the convolution integral reduces to
a finite sum of **precomputed moments** of $h_{\text{inv}}$:

$$ M_k = \int_0^\infty h_{\text{inv}}(\tau)\, \tau^k\,d\tau, \qquad k = 0, 1, \dots, K $$

where $K$ is the maximum polynomial degree of $y$ (typically $K = 3$).

Within each arc where $y(t-\tau) = \sum_{k=0}^{K} c_k(t) \cdot \tau^k$
(expressed as a polynomial in $\tau$ with coefficients depending on $t$), the
convolution is:

$$ x(t) = \sum_{k=0}^{K} c_k(t) \cdot M_k $$

This is a **closed-form linear combination** of the precomputed moments — no
integration is needed at runtime.  The moments $M_k$ are computed once at
setup time (by numerical quadrature on the precomputed $h_{\text{inv}}(t)$).

### State-space alternative

For systems given in state-space form $(A, B, C, D)$ with $D \neq 0$:

$$ x(t) = D^{-1}\bigl(y(t) - C\,v(t)\bigr) $$
$$ \dot{v}(t) = A\,v(t) + B\,x(t) = \bigl(A - B\,D^{-1}C\bigr)\,v(t) + B\,D^{-1}\,y(t) $$

This is a linear ODE with **polynomial forcing** $y(t)$.  Within each arc, the
solution is:

$$ v(t) = e^{F\,\Delta t}\,v_0 + \int_0^{\Delta t} e^{F(\Delta t - s)}\, B\,D^{-1}\, y(s)\,ds $$

where $F = A - B\,D^{-1}C$.  The integral of a matrix exponential times a
polynomial has a **closed-form** via the series:

$$ \int_0^T e^{F(T-s)}\, p(s)\,ds = \sum_{k=0}^{K} p_k \cdot \int_0^T e^{F(T-s)}\, s^k\,ds $$

Each $\int_0^T e^{F(T-s)} s^k\,ds$ is computed recursively:

$$ I_0 = F^{-1}(e^{FT} - I), \qquad I_k = F^{-1}(k\,I_{k-1} - T^k\,I) + \frac{T^{k+1}}{k+1}\,F^{-1} $$

(when $F$ is invertible; otherwise use the augmented matrix method).

With Tikhonov regularization, replace $D^{-1}$ with
$(D^T D + \lambda I)^{-1} D^T$.

---

## 5. Analytical Overlap-Add LPV Deconvolution

### Sampled-space form

Block-wise FFT deconvolution with gain-scheduled inverse filters interpolated
from a LUT of operating points.

### Analytical form

The LPV system varies with scheduling parameter $p(t) = v(t)$ (path velocity).
The gain-scheduled regularized inverse $h_{\text{inv}}(p)$ is interpolated from
the LUT.

**Within each arc**, the scheduling parameter $p(t) = v(t)$ is polynomial.
We approximate $p$ as **constant** within the arc, using the arc's
time-averaged velocity:

$$ \bar{p}_{\text{arc}} = \frac{1}{\Delta t}\int_0^{\Delta t} v(\tau)\,d\tau $$

This integral is closed-form (it is the arc-length divided by the duration).
The inverse filter $h_{\text{inv}}(\bar{p}_{\text{arc}})$ is then constant
within the arc, and the convolution reduces to the LTI case (§4):

$$ x(t) = \sum_{k=0}^{K} c_k(t) \cdot M_k(\bar{p}_{\text{arc}}) $$

where $M_k(p) = \int h_{\text{inv}}(p, \tau)\,\tau^k\,d\tau$ are precomputed
moments for each LUT entry, linearly interpolated at $\bar{p}_{\text{arc}}$.

**First-order correction** (optional): account for the variation of $p$ within
the arc by linearizing $h_{\text{inv}}$ around $\bar{p}$:

$$ h_{\text{inv}}(p(t), \tau) \approx h_{\text{inv}}(\bar{p}, \tau) + \frac{\partial h_{\text{inv}}}{\partial p}\bigg|_{\bar{p}} \cdot (p(t) - \bar{p}) $$

The correction term involves $\int h'_{\text{inv}}(\bar{p}, \tau) \cdot (p(t) -
\bar{p}) \cdot y(t-\tau)\,d\tau$, which is another moment-based integral
computable in closed form.

---

## 6. Analytical ARX LPV Inverse Filter

### Sampled-space form

$$ x[n] = \frac{1}{b_0(p[n])}\left(y[n+d] + \sum a_i(p[n])\,y[n+d-i] - \sum b_j(p[n])\,x[n-j]\right) $$

### Analytical form

The continuous-time equivalent of the ARX model is an ordinary differential
equation.  For a first-order model ($N_a = 1, N_b = 0$):

$$ \dot{y}(t) + a(p(t))\,y(t) = b(p(t))\,x(t) $$

Solving for $x(t)$:

$$ x(t) = \frac{\dot{y}(t) + a(p(t))\,y(t)}{b(p(t))} $$

Since $y(t)$ is piecewise polynomial, $\dot{y}(t)$ is piecewise polynomial (one
degree lower).  The scheduling parameter $p(t) = v(t)$ is piecewise polynomial.
The ARX coefficients $a(p), b(p)$ are interpolated from the LUT.

**Within each arc**, using the arc-averaged $\bar{p}$:

$$ x(t) = \frac{\dot{y}(t) + a(\bar{p})\,y(t)}{b(\bar{p})} $$

This is a **ratio of polynomials** — fully closed-form.

For **higher-order** models ($N_a \geq 2$), the continuous-time equivalent is:

$$ \sum_{i=0}^{N_a} a_i(p)\, y^{(i)}(t) = \sum_{j=0}^{N_b} b_j(p)\, x^{(j)}(t) $$

where $y^{(i)}$ denotes the $i$-th time derivative.  Since $y(t)$ is piecewise
polynomial of degree $K$, $y^{(i)}$ is piecewise polynomial of degree
$K - i$ (zero for $i > K$).  The equation is solved algebraically for $x(t)$
when $N_b = 0$, or as an ODE when $N_b > 0$.

### Transport delay

The discrete delay $d$ becomes a continuous-time delay $t_d = d \cdot T_s$.
The analytical form uses a **delayed target**: $y_{\text{tgt}}(t + t_d)$, which
is evaluated by looking ahead in the WSS arc list (the arcs are stored
chronologically, so looking ahead is a simple arc traversal).

---

## 7. Analytical State-Space LPV Input Estimation

### Sampled-space form

$$ x[n] = [C(p[n{+}1])\,B(p[n])]^{+}\,(y[n{+}1] - C(p[n{+}1])\,A(p[n])\,v[n]) $$

### Analytical form

The continuous-time LPV state-space model:

$$ \dot{v}(t) = A(p(t))\,v(t) + B(p(t))\,x(t) $$
$$ y(t) = C(p(t))\,v(t) + D(p(t))\,x(t) $$

For strictly proper systems ($D = 0$), differentiating the output:

$$ \dot{y}(t) = C(p)\,\dot{v}(t) + \dot{C}(p)\,\dot{p}\,v(t) \approx C(p)\,(A(p)\,v + B(p)\,x) $$

(neglecting $\dot{C}$ for slowly-varying $p$).  Solving for $x(t)$:

$$ x(t) = [C(p)\,B(p)]^{+}\,\bigl(\dot{y}(t) - C(p)\,A(p)\,v(t)\bigr) $$

with Tikhonov regularization on $[CB]^{+}$.

**Within each arc**, using arc-averaged $\bar{p}$ (so $A, B, C$ are constant):

1. Compute $\dot{y}(t)$ — piecewise polynomial (closed-form derivative of $y$).
2. Propagate the state $v(t)$ via the matrix exponential:

$$ v(t) = e^{A\,\Delta t}\,v_0 + \int_0^{\Delta t} e^{A(\Delta t - s)}\,B\,x(s)\,ds $$

Since $x(s)$ depends on $v(s)$ and $\dot{y}(s)$, this is an implicit equation.
Substituting:

$$ x(t) = M^{+}\,(\dot{y}(t) - C\,A\,v(t)), \quad M = CB $$

$$ \dot{v} = A\,v + B\,M^{+}\,(\dot{y} - C\,A\,v) = \underbrace{(A - B\,M^{+}\,C\,A)}_{F}\,v + B\,M^{+}\,\dot{y}(t) $$

This is a linear ODE with **polynomial forcing** $\dot{y}(t)$, solved via the
matrix exponential + polynomial integral formulas (same as §4 state-space
alternative).

The input is then:

$$ x(t) = M^{+}\,(\dot{y}(t) - C\,A\,v(t)) $$

All quantities are available in closed form within each arc.

---

## 8. Analytical Flow-Adaptive Heater Controller

### Sampled-space form

PID + pre/post-emphasis feed-forward based on sampled flow $Q[n]$.

### Analytical form

The feed-forward power is:

$$ P_{ff}(t) = \rho\,c_p\,Q(t)\,(T_{\text{target}} - T_{\text{inlet}}) $$

where $Q(t) = \alpha_e \cdot A_f \cdot v(t)$ is **piecewise polynomial**.
Therefore $P_{ff}(t)$ is piecewise polynomial of the same degree as $v(t)$.

**Pre-emphasis** (at flow onset): The steady-state enthalpy power is applied
before the melt zone cools.  The onset time $t_{\text{onset}}$ is identified as
the first time $Q(t)$ crosses a threshold.  The pre-emphasis power is:

$$ P_{\text{pre}}(t) = (1 - \alpha)\,P_{ff}(t_{\text{onset}}) \cdot \mathbf{1}_{[t_{\text{onset}},\, t_{\text{onset}}+\tau_{\text{pre}}]}(t) $$

where $\alpha = G_{sm}/(G_{hs} + G_{sm})$ is the sensor coupling factor and
$\tau_{\text{pre}}$ is the pre-emphasis duration.  This is a **piecewise
constant** function with analytically determined transition times.

**Post-emphasis** (after flow stops): The thermal debt $D(t)$ relaxes
exponentially:

$$ D(t) = D_0\,e^{-(t - t_{\text{stop}})/\tau_{\text{debt}}} $$

The post-emphasis power is:

$$ P_{\text{post}}(t) = (1 - \alpha)\,D(t) \cdot \mathbf{1}_{[t_{\text{stop}},\, \infty)}(t) $$

This is a **piecewise exponential** function.

The total analytical feed-forward is $P_{ff}(t) + P_{\text{pre}}(t) +
P_{\text{post}}(t)$, which is piecewise polynomial + piecewise constant +
piecewise exponential — all in closed form.

---

## 9. Analytical Melt-Zone Thermal Observer

### Sampled-space form

Forward-Euler integration of three coupled thermal ODEs.

### Analytical form

The three-state thermal model:

$$ C_h\,\dot{T}_h = P_{\text{heater}} - G_{hs}(T_h - T_s) $$
$$ C_s\,\dot{T}_s = G_{hs}(T_h - T_s) - G_{sm}(T_s - T_m) $$
$$ C_m\,\dot{T}_m = G_{sm}(T_s - T_m) - \rho\,c_p\,Q(t)\,(T_m - T_{\text{inlet}}) $$

In matrix form: $\dot{\mathbf{T}} = A_{\text{th}}\,\mathbf{T} + \mathbf{b}(t)$,
where $\mathbf{T} = [T_h, T_s, T_m]^T$.

The matrix $A_{\text{th}}$ is **constant** (thermal capacitances and
conductances don't change).  The forcing vector $\mathbf{b}(t)$ contains the
heater power and the enthalpy drain $\rho\,c_p\,Q(t)\,T_{\text{inlet}}$.

However, the enthalpy drain term $-\rho\,c_p\,Q(t)\,T_m$ makes the system
**linear time-varying** (LTV) because $Q(t)$ multiplies the state $T_m$.

**Piecewise-constant Q approximation (per arc):**

Within each arc, approximate $Q(t) \approx \bar{Q}_{\text{arc}}$ (the arc's
time-averaged flow).  The system becomes **LTI** with constant forcing:

$$ \dot{\mathbf{T}} = A_{\text{arc}}\,\mathbf{T} + \mathbf{b}_{\text{arc}} $$

where $A_{\text{arc}}$ incorporates $\bar{Q}_{\text{arc}}$ in the $(3,3)$
entry.  The solution is:

$$ \mathbf{T}(t) = e^{A_{\text{arc}}\,\Delta t}\,\mathbf{T}_0 + A_{\text{arc}}^{-1}(e^{A_{\text{arc}}\,\Delta t} - I)\,\mathbf{b}_{\text{arc}} $$

This is a **matrix exponential** — computed once per arc via Eigen's
`expm()`.  The state at the end of the arc seeds the next arc.

**Piecewise-linear Q refinement (optional):**

For higher accuracy, model $Q(t) = \bar{Q} + \dot{Q}\,\tau$ within each arc
(linear approximation).  The system becomes:

$$ \dot{\mathbf{T}} = A_0\,\mathbf{T} + \dot{Q}\,\tau\,E_{33}\,\mathbf{T} + \mathbf{b}_0 + \dot{Q}\,\tau\,\mathbf{e} $$

where $E_{33}$ is the matrix with 1 in position $(3,3)$.  This is solved by
augmenting the state with $\tau$ and $\tau\,T_m$ (state augmentation to 5
dimensions), yielding a 5×5 LTI system with constant forcing — again a matrix
exponential.

### Luenberger correction

The analytical observer includes the Luenberger correction:

$$ \mathbf{T} \mathrel{+}= L\,(T_{s,\text{measured}} - T_s)\,\Delta t $$

In the analytical version, this is applied as an **impulsive correction** at
the measurement time: $\mathbf{T}^+ = \mathbf{T}^- + L\,(T_{s,\text{meas}} -
T_s^-)\,\Delta t$.  Between measurements, the state evolves via the matrix
exponential.

---

## Implementation Notes

### Common infrastructure

All analytical extrusion classes share a common base pattern:

1. **Construction**: Takes a `WeightedSwitchingStructure<Dim, T>` (or
   `SwitchingStructureRepresentation<Dim, T>`) and model parameters.
2. **Precomputation**: At construction, precomputes per-arc coefficients,
   moments, or matrix exponentials.
3. **Evaluation**: `offsetAtTime(t)` returns the extruder position offset
   $\delta e(t)$ in closed form.  `offsetSeries(times)` evaluates at multiple
   times.  `integratedOffsetAtTime(t)` returns the cumulative offset
   $\Delta e(t)$.
4. **Arc subdivision**: Some models (Cross-WLF, LPV) subdivide arcs at
   scheduling-parameter or LUT-cell boundaries.

### Extrusion ratio

The extrusion ratio $\alpha_e = \Delta e / \Delta s$ is computed from the
G-code move that generated the path segment.  For moves with E extrusion:

$$ \alpha_e = \frac{\Delta E}{\sqrt{\Delta X^2 + \Delta Y^2 + \Delta Z^2}} $$

For travel moves (no extrusion), $\alpha_e = 0$ and all offsets are zero.

### Compatibility

The analytical classes are **additive** to the existing sampled-space
implementations — they do not replace them.  The `MotionTranslator` can use
either the sampled-space or the analytical variant, selected by configuration.

### Numerical considerations

- **Matrix exponentials**: Computed via Eigen's `unsupported/MatrixFunctions`
  module (`Eigen::MatrixBase::exp()`).
- **Polynomial integrals**: Computed via Horner-form evaluation of the
  antiderivative coefficients.
- **Root finding** (for arc subdivision): Newton's method with fallback to
  bisection, using the closed-form derivative.
- **Moment precomputation**: Gauss-Legendre quadrature of order 16 on the
  precomputed inverse impulse response.

## Source files

| File | Description |
|------|-------------|
| `include/tether/motion_planner/analytical/extrusion/AnalyticalExtrusionTypes.hpp` | Common types (extrusion arc, extrusion ratio, offset representation) |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalLinearPressureAdvance.hpp` | Analytical linear PressureAdvance |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalPowerLawPressureAdvance.hpp` | Analytical power-law PressureAdvance |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalCrossWLFPressureAdvance.hpp` | Analytical Cross-WLF PressureAdvance |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalLTIDeconvolution.hpp` | Analytical LTI deconvolution |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalOverlapAddLPV.hpp` | Analytical overlap-add LPV |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalARXLPVInverse.hpp` | Analytical ARX LPV inverse |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalStateSpaceLPV.hpp` | Analytical state-space LPV |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalFlowAdaptiveHeater.hpp` | Flow-adaptive heater, no explicit sampling |
|| `include/tether/motion_planner/analytical/extrusion/FlowAdaptiveHeaterBase.hpp` | Parameters and shared flow-adaptive heater logic |
|| `include/tether/motion_planner/analytical/extrusion/SamplingFlowAdaptiveHeater.hpp` | Flow-adaptive heater, explicit sampling reference |
| `include/tether/motion_planner/analytical/extrusion/AnalyticalMeltZoneThermalObserver.hpp` | Analytical melt-zone thermal observer |
| `tests/motion_planner/test_analytical_extrusion.cpp` | Unit tests for all analytical extrusion algorithms |
| `tests/motion_planner/test_analytical_extrusion_gcode.cpp` | G-code integration tests |
