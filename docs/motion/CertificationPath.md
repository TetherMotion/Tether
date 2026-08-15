# Certification Path of the Continuity Algorithm

This document describes the certification framework that guarantees
Tether's corner blends are geometrically continuous and stay within
tolerance. It is organized in three parts: **goals** (what we want to
prove), **how it is achieved** (the algorithmic approach), and
**mathematical derivation** (the formulas and proofs).

Code comments reference equation numbers as `(M.x)` for blend-layer
equations (defined in `BlendingAlgorithm.md`) and `(G.x)` for geometry
core equations (defined in `GeometryFoundations.md`). This document
introduces new equation numbers `(C.x)` for certification-specific
results.

---

## Part I: Goals

### Goal 1: Geometric Continuity (T1)

At each junction between path pieces, the blend curve must match the
neighboring pieces with the requested geometric continuity:

| Level | Meaning | What Matches | Blend Degree |
|---|---|---|---|
| G¹ | Continuous tangent direction | Position, unit tangent | PH quintic (5) |
| G² | Continuous curvature | Position, tangent, curvature vector | Bézier quintic (5) |
| G³ | Continuous curvature derivative | Position, tangent, curvature, jounce | Bézier septic (7) |

**Why it matters:** Discontinuous curvature (G¹-only) causes a step in
centripetal acceleration $v^2 \cdot \Delta\kappa$ at blend boundaries,
which excites mechanical vibrations. G² eliminates this step. G³
additionally eliminates the jerk step, giving the smoothest possible
motion.

### Goal 2: Tolerance Compliance (T3)

The blend curve must not deviate from the original path by more than
the specified tolerance $\delta$:

$$
d_H(B, \Omega) \leq |\delta|
$$

where $d_H$ is the Hausdorff distance (defined below) and $\Omega$ is
the original path near the corner.

**Why it matters:** The tolerance represents the maximum acceptable
deviation from the commanded path. For CNC milling, this directly
controls dimensional accuracy. For 3D printing, it controls surface
quality and dimensional tolerance.

**Critical guarantee:** The solver **never** returns a blend that
violates the tolerance. If no acceptable blend exists, it returns an
exact stop (full stop at the corner). There is no silent fallback.

### Goal 3: Certified Curvature Bounds (for velocity profiling)

The velocity profiler needs a guaranteed upper bound on curvature
$\kappa(s)$ along the path to compute the centripetal acceleration
limit $v \leq \sqrt{a_{\text{cent}} / \kappa}$. If the curvature bound
is wrong, the profiler will violate the centripetal acceleration
constraint, causing the tool to exceed the commanded acceleration
when traversing curves.

**Why it matters:** An uncertified curvature sample could miss a
curvature peak between samples, leading to excessive velocity at that
point and mechanical vibration or step loss.

---

## Part II: How It Is Achieved

### Overview

```
┌─────────────────────────────────────────────────────────────┐
│  BlendSolver (M15)                                          │
│  Bisection on speed parameters α₁, β₁                       │
│                                                             │
│  For each candidate speed:                                  │
│    1. Build blend curve (Bézier or PH quintic)              │
│    2. Coarse pre-check (16 samples)                         │
│    3. Certify deviation via DeviationCertifier (M10/M14)    │
│    4. Accept if certified upper bound ≤ |δ|                 │
│                                                             │
│  Guarantee: T3 (never returns a violating blend)            │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  DeviationCertifier (M10/M14)                               │
│  Certified Hausdorff distance via Lipschitz sampling        │
│                                                             │
│  1. Compute Lipschitz constant L for blend curve            │
│  2. Sample on grid with spacing h = 2ε/L                    │
│  3. δ ∈ [max_sample, max_sample + L·h/2]                   │
│  4. Guarantee: upper - lower ≤ ε                            │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  CertifiedCurvatureSampler                                  │
│  Certified max curvature per span via Lipschitz bounds      │
│                                                             │
│  1. Compute Lipschitz constant L_κ for dκ/ds                │
│  2. Sample on grid with spacing h = 2ε/L_κ                  │
│  3. κ_max ∈ [max_sample, max_sample + L_κ·h/2]             │
│  4. Guarantee: true κ_max ≤ certified upper bound           │
└─────────────────────────────────────────────────────────────┘
```

### The Key Idea: Lipschitz Certification

All certification in Tether relies on a single principle:

> If a function $f$ is $L$-Lipschitz (i.e., $|f(x) - f(y)| \leq L|x - y|$),
> then sampling $f$ on a grid with spacing $h$ gives a certified bound:
>
> $$\max f \in \left[\max_{\text{samples}} f,\; \max_{\text{samples}} f + \frac{L \cdot h}{2}\right]$$

The term $L \cdot h / 2$ is the **Lipschitz slack** — the maximum amount
by which the true maximum could exceed the sampled maximum. By choosing
$h$ small enough, we can make the slack arbitrarily small.

This approach is used in two places:
1. **DeviationCertifier:** $f(t) = \text{dist}(B(t), \Omega)$ is
   $L$-Lipschitz, so the Hausdorff distance is certified.
2. **CertifiedCurvatureSampler:** $\kappa(s)$ has a bounded derivative
   $\|d\kappa/ds\| \leq L_\kappa$, so the max curvature is certified.

### Why Not Exact Computation?

For Bézier curves of degree 5 (quintic) or 7 (septic), the curvature
extrema have no closed-form solution. The stationarity equation
$d\kappa/du = 0$ reduces to a polynomial of degree up to 16 (quintic)
or 24 (septic) — beyond the Abel-Ruffini barrier (no general formula
for polynomials of degree ≥ 5).

Similarly, the exact Hausdorff distance between two Bézier curves
requires solving high-degree polynomial systems. Certified sampling
avoids this by using Lipschitz bounds to bracket the answer.

### The Solver Loop (M15)

The `BlendSolver` uses bisection to find speed parameters that produce
a blend within tolerance:

1. **Initialize:** `speedLo = 1e-6 · speedHi`, `speedHi = min(maxTrimIn, maxTrimOut)`
2. **Bisection:** `speed = (speedLo + speedHi) / 2`
3. **Build:** Construct the blend curve at the current speed
4. **Pre-check:** Sample at 16 points; if the lower bound on deviation
   already exceeds $|\delta|$, skip expensive certification
5. **Certify:** Run `DeviationCertifier` to get certified deviation bounds
6. **Accept/reject:**
   - If `certificate.upper ≤ |δ|`: accept and return
   - If deviation too large: `speedHi = speed` (reduce speed)
   - If deviation too small: `speedLo = speed` (increase speed)
7. **Fallback:** If the speed range is exhausted, return `ExactStop`

The bisection converges because of Theorem T2 (see below): for any
$\delta > 0$, there exist speeds that give deviation $\leq \delta$.

---

## Part III: Mathematical Derivation

### (C.1) Hausdorff Distance

The **Hausdorff distance** between two compact sets $A, B \subset \mathbb{R}^n$ is:

$$
d_H(A, B) = \max\left\{ \sup_{a \in A} \inf_{b \in B} \|a - b\|, \; \sup_{b \in B} \inf_{a \in A} \|a - b\| \right\}
$$

It is the maximum distance from any point on one set to the nearest
point on the other set, taken in both directions. It is a true metric
on compact sets: $d_H(A, B) = 0$ iff $A = B$.

In Tether, $A = B([0,1])$ is the blend curve and $B = \Omega$ is the
original path (union of the two trimmed neighbor pieces). The
deviation of the blend from the original path is $d_H(B, \Omega)$.

### (C.2) Lipschitz Property of Distance Functions

**Claim:** The function $f(t) = \text{dist}(B(t), \Omega) = \inf_{q \in \Omega} \|B(t) - q\|$ is $L$-Lipschitz with $L = \sup_t \|B'(t)\|$.

**Proof:** For any $t_1, t_2$ and any $q \in \Omega$:

$$
\|B(t_1) - q\| \leq \|B(t_1) - B(t_2)\| + \|B(t_2) - q\|
$$

Taking the infimum over $q$ on the right:

$$
f(t_1) \leq \|B(t_1) - B(t_2)\| + f(t_2)
$$

By the mean value theorem, $\|B(t_1) - B(t_2)\| \leq L|t_1 - t_2|$ where
$L = \sup \|B'(t)\|$. Therefore:

$$
|f(t_1) - f(t_2)| \leq L |t_1 - t_2| \qquad \square
$$

**Similarly:** The function $g(q) = \text{dist}(q, B)$ is 1-Lipschitz
when $\Omega$ is parameterized by arc length, because $\|q_1 - q_2\| \leq
|s_1 - s_2|$ (arc length is an upper bound on Euclidean distance).

### (C.3) Lipschitz Constant for Bézier Curves

For a Bézier curve of degree $n$ with control points $P_0, \dots, P_n$:

$$
\|B'(t)\| = \left\| n \sum_{i=0}^{n-1} B_{i,n-1}(t) (P_{i+1} - P_i) \right\|
$$

By the triangle inequality and the partition-of-unity property of
Bernstein polynomials ($\sum B_{i,n-1}(t) = 1$, $B_{i,n-1} \geq 0$):

$$
\|B'(t)\| \leq n \cdot \max_i \|P_{i+1} - P_i\|
$$

This is the **control-polygon derivative bound** — a cornerstone of
certified geometric computing. It gives a rigorous upper bound on the
speed of the curve from the control polygon alone, without evaluating
the curve.

### (C.4) Certified Hausdorff via Lipschitz Sampling (M10/M14)

**Setup:** Sample $f(t) = \text{dist}(B(t), \Omega)$ on a uniform grid
$t_i = i/N$, $i = 0, \dots, N$, with spacing $h = 1/N$.

**Lower bound:** The true maximum is at least the sampled maximum:

$$
\delta_{\text{lower}} = \max_{i} f(t_i) \leq \sup_t f(t)
$$

**Upper bound:** By the Lipschitz property (C.2), for any $t$ there
exists a sample $t_i$ with $|t - t_i| \leq h/2$, so:

$$
f(t) \leq f(t_i) + L \cdot h/2 \leq \delta_{\text{lower}} + L \cdot h/2
$$

Therefore:

$$
\sup_t f(t) \leq \delta_{\text{lower}} + \frac{L \cdot h}{2}
$$

**Certificate interval:**

$$
\boxed{\delta \in \left[\delta_{\text{lower}}, \; \delta_{\text{lower}} + \frac{L \cdot h}{2}\right]}
$$

**Grid choice:** To achieve certificate width $\leq \varepsilon$:

$$
N = \left\lceil \frac{L}{2\varepsilon} \right\rceil, \qquad h = \frac{1}{N}
$$

**Two-sided Hausdorff:** The full Hausdorff distance requires sampling
both directions:
- $f(t) = \text{dist}(B(t), \Omega)$ on the blend (Lipschitz constant $L_B$)
- $g(s) = \text{dist}(\Omega(s), B)$ on the original path (Lipschitz constant 1)

The certified Hausdorff distance is:

$$
d_H(B, \Omega) \in \left[\max(\delta_B^{\text{lo}}, \delta_\Omega^{\text{lo}}), \; \max(\delta_B^{\text{lo}}, \delta_\Omega^{\text{lo}}) + \max\left(\frac{L_B h_B}{2}, \frac{h_\Omega}{2}\right)\right]
$$

### (C.5) Theorem T1: Geometric Continuity

**Statement:** The blend curve $B$ meets the neighbor pieces with true
$G^k$ continuity ($k = 1, 2, 3$) at the trim points.

**Proof:** The blend is constructed so that its parametric derivatives
at the endpoints match the arc-length derivatives of the neighbors:

- **Position:** $B(0) = p_A$, $B(1) = p_B$ (by construction)
- **Tangent:** $\|B'(0)\| = \alpha_1$, so $B'(0) = \alpha_1 \cdot T_A$,
  giving $dB/ds = B'(0)/\|B'(0)\| = T_A$
- **Curvature:** $B''(0) = \alpha_1^2 \cdot \vec{\kappa}_A$, so
  $d^2B/ds^2 = B''(0)/\|B'(0)\|^2 = \vec{\kappa}_A$
- **Jounce:** $B'''(0) = \alpha_1^3 \cdot \vec{\jmath}_A$, so
  $d^3B/ds^3 = B'''(0)/\|B'(0)\|^3 = \vec{\jmath}_A$

The same holds at $B(1)$ with $\beta_1$ and the outgoing piece. $\square$

**Key insight:** The speed parameters $\alpha_1, \beta_1$ serve double
duty: they control both the blend's deviation (larger speed = wider
blend = more deviation) and the parametric-to-arc-length conversion
for continuity matching.

### (C.6) Theorem T2: Tolerance Acceptance

**Statement:** For any corner with angle $\theta \in (0, \pi)$ and any
tolerance $\delta > 0$, there exist speed parameters $\alpha_1, \beta_1
> 0$ such that the quintic blend has certified Hausdorff deviation
$\leq \delta$.

**Proof sketch:** As $\alpha_1, \beta_1 \to 0$, the blend curve
collapses to the corner point $V$, and the trimmed region $\Omega$
shrinks to zero length. The Hausdorff distance $d_H(B, \Omega) \to 0$
continuously. Therefore, for any $\delta > 0$, sufficiently small
speeds give deviation $< \delta$. $\square$

**Consequence:** The bisection in `BlendSolver` is guaranteed to find
an acceptable blend (or determine that the speed range is exhausted,
in which case it returns `ExactStop`).

### (C.7) Theorem T3: Tolerance Guarantee

**Statement:** The solver returns either:
- A blend with certified Hausdorff deviation $\leq |\delta|$, or
- `ExactStop` (full stop at the corner)

It **never** returns a blend that violates the tolerance.

**Proof:** The solver only accepts a blend when
`certificate.upper ≤ |δ|`. By (C.4), the true deviation is at most
`certificate.upper`. Therefore, any accepted blend has true deviation
$\leq |\delta|$. If no blend is accepted, the solver returns
`ExactStop`. $\square$

### (C.8) Certified Curvature Sampling

**Problem:** Find a certified upper bound on $\kappa(s)$ for each span
of a NURBS piece, for use in the velocity limit curve
$v_{\text{lim}} = \sqrt{a_{\text{cent}} / \kappa}$.

**Why it's hard:** For degree-$n$ Bézier curves, curvature is:

$$
\kappa(u) = \frac{\|C'(u) \times C''(u)\|}{\|C'(u)\|^3}
$$

The stationarity equation $d\kappa/du = 0$ is a rational polynomial
equation of degree $4n - 4$ for planar curves. For quintics ($n=5$),
this is degree 16; for septics ($n=7$), degree 24. Both exceed the
Abel-Ruffini limit (no closed-form roots for degree ≥ 5).

**Approach:** Use Lipschitz certification on $\kappa(s)$.

### (C.9) Lipschitz Bound on $d\kappa/ds$

The derivative of curvature with respect to arc length is:

$$
\left\|\frac{d\kappa}{ds}\right\| \leq \frac{\|C' \times C'''\| + 3\|C'' \times C''\|}{\|C'\|^4} + \frac{3\|C' \times C''\| \cdot \|C''\|}{\|C'\|^5}
$$

This bound is derived by differentiating $\kappa = \|C' \times C''\| / \|C'\|^3$
with respect to $u$, converting $d/ds = (1/\|C'\|) d/du$, and applying
the triangle inequality.

**Control-polygon bounds** (from G.4 + triangle inequality):

$$
\begin{aligned}
\|C'(u)\| &\leq n \cdot \max_i \|P_{i+1} - P_i\| \\
\|C''(u)\| &\leq n(n-1) \cdot \max_i \|P_{i+2} - 2P_{i+1} + P_i\| \\
\|C'''(u)\| &\leq n(n-1)(n-2) \cdot \max_i \|\Delta^3 P_i\|
\end{aligned}
$$

where $\Delta^3 P_i = P_{i+3} - 3P_{i+2} + 3P_{i+1} - P_i$.

**Minimum speed:** The bounds involve $1/\|C'\|^4$ and $1/\|C'\|^5$,
which blow up if $\|C'\| \to 0$. To handle this, a coarse probe (64
samples) estimates $\sigma_{\min} = \min_u \|C'(u)\|$, and a safety
factor of 0.5 is applied: $\sigma_{\text{safe}} = 0.5 \cdot \sigma_{\min}$.

**Lipschitz constant:**

$$
L_\kappa = \frac{B_1 B_3}{\sigma_{\text{safe}}^4} + \frac{3 B_1 B_2^2}{\sigma_{\text{safe}}^5}
$$

where $B_1, B_2, B_3$ are the control-polygon bounds on $\|C'\|, \|C''\|, \|C'''\|$.

### (C.10) Certified Curvature Certificate

Sample $\kappa(s)$ on a uniform grid with spacing $h$. By the Lipschitz
property:

$$
\kappa_{\max} \in \left[\max_{\text{samples}} \kappa, \; \max_{\text{samples}} \kappa + \frac{L_\kappa \cdot h}{2}\right]
$$

**Grid choice:** $N = \lceil L_\kappa / (2\varepsilon_{\text{cert}}) \rceil$

**Guarantee:** The true maximum curvature is at most the certified
upper bound. The velocity profiler uses this upper bound to compute
$v_{\text{lim}}$, ensuring the centripetal acceleration constraint is
never violated.

**Implementation details:**
- Lazy memoization per span (only queried spans are sampled)
- Thread-safe cache with mutex
- Hard cap of 4096 samples per span (relaxes certificate if exceeded)
- For polylines (degree 1), curvature is exactly zero (no sampling)

### (C.11) PH Quintic Closed-Form Curvature (M16)

For Pythagorean-Hodograph quintic curves, the parametric speed is a
polynomial (not a square root), giving closed-form curvature:

$$
\sigma(\xi) = \|r'(\xi)\| = |\omega(\xi)|^2 = u(\xi)^2 + v(\xi)^2
$$

$$
\kappa(\xi) = \frac{2(u v' - u' v)}{\sigma(\xi)^2}
$$

This eliminates the need for certified curvature sampling on PH
spans — the exact curvature is available in closed form. The
`CertifiedCurvatureSampler` detects PH spans and uses this formula
directly.

### (C.12) Signed Split for Negative Tolerance (M20)

For negative tolerance (outside/ear blends), the deviation is split
into inside and outside components:

- **Inside component:** samples where $(p - V) \cdot c > \varepsilon$
  (forbidden interior cut)
- **Outside component:** samples where $(p - V) \cdot c < -\varepsilon$
  (ear height)

where $c = \text{normalize}(T_B - T_A)$ is the wedge bisector (not the
tangent bisector).

**Acceptance criterion:** `insideHi ≤ 10·ε AND outsideHi ≤ |δ|`

This ensures the blend does not cut into the interior beyond the
forbidden zone and the ear height stays within tolerance.

---

## Summary of Guarantees

| Guarantee | Theorem | What It Ensures |
|---|---|---|
| Geometric continuity | T1 (C.5) | Blend matches neighbors at $G^k$ level |
| Tolerance acceptance | T2 (C.6) | For any $\delta > 0$, a valid blend exists |
| Tolerance guarantee | T3 (C.7) | Solver never returns a violating blend |
| Hausdorff certification | M10/M14 (C.4) | Deviation bound is certified, not estimated |
| Curvature certification | C.10 | Max curvature bound is certified |
| PH closed-form | M16 (C.11) | Exact curvature for PH quintics |

---

## See Also

| Document | Scope |
|---|---|
| [BlendingAlgorithm.md](BlendingAlgorithm.md) | Blend construction math (M11-M20) |
| [GeometryFoundations.md](GeometryFoundations.md) | NURBS/Bézier geometry core (G.1-G.27) |
| [ToppraDerivation.md](ToppraDerivation.md) | TOPP-RA velocity profiling derivation |
| [MotionChain.md](MotionChain.md) | Full motion pipeline |
| [Architecture.md](Architecture.md) | Motion planner architecture |
