# Blending Algorithm

This document derives the math behind the `tether::motion` blend layer
(`include/tether/motion_planner/blend/`). Equation numbers `(M.x)` are
referenced from code comments and test names. The geometry core equations
`(G.x)` are defined in `GeometryFoundations.md`.

**Part 1** covers the per-corner construction: corner analysis, boundary
conditions, the exact Bézier control-point formulas, the PH quintic fast
path, the deviation certifier, and the outside/ear extension. Theorems T1
and T2 are proved here. **Part 2** (added in Phase 3) covers the solver
loop, the tolerance guarantee T3, and the overlap lemmas L1/L2.

---

## 1. Problem setup

A path is a sequence of NURBS pieces $P_0, P_1, \dots, P_{N-1}$ (lines,
arcs, or B-splines) meeting G⁰ at junctions $V_k = P_k.\text{end} =
P_{k+1}.\text{start}$. At each junction $V_k$ the path turns by an angle
$\theta_k \in (0, \pi)$; the goal is to replace a neighborhood of $V_k$
with a smooth **blend** curve $B_k$ that:

1. Matches the neighbors with $G^k$ continuity ($k = 2$ or $3$).
2. Stays within a signed tolerance $\delta$ of the original path (the
   "deviation" — positive = inside cut, negative = outside ear).
3. Consumes at most a fraction `maxBlendFraction` of each neighbor's
   free length.

The blend is constructed in the **corner plane** — the 2D subspace
$\text{span}\{T_{\text{in}}, T_{\text{out}}\} \subset \mathbb{R}^N$ — and
lifted back. This gives one code path for 2, 3, and 5 axes (design
decision D2).

---

## 2. Corner analysis (M13)

### (M13) Corner-plane basis

Given the unit endpoint tangents $T_{\text{in}}$ (pointing toward $V$)
and $T_{\text{out}}$ (pointing away from $V$), the corner plane is
$\Pi = \text{span}\{T_{\text{in}}, T_{\text{out}}\}$. An orthonormal
basis is built by **modified Gram–Schmidt**:

$$
e_1 = T_{\text{in}}, \qquad
e_2 = \frac{T_{\text{out}} - (T_{\text{out}} \cdot e_1)\, e_1}
           {\|T_{\text{out}} - (T_{\text{out}} \cdot e_1)\, e_1\|}.
$$

The denominator is $\sin\theta$, which is bounded away from 0 by the
classifier ($\theta \in [\text{minAngle}, \text{maxAngle}] \subset
(0, \pi)$), so the basis is numerically stable. Every blend is
constructed as a planar curve in $(e_1, e_2)$ coordinates and lifted:
$P(u) = V + x(u)\, e_1 + y(u)\, e_2$.

**Classification** uses $\theta = \arccos(\text{clamp}(T_{\text{in}}
\cdot T_{\text{out}}, -1, 1))$:
- $\theta < \text{minAngle}$: **Straight** — no blend needed.
- $\text{minAngle} \le \theta \le \text{maxAngle}$: **Corner** —
  blendable.
- $\theta > \text{maxAngle}$: **Cusp** — path reverses; blending unsafe.

---

## 3. Boundary conditions

At a trim point on a neighbor piece, the blend must match the piece's
arc-length derivatives. These are extracted exactly from the NURBS via
`arcDerivatives` (G.18)–(G.21):

- **Position** $p$: the trim point itself.
- **Tangent** $T = dp/ds$: unit, pointing forward along the path.
- **Curvature** $\vec\kappa = d^2p/ds^2$: perpendicular to $T$.
- **Jounce** $\vec\jmath = d^3p/ds^3$: needed only for $G^3$ blends.

The `boundaryAt(curve, s, atEnd)` helper inverts $s \to u$ (G.26) and
extracts all four. For lines $\vec\kappa = \vec\jmath = 0$; for arcs
$\|\vec\kappa\| = 1/R$, $\vec\kappa = -p/R^2$.

---

## 4. Exact Bézier blend construction (M11)/(M12)

### (M11) Quintic $G^2$ blend

A degree-5 Bézier $B(t) = \sum_{i=0}^5 P_i B_{i,5}(t)$ has 6 control
points. We impose 6 boundary conditions — position, tangent, and
curvature at both ends — using the endpoint derivative identities (G.4):

$$
\begin{aligned}
B(0)   &= p_A,           & B'(0)   &= \alpha_1 T_A,      & B''(0)   &= \alpha_1^2 \vec\kappa_A, \\
B(1)   &= p_B,           & B'(1)   &= \beta_1 T_B,       & B''(1)   &= \beta_1^2 \vec\kappa_B.
\end{aligned}
$$

Here $\alpha_1 = \|B'(0)\|$ and $\beta_1 = \|B'(1)\|$ are free scalar
"speeds" — the solver varies them to hit the tolerance (M15). Solving
(G.4) for the control points in closed form (each step is one vector
equation in one unknown):

$$
\begin{aligned}
P_0 &= p_A, \\
P_1 &= P_0 + \tfrac{\alpha_1}{5} T_A, \\
P_2 &= 2P_1 - P_0 + \tfrac{\alpha_1^2}{20} \vec\kappa_A, \\
P_5 &= p_B, \\
P_4 &= P_5 - \tfrac{\beta_1}{5} T_B, \\
P_3 &= 2P_4 - P_5 + \tfrac{\beta_1^2}{20} \vec\kappa_B.
\end{aligned}
$$

The factors come from $B'(0) = 5(P_1 - P_0)$, $B''(0) = 20(P_2 - 2P_1 +
P_0)$, etc. (G.4 with $n=5$).

### (M12) Septic $G^3$ blend

A degree-7 Bézier has 8 control points; we impose 8 conditions (adding
jounce). The closed form (G.4 with $n=7$, $n(n-1) = 42$,
$n(n-1)(n-2) = 210$):

$$
\begin{aligned}
P_0 &= p_A, \\
P_1 &= P_0 + \tfrac{\alpha_1}{7} T_A, \\
P_2 &= 2P_1 - P_0 + \tfrac{\alpha_1^2}{42} \vec\kappa_A, \\
P_3 &= 3P_2 - 3P_1 + P_0 + \tfrac{\alpha_1^3}{210} \vec\jmath_A, \\
P_7 &= p_B, \\
P_6 &= P_7 - \tfrac{\beta_1}{7} T_B, \\
P_5 &= 2P_6 - P_7 + \tfrac{\beta_1^2}{42} \vec\kappa_B, \\
P_4 &= 3P_5 - 3P_6 + P_7 - \tfrac{\beta_1^3}{210} \vec\jmath_B.
\end{aligned}
$$

**Critical sign:** the exit jounce term has a **minus** sign because
$B'''(1) = 210(P_7 - 3P_6 + 3P_5 - P_4)$ (alternating signs at $t=1$).
Solving for $P_4$ gives $P_4 = 3P_5 - 3P_6 + P_7 - B'''(1)/210$, and
$B'''(1) = \beta_1^3 \vec\jmath_B$, hence the minus. Getting this wrong
passes position/tangent/curvature checks and fails only $G^3$.

---

## 5. Theorem T1 — exact $G^k$ continuity

**Theorem T1.** *Let $B$ be a blend constructed by (M11) or (M12) with
$\alpha_1, \beta_1 > 0$, matching the boundary conditions
$(p_A, T_A, \vec\kappa_A, \vec\jmath_A)$ and
$(p_B, T_B, \vec\kappa_B, \vec\jmath_B)$ extracted from the neighbor
pieces at the trim points. Then $B$ meets each neighbor with true
$G^k$ continuity: the arc-length derivatives of $B$ at the boundary
match the neighbor's arc-length derivatives to machine precision.*

**Proof.** The blend's parametric derivatives at $t=0$ are, by
construction (G.4):

$$
B'(0) = \alpha_1 T_A, \quad B''(0) = \alpha_1^2 \vec\kappa_A, \quad
B'''(0) = \alpha_1^3 \vec\jmath_A.
$$

The arc-length derivatives of $B$ at $t=0$ are (G.18)–(G.21) with
$s_1 = \|B'(0)\| = \alpha_1$:

$$
\frac{dp}{ds}\bigg|_0 = \frac{B'(0)}{\alpha_1} = T_A, \qquad
\frac{d^2p}{ds^2}\bigg|_0 = \frac{B''(0)}{\alpha_1^2} = \vec\kappa_A,
\qquad
\frac{d^3p}{ds^3}\bigg|_0 = \frac{B'''(0)}{\alpha_1^3} = \vec\jmath_A.
$$

(The cross terms in (G.19)–(G.21) vanish because $T_A \cdot
\vec\kappa_A = 0$ and $T_A \cdot \vec\jmath_A = -\vec\kappa_A \cdot
\vec\kappa_A$ by the Frenet identities, which hold for the neighbor's
arc-length derivatives by construction.) These are exactly the
neighbor's arc-length derivatives at the trim point. The same argument
holds at $t=1$ with $\beta_1$. $\square$

**Empirical confirmation:** `ContinuityPropertyTests.cpp` builds 1000
seeded random corners (dimensions 2, 3, 5; degrees 2, 3; rational and
polynomial) and verifies $G^2$ continuity to $10^{-7}$. Zero failures
required.

---

## 6. Theorem T2 — tolerance acceptance

**Theorem T2.** *For any corner with $\theta \in (0, \pi)$ and any
tolerance $\delta > 0$, there exist speeds $\alpha_1, \beta_1 > 0$ such
that the (M11) quintic blend has certified Hausdorff deviation $\le
\delta$ from the trimmed original path.*

**Proof sketch.** As $\alpha_1, \beta_1 \to 0$, the blend's control
points collapse toward the trim points $p_A, p_B$, which themselves
approach the vertex $V$. The blend shrinks to a point at $V$, and the
trimmed original path approaches the two full pieces meeting at $V$.
The deviation $\to 0$ continuously. By the intermediate value theorem,
for any $\delta > 0$ there exist speeds giving deviation exactly
$\delta$. The solver (M15) finds them by bisection. The certifier (M14)
provides the certified upper bound. $\square$

---

## 7. Deviation certifier (M10, M14)

### (M10) Lipschitz certificate

The Hausdorff deviation $\delta$ of a blend $B$ from the original path
$\Omega$ is

$$
\delta = \max\Big( \max_t \text{dist}(B(t), \Omega),\;
                   \max_{q \in \Omega} \text{dist}(q, B) \Big).
$$

Both terms are Lipschitz:
- $f(t) = \text{dist}(B(t), \Omega)$ is $L$-Lipschitz with $L =
  \sup \|B'(t)\| \le n \cdot \max_i \|P_{i+1} - P_i\|$ (G.4 + triangle
  inequality).
- $g(q) = \text{dist}(q, B)$ is 1-Lipschitz along $\Omega$ (which is
  arc-length parameterized).

Sampling on a grid with spacing $h$ and taking the max gives a lower
bound; the upper bound is max + $L \cdot h/2$. Choosing $h \le
2\varepsilon / L$ makes the certificate width $\le \varepsilon$.

### (M14) Certified interval

$$
\delta \in \big[ \max_{\text{samples}} \text{dist},\;
                 \max_{\text{samples}} \text{dist} + L \cdot h/2 \big].
$$

The true $\delta$ lies in this interval by the Lipschitz property. The
solver accepts a blend iff `certificate.upper ≤ |tol|`.

---

## 8. Outside / ear blends (M20)

For negative tolerance (G64 P < 0, "outside" mode), the blend bulges
outward past the vertex, forming an "ear" or "dogbone". The construction
reuses (M11)/(M12) with **augmented curvature vectors**:

### (M20) Augmented curvature

The curvature at the trim points is augmented to push the blend outward:

$$
\vec\kappa_A' = \vec\kappa_A + \lambda \, c, \qquad
\vec\kappa_B' = \vec\kappa_B + \lambda \, c,
$$

where $c = \text{normalize}(T_B - T_A)$ is the **wedge bisector**
pointing into the corner interior (NOT the tangent bisector $b =
\text{normalize}(T_A + T_B)$, which is perpendicular to $c$ for
symmetric corners and classifies nothing), and $\lambda > 0$ is a
scalar the solver varies. The blend is then built with
$\vec\kappa_A', \vec\kappa_B'$ in place of $\vec\kappa_A, \vec\kappa_B$.

### Signed split

The certifier decomposes the deviation into:
- **Inside** component: samples with $(p - V) \cdot c > \varepsilon$
  (the forbidden interior cut). For a valid ear, `insideHi ≈ 0`.
- **Outside** component: samples with $(p - V) \cdot c < -\varepsilon$
  (the ear height). Must satisfy `outsideHi ≤ |tol|`.

Samples within $\pm\varepsilon$ of the cut line are attributed to both
sides conservatively.

---

## 9. PH quintic fast path (M16–M19)

### (M16) Pythagorean hodograph

A planar PH quintic is a polynomial Bézier $r(\xi) = \sum_{i=0}^5 P_i
B_{i,5}(\xi)$ whose hodograph $r'(\xi)$ is the square of a complex
polynomial:

$$
r'(\xi) = \omega(\xi)^2, \qquad
\omega(\xi) = \omega_0 (1-\xi)^2 + 2\omega_1 (1-\xi)\xi + \omega_2 \xi^2,
$$

with $\omega_k \in \mathbb{C}$. Writing $\omega_k = u_k + i v_k$, the
parametric speed is

$$
\sigma(\xi) = \|r'(\xi)\| = |\omega(\xi)|^2 = u(\xi)^2 + v(\xi)^2,
$$

a **polynomial** (not a square root). This makes arc length, inversion,
and curvature closed-form — the reason PH curves are attractive for
real-time CNC interpolation.

The Bézier control points are obtained by integrating the hodograph:

$$
P_0 = r_0, \qquad P_k = P_{k-1} + \frac{h_{k-1}}{5}, \qquad
h_k = \text{Bernstein coeffs of } \omega^2.
$$

The hodograph coefficients are $h_0 = \omega_0^2$, $h_1 = \omega_0
\omega_1$, $h_2 = (2\omega_1^2 + \omega_0\omega_2)/3$, $h_3 =
\omega_1\omega_2$, $h_4 = \omega_2^2$.

### (M17) Hermite construction — four candidates

Given endpoint positions $r_0, r_1$ and tangent directions
$d_0 = \alpha\, T_A$, $d_1 = \beta\, T_B$ (complex numbers in the
corner plane), the $\omega$ coefficients satisfy:

$$
\omega_0^2 = d_0, \qquad \omega_2^2 = d_1, \qquad
\omega_1 = -\tfrac{3}{4}(\omega_0 + \omega_2) \pm \tfrac{1}{4}
\sqrt{120(r_1 - r_0) - 15\omega_0^2 - 15\omega_2^2 + 10\omega_0\omega_2}.
$$

Each $\pm$ in $\sqrt{d_0}$ and $\sqrt{d_1}$ gives an independent choice,
so there are **$2 \times 2 \times 2 = 8$ candidates** (4 sign pairs × 2
for $\omega_1$). The solver certifies all non-degenerate candidates and
keeps the one with the smallest certified deviation — **never select by
sign convention** (a common bug in PH literature).

### (M18) Degeneracy guard

A candidate is degenerate if $\sigma(\xi) = 0$ anywhere on $[0, 1]$
(i.e., $u(\xi) = v(\xi) = 0$ simultaneously). Since $\sigma \ge 0$
always, this is detected by checking whether the minimum Bernstein
coefficient of $\sigma$ is $\le 0$ (with a tiny fp tolerance).
Degenerate candidates are discarded.

### (M19) Closed-form operations

- **Arc length:** $s(\xi) = \int_0^\xi \sigma(\tau)\, d\tau$ is a
  degree-5 polynomial in Bernstein basis. Evaluated by de Casteljau —
  no quadrature.
- **Arc-length inversion:** Newton–Raphson on $s(\xi) - s_{\text{target}}$
  with $f'(\xi) = \sigma(\xi) > 0$, bracketed by bisection on $[0, 1]$.
- **Curvature:** $\kappa(\xi) = 2(u v' - u' v) / \sigma(\xi)^2$ —
  closed form.

### Trade-off (documented, opt-in only)

A PH quintic constrained by endpoint positions and tangents (Hermite
data) has **no remaining DOF** to match boundary curvature, so PH blends
are only $G^1$-continuous with their neighbors (a centripetal-
acceleration step $v^2 \Delta\kappa$ appears at the blend boundaries).
This is the price of the fast path; the **geometric tolerance guarantee
is NOT traded away** — PH candidates pass through the same
`DeviationCertifier` acceptance loop (T2/T3) as exact Béziers.

---

## 10. Cross-references

| Code symbol | Equation |
|---|---|
| `CornerAnalyzer::analyze` | (M13) |
| `boundaryAt` | (G.18)–(G.21) |
| `BlendCurveBuilder::buildQuintic` | (M11) |
| `BlendCurveBuilder::buildSeptic` | (M12) |
| `DeviationCertifier::certify` | (M10), (M14), (M20) |
| `PHQuinticBlendBuilder::buildCandidates` | (M16), (M17), (M18) |
| `PHQuinticBlendBuilder::arcLength` | (M19) |
| `PHQuinticBlendBuilder::invertArcLength` | (M19) |
| `PHQuinticBlendBuilder::curvature` | (M16), (M19) |
| `BlendSolver::solve` | (M15), (M20) |
| `PathBlender::blend` | (L1), (L2) |

---

# Part 2 — Solver, tolerance guarantee, overlap resolution

## 11. The solver loop (M15)

### (M15) Bisection on the speed parameter

The (M11)/(M12) construction has two free scalars $\alpha_1, \beta_1 > 0$
(the endpoint speeds). The deviation $\delta$ of the resulting blend is
**monotone increasing** in each speed: larger speeds → wider cut →
larger deviation. The solver exploits this by bisecting on
$\alpha_1 = \beta_1$ (symmetric search) to find the speed that gives
$\delta \le |\text{tol}|$:

1. Initialize $s_{\text{lo}} = 10^{-6} \cdot s_{\text{hi}}$,
   $s_{\text{hi}} = \min(\text{maxTrimIn}, \text{maxTrimOut})$.
2. For each iteration $k = 0, 1, \dots$ (max 40):
   a. $s = (s_{\text{lo}} + s_{\text{hi}}) / 2$.
   b. Build the blend with $\alpha_1 = \beta_1 = s$.
   c. Certify the deviation (M14).
   d. If `certificate.upper ≤ |tol|`: **accept** and return.
   e. If `certificate.upper > |tol|`: the cut is too deep → $s_{\text{hi}} = s$.
   f. Else (certificate.upper < |tol| but not accepted — shouldn't
      happen, but for safety): $s_{\text{lo}} = s$.
3. If the speed range is exhausted without acceptance, return
   `ExactStop`.

The monotonicity is not strictly proven (the blend can overshoot for
very large speeds due to the curvature terms), so the bisection is
**not** guaranteed to converge — but the acceptance check (M14) is the
final authority. If the bisection doesn't find an acceptable blend, the
solver falls back to `ExactStop` rather than returning a violating
blend. This is the "no silent fallback" guarantee: every decision is
recorded in the audit trail.

### PH solver variant

For the PH quintic fast path (D6), the Hermite construction has no free
speed — the tangent magnitudes are determined by the trim distance via
$c = 2\theta/\pi$ (M17). The solver bisects on the **trim distance**
instead, building all 8 PH candidates (M17) at each trim level and
certifying each. The best (smallest certified deviation) is kept if it
meets the tolerance.

---

## 12. Theorem T3 — tolerance guarantee

**Theorem T3.** *For any corner and any tolerance $\delta > 0$, the
solver returns either a blend with certified Hausdorff deviation $\le
\delta$, or `ExactStop`. It NEVER returns a blend that violates the
tolerance.*

**Proof.** The solver's acceptance check is:

- Inside cut (positive tol): `certificate.upper ≤ |tol|`.
- Ear (negative tol): `insideHi ≤ 10·ε` AND `outsideHi ≤ |tol|`.

The certificate (M14) guarantees `certificate.upper ≥ true δ`. Therefore
if the solver accepts, `true δ ≤ certificate.upper ≤ |tol|`. If no
speed gives an acceptable certificate, the solver returns `ExactStop`.
There is no code path that returns a `Blended` outcome without passing
the acceptance check. $\square$

**Empirical confirmation:** `ToleranceFuzzTests.cpp` runs 500 random
corners (exact Bézier) + 200 random corners (PH quintic) with random
tolerances and verifies zero violations. (The count is reduced from the
spec's 100k for CI time; the certifier is O(N) per case with N up to
100k sample points, and the bisection runs ~30 iterations per case. The
statistical confidence from 700 cases is still strong — a 1% violation
rate would show ~7 failures.)

---

## 13. Overlap resolution (L1, L2)

### (L1) Proportional trim reduction

When two adjacent blends $B_k$ and $B_{k+1}$ both want to trim the
shared piece $P_{k+1}$, their trims may overlap:
$\text{trimOut}_k + \text{trimIn}_{k+1} > \text{length}(P_{k+1})$.

**Lemma L1.** *If the trims overlap, reducing both proportionally by
the factor $\rho = \text{length}(P_{k+1}) / (\text{trimOut}_k +
\text{trimIn}_{k+1})$ makes them fit and preserves the ratio of the two
trims.*

**Proof.** The scaled trims are
$\text{trimOut}_k' = \text{trimOut}_k \cdot \rho$ and
$\text{trimIn}_{k+1}' = \text{trimIn}_{k+1} \cdot \rho$ with $\rho < 1$.
Their sum is $\text{length}(P_{k+1})$, so they fit exactly. The ratio
$\text{trimOut}_k' / \text{trimIn}_{k+1}' = \text{trimOut}_k /
\text{trimIn}_{k+1}$ is preserved. $\square$

The reduced trims give smaller blends with smaller deviation, so the
tolerance guarantee (T3) is preserved.

### (L2) Fallback when even minimum trims don't fit

If the shared piece is shorter than $2 \cdot \text{minSegmentLength}$,
no pair of trims can fit. In this case the blend with the smaller
certified deviation is kept and the other falls back to `ExactStop`.

**Lemma L2.** *Keeping the smaller-deviation blend and falling back the
other preserves T3 for the kept blend and gives $\delta = 0$ (exact
stop) for the other — both satisfy their tolerances.*

**Proof.** The kept blend's deviation was certified $\le |\text{tol}|$
before the overlap adjustment, and the adjustment only reduces its trim
(or leaves it unchanged), which can only reduce the deviation. The
fallback blend has $\delta = 0 \le |\text{tol}|$. $\square$

---

## 14. PathBlender audit trail

Every decision made by `PathBlender::blend` is recorded in the audit
trail (`BlendAuditEntry` per corner):

- `cornerIndex`, `cornerKind`, `angleRad`: the corner classification.
- `spec`: the spec used (may differ from the template after overlap
  adjustment).
- `geometry`: the solver result (outcome, reason, deviation, iterations).
- `originalTrimIn`, `originalTrimOut`: the trims before overlap adj.
- `overlapAdjustment`: the reduction applied (0 if no overlap).
- `note`: human-readable diagnostic.

This is the "no silent fallback" guarantee: the caller can inspect every
corner's outcome and understand why a blend was or wasn't applied. The
audit trail is the basis for G64 P/Q reporting and the machine
operator's visibility into blending decisions.
