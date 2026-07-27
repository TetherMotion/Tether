# Geometry Foundations

This document derives every formula used by the `tether::motion` geometry
core (`include/tether/motion_planner/geometry/`). Code comments cite the
equation numbers as `(G.x)`; this file is the authoritative source for
those numbers. The audience is a developer with calculus and linear
algebra but no prior CAGD background.

---

## 1. Bézier curves (G.1)–(G.6)

### (G.1) Definition

A Bézier curve of degree $n$ with control points $P_0, \dots, P_n \in
\mathbb{R}^N$ is

$$
B(t) = \sum_{i=0}^{n} \binom{n}{i} (1-t)^{n-i} t^i P_i, \qquad t \in [0,1].
$$

The basis functions $B_{i,n}(t) = \binom{n}{i}(1-t)^{n-i}t^i$ are the
**Bernstein polynomials** of degree $n$. They are non-negative on $[0,1]$
and sum to $1$ (the binomial theorem with $x=t,\; y=1-t$), so every point
of $B$ is a convex combination of the control points.

### (G.2) Endpoint interpolation

Because $B_{0,n}(0) = 1$ and $B_{i,n}(0) = 0$ for $i>0$: $B(0) = P_0$.
Symmetrically $B(1) = P_n$.

### (G.3) De Casteljau evaluation

Recursive subdivision is the numerically stable way to evaluate and
subdivide. Define $P_i^{(0)} = P_i$ and

$$
P_i^{(r)} = (1-t) P_i^{(r-1)} + t\, P_{i+1}^{(r-1)}, \qquad
r = 1, \dots, n,\; i = 0, \dots, n-r.
$$

Then $B(t) = P_0^{(n)}$. The same recurrence yields the subdivision: the
left half on $[0,t]$ has control points $P_0^{(0)}, P_0^{(1)}, \dots,
P_0^{(n)}$ and the right half on $[t,1]$ has $P_0^{(n)}, P_1^{(n-1)},
\dots, P_n^{(0)}$.

### (G.4) Endpoint derivative identities

Differentiating (G.1) termwise and using that only $B_{0,n}$ and
$B_{n,n}$ are nonzero at the endpoints, together with
$B_{i,n}'(0) = n\,[B_{i-1,n-1}(0) - B_{i,n-1}(0)]$:

$$
\begin{aligned}
B'(0)  &= n(P_1 - P_0) \\
B''(0) &= n(n-1)(P_2 - 2P_1 + P_0) \\
B'''(0)&= n(n-1)(n-2)(P_3 - 3P_2 + 3P_1 - P_0) \\
B'(1)  &= n(P_n - P_{n-1}) \\
B''(1) &= n(n-1)(P_n - 2P_{n-1} + P_{n-2}) \\
B'''(1)&= n(n-1)(n-2)(P_n - 3P_{n-1} + 3P_{n-2} - P_{n-3}).
\end{aligned}
$$

These are the identities the blend builder solves to match boundary
conditions (see `BlendingAlgorithm.md`).

### (G.5) Convex-hull property

Since the Bernstein basis is a partition of unity on $[0,1]$, $B(t)$ lies
in the convex hull of $\{P_i\}$. For a scalar polynomial $p(t) =
\sum b_i B_{i,n}(t)$ this means $p(t) \in [\min b_i, \max b_i]$ — the
coefficients bracket the values. Used by the root isolator.

### (G.6) Variation diminishing

A scalar Bernstein polynomial crosses any value $c$ at most as many times
as the coefficient sequence $\{b_i - c\}$ changes sign, and with the same
parity. In particular: no sign change $\Rightarrow$ no root in $[0,1]$;
odd sign changes $\Rightarrow$ odd number of roots. This is the basis of
certified root isolation (G.27).

---

## 2. B-splines and NURBS (G.7)–(G.16)

### (G.7) B-spline basis

A degree-$p$ B-spline with knot vector $U = \{u_0, \dots, u_m\}$ (non-
decreasing, $m = n + p + 1$ for $n+1$ control points) is

$$
C(u) = \sum_{i=0}^{n} N_{i,p}(u)\, P_i,
$$

where the basis functions are defined by the Cox–de Boor recurrence

$$
N_{i,0}(u) = [u_i \le u < u_{i+1}], \qquad
N_{i,p}(u) = \frac{u - u_i}{u_{i+p} - u_i} N_{i,p-1}(u)
           + \frac{u_{i+p+1} - u}{u_{i+p+1} - u_{i+1}} N_{i+1,p-1}(u).
$$

A **clamped** knot vector has the first and last knot each repeated $p+1$
times, so the curve interpolates the first and last control point.

### (G.8) De Boor evaluation

Evaluating $C(u)$ stably: find the knot span $k$ with $u \in [u_k,
u_{k+1})$, then run the triangular recurrence

$$
D_j^{(0)} = P_{k-p+j}, \qquad
D_j^{(r)} = (1-\alpha_r) D_{j-1}^{(r-1)} + \alpha_r D_j^{(r-1)},
$$

with $\alpha_r = (u - u_{k-p+j}) / (u_{j+1+p-r} - u_{k-p+j})$. Then
$C(u) = D_p^{(p)}$. This is the algorithm in
`NurbsCurve::evaluateHomogeneous`.

### (G.9) Homogeneous form

A NURBS curve of degree $p$ with control points $P_i$, weights $w_i > 0$,
and knots $U$ is the perspective projection of a polynomial B-spline in
homogeneous coordinates:

$$
C^w(u) = \sum_i N_{i,p}(u)\, w_i (P_i, 1) = (A(u),\, w(u)), \qquad
C(u) = A(u) / w(u).
$$

$A(u) = \sum_i N_{i,p}(u) w_i P_i$ is the (vector) homogeneous numerator
and $w(u) = \sum_i N_{i,p}(u) w_i$ is the (scalar) weight polynomial.
**This is why NURBS is the canonical representation**: lines are degree-1
NURBS with weights $1$, arcs are degree-2 rational NURBS, and B-splines
are NURBS with all weights $1$.

### (G.10) Knot insertion

Inserting a knot $u$ into a degree-$p$ curve increases the multiplicity
of $u$ by $1$; inserting it $p$ times (to full multiplicity) splits the
curve at $u$ into two curves whose union is point-for-point identical to
the original. The new control points are computed in closed form by the
Oslo / Böhm algorithm. The junction control point is exactly $C(u)$.
This makes arc-length trimming **exact**: invert $s^* \to u^*$ (G.26),
then split at $u^*$.

### (G.11) B-spline basis derivatives

The $k$-th derivative of the basis is (Piegl & Tiller, *The NURBS Book*,
Alg. A2.3):

$$
N_{i,p}^{(k)}(u) = \frac{p}{p-1}\Big( \cdots \Big),
$$

computed by the triangular `ndu`/`ders` recurrence in
`NurbsCurve::homogeneousDerivatives`. The $k$-th derivative of the
homogeneous curve is $C^{w\,(k)}(u) = \sum_i N_{i,p}^{(k)}(u) w_i (P_i,
1) = (A^{(k)}(u), w^{(k)}(u))$.

### (G.12) De Boor on homogeneous control points

To evaluate $C(u)$: run De Boor (G.8) on the homogeneous control points
$w_i (P_i, 1)$ to obtain $(A(u), w(u))$, then divide: $C(u) = A(u) /
w(u)$.

### (G.13) Bézier decomposition

Inserting every interior knot to full multiplicity produces a sequence
of single-span Bézier pieces, each with knots $\{a \times (p+1),\, b
\times (p+1)\}$ and $p+1$ control points. The union is point-for-point
identical to the original NURBS. This is the bridge between the NURBS
world and the Bézier world used by the certifier (G.28) and the distance
solver (G.29).

### (G.14)–(G.16) Rational derivatives (quotient rule)

With $C = A/w$ and the polynomial derivatives $A', A'', A'''$ and $w',
w'', w'''$ from (G.11):

$$
\begin{aligned}
\text{(G.14)}\quad C'  &= \frac{A' - w' C}{w} \\
\text{(G.15)}\quad C'' &= \frac{A'' - 2 w' C' - w'' C}{w} \\
\text{(G.16)}\quad C'''&= \frac{A''' - 3 w' C'' - 3 w'' C' - w''' C}{w}.
\end{aligned}
$$

**Critical pitfall:** the derivative of a rational curve is *not* the
derivative of $A$ divided by $w$ — the quotient rule mixes lower-order
terms. In particular, a degree-2 rational curve has $A''' = 0$ and
$w''' = 0$ but $C''' = -(3 w' C'' + 3 w'' C')/w \ne 0$ in general. The
implementation must not short-circuit orders above the degree for
rational curves.

---

## 3. Arc-length derivatives in $\mathbb{R}^N$ (G.17)–(G.21)

Let $v = C'(u)$, $a = C''(u)$, $j = C'''(u)$, and $s_1 = \|v\| = ds/du$.
Since $d/ds = (1/s_1)\, d/du$:

### (G.17) Parametric speed
$$
\sigma(u) = \frac{ds}{du} = \|C'(u)\| = s_1.
$$

### (G.18) Unit tangent
$$
T = \frac{dp}{ds} = \frac{v}{s_1}.
$$

### (G.19) Curvature vector
$$
\kappa = \frac{d^2p}{ds^2} = \frac{dT}{ds}
       = \frac{1}{s_1} \frac{d}{du}\!\left(\frac{v}{s_1}\right)
       = \frac{a\, s_1^2 - v\,(v \cdot a)}{s_1^4}
       = \frac{a}{s_1^2} - \frac{v\,(v \cdot a)}{s_1^4}.
$$

Derivation: $d(v/s_1)/du = (a s_1 - v\, s_1')/s_1^2$ with $s_1' = (v
\cdot a)/s_1$, giving $(a s_1^2 - v(v \cdot a))/s_1^3$; divide by $s_1$
for $d/ds$.

### (G.20) Jounce vector
$$
j = \frac{d^3p}{ds^3} = \frac{d\kappa}{ds}
  = \frac{1}{s_1}\, \frac{d\kappa}{du}
  = \frac{j}{s_1^3}
    - \frac{3 a\,(v \cdot a)}{s_1^5}
    - \frac{v\,(\|a\|^2 + v \cdot j)}{s_1^5}
    + \frac{4 v\,(v \cdot a)^2}{s_1^7}.
$$

Derivation: differentiate (G.19) by the product/quotient rule. The five
terms come from differentiating $a/s_1^2$ (two terms: $j/s_1^2$ and
$-2 a\, s_1'/s_1^4 = -2 a (v \cdot a)/s_1^5$) and $-v(v \cdot a)/s_1^4$
(three terms: $-a(v\cdot a)/s_1^4$, $-v(\|a\|^2 + v\cdot j)/s_1^4$, and
$+4 v (v\cdot a)^2/s_1^6$); collect the $1/s_1^5$ and $1/s_1^7$ pieces
after the final $1/s_1$ factor.

**Sanity checks (used in tests):** for a circle of radius $R$
parameterized by arc length, $\|\kappa\| = 1/R$, $\|j\| = 1/R^2$,
$\kappa \cdot T = 0$, $j \cdot \kappa = 0$, and $j = -T/R^2$.

### (G.21) Degenerate parameterization guard
When $s_1 \approx 0$ (cusp, duplicate control points) the formulas above
divide by zero. The implementation detects this by comparing $s_1$ to a
scale-relative threshold ($10^{-14}$ times the control-polygon length)
and throws `std::domain_error` — never silently returns NaN.

---

## 4. Arc length and its inverse (G.22)–(G.26)

### (G.22) No closed form in general
The arc length of a polynomial or rational curve of degree $\ge 2$ is

$$
L = \int_{u_{\min}}^{u_{\max}} \|C'(u)\|\, du
  = \int \sqrt{C'_1(u)^2 + \cdots + C'_N(u)^2}\, du,
$$

and the square root of a sum of squares of polynomials has no elementary
antiderivative in general. (Lines and circles are the exceptions: lines
have constant speed, and the rational-quadratic circle has speed
proportional to $1/(1 + t^2)$ which integrates to an arctangent.)

### (G.23) Adaptive Gauss–Legendre quadrature
On each knot span $[u_k, u_{k+1}]$ we approximate the integral by the
8-point Gauss–Legendre rule (4 positive nodes, mirrored). The error is
estimated by comparing the 8-point result on $[a,b]$ to the sum of two
8-point results on $[a, (a+b)/2]$ and $[(a+b)/2, b]$; if the difference
exceeds a tolerance proportional to the control-polygon scale, the span
is recursively subdivided. This is `NurbsCurve::quadratureRecursive`.

### (G.24) Per-span memoization
Each span's length is computed at most once and cached in a `mutable`
per-curve table. `PiecewiseNurbsPath` extends this to a prefix-sum
watermark: piece $i$'s length is computed only when a query needs
$\text{prefix}[i]$, and pieces past the queried index are never touched
(see §6).

### (G.25) Exact polyline length
For degree-1 curves the speed is piecewise constant on each knot span and
the length is the sum of control-polygon edge lengths weighted by the
knot intervals — closed form, no quadrature, and
`arcLengthComputationCount()` stays $0$.

### (G.26) Arc-length inversion $s \to u$
Because $s(u) = \int \|C'\|$ is strictly monotone ($\|C'\| > 0$ on a
non-degenerate curve), $s(u)$ is invertible. We solve $f(u) = s(u) -
s_{\text{target}} = 0$ by Newton–Raphson with $f'(u) = \|C'(u)\|$,
bracketed by bisection on $[u_{\min}, u_{\max}]$ (Newton steps that
leave the bracket are replaced by bisection steps). Convergence is
quadratic near the root and the bisection fallback guarantees
termination in $\le 60$ iterations. This is
`NurbsCurve::invertLength`.

---

## 5. Certified point–curve distance (G.27)–(G.30)

### (G.27) Bernstein root isolation
A scalar polynomial $p(t) = \sum_{i=0}^m b_i B_{i,m}(t)$ on $[0,1]$ is
isolated by recursive subdivision driven by (G.5)–(G.6):

1. If all $b_i$ have the same sign (no sign change), $p$ has no root in
   $[0,1]$ — discard the interval.
2. If the interval width is $< 2\varepsilon$, report it as a root (or
   root cluster — near-multiple roots may merge into one interval;
   documented behavior).
3. Otherwise subdivide at $t = 1/2$ by De Casteljau (G.3) and recurse on
   both halves.

Every root of $p$ in $[0,1]$ lies inside one of the returned disjoint
intervals; each interval has width $< 2\varepsilon$. This is
`bernstein::isolateRoots`. Roots are then polished by a few Newton steps
on the derivative polynomial to pin them down to machine precision.

### (G.28) Bézier decomposition brings any NURBS into Bézier form
The distance algorithm works per Bézier span. `NurbsCurve::bezierDecompose`
returns the single-span pieces (G.13); the global minimum is the min over
spans.

### (G.29) The distance polynomial
On one Bézier span write the rational curve as $C(t) = A(t)/w(t)$ with
$A(t) = \sum w_i P_i B_{i,n}(t)$ the homogeneous numerator (vector, one
polynomial per coordinate) and $w(t) = \sum w_i B_{i,n}(t)$ the scalar
weight. The squared distance to a fixed point $p$ is

$$
D(t) = \left\| \frac{A(t)}{w(t)} - p \right\|^2.
$$

Differentiating and clearing the (positive) denominator $w^3$:

$$
\text{(G.30)}\quad
N(t) = \sum_d \bigl(A_d(t) - p_d\, w(t)\bigr)\,
            \bigl(A_d'(t)\, w(t) - A_d(t)\, w'(t)\bigr),
$$

so $D'(t) = 2 N(t) / w(t)^3$. Since $w > 0$ on the span, the stationary
points of $D$ are exactly the roots of the **polynomial** $N$. Convert
$N$ to Bernstein basis (G.5), isolate all roots (G.27), evaluate $D$ at
every root and at both endpoints, and take the minimum. Because every
stationary point is found, the result is the **certified global minimum**
— no sampling, no missed minima.

**Pitfall:** the factors must use the homogeneous numerator $A_d = w
\cdot C_d$, not the rational control points $C_d$. Using $C_d$ directly
gives a different polynomial whose roots are *not* the stationary points
of $D$.

---

## 6. Piecewise path and laziness (G.31)–(G.33)

### (G.31) Prefix-sum watermark
`PiecewiseNurbsPath` stores pieces plus a prefix array
$\text{prefix}[i] = \sum_{j < i} \text{length}(\text{piece}_j)$. A
watermark `computed_` tracks the highest index whose length has been
computed; entries past the watermark are not yet valid.

### (G.32) Lazy forward scan for `locate(s)`
For a query at arc length $s$ when the total is not yet known, the path
walks forward from `computed_`, computing one piece length at a time
until $\text{prefix}[k+1] > s$. Only pieces $[0..k]$ are touched — a
query near the start of a $10^6$-piece path is $O(1)$, not $O(n)$. Once
the total is known (all pieces computed), subsequent queries use binary
search over the full prefix array.

### (G.33) Huge-path memory
Each `NurbsCurve` stores only its control points, weights, and knots —
no sample tables. A $10^6$-piece path of line segments occupies
$\sim 10^6 \times (\text{2 RVec} + \text{4 doubles}) \approx 80$ MB;
nothing is pre-sampled. `estimatedMemoryBytes()` exposes this to the
huge-path test.

---

## 7. Proof P0 — exact circular arcs as rational quadratics

**Claim.** An arc of radius $R$ sweeping angle $\varphi \le \pi$ is
represented exactly by a degree-2 NURBS with three control points
$P_0, P_1, P_2$ and weights $w_0 = w_2 = 1$, $w_1 = \cos(\varphi/2)$,
knots $\{0,0,0,1,1,1\}$.

**Setup.** Place $P_0$ and $P_2$ on the circle at the arc endpoints, and
$P_1$ at the intersection of the endpoint tangents (the "shoulder"
point). With the arc centered at the origin in the $(e_1, e_2)$ plane and
spanning $[-\varphi/2, +\varphi/2]$:

$$
P_0 = R(\cos\tfrac{\varphi}{2},\, -\sin\tfrac{\varphi}{2}), \quad
P_2 = R(\cos\tfrac{\varphi}{2},\, +\sin\tfrac{\varphi}{2}), \quad
P_1 = (R/\cos\tfrac{\varphi}{2},\, 0).
$$

**Proof sketch.** The rational quadratic is $C(t) = \frac{(1-t)^2 w_0
P_0 + 2t(1-t) w_1 P_1 + t^2 w_2 P_2}{(1-t)^2 w_0 + 2t(1-t) w_1 + t^2
w_2}$. Substitute $t = \tan(\theta/2)$ (the standard rational
reparameterization of the circle) and use $w_1 = \cos(\varphi/2)$; after
expanding and collecting, $\|C(t)\|^2 = R^2$ identically. The middle
weight $\cos(\varphi/2)$ is exactly what makes the shoulder point lie on
the tangent intersection *and* the curve lie on the circle. Full detail
in Piegl & Tiller, *The NURBS Book*, §1.4.

**Span limit.** At $\varphi = \pi$ the middle weight is $\cos(\pi/2) =
0$ — numerically degenerate. We therefore split arcs into spans of
$\le 120°$ so that $w_1 \ge \cos(60°) = 0.5$ stays safely positive. The
spans lie on the same circle, so they meet with $G^1$ continuity (same
tangent direction and curvature) even though the full-multiplicity
internal knots make the parameterization $C^0$ at the junctions.

---

## 8. Cross-references

| Code symbol | Equation |
|---|---|
| `bernstein::evaluate` | (G.3) |
| `bernstein::derivative` | (G.4) |
| `bernstein::isolateRoots` | (G.6), (G.27) |
| `NurbsCurve::evaluateHomogeneous` | (G.8), (G.12) |
| `NurbsCurve::homogeneousDerivatives` | (G.11) |
| `NurbsCurve::derivative` | (G.14)–(G.16) |
| `NurbsCurve::arcDerivatives` | (G.17)–(G.21) |
| `NurbsCurve::fromArc` | Proof P0 |
| `NurbsCurve::split` / `bezierDecompose` | (G.10), (G.13) |
| `NurbsCurve::quadratureRecursive` | (G.23) |
| `NurbsCurve::invertLength` | (G.26) |
| `pointCurveDistance` | (G.28)–(G.30) |
| `PiecewiseNurbsPath::locate` | (G.31)–(G.32) |
