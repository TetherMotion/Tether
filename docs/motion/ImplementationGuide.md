# Implementation Guide

This document is a reading guide for the `tether::motion` blend stack.
It maps each class to the math, lists recipes for common extensions,
and documents the numerical pitfalls that have caused bugs.

---

## 1. Reading order

1. **`docs/motion/GeometryFoundations.md`** — the math foundation
   (equations G.1-G.26, proofs P0-P5). Read this first.
2. **`docs/motion/BlendingAlgorithm.md`** — the blend math
   (equations M11-M20, theorems T1-T3, lemmas L1-L2).
3. **`docs/motion/Architecture.md`** — the class hierarchy and data
   flow.
4. **`docs/motion/AlgorithmComparison.md`** — why this approach.

Then read the code in this order:

1. `include/tether/motion_planner/geometry/Vector.hpp` — `RVec`.
2. `include/tether/motion_planner/geometry/NurbsCurve.hpp` — the core
   curve type. Start with the public interface; the private helpers
   (`HomPoint`, `gaussLegendre8`) are implementation details.
3. `include/tether/motion_planner/geometry/PiecewiseNurbsPath.hpp` —
   the path type.
4. `include/tether/motion_planner/blend/CornerAnalysis.hpp` — corner
   classification.
5. `include/tether/motion_planner/blend/BoundaryConditions.hpp` —
   arc-length derivative extraction.
6. `include/tether/motion_planner/blend/BlendCurveBuilder.hpp` — the
   exact Bézier construction (M11/M12).
7. `include/tether/motion_planner/blend/DeviationCertifier.hpp` —
   the Lipschitz certificate (M10/M14).
8. `include/tether/motion_planner/blend/PHQuinticBlendBuilder.hpp` —
   the PH fast path (M16-M19).
9. `include/tether/motion_planner/blend/BlendSolver.hpp` — the
   bisection loop (M15).
10. `include/tether/motion_planner/blend/PathBlender.hpp` — the
    whole-path orchestrator (L1/L2).
11. `include/tether/motion_planner/blend/SegmentConverter.hpp` — the
    bridge from legacy `MotionSegment`.

---

## 2. Class-to-math mapping

| Class | Math | Key methods |
|---|---|---|
| `RVec` | — | `zero`, `normalized`, `dot` |
| `NurbsCurve` | G.4, G.18-G.26 | `evaluate`, `derivative`, `arcDerivatives`, `length`, `trim`, `split` |
| `PiecewiseNurbsPath` | — | `piece`, `numPieces`, `length` |
| `CornerAnalyzer` | M13 | `analyze` |
| `boundaryAt` | G.18-G.21 | free function |
| `BlendCurveBuilder` | M11, M12 | `buildQuintic`, `buildSeptic` |
| `DeviationCertifier` | M10, M14, M20 | `certify` |
| `PHQuinticBlendBuilder` | M16-M19 | `buildCandidates`, `arcLength`, `invertArcLength` |
| `BlendSolver` | M15, M20 | `solve` |
| `PathBlender` | L1, L2 | `blend` |
| `SegmentConverter` | — | `convert`, `convertAll` |

---

## 3. Recipes

### 3.1 Add a new continuity order (e.g. $G^4$)

1. Add `Continuity::G4` to `BlendSpec.hpp`.
2. In `BlendCurveBuilder`, add `buildNonic` (degree 9) implementing
   the 10-control-point closed form. The endpoint derivative
   identities (G.4) with $n=9$ give:
   - $B'(0) = 9(P_1 - P_0) = \alpha_1 T_A$
   - $B''(0) = 72(P_2 - 2P_1 + P_0) = \alpha_1^2 \vec\kappa_A$
   - $B'''(0) = 504(P_3 - 3P_2 + 3P_1 - P_0) = \alpha_1^3 \vec\jmath_A$
   - $B''''(0) = 3024(P_4 - 4P_3 + 6P_2 - 4P_1 + P_0) = \alpha_1^4 \vec{\text{snap}}_A$
   Solve for $P_0..P_4$ from the entry side, $P_5..P_9$ from the exit
   side. **Watch the sign alternation at $t=1$.**
3. Add `BoundaryConditions::snap` (the 4th arc-length derivative).
   Derive it from (G.18)-(G.21) extended to order 4.
4. In `BlendSolver::solveBezier`, handle `G4` by calling
   `buildNonic`.
5. Add a test in `BlendCurveBuilderTests.cpp` verifying $G^4$
   continuity to $10^{-7}$.
6. Update `BlendingAlgorithm.md` with the (M.x) equation number.

### 3.2 Add a new blend placement policy

The current policy is "inside cut" (positive tol) and "ear"
(negative tol, M20). To add a new policy:

1. Add the policy to `BlendSpec` (e.g. a new enum or flag).
2. In `BlendSolver::solveBezier`, branch on the policy to compute
   the augmented boundary conditions and the acceptance check.
3. In `DeviationCertifier::certify`, add the signed-split logic if
   the policy needs per-side certification.
4. Add a fuzz test case for the new policy.
5. Document the policy in `BlendingAlgorithm.md`.

### 3.3 Add a new curve type (e.g. PH septic)

1. Add `BlendCurveType::PHSeptic` to `BlendSpec.hpp`.
2. Create `PHSepticBlendBuilder` with `buildCandidates` implementing
   the degree-7 PH construction. The hodograph is the square of a
   degree-3 complex polynomial $\omega(\xi) = \sum_{k=0}^3 \omega_k
   B_{k,3}(\xi)$, giving 4 complex unknowns = 8 real unknowns. The
   Hermite data (2 positions + 2 tangents = 4 complex equations)
   leaves 4 real DOF; use them to match boundary curvature for $G^2$.
3. In `BlendSolver::solvePH`, dispatch to the new builder.
4. Add tests for the new curve type.
5. Update `BlendingAlgorithm.md` §9.

---

## 4. Numerical pitfalls

These are the bugs that have actually occurred during development.
Each is documented to prevent regression.

### 4.1 Rational derivatives — the weight chain rule

**Pitfall:** the parametric derivative of a rational NURBS is NOT
$P'(u)/w(u)$; it involves the quotient rule. The correct formulas are
(G.18)-(G.21), which compute arc-length derivatives via the
homogeneous representation.

**Symptom:** continuity tests fail at $G^2$ even though the control
points match the formula.

**Fix:** always use `arcDerivatives`, never differentiate the
control points directly.

### 4.2 Clamping acos arguments

**Pitfall:** `std::acos(T_in · T_out)` can return NaN if the dot
product is slightly outside $[-1, 1]$ due to floating-point error.

**Fix:** `std::acos(std::clamp(dot, -1.0, 1.0))`. This is done in
`CornerAnalyzer::analyze`.

### 4.3 Degenerate corner-plane basis

**Pitfall:** if $T_{\text{in}}$ and $T_{\text{out}}$ are parallel
(straight corner) or anti-parallel (cusp), the Gram-Schmidt
denominator $\sin\theta \to 0$ and the basis is undefined.

**Fix:** the classifier rejects $\theta < \text{minAngle}$ (straight)
and $\theta > \text{maxAngle}$ (cusp) before the basis is built. The
basis construction is only reached for $\theta \in [\text{minAngle},
\text{maxAngle}] \subset (0, \pi)$.

### 4.4 Near-multiple roots in point-curve distance

**Pitfall:** the point-curve distance uses Bézier decomposition +
root isolation. When the closest point is near a Bézier boundary,
the root is a near-multiple root and the solver may converge slowly
or to the wrong root.

**Fix:** the `PointCurveDistance` implementation uses subdivision
with a tolerance, and the certifier's Lipschitz bound absorbs the
slack. For the certifier, a slightly wrong closest-point distance
just makes the certificate wider (conservative), not invalid.

### 4.5 The exit-side sign alternation

**Pitfall:** the Bézier endpoint derivative at $t=1$ has alternating
signs: $B^{(k)}(1) = n(n-1)\cdots(n-k+1) \sum_{i=0}^k (-1)^i
\binom{k}{i} P_{n-i}$. Solving for $P_{n-k}$ introduces a **minus**
sign that is easy to miss.

**Symptom:** position, tangent, and curvature match; jounce
($G^3$) fails.

**Fix:** the (M12) formula for $P_4$ has a minus sign on the jounce
term. This is documented in `BlendingAlgorithm.md` §4 and tested in
`BlendCurveBuilderTests.cpp`.

### 4.6 PH candidate selection by sign convention

**Pitfall:** PH curve papers often select the "correct" candidate by
a sign convention (e.g. "choose the positive square root"). This is
fragile — the "correct" sign depends on the corner geometry and can
flip.

**Fix:** `PHQuinticBlendBuilder::buildCandidates` returns ALL 8
candidates (4 sign pairs × 2 for $\omega_1$). The solver certifies
each non-degenerate candidate and keeps the one with the smallest
certified deviation. **Never select by sign convention.**

### 4.7 Deviation certifier — removed vs kept pieces

**Pitfall:** the deviation is the Hausdorff distance from the blend
to the ORIGINAL corner path (the removed pieces from trim points to
vertex). Comparing to the KEPT pieces (the long ends) gives the
distance from the blend to the far end of the kept piece — wrong by
orders of magnitude.

**Fix:** in `BlendSolver`, the certifier is called with
`in_.trim(in_.length() - trimIn, in_.length())` (the removed entry
piece) and `out_.trim(0, trimOut)` (the removed exit piece), NOT the
kept pieces. This was a bug during development; see the commit
message for the fix.

### 4.8 NurbsCurve has no default constructor

**Pitfall:** `NurbsCurve` requires valid control points at
construction; there is no default constructor. This prevents using
`NurbsCurve` in `std::vector` resize, `std::optional` without
`make_optional`, or as a struct member that needs default
construction.

**Fix:** use `std::optional<NurbsCurve>` for members that may not
have a curve (e.g. `BlendGeometry::blendCurve`). Use
`std::vector<NurbsCurve>` with `push_back`/`emplace_back`, never
`resize`.

---

## 5. Running and extending the fuzz tests

### 5.1 Running

```bash
# All blend tests (38 tests, ~160s):
ctest --test-dir build -R motion_blend_new

# Just the fuzz tests (T3 guarantee):
ctest --test-dir build -R motion_blend_new.ToleranceFuzz

# Just the continuity property test (T1, 1000 corners):
ctest --test-dir build -R motion_blend_new.ContinuityProperty
```

### 5.2 Extending

To add a new fuzz stratum:

1. Add a new `TEST(ToleranceFuzz, YourStratum)` in
   `ToleranceFuzzTests.cpp`.
2. Use a seeded `std::mt19937_64` for reproducibility.
3. Build random corners with `RandomCorner` (or a new struct for
   your case).
4. Assert zero tolerance violations (T3).
5. Document the case count and why it's statistically significant.

To increase the case count (for a one-off stress test):

1. Edit `kNumCases` in the test.
2. Increase the ctest timeout in `tests/CMakeLists.txt`:
   `PROPERTIES TIMEOUT 600`.
3. Use a coarser `certEpsilon` (e.g. `0.1 * |tol|`) to keep the
   certifier fast — the certifier is O(N) per case with N up to 100k
   sample points.

---

## 6. Style rules

- **Namespace:** all new code in `tether::motion`.
- **No templates** in the new stack (use `double` and runtime dim).
- **No heap allocation in hot math types:** `RVec` is stack-only.
  `NurbsCurve` heap-allocates control points but is immutable after
  construction.
- **Epsilons are explicit and documented:** no `1e-9` magic numbers.
  Use named constants or comments explaining the tolerance.
- **Tests are seeded:** all randomness uses `std::mt19937_64` with a
  fixed seed.
- **Comments cite equation numbers:** every formula in code has a
  `(M.x)` or `(G.x)` comment pointing to the derivation.
- **No silent fallback:** every decision is recorded in the audit
  trail or a reason string.
