# Architecture

This document describes the architecture of the `tether::motion`
motion-planning stack: the class hierarchy, layering rules, data flow,
and conventions. It is the entry point for maintainers.

---

## 1. Layer diagram

```
┌─────────────────────────────────────────────────────────────────┐
│  Production consumers (Phase 5)                                 │
│  MotionPlan, VelocityProfile, SCurveProfile, G64CornerMode      │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│  Path layer (Phase 3)                                           │
│  PathBlender, BlendedPath, BlendSolver, BlendSpec, BlendGeometry│
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│  Blend layer (Phase 2)                                          │
│  CornerAnalysis, BoundaryConditions, BlendCurveBuilder,         │
│  DeviationCertifier, PHQuinticBlendBuilder, SegmentConverter    │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│  Geometry core (Phase 1)                                        │
│  NurbsCurve, PiecewiseNurbsPath, RVec, Bernstein,               │
│  PointCurveDistance, BoundingBox                                │
└─────────────────────────────────────────────────────────────────┘
```

**Layering rule (no cycles):** each layer may only depend on layers
below it. The geometry core knows nothing about blends or paths. The
blend layer knows about geometry but not about whole-path orchestration.
The path layer orchestrates blends but does not re-implement geometry.

---

## 2. Data flow

```
G-code → MotionSegment list (legacy DTO, MotionPlanner namespace)
    │
    │  SegmentConverter::convertAll
    ▼
std::vector<NurbsCurve>  (tether::motion, dynamic dim 1..5)
    │
    │  PiecewiseNurbsPath constructor
    ▼
PiecewiseNurbsPath  (lazy arc length, certified derivatives)
    │
    │  PathBlender::blend(spec)
    │   ├─ CornerAnalyzer::analyze (per junction)
    │   ├─ BlendSolver::solve (per corner, M15 bisection)
    │   │   ├─ BoundaryConditions::boundaryAt
    │   │   ├─ BlendCurveBuilder::buildQuintic/buildSeptic (M11/M12)
    │   │   │   OR PHQuinticBlendBuilder::buildCandidates (M16-M19)
    │   │   └─ DeviationCertifier::certify (M10/M14)
    │   └─ Overlap resolution (L1/L2)
    ▼
BlendedPath  (new piece sequence + audit trail)
    │
    │  VelocityProfile (Phase 5)
    ▼
Time-parameterized trajectory → SCurveProfile → MotionPlan::evaluateAt
```

---

## 3. Namespace and type conventions

- **All new code:** `namespace tether::motion`.
- **Legacy code:** `namespace MotionPlanner` (the old stack, to be
  deleted in Phase 4 once consumers are rewired).
- **Geometry types:** `RVec` (runtime dimension 1..5, stack-allocated,
  no heap), `NurbsCurve` (heap-allocated control points, lazy caches).
- **No templates in the new stack.** The old stack was templated
  `<Dim, T>`; the new stack uses `double` everywhere and runtime
  dimension. This reduces binary size and compile times.
- **Value types:** `BlendSpec`, `BlendGeometry`, `CornerAnalysis`,
  `BoundaryConditions`, `DeviationCertificate` are plain structs with
  no virtual functions. They are passed by const reference or moved.
- **Optional outputs:** `BlendGeometry::blendCurve` is
  `std::optional<NurbsCurve>` because `NurbsCurve` has no default
  constructor (it requires valid control points).

---

## 4. Immutability and diagnostics conventions

- **NurbsCurve** is immutable after construction. All operations
  (`trim`, `split`, `bezierDecompose`) return new curves. This makes
  aliasing safe and simplifies reasoning.
- **Lazy caches:** `NurbsCurve::length()` memoizes on first call;
  `arcLengthComputationCount()` exposes the count for tests proving
  laziness. Caches are `mutable` and NOT thread-safe (by design — the
  motion planner is single-threaded per path).
- **Diagnostics:** every geometric result carries a reason string or
  certified error bound. `BlendGeometry::reason` explains why a blend
  fell back to `ExactStop`. `DeviationCertificate` carries both
  `upper` and `lower` bounds. The audit trail (`BlendAuditEntry`)
  records every decision.
- **No silent fallback:** the solver never returns a blend that
  violates the tolerance (T3). If no acceptable blend exists, it
  returns `ExactStop` with a reason.

---

## 5. Why a compiled shared library

The motion planner is compiled into `libtether_motion_planner_shared.so`
rather than being header-only because:

1. **Compile times:** the blend layer is ~3000 lines of implementation;
  header-only would force every consumer to compile it.
2. **Binary size:** template instantiation in the old stack caused
  significant code bloat. The new stack uses runtime dimension and
  `double` only, reducing instantiation to one copy.
3. **Encapsulation:** the lazy caches and internal helpers
  (`HomPoint`, `gaussLegendre8`) are implementation details that
  should not be visible to consumers.

---

## 6. Memory model

- **RVec:** 6 doubles on the stack (48 bytes). No heap allocation.
- **NurbsCurve:** control points, weights, knots on the heap. A typical
  blend curve (degree 5, 6 control points, dim 2) is ~200 bytes +
  overhead. The `estimatedMemoryBytes()` method is available for tests.
- **PiecewiseNurbsPath:** vector of `NurbsCurve` (ownership transferred
  on construction). No shared state.
- **BlendedPath:** vector of `NurbsCurve` (the new piece sequence) +
  audit trail. The audit trail is the only "large" addition — one
  `BlendAuditEntry` per corner (~200 bytes).

**Huge-path behavior:** the path layer processes one corner at a time
and does not hold all blends in memory simultaneously during solving
(only after assembly). Memory scales as O(N) in the number of corners,
with a constant of ~500 bytes per corner. A 100k-segment path uses
~50 MB — well within budget.

---

## 7. Test organization

```
tests/motion_planner/
    geometry/         — Phase 1 geometry core tests
    blend/            — Phase 2-3 blend tests
        TestHelpers.hpp  — shared utilities (expectVecNear, etc.)
```

The blend tests are registered under the `motion_blend_new.` ctest
prefix via `gtest_discover_tests` in `tests/CMakeLists.txt`. Run them
with:

```bash
ctest --test-dir build -R motion_blend_new
```

The fuzz tests (`ToleranceFuzz`, `ContinuityProperty`) use seeded
`std::mt19937_64` for reproducibility. Epsilons in assertions are
explicit and documented (no "magic 1e-6").

---

## 8. Cross-references

| Document | Scope |
|---|---|
| `GeometryFoundations.md` | Phase 1 math (G.1-G.26, P0-P5) |
| `BlendingAlgorithm.md` | Phase 2-3 math (M11-M20, T1-T3, L1-L2) |
| `AlgorithmComparison.md` | Why-this vs rejected alternatives |
| `ImplementationGuide.md` | How to read/extend the code |
