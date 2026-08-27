# Weighted Switching Structure (WSS) Reference

`WeightedSwitchingStructure<Dim, T>` is the analytic source representation
produced by `ParetoTimeEnergyOptimalVelocityPlanner`. See
[SnapSpace Time–Energy Profile Planning](ParetoTimeEnergyOptimal.md) for the
planner contract and limitations.

## Source-of-truth rule

The WSS arc list is the trajectory. Sampling, inversion, and the recorded cost
must all describe that same list. WSS accessors do **not** clamp velocity,
acceleration, or jerk after evaluating an arc. If an arc violates a bound, that
is a solver defect or an infeasible request; it must not be hidden at read time.

## State and arc records

The fourth-order scalar state is

$$
x=(s,v,a,j),\qquad \dot{s}=v,\quad\dot{v}=a,\quad
\dot{a}=j,\quad\dot{j}=\sigma.
$$

Each `WeightedArc` contains:

| Field | Meaning |
|---|---|
| `type` | `SNAP_PLUS`, `SNAP_MINUS`, `SINGULAR`, `WALL`, or `DWELL` |
| `s0`, `s1`, `t0`, `duration` | arc interval |
| `v0`, `a0`, `j0` | exact initial state |
| `v1`, `a1`, `j1` | exact terminal state stored by the solver |
| `sigma` | constant snap on a SNAP arc |
| `j_star` | constant jerk on a SINGULAR arc |
| `activeConstraints` | saturated bound mask: velocity `1`, acceleration `2`, jerk `4`, snap `8` |

The normal SnapSpace pulse solver emits SNAP and SINGULAR arcs. `DWELL` is
composed by the planner for path dwell points. `WALL` remains supported for
legacy sources but is not emitted by the current solver.

## Closed-form propagation

For a SNAP arc, with local time $\tau$,

$$
\begin{aligned}
j&=j_0+\sigma\tau,\\
a&=a_0+j_0\tau+\tfrac12\sigma\tau^2,\\
v&=v_0+a_0\tau+\tfrac12j_0\tau^2+\tfrac16\sigma\tau^3,\\
s&=s_0+v_0\tau+\tfrac12a_0\tau^2+
\tfrac16j_0\tau^3+\tfrac1{24}\sigma\tau^4.
\end{aligned}
$$

For a SINGULAR arc, jerk is constant: $j=j_*$ and $\sigma=0$.

`startState()` reads the first arc's initial state; `endState()` reads the last
arc's stored terminal state. These are the endpoint values to validate rather
than values inferred from a sampled table.

## Query semantics

- `arcLength(t)`, `pathVelocity(t)`, `pathAcceleration(t)`, and `pathJerk(t)`
  use the stored formulas.
- `timeAtArcLength(s)` finds the containing arc then bisects only in its
  recorded interval $[0,\text{duration}]$. This avoids selecting a physically
  unrelated root of a cubic or quartic deceleration polynomial.
- `toVelocityProfile(n)` creates a compatibility sampled representation. Use
  the WSS directly when exact derivatives are needed.
- `derivativeOrder()` is `Snap`; consumers can use `hasJerk()` and `hasSnap()`
  through the `VelocityProfile` wrapper.

At a repeated dwell coordinate, the sampled `VelocityProfile` uses a
right-continuous convention: querying the coordinate yields the state at the
end of that dwell.

## Ownership and lifetime

The WSS holds a `shared_ptr<const Path>`. It is safe for the source to outlive
the planner or the temporary path used during construction. This differs from
the legacy third-order SSR representation, whose owner must still call
`setPath()` after moving a motion plan's path.

## Validation checklist

A producer or consumer test should validate:

1. every moving arc has positive duration;
2. independent propagation reaches `s1`, `v1`, `a1`, and `j1`;
3. neighboring arcs are continuous in all four state components;
4. $s(t)$ is monotone and time/arc-length round trips agree;
5. extrema inside each arc satisfy the asserted bounds;
6. the closed-form cost agrees with numerical quadrature.

These checks are implemented for the standard pulse-family scenarios in
`SnapSpaceContractTest.cpp`.
