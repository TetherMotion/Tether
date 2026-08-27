# SnapSpace Velocity Profiler

## Implemented model

The SnapSpace profiler uses the fourth-order scalar path state

$$
(s,v,a,j),\qquad \dot{s}=v,\quad \dot{v}=a,\quad
\dot{a}=j,\quad \dot{j}=\sigma.
$$

`ParetoTimeEnergyOptimalVelocityPlanner` constrains snap $\sigma$, and its WSS
output exposes velocity, acceleration, jerk, and snap-order provenance. Its
endpoint contract is

$$
(0,v_0,0,0) \longrightarrow (L,v_f,0,0).
$$

Nonzero initial acceleration or jerk is rejected because the generic profiler
interface has no matching final $a$/$j$ endpoint parameters. The solver must
not silently reset a requested derivative.

## Solver contract

The solver is deterministic and exact within its chosen trajectory family:

1. It samples the path and constructs a conservative global speed,
   acceleration, and jerk envelope.
2. It creates exact symmetric SNAP/SINGULAR acceleration and braking pulses,
   adding a constant-velocity SINGULAR cruise if necessary.
3. It evaluates 41 fixed smoothness scales from $1$ down to $0.01$.
4. It selects the feasible candidate with lowest closed-form cost

   $$
   J=\int_0^T(w_t+w_jj^2+w_aa^2)\,dt.
   $$

The trajectory used for cost, WSS storage, and all queries is identical.
Sampling never clamps a stored state. A failed feasibility condition returns no
profile and provides `failureReason()`.

This is a **restricted feasible-family optimiser**, not a proof of globally
Pareto-optimal control for arbitrary position-varying walls. The conservative
global envelope can be slower than a future local viable-set optimiser. It was
chosen over the historical heuristic because it has a clear, testable
feasibility and endpoint contract.

## Exact primitive formulas

For an arc of duration $\tau$ with constant snap $\sigma$:

$$
\begin{aligned}
j(\tau)&=j_0+\sigma\tau,\\
a(\tau)&=a_0+j_0\tau+\tfrac12\sigma\tau^2,\\
v(\tau)&=v_0+a_0\tau+\tfrac12j_0\tau^2+\tfrac16\sigma\tau^3,\\
\Delta s(\tau)&=v_0\tau+\tfrac12a_0\tau^2+
\tfrac16j_0\tau^3+\tfrac1{24}\sigma\tau^4.
\end{aligned}
$$

A SINGULAR arc has $\sigma=0$ and constant jerk $j=j_*$. Inversion only uses
the known recorded interval for an arc. The primitive root helpers return NaN
when a requested distance lies beyond the first forward velocity stop, rather
than choosing a later nonphysical polynomial root.

## Constraint scope

Path-level velocity, acceleration, jerk, and snap values are enforced. Axis
velocity, acceleration, and jerk authority is included through
`ConstraintEvaluator` when deriving the conservative envelope.

Per-axis snap limits are intentionally unsupported. Cartesian snap contains a
term involving the fourth arc-length derivative of the path:

$$
q^{(4)}=p^{(4)}v^4+6p^{(3)}v^2a+3p''a^2+4p''vj+p'\sigma.
$$

The available geometry contract ends at order three, so enabling an axis snap
limit is rejected with a diagnostic rather than yielding a false certificate.

## WSS and compatibility sampling

`WeightedSwitchingStructure` stores exact terminal state in every arc (`v1`,
`a1`, `j1`) and provides `startState()`/`endState()`. Its `activeConstraints`
mask uses velocity `1`, acceleration `2`, jerk `4`, and snap `8`.

The compatibility `VelocityProfile` view advertises `ProfileDerivativeOrder::Snap`.
Lower-order profilers advertise only the derivatives they physically represent;
call `hasJerk()` or `hasSnap()` before consuming optional fields.

See [the WSS reference](motion/WeightedSwitchingStructure.md) and
[the planner manual](motion/ParetoTimeEnergyOptimal.md) for detailed API and
failure behavior.

## Verification

`SnapSpaceContractTest.cpp` validates primitive formulas and inverse mappings,
arc endpoint propagation, $C^3$ state continuity, path constraints, round-trip
mappings, closed-form cost against quadrature, curve envelopes, unsupported
axis snap constraints, and numerical edge cases. The wider integration suite
covers TOPPRA, S-curve, analytical SSR/Hybrid, G-code, and extrusion consumers.
