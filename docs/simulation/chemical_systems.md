# Chemical Systems

Chemical systems provide the library's main nonlinear process-control benchmarks. They are the right choice when identification needs to deal with operating-point dependence, slow thermal dynamics, multiple inputs, or physically meaningful disturbances such as feed changes and cooling-load variation.

## Recommended Benchmarks

- Use `CSTR` for coupled concentration and temperature dynamics with strong thermal feedback.
- Use `pHNeutralization` when you want extreme static nonlinearity around the titration curve.
- Use `DistillationColumn` when interaction between manipulated variables is more important than single-loop fit.

## Identification Notes

Process models in this family are often identified around a chosen operating point rather than globally. That makes them useful complements to the mechanical benchmarks: the same estimator can be evaluated under input noise, heat-load disturbances, and bias terms that would be unrealistic in a simple oscillator.

## Tradeoffs

- Advantage: Good coverage of nonlinear operating-point sensitivity and multi-input process behavior.
- Advantage: Disturbances have clear physical interpretations such as feed concentration, flow, or cooling changes.
- Disadvantage: Global low-order linear fits are usually poor unless the experiment stays near a single operating region.
- Disadvantage: Thermal states can evolve slowly, so tests need longer horizons than the mechanical benchmarks.
