# G64 Path Deviation Example

This example demonstrates G64 (path blending) corner deviations and produces an SVG using a minimal exporter example.

Files added:
- `g64_path_deviation.gcode` — Example G-code with G61 (exact) and G64 (blended) sections.
- `g64_demo_export.cpp` — Minimal example that parses X/Y moves and exports to SVG using `SVGExporter`.

Quick usage (from repository root):

```bash
# Build and run the exporter script (creates outputs/g64_path_deviation_parsed.svg)
./scripts/export_g64_svg.sh build_examples

# Or build and run the minimal example directly:
mkdir -p build && cmake -S Tether -B build -DTETHER_BUILD_EXAMPLES=ON && cmake --build build --target g64_demo_export -- -j
./build/examples/g64_demo_export Tether/examples/g64_path_deviation.gcode -o outputs/g64_path_deviation
```

Notes:
- The minimal parser used by `g64_demo_export` handles simple X/Y linear moves and is intended for visualization only (not a full G-code interpreter).
- For higher-fidelity visualization (blended trajectories) use the full analytical planner or the Python bindings if available.
