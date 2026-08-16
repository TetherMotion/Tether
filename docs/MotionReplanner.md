# Motion Replanning Framework

A comprehensive C++ framework for online motion replanning based on closed-loop feedback, with support for system identification, performance heatmaps, test pattern generation, and Python visualization.

## Features

### Core Capabilities

- **Closed-Loop Feedback Analysis**
  - Real-time trajectory tracking error computation
  - System delay compensation (configurable latency)
  - Per-sample and segment-level statistics
  - Corner detection and specialized corner error analysis

- **Error Statistics**
  - Min, max, mean, geometric mean, standard deviation
  - RMS error, percentiles (P95, P99)
  - Rolling window statistics for online monitoring
  - Separate statistics for corners vs straight segments

- **Operation Modes**
  - **Monitor**: Passive observation, statistics collection
  - **Suggest**: Generate parameter improvement recommendations
  - **Adjust**: Apply smooth, C2-continuous limit changes

### Performance Heatmaps

- **1D Heatmaps**: Per-axis performance visualization
- **2D Heatmaps**: XY, XZ, YZ plane projections
- **3D Heatmaps**: Sparse voxel representation for full workspace
- **Differential Heatmaps**: Compare expected vs actual performance

### Machine Testing

- **Single-Axis Tests**
  - Sinusoid (frequency response)
  - Ramp (constant velocity)
  - S-Curve (jerk-limited motion)
  - Step response

- **Multi-Axis Tests**
  - Circle (constant curvature)
  - Ellipse (varying curvature, rotatable)
  - Helix (3D circular motion)
  - Lissajous patterns (frequency ratio combinations)
  - Square (corner performance)

### System Identification

- **Delay Detection**: Cross-correlation based latency identification
- **Friction Models**: Coulomb, Viscous, Stribeck, LuGre
- **PID Analysis**: Overshoot, settling time, rise time, stability margins
- **Online Estimation**: Real-time delay and parameter tracking

### G-Code Generation

- **Multi-Dialect Support**: LinuxCNC, Fanuc, Mach3, Grbl, Marlin, Haas
- **Test Patterns**: Circles, ellipses, squares, sinusoids, friction tests
- **Workspace Sweep**: Full workspace characterization
- **Duration Estimation**: Predicted runtime calculation

### Data Export

- **Formats**: CSV, TSV, JSON, Binary, NumPy-compatible
- **Streaming Export**: Real-time data logging
- **Report Generation**: Markdown, HTML, LaTeX
- **Batch Processing**: Export multiple datasets efficiently

### Python Visualization

- Trajectory comparison plots
- Error heatmaps (1D, 2D, 3D)
- Friction model curves
- PID assessment charts
- Differential analysis
- Interactive 3D plots

## Directory Structure

```
motion_replanner/
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
│
├── MotionReplanner.hpp/cpp     # Core replanning and error analysis
├── PerformanceHeatmap.hpp/cpp  # Spatial performance mapping
├── MachineTester.hpp/cpp       # Test trajectory generation
├── GCodeGenerator.hpp/cpp      # G-Code generation
├── SystemIdentifier.hpp/cpp    # System parameter identification
├── TestDataExporter.hpp/cpp    # Data export utilities
│
├── motion_replanner_main.cpp   # CLI integration tool
├── visualize.py                # Python visualization suite
│
├── examples/
│   ├── trajectory_analysis.cpp # Error statistics example
│   ├── heatmap_generation.cpp  # Heatmap creation example
│   └── gcode_generation.cpp    # G-Code export example
│
└── tests/
    ├── test_motion_replanner.cpp
    ├── test_heatmap.cpp
    └── test_system_identifier.cpp
```

## Building

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.14+
- (Optional) Google Test for unit tests
- (Optional) Python 3.7+ with numpy, matplotlib, pandas for visualization

### Build Commands

```bash
cd host_tests/gcode_export/motion_replanner
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### With Tests

```bash
cmake -DBUILD_TESTS=ON ..
make
ctest --output-on-failure
```

### Install

```bash
sudo make install
```

## Usage

### Command Line Tool

```bash
# Analyze trajectory data
./motion_replanner analyze trajectory.csv --delay 0.001 --output report.json

# Generate machine tests
./motion_replanner test --type circle --radius 50 --output test_circle.ngc

# System identification
./motion_replanner identify data.csv --friction stribeck --output friction_model.json

# Generate heatmap
./motion_replanner heatmap performance_data.csv --type 2d --plane xy --output heatmap.csv

# Export G-Code
./motion_replanner gcode --pattern ellipse --rotation 45 --dialect linuxcnc --output test.ngc

# Generate report
./motion_replanner report test_session/ --format html --output report.html
```

### Python Visualization

```bash
# Plot trajectory comparison
python visualize.py trajectory commanded.csv actual.csv

# Generate error heatmap
python visualize.py heatmap2d heatmap_xy.csv --colormap viridis

# Plot friction model
python visualize.py friction friction_data.json

# Generate all plots for a session
python visualize.py batch session_data/ --output plots/
```

### C++ API

```cpp
#include "MotionReplanner.hpp"
#include "PerformanceHeatmap.hpp"
#include "MachineTester.hpp"

using namespace MotionReplanner;

// Configure replanner
ReplannerConfig config;
config.systemDelay = 0.001;      // 1ms
config.samplePeriod = 0.001;     // 1kHz
config.mode = OperationMode::Monitor;

MotionReplanner replanner(config);

// Process samples
for (const auto& [commanded, actual] : samples) {
    replanner.processSample(commanded, actual);
}

// Get statistics
auto globalStats = replanner.getGlobalStatistics();
auto cornerStats = replanner.getCornerStatistics();

std::cout << "RMS Error: " << globalStats.rms << " mm\n";
std::cout << "Corner Max Error: " << cornerStats.max << " mm\n";
```

### Generating Test Patterns

```cpp
#include "GCodeGenerator.hpp"

GCodeOptions options;
options.dialect = GCodeDialect::LinuxCNC;
options.feedRate = 1000.0;

TestPatternGenerator generator(options);

// Generate circle test
CircleTestConfig circleConfig;
circleConfig.centerX = 100.0;
circleConfig.centerY = 100.0;
circleConfig.radius = 50.0;

auto circleProgram = generator.generateCircleTest(circleConfig);

// Export to file
GCodeExporter exporter;
exporter.exportToFile(circleProgram, "test_circle.ngc");
```

### Building Heatmaps

```cpp
#include "PerformanceHeatmap.hpp"

HeatmapConfig config;
config.xBins = 30;
config.yBins = 20;

Heatmap2D heatmap(Plane::XY, 0.0, 300.0, 0.0, 200.0, config);

for (const auto& measurement : performanceData) {
    heatmap.addSample(measurement);
}

// Get suggested limits
auto suggestions = heatmap.getSuggestedLimits();
std::cout << "Suggested max velocity: " << suggestions.maxSuggestedVelocity << " mm/s\n";
```

### System Identification

```cpp
#include "SystemIdentifier.hpp"

SystemIdentifier identifier;

// Identify system delay
auto delayResult = identifier.identifyDelay(samples, 0.001);
std::cout << "Estimated delay: " << delayResult.estimatedDelay * 1000 << " ms\n";
std::cout << "Confidence: " << delayResult.confidence * 100 << "%\n";

// Identify friction model
auto frictionResult = identifier.identifyFriction(samples, FrictionModelType::Stribeck);
std::cout << "Coulomb friction: " << frictionResult.params.coulombForce << " N\n";
std::cout << "Stribeck velocity: " << frictionResult.params.stribeckVelocity << " mm/s\n";
```

## Configuration Options

### ReplannerConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `systemDelay` | double | 0.001 | System latency in seconds |
| `samplePeriod` | double | 0.001 | Sample period in seconds |
| `errorThreshold` | double | 0.1 | Error threshold for warnings (mm) |
| `cornerVelocityThreshold` | double | 0.1 | Velocity change for corner detection |
| `cornerAccelThreshold` | double | 100.0 | Acceleration for corner detection |
| `mode` | OperationMode | Monitor | Operation mode |
| `maxVelocity` | double | 100.0 | Maximum velocity limit (mm/s) |
| `maxAcceleration` | double | 1000.0 | Maximum acceleration (mm/s²) |
| `maxJerk` | double | 50000.0 | Maximum jerk (mm/s³) |
| `compensateDelay` | bool | true | Enable delay compensation |

### HeatmapConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `xBins` | int | 20 | Number of X-axis bins |
| `yBins` | int | 20 | Number of Y-axis bins |
| `zBins` | int | 10 | Number of Z-axis bins |
| `minSamplesForValid` | int | 5 | Minimum samples for valid cell |
| `interpolateEmpty` | bool | false | Interpolate empty cells |

### GCodeOptions

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dialect` | GCodeDialect | LinuxCNC | G-Code dialect |
| `feedRate` | double | 1000.0 | Default feed rate (mm/min) |
| `rapidFeedRate` | double | 5000.0 | Rapid traverse rate |
| `safeZ` | double | 10.0 | Safe retract height (mm) |
| `useAbsoluteCoords` | bool | true | Use G90 absolute mode |
| `includeComments` | bool | true | Include comments in output |
| `precision` | int | 3 | Decimal places for coordinates |

## Workflow Examples

### Complete Machine Characterization

1. **Generate workspace sweep G-Code**:
   ```bash
   ./motion_replanner gcode --pattern sweep --output sweep.ngc
   ```

2. **Run on machine, capture feedback data**

3. **Analyze captured data**:
   ```bash
   ./motion_replanner analyze captured.csv --output analysis.json
   ```

4. **Generate heatmaps**:
   ```bash
   ./motion_replanner heatmap captured.csv --type 2d --output heatmap_xy.csv
   ```

5. **Visualize results**:
   ```bash
   python visualize.py batch analysis.json heatmap_xy.csv --output plots/
   ```

6. **Generate report**:
   ```bash
   ./motion_replanner report ./ --format html --output characterization_report.html
   ```

### Friction Model Identification

1. **Generate friction test G-Code**:
   ```bash
   ./motion_replanner gcode --pattern friction --axis X --output friction_test.ngc
   ```

2. **Execute test, record velocity and current data**

3. **Identify friction model**:
   ```bash
   ./motion_replanner identify friction_data.csv --friction stribeck --output model.json
   ```

4. **Visualize friction curve**:
   ```bash
   python visualize.py friction model.json --output friction_curve.png
   ```

### System Delay Measurement

1. **Generate step response test**:
   ```bash
   ./motion_replanner test --type step --output step_test.ngc
   ```

2. **Capture high-resolution position feedback**

3. **Identify delay**:
   ```bash
   ./motion_replanner identify step_data.csv --delay --output delay_result.json
   ```

## Error Statistics Explained

- **Min/Max**: Absolute minimum and maximum tracking errors
- **Mean**: Arithmetic mean of all errors
- **Geometric Mean**: n-th root of product of errors (less sensitive to outliers)
- **Std Dev**: Standard deviation, measure of error spread
- **RMS**: Root mean square, combines mean and variation
- **P95/P99**: 95th and 99th percentile errors

## Certified Motion Replanner (Phases 1–8)

The certified motion replanner integrates the motion kernel's certified
algorithms into the replanning framework, replacing the old heuristic
methods with guaranteed bounds and audit trails.

### Architecture

All new certified modules live in `namespace tether::motion::replanner`
and are declared in `include/tether/motion_replanner/`. A unified entry
point is provided by `CertifiedReplanner.hpp`, which also brings the new
types into the legacy `MotionReplanner` namespace for backward
compatibility.

### Phase 1: TrajectorySampleConverter

Converts `vector<GCodeExport::TrajectorySample>` →
`tether::motion::PiecewiseNurbsPath`, bridging the export module's dense
sampled representation to the kernel's certified geometric path type.

- Linear/rapid segments → `NurbsCurve::fromLine` (degree 1, exact)
- Arc segments (G2/G3) → `NurbsCurve::fromArc` via 3-point circumcenter
  fit, with polyline fallback
- Active-axis extraction keeps the RVec dimension minimal

### Phase 2: CertifiedContourError

Replaces tangent-projection contour error with
`tether::motion::pointCurveDistance` (Bernstein root isolation, M8/M9).
Returns the TRUE global minimum distance from the actual position to the
NURBS path, plus lag error decomposition.

- `computeCertifiedContourError`: searches all pieces (global, offline)
- `computeCertifiedContourErrorLocal`: searches ±N pieces (real-time)

### Phase 3: CertifiedCornerDetection

Replaces 3-sample angle-threshold with `tether::motion::CornerAnalyzer`
(M13). Extracts exact tangents from NURBS arc-length derivatives,
classifies Straight/Corner/Cusp, builds the orthonormal (e₁, e₂) plane
basis via Gram-Schmidt.

### Phase 4: CurvatureAwareLimiter

Proactive feed limiting using `v_safe = sqrt(maxCentripetalAccel / kappa)`.

- Immediate: uses the `curvature` field already in `TrajectorySample`
  (previously ignored)
- Certified: uses `CertifiedCurvatureSampler` for a Lipschitz-certified
  upper bound on max curvature per span

### Phase 5: CertifiedSuggestionSolver

Replaces the `1/sqrt(errorRatio)` heuristic with M15-pattern bisection.
Finds the maximum feed rate whose predicted contour error stays within
the tolerance threshold. T3-analog guarantee: the suggested feed never
violates the tolerance, given the error model.

### Phase 6: OnlineReblender

True geometric replanning via `tether::motion::PathBlender::blend`.
Re-blends problematic junctions with tighter tolerance or higher
continuity (G²→G³). The `BlendAuditEntry` trail provides the "no silent
fallback" guarantee — every decision is visible to the caller.

### Phase 7: ProfileReplanner

Replaces scalar `quinticBlend` transitions with:

- `replanProfile`: TOPP-RA velocity profile via
  `MotionPlanner::BasicTOPPRA` (curvature-aware limit curve)
- `computeSCurveTransition`: 7-phase jerk-limited S-curve via
  `MotionPlanner::SCurveProfile`

### Phase 8: Namespace Convergence

New modules use `namespace tether::motion::replanner` per
`Architecture.md` §3. The `CertifiedReplanner.hpp` header provides
`using` declarations so legacy code using `MotionReplanner::` can access
the new certified types. The full namespace migration (renaming all
existing headers from `MotionReplanner` to `tether::motion::replanner`)
is deferred to avoid a flag-day rename of examples and python bindings.

### Certification Summary

| Module | Certification | Guarantee |
|--------|--------------|-----------|
| TrajectorySampleConverter | Exact (lines/arcs) | Geometric fidelity |
| CertifiedContourError | Certified (M8/M9) | True global min distance |
| CertifiedCornerDetection | Exact (G.18–G.21) | True turning angle |
| CurvatureAwareLimiter | Certified (Lipschitz) | Conservative v_lim |
| CertifiedSuggestionSolver | Certified (T3 analog) | No threshold violation |
| OnlineReblender | Certified (M10/M15) | Deviation ≤ tolerance |
| ProfileReplanner | Heuristic (TOPP-RA) | Time-optimal within limits |

### Usage Example

```cpp
#include "tether/motion_replanner/CertifiedReplanner.hpp"

using namespace tether::motion::replanner;

// 1. Build the path
PiecewiseNurbsPath path = convertTrajectory(trajectorySamples);

// 2. Compute certified contour error
CertifiedContourError err = computeCertifiedContourError(
    path, actualPosition, desiredArcLength);

// 3. Detect corners
CertifiedCornerDetection corners = detectCorners(path);

// 4. Get curvature-aware feed limits
CurvatureAwareFeedLimits feedLimits = computeCertifiedFeedLimits(path);

// 5. Get certified feed suggestion
CertifiedSuggestion suggestion = solveCertifiedFeedRate(
    currentFeedRate, err.contourError);

// 6. Re-blend problematic junctions
ReblendResult reblend = reblendJunctions(path, {0, 2});

// 7. Re-plan the velocity profile
auto newPath = extractPath(reblend.blendedPath);
ProfileReplanResult profile = replanProfile(*newPath, suggestedFeedRate);
```

## Path Evaluation Framework

The path evaluation framework provides extensive quantitative and qualitative
evaluators for comparing desired vs. actual trajectories, along with FFT-based
oscillation detection and SVG visualization.

### Components

1. **PathEvaluator** (`PathEvaluator.hpp/.cpp`) — Quantitative and qualitative
   metrics for desired vs. actual path comparison.

2. **PathRelativeFFT** (`PathRelativeFFT.hpp/.cpp`) — FFT-based oscillation
   detection in both spatial (arc-length) and temporal (time) domains, with
   Frenet-frame decomposition and path-geometry correlation.

3. **SvgExporter** (`SvgExporter.hpp/.cpp`) — SVG vector graphics export for
   trajectory comparison, error profiles, spectral plots, and dashboards.

### Quantitative Metrics

The `PathEvaluator::evaluateQuantitative()` method computes:

- **Integral errors**: IAE, ISE, ITAE, ITSE (spatial and temporal)
- **Norm errors**: L1, L2, L∞ for contour, lag, and combined errors
- **Shape distances**: Hausdorff, Frechet, DTW, path length ratio
- **Kinematic tracking**: velocity/acceleration RMS and max errors, jerk, smoothness index
- **Surface finish estimates**: Ra, Rq, Rz (in µm), peak count
- **Following error**: max/mean following error, settling distance, cross-correlation
- **Statistical summaries**: min/max/mean/RMS/std dev/percentiles for each error type

### Qualitative Assessment

The `PathEvaluator::evaluateQualitative()` method produces letter grades (A–F)
with scores (0–1) for:

- **Path fidelity** (max contour error)
- **Surface finish** (Ra)
- **Timing fidelity** (max following error)
- **Smoothness** (max jerk)
- **Oscillation severity** (from spectral analysis)
- **Corner preservation** (max corner error)
- **Overall** (weighted combination)

Each grade comes with human-readable descriptions and actionable recommendations.

### Path-Relative FFT Analysis

The `PathRelativeFFT::evaluate()` method performs spectral analysis in both
domains:

- **Spatial domain** (arc-length resampled, frequency in cycles/mm)
- **Temporal domain** (time resampled, frequency in Hz)

For each domain, it analyzes four Frenet-frame components:
- Contour (normal to path)
- Lag (along path)
- Binormal (out-of-plane)
- Combined (3D magnitude)

Key features:
- PCHIP interpolation for uniform resampling
- Detrending (DC removal + linear trend)
- Windowing (Hann, Hamming, Blackman, Rectangular)
- Cooley-Tukey radix-2 FFT
- Peak detection with prominence filtering
- Spectral entropy and oscillation index
- Band power (low/mid/high)
- Harmonic distortion (2nd + 3rd / fundamental)
- Cross-domain comparison (feed-rate modulation index)
- Path-geometry correlation (corner/segment/arc frequencies)

### SVG Export

The `SvgExporter` generates standalone .svg files:

- **Trajectory projections**: XY, XZ, YZ (2D), 3D isometric
- **Error profiles**: vs path length, vs time
- **Error histogram**: distribution with P95/P99 markers
- **Error envelope**: magnified deviation band
- **Spectral plots**: magnitude/phase with peak markers and geometry lines
- **Kinematic profiles**: velocity, acceleration
- **Phase portrait**: position vs velocity
- **Dashboard**: all plots in a single SVG

### CLI Usage

```bash
# Evaluate a trajectory and export all results
./motion_replanner_cli evaluate -i trajectory.csv -o results/

# Skip FFT analysis for faster evaluation
./motion_replanner_cli evaluate -i trajectory.csv -o results/ --no-fft

# Export as JSON
./motion_replanner_cli evaluate -i trajectory.csv -o results/ --format json
```

### C++ API Usage

```cpp
#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/PathRelativeFFT.hpp"
#include "tether/motion_replanner/SvgExporter.hpp"

using namespace tether::motion::replanner;

// Quantitative evaluation
PathEvaluator evaluator;
auto quant = evaluator.evaluateQuantitative(desired, actual);

// Spectral evaluation
PathRelativeFFT fftEval;
auto spectral = fftEval.evaluate(desired, actual);

// Qualitative evaluation (uses spectral data for oscillation grade)
auto qual = evaluator.evaluateQualitative(quant, &spectral);

// SVG export
SvgExporter svgExporter;
svgExporter.exportDashboard("dashboard.svg", desired, actual, quant, spectral);
```

## KDE Derivative-vs-Deviation Analysis

The `KdeDerivativeAnalyzer` estimates the joint probability density
`p(derivative, deviation)` from desired-vs-actual trajectory sample pairs using
2D kernel density estimation (KDE). This reveals statistical relationships
between kinematic demands (velocity, acceleration, jerk, curvature) and tracking
error that are invisible in scatter plots for large N and impossible to extract
from summary statistics alone.

### Why Derivative vs Deviation?

Tracking error typically scales with kinematic demands:

| Derivative | Reveals |
|---|---|
| **Velocity** | Servo lag, resonance bands, feed-rate safety envelopes |
| **Acceleration** | Inertia-induced overshoot, structural compliance |
| **Jerk** | Discontinuity-induced vibration, surface finish degradation |
| **Curvature** | Corner-rounding, controller tolerance limits |

### Algorithm

1. **Sample extraction**: For each `(desired[i], actual[i])` pair, compute the
   kinematic derivative `d_i` and the deviation `e_i`.

2. **Bandwidth selection**: Controls smoothness of the estimate.
   - **Silverman** (default): $h = 1.06 \cdot \sigma \cdot n^{-1/5}$
   - **Scott**: $h = \sigma \cdot n^{-1/5}$
   - **ISJ**: Improved Sheather-Jones (data-driven, robust for multimodal)
   - **Fixed**: User-specified
   - **LSCV / Likelihood CV**: Cross-validation (expensive)

3. **KDE evaluation**: 2D density at grid point $(d, e)$:

$$
   p(d, e) = \frac{1}{n \cdot h_d \cdot h_e} \sum_i K\!\left(\frac{d - d_i}{h_d}\right) \cdot K\!\left(\frac{e - e_i}{h_e}\right)
$$

   Supported kernels: Gaussian, Epanechnikov, Uniform, Triangular, Quartic
   (biweight), Cosine. For large N (>5000), a binned approximation is used.

4. **Derived metrics**: Marginal densities, conditional density `p(e|d)`,
   conditional mean/variance, quantile contours, mutual information,
   correlation ratio, distance correlation, tail risk (VaR/CVaR).

5. **Threshold extraction**: Finds the derivative value at which deviation
   exceeds a tolerance with a given probability.

### Configuration

```cpp
#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"

using namespace tether::motion::replanner;

KdeConfig config;
config.derivativeAxis = DerivativeAxis::Velocity;
config.deviationAxis = DeviationAxis::ContourError;
config.kernel = KernelType::Gaussian;
config.bandwidthMethod = BandwidthMethod::Silverman;
config.gridX = 128;
config.gridY = 128;
config.tolerances = {0.005, 0.01, 0.02, 0.05, 0.1, 0.2};
config.thresholdProbability = 0.05;
config.useCertifiedContourError = true;

KdeDerivativeAnalyzer analyzer(config);
KdeEvaluation eval = analyzer.evaluate(desired, actual);

if (eval.hasSufficientData) {
    std::cout << "Mutual information: " << eval.mutualInformation << " bits\n";
    std::cout << "Correlation ratio: " << eval.correlationRatio << "\n";
    std::cout << "Distance correlation: " << eval.distanceCorrelation << "\n";
    std::cout << "VaR95: " << eval.var95 << " mm\n";
    std::cout << "CVaR95: " << eval.conditionalVar95 << " mm\n";

    for (const auto& t : eval.thresholds) {
        if (t.found) std::cout << t.description << "\n";
    }
}
```

### Available Axes

**Derivative axes** (`DerivativeAxis`):
- `Velocity` — speed magnitude (mm/s)
- `Acceleration` — acceleration magnitude (mm/s²)
- `Jerk` — jerk magnitude (mm/s³)
- `Curvature` — path curvature (1/mm)
- `FeedRate` — commanded feed rate (mm/s)
- `ArcLength` — arc length position (mm)
- `Time` — time (s)

**Deviation axes** (`DeviationAxis`):
- `ContourError` — perpendicular distance to path (mm)
- `LagError` — signed arc-length offset (mm)
- `CombinedError` — 3D Euclidean distance (mm)
- `BinormalError` — out-of-plane component (mm)
- `TrackingError` — position error magnitude (mm)
- `VelocityError` — velocity tracking error (mm/s)
- `AccelerationError` — acceleration tracking error (mm/s²)

### Dependence Metrics

The analyzer computes multiple dependence measures between the derivative and
deviation:

| Metric | Range | Captures |
|---|---|---|
| Pearson $r$ | $[-1, 1]$ | Linear correlation |
| Spearman $\rho$ | $[-1, 1]$ | Monotonic correlation |
| Kendall $\tau$ | $[-1, 1]$ | Rank correlation |
| Mutual information | $[0, \infty)$ bits | Any dependence |
| Correlation ratio $\eta^2$ | $[0, 1]$ | Nonlinear dependence |
| Distance correlation | $[0, 1]$ | Any dependence |
| Normalized MI | $[0, 1]$ | Symmetric dependence |

### SVG Heatmap Export

The `SvgExporter` renders KDE evaluations as SVG heatmaps:

```cpp
SvgConfig svgConfig;
svgConfig.kdeColormap = KdeColormap::Viridis;
svgConfig.kdeLogScale = false;
svgConfig.kdeShowMarginals = true;
svgConfig.kdeShowConditionalMean = true;
svgConfig.kdeShowScatter = false;

SvgExporter svgExporter(svgConfig);
svgExporter.exportKdeHeatmap("kde_heatmap.svg", eval);
svgExporter.exportKdeConditional("kde_conditional.svg", eval);
svgExporter.exportKdeDashboard("kde_dashboard.svg", eval);
```

Available colormaps: Viridis, Inferno, Plasma, Magma, Jet, Hot, Cool, Grayscale,
BlueRed.

### Raw Data Export

The `BatchExporter` exports KDE data in multiple formats:

```cpp
BatchExporter exporter("output_dir/", exportConfig);
exporter.exportKdeData(eval, svgConfig);
exporter.generateManifest("manifest.json");
```

This generates:
- `kde_samples.csv` — raw (derivative, deviation) sample pairs
- `kde_density_grid.csv` — density grid as a matrix
- `kde_density_grid.json` — density grid with metadata
- `kde_conditional.csv` — conditional statistics per X bin
- `kde_marginals.csv` — marginal statistics for both axes
- `kde_dependence.csv` — all dependence metrics
- `kde_thresholds.csv` — deviation threshold analysis
- `kde_tail_risk.csv` — VaR/CVaR/ETD metrics
- `kde_heatmap.svg`, `kde_conditional.svg`, `kde_marginal_x.svg`,
  `kde_marginal_y.svg`, `kde_dashboard.svg`

### CLI Usage

The `evaluate` command automatically runs KDE analysis:

```bash
# Default: velocity vs contour error, Gaussian kernel, Silverman bandwidth
./motion_replanner_cli evaluate -i trajectory.csv -o results/ --format all

# Custom KDE configuration
./motion_replanner_cli evaluate -i trajectory.csv -o results/ \
    --kde-derivative acceleration \
    --kde-deviation contour \
    --kde-kernel epanechnikov \
    --kde-bandwidth isj \
    --kde-colormap inferno

# Disable KDE analysis
./motion_replanner_cli evaluate -i trajectory.csv -o results/ --no-kde
```

CLI options:
- `--no-kde` / `--kde` — disable/enable KDE analysis (default: enabled)
- `--kde-derivative <ax>` — velocity, acceleration, jerk, curvature, feedrate,
  arclength, time
- `--kde-deviation <ax>` — contour, lag, combined, binormal, tracking, velocity,
  acceleration
- `--kde-kernel <k>` — gaussian, epanechnikov, uniform, triangular, quartic,
  cosine
- `--kde-bandwidth <m>` — silverman, scott, isj, fixed, lscv, likelihoodcv
- `--kde-colormap <c>` — viridis, inferno, plasma, magma, jet, hot, cool,
  grayscale, bluered

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## Related Tools

- [LinuxCNC](https://linuxcnc.org/) - Open-source CNC controller
- [CAMotics](https://camotics.org/) - G-Code simulator
- [NCViewer](https://ncviewer.com/) - Online G-Code viewer
