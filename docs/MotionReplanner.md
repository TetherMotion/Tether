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
