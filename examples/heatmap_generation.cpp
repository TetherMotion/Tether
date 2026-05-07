/**
 * @file heatmap_generation.cpp
 * @brief Example: Performance heatmap generation
 * 
 * Demonstrates 1D, 2D, and 3D heatmap creation from trajectory data,
 * including differential heatmaps comparing expected vs actual performance.
 */

#include "PerformanceHeatmap.hpp"
#include "TestDataExporter.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>

using namespace MotionReplanner;

// Generate grid test data covering workspace
std::vector<PerformanceMetrics> generateWorkspaceData(
    double xMin, double xMax,
    double yMin, double yMax,
    double zMin, double zMax,
    int samples,
    std::mt19937& rng
) {
    std::vector<PerformanceMetrics> data;
    
    std::uniform_real_distribution<double> xDist(xMin, xMax);
    std::uniform_real_distribution<double> yDist(yMin, yMax);
    std::uniform_real_distribution<double> zDist(zMin, zMax);
    std::uniform_real_distribution<double> noise(0.8, 1.2);
    
    // Simulate position-dependent performance characteristics
    // - Better performance near center
    // - Reduced accuracy at edges and corners
    // - Axis-dependent limits
    
    double xCenter = (xMin + xMax) / 2.0;
    double yCenter = (yMin + yMax) / 2.0;
    double zCenter = (zMin + zMax) / 2.0;
    
    double xRange = xMax - xMin;
    double yRange = yMax - yMin;
    double zRange = zMax - zMin;
    
    for (int i = 0; i < samples; ++i) {
        double x = xDist(rng);
        double y = yDist(rng);
        double z = zDist(rng);
        
        // Distance from center (normalized)
        double dx = (x - xCenter) / (xRange / 2.0);
        double dy = (y - yCenter) / (yRange / 2.0);
        double dz = (z - zCenter) / (zRange / 2.0);
        double distFromCenter = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Edge proximity factor (0 = center, 1 = edge)
        double edgeFactor = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
        
        PerformanceMetrics pm;
        pm.position = {x, y, z};
        
        // Tracking error increases near edges
        pm.trackingError = 0.01 * (1.0 + 0.5 * edgeFactor) * noise(rng);
        
        // Velocity achieved decreases near edges
        pm.achievedVelocity = 100.0 * (1.0 - 0.3 * edgeFactor) * noise(rng);
        pm.commandedVelocity = 100.0;
        
        // Acceleration capability depends on position
        pm.achievedAcceleration = 2000.0 * (1.0 - 0.4 * distFromCenter) * noise(rng);
        pm.commandedAcceleration = 2000.0;
        
        // Jerk (more variation at edges)
        pm.jerk = 50000.0 * (1.0 + 0.6 * edgeFactor * noise(rng));
        
        // Position error
        pm.positionError = 0.005 * (1.0 + edgeFactor) * noise(rng);
        
        data.push_back(pm);
    }
    
    return data;
}

void printHeatmap1DSummary(const Heatmap1D& heatmap) {
    const auto& data = heatmap.getData();
    
    std::cout << "  Bins: " << data.size() << "\n";
    
    // Find min/max achieved velocity
    double minVel = 1e9, maxVel = 0;
    for (const auto& cell : data) {
        if (cell.sampleCount > 0) {
            if (cell.avgAchievedVelocity < minVel) minVel = cell.avgAchievedVelocity;
            if (cell.avgAchievedVelocity > maxVel) maxVel = cell.avgAchievedVelocity;
        }
    }
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Velocity range: " << minVel << " - " << maxVel << " mm/s\n";
}

void printHeatmap2DSummary(const Heatmap2D& heatmap) {
    const auto& data = heatmap.getData();
    
    int totalCells = data.size() * (data.empty() ? 0 : data[0].size());
    int filledCells = 0;
    double minError = 1e9, maxError = 0;
    
    for (const auto& row : data) {
        for (const auto& cell : row) {
            if (cell.sampleCount > 0) {
                filledCells++;
                if (cell.avgTrackingError < minError) minError = cell.avgTrackingError;
                if (cell.avgTrackingError > maxError) maxError = cell.avgTrackingError;
            }
        }
    }
    
    std::cout << "  Grid: " << data.size() << "x" 
              << (data.empty() ? 0 : data[0].size()) << "\n";
    std::cout << "  Filled cells: " << filledCells << "/" << totalCells << "\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Error range: " << minError << " - " << maxError << " mm\n";
}

int main() {
    std::cout << "=== Motion Replanner: Heatmap Generation Example ===\n\n";
    
    std::mt19937 rng(42);  // Reproducible results
    
    // Workspace bounds
    double xMin = 0, xMax = 300;
    double yMin = 0, yMax = 200;
    double zMin = 0, zMax = 100;
    
    std::cout << "Workspace: " << xMax - xMin << "x" << yMax - yMin << "x" 
              << zMax - zMin << " mm\n\n";
    
    // Generate test data
    std::cout << "Generating performance data...\n";
    auto data = generateWorkspaceData(xMin, xMax, yMin, yMax, zMin, zMax, 10000, rng);
    std::cout << "Generated " << data.size() << " samples\n\n";
    
    // --- 1D Heatmaps (per-axis) ---
    std::cout << "=== 1D Heatmaps (Per-Axis) ===\n\n";
    
    HeatmapConfig config1D;
    config1D.xBins = 30;
    
    std::cout << "X-Axis Heatmap:\n";
    Heatmap1D heatmapX(Axis::X, xMin, xMax, config1D);
    for (const auto& pm : data) {
        heatmapX.addSample(pm);
    }
    printHeatmap1DSummary(heatmapX);
    
    std::cout << "\nY-Axis Heatmap:\n";
    Heatmap1D heatmapY(Axis::Y, yMin, yMax, config1D);
    for (const auto& pm : data) {
        heatmapY.addSample(pm);
    }
    printHeatmap1DSummary(heatmapY);
    
    std::cout << "\nZ-Axis Heatmap:\n";
    Heatmap1D heatmapZ(Axis::Z, zMin, zMax, config1D);
    for (const auto& pm : data) {
        heatmapZ.addSample(pm);
    }
    printHeatmap1DSummary(heatmapZ);
    
    // --- 2D Heatmaps ---
    std::cout << "\n=== 2D Heatmaps ===\n\n";
    
    HeatmapConfig config2D;
    config2D.xBins = 30;
    config2D.yBins = 20;
    
    std::cout << "XY Plane Heatmap:\n";
    Heatmap2D heatmapXY(Plane::XY, xMin, xMax, yMin, yMax, config2D);
    for (const auto& pm : data) {
        heatmapXY.addSample(pm);
    }
    printHeatmap2DSummary(heatmapXY);
    
    std::cout << "\nXZ Plane Heatmap:\n";
    config2D.yBins = 10;
    Heatmap2D heatmapXZ(Plane::XZ, xMin, xMax, zMin, zMax, config2D);
    for (const auto& pm : data) {
        heatmapXZ.addSample(pm);
    }
    printHeatmap2DSummary(heatmapXZ);
    
    // --- 3D Heatmap (sparse voxel) ---
    std::cout << "\n=== 3D Heatmap (Sparse Voxel) ===\n\n";
    
    HeatmapConfig config3D;
    config3D.xBins = 15;
    config3D.yBins = 10;
    config3D.zBins = 5;
    
    Heatmap3D heatmap3D(xMin, xMax, yMin, yMax, zMin, zMax, config3D);
    for (const auto& pm : data) {
        heatmap3D.addSample(pm);
    }
    
    const auto& voxels = heatmap3D.getData();
    std::cout << "  Filled voxels: " << voxels.size() << "\n";
    
    // --- Differential Heatmap ---
    std::cout << "\n=== Differential Heatmap ===\n\n";
    
    // Generate "expected" data (theoretical perfect machine)
    std::cout << "Generating expected (ideal) performance data...\n";
    std::vector<PerformanceMetrics> expectedData;
    for (const auto& pm : data) {
        PerformanceMetrics expected = pm;
        expected.trackingError = 0.005;  // Ideal error
        expected.achievedVelocity = 100.0;  // Full velocity
        expected.achievedAcceleration = 2000.0;  // Full acceleration
        expectedData.push_back(expected);
    }
    
    HeatmapConfig configDiff;
    configDiff.xBins = 30;
    configDiff.yBins = 20;
    
    Heatmap2D expectedHeatmap(Plane::XY, xMin, xMax, yMin, yMax, configDiff);
    Heatmap2D actualHeatmap(Plane::XY, xMin, xMax, yMin, yMax, configDiff);
    
    for (const auto& pm : expectedData) {
        expectedHeatmap.addSample(pm);
    }
    for (const auto& pm : data) {
        actualHeatmap.addSample(pm);
    }
    
    DifferentialHeatmap diffHeatmap(expectedHeatmap, actualHeatmap);
    
    const auto& diffData = diffHeatmap.getData();
    int degradedCells = 0;
    double maxDegradation = 0;
    
    for (const auto& row : diffData) {
        for (const auto& cell : row) {
            if (cell.velocityRatio < 1.0) {
                degradedCells++;
            }
            double degradation = 1.0 - cell.velocityRatio;
            if (degradation > maxDegradation) {
                maxDegradation = degradation;
            }
        }
    }
    
    std::cout << "  Degraded cells: " << degradedCells << "\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Max degradation: " << (maxDegradation * 100) << "%\n";
    
    // --- Export to files ---
    std::cout << "\n=== Exporting Data ===\n\n";
    
    HeatmapExporter exporter;
    
    ExportConfig exportCfg;
    exportCfg.format = ExportFormat::CSV;
    exportCfg.outputPath = "heatmap_xy.csv";
    
    if (exporter.exportHeatmap2D(heatmapXY, exportCfg)) {
        std::cout << "Exported 2D heatmap to: " << exportCfg.outputPath << "\n";
    }
    
    exportCfg.format = ExportFormat::JSON;
    exportCfg.outputPath = "heatmap_xy.json";
    
    if (exporter.exportHeatmap2D(heatmapXY, exportCfg)) {
        std::cout << "Exported 2D heatmap to: " << exportCfg.outputPath << "\n";
    }
    
    exportCfg.outputPath = "heatmap_diff.json";
    if (exporter.exportDifferentialHeatmap(diffHeatmap, exportCfg)) {
        std::cout << "Exported differential heatmap to: " << exportCfg.outputPath << "\n";
    }
    
    std::cout << "\n=== Heatmap Generation Complete ===\n";
    std::cout << "\nUse visualize.py to generate plots:\n";
    std::cout << "  python visualize.py heatmap2d heatmap_xy.csv\n";
    std::cout << "  python visualize.py heatmap2d heatmap_diff.json --differential\n";
    
    return 0;
}
