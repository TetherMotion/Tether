/**
 * @file TestDataExporterHeatmap.cpp
 * @brief Heatmap data exporter implementation
 */

#include "TestDataExporter.hpp"

namespace MotionReplanner {

//=============================================================================
// HeatmapExporter Implementation
//=============================================================================

bool HeatmapExporter::exportHeatmap1D(const std::string& filename, const Heatmap1D& heatmap) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    auto data = heatmap.getAllBins();
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        file << "position" << d
             << "velocity_limit" << d << "acceleration_limit" << d
             << "max_error" << d << "mean_error" << d
             << "sample_count\n";
    }
    
    for (const auto& cell : data) {
        file << formatDouble(cell.position) << d
             << formatDouble(cell.metrics.maxAchievedVelocity) << d
             << formatDouble(cell.metrics.maxAchievedAccel) << d
             << formatDouble(cell.metrics.maxTrackingError) << d
             << formatDouble(cell.metrics.meanTrackingError) << d
             << cell.metrics.sampleCount << "\n";
    }
    
    return file.good();
}

bool HeatmapExporter::exportHeatmap2D(const std::string& filename, const Heatmap2D& heatmap) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    if (config_.format == ExportFormat::Numpy) {
        writeNumpyHeatmap2D(file, heatmap);
    } else if (config_.format == ExportFormat::JSON || config_.format == ExportFormat::JSONPretty) {
        writeJSONHeatmap2D(file, heatmap);
    } else {
        writeCSVHeatmap2D(file, heatmap);
    }
    
    return file.good();
}

void HeatmapExporter::writeCSVHeatmap2D(std::ostream& out, const Heatmap2D& heatmap) {
    auto velocityGrid = heatmap.getVelocityGrid();
    auto accelGrid = heatmap.getAccelGrid();
    char d = config_.delimiter;
    
    size_t rows = velocityGrid.rows;
    size_t cols = velocityGrid.cols;
    
    out << "# Velocity Limit Grid\n";
    for (size_t j = 0; j < rows; ++j) {
        for (size_t i = 0; i < cols; ++i) {
            if (i > 0) out << d;
            out << formatDouble(velocityGrid.data[j * cols + i]);
        }
        out << "\n";
    }
    
    out << "\n# Acceleration Limit Grid\n";
    for (size_t j = 0; j < rows; ++j) {
        for (size_t i = 0; i < cols; ++i) {
            if (i > 0) out << d;
            out << formatDouble(accelGrid.data[j * cols + i]);
        }
        out << "\n";
    }
}

void HeatmapExporter::writeJSONHeatmap2D(std::ostream& out, const Heatmap2D& heatmap) {
    auto velocityGrid = heatmap.getVelocityGrid();
    auto accelGrid = heatmap.getAccelGrid();
    bool pretty = (config_.format == ExportFormat::JSONPretty);
    
    JSONBuilder json(pretty);
    
    json.beginObject();
    
    json.key("config");
    json.beginObject();
    json.keyValue("rows", static_cast<int>(velocityGrid.rows));
    json.keyValue("cols", static_cast<int>(velocityGrid.cols));
    json.key("minBounds");
    json.doubleArray({velocityGrid.minU, velocityGrid.minV, 0.0});
    json.key("maxBounds");
    json.doubleArray({velocityGrid.maxU, velocityGrid.maxV, 0.0});
    json.endObject();
    
    json.doubleArray("velocityLimit", velocityGrid.data);
    json.doubleArray("accelerationLimit", accelGrid.data);
    
    json.endObject();
    
    out << json.str();
}

void HeatmapExporter::writeNumpyHeatmap2D(std::ostream& out, const Heatmap2D& heatmap) {
    auto velocityGrid = heatmap.getVelocityGrid();
    auto accelGrid = heatmap.getAccelGrid();
    
    // NumPy .npy format header
    // Magic number + version
    out.write("\x93NUMPY", 6);
    out.put(0x01);  // Major version
    out.put(0x00);  // Minor version
    
    // Build header string
    std::ostringstream header;
    header << "{'descr': '<f8', 'fortran_order': False, 'shape': ("
           << velocityGrid.rows << ", " << velocityGrid.cols << ", 2), }";
    
    std::string headerStr = header.str();
    // Pad to multiple of 64
    size_t padding = 64 - ((10 + headerStr.size()) % 64);
    headerStr += std::string(padding - 1, ' ') + '\n';
    
    uint16_t headerLen = static_cast<uint16_t>(headerStr.size());
    out.write(reinterpret_cast<char*>(&headerLen), 2);
    out << headerStr;
    
    // Write data
    for (size_t i = 0; i < velocityGrid.data.size(); ++i) {
        double v = velocityGrid.data[i];
        double a = accelGrid.data[i];
        out.write(reinterpret_cast<char*>(&v), sizeof(double));
        out.write(reinterpret_cast<char*>(&a), sizeof(double));
    }
}

bool HeatmapExporter::exportHeatmap3D(const std::string& filename, const Heatmap3D& heatmap) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        file << "x" << d << "y" << d << "z" << d
             << "velocity_limit" << d << "acceleration_limit" << d
             << "max_error" << d << "sample_count\n";
    }
    
    auto data = heatmap.getAllVoxels();
    for (const auto& voxel : data) {
        file << formatDouble(voxel.position[0]) << d
             << formatDouble(voxel.position[1]) << d
             << formatDouble(voxel.position[2]) << d
             << formatDouble(voxel.metrics.maxAchievedVelocity) << d
             << formatDouble(voxel.metrics.maxAchievedAccel) << d
             << formatDouble(voxel.metrics.maxTrackingError) << d
             << voxel.metrics.sampleCount << "\n";
    }
    
    return file.good();
}

bool HeatmapExporter::exportDifferentialHeatmap(const std::string& filename,
    const DifferentialHeatmap& heatmap) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    // Export problem areas as CSV
    char d = config_.delimiter;
    
    // Header
    file << "# Differential Heatmap - Problem Areas\n";
    file << "x" << d << "y" << d << "z" << d
         << "velocity_ratio" << d << "accel_ratio" << d
         << "underperforming\n";
    
    auto problemAreas = heatmap.getProblemAreas(0.8);
    for (const auto& voxel : problemAreas) {
        auto diff = heatmap.getDifferential(voxel.position);
        file << formatDouble(voxel.position[0]) << d
             << formatDouble(voxel.position[1]) << d
             << formatDouble(voxel.position[2]) << d
             << formatDouble(diff.velocityRatio) << d
             << formatDouble(diff.accelRatio) << d
             << (diff.underperforming ? "1" : "0") << "\n";
    }
    
    return file.good();
}

bool HeatmapExporter::exportSuggestedLimits(const std::string& filename,
    const SuggestedLimits& limits) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    
    json.key("globalLimits");
    json.beginObject();
    json.keyValue("maxVelocity", limits.maxVelocity);
    json.keyValue("maxAcceleration", limits.maxAcceleration);
    json.keyValue("maxJerk", limits.maxJerk);
    json.endObject();
    
    json.keyValue("confidence", limits.confidence);
    json.keyValue("limitingFactor", limits.limitingFactor);
    
    json.endObject();
    
    file << json.str();
    return file.good();
}

} // namespace MotionReplanner
