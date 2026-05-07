/**
 * @file PerformanceHeatmap.cpp
 * @brief Implementation of performance limit heatmaps
 */

#include "PerformanceHeatmap.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MotionReplanner {

//=============================================================================
// Heatmap1D Implementation
//=============================================================================

Heatmap1D::Heatmap1D(int axis, const HeatmapConfig& config)
    : axis_(axis), config_(config) {
    rebuildBins();
}

void Heatmap1D::rebuildBins() {
    double range = config_.maxBounds[axis_] - config_.minBounds[axis_];
    size_t numBins = static_cast<size_t>(std::ceil(range / config_.resolution1D));
    bins_.clear();
    bins_.resize(numBins);
}

size_t Heatmap1D::positionToBin(double pos) const {
    double normalized = (pos - config_.minBounds[axis_]) / config_.resolution1D;
    size_t bin = static_cast<size_t>(std::max(0.0, normalized));
    return std::min(bin, bins_.size() - 1);
}

double Heatmap1D::binToPosition(size_t bin) const {
    return config_.minBounds[axis_] + (bin + 0.5) * config_.resolution1D;
}

void Heatmap1D::addSample(double position, double velocity, double accel,
                          double trackingError, double feedRateRatio) {
    if (bins_.empty()) return;
    
    size_t bin = positionToBin(position);
    auto& metrics = bins_[bin];
    
    metrics.sampleCount++;
    double n = static_cast<double>(metrics.sampleCount);
    
    // Update maximums
    metrics.maxAchievedVelocity = std::max(metrics.maxAchievedVelocity, velocity);
    metrics.maxAchievedAccel = std::max(metrics.maxAchievedAccel, accel);
    metrics.maxTrackingError = std::max(metrics.maxTrackingError, trackingError);
    
    // Update means using Welford's algorithm
    double deltaError = trackingError - metrics.meanTrackingError;
    metrics.meanTrackingError += deltaError / n;
    
    double deltaRatio = feedRateRatio - metrics.meanFeedRateRatio;
    metrics.meanFeedRateRatio += deltaRatio / n;
}

PerformanceMetrics Heatmap1D::getMetrics(double position) const {
    if (bins_.empty()) return {};
    size_t bin = positionToBin(position);
    return bins_[bin];
}

SuggestedLimits Heatmap1D::getSuggestedLimits(double position) const {
    SuggestedLimits suggested;
    
    if (bins_.empty()) {
        suggested.maxVelocity = config_.defaultMaxVelocity;
        suggested.maxAcceleration = config_.defaultMaxAccel;
        suggested.maxJerk = config_.defaultMaxJerk;
        return suggested;
    }
    
    const auto& metrics = bins_[positionToBin(position)];
    
    if (metrics.sampleCount < config_.minSamplesForValidity) {
        suggested.maxVelocity = config_.defaultMaxVelocity;
        suggested.maxAcceleration = config_.defaultMaxAccel;
        suggested.maxJerk = config_.defaultMaxJerk;
        suggested.confidence = 0.0;
        suggested.limitingFactor = "Insufficient data";
        return suggested;
    }
    
    // Suggest limits based on achieved performance with safety margin
    double safetyFactor = 1.0 - config_.safetyMargin;
    
    suggested.maxVelocity = metrics.maxAchievedVelocity * safetyFactor;
    suggested.maxAcceleration = metrics.maxAchievedAccel * safetyFactor;
    
    // Reduce further if tracking error was high
    if (metrics.meanTrackingError > 0.01) { // 10 microns
        double errorFactor = 0.01 / metrics.meanTrackingError;
        errorFactor = std::max(0.5, std::min(1.0, errorFactor));
        suggested.maxVelocity *= errorFactor;
        suggested.maxAcceleration *= errorFactor;
        suggested.limitingFactor = "Tracking error";
    } else {
        suggested.limitingFactor = "Achieved performance";
    }
    
    suggested.confidence = metrics.confidence();
    
    // Clamp to configured maximum
    suggested.maxVelocity = std::min(suggested.maxVelocity, config_.defaultMaxVelocity);
    suggested.maxAcceleration = std::min(suggested.maxAcceleration, config_.defaultMaxAccel);
    suggested.maxJerk = config_.defaultMaxJerk;
    
    return suggested;
}

std::vector<Heatmap1D::BinData> Heatmap1D::getAllBins() const {
    std::vector<BinData> result;
    result.reserve(bins_.size());
    
    for (size_t i = 0; i < bins_.size(); ++i) {
        BinData data;
        data.position = binToPosition(i);
        data.width = config_.resolution1D;
        data.metrics = bins_[i];
        data.suggested = getSuggestedLimits(data.position);
        result.push_back(data);
    }
    
    return result;
}

std::vector<Heatmap1D::DifferentialData> Heatmap1D::getDifferential() const {
    std::vector<DifferentialData> result;
    result.reserve(bins_.size());
    
    for (size_t i = 0; i < bins_.size(); ++i) {
        const auto& metrics = bins_[i];
        if (metrics.sampleCount < config_.minSamplesForValidity) continue;
        
        DifferentialData diff;
        diff.position = binToPosition(i);
        diff.velocityDiff = metrics.maxAchievedVelocity - config_.expectedVelocity;
        diff.accelDiff = metrics.maxAchievedAccel - config_.expectedAccel;
        diff.performanceRatio = (config_.expectedVelocity > 0) ?
            metrics.maxAchievedVelocity / config_.expectedVelocity : 1.0;
        
        result.push_back(diff);
    }
    
    return result;
}

void Heatmap1D::clear() {
    for (auto& bin : bins_) {
        bin = PerformanceMetrics{};
    }
}

//=============================================================================
// Heatmap2D Implementation
//=============================================================================

Heatmap2D::Heatmap2D(Plane plane, const HeatmapConfig& config)
    : plane_(plane), config_(config) {
    rebuildGrid();
}

std::pair<int, int> Heatmap2D::getPlaneAxes() const {
    switch (plane_) {
        case Plane::XY: return {0, 1};
        case Plane::XZ: return {0, 2};
        case Plane::YZ: return {1, 2};
        default: return {0, 1};
    }
}

void Heatmap2D::rebuildGrid() {
    auto [uAxis, vAxis] = getPlaneAxes();
    
    double uRange = config_.maxBounds[uAxis] - config_.minBounds[uAxis];
    double vRange = config_.maxBounds[vAxis] - config_.minBounds[vAxis];
    
    colCount_ = static_cast<size_t>(std::ceil(uRange / config_.resolution2D));
    rowCount_ = static_cast<size_t>(std::ceil(vRange / config_.resolution2D));
    
    grid_.clear();
    grid_.resize(rowCount_, std::vector<PerformanceMetrics>(colCount_));
}

std::pair<size_t, size_t> Heatmap2D::positionToCell(double u, double v) const {
    auto [uAxis, vAxis] = getPlaneAxes();
    
    double uNorm = (u - config_.minBounds[uAxis]) / config_.resolution2D;
    double vNorm = (v - config_.minBounds[vAxis]) / config_.resolution2D;
    
    size_t col = static_cast<size_t>(std::max(0.0, uNorm));
    size_t row = static_cast<size_t>(std::max(0.0, vNorm));
    
    col = std::min(col, colCount_ - 1);
    row = std::min(row, rowCount_ - 1);
    
    return {row, col};
}

std::pair<double, double> Heatmap2D::cellToPosition(size_t row, size_t col) const {
    auto [uAxis, vAxis] = getPlaneAxes();
    
    double u = config_.minBounds[uAxis] + (col + 0.5) * config_.resolution2D;
    double v = config_.minBounds[vAxis] + (row + 0.5) * config_.resolution2D;
    
    return {u, v};
}

void Heatmap2D::addSample(double u, double v, double velocity, double accel,
                          double trackingError, double feedRateRatio) {
    if (grid_.empty()) return;
    
    auto [row, col] = positionToCell(u, v);
    auto& metrics = grid_[row][col];
    
    metrics.sampleCount++;
    double n = static_cast<double>(metrics.sampleCount);
    
    metrics.maxAchievedVelocity = std::max(metrics.maxAchievedVelocity, velocity);
    metrics.maxAchievedAccel = std::max(metrics.maxAchievedAccel, accel);
    metrics.maxTrackingError = std::max(metrics.maxTrackingError, trackingError);
    
    double deltaError = trackingError - metrics.meanTrackingError;
    metrics.meanTrackingError += deltaError / n;
    
    double deltaRatio = feedRateRatio - metrics.meanFeedRateRatio;
    metrics.meanFeedRateRatio += deltaRatio / n;
}

void Heatmap2D::addSample3D(const std::array<double, 3>& position,
                            double velocity, double accel, double trackingError,
                            double feedRateRatio) {
    auto [uAxis, vAxis] = getPlaneAxes();
    addSample(position[uAxis], position[vAxis], velocity, accel, 
              trackingError, feedRateRatio);
}

PerformanceMetrics Heatmap2D::getMetrics(double u, double v) const {
    if (grid_.empty()) return {};
    auto [row, col] = positionToCell(u, v);
    return grid_[row][col];
}

SuggestedLimits Heatmap2D::getSuggestedLimits(double u, double v) const {
    SuggestedLimits suggested;
    
    if (grid_.empty()) {
        suggested.maxVelocity = config_.defaultMaxVelocity;
        suggested.maxAcceleration = config_.defaultMaxAccel;
        suggested.maxJerk = config_.defaultMaxJerk;
        return suggested;
    }
    
    auto [row, col] = positionToCell(u, v);
    const auto& metrics = grid_[row][col];
    
    if (metrics.sampleCount < config_.minSamplesForValidity) {
        suggested.maxVelocity = config_.defaultMaxVelocity;
        suggested.maxAcceleration = config_.defaultMaxAccel;
        suggested.maxJerk = config_.defaultMaxJerk;
        suggested.confidence = 0.0;
        return suggested;
    }
    
    double safetyFactor = 1.0 - config_.safetyMargin;
    
    suggested.maxVelocity = metrics.maxAchievedVelocity * safetyFactor;
    suggested.maxAcceleration = metrics.maxAchievedAccel * safetyFactor;
    suggested.maxJerk = config_.defaultMaxJerk;
    suggested.confidence = metrics.confidence();
    
    suggested.maxVelocity = std::min(suggested.maxVelocity, config_.defaultMaxVelocity);
    suggested.maxAcceleration = std::min(suggested.maxAcceleration, config_.defaultMaxAccel);
    
    return suggested;
}

PerformanceMetrics Heatmap2D::getInterpolatedMetrics(double u, double v) const {
    if (grid_.empty()) return {};
    
    auto [uAxis, vAxis] = getPlaneAxes();
    
    // Get fractional cell position
    double uNorm = (u - config_.minBounds[uAxis]) / config_.resolution2D;
    double vNorm = (v - config_.minBounds[vAxis]) / config_.resolution2D;
    
    size_t col0 = static_cast<size_t>(std::max(0.0, std::floor(uNorm)));
    size_t row0 = static_cast<size_t>(std::max(0.0, std::floor(vNorm)));
    size_t col1 = std::min(col0 + 1, colCount_ - 1);
    size_t row1 = std::min(row0 + 1, rowCount_ - 1);
    
    double tu = uNorm - std::floor(uNorm);
    double tv = vNorm - std::floor(vNorm);
    
    // Bilinear interpolation
    const auto& m00 = grid_[row0][col0];
    const auto& m01 = grid_[row0][col1];
    const auto& m10 = grid_[row1][col0];
    const auto& m11 = grid_[row1][col1];
    
    PerformanceMetrics result;
    
    auto interpolate = [&](double v00, double v01, double v10, double v11) {
        double v0 = v00 * (1.0 - tu) + v01 * tu;
        double v1 = v10 * (1.0 - tu) + v11 * tu;
        return v0 * (1.0 - tv) + v1 * tv;
    };
    
    result.maxAchievedVelocity = interpolate(
        m00.maxAchievedVelocity, m01.maxAchievedVelocity,
        m10.maxAchievedVelocity, m11.maxAchievedVelocity);
    
    result.maxAchievedAccel = interpolate(
        m00.maxAchievedAccel, m01.maxAchievedAccel,
        m10.maxAchievedAccel, m11.maxAchievedAccel);
    
    result.meanTrackingError = interpolate(
        m00.meanTrackingError, m01.meanTrackingError,
        m10.meanTrackingError, m11.meanTrackingError);
    
    result.sampleCount = static_cast<size_t>(interpolate(
        static_cast<double>(m00.sampleCount), static_cast<double>(m01.sampleCount),
        static_cast<double>(m10.sampleCount), static_cast<double>(m11.sampleCount)));
    
    return result;
}

std::vector<Heatmap2D::CellData> Heatmap2D::getAllCells() const {
    std::vector<CellData> result;
    result.reserve(rowCount_ * colCount_);
    
    for (size_t row = 0; row < rowCount_; ++row) {
        for (size_t col = 0; col < colCount_; ++col) {
            auto [u, v] = cellToPosition(row, col);
            CellData cell;
            cell.u = u;
            cell.v = v;
            cell.width = config_.resolution2D;
            cell.height = config_.resolution2D;
            cell.metrics = grid_[row][col];
            cell.suggested = getSuggestedLimits(u, v);
            result.push_back(cell);
        }
    }
    
    return result;
}

Heatmap2D::GridData Heatmap2D::getVelocityGrid() const {
    GridData grid;
    grid.rows = rowCount_;
    grid.cols = colCount_;
    grid.data.reserve(rowCount_ * colCount_);
    
    auto [uAxis, vAxis] = getPlaneAxes();
    grid.minU = config_.minBounds[uAxis];
    grid.maxU = config_.maxBounds[uAxis];
    grid.minV = config_.minBounds[vAxis];
    grid.maxV = config_.maxBounds[vAxis];
    
    for (size_t row = 0; row < rowCount_; ++row) {
        for (size_t col = 0; col < colCount_; ++col) {
            grid.data.push_back(grid_[row][col].maxAchievedVelocity);
        }
    }
    
    return grid;
}

Heatmap2D::GridData Heatmap2D::getAccelGrid() const {
    GridData grid;
    grid.rows = rowCount_;
    grid.cols = colCount_;
    grid.data.reserve(rowCount_ * colCount_);
    
    auto [uAxis, vAxis] = getPlaneAxes();
    grid.minU = config_.minBounds[uAxis];
    grid.maxU = config_.maxBounds[uAxis];
    grid.minV = config_.minBounds[vAxis];
    grid.maxV = config_.maxBounds[vAxis];
    
    for (size_t row = 0; row < rowCount_; ++row) {
        for (size_t col = 0; col < colCount_; ++col) {
            grid.data.push_back(grid_[row][col].maxAchievedAccel);
        }
    }
    
    return grid;
}

Heatmap2D::GridData Heatmap2D::getErrorGrid() const {
    GridData grid;
    grid.rows = rowCount_;
    grid.cols = colCount_;
    grid.data.reserve(rowCount_ * colCount_);
    
    auto [uAxis, vAxis] = getPlaneAxes();
    grid.minU = config_.minBounds[uAxis];
    grid.maxU = config_.maxBounds[uAxis];
    grid.minV = config_.minBounds[vAxis];
    grid.maxV = config_.maxBounds[vAxis];
    
    for (size_t row = 0; row < rowCount_; ++row) {
        for (size_t col = 0; col < colCount_; ++col) {
            grid.data.push_back(grid_[row][col].meanTrackingError);
        }
    }
    
    return grid;
}

void Heatmap2D::clear() {
    for (auto& row : grid_) {
        for (auto& cell : row) {
            cell = PerformanceMetrics{};
        }
    }
}

//=============================================================================
// Heatmap3D Implementation
//=============================================================================

Heatmap3D::Heatmap3D(const HeatmapConfig& config)
    : config_(config) {
    rebuildGrid();
}

void Heatmap3D::rebuildGrid() {
    nx_ = static_cast<size_t>(std::ceil(
        (config_.maxBounds[0] - config_.minBounds[0]) / config_.resolution3D));
    ny_ = static_cast<size_t>(std::ceil(
        (config_.maxBounds[1] - config_.minBounds[1]) / config_.resolution3D));
    nz_ = static_cast<size_t>(std::ceil(
        (config_.maxBounds[2] - config_.minBounds[2]) / config_.resolution3D));
    
    voxels_.clear();
}

std::tuple<size_t, size_t, size_t> Heatmap3D::positionToVoxel(
    const std::array<double, 3>& pos) const {
    
    auto toIndex = [&](int axis) {
        double norm = (pos[axis] - config_.minBounds[axis]) / config_.resolution3D;
        size_t idx = static_cast<size_t>(std::max(0.0, norm));
        size_t max = (axis == 0) ? nx_ : (axis == 1) ? ny_ : nz_;
        return std::min(idx, max - 1);
    };
    
    return {toIndex(0), toIndex(1), toIndex(2)};
}

std::array<double, 3> Heatmap3D::voxelToPosition(size_t ix, size_t iy, size_t iz) const {
    return {
        config_.minBounds[0] + (ix + 0.5) * config_.resolution3D,
        config_.minBounds[1] + (iy + 0.5) * config_.resolution3D,
        config_.minBounds[2] + (iz + 0.5) * config_.resolution3D
    };
}

void Heatmap3D::addSample(const std::array<double, 3>& position,
                          double velocity, double accel, double trackingError,
                          double feedRateRatio) {
    auto [ix, iy, iz] = positionToVoxel(position);
    auto key = std::make_tuple(ix, iy, iz);
    
    auto& metrics = voxels_[key];
    
    metrics.sampleCount++;
    double n = static_cast<double>(metrics.sampleCount);
    
    metrics.maxAchievedVelocity = std::max(metrics.maxAchievedVelocity, velocity);
    metrics.maxAchievedAccel = std::max(metrics.maxAchievedAccel, accel);
    metrics.maxTrackingError = std::max(metrics.maxTrackingError, trackingError);
    
    double deltaError = trackingError - metrics.meanTrackingError;
    metrics.meanTrackingError += deltaError / n;
    
    double deltaRatio = feedRateRatio - metrics.meanFeedRateRatio;
    metrics.meanFeedRateRatio += deltaRatio / n;
}

PerformanceMetrics Heatmap3D::getMetrics(const std::array<double, 3>& position) const {
    auto key = positionToVoxel(position);
    auto it = voxels_.find(key);
    if (it != voxels_.end()) {
        return it->second;
    }
    return {};
}

SuggestedLimits Heatmap3D::getSuggestedLimits(const std::array<double, 3>& position) const {
    SuggestedLimits suggested;
    
    const auto& metrics = getMetrics(position);
    
    if (metrics.sampleCount < config_.minSamplesForValidity) {
        suggested.maxVelocity = config_.defaultMaxVelocity;
        suggested.maxAcceleration = config_.defaultMaxAccel;
        suggested.maxJerk = config_.defaultMaxJerk;
        suggested.confidence = 0.0;
        return suggested;
    }
    
    double safetyFactor = 1.0 - config_.safetyMargin;
    
    suggested.maxVelocity = metrics.maxAchievedVelocity * safetyFactor;
    suggested.maxAcceleration = metrics.maxAchievedAccel * safetyFactor;
    suggested.maxJerk = config_.defaultMaxJerk;
    suggested.confidence = metrics.confidence();
    
    suggested.maxVelocity = std::min(suggested.maxVelocity, config_.defaultMaxVelocity);
    suggested.maxAcceleration = std::min(suggested.maxAcceleration, config_.defaultMaxAccel);
    
    return suggested;
}

PerformanceMetrics Heatmap3D::getInterpolatedMetrics(
    const std::array<double, 3>& position) const {
    
    // Get fractional voxel position
    auto toFrac = [&](int axis) {
        return (position[axis] - config_.minBounds[axis]) / config_.resolution3D;
    };
    
    double fx = toFrac(0), fy = toFrac(1), fz = toFrac(2);
    
    size_t x0 = static_cast<size_t>(std::max(0.0, std::floor(fx)));
    size_t y0 = static_cast<size_t>(std::max(0.0, std::floor(fy)));
    size_t z0 = static_cast<size_t>(std::max(0.0, std::floor(fz)));
    
    size_t x1 = std::min(x0 + 1, nx_ - 1);
    size_t y1 = std::min(y0 + 1, ny_ - 1);
    size_t z1 = std::min(z0 + 1, nz_ - 1);
    
    double tx = fx - std::floor(fx);
    double ty = fy - std::floor(fy);
    double tz = fz - std::floor(fz);
    
    // Get all 8 corner voxels
    auto getVoxel = [&](size_t x, size_t y, size_t z) -> const PerformanceMetrics& {
        static PerformanceMetrics empty;
        auto it = voxels_.find(std::make_tuple(x, y, z));
        return (it != voxels_.end()) ? it->second : empty;
    };
    
    // Trilinear interpolation
    auto trilinear = [&](auto getter) {
        double c00 = getter(getVoxel(x0, y0, z0)) * (1-tx) + getter(getVoxel(x1, y0, z0)) * tx;
        double c01 = getter(getVoxel(x0, y0, z1)) * (1-tx) + getter(getVoxel(x1, y0, z1)) * tx;
        double c10 = getter(getVoxel(x0, y1, z0)) * (1-tx) + getter(getVoxel(x1, y1, z0)) * tx;
        double c11 = getter(getVoxel(x0, y1, z1)) * (1-tx) + getter(getVoxel(x1, y1, z1)) * tx;
        
        double c0 = c00 * (1-ty) + c10 * ty;
        double c1 = c01 * (1-ty) + c11 * ty;
        
        return c0 * (1-tz) + c1 * tz;
    };
    
    PerformanceMetrics result;
    result.maxAchievedVelocity = trilinear([](const auto& m) { return m.maxAchievedVelocity; });
    result.maxAchievedAccel = trilinear([](const auto& m) { return m.maxAchievedAccel; });
    result.meanTrackingError = trilinear([](const auto& m) { return m.meanTrackingError; });
    result.sampleCount = static_cast<size_t>(trilinear(
        [](const auto& m) { return static_cast<double>(m.sampleCount); }));
    
    return result;
}

std::vector<Heatmap3D::VoxelData> Heatmap3D::getAllVoxels() const {
    std::vector<VoxelData> result;
    result.reserve(nx_ * ny_ * nz_);
    
    for (size_t iz = 0; iz < nz_; ++iz) {
        for (size_t iy = 0; iy < ny_; ++iy) {
            for (size_t ix = 0; ix < nx_; ++ix) {
                VoxelData voxel;
                voxel.position = voxelToPosition(ix, iy, iz);
                voxel.size = {config_.resolution3D, config_.resolution3D, config_.resolution3D};
                
                auto it = voxels_.find(std::make_tuple(ix, iy, iz));
                if (it != voxels_.end()) {
                    voxel.metrics = it->second;
                }
                voxel.suggested = getSuggestedLimits(voxel.position);
                
                result.push_back(voxel);
            }
        }
    }
    
    return result;
}

std::vector<Heatmap3D::VoxelData> Heatmap3D::getPopulatedVoxels() const {
    std::vector<VoxelData> result;
    result.reserve(voxels_.size());
    
    for (const auto& [key, metrics] : voxels_) {
        auto [ix, iy, iz] = key;
        VoxelData voxel;
        voxel.position = voxelToPosition(ix, iy, iz);
        voxel.size = {config_.resolution3D, config_.resolution3D, config_.resolution3D};
        voxel.metrics = metrics;
        voxel.suggested = getSuggestedLimits(voxel.position);
        result.push_back(voxel);
    }
    
    return result;
}

Heatmap2D Heatmap3D::getSliceAtZ(double z) const {
    Heatmap2D slice(Heatmap2D::Plane::XY, config_);
    
    size_t iz = static_cast<size_t>(std::max(0.0,
        (z - config_.minBounds[2]) / config_.resolution3D));
    iz = std::min(iz, nz_ - 1);
    
    for (const auto& [key, metrics] : voxels_) {
        if (std::get<2>(key) == iz) {
            auto pos = voxelToPosition(std::get<0>(key), std::get<1>(key), iz);
            slice.addSample(pos[0], pos[1], 
                           metrics.maxAchievedVelocity,
                           metrics.maxAchievedAccel,
                           metrics.meanTrackingError,
                           metrics.meanFeedRateRatio);
        }
    }
    
    return slice;
}

Heatmap3D::VolumeData Heatmap3D::getVelocityVolume() const {
    VolumeData vol;
    vol.nx = nx_;
    vol.ny = ny_;
    vol.nz = nz_;
    vol.origin = config_.minBounds;
    vol.spacing = {config_.resolution3D, config_.resolution3D, config_.resolution3D};
    vol.data.resize(nx_ * ny_ * nz_, 0.0);
    
    for (const auto& [key, metrics] : voxels_) {
        auto [ix, iy, iz] = key;
        size_t idx = ix + iy * nx_ + iz * nx_ * ny_;
        vol.data[idx] = metrics.maxAchievedVelocity;
    }
    
    return vol;
}

Heatmap3D::VolumeData Heatmap3D::getErrorVolume() const {
    VolumeData vol;
    vol.nx = nx_;
    vol.ny = ny_;
    vol.nz = nz_;
    vol.origin = config_.minBounds;
    vol.spacing = {config_.resolution3D, config_.resolution3D, config_.resolution3D};
    vol.data.resize(nx_ * ny_ * nz_, 0.0);
    
    for (const auto& [key, metrics] : voxels_) {
        auto [ix, iy, iz] = key;
        size_t idx = ix + iy * nx_ + iz * nx_ * ny_;
        vol.data[idx] = metrics.meanTrackingError;
    }
    
    return vol;
}

void Heatmap3D::clear() {
    voxels_.clear();
}

//=============================================================================
// DifferentialHeatmap Implementation
//=============================================================================

DifferentialHeatmap::DifferentialHeatmap(const HeatmapConfig& config)
    : config_(config), actualHeatmap_(config) {}

void DifferentialHeatmap::addActualSample(const std::array<double, 3>& position,
                                          double velocity, double accel,
                                          double trackingError) {
    actualHeatmap_.addSample(position, velocity, accel, trackingError, 1.0);
}

DifferentialHeatmap::Differential DifferentialHeatmap::getDifferential(
    const std::array<double, 3>& position) const {
    
    Differential diff;
    
    auto actual = actualHeatmap_.getMetrics(position);
    
    double expectedVel = expectedVelocity_;
    double expectedAcc = expectedAccel_;
    
    if (expectedFunc_) {
        auto expected = expectedFunc_(position);
        expectedVel = expected.maxAchievedVelocity;
        expectedAcc = expected.maxAchievedAccel;
    }
    
    diff.velocityRatio = (expectedVel > 0) ? actual.maxAchievedVelocity / expectedVel : 1.0;
    diff.accelRatio = (expectedAcc > 0) ? actual.maxAchievedAccel / expectedAcc : 1.0;
    diff.errorRatio = actual.meanTrackingError / 0.01; // 10 micron reference
    
    diff.underperforming = diff.velocityRatio < 0.9 || diff.accelRatio < 0.9;
    diff.overErroring = diff.errorRatio > 1.0;
    
    return diff;
}

std::vector<Heatmap3D::VoxelData> DifferentialHeatmap::getProblemAreas(double threshold) const {
    std::vector<Heatmap3D::VoxelData> problems;
    
    auto voxels = actualHeatmap_.getPopulatedVoxels();
    for (const auto& voxel : voxels) {
        auto diff = getDifferential(voxel.position);
        if (diff.velocityRatio < threshold || diff.accelRatio < threshold || diff.overErroring) {
            problems.push_back(voxel);
        }
    }
    
    return problems;
}

Heatmap2D::GridData DifferentialHeatmap::getVelocityRatioGrid(
    Heatmap2D::Plane plane, double slice) const {
    
    auto slice2D = actualHeatmap_.getSliceAtZ(slice);
    auto grid = slice2D.getVelocityGrid();
    
    // Convert to ratio
    for (auto& val : grid.data) {
        val = (expectedVelocity_ > 0) ? val / expectedVelocity_ : 1.0;
    }
    
    return grid;
}

//=============================================================================
// HeatmapBuilder Implementation
//=============================================================================

HeatmapBuilder::HeatmapBuilder(const HeatmapConfig& config)
    : config_(config),
      xyHeatmap_(Heatmap2D::Plane::XY, config),
      xzHeatmap_(Heatmap2D::Plane::XZ, config),
      yzHeatmap_(Heatmap2D::Plane::YZ, config),
      heatmap3D_(config) {
    
    for (int i = 0; i < 9; ++i) {
        axisHeatmaps_[i] = Heatmap1D(i, config);
    }
}

void HeatmapBuilder::processSample(const std::array<double, 9>& position,
                                   const std::array<double, 9>& velocity,
                                   double trackingError,
                                   double contourError,
                                   double commandedFeedRate,
                                   double actualFeedRate) {
    // Calculate velocity magnitude
    double velMag = std::sqrt(
        velocity[0] * velocity[0] +
        velocity[1] * velocity[1] +
        velocity[2] * velocity[2]
    ) * 60.0; // Convert to mm/min
    
    // Calculate acceleration (would need previous velocity, simplified here)
    double accel = 0.0; // Would need to compute from velocity difference
    
    double feedRatio = (commandedFeedRate > 0) ? actualFeedRate / commandedFeedRate : 1.0;
    
    // Update per-axis heatmaps
    for (int i = 0; i < 9; ++i) {
        axisHeatmaps_[i].addSample(position[i], std::abs(velocity[i]) * 60.0,
                                   accel, trackingError, feedRatio);
    }
    
    // Update 2D heatmaps
    std::array<double, 3> pos3D = {position[0], position[1], position[2]};
    
    xyHeatmap_.addSample3D(pos3D, velMag, accel, contourError, feedRatio);
    xzHeatmap_.addSample3D(pos3D, velMag, accel, contourError, feedRatio);
    yzHeatmap_.addSample3D(pos3D, velMag, accel, contourError, feedRatio);
    
    // Update 3D heatmap
    heatmap3D_.addSample(pos3D, velMag, accel, contourError, feedRatio);
}

void HeatmapBuilder::clear() {
    for (auto& hm : axisHeatmaps_) {
        hm.clear();
    }
    xyHeatmap_.clear();
    xzHeatmap_.clear();
    yzHeatmap_.clear();
    heatmap3D_.clear();
}

void HeatmapBuilder::configure(const HeatmapConfig& config) {
    config_ = config;
    
    for (int i = 0; i < 9; ++i) {
        axisHeatmaps_[i].configure(config);
    }
    xyHeatmap_.configure(config);
    xzHeatmap_.configure(config);
    yzHeatmap_.configure(config);
    heatmap3D_.configure(config);
}

} // namespace MotionReplanner
