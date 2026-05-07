/**
 * @file PerformanceHeatmap.hpp
 * @brief Performance limit heatmaps in 1D, 2D, and 3D
 * 
 * Features:
 * - Per-axis 1D heatmaps showing achievable limits along each axis
 * - 2D Cartesian heatmaps for XY, XZ, YZ planes
 * - 3D voxel-based heatmaps for full workspace analysis
 * - Dynamic building from live performance data
 * - Differential heatmaps (actual vs expected)
 * - Export to various formats for visualization
 */

#pragma once

#include <vector>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <optional>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief Performance metrics at a spatial location
 */
struct PerformanceMetrics {
    double maxAchievedVelocity = 0.0;       ///< mm/min
    double maxAchievedAccel = 0.0;          ///< mm/s²
    double maxAchievedJerk = 0.0;           ///< mm/s³
    
    double meanTrackingError = 0.0;          ///< mm
    double maxTrackingError = 0.0;           ///< mm
    double contourErrorAt95Pct = 0.0;        ///< 95th percentile contour error
    
    double meanFeedRateRatio = 1.0;          ///< achieved/commanded
    
    size_t sampleCount = 0;
    double lastUpdateTime = 0.0;
    
    // Confidence based on sample count and recency
    double confidence() const {
        if (sampleCount == 0) return 0.0;
        double countFactor = std::min(1.0, sampleCount / 100.0);
        return countFactor;
    }
};

/**
 * @brief Suggested limits at a spatial location
 */
struct SuggestedLimits {
    double maxVelocity = 10000.0;            ///< mm/min
    double maxAcceleration = 5000.0;         ///< mm/s²
    double maxJerk = 50000.0;                ///< mm/s³
    
    double confidence = 0.0;                  ///< 0-1
    std::string limitingFactor;               ///< What's constraining
};

/**
 * @brief Configuration for heatmap generation
 */
struct HeatmapConfig {
    // Workspace bounds
    std::array<double, 3> minBounds = {-500.0, -500.0, 0.0};
    std::array<double, 3> maxBounds = {500.0, 500.0, 400.0};
    
    // Resolution
    double resolution1D = 10.0;               ///< mm for 1D heatmaps
    double resolution2D = 20.0;               ///< mm for 2D heatmaps
    double resolution3D = 50.0;               ///< mm for 3D heatmaps
    
    // Default limits (user configured maximum)
    double defaultMaxVelocity = 10000.0;      ///< mm/min
    double defaultMaxAccel = 5000.0;          ///< mm/s²
    double defaultMaxJerk = 50000.0;          ///< mm/s³
    
    // Safety margin for suggested limits
    double safetyMargin = 0.1;                ///< 10%
    
    // Minimum samples needed for valid data
    size_t minSamplesForValidity = 5;
    
    // Enable differential mode
    bool differentialMode = false;
    
    // Expected performance (for differential mode)
    double expectedVelocity = 6000.0;
    double expectedAccel = 1000.0;
};

//=============================================================================
// 1D Per-Axis Heatmap
//=============================================================================

/**
 * @brief 1D heatmap along a single axis
 */
class Heatmap1D {
public:
    explicit Heatmap1D(int axis = 0, const HeatmapConfig& config = {});
    
    /**
     * @brief Add a performance sample at a position
     */
    void addSample(double position, double velocity, double accel, 
                   double trackingError, double feedRateRatio);
    
    /**
     * @brief Get performance metrics at a position
     */
    PerformanceMetrics getMetrics(double position) const;
    
    /**
     * @brief Get suggested limits at a position
     */
    SuggestedLimits getSuggestedLimits(double position) const;
    
    /**
     * @brief Get all bin data for export
     */
    struct BinData {
        double position;
        double width;
        PerformanceMetrics metrics;
        SuggestedLimits suggested;
    };
    std::vector<BinData> getAllBins() const;
    
    /**
     * @brief Get differential data (actual - expected)
     */
    struct DifferentialData {
        double position;
        double velocityDiff;          ///< actual - expected
        double accelDiff;
        double performanceRatio;      ///< actual / expected
    };
    std::vector<DifferentialData> getDifferential() const;
    
    /**
     * @brief Clear all data
     */
    void clear();
    
    int axis() const { return axis_; }
    void configure(const HeatmapConfig& config) { config_ = config; rebuildBins(); }
    
private:
    int axis_;
    HeatmapConfig config_;
    std::vector<PerformanceMetrics> bins_;
    
    void rebuildBins();
    size_t positionToBin(double pos) const;
    double binToPosition(size_t bin) const;
};

//=============================================================================
// 2D Cartesian Heatmap
//=============================================================================

/**
 * @brief 2D heatmap for a Cartesian plane
 */
class Heatmap2D {
public:
    enum class Plane { XY, XZ, YZ };
    
    explicit Heatmap2D(Plane plane = Plane::XY, const HeatmapConfig& config = {});
    
    /**
     * @brief Add a performance sample at a 2D position
     */
    void addSample(double u, double v, double velocity, double accel,
                   double trackingError, double feedRateRatio);
    
    /**
     * @brief Add sample with full 3D position (projected to plane)
     */
    void addSample3D(const std::array<double, 3>& position,
                     double velocity, double accel, double trackingError,
                     double feedRateRatio);
    
    /**
     * @brief Get performance metrics at a position
     */
    PerformanceMetrics getMetrics(double u, double v) const;
    
    /**
     * @brief Get suggested limits at a position
     */
    SuggestedLimits getSuggestedLimits(double u, double v) const;
    
    /**
     * @brief Get interpolated metrics at exact position
     */
    PerformanceMetrics getInterpolatedMetrics(double u, double v) const;
    
    /**
     * @brief Get all cell data for export
     */
    struct CellData {
        double u, v;                  ///< Center position
        double width, height;
        PerformanceMetrics metrics;
        SuggestedLimits suggested;
    };
    std::vector<CellData> getAllCells() const;
    
    /**
     * @brief Export as image-friendly format (row-major 2D array)
     */
    struct GridData {
        std::vector<double> data;     ///< Row-major values
        size_t rows, cols;
        double minU, maxU, minV, maxV;
    };
    GridData getVelocityGrid() const;
    GridData getAccelGrid() const;
    GridData getErrorGrid() const;
    
    void clear();
    
    Plane plane() const { return plane_; }
    void configure(const HeatmapConfig& config) { config_ = config; rebuildGrid(); }
    
private:
    Plane plane_;
    HeatmapConfig config_;
    std::vector<std::vector<PerformanceMetrics>> grid_;
    size_t rowCount_ = 0, colCount_ = 0;
    
    void rebuildGrid();
    std::pair<size_t, size_t> positionToCell(double u, double v) const;
    std::pair<double, double> cellToPosition(size_t row, size_t col) const;
    std::pair<int, int> getPlaneAxes() const;
};

//=============================================================================
// 3D Voxel Heatmap
//=============================================================================

/**
 * @brief 3D voxel-based heatmap for full workspace
 */
class Heatmap3D {
public:
    explicit Heatmap3D(const HeatmapConfig& config = {});
    
    /**
     * @brief Add a performance sample at a 3D position
     */
    void addSample(const std::array<double, 3>& position,
                   double velocity, double accel, double trackingError,
                   double feedRateRatio);
    
    /**
     * @brief Get performance metrics at a position
     */
    PerformanceMetrics getMetrics(const std::array<double, 3>& position) const;
    
    /**
     * @brief Get suggested limits at a position
     */
    SuggestedLimits getSuggestedLimits(const std::array<double, 3>& position) const;
    
    /**
     * @brief Get trilinearly interpolated metrics
     */
    PerformanceMetrics getInterpolatedMetrics(const std::array<double, 3>& position) const;
    
    /**
     * @brief Get all voxel data for export
     */
    struct VoxelData {
        std::array<double, 3> position;  ///< Center
        std::array<double, 3> size;      ///< Dimensions
        PerformanceMetrics metrics;
        SuggestedLimits suggested;
    };
    std::vector<VoxelData> getAllVoxels() const;
    
    /**
     * @brief Get only populated voxels (sparse representation)
     */
    std::vector<VoxelData> getPopulatedVoxels() const;
    
    /**
     * @brief Get a 2D slice at constant Z
     */
    Heatmap2D getSliceAtZ(double z) const;
    
    /**
     * @brief Export as 3D array
     */
    struct VolumeData {
        std::vector<double> data;     ///< X-major, then Y, then Z
        size_t nx, ny, nz;
        std::array<double, 3> origin;
        std::array<double, 3> spacing;
    };
    VolumeData getVelocityVolume() const;
    VolumeData getErrorVolume() const;
    
    void clear();
    void configure(const HeatmapConfig& config) { config_ = config; rebuildGrid(); }
    
private:
    HeatmapConfig config_;
    
    // Sparse storage using map for memory efficiency
    std::map<std::tuple<size_t, size_t, size_t>, PerformanceMetrics> voxels_;
    size_t nx_ = 0, ny_ = 0, nz_ = 0;
    
    void rebuildGrid();
    std::tuple<size_t, size_t, size_t> positionToVoxel(
        const std::array<double, 3>& pos) const;
    std::array<double, 3> voxelToPosition(size_t ix, size_t iy, size_t iz) const;
};

//=============================================================================
// Differential Heatmap
//=============================================================================

/**
 * @brief Differential heatmap comparing actual vs expected performance
 */
class DifferentialHeatmap {
public:
    using VoxelData = Heatmap3D::VoxelData;
    DifferentialHeatmap(const HeatmapConfig& config = {});
    
    /**
     * @brief Set expected performance model
     */
    void setExpectedPerformance(double velocity, double accel, double jerk) {
        expectedVelocity_ = velocity;
        expectedAccel_ = accel;
        expectedJerk_ = jerk;
    }
    
    /**
     * @brief Set position-dependent expected performance
     */
    using PerformanceFunction = std::function<PerformanceMetrics(
        const std::array<double, 3>&)>;
    void setExpectedFunction(PerformanceFunction func) {
        expectedFunc_ = func;
    }
    
    /**
     * @brief Add actual performance sample
     */
    void addActualSample(const std::array<double, 3>& position,
                         double velocity, double accel, double trackingError);
    
    /**
     * @brief Differential at a position
     */
    struct Differential {
        double velocityRatio;         ///< actual / expected
        double accelRatio;
        double errorRatio;            ///< actual error / acceptable error
        bool underperforming;         ///< ratio < 1 for velocity/accel
        bool overErroring;            ///< errorRatio > 1
    };
    
    Differential getDifferential(const std::array<double, 3>& position) const;
    
    /**
     * @brief Get problem areas (where actual << expected)
     */
    std::vector<VoxelData> getProblemAreas(double threshold = 0.8) const;
    
    /**
     * @brief Export differential grid for visualization
     */
    Heatmap2D::GridData getVelocityRatioGrid(Heatmap2D::Plane plane, double slice) const;
    
private:
    HeatmapConfig config_;
    Heatmap3D actualHeatmap_;
    
    double expectedVelocity_ = 6000.0;
    double expectedAccel_ = 1000.0;
    double expectedJerk_ = 10000.0;
    PerformanceFunction expectedFunc_;
    
};

//=============================================================================
// Heatmap Builder (Online)
//=============================================================================

/**
 * @brief Builds heatmaps online from live trajectory data
 */
class HeatmapBuilder {
public:
    explicit HeatmapBuilder(const HeatmapConfig& config = {});
    
    /**
     * @brief Process a trajectory sample with error info
     */
    void processSample(const std::array<double, 9>& position,
                       const std::array<double, 9>& velocity,
                       double trackingError,
                       double contourError,
                       double commandedFeedRate,
                       double actualFeedRate);
    
    /**
     * @brief Get the built heatmaps
     */
    const std::array<Heatmap1D, 9>& getAxisHeatmaps() const { return axisHeatmaps_; }
    const Heatmap2D& getXYHeatmap() const { return xyHeatmap_; }
    const Heatmap2D& getXZHeatmap() const { return xzHeatmap_; }
    const Heatmap2D& getYZHeatmap() const { return yzHeatmap_; }
    const Heatmap3D& get3DHeatmap() const { return heatmap3D_; }
    
    /**
     * @brief Get mutable heatmaps for direct manipulation
     */
    std::array<Heatmap1D, 9>& axisHeatmaps() { return axisHeatmaps_; }
    Heatmap2D& xyHeatmap() { return xyHeatmap_; }
    Heatmap2D& xzHeatmap() { return xzHeatmap_; }
    Heatmap2D& yzHeatmap() { return yzHeatmap_; }
    Heatmap3D& heatmap3D() { return heatmap3D_; }
    
    /**
     * @brief Clear all heatmaps
     */
    void clear();
    
    void configure(const HeatmapConfig& config);
    
private:
    HeatmapConfig config_;
    
    std::array<Heatmap1D, 9> axisHeatmaps_;
    Heatmap2D xyHeatmap_;
    Heatmap2D xzHeatmap_;
    Heatmap2D yzHeatmap_;
    Heatmap3D heatmap3D_;
};

} // namespace MotionReplanner
