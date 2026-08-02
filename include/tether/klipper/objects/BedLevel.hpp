/**
 * @file BedLevel.hpp
 * @brief Bed leveling and mesh compensation.
 *
 * Provides:
 *   - BedMesh: 2D mesh of Z offsets for bed leveling
 *   - MeshPoint: individual mesh probe point
 *   - Bilinear interpolation for Z compensation
 *   - Mesh save/load (in-memory)
 *
 * @par Klipper-specific scope
 * This module is **Klipper-specific** and lives in the
 * `tether::klipper::objects` namespace. It is not promoted to a shared
 * `tether/compensation/` directory because bed mesh leveling is a
 * 3D-printer-specific concept with no equivalent in the main Tether
 * RS274/NGC CNC interpreter.
 *
 * The main Tether interpreter handles machine compensation through:
 * - Tool compensation (G40-G42, G43-G49) in `tether/gcode/motion/GCodeToolComp.hpp`
 * - Volumetric compensation (3D error grid) in `tether/gcode/motion/GCodeAdvancedMotion.hpp`
 * - Backlash compensation (per-axis) in `tether/gcode/motion/GCodeAdvancedMotion.hpp`
 *
 * The additional Klipper leveling methods (ZTilt, QuadGantryLevel,
 * ScrewsTiltAdjust, BedScrews, BedTilt, DeltaCalibrate) are implemented
 * as extended Klipper commands in KlippyInstanceExtendedCommands.ipp,
 * not as part of this module. Only BedMesh (the 2D mesh with bilinear
 * interpolation) is provided here.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace tether::klipper::objects {

/// @brief A single mesh probe point.
struct MeshPoint {
    double x, y, z;
};

/// @brief Bed mesh for Z compensation.
class BedMesh {
public:
    /// @brief Configure mesh dimensions.
    void configure(double minX, double maxX, double minY, double maxY,
                   int xPoints, int yPoints) {
        minX_ = minX; maxX_ = maxX;
        minY_ = minY; maxY_ = maxY;
        xPoints_ = xPoints; yPoints_ = yPoints;
        mesh_.assign(xPoints, std::vector<double>(yPoints, 0.0));
        probed_.assign(xPoints, std::vector<bool>(yPoints, false));
    }

    /// @brief Set a mesh point value.
    void setPoint(int xIdx, int yIdx, double z) {
        if (xIdx >= 0 && xIdx < xPoints_ && yIdx >= 0 && yIdx < yPoints_) {
            mesh_[xIdx][yIdx] = z;
            probed_[xIdx][yIdx] = true;
        }
    }

    /// @brief Get Z compensation at a given XY position using bilinear interpolation.
    double compensationAt(double x, double y) const {
        if (xPoints_ < 2 || yPoints_ < 2) return 0.0;
        if (x < minX_ || x > maxX_ || y < minY_ || y > maxY_) return 0.0;

        // Calculate fractional indices
        double fx = static_cast<double>(xPoints_ - 1) * (x - minX_) / (maxX_ - minX_);
        double fy = static_cast<double>(yPoints_ - 1) * (y - minY_) / (maxY_ - minY_);

        int x0 = static_cast<int>(fx);
        int y0 = static_cast<int>(fy);
        int x1 = std::min(x0 + 1, xPoints_ - 1);
        int y1 = std::min(y0 + 1, yPoints_ - 1);

        double tx = fx - x0;
        double ty = fy - y0;

        // Bilinear interpolation
        double z00 = mesh_[x0][y0];
        double z01 = mesh_[x0][y1];
        double z10 = mesh_[x1][y0];
        double z11 = mesh_[x1][y1];

        double z0 = z00 * (1 - tx) + z10 * tx;
        double z1 = z01 * (1 - tx) + z11 * tx;
        return z0 * (1 - ty) + z1 * ty;
    }

    /// @brief Check if all points have been probed.
    bool isComplete() const {
        for (int i = 0; i < xPoints_; ++i) {
            for (int j = 0; j < yPoints_; ++j) {
                if (!probed_[i][j]) return false;
            }
        }
        return true;
    }

    /// @brief Clear the mesh.
    void clear() {
        std::ranges::for_each(mesh_, [](auto& col) { std::ranges::fill(col, 0.0); });
        std::ranges::for_each(probed_, [](auto& col) { std::ranges::fill(col, false); });
    }

    /// @brief Get mesh dimensions.
    int xPoints() const { return xPoints_; }
    int yPoints() const { return yPoints_; }
    double minX() const { return minX_; }
    double maxX() const { return maxX_; }
    double minY() const { return minY_; }
    double maxY() const { return maxY_; }

    /// @brief Get all mesh points as a flat list.
    std::vector<MeshPoint> points() const {
        std::vector<MeshPoint> result;
        for (int i = 0; i < xPoints_; ++i) {
            for (int j = 0; j < yPoints_; ++j) {
                double x = minX_ + static_cast<double>(i) * (maxX_ - minX_) / (xPoints_ - 1);
                double y = minY_ + static_cast<double>(j) * (maxY_ - minY_) / (yPoints_ - 1);
                result.push_back({x, y, mesh_[i][j]});
            }
        }
        return result;
    }

    /// @brief Get mesh as a 2D matrix (for serialization).
    const std::vector<std::vector<double>>& matrix() const { return mesh_; }

private:
    double minX_ = 0, maxX_ = 200, minY_ = 0, maxY_ = 200;
    int xPoints_ = 0, yPoints_ = 0;
    std::vector<std::vector<double>> mesh_;
    std::vector<std::vector<bool>> probed_;
};

/// @brief Bed leveling controller that manages mesh probing and compensation.
class BedLevelController {
public:
    using ProbeFunc = std::function<double(double x, double y)>;

    BedLevelController(BedMesh& mesh) : mesh_(mesh) {}

    /// @brief Probe the mesh at all points.
    void probeMesh(ProbeFunc probeFunc) {
        auto pts = mesh_.points();
        for (const auto& p : pts) {
            double z = probeFunc(p.x, p.y);
            // Find indices
            double fx = static_cast<double>(mesh_.xPoints() - 1) *
                (p.x - mesh_.minX()) / (mesh_.maxX() - mesh_.minX());
            double fy = static_cast<double>(mesh_.yPoints() - 1) *
                (p.y - mesh_.minY()) / (mesh_.maxY() - mesh_.minY());
            int xi = static_cast<int>(std::round(fx));
            int yi = static_cast<int>(std::round(fy));
            mesh_.setPoint(xi, yi, z);
        }
    }

    /// @brief Get Z compensation at a position.
    double compensationAt(double x, double y) const {
        return mesh_.compensationAt(x, y);
    }

    /// @brief Apply compensation to a target Z position.
    double applyCompensation(double x, double y, double z) const {
        return z + compensationAt(x, y);
    }

private:
    BedMesh& mesh_;
};

} // namespace tether::klipper::objects
