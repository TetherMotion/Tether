/**
 * @file Cia402MotionController.hpp
 * @brief Protocol-agnostic high-level motion controller built on tether::common::IAxis
 *
 * @details
 * This is the motion_control layer implementation of tether::common::IMotionController.
 * It coordinates one or more tether::common::IAxis instances and reuses the CiA 402
 * multi-axis path geometry for coordinated motion.
 */

#pragma once

#include "tether/common/IMotionController.hpp"
#include "tether/common/IAxis.hpp"
#include "tether/common/MotionProfile.hpp"
#include "tether/profiles/cia402/MultiAxisPath.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace tether::motion {

/**
 * @brief Multi-axis coordinated motion controller
 */
class Cia402MotionController : public tether::common::IMotionController {
public:
    using AxisId = tether::common::IAxis::AxisId;
    using AxisMap = std::map<AxisId, std::shared_ptr<tether::common::IAxis>>;

    /**
     * @brief Global motion parameters
     */
    struct GlobalParams {
        float speedFactor{1.0f};        // Global speed override (0.0 to 2.0)
        bool allowNegativeSpeed{false}; // Allow negative speed factor (reverse)
        float minSpeedFactor{0.0f};
        float maxSpeedFactor{2.0f};
    };

    Cia402MotionController();
    ~Cia402MotionController() override;

    // ========================================================================
    // Axis Management
    // ========================================================================

    /**
     * @brief Add an axis to the controller
     */
    std::shared_ptr<tether::common::IAxis> addAxis(AxisId id, std::shared_ptr<tether::common::IAxis> axis);

    /**
     * @brief Get axis by ID
     */
    std::shared_ptr<tether::common::IAxis> getAxis(AxisId id);

    /**
     * @brief Get all axes
     */
    const AxisMap& getAxes() const { return m_axes; }

    /**
     * @brief Remove axis
     */
    bool removeAxis(AxisId id);

    /**
     * @brief Get number of axes
     */
    size_t getAxisCount() const { return m_axes.size(); }

    // ========================================================================
    // Group Operations
    // ========================================================================

    bool enableAll(uint32_t timeoutMs = 5000) override;
    bool disableAll(uint32_t timeoutMs = 5000) override;

    /**
     * @brief Quick stop all axes
     */
    void quickStopAll();

    /**
     * @brief Clear faults on all axes
     */
    void clearAllFaults();

    /**
     * @brief Check if all axes are enabled
     */
    bool allEnabled() const;

    /**
     * @brief Check if any axis has fault
     */
    bool anyFault() const;

    // ========================================================================
    // Multi-Axis Coordinated Motion
    // ========================================================================

    /**
     * @brief Set axes for coordinated motion
     */
    void setCoordinatedAxes(const std::vector<AxisId>& axisIds);

    bool moveLinear(const std::vector<double>& targetPositions, double velocity);
    bool moveCircular(double centerX, double centerY, double endX, double endY,
                      bool clockwise, double velocity);
    bool moveHelical(double centerX, double centerY, double endX, double endY,
                     double pitch, bool clockwise, double velocity);

    bool executePath(CiA402::MultiSegmentPath& path, double velocity);
    bool addPathSegment(std::unique_ptr<CiA402::PathSegment> segment);
    bool startPath(double velocity);
    void stopPath();
    bool isPathExecuting() const { return m_pathExecuting; }

    // ========================================================================
    // Global Parameters
    // ========================================================================

    void setSpeedFactor(float factor);
    float getSpeedFactor() const { return m_globalParams.speedFactor; }
    void setGlobalParams(const GlobalParams& params);
    const GlobalParams& getGlobalParams() const { return m_globalParams; }

    // ========================================================================
    // Homing
    // ========================================================================

    bool homeAxis(AxisId id, const tether::common::HomingCommand& cmd);
    bool homeAxesSequential(const std::vector<AxisId>& ids,
                            const tether::common::HomingCommand& cmd);
    bool homeAxesSimultaneous(const std::vector<AxisId>& ids,
                              const tether::common::HomingCommand& cmd);
    bool allHomed() const;

    // ========================================================================
    // Cycle Update
    // ========================================================================

    void update(double dtSeconds) override;

    void setCycleTimeUs(uint32_t cycleTimeUs) { m_cycleTimeUs = cycleTimeUs; }
    uint32_t getCycleTimeUs() const { return m_cycleTimeUs; }

private:
    void updatePathExecution(double dt);
    void applyPathPoint(const CiA402::PathPoint& point);

    AxisMap m_axes;
    GlobalParams m_globalParams;

    std::vector<AxisId> m_coordinatedAxes;
    std::unique_ptr<CiA402::MultiSegmentPath> m_activePath;
    std::unique_ptr<CiA402::PathSampler> m_pathSampler;
    double m_pathTime{0.0};
    bool m_pathExecuting{false};

    uint32_t m_cycleTimeUs{1000}; // 1ms default

    mutable std::mutex m_mutex;
};

} // namespace tether::motion
