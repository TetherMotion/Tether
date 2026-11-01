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
     * @brief Register an axis under a logical ID.
     *
     * The controller shares ownership via the shared_ptr — destroying the
     * caller's handle does not detach the axis. Registering an ID that is
     * already in use replaces the previous axis; coordinated-motion and
     * homing calls refer to axes by this ID.
     *
     * @param id    Logical axis identifier used by getAxis() and the
     *              move/home APIs.
     * @param axis  Axis implementation to register; must not be null.
     * @return The axis that was registered (same object as @p axis).
     */
    std::shared_ptr<tether::common::IAxis> addAxis(AxisId id, std::shared_ptr<tether::common::IAxis> axis);

    /**
     * @brief Look up a registered axis by ID.
     *
     * @param id  Logical axis identifier passed to addAxis().
     * @return Shared pointer to the axis, or nullptr if no axis is
     *         registered under @p id.
     */
    std::shared_ptr<tether::common::IAxis> getAxis(AxisId id);

    /**
     * @brief All registered axes keyed by ID.
     *
     * @return Const reference to the internal map — stable only until the
     *         next addAxis()/removeAxis() call; do not hold it across
     *         axis-management operations.
     */
    const AxisMap& getAxes() const { return m_axes; }

    /**
     * @brief Remove axis
     *
     * Erases the ID from the axis map only — a previously configured
     * coordinated list keeps the ID, but it resolves to no axis in future
     * moves. The axis object itself survives in any other shared_ptr
     * owner.
     *
     * @param id  Logical axis identifier to remove.
     * @return true if the axis existed and was removed.
     */
    bool removeAxis(AxisId id);

    /**
     * @brief Number of registered axes.
     *
     * @return Count of IDs currently in the axis map.
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
     *
     * Vacuously true when no axes are registered.
     *
     * @return true if every registered axis reports enabled.
     */
    bool allEnabled() const;

    /**
     * @brief Check if any axis has fault
     *
     * @return true if at least one registered axis reports a fault state.
     */
    bool anyFault() const;

    // ========================================================================
    // Multi-Axis Coordinated Motion
    // ========================================================================

    /**
     * @brief Select which axes participate in coordinated moves.
     *
     * The listed IDs define the axis order for moveLinear()/
     * moveCircular()/moveHelical() target positions — element i of a
     * target vector maps to axisIds[i]. IDs not registered via addAxis()
     * are skipped when a move executes. Must be called before coordinated
     * moves; a stale list after removeAxis() simply ignores the missing
     * axis.
     *
     * @param axisIds  Axis IDs in coordinate order.
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
