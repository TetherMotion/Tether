/**
 * @file ElectronicGearing.hpp
 * @brief Electronic gearing implementation for CiA 402
 * 
 * @details
 * Implements electronic gearing (master-slave synchronization) where
 * one or more slave axes follow a master axis with configurable gear ratio.
 * 
 * ## Features
 * - Configurable gear ratio (numerator/denominator)
 * - Soft engage/disengage with ramping
 * - Multiple slaves per master
 * - Offset adjustment during operation
 * - Feed-forward for improved tracking
 * 
 * ## Architecture
 * 
 * ```
 *  ┌──────────────┐
 *  │    Master    │
 *  │    Axis      │────────┐
 *  └──────────────┘        │
 *         │                │ Position feedback
 *         │ Gear Ratio     │
 *         ▼                ▼
 *  ┌──────────────────────────────┐
 *  │    GearingController         │
 *  │  ┌─────────────────────────┐ │
 *  │  │ Slave[0]: ratio, offset │ │
 *  │  │ Slave[1]: ratio, offset │ │
 *  │  │ ...                     │ │
 *  │  └─────────────────────────┘ │
 *  └──────────────────────────────┘
 *         │
 *         ▼
 *  ┌──────────────┐
 *  │   Slave(s)   │
 *  │    Axes      │
 *  └──────────────┘
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * GearingController gearing;
 * 
 * // Configure master
 * gearing.setMasterBackend(masterDrive);
 * 
 * // Add slave with 2:1 ratio
 * gearing.addSlave(slaveDrive, 2, 1);
 * 
 * // Engage with soft start
 * gearing.engage(true);
 * 
 * // In cyclic task:
 * while (running) {
 *     gearing.update();
 *     vTaskDelay(pdMS_TO_TICKS(1));
 * }
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include "DriveBackend.hpp"
#include <vector>
#include <memory>
#include <functional>

namespace CiA402 {

/**
 * @brief Gearing engagement state
 */
enum class GearingState {
    Disengaged,     ///< Not following master
    Engaging,       ///< Ramping up to full ratio
    Engaged,        ///< Fully synchronized
    Disengaging,    ///< Ramping down from full ratio
    Error           ///< Error condition
};

/**
 * @brief Slave configuration
 */
struct GearingSlaveConfig {
    int32_t numerator{1};       ///< Gear ratio numerator
    int32_t denominator{1};     ///< Gear ratio denominator
    int32_t offset{0};          ///< Position offset
    bool enableFeedForward{true};  ///< Enable velocity feed-forward
    double feedForwardGain{1.0};   ///< Feed-forward gain
};

/**
 * @brief Slave runtime data
 */
struct GearingSlave {
    DriveBackendPtr backend;        ///< Slave drive backend
    GearingSlaveConfig config;      ///< Slave configuration
    GearingState state{GearingState::Disengaged};
    
    double currentRatio{0.0};       ///< Current effective ratio (during ramp)
    int32_t lastMasterPos{0};       ///< Last master position
    int32_t syncPosition{0};        ///< Position at engage
    int32_t targetPosition{0};      ///< Calculated target position
    uint64_t engageStartTime{0};    ///< Time engage started
};

/**
 * @brief Gearing event callback
 */
using GearingEventCallback = std::function<void(size_t slaveIndex, GearingState state)>;

/**
 * @brief Electronic Gearing Controller
 */
class GearingController {
public:
    GearingController();
    ~GearingController() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set master drive backend
     */
    void setMasterBackend(DriveBackendPtr backend);
    
    /**
     * @brief Add slave drive
     * 
     * @param backend Slave drive backend
     * @param numerator Gear ratio numerator
     * @param denominator Gear ratio denominator
     * @return Slave index
     */
    size_t addSlave(DriveBackendPtr backend, 
                   int32_t numerator = 1, 
                   int32_t denominator = 1);
    
    /**
     * @brief Configure slave
     * 
     * @param slaveIndex Slave index
     * @param config Slave configuration
     */
    void configureSlave(size_t slaveIndex, const GearingSlaveConfig& config);
    
    /**
     * @brief Remove slave
     */
    void removeSlave(size_t slaveIndex);
    
    /**
     * @brief Clear all slaves
     */
    void clearSlaves();
    
    /**
     * @brief Get number of slaves
     */
    size_t getSlaveCount() const { return m_slaves.size(); }
    
    /**
     * @brief Set soft engage/disengage time
     * 
     * @param rampTimeMs Time to ramp ratio from 0 to 1 (or 1 to 0)
     */
    void setRampTime(uint32_t rampTimeMs) { m_rampTimeMs = rampTimeMs; }
    
    /**
     * @brief Set event callback
     */
    void setEventCallback(GearingEventCallback callback);
    
    // ========================================================================
    // Operations
    // ========================================================================
    
    /**
     * @brief Engage gearing
     * 
     * @param softStart Use ramped engage
     * @return true if engage started
     */
    bool engage(bool softStart = true);
    
    /**
     * @brief Engage single slave
     */
    bool engageSlave(size_t slaveIndex, bool softStart = true);
    
    /**
     * @brief Disengage gearing
     * 
     * @param softStop Use ramped disengage
     */
    void disengage(bool softStop = true);
    
    /**
     * @brief Disengage single slave
     */
    void disengageSlave(size_t slaveIndex, bool softStop = true);
    
    /**
     * @brief Update gearing - call cyclically
     * 
     * Reads master position and calculates slave targets.
     */
    void update();
    
    /**
     * @brief Check if all slaves are engaged
     */
    bool isEngaged() const;
    
    /**
     * @brief Check if any slave is engaged
     */
    bool isAnyEngaged() const;
    
    /**
     * @brief Get slave state
     */
    GearingState getSlaveState(size_t slaveIndex) const;
    
    // ========================================================================
    // Runtime Adjustment
    // ========================================================================
    
    /**
     * @brief Set gear ratio during operation
     * 
     * @param slaveIndex Slave index
     * @param numerator New numerator
     * @param denominator New denominator
     * @param ramp Ramp to new ratio
     */
    void setGearRatio(size_t slaveIndex, int32_t numerator, int32_t denominator,
                     bool ramp = true);
    
    /**
     * @brief Adjust slave offset
     * 
     * @param slaveIndex Slave index
     * @param offsetDelta Offset change (positive = advance)
     */
    void adjustOffset(size_t slaveIndex, int32_t offsetDelta);
    
    /**
     * @brief Set absolute slave offset
     */
    void setOffset(size_t slaveIndex, int32_t offset);
    
    /**
     * @brief Synchronize slave to current master position
     * 
     * Resets tracking error.
     */
    void synchronize(size_t slaveIndex);
    
    /**
     * @brief Synchronize all slaves
     */
    void synchronizeAll();
    
    // ========================================================================
    // Status
    // ========================================================================
    
    /**
     * @brief Get current master position
     */
    int32_t getMasterPosition() const { return m_masterPosition; }
    
    /**
     * @brief Get current master velocity
     */
    int32_t getMasterVelocity() const { return m_masterVelocity; }
    
    /**
     * @brief Get slave target position
     */
    int32_t getSlaveTarget(size_t slaveIndex) const;
    
    /**
     * @brief Get slave following error
     */
    int32_t getSlaveFollowingError(size_t slaveIndex) const;
    
    /**
     * @brief Get effective gear ratio (during ramping)
     */
    double getEffectiveRatio(size_t slaveIndex) const;
    
private:
    /**
     * @brief Update single slave
     */
    void updateSlave(GearingSlave& slave, int32_t masterPos, int32_t masterVel);
    
    /**
     * @brief Calculate ramp factor
     * 
     * @param slave Slave data
     * @return Factor 0.0 to 1.0
     */
    double calculateRampFactor(const GearingSlave& slave) const;
    
    /**
     * @brief Notify state change
     */
    void notifyStateChange(size_t slaveIndex, GearingState newState);
    
    // Master
    DriveBackendPtr m_masterBackend;
    int32_t m_masterPosition{0};
    int32_t m_masterVelocity{0};
    int32_t m_lastMasterPosition{0};
    
    // Slaves
    std::vector<GearingSlave> m_slaves;
    
    // Configuration
    uint32_t m_rampTimeMs{CIA402_GEARING_RAMP_TIME_MS};
    bool m_softStartEnabled{CIA402_GEARING_SOFT_START};
    
    // Callbacks
    GearingEventCallback m_eventCallback;
    
    // Timing
    uint64_t m_lastUpdateTime{0};
};

/**
 * @brief Multi-master gearing for complex configurations
 * 
 * Allows slaves to follow multiple masters with weighted ratios.
 */
class MultiMasterGearing {
public:
    /**
     * @brief Master input configuration
     */
    struct MasterInput {
        DriveBackendPtr backend;
        double weight{1.0};     ///< Weight (0.0 to 1.0)
        int32_t numerator{1};
        int32_t denominator{1};
    };
    
    /**
     * @brief Add master input
     */
    void addMaster(DriveBackendPtr backend, double weight = 1.0,
                  int32_t num = 1, int32_t den = 1);
    
    /**
     * @brief Set slave backend
     */
    void setSlaveBackend(DriveBackendPtr backend);
    
    /**
     * @brief Update - calculate weighted sum of masters
     */
    void update();
    
    /**
     * @brief Get calculated slave position
     */
    int32_t getSlaveTarget() const { return m_slaveTarget; }
    
private:
    std::vector<MasterInput> m_masters;
    DriveBackendPtr m_slaveBackend;
    int32_t m_slaveTarget{0};
};

} // namespace CiA402
