/**
 * @file ProfileSlave.hpp
 * @brief Base class and interfaces for CiA profile slave implementations
 *
 * @details
 * This file provides the infrastructure for implementing CiA profile-specific
 * EtherCAT slaves. Each profile (CiA 401, CiA 402, etc.) extends the base
 * ProfileSlave class with profile-specific functionality.
 *
 * Supported profiles:
 * - CiA 401: Generic I/O modules (digital/analog I/O)
 * - CiA 402: Drives and motion control
 * - CiA 404: Measuring devices and closed-loop controllers
 * - CiA 405: Programmable devices (PLCs)
 * - CiA 406: Encoders
 * - CiA 408: Fluid power (hydraulic valves)
 * - CiA 410: Inclinometers
 * - CiA 417: Lift controllers
 * - CiA 430: Power supplies
 */

#pragma once

#include "slave/core/SlaveCore.hpp"
#include "slave/core/SlaveTypes.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <memory>
#include <string>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Profile Types
// ============================================================================

/**
 * @brief CiA profile identifiers
 */
enum class CiAProfile : uint16_t {
    Generic     = 0,      ///< No specific profile
    CiA301      = 301,    ///< CANopen communication profile
    CiA401      = 401,    ///< Generic I/O modules
    CiA402      = 402,    ///< Drives and motion control
    CiA404      = 404,    ///< Measuring devices
    CiA405      = 405,    ///< Programmable devices (PLCs)
    CiA406      = 406,    ///< Encoders
    CiA408      = 408,    ///< Hydraulic drives
    CiA410      = 410,    ///< Inclinometers
    CiA417      = 417,    ///< Lift controllers
    CiA430      = 430,    ///< Power supplies
};

// ============================================================================
// ProfileSlave Base Class
// ============================================================================

/**
 * @brief Base class for CiA profile slave implementations
 *
 * ProfileSlave extends SlaveCore with profile-specific functionality:
 * - Standard object dictionary entries for the profile
 * - PDO configuration appropriate for the profile
 * - Profile-specific state machine (e.g., CiA 402 drive states)
 *
 * Usage:
 * @code
 * // Create a CiA 402 drive slave
 * CiA402SlaveConfig driveConfig;
 * driveConfig.vendorId = 0x1234;
 * driveConfig.productCode = 0x5678;
 * driveConfig.modes = CiA402::PP | CiA402::CSP;  // Profile Position + CSP
 *
 * auto drive = std::make_unique<CiA402Slave>(driveConfig);
 * drive->setHAL(hal);
 * drive->start();
 * @endcode
 */
class ProfileSlave {
public:
    /**
     * @brief Constructor
     * @param profile CiA profile type
     * @param config Slave configuration
     */
    ProfileSlave(CiAProfile profile, const SlaveConfig& config);
    
    /**
     * @brief Destructor
     */
    virtual ~ProfileSlave();
    
    // Non-copyable
    ProfileSlave(const ProfileSlave&) = delete;
    ProfileSlave& operator=(const ProfileSlave&) = delete;
    
    // ========================================================================
    // Core Access
    // ========================================================================
    
    /**
     * @brief Get the underlying SlaveCore
     */
    SlaveCore& getCore() { return *core_; }
    const SlaveCore& getCore() const { return *core_; }
    
    /**
     * @brief Get the object dictionary
     */
    IObjectDictionary& getObjectDictionary() { return *objectDictionary_; }
    const IObjectDictionary& getObjectDictionary() const { return *objectDictionary_; }
    
    // ========================================================================
    // Control
    // ========================================================================
    
    /**
     * @brief Set HAL
     */
    void setHAL(std::shared_ptr<ISlaveHAL> hal);
    
    /**
     * @brief Start the slave
     */
    bool start();
    
    /**
     * @brief Stop the slave
     */
    void stop();
    
    /**
     * @brief Check if running
     */
    bool isRunning() const;
    
    // ========================================================================
    // Profile Information
    // ========================================================================
    
    /**
     * @brief Get profile type
     */
    CiAProfile getProfile() const { return profile_; }
    
    /**
     * @brief Get profile name
     */
    virtual const char* getProfileName() const = 0;
    
    /**
     * @brief Get device type (object 0x1000)
     */
    virtual uint32_t getDeviceType() const = 0;
    
    // ========================================================================
    // Simulation
    // ========================================================================
    
    /**
     * @brief Simulate one cycle
     *
     * Profile-specific simulation logic (e.g., motor motion for CiA 402).
     *
     * @param deltaNs Time delta in nanoseconds
     */
    virtual void simulate(uint64_t deltaNs);
    
    // ========================================================================
    // PDO Exchange
    // ========================================================================
    
    /**
     * @brief Called before PDO exchange
     *
     * Update TxPDO data from internal state.
     */
    virtual void updateTxPDO() = 0;
    
    /**
     * @brief Called after PDO exchange
     *
     * Process RxPDO data into internal state.
     */
    virtual void processRxPDO() = 0;
    
protected:
    // ========================================================================
    // Virtual Methods for Subclass Implementation
    // ========================================================================
    
    /**
     * @brief Initialize profile-specific object dictionary entries
     */
    virtual void initObjectDictionary() = 0;
    
    /**
     * @brief Initialize profile-specific PDO mappings
     */
    virtual void initPDOMappings() = 0;
    
    /**
     * @brief Handle state change
     */
    virtual void onStateChange(SlaveState oldState, SlaveState newState);
    
    /**
     * @brief Handle SYNC event
     */
    virtual void onSync(int syncNum, uint64_t timestamp);
    
    // ========================================================================
    // Helper Methods
    // ========================================================================
    
    /**
     * @brief Register standard CiA 301 communication objects
     */
    void registerCiA301Objects();
    
    /**
     * @brief Register PDO mapping object
     *
     * @param index PDO mapping index (0x1600-0x17FF or 0x1A00-0x1BFF)
     * @param entries PDO entries (index:subindex:bitlength)
     */
    void registerPDOMapping(uint16_t index, const std::vector<uint32_t>& entries);
    
    /**
     * @brief Get pointer to TxPDO data at offset
     */
    template<typename T>
    T* getTxPDOPtr(size_t offset) {
        return reinterpret_cast<T*>(core_->getTxPDOData() + offset);
    }
    
    /**
     * @brief Get pointer to RxPDO data at offset
     */
    template<typename T>
    const T* getRxPDOPtr(size_t offset) const {
        return reinterpret_cast<const T*>(core_->getRxPDOData() + offset);
    }
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    CiAProfile profile_;
    std::unique_ptr<SlaveCore> core_;
    std::shared_ptr<IObjectDictionary> objectDictionary_;
    std::vector<std::shared_ptr<IMailboxHandler>> mailboxHandlers_;
};

// ============================================================================
// PDO Mapping Helper
// ============================================================================

/**
 * @brief Create PDO mapping entry
 *
 * @param index Object index
 * @param subindex Object subindex
 * @param bitLength Bit length
 * @return Packed PDO mapping entry
 */
constexpr uint32_t PDOMapEntry(uint16_t index, uint8_t subindex, uint8_t bitLength) {
    return (static_cast<uint32_t>(index) << 16) |
           (static_cast<uint32_t>(subindex) << 8) |
           bitLength;
}

}  // namespace slave
}  // namespace EtherCAT
