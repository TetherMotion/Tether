/**
 * @file EtherCATBackend.hpp
 * @brief EtherCAT drive backend implementation
 * 
 * @details
 * Implements DriveBackend interface for EtherCAT communication.
 * Designed to work with different EtherCAT master implementations.
 * 
 * ## Supported Masters
 * 
 * This backend provides an abstraction that can work with:
 * - SOEM (Simple Open EtherCAT Master)
 * - IgH EtherCAT Master
 * - Custom implementations
 * 
 * ## PDO Mapping
 * 
 * Default PDO mapping for CiA 402 drives:
 * 
 * ```
 * RxPDO (Master -> Slave):
 *   0x6040 - Controlword
 *   0x607A - Target Position
 *   0x60FF - Target Velocity
 *   0x6071 - Target Torque
 *   0x6060 - Modes of Operation
 * 
 * TxPDO (Slave -> Master):
 *   0x6041 - Statusword
 *   0x6064 - Position Actual Value
 *   0x606C - Velocity Actual Value
 *   0x6077 - Torque Actual Value
 *   0x6061 - Modes of Operation Display
 * ```
 */

#pragma once

#include "DriveBackend.hpp"
#include <mutex>
#include <atomic>

namespace CiA402 {

/**
 * @brief PDO mapping configuration
 */
struct PDOMapping {
    // RxPDO (outputs to slave)
    size_t controlWordOffset{0};
    size_t targetPositionOffset{2};
    size_t targetVelocityOffset{6};
    size_t targetTorqueOffset{10};
    size_t modesOfOperationOffset{12};
    size_t positionOffsetOffset{13};     // For CSP mode
    size_t velocityOffsetOffset{17};     // For CSV mode
    size_t torqueOffsetOffset{21};       // For CST mode
    
    // TxPDO (inputs from slave)
    size_t statusWordOffset{0};
    size_t actualPositionOffset{2};
    size_t actualVelocityOffset{6};
    size_t actualTorqueOffset{10};
    size_t modesDisplayOffset{12};
    size_t errorCodeOffset{13};
    
    // Sizes
    size_t rxPdoSize{24};
    size_t txPdoSize{16};
};

/**
 * @brief EtherCAT slave configuration
 */
struct EtherCATSlaveConfig {
    uint32_t slaveId{0};            ///< Slave position on bus
    uint32_t vendorId{0};           ///< Vendor ID
    uint32_t productCode{0};        ///< Product code
    PDOMapping pdoMapping;          ///< PDO mapping configuration
    std::string name;               ///< Slave name/description
};

/**
 * @brief Abstract EtherCAT master interface
 * 
 * Implement this for your specific EtherCAT master library.
 */
class MasterInterface {
public:
    virtual ~MasterInterface() = default;
    
    /**
     * @brief Read process data from slave
     * 
     * @param slaveId Slave index
     * @param data Buffer to store data
     * @param size Size of data
     * @return true if successful
     */
    virtual bool readProcessData(uint32_t slaveId, uint8_t* data, size_t size) = 0;
    
    /**
     * @brief Write process data to slave
     * 
     * @param slaveId Slave index
     * @param data Data to write
     * @param size Size of data
     * @return true if successful
     */
    virtual bool writeProcessData(uint32_t slaveId, const uint8_t* data, size_t size) = 0;
    
    /**
     * @brief Read SDO
     */
    virtual SDOResult readSDO(uint32_t slaveId, uint16_t index, uint8_t subindex,
                             void* data, size_t size) = 0;
    
    /**
     * @brief Write SDO
     */
    virtual SDOResult writeSDO(uint32_t slaveId, uint16_t index, uint8_t subindex,
                              const void* data, size_t size) = 0;
    
    /**
     * @brief Check if slave is operational
     */
    virtual bool isSlaveOperational(uint32_t slaveId) = 0;
    
    /**
     * @brief Get slave state
     */
    virtual int getSlaveState(uint32_t slaveId) = 0;
};

/**
 * @brief Shared pointer for master interface
 */
using EtherCATMasterPtr = std::shared_ptr<MasterInterface>;

/**
 * @brief EtherCAT drive backend
 */
class EtherCATBackend : public DriveBackend {
public:
    /**
     * @brief Constructor
     * 
     * @param master EtherCAT master interface
     * @param config Slave configuration
     */
    EtherCATBackend(EtherCATMasterPtr master, const EtherCATSlaveConfig& config);
    
    ~EtherCATBackend() override;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    bool initialize() override;
    void deinitialize() override;
    bool isConnected() const override;
    std::string getName() const override;
    
    // ========================================================================
    // Process Data
    // ========================================================================
    
    bool updateInputs() override;
    bool updateOutputs() override;
    DriveState getState() const override;
    
    // ========================================================================
    // Control/Status Words
    // ========================================================================
    
    uint16_t readStatusWord() override;
    void writeControlWord(uint16_t controlWord) override;
    uint16_t readControlWord() const override;
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    bool setOperatingMode(OperatingMode mode) override;
    OperatingMode getOperatingMode() const override;
    OperatingMode getDisplayedMode() const override;
    
    // ========================================================================
    // Position
    // ========================================================================
    
    void setTargetPosition(int32_t position) override;
    int32_t getActualPosition() const override;
    int32_t getPositionDemand() const override;
    int32_t getFollowingError() const override;
    void setPositionOffset(int32_t offset) override;
    
    // ========================================================================
    // Velocity
    // ========================================================================
    
    void setTargetVelocity(int32_t velocity) override;
    int32_t getActualVelocity() const override;
    int32_t getVelocityDemand() const override;
    void setVelocityOffset(int32_t offset) override;
    
    // ========================================================================
    // Torque
    // ========================================================================
    
    void setTargetTorque(int16_t torque) override;
    int16_t getActualTorque() const override;
    void setTorqueOffset(int16_t offset) override;
    
    // ========================================================================
    // Profile Parameters
    // ========================================================================
    
    void setProfileVelocity(uint32_t velocity) override;
    void setProfileAcceleration(uint32_t acceleration) override;
    void setProfileDeceleration(uint32_t deceleration) override;
    void setMotionProfileType(int16_t type) override;
    
    // ========================================================================
    // Homing
    // ========================================================================
    
    bool configureHoming(const HomingParams& params) override;
    HomingParams getHomingParams() const override;
    
    // ========================================================================
    // Interpolation
    // ========================================================================
    
    bool configureInterpolation(const InterpolationParams& params) override;
    bool addInterpolationPoint(int32_t position) override;
    void clearInterpolationBuffer() override;
    
    // ========================================================================
    // SDO Access
    // ========================================================================
    
    SDOResult readSDO(uint16_t index, uint8_t subindex, 
                     void* data, size_t size) override;
    SDOResult writeSDO(uint16_t index, uint8_t subindex,
                      const void* data, size_t size) override;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    bool configure(const DriveConfig& config) override;
    DriveConfig getConfiguration() const override;
    bool storeParameters() override;
    bool restoreParameters() override;
    
    // ========================================================================
    // Error Handling
    // ========================================================================
    
    uint16_t getErrorCode() const override;
    uint8_t getErrorRegister() const override;
    std::vector<uint16_t> getErrorHistory() const override;
    bool clearErrorHistory() override;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setStateChangeCallback(StateChangeCallback callback) override;
    void setErrorCallback(ErrorCallback callback) override;
    void setWarningCallback(WarningCallback callback) override;
    void setSyncCallback(SyncCallback callback) override;
    
    // ========================================================================
    // Timing
    // ========================================================================
    
    uint32_t getCycleTimeUs() const override;
    bool setCycleTimeUs(uint32_t cycleTimeUs) override;
    uint64_t getLastUpdateTimestamp() const override;
    
    // ========================================================================
    // EtherCAT Specific
    // ========================================================================
    
    /**
     * @brief Get slave configuration
     */
    const EtherCATSlaveConfig& getSlaveConfig() const { return m_config; }
    
    /**
     * @brief Set PDO mapping
     */
    void setPDOMapping(const PDOMapping& mapping);
    
    /**
     * @brief Get raw RxPDO buffer
     */
    const uint8_t* getRxPDOBuffer() const { return m_rxPdo; }
    
    /**
     * @brief Get raw TxPDO buffer
     */
    const uint8_t* getTxPDOBuffer() const { return m_txPdo; }
    
private:
    /**
     * @brief Write to RxPDO buffer at offset
     */
    template<typename T>
    void writeRxPDO(size_t offset, T value);
    
    /**
     * @brief Read from TxPDO buffer at offset
     */
    template<typename T>
    T readTxPDO(size_t offset) const;
    
    /**
     * @brief Decode state from status word
     */
    State decodeState() const;
    
    /**
     * @brief Check for state change and notify
     */
    void checkStateChange();
    
    // Master interface
    EtherCATMasterPtr m_master;
    EtherCATSlaveConfig m_config;
    
    // PDO buffers
    uint8_t m_rxPdo[64];    // Output to slave
    uint8_t m_txPdo[64];    // Input from slave
    
    // Cached state
    DriveState m_state;
    State m_cia402State{State::NotReadyToSwitchOn};
    State m_lastState{State::NotReadyToSwitchOn};
    OperatingMode m_operatingMode{OperatingMode::ProfilePosition};
    DriveConfig m_driveConfig;
    HomingParams m_homingParams;
    
    // Callbacks
    StateChangeCallback m_stateCallback;
    ErrorCallback m_errorCallback;
    WarningCallback m_warningCallback;
    SyncCallback m_syncCallback;
    
    // Timing
    uint32_t m_cycleTimeUs{CIA402_DEFAULT_CYCLE_TIME_US};
    uint64_t m_lastUpdateTimestamp{0};
    
    // Status
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_initialized{false};
    mutable std::mutex m_mutex;
};

/**
 * @brief Factory for creating EtherCAT backends
 */
class EtherCATBackendFactory {
public:
    /**
     * @brief Set master interface
     */
    void setMaster(EtherCATMasterPtr master) { m_master = master; }
    
    /**
     * @brief Create backend for slave
     */
    DriveBackendUPtr createBackend(uint32_t slaveId);
    
    /**
     * @brief Create backend with custom config
     */
    DriveBackendUPtr createBackend(const EtherCATSlaveConfig& config);
    
private:
    EtherCATMasterPtr m_master;
};

} // namespace CiA402
