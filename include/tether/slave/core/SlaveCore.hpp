/**
 * @file SlaveCore.hpp
 * @brief Core EtherCAT Slave Controller implementation
 *
 * @details
 * This file implements a complete software EtherCAT slave including:
 * - ESC (EtherCAT Slave Controller) register emulation
 * - AL (Application Layer) state machine
 * - FMMU (Fieldbus Memory Management Unit) processing
 * - Sync Manager processing
 * - Distributed Clock support
 * - SII (EEPROM) emulation
 *
 * The slave can be connected to a master via:
 * - Direct in-process connection (for unit testing)
 * - POSIX FIFO (for inter-process communication)
 * - Real network interface (for hardware testing)
 *
 * @note This implementation follows ETG.1000 EtherCAT specification
 */

#pragma once

#include "tether/slave/core/SlaveTypes.hpp"
#include "tether/slave/logging/SlaveLogger.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace EtherCAT {
namespace slave {

// Forward declarations
class ISlaveHAL;
class IObjectDictionary;
class IMailboxHandler;

// ============================================================================
// Slave Configuration
// ============================================================================

/**
 * @brief Configuration for slave instance
 */
struct SlaveConfig {
    // Identity
    SlaveIdentity identity;
    
    // ESC hardware config
    ESCConfig escConfig;
    
    // Mailbox configuration
    uint16_t mailboxOutOffset = 0x1000;   ///< SM0 (MbxOut/M→S): Master→Slave mailbox offset
    uint16_t mailboxOutSize = 128;        ///< SM0: Mailbox size
    uint16_t mailboxInOffset = 0x1080;    ///< SM1 (MbxIn/S→M): Slave→Master mailbox offset
    uint16_t mailboxInSize = 128;         ///< SM1: Mailbox size
    uint16_t mailboxProtocol = 0x000C;    ///< Supported protocols (CoE + FoE)
    
    // Process data configuration
    uint16_t rxPdoOffset = 0x1100;        ///< SM2: RxPDO offset
    uint16_t rxPdoSize = 0;               ///< SM2: RxPDO size (0 = auto from mapping)
    uint16_t txPdoOffset = 0x1180;        ///< SM3: TxPDO offset
    uint16_t txPdoSize = 0;               ///< SM3: TxPDO size (0 = auto from mapping)
    
    // DC configuration
    bool supportsDC = true;               ///< DC support enabled
    uint32_t defaultCycleTime = 1000000;  ///< Default cycle time in ns (1ms)
    
    // Watchdog
    bool watchdogEnabled = true;
    uint16_t watchdogDivider = 2498;      ///< ~100µs tick
    uint16_t watchdogTimeout = 1000;      ///< 100ms default
    
    // Bootstrap support
    bool supportsBootstrap = false;
    
    // Logging
    SlaveLogConfig logConfig;
};

// ============================================================================
// Slave Core Class
// ============================================================================

/**
 * @brief Core EtherCAT Slave Controller implementation
 *
 * This class implements the complete slave functionality including:
 * - Frame processing (APRD, APWR, FPRD, FPWR, BRD, BWR, LRD, LWR, LRW)
 * - State machine management
 * - FMMU and Sync Manager processing
 * - Distributed Clock
 * - SII (EEPROM) emulation
 *
 * Usage:
 * @code
 * SlaveConfig config;
 * config.identity.vendorId = 0x1234;
 * config.identity.productCode = 0x5678;
 *
 * auto slave = std::make_unique<SlaveCore>(config);
 * slave->setHAL(std::make_shared<DirectSlaveHAL>());
 * slave->start();
 * @endcode
 */
class SlaveCore {
public:
    /**
     * @brief Constructor
     * @param config Slave configuration
     */
    explicit SlaveCore(const SlaveConfig& config);
    
    /**
     * @brief Destructor
     */
    ~SlaveCore();
    
    // Non-copyable, movable
    SlaveCore(const SlaveCore&) = delete;
    SlaveCore& operator=(const SlaveCore&) = delete;
    SlaveCore(SlaveCore&&) noexcept;
    SlaveCore& operator=(SlaveCore&&) noexcept;
    
    // ========================================================================
    // Initialization and Control
    // ========================================================================
    
    /**
     * @brief Set the HAL for frame transmission/reception
     * @param hal HAL implementation
     */
    void setHAL(std::shared_ptr<ISlaveHAL> hal);
    
    /**
     * @brief Set the object dictionary
     * @param od Object dictionary implementation (for CoE)
     */
    void setObjectDictionary(std::shared_ptr<IObjectDictionary> od);
    
    /**
     * @brief Add a mailbox handler
     * @param handler Mailbox protocol handler (CoE, FoE, etc.)
     */
    void addMailboxHandler(std::shared_ptr<IMailboxHandler> handler);
    
    /**
     * @brief Start the slave
     * @return true if started successfully
     */
    bool start();
    
    /**
     * @brief Stop the slave
     */
    void stop();
    
    /**
     * @brief Check if slave is running
     */
    bool isRunning() const { return running_.load(); }
    
    // ========================================================================
    // Frame Processing
    // ========================================================================
    
    /**
     * @brief Process an incoming EtherCAT frame
     * @param frame Frame data (Ethernet header + EtherCAT payload)
     * @param length Frame length
     * @return Response frame (modified in place for most commands)
     */
    std::vector<uint8_t> processFrame(const uint8_t* frame, size_t length);
    
    /**
     * @brief Process a single datagram
     * @param header Datagram header
     * @param data Data payload
     * @return Working counter increment (0, 1, 2, or 3)
     */
    uint16_t processDatagram(DatagramHeader& header, uint8_t* data);
    
    // ========================================================================
    // State Management
    // ========================================================================
    
    /**
     * @brief Get current slave state
     */
    SlaveState getState() const { return alStatus_.state; }
    
    /**
     * @brief Get AL status
     */
    ALStatus getALStatus() const;
    
    /**
     * @brief Request state change (from master)
     * @param control AL control value
     * @return true if state change initiated
     */
    bool requestStateChange(const ALControl& control);
    
    /**
     * @brief Set state change callback
     */
    void setStateChangeCallback(StateChangeCallback callback);
    
    /**
     * @brief Force error state
     * @param code AL status code
     */
    void setError(ALStatusCode code);
    
    /**
     * @brief Clear error state
     */
    void clearError();
    
    // ========================================================================
    // FMMU Access
    // ========================================================================
    
    /**
     * @brief Get FMMU configuration
     * @param index FMMU index (0-15)
     */
    const FMMUConfig& getFMMU(size_t index) const;
    
    /**
     * @brief Set FMMU configuration
     * @param index FMMU index
     * @param config New configuration
     */
    void setFMMU(size_t index, const FMMUConfig& config);
    
    /**
     * @brief Get number of configured FMMUs
     */
    size_t getFMMUCount() const { return config_.escConfig.fmmuCount; }
    
    // ========================================================================
    // Sync Manager Access
    // ========================================================================
    
    /**
     * @brief Get Sync Manager configuration
     * @param index SM index (0-7)
     */
    const SyncManagerConfig& getSyncManager(size_t index) const;
    
    /**
     * @brief Set Sync Manager configuration
     * @param index SM index
     * @param config New configuration
     */
    void setSyncManager(size_t index, const SyncManagerConfig& config);
    
    /**
     * @brief Get number of Sync Managers
     */
    size_t getSyncManagerCount() const { return config_.escConfig.smCount; }
    
    // ========================================================================
    // Process Data Access
    // ========================================================================
    
    /**
     * @brief Get pointer to RxPDO data (master → slave)
     * @return Pointer to RxPDO buffer
     */
    uint8_t* getRxPDOData();
    const uint8_t* getRxPDOData() const;
    size_t getRxPDOSize() const;
    
    /**
     * @brief Get pointer to TxPDO data (slave → master)
     * @return Pointer to TxPDO buffer
     */
    uint8_t* getTxPDOData();
    const uint8_t* getTxPDOData() const;
    size_t getTxPDOSize() const;
    
    /**
     * @brief Set PDO exchange callback (called after each exchange)
     */
    void setPDOExchangeCallback(PDOExchangeCallback callback);
    
    // ========================================================================
    // Distributed Clock
    // ========================================================================
    
    /**
     * @brief Get DC state
     */
    const DCState& getDCState() const { return dcState_; }
    DCState& getDCState() { return dcState_; }
    
    /**
     * @brief Advance DC time
     * @param deltaNs Time delta in nanoseconds
     */
    void advanceDCTime(uint64_t deltaNs);
    
    /**
     * @brief Set SYNC callback
     */
    void setSyncCallback(SyncCallback callback);
    
    /**
     * @brief Get current DC system time
     */
    uint64_t getDCSystemTime() const { return dcState_.systemTime; }
    
    // ========================================================================
    // SII (EEPROM) Access
    // ========================================================================
    
    /**
     * @brief Set SII EEPROM data
     * @param data EEPROM content (word-aligned)
     */
    void setSIIData(const std::vector<uint8_t>& data);
    
    /**
     * @brief Get SII EEPROM data
     */
    const std::vector<uint8_t>& getSIIData() const;
    
    /**
     * @brief Read SII word
     * @param wordAddr Word address
     * @return Word data (0xFFFF on error)
     */
    uint16_t readSIIWord(uint16_t wordAddr) const;
    
    /**
     * @brief Write SII word
     * @param wordAddr Word address
     * @param data Word data
     * @return true if successful
     */
    bool writeSIIWord(uint16_t wordAddr, uint16_t data);
    
    // ========================================================================
    // Watchdog
    // ========================================================================
    
    /**
     * @brief Get watchdog state
     */
    const WatchdogState& getWatchdogState() const { return watchdog_; }
    
    /**
     * @brief Set watchdog callback
     */
    void setWatchdogCallback(WatchdogCallback callback);
    
    /**
     * @brief Reset watchdog
     */
    void resetWatchdog();
    
    // ========================================================================
    // Mailbox
    // ========================================================================
    
    /**
     * @brief Check if mailbox out has data (master → slave)
     */
    bool hasMailboxRequest() const;
    
    /**
     * @brief Get pending mailbox request
     * @return Request data (empty if none)
     */
    std::vector<uint8_t> getMailboxRequest();
    
    /**
     * @brief Set mailbox response
     * @param response Response data
     */
    void setMailboxResponse(const std::vector<uint8_t>& response);
    
    /**
     * @brief Check if mailbox in has response (slave → master)
     */
    bool hasMailboxResponse() const;
    
    // ========================================================================
    // Configuration and Identity
    // ========================================================================
    
    /**
     * @brief Get slave configuration
     */
    const SlaveConfig& getConfig() const { return config_; }
    
    /**
     * @brief Get slave identity
     */
    const SlaveIdentity& getIdentity() const { return config_.identity; }
    
    /**
     * @brief Get configured station address
     */
    uint16_t getConfiguredAddress() const { return configuredAddress_; }
    
    /**
     * @brief Get station alias
     */
    uint16_t getStationAlias() const { return stationAlias_; }
    
    /**
     * @brief Get position (auto-increment counter)
     */
    uint16_t getPosition() const { return position_; }
    
    /**
     * @brief Set position (for network topology)
     */
    void setPosition(uint16_t pos) { position_ = pos; }
    
    // ========================================================================
    // Logging
    // ========================================================================
    
    /**
     * @brief Get logger
     */
    SlaveLogger& getLogger() { return *logger_; }
    const SlaveLogger& getLogger() const { return *logger_; }
    
    /**
     * @brief Enable/disable logging categories
     */
    void setLogEnabled(SlaveLogCategory category, bool enabled);
    
    // ========================================================================
    // Simulation/Testing
    // ========================================================================
    
    /**
     * @brief Simulate one cycle (for testing)
     * @param deltaNs Time delta in nanoseconds
     */
    void simulate(uint64_t deltaNs);
    
    /**
     * @brief Get ESC register memory (for debugging)
     */
    const uint8_t* getRegisterMemory() const { return registers_.data(); }
    
    /**
     * @brief Get process data RAM (for debugging)
     */
    const uint8_t* getProcessDataRAM() const { return processDataRAM_.data(); }
    
private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    // Register access
    bool readRegister(uint16_t addr, uint8_t* data, uint16_t len);
    bool writeRegister(uint16_t addr, const uint8_t* data, uint16_t len);
    
    // Process data RAM access
    bool readProcessData(uint16_t addr, uint8_t* data, uint16_t len);
    bool writeProcessData(uint16_t addr, const uint8_t* data, uint16_t len);
    
    // Logical address processing
    bool processLogicalRead(uint32_t logicalAddr, uint8_t* data, uint16_t len);
    bool processLogicalWrite(uint32_t logicalAddr, const uint8_t* data, uint16_t len);
    
    // State machine
    bool canTransition(SlaveState from, SlaveState to);
    void doStateTransition(SlaveState newState);
    void onEnterState(SlaveState state);
    void onExitState(SlaveState state);
    void setErrorLocked(ALStatusCode code);   ///< setError without locking stateMutex_
    void clearErrorLocked();                  ///< clearError without locking stateMutex_
    
    // SII processing
    void processSIICommand();
    
    // Watchdog processing
    void updateWatchdog(uint64_t deltaNs);
    
    // Sync manager processing
    void processSyncManager(size_t smIndex);
    
    // Mailbox processing
    void processMailboxOut();
    void processMailboxIn();
    
    // Initialize SII from config
    void initializeSII();
    
    // Initialize default SM configuration
    void initializeSyncManagers();
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    // Configuration
    SlaveConfig config_;
    
    // HAL
    std::shared_ptr<ISlaveHAL> hal_;
    
    // Object dictionary
    std::shared_ptr<IObjectDictionary> objectDictionary_;
    
    // Mailbox handlers
    std::vector<std::shared_ptr<IMailboxHandler>> mailboxHandlers_;
    
    // Logger
    std::unique_ptr<SlaveLogger> logger_;
    
    // Running state
    std::atomic<bool> running_{false};
    
    // AL state
    ALStatus alStatus_;
    ALStatusCode alStatusCode_ = ALStatusCode::NoError;
    std::mutex stateMutex_;
    
    // Addressing
    uint16_t configuredAddress_ = 0;
    uint16_t stationAlias_ = 0;
    uint16_t position_ = 0;
    
    // ESC registers (0x0000 - 0x0FFF)
    std::array<uint8_t, 4096> registers_;
    
    // Process data RAM (starts at 0x1000)
    std::vector<uint8_t> processDataRAM_;
    
    // FMMUs
    std::array<FMMUConfig, 16> fmmus_;
    
    // Sync Managers
    std::array<SyncManagerConfig, 8> syncManagers_;
    
    // DC state
    DCState dcState_;
    
    // SII state
    SIIState siiState_;
    
    // Watchdog state
    WatchdogState watchdog_;
    
    // Mailbox buffers
    std::vector<uint8_t> mailboxOut_;  // Master → Slave
    std::vector<uint8_t> mailboxIn_;   // Slave → Master
    std::atomic<bool> mailboxOutFull_{false};
    std::atomic<bool> mailboxInFull_{false};
    
    // Callbacks
    StateChangeCallback stateChangeCallback_;
    SyncCallback syncCallback_;
    PDOExchangeCallback pdoExchangeCallback_;
    WatchdogCallback watchdogCallback_;
    
    // Mutex for thread safety
    mutable std::mutex mutex_;
};

}  // namespace slave
}  // namespace EtherCAT
