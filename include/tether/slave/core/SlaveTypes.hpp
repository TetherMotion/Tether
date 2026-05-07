/**
 * @file SlaveTypes.hpp
 * @brief Core type definitions for EtherCAT slave implementation
 *
 * @details
 * This file contains all fundamental types used throughout the slave implementation:
 * - ESC (EtherCAT Slave Controller) register definitions
 * - FMMU (Fieldbus Memory Management Unit) structures
 * - Sync Manager structures
 * - Distributed Clock structures
 * - AL (Application Layer) state machine types
 *
 * All structures are designed to match the EtherCAT specification (ETG.1000)
 * and support both physical and emulated slave implementations.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// ESC Register Addresses (ETG.1000 Part 4)
// ============================================================================

namespace ESCReg {
    // Information registers
    constexpr uint16_t Type                     = 0x0000;  // ESC type (2 bytes)
    constexpr uint16_t Revision                 = 0x0001;  // ESC revision
    constexpr uint16_t Build                    = 0x0002;  // ESC build (2 bytes)
    constexpr uint16_t FMMUCount                = 0x0004;  // Number of FMMUs
    constexpr uint16_t SyncManagerCount         = 0x0005;  // Number of SMs
    constexpr uint16_t RAMSize                  = 0x0006;  // Process data RAM (KB)
    constexpr uint16_t PortDescriptor           = 0x0007;  // Port descriptor
    constexpr uint16_t Features                 = 0x0008;  // ESC features (2 bytes)
    
    // Configured Station Address
    constexpr uint16_t ConfiguredStationAddress = 0x0010;  // Configured address (2 bytes)
    constexpr uint16_t ConfiguredStationAlias   = 0x0012;  // Alias address (2 bytes)
    
    // DL registers
    constexpr uint16_t DLControl                = 0x0100;  // DL control (4 bytes)
    constexpr uint16_t DLStatus                 = 0x0110;  // DL status (2 bytes)
    
    // AL registers
    constexpr uint16_t ALControl                = 0x0120;  // AL control (2 bytes)
    constexpr uint16_t ALStatus                 = 0x0130;  // AL status (2 bytes)
    constexpr uint16_t ALStatusCode             = 0x0134;  // AL status code (2 bytes)
    
    // PDI registers
    constexpr uint16_t PDIControl               = 0x0140;  // PDI control (2 bytes)
    constexpr uint16_t PDIConfig                = 0x0150;  // PDI configuration (2 bytes)
    
    // Watchdog registers
    constexpr uint16_t WatchdogDivider          = 0x0400;  // Watchdog divider (2 bytes)
    constexpr uint16_t PDIWatchdog              = 0x0410;  // PDI watchdog (2 bytes)
    constexpr uint16_t SyncManagerWatchdog      = 0x0420;  // SM watchdog (2 bytes)
    constexpr uint16_t WatchdogStatus           = 0x0440;  // Watchdog status
    constexpr uint16_t WatchdogCounter          = 0x0442;  // SM watchdog counter (2 bytes)
    constexpr uint16_t PDIWatchdogCounter       = 0x0443;  // PDI watchdog counter
    
    // SII/EEPROM registers
    constexpr uint16_t SIIConfig                = 0x0500;  // SII config (2 bytes)
    constexpr uint16_t SIIControl               = 0x0502;  // SII control (2 bytes)
    constexpr uint16_t SIIAddress               = 0x0504;  // SII address (4 bytes)
    constexpr uint16_t SIIData                  = 0x0508;  // SII data (8 bytes)
    
    // FMMU registers (16 bytes each)
    constexpr uint16_t FMMU0                    = 0x0600;
    constexpr uint16_t FMMUSize                 = 16;  // Bytes per FMMU
    
    // Sync Manager registers (8 bytes each)
    constexpr uint16_t SM0                      = 0x0800;
    constexpr uint16_t SMSize                   = 8;   // Bytes per SM
    
    // DC registers
    constexpr uint16_t DCReceiveTime            = 0x0900;  // Port 0 receive time (8 bytes)
    constexpr uint16_t DCSystemTime             = 0x0910;  // System time (8 bytes)
    constexpr uint16_t DCReceiveTimePort1       = 0x0918;  // Port 1 receive time (8 bytes)
    constexpr uint16_t DCSystemTimeOffset       = 0x0920;  // System time offset (8 bytes)
    constexpr uint16_t DCSystemTimeDelay        = 0x0928;  // System time delay (4 bytes)
    constexpr uint16_t DCSystemTimeDifference   = 0x092C;  // System time difference (4 bytes)
    constexpr uint16_t DCSpeedCounterStart      = 0x0930;  // Speed counter start (2 bytes)
    constexpr uint16_t DCSpeedCounterDiff       = 0x0932;  // Speed counter diff (2 bytes)
    constexpr uint16_t DCSystemTimeDiffFilter   = 0x0934;  // Filter depth (1 byte)
    constexpr uint16_t DCControlLoop            = 0x0935;  // Control loop (1 byte)
    
    // DC SYNC registers
    constexpr uint16_t DCSyncActivation         = 0x0980;  // SYNC activation (2 bytes)
    constexpr uint16_t DCSyncStartTime          = 0x0990;  // Start time (8 bytes)
    constexpr uint16_t DCSync0CycleTime         = 0x09A0;  // SYNC0 cycle (4 bytes)
    constexpr uint16_t DCSync1CycleTime         = 0x09A4;  // SYNC1 cycle (4 bytes)
    
    // DC Latch registers
    constexpr uint16_t DCLatchControl           = 0x09A8;  // Latch control
    constexpr uint16_t DCLatchStatus            = 0x09AE;  // Latch status
    constexpr uint16_t DCLatch0Time             = 0x09B0;  // Latch 0 time pos (8 bytes)
    constexpr uint16_t DCLatch0TimeNeg          = 0x09B8;  // Latch 0 time neg (8 bytes)
    constexpr uint16_t DCLatch1Time             = 0x09C0;  // Latch 1 time pos (8 bytes)
    constexpr uint16_t DCLatch1TimeNeg          = 0x09C8;  // Latch 1 time neg (8 bytes)
    
    // Process data RAM starts at 0x1000
    constexpr uint16_t ProcessDataRAM           = 0x1000;
}

// ============================================================================
// AL (Application Layer) States
// ============================================================================

/**
 * @brief EtherCAT slave states per ETG.1000
 */
enum class SlaveState : uint8_t {
    INIT      = 0x01,  ///< Initialization state
    PRE_OP    = 0x02,  ///< Pre-Operational state
    BOOT      = 0x03,  ///< Bootstrap state
    SAFE_OP   = 0x04,  ///< Safe-Operational state
    OP        = 0x08,  ///< Operational state
};

/**
 * @brief Convert slave state to string
 */
inline const char* slaveStateToString(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "INIT";
        case SlaveState::PRE_OP:  return "PRE_OP";
        case SlaveState::BOOT:    return "BOOT";
        case SlaveState::SAFE_OP: return "SAFE_OP";
        case SlaveState::OP:      return "OP";
        default:                  return "UNKNOWN";
    }
}

/**
 * @brief AL Status register structure
 */
struct ALStatus {
    SlaveState state = SlaveState::INIT;
    bool error = false;
    bool identificationRequest = false;
    
    uint16_t toRegister() const {
        uint16_t val = static_cast<uint16_t>(state);
        if (error) val |= 0x0010;
        if (identificationRequest) val |= 0x0020;
        return val;
    }
    
    static ALStatus fromRegister(uint16_t reg) {
        ALStatus status;
        status.state = static_cast<SlaveState>(reg & 0x0F);
        status.error = (reg & 0x0010) != 0;
        status.identificationRequest = (reg & 0x0020) != 0;
        return status;
    }
};

/**
 * @brief AL Control register structure
 */
struct ALControl {
    SlaveState requestedState = SlaveState::INIT;
    bool acknowledgeError = false;
    bool identificationRequest = false;
    
    uint16_t toRegister() const {
        uint16_t val = static_cast<uint16_t>(requestedState);
        if (acknowledgeError) val |= 0x0010;
        if (identificationRequest) val |= 0x0020;
        return val;
    }
    
    static ALControl fromRegister(uint16_t reg) {
        ALControl ctrl;
        ctrl.requestedState = static_cast<SlaveState>(reg & 0x0F);
        ctrl.acknowledgeError = (reg & 0x0010) != 0;
        ctrl.identificationRequest = (reg & 0x0020) != 0;
        return ctrl;
    }
};

/**
 * @brief AL Status Codes per ETG.1000
 */
enum class ALStatusCode : uint16_t {
    NoError                     = 0x0000,
    UnspecifiedError            = 0x0001,
    NoMemory                    = 0x0002,
    InvalidStateChange          = 0x0011,
    UnknownRequestedState       = 0x0012,
    BootstrapNotSupported       = 0x0013,
    NoValidFirmware             = 0x0014,
    InvalidMailboxConfig        = 0x0016,
    InvalidMailboxConfig2       = 0x0017,
    InvalidSyncManagerConfig    = 0x0018,
    NoValidInputs               = 0x0019,
    NoValidOutputs              = 0x001A,
    SynchronizationError        = 0x001B,
    SyncManagerWatchdog         = 0x001C,
    InvalidSyncTypes            = 0x001D,
    InvalidOutputConfig         = 0x001E,
    InvalidInputConfig          = 0x001F,
    InvalidWatchdogConfig       = 0x0020,
    SlaveNeedsColdStart         = 0x0021,
    SlaveNeedsInit              = 0x0022,
    SlaveNeedsPreOp             = 0x0023,
    SlaveNeedsSafeOp            = 0x0024,
    InvalidInputMapping         = 0x0025,
    InvalidOutputMapping        = 0x0026,
    InconsistentSettings        = 0x0027,
    FreeRunNotSupported         = 0x0028,
    SyncNotSupported            = 0x0029,
    FreeRunNeeds3Buffer         = 0x002A,
    BackgroundWatchdog          = 0x002B,
    NoValidInputsOutputs        = 0x002C,
    FatalSyncError              = 0x002D,
    NoSyncError                 = 0x002E,
    InvalidDCSync               = 0x0030,
    InvalidDCLatch              = 0x0031,
    PLLError                    = 0x0032,
    DCSync0Missing              = 0x0033,
    DCSync1Missing              = 0x0034,
    DCSyncIOError               = 0x0035,
    ApplicationControllerAvail  = 0x0050,
    InvalidOutputData           = 0x0060,
    InvalidInputData            = 0x0061,
};

// ============================================================================
// FMMU (Fieldbus Memory Management Unit) Structure
// ============================================================================

/**
 * @brief FMMU configuration structure (16 bytes in ESC)
 */
struct FMMUConfig {
    uint32_t logicalStartAddr = 0;    ///< Logical start address (4 bytes)
    uint16_t length = 0;               ///< Data length (2 bytes)
    uint8_t  logicalStartBit = 0;      ///< Logical start bit (1 byte)
    uint8_t  logicalEndBit = 7;        ///< Logical end bit (1 byte)
    uint16_t physicalStartAddr = 0;    ///< Physical start address (2 bytes)
    uint8_t  physicalStartBit = 0;     ///< Physical start bit (1 byte)
    uint8_t  type = 0;                 ///< Type (read=1, write=2, r/w=3) (1 byte)
    uint8_t  activate = 0;             ///< Activate (1 byte)
    uint8_t  reserved[3] = {0};        ///< Reserved (3 bytes)
    
    bool isEnabled() const { return (activate & 0x01) != 0; }
    bool isReadEnabled() const { return (type & 0x01) != 0; }
    bool isWriteEnabled() const { return (type & 0x02) != 0; }
    
    void setEnabled(bool enable) {
        activate = enable ? 0x01 : 0x00;
    }
    
    /**
     * @brief Check if logical address falls within this FMMU
     */
    bool containsLogicalAddress(uint32_t addr, uint16_t len) const {
        if (!isEnabled()) return false;
        return addr < (logicalStartAddr + length) && (addr + len) > logicalStartAddr;
    }
    
    /**
     * @brief Translate logical address to physical
     */
    uint16_t translateToPhysical(uint32_t logicalAddr) const {
        return physicalStartAddr + (logicalAddr - logicalStartAddr);
    }
    
    /**
     * @brief Serialize to ESC register format (16 bytes)
     */
    void toBytes(uint8_t* out) const {
        out[0] = logicalStartAddr & 0xFF;
        out[1] = (logicalStartAddr >> 8) & 0xFF;
        out[2] = (logicalStartAddr >> 16) & 0xFF;
        out[3] = (logicalStartAddr >> 24) & 0xFF;
        out[4] = length & 0xFF;
        out[5] = (length >> 8) & 0xFF;
        out[6] = logicalStartBit;
        out[7] = logicalEndBit;
        out[8] = physicalStartAddr & 0xFF;
        out[9] = (physicalStartAddr >> 8) & 0xFF;
        out[10] = physicalStartBit;
        out[11] = type;
        out[12] = activate;
        out[13] = 0; out[14] = 0; out[15] = 0;  // Reserved
    }
    
    /**
     * @brief Deserialize from ESC register format (16 bytes)
     */
    void fromBytes(const uint8_t* in) {
        logicalStartAddr = in[0] | (in[1] << 8) | (in[2] << 16) | (in[3] << 24);
        length = in[4] | (in[5] << 8);
        logicalStartBit = in[6];
        logicalEndBit = in[7];
        physicalStartAddr = in[8] | (in[9] << 8);
        physicalStartBit = in[10];
        type = in[11];
        activate = in[12];
    }
};

// ============================================================================
// Sync Manager Structure
// ============================================================================

/**
 * @brief Sync Manager types
 */
enum class SyncManagerType : uint8_t {
    Unused       = 0x00,  ///< Not configured
    MailboxOut   = 0x01,  ///< Mailbox out (master → slave)
    MailboxIn    = 0x02,  ///< Mailbox in (slave → master)
    ProcessOut   = 0x03,  ///< Process data out (RxPDO, master → slave)
    ProcessIn    = 0x04,  ///< Process data in (TxPDO, slave → master)
};

/**
 * @brief Sync Manager control register bits
 */
namespace SMControl {
    constexpr uint8_t OperationMode  = 0x04;  // 0=3-buffer, 1=mailbox
    constexpr uint8_t Direction      = 0x02;  // 0=read (out), 1=write (in)
    constexpr uint8_t IntECAT        = 0x01;  // Interrupt on EtherCAT
    constexpr uint8_t WatchdogEnable = 0x40;  // Watchdog enable
}

/**
 * @brief Sync Manager status register bits
 */
namespace SMStatus {
    constexpr uint8_t IntWrite       = 0x01;  // Interrupt write
    constexpr uint8_t IntRead        = 0x02;  // Interrupt read
    constexpr uint8_t MailboxStatus  = 0x08;  // Mailbox full (out) / empty (in)
    constexpr uint8_t BufferedState  = 0x30;  // 3-buffer state
}

/**
 * @brief Sync Manager configuration (8 bytes in ESC)
 */
struct SyncManagerConfig {
    uint16_t physicalAddr = 0;     ///< Physical start address
    uint16_t length = 0;           ///< Data length
    uint8_t  control = 0;          ///< Control register
    uint8_t  status = 0;           ///< Status register
    uint8_t  activate = 0;         ///< Activate register
    uint8_t  pdoDisable = 0;       ///< PDI disable
    
    SyncManagerType type = SyncManagerType::Unused;  ///< Logical type (not in ESC)
    
    bool isEnabled() const { return (activate & 0x01) != 0; }
    bool isMailbox() const { return (control & SMControl::OperationMode) == 0; }
    bool isBuffered() const { return (control & SMControl::OperationMode) != 0; }
    bool isDirectionOutput() const { return (control & SMControl::Direction) == 0; }
    bool isDirectionInput() const { return (control & SMControl::Direction) != 0; }
    bool watchdogEnabled() const { return (control & SMControl::WatchdogEnable) != 0; }
    
    void setEnabled(bool enable) {
        if (enable) activate |= 0x01;
        else activate &= ~0x01;
    }
    
    /**
     * @brief Serialize to ESC register format (8 bytes)
     */
    void toBytes(uint8_t* out) const {
        out[0] = physicalAddr & 0xFF;
        out[1] = (physicalAddr >> 8) & 0xFF;
        out[2] = length & 0xFF;
        out[3] = (length >> 8) & 0xFF;
        out[4] = control;
        out[5] = status;
        out[6] = activate;
        out[7] = pdoDisable;
    }
    
    /**
     * @brief Deserialize from ESC register format (8 bytes)
     */
    void fromBytes(const uint8_t* in) {
        physicalAddr = in[0] | (in[1] << 8);
        length = in[2] | (in[3] << 8);
        control = in[4];
        status = in[5];
        activate = in[6];
        pdoDisable = in[7];
    }
};

// ============================================================================
// Distributed Clock Structure
// ============================================================================

/**
 * @brief DC SYNC activation bits
 */
namespace DCSyncActivation {
    constexpr uint16_t CyclicOperation    = 0x0001;  // Cyclic operation
    constexpr uint16_t Sync0Enable        = 0x0100;  // SYNC0 enable
    constexpr uint16_t Sync1Enable        = 0x0200;  // SYNC1 enable
    constexpr uint16_t Sync0StartTime     = 0x0400;  // Use start time
}

/**
 * @brief Distributed Clock state
 */
struct DCState {
    // Time registers
    uint64_t systemTime = 0;          ///< System time (0x0910)
    uint64_t receiveTimePort0 = 0;    ///< Receive time port 0 (0x0900)
    uint64_t receiveTimePort1 = 0;    ///< Receive time port 1 (0x0918)
    int64_t  systemTimeOffset = 0;    ///< System time offset (0x0920)
    int32_t  systemTimeDelay = 0;     ///< System time delay (0x0928)
    int32_t  systemTimeDiff = 0;      ///< System time difference (0x092C)
    
    // Speed counter
    uint16_t speedCounterStart = 0;   ///< Speed counter start (0x0930)
    uint16_t speedCounterDiff = 0;    ///< Speed counter diff (0x0932)
    uint8_t  filterDepth = 4;         ///< System time diff filter (0x0934)
    uint8_t  controlLoop = 0;         ///< Control loop (0x0935)
    
    // SYNC configuration
    uint16_t syncActivation = 0;      ///< SYNC activation (0x0980)
    uint64_t syncStartTime = 0;       ///< Start time (0x0990)
    uint32_t sync0CycleTime = 0;      ///< SYNC0 cycle time (0x09A0)
    uint32_t sync1CycleTime = 0;      ///< SYNC1 cycle time (0x09A4)
    
    // Latch
    uint8_t  latchControl = 0;        ///< Latch control (0x09A8)
    uint8_t  latchStatus = 0;         ///< Latch status (0x09AE)
    uint64_t latch0TimePos = 0;       ///< Latch 0 positive edge
    uint64_t latch0TimeNeg = 0;       ///< Latch 0 negative edge
    uint64_t latch1TimePos = 0;       ///< Latch 1 positive edge
    uint64_t latch1TimeNeg = 0;       ///< Latch 1 negative edge
    
    // Runtime state
    bool     sync0Active = false;     ///< SYNC0 signal active
    bool     sync1Active = false;     ///< SYNC1 signal active
    uint64_t lastSync0Time = 0;       ///< Last SYNC0 trigger time
    uint64_t lastSync1Time = 0;       ///< Last SYNC1 trigger time
    
    bool isCyclicOperation() const {
        return (syncActivation & DCSyncActivation::CyclicOperation) != 0;
    }
    
    bool isSync0Enabled() const {
        return (syncActivation & DCSyncActivation::Sync0Enable) != 0;
    }
    
    bool isSync1Enabled() const {
        return (syncActivation & DCSyncActivation::Sync1Enable) != 0;
    }
    
    /**
     * @brief Update DC time
     * @param deltaNs Delta time in nanoseconds
     */
    void advanceTime(uint64_t deltaNs) {
        systemTime += deltaNs;
    }
    
    /**
     * @brief Check and trigger SYNC signals
     * @param callback Callback for SYNC events (sync_num, timestamp)
     * @return true if any SYNC was triggered
     */
    bool checkSyncTrigger(std::function<void(int, uint64_t)> callback) {
        bool triggered = false;
        
        if (isSync0Enabled() && sync0CycleTime > 0) {
            if ((systemTime - lastSync0Time) >= sync0CycleTime) {
                lastSync0Time += sync0CycleTime;
                if (callback) callback(0, systemTime);
                triggered = true;
            }
        }
        
        if (isSync1Enabled() && sync1CycleTime > 0) {
            if ((systemTime - lastSync1Time) >= sync1CycleTime) {
                lastSync1Time += sync1CycleTime;
                if (callback) callback(1, systemTime);
                triggered = true;
            }
        }
        
        return triggered;
    }
};

// ============================================================================
// Watchdog Structure
// ============================================================================

/**
 * @brief Watchdog state
 */
struct WatchdogState {
    uint16_t divider = 2498;         ///< Watchdog divider (default ~100µs tick)
    uint16_t pdiTimeout = 1000;      ///< PDI watchdog timeout
    uint16_t smTimeout = 1000;       ///< SM watchdog timeout
    
    uint8_t  status = 0;             ///< Watchdog status
    uint16_t smCounter = 0;          ///< SM watchdog counter
    uint8_t  pdiCounter = 0;         ///< PDI watchdog counter
    
    // Runtime
    uint64_t lastPdiAccess = 0;      ///< Last PDI access timestamp
    uint64_t lastSmAccess = 0;       ///< Last SM access timestamp
    
    bool isPdiTriggered() const { return (status & 0x01) != 0; }
    bool isSmTriggered() const { return (status & 0x02) != 0; }
    
    void resetPdiWatchdog() {
        pdiCounter = 0;
        status &= ~0x01;
    }
    
    void resetSmWatchdog() {
        smCounter = 0;
        status &= ~0x02;
    }
};

// ============================================================================
// SII (Slave Information Interface) Structure
// ============================================================================

/**
 * @brief SII Control register bits
 */
namespace SIIControl {
    constexpr uint16_t ReadOperation    = 0x0001;
    constexpr uint16_t WriteOperation   = 0x0002;
    constexpr uint16_t ReloadOperation  = 0x0004;
    constexpr uint16_t CRCError         = 0x0008;
    constexpr uint16_t LoadingError     = 0x0010;
    constexpr uint16_t AckError         = 0x0020;
    constexpr uint16_t WriteError       = 0x0040;
    constexpr uint16_t Busy             = 0x8000;
}

/**
 * @brief SII state
 */
struct SIIState {
    uint16_t config = 0;              ///< SII config (0x0500)
    uint16_t control = 0;             ///< SII control (0x0502)
    uint32_t address = 0;             ///< SII address (0x0504)
    uint8_t  data[8] = {0};           ///< SII data (0x0508)
    
    std::vector<uint8_t> eepromData;  ///< EEPROM content
    
    bool isBusy() const { return (control & SIIControl::Busy) != 0; }
    bool isReadOperation() const { return (control & SIIControl::ReadOperation) != 0; }
    bool isWriteOperation() const { return (control & SIIControl::WriteOperation) != 0; }
};

// ============================================================================
// ESC (EtherCAT Slave Controller) Configuration
// ============================================================================

/**
 * @brief ESC feature bits
 */
namespace ESCFeature {
    constexpr uint16_t FMMU           = 0x0001;  // FMMU supported
    constexpr uint16_t SyncManager    = 0x0002;  // SM supported
    constexpr uint16_t DC             = 0x0004;  // DC supported
    constexpr uint16_t DCWidth64      = 0x0008;  // DC 64-bit width
    constexpr uint16_t LowJitter      = 0x0010;  // Low jitter EBUS
    constexpr uint16_t EnhancedLink   = 0x0020;  // Enhanced link detection
    constexpr uint16_t DCEnhanced     = 0x0040;  // Enhanced DC sync
    constexpr uint16_t FMMUExFCS      = 0x0080;  // FMMU extra for FCS
    constexpr uint16_t EnhancedSM     = 0x0100;  // Enhanced SM (1st OP)
}

/**
 * @brief ESC hardware configuration
 */
struct ESCConfig {
    uint8_t  type = 0x02;             ///< ESC type (IP core type)
    uint8_t  revision = 0x01;         ///< ESC revision
    uint16_t build = 0x0001;          ///< ESC build number
    uint8_t  fmmuCount = 8;           ///< Number of FMMUs (max 16)
    uint8_t  smCount = 8;             ///< Number of SMs (max 32)
    uint8_t  ramSizeKB = 8;           ///< Process data RAM in KB
    uint8_t  portDescriptor = 0x04;   ///< Port descriptor
    uint16_t features = ESCFeature::FMMU | ESCFeature::SyncManager | ESCFeature::DC;
    
    uint16_t processDataRamSize() const { return ramSizeKB * 1024; }
};

// ============================================================================
// Slave Identity
// ============================================================================

/**
 * @brief Slave identity information
 */
struct SlaveIdentity {
    uint32_t vendorId = 0x00000000;
    uint32_t productCode = 0x00000000;
    uint32_t revisionNumber = 0x00000000;
    uint32_t serialNumber = 0x00000000;
    
    std::string deviceName = "EtherCAT Slave";
    std::string hwVersion = "1.0";
    std::string swVersion = "1.0";
};

// ============================================================================
// Callback Types
// ============================================================================

/// Callback for state change events
using StateChangeCallback = std::function<void(SlaveState oldState, SlaveState newState)>;

/// Callback for SYNC events
using SyncCallback = std::function<void(int syncNum, uint64_t timestamp)>;

/// Callback for PDO exchange
using PDOExchangeCallback = std::function<void()>;

/// Callback for watchdog timeout
using WatchdogCallback = std::function<void(bool pdi, bool sm)>;

}  // namespace slave
}  // namespace EtherCAT
