/**
 * @file SlaveCore.cpp
 * @brief Implementation of EtherCAT Slave Core
 */

#include "tether/slave/core/SlaveCore.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include <algorithm>
#include <cstring>
#include <chrono>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Constructor / Destructor
// ============================================================================

SlaveCore::SlaveCore(const SlaveConfig& config)
    : config_(config)
    , logger_(std::make_unique<SlaveLogger>(config.logConfig))
    , alStatus_()
    , dcState_()
    , siiState_()
    , watchdog_()
{
    // Initialize registers to zero
    registers_.fill(0);
    
    // Initialize FMMUs
    for (auto& fmmu : fmmus_) {
        fmmu = FMMUConfig{};
    }
    
    // Initialize Sync Managers
    for (auto& sm : syncManagers_) {
        sm = SyncManagerConfig{};
    }
    
    // Set ESC info in registers
    registers_[ESCReg::Type] = config_.escConfig.type;
    registers_[ESCReg::Revision] = config_.escConfig.revision;
    registers_[ESCReg::Build] = config_.escConfig.build & 0xFF;
    registers_[ESCReg::Build + 1] = (config_.escConfig.build >> 8) & 0xFF;
    registers_[ESCReg::FMMUCount] = config_.escConfig.fmmuCount;
    registers_[ESCReg::SyncManagerCount] = config_.escConfig.smCount;
    registers_[ESCReg::RAMSize] = config_.escConfig.ramSizeKB;
    registers_[ESCReg::PortDescriptor] = config_.escConfig.portDescriptor;
    registers_[ESCReg::Features] = config_.escConfig.features & 0xFF;
    registers_[ESCReg::Features + 1] = (config_.escConfig.features >> 8) & 0xFF;
    
    // Initialize process data RAM
    processDataRAM_.resize(config_.escConfig.processDataRamSize(), 0);
    
    // Initialize mailbox buffers
    mailboxOut_.resize(config_.mailboxOutSize, 0);
    mailboxIn_.resize(config_.mailboxInSize, 0);
    
    // Initialize SII
    initializeSII();
    
    // Initialize sync managers
    initializeSyncManagers();
    
    // Set initial AL status
    alStatus_.state = SlaveState::INIT;
    alStatus_.error = false;
    
    // Write initial AL status to registers
    uint16_t alStatusReg = alStatus_.toRegister();
    registers_[ESCReg::ALStatus] = alStatusReg & 0xFF;
    registers_[ESCReg::ALStatus + 1] = (alStatusReg >> 8) & 0xFF;
    
    // Initialize watchdog
    watchdog_.divider = config_.watchdogDivider;
    watchdog_.smTimeout = config_.watchdogTimeout;
}

SlaveCore::~SlaveCore() {
    stop();
}

// Move operations need explicit implementation due to atomic/mutex members
SlaveCore::SlaveCore(SlaveCore&& other) noexcept
    : config_(std::move(other.config_))
    , logger_(std::move(other.logger_))
    , hal_(std::move(other.hal_))
    , objectDictionary_(std::move(other.objectDictionary_))
    , mailboxHandlers_(std::move(other.mailboxHandlers_))
    , running_(other.running_.load())
    , alStatus_(other.alStatus_)
    , alStatusCode_(other.alStatusCode_)
    , configuredAddress_(other.configuredAddress_)
    , stationAlias_(other.stationAlias_)
    , registers_(std::move(other.registers_))
    , processDataRAM_(std::move(other.processDataRAM_))
    , fmmus_(std::move(other.fmmus_))
    , syncManagers_(std::move(other.syncManagers_))
    , dcState_(std::move(other.dcState_))
    , siiState_(std::move(other.siiState_))
    , watchdog_(other.watchdog_)
    , mailboxOut_(std::move(other.mailboxOut_))
    , mailboxIn_(std::move(other.mailboxIn_))
    , mailboxOutFull_(other.mailboxOutFull_.load())
    , mailboxInFull_(other.mailboxInFull_.load())
{}

SlaveCore& SlaveCore::operator=(SlaveCore&& other) noexcept {
    if (this != &other) {
        config_ = std::move(other.config_);
        logger_ = std::move(other.logger_);
        hal_ = std::move(other.hal_);
        objectDictionary_ = std::move(other.objectDictionary_);
        mailboxHandlers_ = std::move(other.mailboxHandlers_);
        running_.store(other.running_.load());
        alStatus_ = other.alStatus_;
        alStatusCode_ = other.alStatusCode_;
        configuredAddress_ = other.configuredAddress_;
        stationAlias_ = other.stationAlias_;
        registers_ = std::move(other.registers_);
        processDataRAM_ = std::move(other.processDataRAM_);
        fmmus_ = std::move(other.fmmus_);
        syncManagers_ = std::move(other.syncManagers_);
        dcState_ = std::move(other.dcState_);
        siiState_ = std::move(other.siiState_);
        watchdog_ = other.watchdog_;
        mailboxOut_ = std::move(other.mailboxOut_);
        mailboxIn_ = std::move(other.mailboxIn_);
        mailboxOutFull_.store(other.mailboxOutFull_.load());
        mailboxInFull_.store(other.mailboxInFull_.load());
    }
    return *this;
}

// ============================================================================
// Initialization and Control
// ============================================================================

void SlaveCore::setHAL(std::shared_ptr<ISlaveHAL> hal) {
    hal_ = std::move(hal);
}

void SlaveCore::setObjectDictionary(std::shared_ptr<IObjectDictionary> od) {
    objectDictionary_ = std::move(od);
}

void SlaveCore::addMailboxHandler(std::shared_ptr<IMailboxHandler> handler) {
    mailboxHandlers_.push_back(std::move(handler));
}

bool SlaveCore::start() {
    if (running_.load()) return false;
    running_.store(true);
    return true;
}

void SlaveCore::stop() {
    running_.store(false);
}

// ============================================================================
// Frame Processing
// ============================================================================

std::vector<uint8_t> SlaveCore::processFrame(const uint8_t* frame, size_t length) {
    // Minimum frame: Ethernet header (14) + EtherCAT header (2) + datagram header (10)
    if (length < 26) {
        return {};
    }
    
    // Create response (copy of input frame)
    std::vector<uint8_t> response(frame, frame + length);
    
    // Skip Ethernet header (14 bytes)
    size_t offset = 14;
    
    // Check EtherType (0x88A4)
    uint16_t etherType = (frame[12] << 8) | frame[13];
    if (etherType != 0x88A4) {
        return {};
    }
    
    // EtherCAT header (2 bytes)
    uint16_t ecatHeader = frame[offset] | (frame[offset + 1] << 8);
    uint16_t ecatLength = ecatHeader & 0x07FF;
    offset += 2;
    
    // Process datagrams
    size_t remaining = ecatLength;
    while (remaining >= 10) {  // Minimum datagram header size
        // Parse datagram header using the EtherCATTypes DatagramHeader structure
        DatagramHeader header;
        std::memcpy(&header, frame + offset, sizeof(DatagramHeader));
        
        uint16_t dataLen = header.dataLength();
        
        // Process this datagram
        uint8_t* dataPtr = response.data() + offset + sizeof(DatagramHeader);
        uint16_t wkcIncrement = processDatagram(header, dataPtr);
        
        // Update WKC in response (WKC follows datagram data)
        size_t wkcOffset = offset + sizeof(DatagramHeader) + dataLen;
        uint16_t currentWkc = response[wkcOffset] | (response[wkcOffset + 1] << 8);
        currentWkc += wkcIncrement;
        response[wkcOffset] = currentWkc & 0xFF;
        response[wkcOffset + 1] = (currentWkc >> 8) & 0xFF;
        
        // Copy back modified header (for auto-increment address update)
        std::memcpy(response.data() + offset, &header, sizeof(DatagramHeader));
        
        // Move to next datagram
        size_t datagramSize = sizeof(DatagramHeader) + dataLen + 2;  // header + data + WKC
        offset += datagramSize;
        remaining -= datagramSize;
        
        if (!header.more()) break;
    }
    
    return response;
}

uint16_t SlaveCore::processDatagram(DatagramHeader& header, uint8_t* data) {
    uint16_t wkc = 0;
    // ADP = adp_le = address position (station offset for auto-increment)
    // ADO = ado_le = address offset (register/memory offset)
    uint16_t adp = header.adp_le;  // Address position
    uint16_t ado = header.ado_le;  // Address offset (register address)
    uint16_t len = header.dataLength();
    uint8_t cmdByte = static_cast<uint8_t>(header.cmd);
    
    // Command codes per ETG.1000
    enum EcCmd : uint8_t {
        NOP  = 0x00,
        APRD = 0x01,  // Auto Increment Read
        APWR = 0x02,  // Auto Increment Write
        APRW = 0x03,  // Auto Increment Read Write
        FPRD = 0x04,  // Configured Address Read
        FPWR = 0x05,  // Configured Address Write
        FPRW = 0x06,  // Configured Address Read Write
        BRD  = 0x07,  // Broadcast Read
        BWR  = 0x08,  // Broadcast Write
        BRW  = 0x09,  // Broadcast Read Write
        LRD  = 0x0A,  // Logical Read
        LWR  = 0x0B,  // Logical Write
        LRW  = 0x0C,  // Logical Read Write
        ARMW = 0x0D,  // Auto Increment Read Multiple Write
        FRMW = 0x0E,  // Configured Address Read Multiple Write
    };
    
    bool addressMatch = false;
    int16_t autoIncrement = static_cast<int16_t>(adp);
    
    switch (cmdByte) {
        case APRD:
        case APWR:
        case APRW:
        case ARMW:
            // Auto increment: respond if position counter == 0
            if (autoIncrement == 0) {
                addressMatch = true;
            }
            // Decrement position counter for next slave
            header.adp_le = static_cast<uint16_t>(autoIncrement - 1);
            break;
            
        case FPRD:
        case FPWR:
        case FPRW:
        case FRMW:
            // Configured address: respond if address matches
            addressMatch = (adp == configuredAddress_) || 
                          (adp == stationAlias_ && stationAlias_ != 0);
            break;
            
        case BRD:
        case BWR:
        case BRW:
            // Broadcast: always respond
            addressMatch = true;
            break;
            
        case LRD:
        case LWR:
        case LRW: {
            // Logical addressing via FMMU
            uint32_t logicalAddr = header.logicalAddress();
            uint16_t len = header.dataLength();
            uint16_t wkc = 0;
            
            if (cmdByte == LRD || cmdByte == LRW) {
                if (processLogicalRead(logicalAddr, data, len)) {
                    wkc |= 1;
                }
            }
            if (cmdByte == LWR || cmdByte == LRW) {
                if (processLogicalWrite(logicalAddr, data, len)) {
                    wkc |= 2;
                }
            }
            
            // Trigger PDO callback if we processed data
            if (wkc > 0 && pdoExchangeCallback_) {
                pdoExchangeCallback_();
            }
            
            return wkc;
        }
            
        default:
            return 0;
    }
    
    if (!addressMatch) {
        return 0;
    }
    
    // Process physical address access (ADO is the register address)
    switch (cmdByte) {
        case APRD:
        case FPRD:
        case BRD:
            if (readRegister(ado, data, len)) {
                wkc = 1;
            }
            break;
            
        case APWR:
        case FPWR:
        case BWR:
            if (writeRegister(ado, data, len)) {
                wkc = 1;
            }
            break;
            
        case APRW:
        case FPRW:
        case BRW: {
            // Read first, then write
            std::vector<uint8_t> readData(len);
            if (readRegister(ado, readData.data(), len)) {
                wkc += 1;
            }
            if (writeRegister(ado, data, len)) {
                wkc += 2;
            }
            // Return read data
            std::memcpy(data, readData.data(), len);
            break;
        }
        
        case ARMW:
        case FRMW:
            // Read multiple write (used for DC)
            if (readRegister(ado, data, len)) {
                wkc = 1;
            }
            break;
    }
    
    return wkc;
}

// ============================================================================
// State Management
// ============================================================================

ALStatus SlaveCore::getALStatus() const {
    // Note: stateMutex_ should be mutable in the header, or we return a copy
    // For now, return the cached value without locking (read is atomic for simple structs)
    return alStatus_;
}

bool SlaveCore::requestStateChange(const ALControl& control) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    // Check if transition is valid
    if (!canTransition(alStatus_.state, control.requestedState)) {
        setErrorLocked(ALStatusCode::InvalidStateChange);
        return false;
    }
    
    // Handle error acknowledgement
    if (control.acknowledgeError && alStatus_.error) {
        clearErrorLocked();
    }
    
    // Perform state change
    doStateTransition(control.requestedState);
    return true;
}

void SlaveCore::setStateChangeCallback(StateChangeCallback callback) {
    stateChangeCallback_ = std::move(callback);
}

void SlaveCore::setErrorLocked(ALStatusCode code) {
    alStatusCode_ = code;
    alStatus_.error = true;
    
    // Update registers
    registers_[ESCReg::ALStatusCode] = static_cast<uint16_t>(code) & 0xFF;
    registers_[ESCReg::ALStatusCode + 1] = (static_cast<uint16_t>(code) >> 8) & 0xFF;
    
    uint16_t statusReg = alStatus_.toRegister();
    registers_[ESCReg::ALStatus] = statusReg & 0xFF;
    registers_[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;
}

void SlaveCore::setError(ALStatusCode code) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    setErrorLocked(code);
}

void SlaveCore::clearErrorLocked() {
    alStatusCode_ = ALStatusCode::NoError;
    alStatus_.error = false;
    
    registers_[ESCReg::ALStatusCode] = 0;
    registers_[ESCReg::ALStatusCode + 1] = 0;
    
    uint16_t statusReg = alStatus_.toRegister();
    registers_[ESCReg::ALStatus] = statusReg & 0xFF;
    registers_[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;
}

void SlaveCore::clearError() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    clearErrorLocked();
}

bool SlaveCore::canTransition(SlaveState from, SlaveState to) {
    // Valid transitions per ETG.1000
    if (from == to) return true;
    
    switch (from) {
        case SlaveState::INIT:
            return to == SlaveState::PRE_OP || to == SlaveState::BOOT;
        case SlaveState::PRE_OP:
            return to == SlaveState::INIT || to == SlaveState::SAFE_OP;
        case SlaveState::BOOT:
            return to == SlaveState::INIT;
        case SlaveState::SAFE_OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::OP;
        case SlaveState::OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::SAFE_OP;
        default:
            return false;
    }
}

void SlaveCore::doStateTransition(SlaveState newState) {
    SlaveState oldState = alStatus_.state;
    
    onExitState(oldState);
    alStatus_.state = newState;
    onEnterState(newState);
    
    // Update AL status register
    uint16_t statusReg = alStatus_.toRegister();
    registers_[ESCReg::ALStatus] = statusReg & 0xFF;
    registers_[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;
    
    // Notify callback
    if (stateChangeCallback_) {
        stateChangeCallback_(oldState, newState);
    }
}

void SlaveCore::onEnterState(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:
            // Reset watchdog
            watchdog_.resetPdiWatchdog();
            watchdog_.resetSmWatchdog();
            break;
        case SlaveState::PRE_OP:
            // Mailbox communication starts
            break;
        case SlaveState::SAFE_OP:
            // Inputs active, outputs safe
            break;
        case SlaveState::OP:
            // Full operation
            break;
        default:
            break;
    }
}

void SlaveCore::onExitState(SlaveState state) {
    (void)state;  // Currently no exit actions
}

// ============================================================================
// FMMU Access
// ============================================================================

const FMMUConfig& SlaveCore::getFMMU(size_t index) const {
    static const FMMUConfig empty{};
    if (index >= fmmus_.size()) return empty;
    return fmmus_[index];
}

void SlaveCore::setFMMU(size_t index, const FMMUConfig& config) {
    if (index >= fmmus_.size()) return;
    fmmus_[index] = config;
    
    // Update registers
    uint8_t regData[16];
    config.toBytes(regData);
    std::memcpy(&registers_[ESCReg::FMMU0 + index * ESCReg::FMMUSize], regData, 16);
}

// ============================================================================
// Sync Manager Access
// ============================================================================

const SyncManagerConfig& SlaveCore::getSyncManager(size_t index) const {
    static const SyncManagerConfig empty{};
    if (index >= syncManagers_.size()) return empty;
    return syncManagers_[index];
}

void SlaveCore::setSyncManager(size_t index, const SyncManagerConfig& config) {
    if (index >= syncManagers_.size()) return;
    syncManagers_[index] = config;
    
    // Update registers
    uint8_t regData[8];
    config.toBytes(regData);
    std::memcpy(&registers_[ESCReg::SM0 + index * ESCReg::SMSize], regData, 8);
}

// ============================================================================
// Process Data Access
// ============================================================================

uint8_t* SlaveCore::getRxPDOData() {
    if (config_.rxPdoOffset < ESCReg::ProcessDataRAM) return nullptr;
    size_t offset = config_.rxPdoOffset - ESCReg::ProcessDataRAM;
    if (offset >= processDataRAM_.size()) return nullptr;
    return processDataRAM_.data() + offset;
}

const uint8_t* SlaveCore::getRxPDOData() const {
    return const_cast<SlaveCore*>(this)->getRxPDOData();
}

size_t SlaveCore::getRxPDOSize() const {
    return config_.rxPdoSize;
}

uint8_t* SlaveCore::getTxPDOData() {
    if (config_.txPdoOffset < ESCReg::ProcessDataRAM) return nullptr;
    size_t offset = config_.txPdoOffset - ESCReg::ProcessDataRAM;
    if (offset >= processDataRAM_.size()) return nullptr;
    return processDataRAM_.data() + offset;
}

const uint8_t* SlaveCore::getTxPDOData() const {
    return const_cast<SlaveCore*>(this)->getTxPDOData();
}

size_t SlaveCore::getTxPDOSize() const {
    return config_.txPdoSize;
}

void SlaveCore::setPDOExchangeCallback(PDOExchangeCallback callback) {
    pdoExchangeCallback_ = std::move(callback);
}

// ============================================================================
// Distributed Clock
// ============================================================================

void SlaveCore::advanceDCTime(uint64_t deltaNs) {
    dcState_.advanceTime(deltaNs);
    
    // Update system time register
    for (int i = 0; i < 8; ++i) {
        registers_[ESCReg::DCSystemTime + i] = (dcState_.systemTime >> (i * 8)) & 0xFF;
    }
    
    // Check for SYNC triggers
    dcState_.checkSyncTrigger([this](int syncNum, uint64_t timestamp) {
        if (syncCallback_) {
            syncCallback_(syncNum, timestamp);
        }
    });
}

void SlaveCore::setSyncCallback(SyncCallback callback) {
    syncCallback_ = std::move(callback);
}

// ============================================================================
// SII (EEPROM) Access
// ============================================================================

void SlaveCore::setSIIData(const std::vector<uint8_t>& data) {
    siiState_.eepromData = data;
}

const std::vector<uint8_t>& SlaveCore::getSIIData() const {
    return siiState_.eepromData;
}

uint16_t SlaveCore::readSIIWord(uint16_t wordAddr) const {
    size_t byteAddr = wordAddr * 2;
    if (byteAddr + 1 >= siiState_.eepromData.size()) {
        return 0xFFFF;
    }
    return siiState_.eepromData[byteAddr] | (siiState_.eepromData[byteAddr + 1] << 8);
}

bool SlaveCore::writeSIIWord(uint16_t wordAddr, uint16_t data) {
    size_t byteAddr = wordAddr * 2;
    if (byteAddr + 1 >= siiState_.eepromData.size()) {
        return false;
    }
    siiState_.eepromData[byteAddr] = data & 0xFF;
    siiState_.eepromData[byteAddr + 1] = (data >> 8) & 0xFF;
    return true;
}

// ============================================================================
// Watchdog
// ============================================================================

void SlaveCore::setWatchdogCallback(WatchdogCallback callback) {
    watchdogCallback_ = std::move(callback);
}

void SlaveCore::resetWatchdog() {
    watchdog_.resetPdiWatchdog();
    watchdog_.resetSmWatchdog();
}

// ============================================================================
// Mailbox
// ============================================================================

bool SlaveCore::hasMailboxRequest() const {
    return mailboxOutFull_.load();
}

std::vector<uint8_t> SlaveCore::getMailboxRequest() {
    if (!mailboxOutFull_.load()) {
        return {};
    }
    std::vector<uint8_t> result = mailboxOut_;
    mailboxOutFull_.store(false);
    return result;
}

void SlaveCore::setMailboxResponse(const std::vector<uint8_t>& response) {
    if (response.size() > mailboxIn_.size()) return;
    std::memcpy(mailboxIn_.data(), response.data(), response.size());
    mailboxInFull_.store(true);
}

bool SlaveCore::hasMailboxResponse() const {
    return mailboxInFull_.load();
}

// ============================================================================
// Logging
// ============================================================================

void SlaveCore::setLogEnabled(SlaveLogCategory category, bool enabled) {
    if (logger_) {
        logger_->setCategoryEnabled(category, enabled);
    }
}

// ============================================================================
// Simulation/Testing
// ============================================================================

void SlaveCore::simulate(uint64_t deltaNs) {
    // Advance DC time
    advanceDCTime(deltaNs);
    
    // Update watchdog
    updateWatchdog(deltaNs);
    
    // Process mailbox
    processMailboxOut();
    processMailboxIn();
}

// ============================================================================
// Private Methods
// ============================================================================

bool SlaveCore::readRegister(uint16_t addr, uint8_t* data, uint16_t len) {
    if (addr + len > registers_.size()) {
        // Try process data RAM
        if (addr >= ESCReg::ProcessDataRAM) {
            return readProcessData(addr - ESCReg::ProcessDataRAM, data, len);
        }
        return false;
    }
    std::memcpy(data, &registers_[addr], len);
    return true;
}

bool SlaveCore::writeRegister(uint16_t addr, const uint8_t* data, uint16_t len) {
    // Handle special registers
    if (addr == ESCReg::ConfiguredStationAddress && len >= 2) {
        configuredAddress_ = data[0] | (data[1] << 8);
    }
    if (addr == ESCReg::ConfiguredStationAlias && len >= 2) {
        stationAlias_ = data[0] | (data[1] << 8);
    }
    
    // Handle AL Control write
    if (addr == ESCReg::ALControl && len >= 2) {
        uint16_t controlReg = data[0] | (data[1] << 8);
        ALControl control = ALControl::fromRegister(controlReg);
        requestStateChange(control);
    }
    
    // Handle SII access
    if (addr == ESCReg::SIIControl && len >= 2) {
        siiState_.control = data[0] | (data[1] << 8);
        processSIICommand();
    }
    if (addr == ESCReg::SIIAddress && len >= 4) {
        siiState_.address = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    }
    
    // Handle FMMU writes
    if (addr >= ESCReg::FMMU0 && addr < ESCReg::FMMU0 + 16 * ESCReg::FMMUSize) {
        size_t fmmuIndex = (addr - ESCReg::FMMU0) / ESCReg::FMMUSize;
        if (fmmuIndex < fmmus_.size()) {
            // Update from register
            size_t regOffset = addr - (ESCReg::FMMU0 + fmmuIndex * ESCReg::FMMUSize);
            std::memcpy(&registers_[addr], data, len);
            fmmus_[fmmuIndex].fromBytes(&registers_[ESCReg::FMMU0 + fmmuIndex * ESCReg::FMMUSize]);
        }
    }
    
    // Handle SM writes
    if (addr >= ESCReg::SM0 && addr < ESCReg::SM0 + 8 * ESCReg::SMSize) {
        size_t smIndex = (addr - ESCReg::SM0) / ESCReg::SMSize;
        if (smIndex < syncManagers_.size()) {
            std::memcpy(&registers_[addr], data, len);
            syncManagers_[smIndex].fromBytes(&registers_[ESCReg::SM0 + smIndex * ESCReg::SMSize]);
        }
    }
    
    if (addr + len > registers_.size()) {
        // Try process data RAM
        if (addr >= ESCReg::ProcessDataRAM) {
            return writeProcessData(addr - ESCReg::ProcessDataRAM, data, len);
        }
        return false;
    }
    std::memcpy(&registers_[addr], data, len);
    return true;
}

bool SlaveCore::readProcessData(uint16_t addr, uint8_t* data, uint16_t len) {
    if (addr + len > processDataRAM_.size()) return false;
    std::memcpy(data, &processDataRAM_[addr], len);
    return true;
}

bool SlaveCore::writeProcessData(uint16_t addr, const uint8_t* data, uint16_t len) {
    if (addr + len > processDataRAM_.size()) return false;
    std::memcpy(&processDataRAM_[addr], data, len);
    return true;
}

bool SlaveCore::processLogicalRead(uint32_t logicalAddr, uint8_t* data, uint16_t len) {
    bool success = false;
    for (const auto& fmmu : fmmus_) {
        if (!fmmu.isEnabled() || !fmmu.isReadEnabled()) continue;
        if (!fmmu.containsLogicalAddress(logicalAddr, len)) continue;
        
        uint16_t physAddr = fmmu.translateToPhysical(logicalAddr);
        if (readProcessData(physAddr, data, len)) {
            success = true;
        }
    }
    return success;
}

bool SlaveCore::processLogicalWrite(uint32_t logicalAddr, const uint8_t* data, uint16_t len) {
    bool success = false;
    for (const auto& fmmu : fmmus_) {
        if (!fmmu.isEnabled() || !fmmu.isWriteEnabled()) continue;
        if (!fmmu.containsLogicalAddress(logicalAddr, len)) continue;
        
        uint16_t physAddr = fmmu.translateToPhysical(logicalAddr);
        if (writeProcessData(physAddr, data, len)) {
            success = true;
        }
    }
    return success;
}

void SlaveCore::processSIICommand() {
    if (siiState_.control & SIIControl::ReadOperation) {
        // Read operation
        uint16_t word = readSIIWord(static_cast<uint16_t>(siiState_.address));
        siiState_.data[0] = word & 0xFF;
        siiState_.data[1] = (word >> 8) & 0xFF;
        
        // Update data register
        registers_[ESCReg::SIIData] = siiState_.data[0];
        registers_[ESCReg::SIIData + 1] = siiState_.data[1];
        
        // Clear read bit and busy
        siiState_.control &= ~(SIIControl::ReadOperation | SIIControl::Busy);
        registers_[ESCReg::SIIControl] = siiState_.control & 0xFF;
        registers_[ESCReg::SIIControl + 1] = (siiState_.control >> 8) & 0xFF;
    }
    
    if (siiState_.control & SIIControl::WriteOperation) {
        // Write operation
        uint16_t word = siiState_.data[0] | (siiState_.data[1] << 8);
        writeSIIWord(static_cast<uint16_t>(siiState_.address), word);
        
        // Clear write bit and busy
        siiState_.control &= ~(SIIControl::WriteOperation | SIIControl::Busy);
        registers_[ESCReg::SIIControl] = siiState_.control & 0xFF;
        registers_[ESCReg::SIIControl + 1] = (siiState_.control >> 8) & 0xFF;
    }
}

void SlaveCore::updateWatchdog(uint64_t deltaNs) {
    if (!config_.watchdogEnabled) return;
    
    // Convert to watchdog ticks (divider is ~100µs)
    uint64_t ticksNs = static_cast<uint64_t>(watchdog_.divider) * 40;  // 40ns per tick
    
    // Check SM watchdog
    if (alStatus_.state == SlaveState::OP) {
        watchdog_.smCounter++;
        if (watchdog_.smCounter >= watchdog_.smTimeout) {
            watchdog_.status |= 0x02;  // SM watchdog triggered
            if (watchdogCallback_) {
                watchdogCallback_(false, true);
            }
        }
    }
}

void SlaveCore::processSyncManager(size_t smIndex) {
    if (smIndex >= syncManagers_.size()) return;
    auto& sm = syncManagers_[smIndex];
    if (!sm.isEnabled()) return;
    
    // Process based on SM type
    switch (sm.type) {
        case SyncManagerType::MailboxOut:
            processMailboxOut();
            break;
        case SyncManagerType::MailboxIn:
            processMailboxIn();
            break;
        default:
            break;
    }
}

void SlaveCore::processMailboxOut() {
    // Check if SM0 (mailbox out) has data from master
    if (syncManagers_[0].isEnabled()) {
        auto& sm = syncManagers_[0];
        // Check mailbox status bit
        if (sm.status & SMStatus::MailboxStatus) {
            // Copy data from process RAM to mailbox buffer
            if (sm.physicalAddr >= ESCReg::ProcessDataRAM) {
                size_t ramOffset = sm.physicalAddr - ESCReg::ProcessDataRAM;
                if (ramOffset + sm.length <= processDataRAM_.size()) {
                    std::memcpy(mailboxOut_.data(), &processDataRAM_[ramOffset], 
                               std::min(static_cast<size_t>(sm.length), mailboxOut_.size()));
                    mailboxOutFull_.store(true);
                    sm.status &= ~SMStatus::MailboxStatus;  // Clear mailbox full
                }
            }
        }
    }
}

void SlaveCore::processMailboxIn() {
    // Check if SM1 (mailbox in) is ready for response
    if (syncManagers_[1].isEnabled() && mailboxInFull_.load()) {
        auto& sm = syncManagers_[1];
        // Copy data from mailbox buffer to process RAM
        if (sm.physicalAddr >= ESCReg::ProcessDataRAM) {
            size_t ramOffset = sm.physicalAddr - ESCReg::ProcessDataRAM;
            if (ramOffset + sm.length <= processDataRAM_.size()) {
                std::memcpy(&processDataRAM_[ramOffset], mailboxIn_.data(),
                           std::min(static_cast<size_t>(sm.length), mailboxIn_.size()));
                mailboxInFull_.store(false);
                sm.status |= SMStatus::MailboxStatus;  // Set mailbox full
            }
        }
    }
}

void SlaveCore::initializeSII() {
    // Create minimal SII data based on identity
    std::vector<uint8_t> sii;
    sii.resize(128, 0xFF);  // Default to 0xFF (empty EEPROM)
    
    // SII Header (first 16 bytes)
    // Word 0: PDI Control
    sii[0] = 0x00; sii[1] = 0x00;
    // Word 1: PDI Config
    sii[2] = 0x00; sii[3] = 0x00;
    // Word 2: Sync Impulse Length
    sii[4] = 0x00; sii[5] = 0x00;
    // Word 3: PDI Config 2
    sii[6] = 0x00; sii[7] = 0x00;
    // Word 4: Station Alias
    sii[8] = 0x00; sii[9] = 0x00;
    // Word 5-6: Reserved
    // Word 7: Checksum (calculated later)
    
    // Words 8-11: Vendor ID (4 bytes)
    sii[16] = config_.identity.vendorId & 0xFF;
    sii[17] = (config_.identity.vendorId >> 8) & 0xFF;
    sii[18] = (config_.identity.vendorId >> 16) & 0xFF;
    sii[19] = (config_.identity.vendorId >> 24) & 0xFF;
    
    // Words 12-13: Product Code
    sii[24] = config_.identity.productCode & 0xFF;
    sii[25] = (config_.identity.productCode >> 8) & 0xFF;
    sii[26] = (config_.identity.productCode >> 16) & 0xFF;
    sii[27] = (config_.identity.productCode >> 24) & 0xFF;
    
    // Words 14-15: Revision Number
    sii[28] = config_.identity.revisionNumber & 0xFF;
    sii[29] = (config_.identity.revisionNumber >> 8) & 0xFF;
    sii[30] = (config_.identity.revisionNumber >> 16) & 0xFF;
    sii[31] = (config_.identity.revisionNumber >> 24) & 0xFF;
    
    // Words 16-17: Serial Number
    sii[32] = config_.identity.serialNumber & 0xFF;
    sii[33] = (config_.identity.serialNumber >> 8) & 0xFF;
    sii[34] = (config_.identity.serialNumber >> 16) & 0xFF;
    sii[35] = (config_.identity.serialNumber >> 24) & 0xFF;
    
    siiState_.eepromData = std::move(sii);
}

void SlaveCore::initializeSyncManagers() {
    // SM0: Mailbox Out (Slave → Master)
    syncManagers_[0].physicalAddr = config_.mailboxOutOffset;
    syncManagers_[0].length = config_.mailboxOutSize;
    syncManagers_[0].control = 0x26;  // Mailbox, write, interrupt
    syncManagers_[0].type = SyncManagerType::MailboxOut;
    
    // SM1: Mailbox In (Master → Slave)
    syncManagers_[1].physicalAddr = config_.mailboxInOffset;
    syncManagers_[1].length = config_.mailboxInSize;
    syncManagers_[1].control = 0x22;  // Mailbox, read
    syncManagers_[1].type = SyncManagerType::MailboxIn;
    
    // SM2: RxPDO (Master → Slave outputs)
    if (config_.rxPdoSize > 0) {
        syncManagers_[2].physicalAddr = config_.rxPdoOffset;
        syncManagers_[2].length = config_.rxPdoSize;
        syncManagers_[2].control = 0x64;  // Buffered, write
        syncManagers_[2].type = SyncManagerType::ProcessOut;
    }
    
    // SM3: TxPDO (Slave → Master inputs)
    if (config_.txPdoSize > 0) {
        syncManagers_[3].physicalAddr = config_.txPdoOffset;
        syncManagers_[3].length = config_.txPdoSize;
        syncManagers_[3].control = 0x20;  // Buffered, read
        syncManagers_[3].type = SyncManagerType::ProcessIn;
    }
    
    // Update registers
    for (size_t i = 0; i < syncManagers_.size(); ++i) {
        uint8_t regData[8];
        syncManagers_[i].toBytes(regData);
        std::memcpy(&registers_[ESCReg::SM0 + i * ESCReg::SMSize], regData, 8);
    }
}

}  // namespace slave
}  // namespace EtherCAT
