/**
 * @file SlaveCore.cpp
 * @brief Implementation of EtherCAT Slave Core
 */

#include "tether/slave/core/SlaveCore.hpp"
#include "tether/slave/mailbox/IMailboxHandler.hpp"
#include "tether/ethercat/Types.hpp"
#include <algorithm>
#include <cstring>
#include <chrono>
#include <bit>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Constructor / Destructor
// ============================================================================

SlaveCore::SlaveCore(const SlaveConfig& config)
    : config_(config)
    , logger_(std::make_unique<SlaveLogger>(config.logConfig))
    , stateMgr_(&registers_)
    , dcState_()
    , siiEmulator_(&registers_)
    , watchdog_()
    , watchdogMgr_(config_.watchdogEnabled, &watchdog_)
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
    uint16_t featuresRaw = std::bit_cast<uint16_t>(config_.escConfig.features);
    registers_[ESCReg::Features] = featuresRaw & 0xFF;
    registers_[ESCReg::Features + 1] = (featuresRaw >> 8) & 0xFF;
    
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
    stateMgr_.setInitialState(SlaveState::INIT);

    // Wire state manager to reset watchdog on entering INIT
    stateMgr_.setOnInitCallback([this]() { watchdogMgr_.reset(); });
    
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
    , stateMgr_(&registers_)
    , configuredAddress_(other.configuredAddress_)
    , stationAlias_(other.stationAlias_)
    , registers_(std::move(other.registers_))
    , processDataRAM_(std::move(other.processDataRAM_))
    , fmmus_(std::move(other.fmmus_))
    , syncManagers_(std::move(other.syncManagers_))
    , dcState_(std::move(other.dcState_))
    , siiEmulator_(&registers_)
    , watchdog_(other.watchdog_)
    , watchdogMgr_(config_.watchdogEnabled, &watchdog_)
    , mailboxOut_(std::move(other.mailboxOut_))
    , mailboxIn_(std::move(other.mailboxIn_))
    , mailboxOutFull_(other.mailboxOutFull_.load())
    , mailboxInFull_(other.mailboxInFull_.load())
{
    // Copy SII state from the moved-from emulator (registers_ pointer is set above)
    siiEmulator_.state() = std::move(other.siiEmulator_.state());
    // Copy AL state from the moved-from state manager
    stateMgr_.statusRef() = other.stateMgr_.status();
    stateMgr_.setOnInitCallback([this]() { watchdogMgr_.reset(); });
}

SlaveCore& SlaveCore::operator=(SlaveCore&& other) noexcept {
    if (this != &other) {
        config_ = std::move(other.config_);
        logger_ = std::move(other.logger_);
        hal_ = std::move(other.hal_);
        objectDictionary_ = std::move(other.objectDictionary_);
        mailboxHandlers_ = std::move(other.mailboxHandlers_);
        running_.store(other.running_.load());
        stateMgr_.statusRef() = other.stateMgr_.status();
        configuredAddress_ = other.configuredAddress_;
        stationAlias_ = other.stationAlias_;
        registers_ = std::move(other.registers_);
        processDataRAM_ = std::move(other.processDataRAM_);
        fmmus_ = std::move(other.fmmus_);
        syncManagers_ = std::move(other.syncManagers_);
        dcState_ = std::move(other.dcState_);
        siiEmulator_.state() = std::move(other.siiEmulator_.state());
        watchdog_ = other.watchdog_;
        // watchdogMgr_ already points to our watchdog_
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
// State Management (delegated to StateManager)
// ============================================================================

ALStatus SlaveCore::getALStatus() const {
    return stateMgr_.status();
}

bool SlaveCore::requestStateChange(const ALControl& control) {
    return stateMgr_.requestStateChange(control);
}

void SlaveCore::setStateChangeCallback(StateChangeCallback callback) {
    stateMgr_.setStateChangeCallback(std::move(callback));
}

void SlaveCore::setErrorLocked(ALStatusCode code) {
    stateMgr_.setErrorLocked(code);
}

void SlaveCore::setError(ALStatusCode code) {
    stateMgr_.setError(code);
}

void SlaveCore::clearErrorLocked() {
    stateMgr_.clearErrorLocked();
}

void SlaveCore::clearError() {
    stateMgr_.clearError();
}

bool SlaveCore::canTransition(SlaveState from, SlaveState to) {
    return StateManager::canTransition(from, to);
}

void SlaveCore::doStateTransition(SlaveState newState) {
    // Delegate via requestStateChange-like path; this is called internally
    // so we use the state manager's internal transition.
    // Note: doStateTransition is private and called from requestStateChange,
    // which is now delegated to stateMgr_. This stub is kept for ABI compat.
    (void)newState;
}

void SlaveCore::onEnterState(SlaveState state) {
    (void)state;  // Delegated to StateManager
}

void SlaveCore::onExitState(SlaveState state) {
    (void)state;  // Delegated to StateManager
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
    siiEmulator_.setData(data);
}

const std::vector<uint8_t>& SlaveCore::getSIIData() const {
    return siiEmulator_.getData();
}

uint16_t SlaveCore::readSIIWord(uint16_t wordAddr) const {
    return siiEmulator_.readWord(wordAddr);
}

bool SlaveCore::writeSIIWord(uint16_t wordAddr, uint16_t data) {
    return siiEmulator_.writeWord(wordAddr, data);
}

// ============================================================================
// Watchdog
// ============================================================================

void SlaveCore::setWatchdogCallback(WatchdogCallback callback) {
    watchdogMgr_.setCallback(std::move(callback));
}

void SlaveCore::resetWatchdog() {
    watchdogMgr_.reset();
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
    
    // Handle SII access (delegated to SIIEmulator)
    siiEmulator_.handleRegisterWrite(addr, data, len);
    
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

    // If this read falls within a mailbox read-direction SM (SM1, slave→master),
    // clear the MailboxStatus bit — the master has consumed the response.
    const uint16_t physAddr = addr + ESCReg::ProcessDataRAM;
    for (size_t i = 0; i < syncManagers_.size(); ++i) {
        auto& sm = syncManagers_[i];
        if (!sm.isEnabled() || !sm.isMailbox()) continue;
        if (sm.isDirectionRead()) {
            if (physAddr >= sm.physicalAddr &&
                physAddr + len <= sm.physicalAddr + sm.length) {
                sm.status = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(static_cast<uint8_t>(std::bit_cast<uint8_t>(sm.status) & ~SMStatus::MailboxStatus));
                // Sync status byte back to register image
                registers_[ESCReg::SM0 + i * ESCReg::SMSize + 5] = std::bit_cast<uint8_t>(sm.status);
                break;
            }
        }
    }

    return true;
}

bool SlaveCore::writeProcessData(uint16_t addr, const uint8_t* data, uint16_t len) {
    if (addr + len > processDataRAM_.size()) return false;

    // ESC mailbox protection: check if this write targets a mailbox SM.
    const uint16_t physAddr = addr + ESCReg::ProcessDataRAM;
    for (size_t i = 0; i < syncManagers_.size(); ++i) {
        auto& sm = syncManagers_[i];
        if (!sm.isEnabled() || !sm.isMailbox()) continue;
        if (physAddr >= sm.physicalAddr &&
            physAddr + len <= sm.physicalAddr + sm.length) {
            // Reject writes to a read-direction SM (slave→master).
            // The ESC hardware rejects PWR to a read-direction sync manager with wkc=0.
            if (sm.isDirectionRead()) {
                return false;
            }
            // Reject writes to a full mailbox — the ESC protects unread data.
            if (std::bit_cast<uint8_t>(sm.status) & SMStatus::MailboxStatus) {
                return false;
            }
            // Write-direction mailbox SM: write data and set MailboxStatus.
            std::memcpy(&processDataRAM_[addr], data, len);
            sm.status = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(static_cast<uint8_t>(std::bit_cast<uint8_t>(sm.status) | SMStatus::MailboxStatus));
            // Sync status byte back to register image
            registers_[ESCReg::SM0 + i * ESCReg::SMSize + 5] = std::bit_cast<uint8_t>(sm.status);
            return true;
        }
    }

    // Non-mailbox write: proceed normally
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
    siiEmulator_.processCommand();
}

void SlaveCore::updateWatchdog(uint64_t deltaNs) {
    watchdogMgr_.update(stateMgr_.status().state, deltaNs);
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
        if (std::bit_cast<uint8_t>(sm.status) & SMStatus::MailboxStatus) {
            // ESC guard: if SM1 (slave→master) still has an unconsumed response
            // (MailboxStatus set), do not process the new request — the master
            // must first read the pending response. Real ESC hardware enforces
            // this by blocking writes to a full read-direction sync manager.
            if (syncManagers_[1].isEnabled() &&
                (std::bit_cast<uint8_t>(syncManagers_[1].status) & SMStatus::MailboxStatus)) {
                return;
            }
            // Copy data from process RAM to mailbox buffer
            if (sm.physicalAddr >= ESCReg::ProcessDataRAM) {
                size_t ramOffset = sm.physicalAddr - ESCReg::ProcessDataRAM;
                if (ramOffset + sm.length <= processDataRAM_.size()) {
                    std::memcpy(mailboxOut_.data(), &processDataRAM_[ramOffset], 
                               std::min(static_cast<size_t>(sm.length), mailboxOut_.size()));
                    mailboxOutFull_.store(true);
                    sm.status = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(static_cast<uint8_t>(std::bit_cast<uint8_t>(sm.status) & ~SMStatus::MailboxStatus));  // Clear mailbox full
                    // Sync status byte back to register image
                    registers_[ESCReg::SM0 + 5] = std::bit_cast<uint8_t>(sm.status);

                    // Dispatch to mailbox handlers and generate response
                    for (auto& handler : mailboxHandlers_) {
                        uint8_t responseBuf[256] = {0};
                        size_t responseLen = sizeof(responseBuf);
                        if (handler->processRequest(mailboxOut_.data(), sm.length,
                                                     responseBuf, responseLen)) {
                            // Clamp responseLen to buffer capacity as a safety
                            // net — handlers should respect the input value of
                            // responseLen but this prevents overflow if they don't.
                            if (responseLen > sizeof(responseBuf)) {
                                responseLen = sizeof(responseBuf);
                            }
                            std::memcpy(mailboxIn_.data(), responseBuf,
                                       std::min(responseLen, mailboxIn_.size()));
                            mailboxInFull_.store(true);
                            break;
                        }
                    }
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
                sm.status = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(static_cast<uint8_t>(std::bit_cast<uint8_t>(sm.status) | SMStatus::MailboxStatus));  // Set mailbox full
                // Sync status byte back to register image
                registers_[ESCReg::SM0 + 1 * ESCReg::SMSize + 5] = std::bit_cast<uint8_t>(sm.status);
            }
        }
    }
}

void SlaveCore::initializeSII() {
    siiEmulator_.initializeFromIdentity(config_.identity);
}

void SlaveCore::initializeSyncManagers() {
    // SM0: Mailbox In (Master → Slave)
    syncManagers_[0].physicalAddr = config_.mailboxOutOffset;
    syncManagers_[0].length = config_.mailboxOutSize;
    syncManagers_[0].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x26));  // Mailbox, write, watchdog
    syncManagers_[0].type = SyncManagerType::MailboxOut;
    
    // SM1: Mailbox Out (Slave → Master)
    syncManagers_[1].physicalAddr = config_.mailboxInOffset;
    syncManagers_[1].length = config_.mailboxInSize;
    syncManagers_[1].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x22));  // Mailbox, read, watchdog
    syncManagers_[1].type = SyncManagerType::MailboxIn;
    
    // SM2: RxPDO (Master → Slave outputs)
    if (config_.rxPdoSize > 0) {
        syncManagers_[2].physicalAddr = config_.rxPdoOffset;
        syncManagers_[2].length = config_.rxPdoSize;
        syncManagers_[2].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x44));  // Buffered, write, repeat request
        syncManagers_[2].type = SyncManagerType::ProcessOut;
    }
    
    // SM3: TxPDO (Slave → Master inputs)
    if (config_.txPdoSize > 0) {
        syncManagers_[3].physicalAddr = config_.txPdoOffset;
        syncManagers_[3].length = config_.txPdoSize;
        syncManagers_[3].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x40));  // Buffered, read, repeat request
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
