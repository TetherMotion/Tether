/**
 * @file CiA405Slave.cpp
 * @brief CiA 405 Programmable Devices (IEC 61131-3 PLC) Slave Implementation
 *
 * @details
 * This implementation provides a complete CiA 405 compliant programmable device
 * slave with:
 * - Program state machine (stop/run/hold)
 * - Variable access via SDO (read/write)
 * - I/O area mapping (inputs/outputs)
 * - Program cycle execution
 * - PDO mappings for real-time I/O exchange
 *
 * CiA 405 Object Index Ranges:
 * - 0x4000-0x4FFF: Profile-specific objects
 * - 0x4000: Program control
 * - 0x4100-0x41FF: Input variables
 * - 0x4200-0x42FF: Output variables
 * - 0x4300-0x43FF: Internal variables
 * - 0x4400-0x44FF: Program information
 */

#include "slave/profiles/CiA405Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 405 Object Dictionary Indices
// ============================================================================

namespace CiA405Index {
    // Program Control (0x4000-0x40FF)
    constexpr uint16_t ProgramControl           = 0x4000;  // Program control word
    constexpr uint16_t ProgramStatus            = 0x4001;  // Program status
    constexpr uint16_t ProgramState             = 0x4002;  // Current program state
    constexpr uint16_t ProgramError             = 0x4003;  // Program error code
    constexpr uint16_t CycleTime                = 0x4010;  // Actual cycle time (ns)
    constexpr uint16_t CycleCount               = 0x4011;  // Cycle counter
    constexpr uint16_t MaxCycleTime             = 0x4012;  // Maximum cycle time observed
    constexpr uint16_t MinCycleTime             = 0x4013;  // Minimum cycle time observed
    
    // Input Variables (0x4100-0x41FF)
    constexpr uint16_t InputArea                = 0x4100;  // Input variable area (byte array)
    constexpr uint16_t InputAreaSize            = 0x4101;  // Input area size
    
    // Output Variables (0x4200-0x42FF)
    constexpr uint16_t OutputArea               = 0x4200;  // Output variable area (byte array)
    constexpr uint16_t OutputAreaSize           = 0x4201;  // Output area size
    
    // Internal Variables (0x4300-0x43FF)
    constexpr uint16_t InternalArea             = 0x4300;  // Internal variable area
    constexpr uint16_t InternalAreaSize         = 0x4301;  // Internal area size
    
    // Program Information (0x4400-0x44FF)
    constexpr uint16_t ProgramName              = 0x4400;  // Program name
    constexpr uint16_t ProgramVersion           = 0x4401;  // Program version
    constexpr uint16_t ProgramSize              = 0x4402;  // Program size in bytes
    constexpr uint16_t ProgramChecksum          = 0x4403;  // Program checksum
    constexpr uint16_t ProgramDownload          = 0x4410;  // Program download object
    
    // Variable Directory (0x4500-0x45FF)
    constexpr uint16_t VariableCount            = 0x4500;  // Number of registered variables
    constexpr uint16_t VariableName             = 0x4510;  // Variable name (array)
    constexpr uint16_t VariableOffset           = 0x4520;  // Variable offset (array)
    constexpr uint16_t VariableSize             = 0x4530;  // Variable size (array)
}

// ============================================================================
// Status Bit Definitions
// ============================================================================

namespace CiA405Status {
    // Program status bits (0x4001)
    constexpr uint16_t ProgramLoaded            = 0x0001;  // Program is loaded
    constexpr uint16_t ProgramValid             = 0x0002;  // Program is valid
    constexpr uint16_t ProgramRunning           = 0x0004;  // Program is running
    constexpr uint16_t ProgramHalted            = 0x0008;  // Program is halted
    constexpr uint16_t ProgramError             = 0x0010;  // Program error
    constexpr uint16_t IOError                  = 0x0020;  // I/O error
    constexpr uint16_t OverrunError             = 0x0040;  // Cycle time overrun
    
    // Program control commands (0x4000)
    constexpr uint16_t CmdStop                  = 0x0001;  // Stop program
    constexpr uint16_t CmdRun                   = 0x0002;  // Run program
    constexpr uint16_t CmdReset                 = 0x0004;  // Reset program
    constexpr uint16_t CmdHalt                  = 0x0008;  // Halt program
    constexpr uint16_t CmdResume                = 0x0010;  // Resume from halt
    constexpr uint16_t CmdClearError            = 0x0020;  // Clear errors
}

// ============================================================================
// Static Helper Functions (implementation-local)
// ============================================================================

/**
 * @brief Build the status word from current program state
 */
static uint16_t buildStatusWord(ProgramState programState, bool hasProgram) {
    uint16_t status = 0;
    
    if (hasProgram) {
        status |= CiA405Status::ProgramLoaded;
        status |= CiA405Status::ProgramValid;  // Assume valid if loaded
    }
    
    switch (programState) {
        case ProgramState::Running:
            status |= CiA405Status::ProgramRunning;
            break;
        case ProgramState::Halted:
            status |= CiA405Status::ProgramHalted;
            break;
        default:
            break;
    }
    
    return status;
}

/**
 * @brief Calculate CRC32-style checksum for program data
 */
static uint32_t calculateChecksum(const std::vector<uint8_t>& program) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint8_t byte : program) {
        crc ^= byte;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// ============================================================================
// CiA405Slave Implementation
// ============================================================================

CiA405Slave::CiA405Slave(const CiA405SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA405, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = config.supportsProgramDownload
      })
    , config_(config)
{
    // Initialize variable areas
    inputArea_.resize(config_.inputAreaSize, 0);
    outputArea_.resize(config_.outputAreaSize, 0);
    internalArea_.resize(config_.internalAreaSize, 0);
    
    // Reserve space for program
    program_.reserve(config_.maxProgramSize);
}

CiA405Slave::~CiA405Slave() = default;

// ============================================================================
// Object Dictionary Registration
// ============================================================================

void CiA405Slave::initObjectDictionary() {
    // Register standard CiA 301 communication objects
    ProfileSlave::registerCiA301Objects();
    
    auto& od = getObjectDictionary();
    
    // ========================================================================
    // Program Control Objects (0x4000-0x40FF)
    // ========================================================================
    
    // 0x4000 - Program Control Word
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::ProgramControl,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,  // Read-write
            .name = "Program control",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t control = 0;  // Control word is write-only, returns 0
            std::memcpy(data, &control, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t control;
            std::memcpy(&control, data, 2);
            
            // Process control word inline
            if (control & CiA405Status::CmdStop) {
                stopProgram();
            } else if (control & CiA405Status::CmdRun) {
                startProgram();
            } else if (control & CiA405Status::CmdReset) {
                resetProgram();
            } else if (control & CiA405Status::CmdHalt) {
                if (programState_ == ProgramState::Running) {
                    programState_ = ProgramState::Halted;
                }
            } else if (control & CiA405Status::CmdResume) {
                if (programState_ == ProgramState::Halted) {
                    programState_ = ProgramState::Running;
                }
            }
            return SDOAbortCode::Success;
        }
    );
    
    // 0x4001 - Program Status
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::ProgramStatus,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,  // Read-only
            .name = "Program status",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t status = buildStatusWord(programState_, !program_.empty());
            std::memcpy(data, &status, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x4002 - Program State
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::ProgramState,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,
            .name = "Program state",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(programState_);
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x4010 - Cycle Time (computed from accumulated time / cycle count)
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::CycleTime,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Cycle time (ns)",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            // Compute average cycle time from accumulated time
            uint32_t avgCycleTime = (cycleCount_ > 0) 
                ? static_cast<uint32_t>(cycleTimeAccum_ / cycleCount_) 
                : 0;
            std::memcpy(data, &avgCycleTime, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x4011 - Cycle Count
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::CycleCount,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Cycle count",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &cycleCount_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Input Area Objects (0x4100-0x41FF)
    // ========================================================================
    
    // 0x4100 - Input Area (as byte array)
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::InputArea,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,
            .name = "Input area size",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(std::min(inputArea_.size(), size_t(255)));
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // Register individual input bytes (limit to 64 for manageable OD size)
    for (size_t i = 0; i < std::min(static_cast<size_t>(config_.inputAreaSize), size_t(64)); i++) {
        uint8_t subidx = static_cast<uint8_t>(i + 1);
        od.registerObject(
            ODEntryInfo{
                .index = CiA405Index::InputArea,
                .subindex = subidx,
                .dataType = ObjectDictionaryDataType::Unsigned8,
                .bitLength = 8,
                .accessType = 0x3F,  // Read-write (master writes inputs)
                .name = "Input byte",
                .defaultValue = 0
            },
            [this, i](uint8_t* data, size_t& len) -> SDOAbortCode {
                if (len < 1) return SDOAbortCode::DataTypeMismatch;
                data[0] = inputArea_[i];
                len = 1;
                return SDOAbortCode::Success;
            },
            [this, i](const uint8_t* data, size_t len) -> SDOAbortCode {
                if (len != 1) return SDOAbortCode::DataTypeMismatch;
                inputArea_[i] = data[0];
                return SDOAbortCode::Success;
            }
        );
    }
    
    // ========================================================================
    // Output Area Objects (0x4200-0x42FF)
    // ========================================================================
    
    // 0x4200 - Output Area
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::OutputArea,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,
            .name = "Output area size",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(std::min(outputArea_.size(), size_t(255)));
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // Register individual output bytes
    for (size_t i = 0; i < std::min(static_cast<size_t>(config_.outputAreaSize), size_t(64)); i++) {
        uint8_t subidx = static_cast<uint8_t>(i + 1);
        od.registerObject(
            ODEntryInfo{
                .index = CiA405Index::OutputArea,
                .subindex = subidx,
                .dataType = ObjectDictionaryDataType::Unsigned8,
                .bitLength = 8,
                .accessType = 0x01,  // Read-only (PLC writes outputs)
                .name = "Output byte",
                .defaultValue = 0
            },
            [this, i](uint8_t* data, size_t& len) -> SDOAbortCode {
                if (len < 1) return SDOAbortCode::DataTypeMismatch;
                data[0] = outputArea_[i];
                len = 1;
                return SDOAbortCode::Success;
            },
            nullptr
        );
    }
    
    // ========================================================================
    // Program Information Objects (0x4400-0x44FF)
    // ========================================================================
    
    // 0x4402 - Program Size
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::ProgramSize,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Program size",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t size = static_cast<uint32_t>(program_.size());
            std::memcpy(data, &size, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x4403 - Program Checksum
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::ProgramChecksum,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Program checksum",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t checksum = calculateChecksum(program_);
            std::memcpy(data, &checksum, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x4500 - Variable Count
    od.registerObject(
        ODEntryInfo{
            .index = CiA405Index::VariableCount,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Variable count",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t count = static_cast<uint16_t>(variables_.size());
            std::memcpy(data, &count, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
}

// ============================================================================
// PDO Mappings
// ============================================================================

void CiA405Slave::initPDOMappings() {
    // ========================================================================
    // TxPDO Mapping (Slave -> Master, SM3)
    // ========================================================================
    // CiA 405 TxPDO:
    // - Program status (0x4001:0) - 16 bits
    // - Program state (0x4002:0) - 8 bits
    // - Output area bytes (0x4200:1..n)
    
    std::vector<uint32_t> txPdoMapping;
    
    // Status word (2 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA405Index::ProgramStatus, 0, 16));
    pdoLayout_.statusOffset = 0;
    
    // Program state (1 byte)
    txPdoMapping.push_back(PDOMapEntry(CiA405Index::ProgramState, 0, 8));
    pdoLayout_.stateOffset = 2;
    
    size_t offset = 3;
    pdoLayout_.outputAreaOffset = offset;
    
    // Output area bytes (PLC outputs -> master reads)
    // Limit to reasonable PDO size (32 bytes typical)
    size_t pdoOutputBytes = std::min(static_cast<size_t>(config_.outputAreaSize), size_t(32));
    for (size_t i = 0; i < pdoOutputBytes; i++) {
        txPdoMapping.push_back(PDOMapEntry(CiA405Index::OutputArea, static_cast<uint8_t>(i + 1), 8));
        offset += 1;
    }
    
    registerPDOMapping(0x1A00, txPdoMapping);
    
    // ========================================================================
    // RxPDO Mapping (Master -> Slave, SM2)
    // ========================================================================
    // CiA 405 RxPDO:
    // - Control word (0x4000:0) - 16 bits
    // - Input area bytes (0x4100:1..n)
    
    std::vector<uint32_t> rxPdoMapping;
    
    // Control word (2 bytes)
    rxPdoMapping.push_back(PDOMapEntry(CiA405Index::ProgramControl, 0, 16));
    pdoLayout_.controlOffset = 0;
    
    offset = 2;
    pdoLayout_.inputAreaOffset = offset;
    
    // Input area bytes (master -> PLC inputs)
    size_t pdoInputBytes = std::min(static_cast<size_t>(config_.inputAreaSize), size_t(32));
    for (size_t i = 0; i < pdoInputBytes; i++) {
        rxPdoMapping.push_back(PDOMapEntry(CiA405Index::InputArea, static_cast<uint8_t>(i + 1), 8));
        offset += 1;
    }
    
    registerPDOMapping(0x1600, rxPdoMapping);
}

// ============================================================================
// TxPDO Update (Slave -> Master)
// ============================================================================

void CiA405Slave::updateTxPDO() {
    auto* txData = getTxPDOPtr<uint8_t>(0);
    if (!txData) return;
    
    // Build and write status word
    uint16_t status = buildStatusWord(programState_, !program_.empty());
    std::memcpy(txData + pdoLayout_.statusOffset, &status, 2);
    
    // Write program state
    txData[pdoLayout_.stateOffset] = static_cast<uint8_t>(programState_);
    
    // Write output area bytes (PLC outputs to master)
    size_t pdoOutputBytes = std::min(static_cast<size_t>(config_.outputAreaSize), size_t(32));
    for (size_t i = 0; i < pdoOutputBytes; i++) {
        txData[pdoLayout_.outputAreaOffset + i] = outputArea_[i];
    }
}

// ============================================================================
// RxPDO Processing (Master -> Slave)
// ============================================================================

void CiA405Slave::processRxPDO() {
    const auto* rxData = getRxPDOPtr<uint8_t>(0);
    if (!rxData) return;
    
    // Read control word and process commands
    uint16_t controlWord;
    std::memcpy(&controlWord, rxData + pdoLayout_.controlOffset, 2);
    
    // Process control word inline
    if (controlWord & CiA405Status::CmdStop) {
        stopProgram();
    } else if (controlWord & CiA405Status::CmdRun) {
        startProgram();
    } else if (controlWord & CiA405Status::CmdReset) {
        resetProgram();
    } else if (controlWord & CiA405Status::CmdHalt) {
        if (programState_ == ProgramState::Running) {
            programState_ = ProgramState::Halted;
        }
    } else if (controlWord & CiA405Status::CmdResume) {
        if (programState_ == ProgramState::Halted) {
            programState_ = ProgramState::Running;
        }
    }
    
    // Read input area bytes (from master to PLC inputs)
    size_t pdoInputBytes = std::min(static_cast<size_t>(config_.inputAreaSize), size_t(32));
    for (size_t i = 0; i < pdoInputBytes; i++) {
        inputArea_[i] = rxData[pdoLayout_.inputAreaOffset + i];
    }
}

// ============================================================================
// Simulation / Program Execution
// ============================================================================

void CiA405Slave::simulate(uint64_t deltaNs) {
    // Track cycle time
    cycleTimeAccum_ += deltaNs;
    
    // Only execute if program is running
    if (programState_ != ProgramState::Running) {
        return;
    }
    
    // Increment cycle count
    cycleCount_++;
    
    // Execute user callback if registered
    if (cycleCallback_) {
        cycleCallback_();
    }
    
    // Default simulation: simple pass-through with modifications
    // Copy first N input bytes to first N output bytes (echo mode)
    // This simulates a basic PLC program
    size_t copySize = std::min(inputArea_.size(), outputArea_.size());
    for (size_t i = 0; i < copySize; i++) {
        outputArea_[i] = inputArea_[i];
    }
}

// ============================================================================
// Program Control
// ============================================================================

void CiA405Slave::setProgramState(ProgramState state) {
    programState_ = state;
}

void CiA405Slave::startProgram() {
    if (program_.empty() && !cycleCallback_) {
        // No program loaded and no callback - still allow running for I/O simulation
    }
    
    if (programState_ == ProgramState::Stopped || 
        programState_ == ProgramState::Halted) {
        ProgramState oldState = programState_;
        programState_ = ProgramState::Running;
        if (oldState == ProgramState::Stopped) {
            // Full restart - reset cycle count
            cycleCount_ = 0;
            cycleTimeAccum_ = 0;
        }
    }
}

void CiA405Slave::stopProgram() {
    programState_ = ProgramState::Stopped;
}

void CiA405Slave::resetProgram() {
    programState_ = ProgramState::Stopped;
    cycleCount_ = 0;
    cycleTimeAccum_ = 0;
    
    // Clear variable areas
    std::fill(inputArea_.begin(), inputArea_.end(), 0);
    std::fill(outputArea_.begin(), outputArea_.end(), 0);
    std::fill(internalArea_.begin(), internalArea_.end(), 0);
}

// ============================================================================
// Variable Access
// ============================================================================

bool CiA405Slave::readVariable(const std::string& name, std::vector<uint8_t>& data) const {
    auto it = variables_.find(name);
    if (it == variables_.end()) {
        return false;
    }
    
    const VariableInfo& var = it->second;
    
    // Determine which area to read from
    const std::vector<uint8_t>* area = nullptr;
    if (var.isInput) {
        area = &inputArea_;
    } else if (var.isOutput) {
        area = &outputArea_;
    } else {
        area = &internalArea_;
    }
    
    // Check bounds
    if (var.offset + var.size > area->size()) {
        return false;
    }
    
    // Copy data
    data.resize(var.size);
    std::memcpy(data.data(), area->data() + var.offset, var.size);
    return true;
}

bool CiA405Slave::writeVariable(const std::string& name, const std::vector<uint8_t>& data) {
    auto it = variables_.find(name);
    if (it == variables_.end()) {
        return false;
    }
    
    const VariableInfo& var = it->second;
    
    // Check data size
    if (data.size() != var.size) {
        return false;
    }
    
    // Determine which area to write to
    std::vector<uint8_t>* area = nullptr;
    if (var.isInput) {
        area = &inputArea_;
    } else if (var.isOutput) {
        area = &outputArea_;
    } else {
        area = &internalArea_;
    }
    
    // Check bounds
    if (var.offset + var.size > area->size()) {
        return false;
    }
    
    // Copy data
    std::memcpy(area->data() + var.offset, data.data(), var.size);
    return true;
}

void CiA405Slave::registerVariable(const std::string& name, uint16_t offset, uint16_t size,
                                   bool isInput, bool isOutput) {
    VariableInfo var;
    var.name = name;
    var.offset = offset;
    var.size = size;
    var.isInput = isInput;
    var.isOutput = isOutput;
    
    variables_[name] = var;
}

// ============================================================================
// Program Download
// ============================================================================

bool CiA405Slave::loadProgram(const std::vector<uint8_t>& program) {
    if (programState_ == ProgramState::Running) {
        return false;  // Cannot load while running
    }
    
    if (program.size() > config_.maxProgramSize) {
        return false;  // Program too large
    }
    
    if (!config_.supportsProgramDownload) {
        return false;
    }
    
    program_ = program;
    return true;
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<CiA405Slave> createCiA405Slave(const CiA405SlaveConfig& config) {
    return std::make_unique<CiA405Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
