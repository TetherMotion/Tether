/**
 * @file CiA405Slave.hpp
 * @brief CiA 405 Programmable Devices (IEC 61131-3 PLC) Slave
 *
 * @details
 * Implements a CiA 405 compliant programmable device slave with:
 * - Program control (run, stop, reset)
 * - Variable access
 * - Error handling
 * - Program download support
 *
 * This profile is used for PLC-like devices that can execute programs.
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 405 Program State
// ============================================================================

enum class ProgramState : uint8_t {
    Stopped    = 0,
    Running    = 1,
    Halted     = 2,
    Suspended  = 3,
};

// ============================================================================
// CiA 405 Slave Configuration
// ============================================================================

struct CiA405SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000195,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 405 PLC",
    };
    
    // Variable areas
    uint16_t inputAreaSize = 256;     // Input variable area size
    uint16_t outputAreaSize = 256;    // Output variable area size
    uint16_t internalAreaSize = 1024; // Internal variable area size
    
    // Program support
    bool supportsProgramDownload = true;
    size_t maxProgramSize = 65536;
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 405 Slave Class
// ============================================================================

class CiA405Slave : public ProfileSlave {
public:
    explicit CiA405Slave(const CiA405SlaveConfig& config);
    ~CiA405Slave() override;
    
    const char* getProfileName() const override { return "CiA 405"; }
    uint32_t getDeviceType() const override { return 0x00000195; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Program control
    ProgramState getProgramState() const { return programState_; }
    void setProgramState(ProgramState state);
    
    void startProgram();
    void stopProgram();
    void resetProgram();
    
    // Variable access
    bool readVariable(const std::string& name, std::vector<uint8_t>& data) const;
    bool writeVariable(const std::string& name, const std::vector<uint8_t>& data);
    
    // Register variable
    void registerVariable(const std::string& name, uint16_t offset, uint16_t size,
                         bool isInput, bool isOutput);
    
    // Program download
    bool loadProgram(const std::vector<uint8_t>& program);
    const std::vector<uint8_t>& getProgram() const { return program_; }
    
    // Cycle callback (called each scan cycle when running)
    using CycleCallback = std::function<void()>;
    void setCycleCallback(CycleCallback callback) { cycleCallback_ = callback; }
    
    // Raw area access (for simulation)
    uint8_t* getInputArea() { return inputArea_.data(); }
    uint8_t* getOutputArea() { return outputArea_.data(); }
    uint8_t* getInternalArea() { return internalArea_.data(); }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA405SlaveConfig config_;
    
    // PDO layout (offsets within PDO data, per-instance)
    struct PDOLayout {
        size_t statusOffset = 0;        // TxPDO: 2 bytes program status
        size_t stateOffset = 2;         // TxPDO: 1 byte program state
        size_t outputAreaOffset = 3;    // TxPDO: variable size output data
        size_t controlOffset = 0;       // RxPDO: 2 bytes control word
        size_t inputAreaOffset = 2;     // RxPDO: variable size input data
    };
    PDOLayout pdoLayout_;
    
    ProgramState programState_ = ProgramState::Stopped;
    std::vector<uint8_t> program_;
    
    // Variable areas
    std::vector<uint8_t> inputArea_;
    std::vector<uint8_t> outputArea_;
    std::vector<uint8_t> internalArea_;
    
    // Variable registry
    struct VariableInfo {
        std::string name;
        uint16_t offset;
        uint16_t size;
        bool isInput;
        bool isOutput;
    };
    std::map<std::string, VariableInfo> variables_;
    
    CycleCallback cycleCallback_;
    
    uint32_t cycleCount_ = 0;
    uint64_t cycleTimeAccum_ = 0;
};

std::unique_ptr<CiA405Slave> createCiA405Slave(const CiA405SlaveConfig& config);

}  // namespace slave
}  // namespace EtherCAT
