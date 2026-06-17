/**
 * @file CiA405PLC.hpp
 * @brief CiA 405 IEC 61131-3 Programmable Device Controller
 *
 * Provides comprehensive control over CiA 405 compliant programmable devices
 * (PLCs, soft PLCs, PACs) following IEC 61131-3 standards.
 *
 * Features:
 * - Program lifecycle management (start/stop/reset)
 * - Task management and monitoring
 * - Variable read/write access by index or name
 * - Memory area access (%I, %Q, %M)
 * - Exception monitoring and handling
 * - Online debugging support
 * - File transfer for program upload/download
 * - Resource monitoring (CPU, memory, cycle time)
 */

#pragma once

#include "profiles/cia405/CiA405Defs.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <map>

namespace EtherCAT { namespace CoE { class CoEManager; } }

namespace CiA405 {

// ============================================================================
// Forward Declarations
// ============================================================================

class PLCDevice;

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Program information
 */
struct ProgramInfo {
    std::string name;
    std::string version;
    uint32_t    checksum = 0;
    uint32_t    size = 0;
    uint8_t     state = ProgramState::Unknown;
    
    bool isRunning() const { return state == ProgramState::Running; }
    bool isStopped() const { return state == ProgramState::Stopped; }
    bool hasException() const { return state == ProgramState::Exception; }
};

/**
 * @brief Task configuration and status
 */
struct TaskInfo {
    uint8_t     id = 0;
    std::string name;
    uint8_t     priority = 0;
    uint32_t    interval_us = 0;     // Task interval in microseconds
    uint32_t    watchdog_ms = 0;     // Watchdog time in milliseconds
    uint8_t     state = TaskState::Unknown;
    
    // Runtime statistics
    uint32_t    cycle_count = 0;
    uint32_t    last_cycle_time_us = 0;
    uint32_t    max_cycle_time_us = 0;
    uint32_t    min_cycle_time_us = UINT32_MAX;
    
    bool isRunning() const { return state == TaskState::Running; }
};

/**
 * @brief Variable descriptor for symbol table
 */
struct VariableInfo {
    uint16_t    index = 0;
    std::string name;
    uint8_t     data_type = 0;
    uint32_t    address = 0;
    uint16_t    size = 0;
    bool        is_retain = false;
    bool        is_constant = false;
    
    // For arrays
    uint16_t    array_length = 0;
    uint8_t     element_type = 0;
};

/**
 * @brief Exception information
 */
struct ExceptionInfo {
    uint16_t    code = ExceptionCodes::None;
    uint8_t     task_id = 0;
    uint32_t    address = 0;
    uint32_t    timestamp = 0;
    std::string message;
    
    bool hasException() const { return code != ExceptionCodes::None; }
};

/**
 * @brief Debug breakpoint configuration
 */
struct Breakpoint {
    uint8_t     id = 0;
    uint32_t    address = 0;
    uint16_t    line = 0;
    std::string pou_name;
    std::string condition;
    bool        enabled = true;
    uint32_t    hit_count = 0;
};

/**
 * @brief Resource usage statistics
 */
struct ResourceUsage {
    // Memory
    uint32_t memory_total = 0;
    uint32_t memory_used = 0;
    uint32_t memory_free = 0;
    uint32_t stack_size = 0;
    uint32_t stack_used = 0;
    
    // CPU
    uint16_t cpu_load = 0;          // In 0.01%
    uint16_t cpu_load_max = 0;
    int16_t  cpu_temperature = 0;   // In 0.1°C
    
    // Timing
    uint16_t cycle_time_us = 0;
    uint16_t max_cycle_time_us = 0;
    uint16_t min_cycle_time_us = 0;
    
    float getCPULoadPercent() const { return cpu_load / 100.0f; }
    float getCPUTempCelsius() const { return cpu_temperature / 10.0f; }
};

/**
 * @brief File transfer status
 */
struct FileTransferStatus {
    uint8_t     state = FileTransferState::Idle;
    std::string filename;
    uint32_t    total_size = 0;
    uint32_t    transferred = 0;
    uint8_t     progress_percent = 0;
    uint32_t    checksum = 0;
    
    bool isActive() const { 
        return state == FileTransferState::Downloading || 
               state == FileTransferState::Uploading; 
    }
};

/**
 * @brief Device capabilities
 */
struct PLCCapabilities {
    uint8_t  max_tasks = 0;
    uint16_t max_variables = 0;
    uint32_t input_size = 0;       // Bytes
    uint32_t output_size = 0;      // Bytes
    uint32_t memory_size = 0;      // Bytes
    bool     supports_debug = false;
    bool     supports_file_transfer = false;
    bool     supports_online_change = false;
    uint8_t  max_breakpoints = 0;
    uint8_t  max_watch_variables = 0;
};

// ============================================================================
// Callback Types
// ============================================================================

using ProgramStateCallback = std::function<void(uint8_t old_state, uint8_t new_state)>;
using TaskStateCallback = std::function<void(uint8_t task_id, uint8_t old_state, uint8_t new_state)>;
using ExceptionCallback = std::function<void(const ExceptionInfo& exception)>;
using BreakpointCallback = std::function<void(const Breakpoint& breakpoint)>;
using VariableCallback = std::function<void(uint16_t index, const void* value, size_t size)>;

// ============================================================================
// PDO Mapping Presets
// ============================================================================

enum class PDOMappingPreset {
    Minimal,            // Basic status only
    Standard,           // Status + timing
    WithTasks,          // Status + all task states
    FullControl,        // Status + timing + I/O image
    Debug,              // Status + debug info + watch variables
    Custom
};

// ============================================================================
// PLC Device Class
// ============================================================================

class PLCDevice {
public:
    // ========================================================================
    // Construction and Lifecycle
    // ========================================================================
    
    /**
     * @brief Construct PLC device controller
     * @param coe CoEManager instance for SDO access (per-slave)
     */
    explicit PLCDevice(EtherCAT::CoE::CoEManager& coe);
    
    ~PLCDevice();
    
    /**
     * @brief Initialize the PLC device
     * @return true if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Check if device is initialized
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get device capabilities
     */
    const PLCCapabilities& getCapabilities() const { return capabilities_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    /**
     * @brief Apply PDO mapping preset
     */
    bool applyPDOMapping(PDOMappingPreset preset);
    
    // ========================================================================
    // Cyclic Update
    // ========================================================================
    
    /**
     * @brief Process received TxPDO data
     */
    void processTxPDO(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare RxPDO data for transmission
     */
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    
    /**
     * @brief Update device state (SDO-based, non-cyclic)
     */
    void update();
    
    // ========================================================================
    // Program Control
    // ========================================================================
    
    /**
     * @brief Start program execution
     * @param cold_start Perform cold start (clear variables)
     */
    bool startProgram(bool cold_start = false);
    
    /**
     * @brief Stop program execution
     */
    bool stopProgram();
    
    /**
     * @brief Halt program (pause, retains state)
     */
    bool haltProgram();
    
    /**
     * @brief Continue from halt
     */
    bool continueProgram();
    
    /**
     * @brief Reset program (reinitialize)
     */
    bool resetProgram();
    
    /**
     * @brief Get current program state
     */
    uint8_t getProgramState() const;
    
    /**
     * @brief Check if program is running
     */
    bool isProgramRunning() const;
    
    /**
     * @brief Get program information
     */
    const ProgramInfo& getProgramInfo() const { return program_info_; }
    
    // ========================================================================
    // Task Management
    // ========================================================================
    
    /**
     * @brief Start a task
     */
    bool startTask(uint8_t task_id);
    
    /**
     * @brief Stop a task
     */
    bool stopTask(uint8_t task_id);
    
    /**
     * @brief Suspend a task
     */
    bool suspendTask(uint8_t task_id);
    
    /**
     * @brief Resume a task
     */
    bool resumeTask(uint8_t task_id);
    
    /**
     * @brief Run single cycle of a task
     */
    bool singleCycleTask(uint8_t task_id);
    
    /**
     * @brief Get task information
     */
    const TaskInfo& getTaskInfo(uint8_t task_id) const;
    
    /**
     * @brief Configure task parameters
     */
    bool configureTask(uint8_t task_id, uint8_t priority, 
                       uint32_t interval_us, uint32_t watchdog_ms = 0);
    
    /**
     * @brief Get number of tasks
     */
    uint8_t getTaskCount() const { return static_cast<uint8_t>(tasks_.size()); }
    
    // ========================================================================
    // Variable Access
    // ========================================================================
    
    /**
     * @brief Read variable by index
     */
    bool readVariable(uint16_t index, void* value, size_t max_len, size_t* actual_len = nullptr);
    
    /**
     * @brief Write variable by index
     */
    bool writeVariable(uint16_t index, const void* value, size_t len);
    
    /**
     * @brief Read variable by name
     */
    bool readVariableByName(const std::string& name, void* value, size_t max_len);
    
    /**
     * @brief Write variable by name
     */
    bool writeVariableByName(const std::string& name, const void* value, size_t len);
    
    /**
     * @brief Get variable info
     */
    const VariableInfo* getVariableInfo(uint16_t index) const;
    const VariableInfo* getVariableInfo(const std::string& name) const;
    
    /**
     * @brief Load symbol table from device
     */
    bool loadSymbolTable();
    
    /**
     * @brief Get all variables
     */
    const std::vector<VariableInfo>& getVariables() const { return variables_; }
    
    // Typed variable access
    bool readBool(uint16_t index, bool& value);
    bool writeBool(uint16_t index, bool value);
    bool readInt16(uint16_t index, int16_t& value);
    bool writeInt16(uint16_t index, int16_t value);
    bool readInt32(uint16_t index, int32_t& value);
    bool writeInt32(uint16_t index, int32_t value);
    bool readReal(uint16_t index, float& value);
    bool writeReal(uint16_t index, float value);
    
    // ========================================================================
    // Memory Area Access
    // ========================================================================
    
    /**
     * @brief Read input memory (%I)
     */
    bool readInputByte(uint16_t offset, uint8_t& value);
    bool readInputWord(uint16_t offset, uint16_t& value);
    bool readInputDWord(uint16_t offset, uint32_t& value);
    
    /**
     * @brief Read output memory (%Q)
     */
    bool readOutputByte(uint16_t offset, uint8_t& value);
    bool readOutputWord(uint16_t offset, uint16_t& value);
    bool readOutputDWord(uint16_t offset, uint32_t& value);
    
    /**
     * @brief Write output memory (%Q)
     */
    bool writeOutputByte(uint16_t offset, uint8_t value);
    bool writeOutputWord(uint16_t offset, uint16_t value);
    bool writeOutputDWord(uint16_t offset, uint32_t value);
    
    /**
     * @brief Read marker/memory (%M)
     */
    bool readMemoryByte(uint16_t offset, uint8_t& value);
    bool readMemoryWord(uint16_t offset, uint16_t& value);
    bool readMemoryDWord(uint16_t offset, uint32_t& value);
    
    /**
     * @brief Write marker/memory (%M)
     */
    bool writeMemoryByte(uint16_t offset, uint8_t value);
    bool writeMemoryWord(uint16_t offset, uint16_t value);
    bool writeMemoryDWord(uint16_t offset, uint32_t value);
    
    // ========================================================================
    // Exception Handling
    // ========================================================================
    
    /**
     * @brief Get current exception
     */
    const ExceptionInfo& getCurrentException() const { return current_exception_; }
    
    /**
     * @brief Check for active exception
     */
    bool hasException() const { return current_exception_.hasException(); }
    
    /**
     * @brief Clear exception
     */
    bool clearException();
    
    /**
     * @brief Get exception history
     */
    const std::vector<ExceptionInfo>& getExceptionHistory() const { return exception_history_; }
    
    // ========================================================================
    // Debug Interface
    // ========================================================================
    
    /**
     * @brief Enable debug mode
     */
    bool enableDebug();
    
    /**
     * @brief Disable debug mode
     */
    bool disableDebug();
    
    /**
     * @brief Check if in debug mode
     */
    bool isDebugEnabled() const { return debug_enabled_; }
    
    /**
     * @brief Set breakpoint
     */
    bool setBreakpoint(uint32_t address, const std::string& condition = "");
    
    /**
     * @brief Clear breakpoint
     */
    bool clearBreakpoint(uint8_t breakpoint_id);
    
    /**
     * @brief Clear all breakpoints
     */
    bool clearAllBreakpoints();
    
    /**
     * @brief Get breakpoints
     */
    const std::vector<Breakpoint>& getBreakpoints() const { return breakpoints_; }
    
    /**
     * @brief Step into (next instruction)
     */
    bool stepInto();
    
    /**
     * @brief Step over (skip function calls)
     */
    bool stepOver();
    
    /**
     * @brief Step out (run until return)
     */
    bool stepOut();
    
    /**
     * @brief Run to cursor
     */
    bool runToCursor(uint32_t address);
    
    /**
     * @brief Get current execution position
     */
    uint32_t getCurrentAddress() const { return current_address_; }
    uint16_t getCurrentLine() const { return current_line_; }
    
    /**
     * @brief Add watch variable
     */
    bool addWatch(uint16_t var_index);
    
    /**
     * @brief Remove watch variable
     */
    bool removeWatch(uint16_t var_index);
    
    // ========================================================================
    // File Transfer
    // ========================================================================
    
    /**
     * @brief Download program to device
     */
    bool downloadProgram(const uint8_t* data, size_t len, const std::string& filename);
    
    /**
     * @brief Upload program from device
     */
    bool uploadProgram(std::vector<uint8_t>& data);
    
    /**
     * @brief Delete file on device
     */
    bool deleteFile(const std::string& filename);
    
    /**
     * @brief Get file transfer status
     */
    const FileTransferStatus& getFileTransferStatus() const { return file_transfer_; }
    
    /**
     * @brief Abort ongoing file transfer
     */
    bool abortFileTransfer();
    
    // ========================================================================
    // Timing and Resources
    // ========================================================================
    
    /**
     * @brief Get resource usage
     */
    const ResourceUsage& getResourceUsage() const { return resources_; }
    
    /**
     * @brief Get system time
     */
    uint32_t getSystemTime() const { return system_time_; }
    
    /**
     * @brief Get current cycle time
     */
    uint16_t getCycleTime() const { return resources_.cycle_time_us; }
    
    /**
     * @brief Set watchdog time
     */
    bool setWatchdogTime(uint32_t time_ms);
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setProgramStateCallback(ProgramStateCallback callback);
    void setTaskStateCallback(TaskStateCallback callback);
    void setExceptionCallback(ExceptionCallback callback);
    void setBreakpointCallback(BreakpointCallback callback);
    void setVariableCallback(VariableCallback callback);
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get diagnostic string
     */
    std::string getDiagnostics() const;

private:
    // ========================================================================
    // Private Methods
    // ========================================================================
    
    bool detectCapabilities();
    bool applyDefaultConfiguration();
    void processStatusPDO(const uint8_t* data, size_t len);
    void processDebugPDO(const uint8_t* data, size_t len);
    void checkStateChanges();
    void fireException(const ExceptionInfo& ex);
    
    bool sendProgramCommand(uint8_t command);
    bool sendTaskCommand(uint8_t task_id, uint8_t command);
    
    uint16_t findVariableByName(const std::string& name) const;
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    // ========================================================================
    // Private Data
    // ========================================================================
    
    EtherCAT::CoE::CoEManager& m_coe;
    bool initialized_;
    
    PLCCapabilities capabilities_;
    PDOMappingPreset current_mapping_;
    
    // Program state
    ProgramInfo program_info_;
    uint8_t prev_program_state_;
    
    // Task management
    std::vector<TaskInfo> tasks_;
    std::vector<uint8_t> prev_task_states_;
    
    // Variables
    std::vector<VariableInfo> variables_;
    std::map<std::string, uint16_t> variable_name_map_;
    
    // Exception handling
    ExceptionInfo current_exception_;
    std::vector<ExceptionInfo> exception_history_;
    
    // Debug state
    bool debug_enabled_;
    std::vector<Breakpoint> breakpoints_;
    std::vector<uint16_t> watch_variables_;
    uint32_t current_address_;
    uint16_t current_line_;
    
    // File transfer
    FileTransferStatus file_transfer_;
    std::vector<uint8_t> file_buffer_;
    
    // Resources and timing
    ResourceUsage resources_;
    uint32_t system_time_;
    
    // Pending commands for PDO
    uint8_t pending_program_cmd_;
    std::vector<uint8_t> pending_task_cmds_;
    
    // Callbacks
    ProgramStateCallback program_state_callback_;
    TaskStateCallback task_state_callback_;
    ExceptionCallback exception_callback_;
    BreakpointCallback breakpoint_callback_;
    VariableCallback variable_callback_;
};

} // namespace CiA405
