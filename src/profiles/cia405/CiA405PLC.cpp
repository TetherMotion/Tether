/**
 * @file CiA405PLC.cpp
 * @brief CiA 405 IEC 61131-3 Programmable Device Implementation
 */

#include "profiles/cia405/CiA405PLC.hpp"
#include "tether/platform/EspCompat.hpp"
#include "EtherCATSDO.hpp"

static const char* TAG = "CiA405";
#define LOG_I(fmt, ...) TETHER_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOG_W(fmt, ...) TETHER_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOG_E(fmt, ...) TETHER_LOGE(TAG, fmt, ##__VA_ARGS__)

#include <cstring>
#include <algorithm>

namespace CiA405 {

// ============================================================================
// Construction and Initialization
// ============================================================================

PLCDevice::PLCDevice(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr)
    : m_sdo(sdo)
    , slave_addr_(slave_addr)
    , use_configured_addr_(use_configured_addr)
    , initialized_(false)
    , capabilities_()
    , current_mapping_(PDOMappingPreset::Minimal)
    , program_info_()
    , prev_program_state_(ProgramState::Unknown)
    , current_exception_()
    , debug_enabled_(false)
    , current_address_(0)
    , current_line_(0)
    , file_transfer_()
    , resources_()
    , system_time_(0)
    , pending_program_cmd_(ProgramCommands::NoOp)
{
}

PLCDevice::~PLCDevice() {
}

bool PLCDevice::initialize() {
    LOG_I("Initializing CiA 405 PLC device at address %u", slave_addr_);
    
    // Verify device type
    uint32_t device_type = 0;
    if (!readSDO(0x1000, 0, &device_type, sizeof(device_type))) {
        LOG_E("Failed to read device type");
        return false;
    }
    
    uint16_t profile = device_type & 0xFFFF;
    if (profile != PROFILE_NUMBER) {
        LOG_W("Device profile %u is not CiA 405 (%u)", profile, PROFILE_NUMBER);
    }
    
    if (!detectCapabilities()) {
        LOG_E("Failed to detect capabilities");
        return false;
    }
    
    LOG_I("Tasks: %u, Variables: %u, Debug: %s",
          capabilities_.max_tasks, capabilities_.max_variables,
          capabilities_.supports_debug ? "Yes" : "No");
    
    // Initialize task list
    tasks_.resize(capabilities_.max_tasks);
    prev_task_states_.resize(capabilities_.max_tasks, TaskState::Unknown);
    pending_task_cmds_.resize(capabilities_.max_tasks, TaskCommands::NoOp);
    
    for (uint8_t i = 0; i < capabilities_.max_tasks; i++) {
        tasks_[i].id = i + 1;
    }
    
    if (!applyDefaultConfiguration()) {
        LOG_W("Failed to apply default configuration");
    }
    
    initialized_ = true;
    return true;
}

bool PLCDevice::detectCapabilities() {
    // Check program control
    uint8_t state = 0;
    if (!readSDO(ProgramStatus, 0, &state, 1)) {
        return false;  // Basic control required
    }
    
    // Check tasks (1-4)
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t task_state = 0;
        if (readSDO(TaskStatus1 + i, 0, &task_state, 1)) {
            capabilities_.max_tasks = i + 1;
        } else {
            break;
        }
    }
    
    // Check variable directory
    uint16_t var_count = 0;
    if (readSDO(VariableCount, 0, &var_count, 2)) {
        capabilities_.max_variables = var_count;
    }
    
    // Check memory areas
    uint32_t size = 0;
    if (readSDO(InputVariables, 0, &size, 4)) {
        capabilities_.input_size = size;
    }
    if (readSDO(OutputVariables, 0, &size, 4)) {
        capabilities_.output_size = size;
    }
    if (readSDO(MemoryVariables, 0, &size, 4)) {
        capabilities_.memory_size = size;
    }
    
    // Check debug support
    uint8_t debug_state = 0;
    if (readSDO(DebugStatus, 0, &debug_state, 1)) {
        capabilities_.supports_debug = true;
        capabilities_.max_breakpoints = 8;
        capabilities_.max_watch_variables = 8;
    }
    
    // Check file transfer
    uint8_t file_state = 0;
    if (readSDO(FileTransferStatusIndex, 0, &file_state, 1)) {
        capabilities_.supports_file_transfer = true;
    }
    
    return true;
}

bool PLCDevice::applyDefaultConfiguration() {
    return true;
}

// ============================================================================
// PDO Configuration
// ============================================================================

bool PLCDevice::applyPDOMapping(PDOMappingPreset preset) {
    current_mapping_ = preset;
    return true;
}

// ============================================================================
// Cyclic Update
// ============================================================================

void PLCDevice::processTxPDO(const uint8_t* data, size_t len) {
    if (!initialized_ || len == 0) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Minimal:
        case PDOMappingPreset::Standard:
            processStatusPDO(data, len);
            break;
            
        case PDOMappingPreset::Debug:
            processStatusPDO(data, len);
            if (len > sizeof(TxPDO_Status)) {
                processDebugPDO(data + sizeof(TxPDO_Status), 
                               len - sizeof(TxPDO_Status));
            }
            break;
            
        default:
            processStatusPDO(data, len);
            break;
    }
    
    checkStateChanges();
}

size_t PLCDevice::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!initialized_) return 0;
    
    if (max_len >= sizeof(RxPDO_Control)) {
        RxPDO_Control* pdo = reinterpret_cast<RxPDO_Control*>(data);
        pdo->program_command = pending_program_cmd_;
        
        // Use struct member access instead of pointer arithmetic
        if (pending_task_cmds_.size() > 0) pdo->task1_command = pending_task_cmds_[0];
        if (pending_task_cmds_.size() > 1) pdo->task2_command = pending_task_cmds_[1];
        if (pending_task_cmds_.size() > 2) pdo->task3_command = pending_task_cmds_[2];
        if (pending_task_cmds_.size() > 3) pdo->task4_command = pending_task_cmds_[3];
        
        // Clear pending commands
        pending_program_cmd_ = ProgramCommands::NoOp;
        std::fill(pending_task_cmds_.begin(), pending_task_cmds_.end(), TaskCommands::NoOp);
        
        return sizeof(RxPDO_Control);
    }
    
    return 0;
}

void PLCDevice::update() {
    if (!initialized_) return;
    
    // Read program state
    uint8_t state = 0;
    if (readSDO(ProgramStatus, 0, &state, 1)) {
        program_info_.state = state;
    }
    
    // Read task states
    for (uint8_t i = 0; i < tasks_.size(); i++) {
        if (readSDO(TaskStatus1 + i, 0, &state, 1)) {
            tasks_[i].state = state;
        }
    }
    
    // Read exception
    uint16_t ex_code = 0;
    if (readSDO(ExceptionCode, 0, &ex_code, 2)) {
        if (ex_code != ExceptionCodes::None && 
            ex_code != current_exception_.code) {
            ExceptionInfo ex;
            ex.code = ex_code;
            readSDO(ExceptionInfoIndex, 0, &ex.address, 4);
            fireException(ex);
        }
        current_exception_.code = ex_code;
    }
    
    // Read timing
    readSDO(CycleTime, 0, &resources_.cycle_time_us, 2);
    readSDO(MaxCycleTime, 0, &resources_.max_cycle_time_us, 2);
    readSDO(SystemTime, 0, &system_time_, 4);
    
    // Read resources
    readSDO(CPULoad, 0, &resources_.cpu_load, 2);
    readSDO(MemoryUsed, 0, &resources_.memory_used, 4);
    
    checkStateChanges();
}

void PLCDevice::processStatusPDO(const uint8_t* data, size_t len) {
    if (len >= sizeof(TxPDO_Status)) {
        const TxPDO_Status* pdo = reinterpret_cast<const TxPDO_Status*>(data);
        
        program_info_.state = pdo->program_state;
        
        if (tasks_.size() > 0) tasks_[0].state = pdo->task1_state;
        if (tasks_.size() > 1) tasks_[1].state = pdo->task2_state;
        if (tasks_.size() > 2) tasks_[2].state = pdo->task3_state;
        if (tasks_.size() > 3) tasks_[3].state = pdo->task4_state;
        
        if (pdo->exception_code != ExceptionCodes::None &&
            pdo->exception_code != current_exception_.code) {
            ExceptionInfo ex;
            ex.code = pdo->exception_code;
            ex.timestamp = system_time_;
            fireException(ex);
        }
        current_exception_.code = pdo->exception_code;
    }
}

void PLCDevice::processDebugPDO(const uint8_t* data, size_t len) {
    if (len >= sizeof(TxPDO_Debug)) {
        const TxPDO_Debug* pdo = reinterpret_cast<const TxPDO_Debug*>(data);
        current_address_ = pdo->current_address;
        current_line_ = pdo->current_line;
        
        if (pdo->breakpoint_hit && breakpoint_callback_) {
            for (const auto& bp : breakpoints_) {
                if (bp.enabled && bp.address == current_address_) {
                    breakpoint_callback_(bp);
                    break;
                }
            }
        }
    }
}

void PLCDevice::checkStateChanges() {
    // Check program state change
    if (program_info_.state != prev_program_state_) {
        if (program_state_callback_) {
            program_state_callback_(prev_program_state_, program_info_.state);
        }
        prev_program_state_ = program_info_.state;
    }
    
    // Check task state changes
    for (size_t i = 0; i < tasks_.size() && i < prev_task_states_.size(); i++) {
        if (tasks_[i].state != prev_task_states_[i]) {
            if (task_state_callback_) {
                task_state_callback_(i + 1, prev_task_states_[i], tasks_[i].state);
            }
            prev_task_states_[i] = tasks_[i].state;
        }
    }
}

void PLCDevice::fireException(const ExceptionInfo& ex) {
    current_exception_ = ex;
    exception_history_.push_back(ex);
    
    // Keep only last 10 exceptions
    if (exception_history_.size() > 10) {
        exception_history_.erase(exception_history_.begin());
    }
    
    if (exception_callback_) {
        exception_callback_(ex);
    }
}

// ============================================================================
// Program Control
// ============================================================================

bool PLCDevice::startProgram(bool cold_start) {
    return sendProgramCommand(cold_start ? ProgramCommands::ColdStart : 
                                          ProgramCommands::Start);
}

bool PLCDevice::stopProgram() {
    return sendProgramCommand(ProgramCommands::Stop);
}

bool PLCDevice::haltProgram() {
    return sendProgramCommand(ProgramCommands::Halt);
}

bool PLCDevice::continueProgram() {
    return sendProgramCommand(ProgramCommands::Continue);
}

bool PLCDevice::resetProgram() {
    return sendProgramCommand(ProgramCommands::Reset);
}

uint8_t PLCDevice::getProgramState() const {
    return program_info_.state;
}

bool PLCDevice::isProgramRunning() const {
    return program_info_.state == ProgramState::Running;
}

bool PLCDevice::sendProgramCommand(uint8_t command) {
    // Set pending for PDO, also try SDO
    pending_program_cmd_ = command;
    return writeSDO(ProgramControl, ProgControlSub::Command, &command, 1);
}

// ============================================================================
// Task Management
// ============================================================================

bool PLCDevice::startTask(uint8_t task_id) {
    return sendTaskCommand(task_id, TaskCommands::Start);
}

bool PLCDevice::stopTask(uint8_t task_id) {
    return sendTaskCommand(task_id, TaskCommands::Stop);
}

bool PLCDevice::suspendTask(uint8_t task_id) {
    return sendTaskCommand(task_id, TaskCommands::Suspend);
}

bool PLCDevice::resumeTask(uint8_t task_id) {
    return sendTaskCommand(task_id, TaskCommands::Resume);
}

bool PLCDevice::singleCycleTask(uint8_t task_id) {
    return sendTaskCommand(task_id, TaskCommands::SingleCycle);
}

const TaskInfo& PLCDevice::getTaskInfo(uint8_t task_id) const {
    static TaskInfo empty;
    if (task_id == 0 || task_id > tasks_.size()) return empty;
    return tasks_[task_id - 1];
}

bool PLCDevice::configureTask(uint8_t task_id, uint8_t priority,
                              uint32_t interval_us, uint32_t watchdog_ms) {
    if (task_id == 0 || task_id > tasks_.size()) return false;
    
    auto& task = tasks_[task_id - 1];
    task.priority = priority;
    task.interval_us = interval_us;
    task.watchdog_ms = watchdog_ms;
    
    bool result = writeSDO(TaskPriority1 + (task_id - 1), 0, &priority, 1);
    result &= writeSDO(TaskInterval1 + (task_id - 1), 0, &interval_us, 4);
    if (watchdog_ms > 0) {
        result &= writeSDO(WatchdogTime, task_id, &watchdog_ms, 4);
    }
    
    return result;
}

bool PLCDevice::sendTaskCommand(uint8_t task_id, uint8_t command) {
    if (task_id == 0 || task_id > pending_task_cmds_.size()) return false;
    
    pending_task_cmds_[task_id - 1] = command;
    return writeSDO(TaskControl1 + (task_id - 1), TaskControlSub::Command, &command, 1);
}

// ============================================================================
// Variable Access
// ============================================================================

bool PLCDevice::readVariable(uint16_t index, void* value, size_t max_len, size_t* actual_len) {
    return readSDO(VariableReadByIndex, index & 0xFF, value, max_len);
}

bool PLCDevice::writeVariable(uint16_t index, const void* value, size_t len) {
    return writeSDO(VariableWriteByIndex, index & 0xFF, value, len);
}

bool PLCDevice::readVariableByName(const std::string& name, void* value, size_t max_len) {
    uint16_t index = findVariableByName(name);
    if (index == 0xFFFF) return false;
    return readVariable(index, value, max_len);
}

bool PLCDevice::writeVariableByName(const std::string& name, const void* value, size_t len) {
    uint16_t index = findVariableByName(name);
    if (index == 0xFFFF) return false;
    return writeVariable(index, value, len);
}

const VariableInfo* PLCDevice::getVariableInfo(uint16_t index) const {
    for (const auto& var : variables_) {
        if (var.index == index) return &var;
    }
    return nullptr;
}

const VariableInfo* PLCDevice::getVariableInfo(const std::string& name) const {
    auto it = variable_name_map_.find(name);
    if (it != variable_name_map_.end()) {
        return getVariableInfo(it->second);
    }
    return nullptr;
}

bool PLCDevice::loadSymbolTable() {
    variables_.clear();
    variable_name_map_.clear();
    
    uint16_t count = 0;
    if (!readSDO(VariableCount, 0, &count, 2)) return false;
    
    for (uint16_t i = 0; i < count && i < 256; i++) {
        VariableInfo info;
        info.index = i;
        
        // Read variable info
        char name_buf[64] = {0};
        if (readSDO(VariableName, i, name_buf, sizeof(name_buf) - 1)) {
            info.name = name_buf;
        }
        
        readSDO(VariableType, i, &info.data_type, 1);
        readSDO(VariableAddress, i, &info.address, 4);
        readSDO(VariableSize, i, &info.size, 2);
        
        variables_.push_back(info);
        if (!info.name.empty()) {
            variable_name_map_[info.name] = info.index;
        }
    }
    
    return true;
}

uint16_t PLCDevice::findVariableByName(const std::string& name) const {
    auto it = variable_name_map_.find(name);
    return (it != variable_name_map_.end()) ? it->second : 0xFFFF;
}

// Typed variable access implementations
bool PLCDevice::readBool(uint16_t index, bool& value) {
    uint8_t v = 0;
    if (!readVariable(index, &v, 1)) return false;
    value = (v != 0);
    return true;
}

bool PLCDevice::writeBool(uint16_t index, bool value) {
    uint8_t v = value ? 1 : 0;
    return writeVariable(index, &v, 1);
}

bool PLCDevice::readInt16(uint16_t index, int16_t& value) {
    return readVariable(index, &value, 2);
}

bool PLCDevice::writeInt16(uint16_t index, int16_t value) {
    return writeVariable(index, &value, 2);
}

bool PLCDevice::readInt32(uint16_t index, int32_t& value) {
    return readVariable(index, &value, 4);
}

bool PLCDevice::writeInt32(uint16_t index, int32_t value) {
    return writeVariable(index, &value, 4);
}

bool PLCDevice::readReal(uint16_t index, float& value) {
    return readVariable(index, &value, 4);
}

bool PLCDevice::writeReal(uint16_t index, float value) {
    return writeVariable(index, &value, 4);
}

// ============================================================================
// Memory Area Access
// ============================================================================

bool PLCDevice::readInputByte(uint16_t offset, uint8_t& value) {
    return readSDO(InputByte, offset & 0xFF, &value, 1);
}

bool PLCDevice::readInputWord(uint16_t offset, uint16_t& value) {
    return readSDO(InputWord, offset & 0xFF, &value, 2);
}

bool PLCDevice::readInputDWord(uint16_t offset, uint32_t& value) {
    return readSDO(InputDWord, offset & 0xFF, &value, 4);
}

bool PLCDevice::readOutputByte(uint16_t offset, uint8_t& value) {
    return readSDO(OutputByte, offset & 0xFF, &value, 1);
}

bool PLCDevice::readOutputWord(uint16_t offset, uint16_t& value) {
    return readSDO(OutputWord, offset & 0xFF, &value, 2);
}

bool PLCDevice::readOutputDWord(uint16_t offset, uint32_t& value) {
    return readSDO(OutputDWord, offset & 0xFF, &value, 4);
}

bool PLCDevice::writeOutputByte(uint16_t offset, uint8_t value) {
    return writeSDO(OutputByte, offset & 0xFF, &value, 1);
}

bool PLCDevice::writeOutputWord(uint16_t offset, uint16_t value) {
    return writeSDO(OutputWord, offset & 0xFF, &value, 2);
}

bool PLCDevice::writeOutputDWord(uint16_t offset, uint32_t value) {
    return writeSDO(OutputDWord, offset & 0xFF, &value, 4);
}

bool PLCDevice::readMemoryByte(uint16_t offset, uint8_t& value) {
    return readSDO(MemoryByte, offset & 0xFF, &value, 1);
}

bool PLCDevice::readMemoryWord(uint16_t offset, uint16_t& value) {
    return readSDO(MemoryWord, offset & 0xFF, &value, 2);
}

bool PLCDevice::readMemoryDWord(uint16_t offset, uint32_t& value) {
    return readSDO(MemoryDWord, offset & 0xFF, &value, 4);
}

bool PLCDevice::writeMemoryByte(uint16_t offset, uint8_t value) {
    return writeSDO(MemoryByte, offset & 0xFF, &value, 1);
}

bool PLCDevice::writeMemoryWord(uint16_t offset, uint16_t value) {
    return writeSDO(MemoryWord, offset & 0xFF, &value, 2);
}

bool PLCDevice::writeMemoryDWord(uint16_t offset, uint32_t value) {
    return writeSDO(MemoryDWord, offset & 0xFF, &value, 4);
}

// ============================================================================
// Exception Handling
// ============================================================================

bool PLCDevice::clearException() {
    current_exception_ = ExceptionInfo();
    uint8_t cmd = ProgramCommands::Reset;
    return writeSDO(ExceptionStatus, 0, &cmd, 1);
}

// ============================================================================
// Debug Interface
// ============================================================================

bool PLCDevice::enableDebug() {
    if (!capabilities_.supports_debug) return false;
    
    uint8_t enable = 1;
    if (writeSDO(DebugControl, DebugSub::Enable, &enable, 1)) {
        debug_enabled_ = true;
        return true;
    }
    return false;
}

bool PLCDevice::disableDebug() {
    uint8_t enable = 0;
    if (writeSDO(DebugControl, DebugSub::Enable, &enable, 1)) {
        debug_enabled_ = false;
        return true;
    }
    return false;
}

bool PLCDevice::setBreakpoint(uint32_t address, const std::string& condition) {
    if (breakpoints_.size() >= capabilities_.max_breakpoints) return false;
    
    Breakpoint bp;
    bp.id = static_cast<uint8_t>(breakpoints_.size() + 1);
    bp.address = address;
    bp.condition = condition;
    bp.enabled = true;
    
    bool result = writeSDO(BreakpointAddress, bp.id, &address, 4);
    breakpoints_.push_back(bp);
    
    return result;
}

bool PLCDevice::clearBreakpoint(uint8_t breakpoint_id) {
    for (auto it = breakpoints_.begin(); it != breakpoints_.end(); ++it) {
        if (it->id == breakpoint_id) {
            breakpoints_.erase(it);
            
            uint8_t cmd = DebugCommands::ClearBreakpoint;
            return writeSDO(DebugControl, breakpoint_id, &cmd, 1);
        }
    }
    return false;
}

bool PLCDevice::clearAllBreakpoints() {
    breakpoints_.clear();
    uint8_t cmd = DebugCommands::ClearAll;
    return writeSDO(DebugControl, 0, &cmd, 1);
}

bool PLCDevice::stepInto() {
    uint8_t cmd = StepCommands::StepInto;
    return writeSDO(SingleStepControl, 0, &cmd, 1);
}

bool PLCDevice::stepOver() {
    uint8_t cmd = StepCommands::StepOver;
    return writeSDO(SingleStepControl, 0, &cmd, 1);
}

bool PLCDevice::stepOut() {
    uint8_t cmd = StepCommands::StepOut;
    return writeSDO(SingleStepControl, 0, &cmd, 1);
}

bool PLCDevice::runToCursor(uint32_t address) {
    if (!writeSDO(BreakpointAddress, 0, &address, 4)) return false;
    uint8_t cmd = StepCommands::RunToCursor;
    return writeSDO(SingleStepControl, 0, &cmd, 1);
}

bool PLCDevice::addWatch(uint16_t var_index) {
    if (watch_variables_.size() >= capabilities_.max_watch_variables) return false;
    watch_variables_.push_back(var_index);
    return true;
}

bool PLCDevice::removeWatch(uint16_t var_index) {
    auto it = std::find(watch_variables_.begin(), watch_variables_.end(), var_index);
    if (it != watch_variables_.end()) {
        watch_variables_.erase(it);
        return true;
    }
    return false;
}

// ============================================================================
// File Transfer
// ============================================================================

bool PLCDevice::downloadProgram(const uint8_t* data, size_t len, const std::string& filename) {
    if (!capabilities_.supports_file_transfer) return false;
    
    file_transfer_.state = FileTransferState::Downloading;
    file_transfer_.filename = filename;
    file_transfer_.total_size = len;
    file_transfer_.transferred = 0;
    
    // Start download
    uint8_t cmd = FileCommands::StartDownload;
    if (!writeSDO(FileTransferControl, 0, &cmd, 1)) {
        file_transfer_.state = FileTransferState::Error;
        return false;
    }
    
    // Write filename
    writeSDO(FileName, 0, filename.c_str(), filename.length() + 1);
    
    // Write size
    uint32_t size = len;
    writeSDO(FileSize, 0, &size, 4);
    
    // Transfer in blocks
    const size_t BLOCK_SIZE = 256;
    for (size_t offset = 0; offset < len; offset += BLOCK_SIZE) {
        size_t block_len = std::min(BLOCK_SIZE, len - offset);
        
        if (!writeSDO(FileData, (offset / BLOCK_SIZE) & 0xFF, data + offset, block_len)) {
            file_transfer_.state = FileTransferState::Error;
            return false;
        }
        
        file_transfer_.transferred = offset + block_len;
        file_transfer_.progress_percent = static_cast<uint8_t>((file_transfer_.transferred * 100) / len);
    }
    
    // End transfer
    cmd = FileCommands::EndTransfer;
    writeSDO(FileTransferControl, 0, &cmd, 1);
    
    file_transfer_.state = FileTransferState::Complete;
    return true;
}

bool PLCDevice::uploadProgram(std::vector<uint8_t>& data) {
    if (!capabilities_.supports_file_transfer) return false;
    
    file_transfer_.state = FileTransferState::Uploading;
    
    // Start upload
    uint8_t cmd = FileCommands::StartUpload;
    if (!writeSDO(FileTransferControl, 0, &cmd, 1)) {
        file_transfer_.state = FileTransferState::Error;
        return false;
    }
    
    // Read size
    uint32_t size = 0;
    if (!readSDO(FileSize, 0, &size, 4) || size == 0) {
        file_transfer_.state = FileTransferState::Error;
        return false;
    }
    
    file_transfer_.total_size = size;
    data.resize(size);
    
    // Read in blocks
    const size_t BLOCK_SIZE = 256;
    for (size_t offset = 0; offset < size; offset += BLOCK_SIZE) {
        size_t block_len = std::min((size_t)BLOCK_SIZE, (size_t)(size - offset));
        
        if (!readSDO(FileData, (offset / BLOCK_SIZE) & 0xFF, data.data() + offset, block_len)) {
            file_transfer_.state = FileTransferState::Error;
            return false;
        }
        
        file_transfer_.transferred = offset + block_len;
        file_transfer_.progress_percent = static_cast<uint8_t>((file_transfer_.transferred * 100) / size);
    }
    
    // End transfer
    cmd = FileCommands::EndTransfer;
    writeSDO(FileTransferControl, 0, &cmd, 1);
    
    file_transfer_.state = FileTransferState::Complete;
    return true;
}

bool PLCDevice::deleteFile(const std::string& filename) {
    writeSDO(FileName, 0, filename.c_str(), filename.length() + 1);
    uint8_t cmd = FileCommands::Delete;
    return writeSDO(FileTransferControl, 0, &cmd, 1);
}

bool PLCDevice::abortFileTransfer() {
    uint8_t cmd = FileCommands::Abort;
    file_transfer_.state = FileTransferState::Idle;
    return writeSDO(FileTransferControl, 0, &cmd, 1);
}

// ============================================================================
// Timing and Resources
// ============================================================================

bool PLCDevice::setWatchdogTime(uint32_t time_ms) {
    return writeSDO(WatchdogTime, 0, &time_ms, 4);
}

// ============================================================================
// Callbacks
// ============================================================================

void PLCDevice::setProgramStateCallback(ProgramStateCallback callback) {
    program_state_callback_ = callback;
}

void PLCDevice::setTaskStateCallback(TaskStateCallback callback) {
    task_state_callback_ = callback;
}

void PLCDevice::setExceptionCallback(ExceptionCallback callback) {
    exception_callback_ = callback;
}

void PLCDevice::setBreakpointCallback(BreakpointCallback callback) {
    breakpoint_callback_ = callback;
}

void PLCDevice::setVariableCallback(VariableCallback callback) {
    variable_callback_ = callback;
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string PLCDevice::getDiagnostics() const {
    std::string diag;
    diag += "CiA 405 PLC Device\n";
    diag += "  Slave: " + std::to_string(slave_addr_) + "\n";
    diag += "  Program State: " + std::string(getProgramStateName(program_info_.state)) + "\n";
    diag += "  Tasks: " + std::to_string(tasks_.size()) + "\n";
    
    for (size_t i = 0; i < tasks_.size(); i++) {
        diag += "    Task " + std::to_string(i + 1) + ": " + 
                std::string(getTaskStateName(tasks_[i].state)) + "\n";
    }
    
    if (current_exception_.hasException()) {
        diag += "  Exception: " + std::string(getExceptionName(current_exception_.code)) + "\n";
    }
    
    diag += "  CPU Load: " + std::to_string(resources_.getCPULoadPercent()) + "%\n";
    diag += "  Cycle Time: " + std::to_string(resources_.cycle_time_us) + " us\n";
    
    return diag;
}

// ============================================================================
// Internal SDO Methods
// ============================================================================

bool PLCDevice::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    return m_sdo.readSync(slave_addr_, index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

bool PLCDevice::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    return m_sdo.writeSync(slave_addr_, index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

} // namespace CiA405
