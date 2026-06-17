/**
 * @file ETG5000ModularDevice.cpp
 * @brief ETG.5000.1 Modular Device Controller Implementation
 */

#include "etg5000/ETG5000ModularDevice.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <cstring>
#include <algorithm>

namespace ETG5000 {

// ============================================================================
// Constructor / Destructor
// ============================================================================

ModularDevice::ModularDevice(EtherCAT::CoE::CoEManager& coe)
    : m_coe(coe)
    , initialized_(false)
    , state_{}
    , prev_statusword_(0)
    , controlword_(0)
    , auto_config_enabled_(false)
    , module_event_callback_(nullptr)
    , config_change_callback_(nullptr)
    , diagnostic_callback_(nullptr)
    , error_callback_(nullptr)
{
}

ModularDevice::~ModularDevice() = default;

// ============================================================================
// Initialization
// ============================================================================

bool ModularDevice::initialize()
{
    if (!scanModules()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

// ============================================================================
// Configuration
// ============================================================================

bool ModularDevice::setExpectedConfiguration(const std::vector<ModuleDescriptor>& descriptors)
{
    expected_config_ = descriptors;
    return validateConfiguration();
}

bool ModularDevice::enableAutoConfiguration(bool enable)
{
    auto_config_enabled_ = enable;
    uint8_t auto_cfg = enable ? 1 : 0;
    return writeSDO(ModuleConfig::AutoConfiguration, 0, &auto_cfg, 1);
}

bool ModularDevice::validateConfiguration()
{
    if (!initialized_) return false;
    
    state_.config.missing_slots.clear();
    state_.config.extra_slots.clear();
    state_.config.mismatched_slots.clear();
    state_.config.config_mismatch = false;
    state_.config.all_modules_ok = true;
    
    // Check each expected module
    for (const auto& expected : expected_config_) {
        const ModuleState* actual = getModuleState(expected.slot);
        
        if (!actual || !actual->isPresent()) {
            if (!expected.optional) {
                state_.config.missing_slots.push_back(expected.slot);
                state_.config.config_mismatch = true;
                state_.config.all_modules_ok = false;
            }
            continue;
        }
        
        // Check module type
        if (expected.expected_type != ModuleType::Unknown &&
            actual->module_type != expected.expected_type) {
            if (!expected.allow_compatible) {
                state_.config.mismatched_slots.push_back(expected.slot);
                state_.config.config_mismatch = true;
            }
        }
        
        if (actual->hasError()) {
            state_.config.all_modules_ok = false;
        }
    }
    
    // Check for extra modules
    for (const auto& module : state_.modules) {
        if (!module.isPresent()) continue;
        
        bool found = false;
        for (const auto& expected : expected_config_) {
            if (expected.slot == module.slot) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            state_.config.extra_slots.push_back(module.slot);
        }
    }
    
    // Update configuration state
    if (state_.config.config_mismatch) {
        if (!state_.config.missing_slots.empty()) {
            state_.config.config_state = ConfigState::ModuleMissing;
        } else {
            state_.config.config_state = ConfigState::ConfigMismatch;
        }
    } else if (!state_.config.extra_slots.empty()) {
        state_.config.config_state = ConfigState::ExtraModules;
    } else {
        state_.config.config_state = ConfigState::ConfigurationValid;
    }
    
    return !state_.config.config_mismatch;
}

bool ModularDevice::acceptConfiguration()
{
    controlword_ |= ControlwordBits::AcceptConfig;
    
    // Write controlword
    if (!writeSDO(0x6000, 0, &controlword_, 2)) {
        controlword_ &= ~ControlwordBits::AcceptConfig;
        return false;
    }
    
    // Clear flag
    controlword_ &= ~ControlwordBits::AcceptConfig;
    
    state_.config.config_state = ConfigState::ConfigurationValid;
    
    if (config_change_callback_) {
        config_change_callback_(state_.config);
    }
    
    return true;
}

bool ModularDevice::saveConfiguration()
{
    controlword_ |= ControlwordBits::SaveConfig;
    bool ok = writeSDO(0x6000, 0, &controlword_, 2);
    controlword_ &= ~ControlwordBits::SaveConfig;
    return ok;
}

// ============================================================================
// Module Enumeration
// ============================================================================

bool ModularDevice::scanModules()
{
    if (!readModuleList()) {
        return false;
    }
    
    // Initialize tracking vectors
    prev_module_status_.resize(state_.modules.size(), ModuleStatus::NotPresent);
    input_offsets_.resize(state_.modules.size(), 0);
    output_offsets_.resize(state_.modules.size(), 0);
    input_sizes_.resize(state_.modules.size(), 0);
    output_sizes_.resize(state_.modules.size(), 0);
    
    // Read PDO info for each module
    for (size_t i = 0; i < state_.modules.size(); ++i) {
        uint8_t slot = state_.modules[i].slot;
        
        uint16_t input_off = 0, output_off = 0;
        uint16_t input_sz = 0, output_sz = 0;
        
        readSDO(ModulePDO::InputOffset, slot, &input_off, 2);
        readSDO(ModulePDO::OutputOffset, slot, &output_off, 2);
        readSDO(ModulePDO::PDOSize, slot, &input_sz, 2);  // subindex for input size
        readSDO(ModulePDO::PDOSize, slot + 64, &output_sz, 2);  // offset for output
        
        input_offsets_[i] = input_off;
        output_offsets_[i] = output_off;
        input_sizes_[i] = input_sz;
        output_sizes_[i] = output_sz;
    }
    
    return true;
}

bool ModularDevice::readModuleList()
{
    // Read number of modules
    uint8_t module_count = 0;
    if (!readSDO(ModuleConfig::DetectedModuleCount, 0, &module_count, 1)) {
        return false;
    }
    
    state_.config.detected_modules = module_count;
    state_.modules.clear();
    state_.modules.reserve(module_count);
    
    // Read each module's info
    for (uint8_t i = 0; i < module_count; ++i) {
        ModuleState mod;
        mod.slot = i;
        
        // Read module type
        readSDO(ModuleConfig::ModuleTypeList, i + 1, &mod.module_type, 2);
        
        // Read module status
        readSDO(ModuleConfig::ModuleStatusList, i + 1, &mod.status, 1);
        
        // Read diagnostics
        readSDO(ModuleDiag::DiagnosticStatus, i + 1, &mod.diag_status, 2);
        readSDO(ModuleDiag::LastError, i + 1, &mod.last_error, 2);
        readSDO(ModuleDiag::Temperature, i + 1, &mod.temperature, 2);
        readSDO(ModuleDiag::SupplyVoltage, i + 1, &mod.supply_voltage, 2);
        readSDO(ModuleDiag::ErrorCount, i + 1, &mod.error_count, 4);
        
        state_.modules.push_back(mod);
    }
    
    return true;
}

bool ModularDevice::getModuleInfo(uint8_t slot, SlotInfo& info)
{
    if (slot >= state_.modules.size()) return false;
    
    info.slot_number = slot;
    
    // Read identity
    readSDO(ModuleIdent::VendorID, slot + 1, &info.vendor_id, 4);
    readSDO(ModuleIdent::ProductCode, slot + 1, &info.product_code, 4);
    readSDO(ModuleIdent::RevisionNumber, slot + 1, &info.revision, 4);
    readSDO(ModuleIdent::SerialNumber, slot + 1, &info.serial, 4);
    readSDO(ModuleIdent::Capabilities, slot + 1, &info.capabilities, 4);
    
    // Copy from module state
    const auto& mod = state_.modules[slot];
    info.module_type = mod.module_type;
    info.status = mod.status;
    info.diag_status = mod.diag_status;
    info.temperature = mod.temperature;
    info.supply_voltage = mod.supply_voltage;
    info.error_count = mod.error_count;
    
    // Read operating hours
    readSDO(ModuleDiag::OperatingHours, slot + 1, &info.operating_hours, 4);
    
    // PDO sizes
    if (slot < input_sizes_.size()) {
        info.input_size = input_sizes_[slot];
        info.output_size = output_sizes_[slot];
        info.input_offset = input_offsets_[slot];
        info.output_offset = output_offsets_[slot];
    }
    
    return true;
}

const ModuleState* ModularDevice::getModuleState(uint8_t slot) const
{
    for (const auto& mod : state_.modules) {
        if (mod.slot == slot) {
            return &mod;
        }
    }
    return nullptr;
}

bool ModularDevice::isModulePresent(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod && mod->isPresent();
}

bool ModularDevice::isModuleOperational(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod && mod->isOperational();
}

// ============================================================================
// Module Control
// ============================================================================

bool ModularDevice::enableModule(uint8_t slot, bool enable)
{
    uint8_t cmd = enable ? 1 : 0;
    return writeSDO(ModuleParam::ParameterLock, slot + 1, &cmd, 1);
}

bool ModularDevice::enableAllModules(bool enable)
{
    if (enable) {
        controlword_ |= ControlwordBits::EnableAll;
    } else {
        controlword_ &= ~ControlwordBits::EnableAll;
    }
    return writeSDO(0x6000, 0, &controlword_, 2);
}

bool ModularDevice::resetModuleErrors(uint8_t slot)
{
    uint8_t reset = 1;
    return writeSDO(ModuleDiag::LastError, slot + 1, &reset, 1);
}

bool ModularDevice::resetAllErrors()
{
    controlword_ |= ControlwordBits::ResetErrors;
    bool ok = writeSDO(0x6000, 0, &controlword_, 2);
    controlword_ &= ~ControlwordBits::ResetErrors;
    return ok;
}

bool ModularDevice::resetModuleToDefaults(uint8_t slot)
{
    uint8_t reset = 1;
    return writeSDO(ModuleParam::FactoryDefaults, slot + 1, &reset, 1);
}

// ============================================================================
// Process Data Access
// ============================================================================

uint16_t ModularDevice::getModuleInputOffset(uint8_t slot) const
{
    if (slot < input_offsets_.size()) {
        return input_offsets_[slot];
    }
    return 0;
}

uint16_t ModularDevice::getModuleOutputOffset(uint8_t slot) const
{
    if (slot < output_offsets_.size()) {
        return output_offsets_[slot];
    }
    return 0;
}

uint16_t ModularDevice::getModuleInputSize(uint8_t slot) const
{
    if (slot < input_sizes_.size()) {
        return input_sizes_[slot];
    }
    return 0;
}

uint16_t ModularDevice::getModuleOutputSize(uint8_t slot) const
{
    if (slot < output_sizes_.size()) {
        return output_sizes_[slot];
    }
    return 0;
}

bool ModularDevice::readModuleInput(uint8_t slot, void* data, size_t len)
{
    if (!data || slot >= input_offsets_.size()) return false;
    
    // Read from module's input PDO area
    return readSDO(ModulePDO::InputPDOMapping, slot + 1, data, len);
}

bool ModularDevice::writeModuleOutput(uint8_t slot, const void* data, size_t len)
{
    if (!data || slot >= output_offsets_.size()) return false;
    
    // Write to module's output PDO area
    return writeSDO(ModulePDO::OutputPDOMapping, slot + 1, data, len);
}

// ============================================================================
// Cyclic Update
// ============================================================================

void ModularDevice::processTxPDO(const uint8_t* data, size_t len)
{
    if (!initialized_ || !data) return;
    
    if (len < sizeof(ModularInputPDO)) return;
    
    const auto* pdo = reinterpret_cast<const ModularInputPDO*>(data);
    
    state_.statusword = pdo->statusword;
    state_.config.detected_modules = pdo->module_count;
    state_.config.config_state = pdo->config_state;
    
    // Update module status from bitmap
    for (size_t i = 0; i < state_.modules.size() && i < 64; ++i) {
        uint8_t word_idx = i / 16;
        uint8_t bit_idx = i % 16;
        
        bool has_diag = (pdo->diag_status_bitmap[word_idx] >> bit_idx) & 1;
        if (has_diag) {
            state_.modules[i].diag_status |= DiagStatus::Warning;
        }
    }
    
    updateModuleStates();
    checkHotSwapEvents();
}

size_t ModularDevice::prepareRxPDO(uint8_t* data, size_t max_len)
{
    if (max_len < sizeof(ModularOutputPDO)) return 0;
    
    auto* pdo = reinterpret_cast<ModularOutputPDO*>(data);
    pdo->controlword = controlword_;
    memset(pdo->reserved, 0, sizeof(pdo->reserved));
    
    return sizeof(ModularOutputPDO);
}

void ModularDevice::update()
{
    updateModuleStates();
    checkConfigurationState();
    prev_statusword_ = state_.statusword;
}

void ModularDevice::updateModuleStates()
{
    for (size_t i = 0; i < state_.modules.size(); ++i) {
        uint8_t new_status = 0;
        readSDO(ModuleConfig::ModuleStatusList, i + 1, &new_status, 1);
        
        uint8_t old_status = state_.modules[i].status;
        state_.modules[i].status = new_status;
        
        // Check for status changes
        if (new_status != old_status) {
            if (old_status == ModuleStatus::NotPresent && 
                new_status != ModuleStatus::NotPresent) {
                notifyModuleEvent(i, ModuleEvent::Inserted);
            } else if (old_status != ModuleStatus::NotPresent && 
                       new_status == ModuleStatus::NotPresent) {
                notifyModuleEvent(i, ModuleEvent::Removed);
            } else if (new_status == ModuleStatus::Operational) {
                notifyModuleEvent(i, ModuleEvent::Operational);
            } else if (new_status == ModuleStatus::Error) {
                notifyModuleEvent(i, ModuleEvent::Error);
                if (error_callback_) {
                    error_callback_(i, state_.modules[i].last_error);
                }
            }
        }
        
        // Update diagnostic status
        uint16_t diag = 0;
        readSDO(ModuleDiag::DiagnosticStatus, i + 1, &diag, 2);
        if (diag != state_.modules[i].diag_status) {
            state_.modules[i].diag_status = diag;
            notifyModuleEvent(i, ModuleEvent::DiagUpdated);
            if (diagnostic_callback_) {
                diagnostic_callback_(i, diag);
            }
        }
    }
}

void ModularDevice::checkConfigurationState()
{
    uint8_t config_state = 0;
    readSDO(ModuleConfig::ConfigurationState, 0, &config_state, 1);
    
    if (config_state != state_.config.config_state) {
        state_.config.config_state = config_state;
        
        validateConfiguration();
        
        if (config_change_callback_) {
            config_change_callback_(state_.config);
        }
    }
}

void ModularDevice::checkHotSwapEvents()
{
    if (state_.statusword & StatuswordBits::HotSwapEvent) {
        // Re-scan modules
        scanModules();
        validateConfiguration();
        
        if (config_change_callback_) {
            config_change_callback_(state_.config);
        }
    }
}

void ModularDevice::notifyModuleEvent(uint8_t slot, uint8_t event_type)
{
    if (module_event_callback_) {
        module_event_callback_(slot, event_type);
    }
}

// ============================================================================
// Diagnostics
// ============================================================================

uint16_t ModularDevice::getModuleDiagStatus(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod ? mod->diag_status : 0;
}

uint16_t ModularDevice::getModuleLastError(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod ? mod->last_error : 0;
}

float ModularDevice::getModuleTemperature(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod ? mod->getTemperature_C() : 0.0f;
}

float ModularDevice::getModuleSupplyVoltage(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod ? mod->getSupplyVoltage_V() : 0.0f;
}

uint32_t ModularDevice::getModuleErrorCount(uint8_t slot) const
{
    const auto* mod = getModuleState(slot);
    return mod ? mod->error_count : 0;
}

uint32_t ModularDevice::getModuleOperatingHours(uint8_t slot) const
{
    uint32_t hours = 0;
    readSDO(ModuleDiag::OperatingHours, slot + 1, &hours, 4);
    return hours;
}

std::string ModularDevice::getDiagnostics() const
{
    std::string diag;
    
    diag += "Modular Device Diagnostics:\n";
    diag += "  Status: 0x" + std::to_string(state_.statusword) + "\n";
    diag += "  Detected Modules: " + std::to_string(state_.config.detected_modules) + "\n";
    diag += "  Config State: " + std::to_string(state_.config.config_state) + "\n";
    diag += "  Config Valid: " + std::string(state_.config.config_state == ConfigState::ConfigurationValid ? "Yes" : "No") + "\n";
    
    if (!state_.config.missing_slots.empty()) {
        diag += "  Missing Slots: ";
        for (auto slot : state_.config.missing_slots) {
            diag += std::to_string(slot) + " ";
        }
        diag += "\n";
    }
    
    if (!state_.config.mismatched_slots.empty()) {
        diag += "  Mismatched Slots: ";
        for (auto slot : state_.config.mismatched_slots) {
            diag += std::to_string(slot) + " ";
        }
        diag += "\n";
    }
    
    diag += "\nModule Details:\n";
    for (const auto& mod : state_.modules) {
        diag += "  Slot " + std::to_string(mod.slot) + ":\n";
        diag += "    Type: 0x" + std::to_string(mod.module_type) + "\n";
        diag += "    Status: " + std::to_string(mod.status) + "\n";
        diag += "    Present: " + std::string(mod.isPresent() ? "Yes" : "No") + "\n";
        diag += "    Operational: " + std::string(mod.isOperational() ? "Yes" : "No") + "\n";
        if (mod.hasError()) {
            diag += "    ERROR: 0x" + std::to_string(mod.last_error) + "\n";
        }
        diag += "    Temperature: " + std::to_string(mod.getTemperature_C()) + " C\n";
        diag += "    Supply: " + std::to_string(mod.getSupplyVoltage_V()) + " V\n";
    }
    
    return diag;
}

// ============================================================================
// Callbacks
// ============================================================================

void ModularDevice::setModuleEventCallback(ModuleEventCallback callback)
{
    module_event_callback_ = std::move(callback);
}

void ModularDevice::setConfigChangeCallback(ConfigChangeCallback callback)
{
    config_change_callback_ = std::move(callback);
}

void ModularDevice::setDiagnosticCallback(DiagnosticCallback callback)
{
    diagnostic_callback_ = std::move(callback);
}

void ModularDevice::setErrorCallback(ErrorCallback callback)
{
    error_callback_ = std::move(callback);
}

// ============================================================================
// SDO Access (Private)
// ============================================================================

bool ModularDevice::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) const
{
    size_t actual_size;
    return m_coe.readSync(index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs, &actual_size);
}

bool ModularDevice::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) const
{
    auto result = m_coe.writeSync(index, subindex, data, len, {.timeout_ms = EtherCAT::SDO::kDefaultSDOTimeoutMs});
    return result.has_value();
}

} // namespace ETG5000
