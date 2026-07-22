/**
 * @file CiA401IO.cpp
 * @brief CiA 401 I/O Module Controller Implementation
 */

#include "tether/profiles/cia401/CiA401IO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/CoEManager.hpp"

static const char* TAG = "CiA401";
#define LOG_I(fmt, ...) TETHER_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOG_W(fmt, ...) TETHER_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOG_E(fmt, ...) TETHER_LOGE(TAG, fmt, ##__VA_ARGS__)

#include <cstring>
#include <algorithm>

namespace CiA401 {

// ============================================================================
// Construction and Initialization
// ============================================================================

IOModule::IOModule(EtherCAT::CoE::CoEManager& coe)
    : m_coe(coe)
    , initialized_(false)
    , capabilities_()
    , current_mapping_(PDOMappingPreset::Minimal)
    , has_fault_(false)
    , last_error_(0)
{
}

IOModule::~IOModule() {
}

bool IOModule::initialize() {
    LOG_I("Initializing CiA 401 I/O module at slave %u", m_coe.slaveIndex());
    
    // Verify device type
    uint32_t device_type = 0;
    if (!readSDO(0x1000, 0, &device_type, sizeof(device_type))) {
        LOG_E("Failed to read device type");
        return false;
    }
    
    // Check profile number (lower 16 bits)
    uint16_t profile = device_type & 0xFFFF;
    if (profile != PROFILE_NUMBER) {
        LOG_W("Device profile %u is not CiA 401 (%u)", profile, PROFILE_NUMBER);
        // Continue anyway - might be compatible
    }
    
    // Detect module capabilities
    if (!detectCapabilities()) {
        LOG_E("Failed to detect module capabilities");
        return false;
    }
    
    LOG_I("Module type: %s", getModuleTypeName(capabilities_.module_type));
    LOG_I("Digital inputs: %u, outputs: %u",
          capabilities_.getTotalDigitalInputs(),
          capabilities_.getTotalDigitalOutputs());
    LOG_I("Analog inputs: %u, outputs: %u",
          capabilities_.getTotalAnalogInputs(),
          capabilities_.getTotalAnalogOutputs());
    
    // Initialize state vectors
    size_t di_blocks = std::max({ (size_t)capabilities_.digital_input_8bit_blocks,
                                  (size_t)capabilities_.digital_input_16bit_blocks * 2,
                                  (size_t)capabilities_.digital_input_32bit_blocks * 4 });
size_t do_blocks = std::max({ (size_t)capabilities_.digital_output_8bit_blocks,
                                  (size_t)capabilities_.digital_output_16bit_blocks * 2,
                                  (size_t)capabilities_.digital_output_32bit_blocks * 4 });
    
    digital_input_state_.resize(di_blocks, 0);
    digital_output_state_.resize(do_blocks, 0);
    prev_digital_input_state_.resize(di_blocks, 0);
    
    analog_input_state_.resize(capabilities_.getTotalAnalogInputs(), 0);
    analog_output_state_.resize(capabilities_.getTotalAnalogOutputs(), 0);
    prev_analog_input_state_.resize(capabilities_.getTotalAnalogInputs(), 0);
    
    ai_configs_.resize(capabilities_.getTotalAnalogInputs());
    ao_configs_.resize(capabilities_.getTotalAnalogOutputs());
    
    if (capabilities_.has_counters) {
        counter_values_.resize(capabilities_.counter_channels, 0);
    }
    
    // Apply default configuration
    if (!applyDefaultConfiguration()) {
        LOG_W("Failed to apply default configuration");
    }
    
    initialized_ = true;
    
    if (event_callback_) {
        event_callback_(IOEvent::Initialized, m_coe.slaveIndex(), 0, 0);
    }
    
    return true;
}

bool IOModule::detectCapabilities() {
    // Check for 8-bit digital inputs
    uint8_t num_blocks = 0;
    if (readSDO(DigitalInput8, 0, &num_blocks, 1)) {
        capabilities_.digital_input_8bit_blocks = num_blocks;
    }
    
    // Check for 16-bit digital inputs
    if (readSDO(DigitalInput16, 0, &num_blocks, 1)) {
        capabilities_.digital_input_16bit_blocks = num_blocks;
    }
    
    // Check for 32-bit digital inputs
    if (readSDO(DigitalInput32, 0, &num_blocks, 1)) {
        capabilities_.digital_input_32bit_blocks = num_blocks;
    }
    
    // Check for 8-bit digital outputs
    if (readSDO(DigitalOutput8, 0, &num_blocks, 1)) {
        capabilities_.digital_output_8bit_blocks = num_blocks;
    }
    
    // Check for 16-bit digital outputs
    if (readSDO(DigitalOutput16, 0, &num_blocks, 1)) {
        capabilities_.digital_output_16bit_blocks = num_blocks;
    }
    
    // Check for 32-bit digital outputs
    if (readSDO(DigitalOutput32, 0, &num_blocks, 1)) {
        capabilities_.digital_output_32bit_blocks = num_blocks;
    }
    
    // Check for 16-bit analog inputs
    if (readSDO(AnalogInput16, 0, &num_blocks, 1)) {
        capabilities_.analog_input_16bit_channels = num_blocks;
    }
    
    // Check for 32-bit analog inputs
    if (readSDO(AnalogInput32, 0, &num_blocks, 1)) {
        capabilities_.analog_input_32bit_channels = num_blocks;
    }
    
    // Check for 16-bit analog outputs
    if (readSDO(AnalogOutput16, 0, &num_blocks, 1)) {
        capabilities_.analog_output_16bit_channels = num_blocks;
    }
    
    // Check for 32-bit analog outputs
    if (readSDO(AnalogOutput32, 0, &num_blocks, 1)) {
        capabilities_.analog_output_32bit_channels = num_blocks;
    }
    
    // Check for counters
    uint32_t counter_val = 0;
    if (readSDO(CounterValue, 0, &num_blocks, 1)) {
        capabilities_.has_counters = true;
        capabilities_.counter_channels = num_blocks;
    }
    
    // Check for frequency inputs
    if (readSDO(FrequencyInput, 0, &num_blocks, 1)) {
        capabilities_.has_frequency_input = true;
        capabilities_.frequency_channels = num_blocks;
    }
    
    // Check for PWM outputs
    if (readSDO(PWMDutyCycle, 0, &num_blocks, 1)) {
        capabilities_.has_pwm_output = true;
        capabilities_.pwm_channels = num_blocks;
    }
    
    // Check for interrupt support
    uint8_t int_enable = 0;
    if (readSDO(DigitalInterruptEnable, 0, &int_enable, 1)) {
        capabilities_.supports_interrupt = true;
    }
    
    // Check for filter support
    uint8_t filter = 0;
    if (readSDO(DigitalInputFilter8, 1, &filter, 1)) {
        capabilities_.supports_filtering = true;
    }
    
    // Check for polarity support
    uint8_t polarity = 0;
    if (readSDO(DigitalInputPolarity8, 1, &polarity, 1)) {
        capabilities_.supports_polarity = true;
    }
    
    // Determine module type
    bool has_di = capabilities_.getTotalDigitalInputs() > 0;
    bool has_do = capabilities_.getTotalDigitalOutputs() > 0;
    bool has_ai = capabilities_.getTotalAnalogInputs() > 0;
    bool has_ao = capabilities_.getTotalAnalogOutputs() > 0;
    
    if (has_di && has_do && has_ai && has_ao) {
        capabilities_.module_type = ModuleType::MixedDigitalAnalog;
    } else if (has_di && has_do) {
        capabilities_.module_type = ModuleType::DigitalIO;
    } else if (has_di) {
        capabilities_.module_type = ModuleType::DigitalInputOnly;
    } else if (has_do) {
        capabilities_.module_type = ModuleType::DigitalOutputOnly;
    } else if (has_ai && has_ao) {
        capabilities_.module_type = ModuleType::AnalogIO;
    } else if (has_ai) {
        capabilities_.module_type = ModuleType::AnalogInputOnly;
    } else if (has_ao) {
        capabilities_.module_type = ModuleType::AnalogOutputOnly;
    } else if (capabilities_.has_counters) {
        capabilities_.module_type = ModuleType::Counter;
    }
    
    return true;
}

bool IOModule::applyDefaultConfiguration() {
    // Set all outputs to safe state
    for (size_t i = 0; i < digital_output_state_.size(); i++) {
        digital_output_state_[i] = 0;
        writeSDO(DigitalOutput8, i + 1, &digital_output_state_[i], 1);
    }
    
    // Set error mode to maintain last value
    for (size_t i = 1; i <= capabilities_.digital_output_8bit_blocks; i++) {
        uint8_t mode = ErrorMode::MaintainLastValue;
        writeSDO(DigitalOutputErrorMode8, i, &mode, 1);
    }
    
    for (size_t i = 1; i <= capabilities_.analog_output_16bit_channels; i++) {
        uint8_t mode = ErrorMode::MaintainLastValue;
        writeSDO(AnalogOutputErrorMode, i, &mode, 1);
    }
    
    return true;
}

// ============================================================================
// PDO Configuration
// ============================================================================

bool IOModule::applyPDOMapping(PDOMappingPreset preset) {
    bool result = false;
    
    switch (preset) {
        case PDOMappingPreset::Minimal:
            result = setupPDOMapping_Minimal();
            break;
        case PDOMappingPreset::DigitalOnly:
        case PDOMappingPreset::Digital16:
        case PDOMappingPreset::Digital32:
            result = setupPDOMapping_DigitalOnly();
            break;
        case PDOMappingPreset::AnalogOnly:
        case PDOMappingPreset::AnalogHighRes:
            result = setupPDOMapping_AnalogOnly();
            break;
        case PDOMappingPreset::Combined:
            result = setupPDOMapping_Combined();
            break;
        case PDOMappingPreset::Full:
            result = setupPDOMapping_Full();
            break;
        default:
            return false;
    }
    
    if (result) {
        current_mapping_ = preset;
    }
    
    return result;
}

bool IOModule::setupPDOMapping_Minimal() {
    // Map only first digital input/output block
    return true; // Placeholder - actual mapping depends on ESC
}

bool IOModule::setupPDOMapping_DigitalOnly() {
    // Map all digital I/O
    return true;
}

bool IOModule::setupPDOMapping_AnalogOnly() {
    // Map all analog I/O
    return true;
}

bool IOModule::setupPDOMapping_Combined() {
    // Map both digital and analog I/O
    return true;
}

bool IOModule::setupPDOMapping_Full() {
    // Map everything including counters
    return true;
}

// ============================================================================
// Update Cycle
// ============================================================================

void IOModule::processTxPDO(const uint8_t* data, size_t len) {
    if (!initialized_ || len == 0) return;
    
    size_t offset = 0;
    
    // Process based on current mapping
    switch (current_mapping_) {
        case PDOMappingPreset::DigitalOnly:
        case PDOMappingPreset::Digital16:
        case PDOMappingPreset::Digital32:
            processDigitalInputs(data, 0, len);
            break;
            
        case PDOMappingPreset::AnalogOnly:
        case PDOMappingPreset::AnalogHighRes:
            processAnalogInputs(data, 0, len);
            break;
            
        case PDOMappingPreset::Combined:
        case PDOMappingPreset::Full:
        case PDOMappingPreset::Minimal:
            processDigitalInputs(data, 0, len);
            offset = digital_input_state_.size();
            if (offset < len) {
                processAnalogInputs(data, offset, len);
            }
            break;
            
        default:
            break;
    }
    
    checkDigitalInputChanges();
    checkAnalogThresholds();
}

size_t IOModule::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!initialized_) return 0;
    
    size_t offset = 0;
    
    // Copy digital outputs
    size_t do_len = std::min(digital_output_state_.size(), max_len);
    memcpy(data, digital_output_state_.data(), do_len);
    offset += do_len;
    
    // Copy analog outputs
    size_t ao_len = std::min(analog_output_state_.size() * 2, max_len - offset);
    if (ao_len > 0) {
        memcpy(data + offset, analog_output_state_.data(), ao_len);
        offset += ao_len;
    }
    
    return offset;
}

void IOModule::update() {
    if (!initialized_) return;
    
    // Read digital inputs via SDO
    for (size_t i = 0; i < capabilities_.digital_input_8bit_blocks; i++) {
        uint8_t val = 0;
        if (readSDO(DigitalInput8, i + 1, &val, 1)) {
            digital_input_state_[i] = val;
        }
    }
    
    // Read analog inputs via SDO
    for (size_t i = 0; i < capabilities_.analog_input_16bit_channels; i++) {
        int16_t val = 0;
        if (readSDO(AnalogInput16, i + 1, &val, 2)) {
            analog_input_state_[i] = val;
        }
    }
    
    // Read counters if available
    for (size_t i = 0; i < capabilities_.counter_channels; i++) {
        uint32_t val = 0;
        if (readSDO(CounterValue, i + 1, &val, 4)) {
            counter_values_[i] = val;
        }
    }
    
    checkDigitalInputChanges();
    checkAnalogThresholds();
    
    // Save previous state for next cycle
    prev_digital_input_state_ = digital_input_state_;
    prev_analog_input_state_ = analog_input_state_;
}

void IOModule::processDigitalInputs(const uint8_t* data, size_t offset, size_t len) {
    if (offset >= len) return;
    size_t copy_len = std::min(digital_input_state_.size(), len - offset);
    memcpy(digital_input_state_.data(), data + offset, copy_len);
}

void IOModule::processAnalogInputs(const uint8_t* data, size_t offset, size_t len) {
    size_t ai_count = analog_input_state_.size();
    for (size_t i = 0; i < ai_count && (offset + i * 2 + 1) < len; i++) {
        analog_input_state_[i] = static_cast<int16_t>(
            data[offset + i * 2] | (data[offset + i * 2 + 1] << 8));
    }
}

void IOModule::checkDigitalInputChanges() {
    if (!digital_callback_) return;
    
    for (size_t block = 0; block < digital_input_state_.size(); block++) {
        uint8_t changed = digital_input_state_[block] ^ prev_digital_input_state_[block];
        if (changed) {
            for (int bit = 0; bit < 8; bit++) {
                if (changed & (1 << bit)) {
                    bool state = digital_input_state_[block] & (1 << bit);
                    digital_callback_(m_coe.slaveIndex(), block * 8 + bit, state);
                }
            }
            
            if (event_callback_) {
                event_callback_(IOEvent::DigitalInputChanged, m_coe.slaveIndex(), 
                               block, digital_input_state_[block]);
            }
        }
    }
}

void IOModule::checkAnalogThresholds() {
    if (!analog_callback_) return;
    
    for (size_t ch = 0; ch < analog_input_state_.size(); ch++) {
        int16_t val = analog_input_state_[ch];
        int16_t prev = prev_analog_input_state_[ch];
        
        if (ch < ai_configs_.size()) {
            const auto& cfg = ai_configs_[ch];
            
            // Check upper limit
            if ((cfg.trigger & InterruptTrigger::UpperLimit) && 
                val >= cfg.upper_limit && prev < cfg.upper_limit) {
                analog_callback_(m_coe.slaveIndex(), ch + 1, val, 1);
                if (event_callback_) {
                    event_callback_(IOEvent::AnalogUpperLimit, m_coe.slaveIndex(), ch + 1, val);
                }
            }
            
            // Check lower limit
            if ((cfg.trigger & InterruptTrigger::LowerLimit) &&
                val <= cfg.lower_limit && prev > cfg.lower_limit) {
                analog_callback_(m_coe.slaveIndex(), ch + 1, val, 2);
                if (event_callback_) {
                    event_callback_(IOEvent::AnalogLowerLimit, m_coe.slaveIndex(), ch + 1, val);
                }
            }
            
            // Check delta
            if ((cfg.trigger & InterruptTrigger::Delta) &&
                std::abs(val - prev) >= cfg.delta) {
                analog_callback_(m_coe.slaveIndex(), ch + 1, val, 3);
                if (event_callback_) {
                    event_callback_(IOEvent::AnalogDeltaTriggered, m_coe.slaveIndex(), ch + 1, val);
                }
            }
        }
    }
}

// ============================================================================
// Digital Input Operations
// ============================================================================

bool IOModule::readDigitalInput(uint8_t channel) {
    uint8_t block = channel / 8;
    uint8_t bit = channel % 8;
    
    if (block >= digital_input_state_.size()) return false;
    
    return (digital_input_state_[block] & (1 << bit)) != 0;
}

uint8_t IOModule::readDigitalInput8(uint8_t block) {
    if (block >= digital_input_state_.size()) return 0;
    return digital_input_state_[block];
}

uint16_t IOModule::readDigitalInput16(uint8_t block) {
    size_t base = block * 2;
    if (base + 1 >= digital_input_state_.size()) return 0;
    
    return digital_input_state_[base] | 
           (static_cast<uint16_t>(digital_input_state_[base + 1]) << 8);
}

uint32_t IOModule::readDigitalInput32(uint8_t block) {
    size_t base = block * 4;
    if (base + 3 >= digital_input_state_.size()) return 0;
    
    return digital_input_state_[base] |
           (static_cast<uint32_t>(digital_input_state_[base + 1]) << 8) |
           (static_cast<uint32_t>(digital_input_state_[base + 2]) << 16) |
           (static_cast<uint32_t>(digital_input_state_[base + 3]) << 24);
}

bool IOModule::setDigitalInputPolarity(uint8_t block, uint8_t mask) {
    return writeSDO(DigitalInputPolarity8, block + 1, &mask, 1);
}

bool IOModule::setDigitalInputFilter(uint8_t block, uint8_t filter_const) {
    return writeSDO(DigitalInputFilter8, block + 1, &filter_const, 1);
}

bool IOModule::setDigitalInputInterrupt(uint8_t block, uint8_t mask, EdgeType edge) {
    bool result = true;
    
    if (static_cast<uint8_t>(edge) & static_cast<uint8_t>(EdgeType::Any)) {
        result &= writeSDO(DigitalInterruptMaskAnyChange8, block + 1, &mask, 1);
    }
    if (static_cast<uint8_t>(edge) & static_cast<uint8_t>(EdgeType::Rising)) {
        result &= writeSDO(DigitalInterruptMaskLowToHigh8, block + 1, &mask, 1);
    }
    if (static_cast<uint8_t>(edge) & static_cast<uint8_t>(EdgeType::Falling)) {
        result &= writeSDO(DigitalInterruptMaskHighToLow8, block + 1, &mask, 1);
    }
    
    return result;
}

bool IOModule::setDigitalInterruptEnable(bool enable) {
    uint8_t val = enable ? 1 : 0;
    return writeSDO(DigitalInterruptEnable, 0, &val, 1);
}

// ============================================================================
// Digital Output Operations
// ============================================================================

bool IOModule::writeDigitalOutput(uint8_t channel, bool state) {
    return setDigitalOutputBit(channel, state);
}

bool IOModule::writeDigitalOutput8(uint8_t block, uint8_t value) {
    if (block >= digital_output_state_.size()) return false;
    
    digital_output_state_[block] = value;
    return writeSDO(DigitalOutput8, block + 1, &value, 1);
}

bool IOModule::writeDigitalOutput16(uint8_t block, uint16_t value) {
    size_t base = block * 2;
    if (base + 1 >= digital_output_state_.size()) return false;
    
    digital_output_state_[base] = value & 0xFF;
    digital_output_state_[base + 1] = (value >> 8) & 0xFF;
    
    return writeSDO(DigitalOutput16, block + 1, &value, 2);
}

bool IOModule::writeDigitalOutput32(uint8_t block, uint32_t value) {
    size_t base = block * 4;
    if (base + 3 >= digital_output_state_.size()) return false;
    
    digital_output_state_[base] = value & 0xFF;
    digital_output_state_[base + 1] = (value >> 8) & 0xFF;
    digital_output_state_[base + 2] = (value >> 16) & 0xFF;
    digital_output_state_[base + 3] = (value >> 24) & 0xFF;
    
    return writeSDO(DigitalOutput32, block + 1, &value, 4);
}

bool IOModule::setDigitalOutputBit(uint8_t channel, bool state) {
    uint8_t block = channel / 8;
    uint8_t bit = channel % 8;
    
    if (block >= digital_output_state_.size()) return false;
    
    if (state) {
        digital_output_state_[block] |= (1 << bit);
    } else {
        digital_output_state_[block] &= ~(1 << bit);
    }
    
    return writeSDO(DigitalOutput8, block + 1, &digital_output_state_[block], 1);
}

bool IOModule::toggleDigitalOutput(uint8_t channel) {
    uint8_t block = channel / 8;
    uint8_t bit = channel % 8;
    
    if (block >= digital_output_state_.size()) return false;
    
    digital_output_state_[block] ^= (1 << bit);
    
    return writeSDO(DigitalOutput8, block + 1, &digital_output_state_[block], 1);
}

bool IOModule::setDigitalOutputPolarity(uint8_t block, uint8_t mask) {
    return writeSDO(DigitalOutputPolarity8, block + 1, &mask, 1);
}

bool IOModule::setDigitalOutputErrorMode(uint8_t block, uint8_t mode, uint8_t value) {
    bool result = writeSDO(DigitalOutputErrorMode8, block + 1, &mode, 1);
    if (result && mode == ErrorMode::UseErrorValue) {
        result &= writeSDO(DigitalOutputErrorValue8, block + 1, &value, 1);
    }
    return result;
}

// ============================================================================
// Analog Input Operations
// ============================================================================

int16_t IOModule::readAnalogInput16(uint8_t channel) {
    if (channel == 0 || channel > analog_input_state_.size()) return 0;
    return analog_input_state_[channel - 1];
}

int32_t IOModule::readAnalogInput32(uint8_t channel) {
    int32_t val = 0;
    readSDO(AnalogInput32, channel, &val, 4);
    return val;
}

int32_t IOModule::readAnalogInputScaled(uint8_t channel) {
    if (channel == 0 || channel > ai_configs_.size()) return 0;
    
    int16_t raw = readAnalogInput16(channel);
    const auto& cfg = ai_configs_[channel - 1];
    
    return applyScaling(raw, cfg.offset, cfg.scaling);
}

bool IOModule::setAnalogInputScaling(uint8_t channel, int32_t offset, int32_t scaling) {
    if (channel == 0 || channel > ai_configs_.size()) return false;
    
    ai_configs_[channel - 1].offset = offset;
    ai_configs_[channel - 1].scaling = scaling;
    
    bool result = writeSDO(AnalogInputOffset, channel, &offset, 4);
    result &= writeSDO(AnalogInputScaling, channel, &scaling, 4);
    
    return result;
}

bool IOModule::setAnalogInputRange(uint8_t channel, int32_t raw_min, int32_t raw_max,
                                   int32_t eng_min, int32_t eng_max) {
    if (raw_max == raw_min) return false;
    
    // Calculate scaling factor (Q16 fixed point)
    int64_t scale = ((int64_t)(eng_max - eng_min) << 16) / (raw_max - raw_min);
    int32_t offset = eng_min - ((raw_min * scale) >> 16);
    
    return setAnalogInputScaling(channel, offset, static_cast<int32_t>(scale));
}

bool IOModule::setAnalogInputSIUnit(uint8_t channel, uint8_t unit_type, uint8_t prefix) {
    uint32_t si_unit = (static_cast<uint32_t>(prefix) << 16) | unit_type;
    return writeSDO(AnalogInputSIUnit, channel, &si_unit, 4);
}

bool IOModule::setAnalogInputThreshold(uint8_t channel, int16_t upper_limit,
                                       int16_t lower_limit, int16_t delta,
                                       uint8_t trigger) {
    if (channel == 0 || channel > ai_configs_.size()) return false;
    
    auto& cfg = ai_configs_[channel - 1];
    cfg.upper_limit = upper_limit;
    cfg.lower_limit = lower_limit;
    cfg.delta = delta;
    cfg.trigger = trigger;
    
    bool result = writeSDO(AnalogInputUpperLimit, channel, &upper_limit, 2);
    result &= writeSDO(AnalogInputLowerLimit, channel, &lower_limit, 2);
    result &= writeSDO(AnalogInputDelta, channel, &delta, 2);
    result &= writeSDO(AnalogInputInterruptTrigger, channel, &trigger, 1);
    
    return result;
}

bool IOModule::setAnalogInputInterruptEnable(bool enable) {
    uint8_t val = enable ? 1 : 0;
    return writeSDO(AnalogInputInterruptEnable, 0, &val, 1);
}

// ============================================================================
// Analog Output Operations
// ============================================================================

bool IOModule::writeAnalogOutput16(uint8_t channel, int16_t value) {
    if (channel == 0 || channel > analog_output_state_.size()) return false;
    
    analog_output_state_[channel - 1] = value;
    return writeSDO(AnalogOutput16, channel, &value, 2);
}

bool IOModule::writeAnalogOutput32(uint8_t channel, int32_t value) {
    return writeSDO(AnalogOutput32, channel, &value, 4);
}

bool IOModule::writeAnalogOutputScaled(uint8_t channel, int32_t value) {
    if (channel == 0 || channel > ao_configs_.size()) return false;
    
    const auto& cfg = ao_configs_[channel - 1];
    int32_t raw = reverseScaling(value, cfg.offset, cfg.scaling);
    
    return writeAnalogOutput16(channel, static_cast<int16_t>(raw));
}

bool IOModule::setAnalogOutputScaling(uint8_t channel, int32_t offset, int32_t scaling) {
    if (channel == 0 || channel > ao_configs_.size()) return false;
    
    ao_configs_[channel - 1].offset = offset;
    ao_configs_[channel - 1].scaling = scaling;
    
    bool result = writeSDO(AnalogOutputOffset, channel, &offset, 4);
    result &= writeSDO(AnalogOutputScaling, channel, &scaling, 4);
    
    return result;
}

bool IOModule::setAnalogOutputErrorMode(uint8_t channel, uint8_t mode, int16_t value) {
    bool result = writeSDO(AnalogOutputErrorMode, channel, &mode, 1);
    if (result && mode == ErrorMode::UseErrorValue) {
        result &= writeSDO(AnalogOutputErrorValue, channel, &value, 2);
    }
    return result;
}

// ============================================================================
// Counter Operations
// ============================================================================

uint32_t IOModule::readCounter(uint8_t channel) {
    if (channel == 0 || channel > counter_values_.size()) return 0;
    return counter_values_[channel - 1];
}

bool IOModule::setCounterPreset(uint8_t channel, uint32_t preset) {
    return writeSDO(CounterPreset, channel, &preset, 4);
}

bool IOModule::loadCounterPreset(uint8_t channel) {
    uint8_t control = CounterControlBits::Preset;
    return writeSDO(CounterControl, channel, &control, 1);
}

bool IOModule::configureCounter(uint8_t channel, bool enable, bool count_up) {
    uint8_t control = 0;
    if (enable) control |= CounterControlBits::Enable;
    if (count_up) control |= CounterControlBits::CountUp;
    return writeSDO(CounterControl, channel, &control, 1);
}

uint8_t IOModule::getCounterStatus(uint8_t channel) {
    uint8_t status = 0;
    readSDO(CounterStatus, channel, &status, 1);
    return status;
}

// ============================================================================
// Frequency Input Operations
// ============================================================================

uint32_t IOModule::readFrequencyInput(uint8_t channel) {
    uint32_t freq = 0;
    readSDO(FrequencyInput, channel, &freq, 4);
    return freq;
}

uint32_t IOModule::readPeriodInput(uint8_t channel) {
    uint32_t period = 0;
    readSDO(PeriodInput, channel, &period, 4);
    return period;
}

uint16_t IOModule::readDutyCycleInput(uint8_t channel) {
    uint16_t duty = 0;
    readSDO(DutyCycleInput, channel, &duty, 2);
    return duty;
}

// ============================================================================
// PWM Output Operations
// ============================================================================

bool IOModule::setPWMDutyCycle(uint8_t channel, uint16_t duty) {
    return writeSDO(PWMDutyCycle, channel, &duty, 2);
}

bool IOModule::setPWMFrequency(uint8_t channel, uint32_t frequency) {
    return writeSDO(PWMFrequency, channel, &frequency, 4);
}

bool IOModule::configurePWM(uint8_t channel, uint32_t frequency, uint16_t duty) {
    bool result = setPWMFrequency(channel, frequency);
    result &= setPWMDutyCycle(channel, duty);
    return result;
}

// ============================================================================
// Event Handling
// ============================================================================

void IOModule::setEventCallback(IOEventCallback callback) {
    event_callback_ = callback;
}

void IOModule::setDigitalInputCallback(DigitalInputCallback callback) {
    digital_callback_ = callback;
}

void IOModule::setAnalogThresholdCallback(AnalogThresholdCallback callback) {
    analog_callback_ = callback;
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string IOModule::getDiagnostics() const {
    std::string diag;
    diag += "CiA 401 I/O Module\n";
    diag += "  Slave: " + std::to_string(m_coe.slaveIndex()) + "\n";
    diag += "  Type: " + std::string(getModuleTypeName(capabilities_.module_type)) + "\n";
    diag += "  Digital In: " + std::to_string(capabilities_.getTotalDigitalInputs()) + "\n";
    diag += "  Digital Out: " + std::to_string(capabilities_.getTotalDigitalOutputs()) + "\n";
    diag += "  Analog In: " + std::to_string(capabilities_.getTotalAnalogInputs()) + "\n";
    diag += "  Analog Out: " + std::to_string(capabilities_.getTotalAnalogOutputs()) + "\n";
    
    if (capabilities_.has_counters) {
        diag += "  Counters: " + std::to_string(capabilities_.counter_channels) + "\n";
    }
    if (capabilities_.has_pwm_output) {
        diag += "  PWM Outputs: " + std::to_string(capabilities_.pwm_channels) + "\n";
    }
    
    return diag;
}

bool IOModule::hasFault() const {
    return has_fault_;
}

bool IOModule::resetFault() {
    has_fault_ = false;
    last_error_ = 0;
    return true;
}

// ============================================================================
// Internal SDO Methods
// ============================================================================

bool IOModule::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    size_t actual_size;
    return m_coe.readSync(index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs, &actual_size);
}

bool IOModule::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    auto result = m_coe.writeSync(index, subindex, data, len, {.timeout_ms = EtherCAT::SDO::kDefaultSDOTimeoutMs});
    return result.has_value();
}

} // namespace CiA401
