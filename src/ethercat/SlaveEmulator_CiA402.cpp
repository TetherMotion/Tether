/**
 * @file SlaveEmulator_CiA402.cpp
 * @brief CiA 402 DriveState emulation.
 *
 * TU split out of SlaveEmulator.cpp.
 */

#include "SlaveEmulator.hpp"
#include "tether/ethercat/DCClass.hpp"
#include "tether/platform/EspCompat.hpp"

#include <bit>

#define LOG_TAG "EC_EMU"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

namespace EtherCAT {
namespace Emulator {

// ============================================================================
// CiA 402 Drive State Implementation
// ============================================================================

namespace CiA402 {

uint16_t DriveState::getStatusWord() const {
    uint16_t sw = 0;
    
    switch (state) {
        case State::NOT_READY_TO_SWITCH_ON:
            sw = 0x0000;
            break;
        case State::SWITCH_ON_DISABLED:
            sw = 0x0040;
            break;
        case State::READY_TO_SWITCH_ON:
            sw = 0x0021;
            break;
        case State::SWITCHED_ON:
            sw = 0x0023;
            break;
        case State::OPERATION_ENABLED:
            sw = 0x0027;
            break;
        case State::QUICK_STOP_ACTIVE:
            sw = 0x0007;
            break;
        case State::FAULT_REACTION_ACTIVE:
            sw = 0x000F;
            break;
        case State::FAULT:
            sw = 0x0008;
            break;
    }
    
    // Add mode-specific bits
    if (operation_mode == target_mode) {
        sw |= 0x0400;  // Target reached / mode-specific
    }
    
    if (homing_complete && operation_mode == 6) {
        sw |= 0x1000;  // Homing attained
    }
    
    return sw;
}

void DriveState::processControlWord(uint16_t cw) {
    // State machine transitions based on control word
    bool reset = false;
    switch (state) {
        case State::NOT_READY_TO_SWITCH_ON:
            // Automatic transition to SWITCH_ON_DISABLED
            state = State::SWITCH_ON_DISABLED;
            break;
            
        case State::SWITCH_ON_DISABLED:
            // Shutdown command: bit 0=0, bit 1=1, bit 2=0
            if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            break;
            
        case State::READY_TO_SWITCH_ON:
            // Switch on command: bit 0=1, bit 1=1, bit 2=0
            if ((cw & 0x0007) == 0x0007) {
                state = State::SWITCHED_ON;
            }
            // Disable voltage: bit 1=0
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::SWITCHED_ON:
            // Enable operation: bit 0=1, bit 1=1, bit 2=1, bit 3=1
            if ((cw & 0x000F) == 0x000F) {
                state = State::OPERATION_ENABLED;
            }
            // Shutdown
            else if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::OPERATION_ENABLED:
            // Quick stop: bit 2=0
            if ((cw & 0x0004) == 0) {
                state = State::QUICK_STOP_ACTIVE;
            }
            // Disable operation: bit 3=0
            else if ((cw & 0x0008) == 0) {
                state = State::SWITCHED_ON;
            }
            // Shutdown
            else if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::QUICK_STOP_ACTIVE:
            // Enable operation from quick stop
            if ((cw & 0x000F) == 0x000F) {
                state = State::OPERATION_ENABLED;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::FAULT_REACTION_ACTIVE:
            // Automatic transition to FAULT
            state = State::FAULT;
            break;
            
        case State::FAULT:
            // Fault reset: bit 7 rising edge
            reset = (cw & 0x0080) != 0;
            if (reset && !prev_reset) {
                state = State::SWITCH_ON_DISABLED;
                error_code = 0;
            }
            prev_reset = reset;
            break;
    }
}

void DriveState::simulate(uint32_t delta_us) {
    if (state != State::OPERATION_ENABLED) {
        actual_velocity = 0;
        return;
    }
    
    // Simple motion simulation based on mode
    switch (operation_mode) {
        case 1:  // Profile Position
        {
            int32_t error = target_position - actual_position;
            int32_t max_step = 1000 * delta_us / 1000;  // Scale with time
            if (error > max_step) error = max_step;
            else if (error < -max_step) error = -max_step;
            actual_position += error;
            actual_velocity = error * 1000000 / static_cast<int32_t>(delta_us);
            break;
        }
        case 3:  // Profile Velocity
            actual_velocity = target_velocity;  // Instant for simulation
            actual_position += actual_velocity * delta_us / 1000000;
            break;
        case 4:  // Profile Torque
            actual_torque = target_torque;
            break;
        case 8:  // Cyclic Sync Position
            actual_position = target_position;
            break;
        case 9:  // Cyclic Sync Velocity
            actual_velocity = target_velocity;
            actual_position += actual_velocity * delta_us / 1000000;
            break;
        case 10: // Cyclic Sync Torque
            actual_torque = target_torque;
            break;
    }
}

}  // namespace CiA402

}  // namespace Emulator
}  // namespace EtherCAT
