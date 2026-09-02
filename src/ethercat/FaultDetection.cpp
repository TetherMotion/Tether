/**
 * @file FaultDetection.cpp
 * @brief Implementation of EtherCAT Fault Detection (instance-based, no globals)
 */

#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>
#include <algorithm>

namespace EtherCAT {

static const char* TAG = "EtherCATFault";

// ============================================================================
// AL Status Code Names
// ============================================================================

const char* getALStatusCodeName(ALStatusCode code) {
    switch (code) {
        case ALStatusCode::NoError: return "No error";
        case ALStatusCode::UnspecifiedError: return "Unspecified error";
        case ALStatusCode::NoMemory: return "No memory";
        case ALStatusCode::InvalidDeviceSetup: return "Invalid device setup";
        case ALStatusCode::ReservedCompatibility: return "Reserved due to compatibility reasons";
        case ALStatusCode::InvalidRequestedStateChange: return "Invalid requested state change";
        case ALStatusCode::UnknownRequestedState: return "Unknown requested state";
        case ALStatusCode::BootstrapNotSupported: return "Bootstrap not supported";
        case ALStatusCode::NoValidFirmware: return "No valid firmware";
        case ALStatusCode::InvalidMailboxConfig: return "Invalid mailbox configuration";
        case ALStatusCode::InvalidMailboxConfigPreOp: return "Invalid mailbox configuration (PRE_OP)";
        case ALStatusCode::InvalidSyncManagerConfig: return "Invalid Sync Manager configuration";
        case ALStatusCode::NoValidInputs: return "No valid inputs available";
        case ALStatusCode::NoValidOutputs: return "No valid outputs";
        case ALStatusCode::SynchronizationError: return "Synchronization error";
        case ALStatusCode::SyncManagerWatchdog: return "Sync Manager watchdog";
        case ALStatusCode::InvalidSyncManagerTypes: return "Invalid Sync Manager types";
        case ALStatusCode::InvalidOutputConfig: return "Invalid output configuration";
        case ALStatusCode::InvalidInputConfig: return "Invalid input configuration";
        case ALStatusCode::InvalidWatchdogConfig: return "Invalid watchdog configuration";
        case ALStatusCode::SlaveNeedsColdStart: return "Slave needs cold start";
        case ALStatusCode::SlaveNeedsInit: return "Slave needs INIT";
        case ALStatusCode::SlaveNeedsPreOp: return "Slave needs PRE_OP";
        case ALStatusCode::SlaveNeedsSafeOp: return "Slave needs SAFE_OP";
        case ALStatusCode::InvalidInputMapping: return "Invalid input mapping";
        case ALStatusCode::InvalidOutputMapping: return "Invalid output mapping";
        case ALStatusCode::InconsistentSettings: return "Inconsistent settings";
        case ALStatusCode::FreeRunNotSupported: return "FreeRun not supported";
        case ALStatusCode::SyncModeNotSupported: return "Sync mode not supported";
        case ALStatusCode::FreeRunNeeds3BufferMode: return "FreeRun needs 3-buffer mode";
        case ALStatusCode::BackgroundWatchdog: return "Background watchdog";
        case ALStatusCode::NoValidInputsAndOutputs: return "No valid inputs and outputs";
        case ALStatusCode::FatalSyncError: return "Fatal sync error";
        case ALStatusCode::NoSyncError: return "No sync error (Err74.1)";
        case ALStatusCode::InvalidDCConfig: return "Invalid DC configuration";
        case ALStatusCode::InvalidDCSyncUnit: return "Invalid DC sync unit";
        case ALStatusCode::InvalidDCCycleTime: return "Invalid DC cycle time";
        case ALStatusCode::InvalidDCLatchConfig: return "Invalid DC latch configuration";
        case ALStatusCode::PLLError: return "PLL error";
        case ALStatusCode::DCSync1CycleTime: return "DC SYNC1 cycle time";
        case ALStatusCode::MBoxEoE: return "Mailbox EoE error";
        case ALStatusCode::MBoxCoE: return "Mailbox CoE error";
        case ALStatusCode::MBoxFoE: return "Mailbox FoE error";
        case ALStatusCode::MBoxSoE: return "Mailbox SoE error";
        case ALStatusCode::MBoxVoE: return "Mailbox VoE error";
        case ALStatusCode::EEPROMNoAccess: return "EEPROM no access";
        case ALStatusCode::EEPROMError: return "EEPROM error";
        case ALStatusCode::ExternalHardwareNotReady: return "External hardware not ready";
        case ALStatusCode::SlaveRestartedLocally: return "Slave restarted locally";
        case ALStatusCode::DeviceIdUpdateError: return "Device ID update error";
        case ALStatusCode::ApplicationControllerAvail: return "Application controller available";
        default: return "Unknown error";
    }
}

const char* getALStatusCodeName(uint16_t code) {
    return getALStatusCodeName(static_cast<ALStatusCode>(code));
}

// ============================================================================
// CiA 402 Error Code Names
// ============================================================================

const char* getCiA402ErrorCodeName(CiA402ErrorCode code) {
    switch (code) {
        case CiA402ErrorCode::NoError: return "No error";
        case CiA402ErrorCode::GenericError: return "Generic error";
        case CiA402ErrorCode::OverCurrent: return "Over current";
        case CiA402ErrorCode::OverCurrentInternal: return "Over current internal";
        case CiA402ErrorCode::OverCurrentOutputA: return "Over current output A";
        case CiA402ErrorCode::OverCurrentOutputB: return "Over current output B";
        case CiA402ErrorCode::OverVoltage: return "Over voltage";
        case CiA402ErrorCode::OverVoltageSupply: return "Over voltage supply";
        case CiA402ErrorCode::UnderVoltage: return "Under voltage";
        case CiA402ErrorCode::UnderVoltageSupply: return "Under voltage supply";
        case CiA402ErrorCode::OverTemperature: return "Over temperature";
        case CiA402ErrorCode::OverTemperatureMotor: return "Over temperature motor";
        case CiA402ErrorCode::OverTemperatureDrive: return "Over temperature drive";
        case CiA402ErrorCode::SupplyVoltageFailure: return "Supply voltage failure";
        case CiA402ErrorCode::InternalSupplyFailed: return "Internal supply failed";
        case CiA402ErrorCode::OutputStageProtection: return "Output stage protection";
        case CiA402ErrorCode::PositionLimitExceeded: return "Position limit exceeded";
        case CiA402ErrorCode::PositionSensorError: return "Position sensor error";
        case CiA402ErrorCode::EncoderError: return "Encoder error";
        case CiA402ErrorCode::FollowingError: return "Following error";
        case CiA402ErrorCode::VelocityTooHigh: return "Velocity too high";
        case CiA402ErrorCode::ExternalError: return "External error";
        case CiA402ErrorCode::SoftwareError: return "Software error";
        case CiA402ErrorCode::SoftwareReset: return "Software reset";
        case CiA402ErrorCode::ObjectDictionaryError: return "Object dictionary error";
        case CiA402ErrorCode::ObjectDictionaryMissing: return "Object dictionary missing";
        case CiA402ErrorCode::CANopenError: return "CANopen error";
        case CiA402ErrorCode::PDOLengthError: return "PDO length error";
        case CiA402ErrorCode::EmergencyBufferFull: return "Emergency buffer full";
        case CiA402ErrorCode::CommWatchdogError: return "Communication watchdog error";
        case CiA402ErrorCode::CommError: return "Communication error";
        default: return "Unknown CiA 402 error";
    }
}

const char* getCiA402ErrorCodeName(uint16_t code) {
    return getCiA402ErrorCodeName(static_cast<CiA402ErrorCode>(code));
}

// ============================================================================
// Manufacturer Fault Parsing
// ============================================================================

ManufacturerFault ManufacturerFault::parse(uint16_t raw_code,
                                             uint32_t vendor_id,
                                             uint32_t product_code) {
    ManufacturerFault fault = {};
    fault.raw_code = raw_code;

    // Common format: class * 10 + subcode
    // Example: Err74.1 = (74 * 10) + 1 = 741
    // Or class.subcode directly encoded

    // Try to decode as "ErrXX.Y" format
    if (raw_code >= 100) {
        fault.class_code = static_cast<uint8_t>(raw_code / 10);
        fault.sub_code = static_cast<uint8_t>(raw_code % 10);
    } else {
        fault.class_code = static_cast<uint8_t>(raw_code);
        fault.sub_code = 0;
    }

    // Known manufacturer fault codes
    switch (raw_code) {
        // Common "No Sync" faults
        case 741: fault.description = "No Sync (Err74.1)"; break;
        case 740: fault.description = "DC Sync Error (Err74.0)"; break;

        // Over current
        case 200: fault.description = "Over current"; break;
        case 201: fault.description = "Over current (output A)"; break;
        case 202: fault.description = "Over current (output B)"; break;

        // Over voltage
        case 300: fault.description = "Over voltage"; break;
        case 310: fault.description = "Under voltage"; break;

        // Temperature
        case 400: fault.description = "Over temperature"; break;
        case 410: fault.description = "Motor over temperature"; break;

        // Encoder
        case 500: fault.description = "Encoder error"; break;
        case 510: fault.description = "Encoder loss"; break;

        // Following error
        case 600: fault.description = "Following error"; break;
        case 610: fault.description = "Velocity error"; break;

        // Communication
        case 700: fault.description = "Communication error"; break;
        case 710: fault.description = "CAN error"; break;
        case 720: fault.description = "EtherCAT error"; break;

        default:
            fault.description = "Unknown manufacturer fault";
            break;
    }

    return fault;
}

size_t ManufacturerFault::format(char* buffer, size_t buffer_size) const {
    if (buffer == nullptr || buffer_size == 0) {
        return 0;
    }

    if (sub_code > 0) {
        return snprintf(buffer, buffer_size, "Err%u.%u", class_code, sub_code);
    } else {
        return snprintf(buffer, buffer_size, "Err%u", class_code);
    }
}

// ============================================================================
// al_status_get_state_name
// ============================================================================

const char* al_status_get_state_name(uint16_t al_status) {
    switch (al_status & 0x000F) {
        case 1: return "INIT";
        case 2: return "PRE_OP";
        case 3: return "BOOTSTRAP";
        case 4: return "SAFE_OP";
        case 8: return "OP";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// FaultDetector implementation
// ============================================================================

FaultDetector::FaultDetector(IFaultTransport& transport)
    : transport_(transport) {
    // Zero-initialize the fault state array
    for (auto& s : slave_faults_) {
        std::memset(&s, 0, sizeof(s));
        s.al_status_code = ALStatusCode::NoError;
    }
}

bool FaultDetector::init(uint16_t slave_count) {
    if (initialized_) {
        return true;  // idempotent
    }

    if (slave_count > kMaxSlaves) {
        TETHER_LOGW("fault", "Slave count {} exceeds Tether fault detection max {} — clamping. "
                     "This is a Tether limit, not a slave limit. "
                     "Increase ECAT_FAULT_DETECTION_MAX_SLAVES in TetherConfig.hpp.",
                     static_cast<unsigned>(slave_count), kMaxSlaves);
    }
    slave_count_ = std::min(slave_count, static_cast<uint16_t>(kMaxSlaves));

    for (auto& state : slave_faults_) {
        state.clear();
        state.fault_count = 0;
        state.sync_error_count = 0;
        state.watchdog_count = 0;
        state.fault_detected_time = 0;
        state.last_poll_time = 0;
    }

    initialized_ = true;
    TETHER_LOGI(TAG, "Fault detection initialized for {} slaves", slave_count_);
    return true;
}

void FaultDetector::shutdown() {
    initialized_ = false;
    fault_callback_ = nullptr;
    slave_count_ = 0;
}

SlaveFaultState FaultDetector::poll(uint16_t slave_index) {
    SlaveFaultState state = {};
    state.al_status_code = ALStatusCode::NoError;

    if (!initialized_ || slave_index >= slave_count_) {
        return state;
    }

    // Read AL_STATUS (register 0x0130, 2 bytes)
    uint16_t al_status_val = 0;
    if (transport_.readRegister(slave_index, 0x0130, &al_status_val, 2)) {
        state.al_status = al_status_val;
    }

    // Read AL_STATUS_CODE (register 0x0134, 2 bytes)
    uint16_t al_code_val = 0;
    if (transport_.readRegister(slave_index, 0x0134, &al_code_val, 2)) {
        state.al_status_code = static_cast<ALStatusCode>(al_code_val);
    }

    // Check for error flag in AL_STATUS
    state.has_fault = al_status_has_error(state.al_status) ||
                       state.al_status_code != ALStatusCode::NoError;

    // Track sync errors specifically
    if (state.al_status_code == ALStatusCode::SynchronizationError ||
        state.al_status_code == ALStatusCode::NoSyncError ||
        state.al_status_code == ALStatusCode::FatalSyncError) {
        slave_faults_[slave_index].sync_error_count++;
    }

    // Track watchdog errors
    if (state.al_status_code == ALStatusCode::SyncManagerWatchdog ||
        state.al_status_code == ALStatusCode::BackgroundWatchdog) {
        slave_faults_[slave_index].watchdog_count++;
    }

    // Update stored state
    SlaveFaultState& stored = slave_faults_[slave_index];
    bool was_faulted = stored.has_fault;

    stored.al_status = state.al_status;
    stored.al_status_code = state.al_status_code;
    stored.has_fault = state.has_fault;
    stored.last_poll_time = transport_.getTimestampMs();

    // Call callback on new fault
    if (state.has_fault && !was_faulted && fault_callback_) {
        stored.fault_count++;
        stored.fault_detected_time = stored.last_poll_time;
        fault_callback_(slave_index, stored);
    }

    // Copy accumulated stats into returned state
    state.fault_count = stored.fault_count;
    state.sync_error_count = stored.sync_error_count;
    state.watchdog_count = stored.watchdog_count;
    state.fault_detected_time = stored.fault_detected_time;
    state.last_poll_time = stored.last_poll_time;

    return state;
}

uint16_t FaultDetector::pollAll() {
    uint16_t faulted = 0;
    for (uint16_t i = 0; i < slave_count_; i++) {
        SlaveFaultState state = poll(i);
        if (state.has_fault) {
            faulted++;
        }
    }
    return faulted;
}

const SlaveFaultState* FaultDetector::getState(uint16_t slave_index) const {
    if (!initialized_ || slave_index >= slave_count_) {
        return nullptr;
    }
    return &slave_faults_[slave_index];
}

bool FaultDetector::anyActive() const {
    for (uint16_t i = 0; i < slave_count_; i++) {
        if (slave_faults_[i].has_fault) {
            return true;
        }
    }
    return false;
}

bool FaultDetector::clear(uint16_t slave_index) {
    if (!initialized_ || slave_index >= slave_count_) {
        return false;
    }

    // Try to acknowledge error by writing to AL_CONTROL
    // Write ACK bit (0x10) to acknowledge error
    uint16_t al_control = 0x0010;

    if (!transport_.writeRegister(slave_index, 0x0120, &al_control, 2)) {
        return false;
    }

    // Wait and re-poll
    transport_.delayMs(50);
    SlaveFaultState state = poll(slave_index);

    if (!state.has_fault) {
        slave_faults_[slave_index].clear();
        TETHER_LOGI(TAG, "{}: Fault cleared", slavePrefix(slave_index).c_str());
        return true;
    }

    TETHER_LOGW(TAG, "{}: Fault still active after clear attempt", slavePrefix(slave_index).c_str());
    return false;
}

void FaultDetector::setCallback(FaultCallback callback) {
    fault_callback_ = std::move(callback);
}

void FaultDetector::diagnose(uint16_t slave_index) const {
    if (!initialized_ || slave_index >= slave_count_) {
        TETHER_LOGE(TAG, "Invalid slave index {}", slave_index);
        return;
    }

    // Poll current state first to get live AL_STATUS values
    // Note: We need to cast away const to call poll() which updates stored state
    const_cast<FaultDetector*>(this)->poll(slave_index);

    const SlaveFaultState& state = slave_faults_[slave_index];

    // Build a single-line diagnostics string.
    // Log level: ERROR if there is an active fault, WARN if there is no active
    // fault but past errors were recorded (statistics), INFO otherwise.
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
             "%s: AL_STATUS=0x%04X (State=%s%s) AL_STATUS_CODE=%s (0x%04X)",
             slavePrefix(slave_index).c_str(),
             state.al_status,
             al_status_get_state_name(state.al_status),
             al_status_has_error(state.al_status) ? ", ERROR" : "",
             getALStatusCodeName(state.al_status_code),
             static_cast<uint16_t>(state.al_status_code));

    if (state.error_code_603f != 0 && n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n,
                      " | CiA 402 Error Code 0x603F=0x%04X (%s)",
                      state.error_code_603f,
                      getCiA402ErrorCodeName(state.error_code_603f));
    }

    if (state.mfr_fault.raw_code != 0 && n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
        char fault_str[32];
        state.mfr_fault.format(fault_str, sizeof(fault_str));
        n += snprintf(buf + n, sizeof(buf) - n,
                      " | Manufacturer Fault: %s (%s)",
                      fault_str, state.mfr_fault.description);
    }

    // Only include statistics when at least one counter is non-zero.
    bool has_stats = (state.fault_count != 0 ||
                      state.sync_error_count != 0 ||
                      state.watchdog_count != 0);
    if (has_stats && n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n,
                      " | stats: faults=%lu sync=%lu wd=%lu",
                      (unsigned long)state.fault_count,
                      (unsigned long)state.sync_error_count,
                      (unsigned long)state.watchdog_count);
    }

    if (state.has_fault) {
        TETHER_LOGE(TAG, "{}", buf);
    } else if (has_stats) {
        TETHER_LOGW(TAG, "{}", buf);
    } else {
        TETHER_LOGI(TAG, "{}", buf);
    }

    // Check for specific error types and provide guidance
    if (state.al_status_code == ALStatusCode::NoSyncError ||
        state.al_status_code == ALStatusCode::SynchronizationError) {
        diagnoseNoSync(slave_index);
    }
}

void FaultDetector::diagnoseNoSync(uint16_t slave_index) const {
    TETHER_LOGE(TAG, "\n>>> NO SYNC ERROR (Err74.1) DIAGNOSIS <<<\n\nThis error indicates the slave's DC synchronization is failing.\nCommon causes:\n  1. SYNC0 start time is in the past\n  2. DC cycle time mismatch\n  3. Master not sending ARMW/FRMW frames\n  4. Propagation delay not properly compensated\n  5. PDO data not being exchanged at DC rate\n\nRecommended actions:\n  - Call dc_read_sync_config({}) to check DC state\n  - Verify SYNC0 start time is in the future\n  - Ensure PDO exchange is enabled before SAFE_OP\n  - Check DC cycle time matches slave expectations",
               slave_index);
} 

} // namespace EtherCAT
