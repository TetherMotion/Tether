/**
 * @file CiA402DriveStateMachine.cpp
 * @brief CiA 402 Drive State Machine and EtherCAT state transitions
 * 
 * This file contains the CiA 402 state machine implementation,
 * EtherCAT state transitions, and drive enable/disable logic.
 */

#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/platform/Platform.hpp"

#include <cinttypes>
#include <cstring>

static const char* TAG = "CiA402StateMachine";

namespace EtherCAT {

// ============================================================================
// State Name Functions
// ============================================================================

const char* getECStateName(ECState state) {
    switch (state) {
        case ECState::Init:      return "INIT";
        case ECState::PreOp:     return "PRE_OP";
        case ECState::Bootstrap: return "BOOTSTRAP";
        case ECState::SafeOp:    return "SAFE_OP";
        case ECState::Op:        return "OP";
        default:                 return "UNKNOWN";
    }
}

/**
 * @brief Format statusword diagnostic information
 * 
 * Returns a complete bit-by-bit breakdown of the CiA 402 statusword.
 * Format: "0x1637: OpEn,Volt,QS,Remote,TR,Bit12 | STATE"
 */
const char* formatStatuswordDiagnostics(uint16_t sw, char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) return "";
    
    buffer[0] = '\0';
    size_t pos = 0;
    
    auto append = [&](const char* fmt, auto... args) {
        if (pos >= buffer_size - 1) return;
        int n = snprintf(buffer + pos, buffer_size - pos, fmt, args...);
        if (n > 0) pos = std::min(pos + static_cast<size_t>(n), buffer_size - 1);
    };
    
    // Start with hex value
    append("0x%04X: ", sw);
    
    // State machine bits (0-2, 5-6)
    bool rtso = sw & (1 << 0);  // ReadyToSwitchOn
    bool so   = sw & (1 << 1);  // SwitchedOn
    bool oe   = sw & (1 << 2);  // OperationEnabled
    bool qs   = sw & (1 << 5);  // QuickStop (1=normal)
    bool sod  = sw & (1 << 6);  // SwitchOnDisabled
    
    if (oe) append("OpEn,");
    else if (so) append("SwitchOn,");
    else if (rtso) append("ReadyToSO,");
    else if (sod) append("SODisabled,");
    
    // Fault and warning
    if (sw & (1 << 3)) append("FAULT,");
    if (sw & (1 << 7)) append("WARN,");
    
    // Voltage enabled
    if (sw & (1 << 4)) append("Volt,");
    else append("NoVolt,");
    
    // Quick stop (inverted logic - 1 is OK)
    if (!qs) append("QSActive,");
    
    // Remote
    if (sw & (1 << 9)) append("Remote,");
    
    // Target reached
    if (sw & (1 << 10)) append("TR,");
    
    // Internal limit
    if (sw & (1 << 11)) append("Limit,");
    
    // Bit 12 (mode-specific: HomingAttained or SetPointAck)
    if (sw & (1 << 12)) append("Bit12,");
    
    // Bit 13 (mode-specific: HomingError or FollowingError)
    if (sw & (1 << 13)) append("Bit13,");
    
    // Remove trailing comma
    if (pos > 0 && buffer[pos-1] == ',') {
        buffer[pos-1] = '\0';
    }
    
    return buffer;
}

const char* getDriveStateName(DriveState state) {
    switch (state) {
        case DriveState::NotReadyToSwitchOn: return "Not Ready to Switch On";
        case DriveState::SwitchOnDisabled:   return "Switch On Disabled";
        case DriveState::ReadyToSwitchOn:    return "Ready to Switch On";
        case DriveState::SwitchedOn:         return "Switched On";
        case DriveState::OperationEnabled:   return "Operation Enabled";
        case DriveState::QuickStopActive:    return "Quick Stop Active";
        case DriveState::FaultReactionActive: return "Fault Reaction Active";
        case DriveState::Fault:              return "Fault";
        default:                             return "Unknown";
    }
}

DriveState decodeDriveState(uint16_t statusword) {
    // Decode according to CiA 402 state encoding
    // See CiA402Defs.hpp StatuswordBits
    
    // Check fault first
    if ((statusword & 0x004F) == 0x0008) {
        return DriveState::Fault;
    }
    if ((statusword & 0x004F) == 0x000F) {
        return DriveState::FaultReactionActive;
    }
    
    // Check other states
    if ((statusword & 0x006F) == 0x0000) {
        return DriveState::NotReadyToSwitchOn;
    }
    if ((statusword & 0x004F) == 0x0040) {
        return DriveState::SwitchOnDisabled;
    }
    if ((statusword & 0x006F) == 0x0021) {
        return DriveState::ReadyToSwitchOn;
    }
    if ((statusword & 0x006F) == 0x0023) {
        return DriveState::SwitchedOn;
    }
    if ((statusword & 0x006F) == 0x0027) {
        return DriveState::OperationEnabled;
    }
    if ((statusword & 0x006F) == 0x0007) {
        return DriveState::QuickStopActive;
    }
    
    return DriveState::Unknown;
}

// ============================================================================
// EtherCAT State Machine
// ============================================================================

ECState CiA402Drive::getECState() {
    uint8_t state_code = 0;
    if (m_master && m_master->readSlaveApplicationLayerState(m_slave_index, state_code)) {
        // Mask out error bit (bit 4) and other flags to get just the state
        return static_cast<ECState>(state_code & 0x0F);
    }
    return ECState::Unknown;
}

bool CiA402Drive::gotoInit() {
    TETHER_LOGI(TAG, "Slave %u: Requesting INIT state", m_slave_index);
    if (!m_master) return false;
    return m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Init));
}

bool CiA402Drive::gotoPreOp() {
    TETHER_LOGI(TAG, "Slave %u: Requesting PRE_OP state (with mailbox config)", m_slave_index);
    if (!m_master) return false;
    return m_master->transitionSlaveToPreOperational(m_slave_index);
}

bool CiA402Drive::gotoSafeOp() {
    TETHER_LOGI(TAG, "Slave %u: Requesting SAFE_OP state", m_slave_index);
    if (!m_master) return false;
    if (!m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::SafeOp)))
        return false;
    
    // Wait for slave to reach SAFE_OP (up to 2 seconds)
    for (int attempt = 0; attempt < 20; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        uint8_t state = 0;
        if (m_master->readSlaveApplicationLayerState(m_slave_index, state)) {
            if (state == static_cast<uint8_t>(ECState::SafeOp)) {
                TETHER_LOGI(TAG, "Slave %u: SAFE_OP confirmed after %d ms", m_slave_index, (attempt+1)*100);
                return true;
            }
        }
    }
    
    // Read final state and error for diagnostics
    uint8_t final_state = 0;
    m_master->readSlaveApplicationLayerState(m_slave_index, final_state);
    uint8_t asc[2] = {0};
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
    uint16_t al_code = asc[0] | (asc[1] << 8);
    TETHER_LOGE(TAG, "Slave %u: SAFE_OP not confirmed after 2s, state=0x%02X AL_CODE=0x%04X",
             m_slave_index, final_state, al_code);
    return false;
}

bool CiA402Drive::gotoOp() {
    TETHER_LOGI(TAG, "Slave %u: Requesting OP state", m_slave_index);
    if (!m_master) return false;
    // Request OP with Error Acknowledge bit (0x08 | 0x10 = 0x18)
    // Some slaves require the ACK bit to clear internal error latches.
    if (!m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Op) | 0x10))
        return false;
    
    // Wait for slave to actually reach OP (up to 5 seconds).
    // The DC realtime task drives PDO exchange when pdo_enabled_ is set,
    // which was done by transitionToOp() before calling this method.
    // The slave needs continuous PDO data on SM2 to accept the OP transition.
    const uint8_t* src_mac = m_master->getSrcMac();
    for (int attempt = 0; attempt < 50; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        
        uint8_t state = 0;
        if (m_master->readSlaveApplicationLayerState(m_slave_index, state)) {
            if (state == static_cast<uint8_t>(ECState::Op)) {
                TETHER_LOGI(TAG, "Slave %u: OP confirmed after %d ms", m_slave_index, (attempt+1)*100);
                return true;
            }
            // Check for unexpected state (not SAFE_OP or OP)
            if (state != static_cast<uint8_t>(ECState::SafeOp) &&
                state != static_cast<uint8_t>(ECState::Op)) {
                uint8_t asc[2] = {0};
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
                uint16_t al_code = asc[0] | (asc[1] << 8);
                TETHER_LOGE(TAG, "Slave %u: Unexpected state 0x%02X during OP transition (AL_CODE=0x%04X)",
                         m_slave_index, state, al_code);
                return false;
            }
            // Re-request OP every second - some slaves need repeated requests
            if ((attempt % 10) == 9) {
                m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Op) | 0x10);
                // Read raw AL_STATUS (16-bit, including error bit)
                uint16_t al_raw = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0130, al_raw, 200);
                // Read AL_STATUS_CODE
                uint16_t al_code = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, al_code, 200);
                // Read DC SYNC active register
                uint8_t dc_sync_act = 0;
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSyncAct, &dc_sync_act, 1, 200);

                // Read DC System Time
                uint8_t dc_time[8] = {0};
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSysTime, dc_time, 8, 200);
                uint64_t sys_time_lo = dc_time[0] | (dc_time[1]<<8) | (dc_time[2]<<16) | (dc_time[3]<<24);

                // Read SYNC Latch status (0x098E) — bit 0 toggles with each SYNC0 event
                uint8_t sync_latch = 0;
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSyncLatch, &sync_latch, 1, 200);
                // Read SM2 SM Event Request (0x0820) to see if outputs are being consumed
                uint8_t sm2_event = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0820, sm2_event, 200);
                // PDO exchange stats
                auto pstats = m_master->pdo().getPhysicalStats();
                TETHER_LOGI(TAG, "Slave %u: Still waiting for OP, AL_STATUS=0x%04X AL_CODE=0x%04X DC_SYNC_ACT=0x%02X DC_SysTime_lo=0x%08lX (%d ms)\n  PDO: fpwr_ok=%u fpwr_err=%u fprd_ok=%u fprd_err=%u  SYNC_LATCH=0x%02X SM2_EVT=0x%02X",
                         m_slave_index, al_raw, al_code, dc_sync_act, (unsigned long)sys_time_lo, (attempt+1)*100,
                         pstats.fpwr_success, pstats.fpwr_wkc_errors, pstats.fprd_success, pstats.fprd_wkc_errors, sync_latch, sm2_event);
            }
        }
    }
    
    // Read final state and error
    uint8_t final_state = 0;
    m_master->readSlaveApplicationLayerState(m_slave_index, final_state);
    uint8_t asc[2] = {0};
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
    uint16_t al_code = asc[0] | (asc[1] << 8);

    // Also read full AL_STATUS for error flag
    uint16_t al_status = 0;
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0130, al_status, 200);
    
    // Read PDO exchange stats to diagnose if PDO was ever active
    auto pstats = m_master->pdo().getPhysicalStats();
    
    TETHER_LOGW(TAG, "Slave %u: OP not confirmed after 5s, current state=0x%02X AL_CODE=0x%04X\n"
             "  AL_STATUS=0x%04X%s\n"
             "  PDO stats: fpwr_ok=%u fpwr_err=%u fprd_ok=%u fprd_err=%u",
             m_slave_index, final_state, al_code,
             al_status, (al_status & 0x10) ? " (ERROR)" : "",
             pstats.fpwr_success, pstats.fpwr_wkc_errors,
             pstats.fprd_success, pstats.fprd_wkc_errors);
    
    if (pstats.fpwr_success == 0 && pstats.fprd_success == 0) {
        TETHER_LOGE(TAG, "Slave %u: PDO exchange never occurred! Check that DC realtime loop has PDO exchange wired in.", m_slave_index);
    }
    
    return (final_state == static_cast<uint8_t>(ECState::Op));
}

bool CiA402Drive::transitionToOp(bool apply_pdo_mapping) {
    TETHER_LOGI(TAG, "Slave %u: Beginning transition to OP", m_slave_index);
    
    // Check if slave is already in PRE_OP. If so, skip gotoPreOp() to avoid
    // re-initializing the slave which would reset all CoE objects (PDO mapping,
    // operating mode, etc.) that were configured by the application.
    uint8_t current_state = 0;
    bool need_preop = true;
    if (m_master && m_master->readSlaveApplicationLayerState(m_slave_index, current_state)) {
        if (current_state == static_cast<uint8_t>(ECState::PreOp)) {
            TETHER_LOGI(TAG, "Slave %u: Already in PRE_OP (0x%02X), skipping gotoPreOp()", 
                     m_slave_index, current_state);
            need_preop = false;
        } else {
            TETHER_LOGI(TAG, "Slave %u: Current state=0x%02X, requesting PRE_OP", 
                     m_slave_index, current_state);
        }
    }
    
    if (need_preop) {
        if (!gotoPreOp()) {
            TETHER_LOGE(TAG, "Slave %u: Failed to reach PRE_OP", m_slave_index);
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(100);
    }
    
    // Apply fixed PDO mapping in PRE_OP (via assignFixedPDOs before this call)
    if (apply_pdo_mapping && !m_pdo_configured && m_rxpdo_size > 0) {
        if (!assignFixedPDOs(m_rxpdo_index, m_txpdo_index, m_rxpdo_size, m_txpdo_size)) {
            TETHER_LOGW(TAG, "Slave %u: PDO mapping failed, continuing anyway", m_slave_index);
        }
    }
    
    // CRITICAL: Configure SM2/SM3 (process data sync managers) from SII before SAFE_OP.
    // The slave validates SM2/SM3 during the PRE_OP → SAFE_OP transition and will
    // reject the transition with AL_STATUS_CODE 0x001E "Invalid input configuration"
    // if SM2/SM3 are not properly configured.
    TETHER_LOGI(TAG, "Slave %u: Configuring PDO sync managers (SM2/SM3) from SII", m_slave_index);
    if (!m_master->configureProcessDataSyncManagersFromSii(m_slave_index)) {
        TETHER_LOGW(TAG, "Slave %u: SM2/SM3 configuration from SII failed, SAFE_OP may fail", m_slave_index);
    }
    
    // CRITICAL: Update SM2/SM3 lengths to match the actual PDO mapping sizes.
    // The SII stores the drive's *default* PDO sizes, but assignFixedPDOs() may have
    // reconfigured the PDOs via SDO to use different sizes. The slave validates that
    // SM2/SM3 lengths match the actual PDO mapping during SAFE_OP transition.
    if (m_pdo_configured) {
        auto* slave_configs = m_master->pdo().slaveConfigs();
        const uint16_t rxpdo_size = m_rxpdo_size;
        const uint16_t txpdo_size = m_txpdo_size;
        
        if (rxpdo_size > 0 && slave_configs[m_slave_index].sm[2].type != PDO::SyncManagerType::Unused) {
            TETHER_LOGI(TAG, "Slave %u: Updating SM2 length: %u -> %u (from PDO config)",
                     m_slave_index, slave_configs[m_slave_index].sm[2].length, rxpdo_size);
            slave_configs[m_slave_index].sm[2].length = rxpdo_size;
            slave_configs[m_slave_index].rxpdo_size = rxpdo_size;
        }
        if (txpdo_size > 0 && slave_configs[m_slave_index].sm[3].type != PDO::SyncManagerType::Unused) {
            TETHER_LOGI(TAG, "Slave %u: Updating SM3 length: %u -> %u (from PDO config)",
                     m_slave_index, slave_configs[m_slave_index].sm[3].length, txpdo_size);
            slave_configs[m_slave_index].sm[3].length = txpdo_size;
            slave_configs[m_slave_index].txpdo_size = txpdo_size;
        }
        
        // Re-write SM2/SM3 to slave with corrected lengths
        // const uint8_t* src_mac = m_master->getSrcMac(); // Not used
        for (uint8_t sm = 2; sm < 4; sm++) {
            const auto& cfg = slave_configs[m_slave_index].sm[sm];
            if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0) {
                uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);

                // Disable SM first
                uint8_t disable = 0x00;
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 6), &disable, 1, 200);

                // Write physical address
                uint16_t addr_le = cfg.phys_start_addr;
                // Write the sync-manager registers with explicit little-endian fields.
                uint8_t addr_buf[2] = {static_cast<uint8_t>(cfg.phys_start_addr & 0xFF),
                                       static_cast<uint8_t>((cfg.phys_start_addr >> 8) & 0xFF)};
                m_master->writeRegister(SlaveAddress(m_slave_index), base, addr_buf, 2, 200);

                // Write length
                uint8_t len_buf[2] = {static_cast<uint8_t>(cfg.length & 0xFF),
                                      static_cast<uint8_t>((cfg.length >> 8) & 0xFF)};
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 2), len_buf, 2, 200);

                // Write control
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 4), &cfg.control, 1, 200);

                // Enable SM
                uint8_t activate = cfg.enable ? 0x01 : 0x00;
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 6), &activate, 1, 200);
                
                TETHER_LOGI(TAG, "Slave %u: Re-wrote SM%u: Addr=0x%04X Len=%u Ctrl=0x%02X",
                         m_slave_index, sm, cfg.phys_start_addr, cfg.length, cfg.control);
            }
        }
    }
    
    // PRE_OP -> SAFE_OP
    if (!gotoSafeOp()) {
        TETHER_LOGE(TAG, "Slave %u: Failed to reach SAFE_OP", m_slave_index);
        return false;
    }
    
    // CRITICAL: Reconfigure DC SYNC signals AFTER SM configuration
    // The initial DC config during slave discovery may have failed because
    // the slave wasn't ready. Now that we're in SAFE_OP with SM configured,
    // retry the DC SYNC configuration.
    TETHER_LOGI(TAG, "Slave %u: Reconfiguring DC SYNC in SAFE_OP", m_slave_index);
    if (m_master) {
        if (!m_master->dc().reconfigureSync(m_slave_index))
            TETHER_LOGW(TAG, "Slave %u: DC reconfiguration failed, continuing anyway", m_slave_index);
    }
    Tether::Platform::Clock::instance().delayMilliseconds(50);
    
    // CRITICAL: Enable PDO exchange BEFORE requesting OP!
    // The slave's PDI watchdog starts in SAFE_OP and expects process data.
    // We must start sending PDO data now to prevent watchdog timeout.
    TETHER_LOGI(TAG, "Slave %u: Enabling PDO exchange in SAFE_OP (pre-OP)", m_slave_index);
    if (m_master) {
        m_master->pdo().resetStats();
        m_master->dc().setPDOEnabled(true);
    }
    
    // Give the PDO exchange a moment to send some frames before OP request
    Tether::Platform::Clock::instance().delayMilliseconds(200);
    
    // DIAGNOSTIC: Read back SM2/SM3 and verify they're correct before OP request
    if (m_master) {
        for (uint8_t sm = 2; sm <= 3; sm++) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t sm_regs[8] = {0};
            m_master->readRegister(SlaveAddress(m_slave_index), base, sm_regs, 8, 200);
            uint16_t addr = sm_regs[0] | (sm_regs[1] << 8);
            uint16_t len  = sm_regs[2] | (sm_regs[3] << 8);
            uint8_t  ctrl = sm_regs[4];
            uint8_t  stat = sm_regs[5];
            uint8_t  act  = sm_regs[6];
            uint8_t  pdi  = sm_regs[7];
            TETHER_LOGI(TAG, "Slave %u: SM%u readback: Addr=0x%04X Len=%u Ctrl=0x%02X Status=0x%02X Act=0x%02X PDI=0x%02X",
                     m_slave_index, sm, addr, len, ctrl, stat, act, pdi);
        }
        // Read watchdog divider/counter
        uint16_t wdt_status = 0;
        m_master->readRegister(SlaveAddress(m_slave_index), 0x0440, wdt_status, 200);
        TETHER_LOGI(TAG, "Slave %u: Watchdog Status=0x%04X", m_slave_index, wdt_status);
    }
    
    // DIAGNOSTIC: Read PDO assignment and mode SDOs before OP request
    if (m_master) {
        m_master->sdoManager(m_slave_index).setDiagEnabled(true);
        auto sm2_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1C12, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (sm2_count_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1C12:0 (SM2 PDO count) = %u", m_slave_index, sm2_count_res.value());
        else
            TETHER_LOGW(TAG, "Slave %u: Failed to read 0x1C12:0", m_slave_index);
        
        auto sm2_pdo_res = m_master->sdoManager(m_slave_index).readU16( 0x1C12, 1, {.timeout_ms = m_sdo_timeout_ms});
        if (sm2_pdo_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1C12:1 (SM2 RxPDO) = 0x%04X", m_slave_index, sm2_pdo_res.value());
        else
            TETHER_LOGW(TAG, "Slave %u: Failed to read 0x1C12:1", m_slave_index);
        
        auto sm3_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1C13, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (sm3_count_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1C13:0 (SM3 PDO count) = %u", m_slave_index, sm3_count_res.value());
        else
            TETHER_LOGW(TAG, "Slave %u: Failed to read 0x1C13:0", m_slave_index);
        
        auto sm3_pdo_res = m_master->sdoManager(m_slave_index).readU16( 0x1C13, 1, {.timeout_ms = m_sdo_timeout_ms});
        if (sm3_pdo_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1C13:1 (SM3 TxPDO) = 0x%04X", m_slave_index, sm3_pdo_res.value());
        else
            TETHER_LOGW(TAG, "Slave %u: Failed to read 0x1C13:1", m_slave_index);
        
        auto mode_op_res = m_master->sdoManager(m_slave_index).readU8( 0x6060, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (mode_op_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x6060 (Modes of Operation) = %d", m_slave_index, (int8_t)mode_op_res.value());
        
        auto mode_disp_res = m_master->sdoManager(m_slave_index).readU8( 0x6061, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (mode_disp_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x6061 (Mode Display) = %d", m_slave_index, (int8_t)mode_disp_res.value());
        
        auto supported_modes_res = m_master->sdoManager(m_slave_index).readU32( 0x6502, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (supported_modes_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x6502 (Supported Modes) = 0x%08X", m_slave_index, supported_modes_res.value());
        
        auto rxpdo_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1600, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (rxpdo_count_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1600:0 (RxPDO entry count) = %u", m_slave_index, rxpdo_count_res.value());
        
        auto txpdo_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1A00, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (txpdo_count_res.has_value())
            TETHER_LOGI(TAG, "Slave %u: 0x1A00:0 (TxPDO entry count) = %u", m_slave_index, txpdo_count_res.value());
        m_master->sdoManager(m_slave_index).setDiagEnabled(false);
    }
    
    // Clear any pending error by writing Error Acknowledge + OP (0x18)
    // Some slaves need the ACK bit before accepting OP.
    if (m_master) {
        // First ack any error in current state
        uint16_t ack = static_cast<uint16_t>(ECState::SafeOp) | 0x10;  // SAFE_OP + ACK
        m_master->writeRegister(SlaveAddress(m_slave_index), 0x0120, ack);  // 0x0120 = AL_CONTROL
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }
    
    // SAFE_OP -> OP
    if (!gotoOp()) {
        TETHER_LOGE(TAG, "Slave %u: Failed to reach OP", m_slave_index);
        // Disable PDO on failure
        if (m_master) {
            m_master->dc().setPDOEnabled(false);
        }
        return false;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    
    TETHER_LOGI(TAG, "Slave %u: Successfully transitioned to OP", m_slave_index);
    return true;
}

// ============================================================================
// CiA 402 State Machine
// ============================================================================

DriveState CiA402Drive::getDriveState() {
    uint16_t sw;
    if (!readStatusword(sw)) {
        return DriveState::Unknown;
    }
    return decodeDriveState(sw);
}

uint16_t CiA402Drive::getStatusword() {
    uint16_t sw;
    if (readStatusword(sw)) {
        return sw;
    }
    return m_statusword;
}

bool CiA402Drive::enable(uint32_t timeout_ms) {
    TETHER_LOGI(TAG, "Slave %u: Enabling drive...", m_slave_index);
    
    // Reset any fault first
    DriveState state = getDriveState();
    if (state == DriveState::Fault) {
        TETHER_LOGI(TAG, "Slave %u: Resetting fault", m_slave_index);
        if (!resetFault()) {
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        state = getDriveState();
    }
    
    // Shutdown (Ready to Switch On)
    if (state == DriveState::SwitchOnDisabled) {
        m_controlword = 0x0006;  // Shutdown
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::ReadyToSwitchOn, timeout_ms)) {
            return false;
        }
        state = getDriveState();
    }
    
    // Switch On
    if (state == DriveState::ReadyToSwitchOn) {
        m_controlword = 0x0007;  // Switch On
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::SwitchedOn, timeout_ms)) {
            return false;
        }
        state = getDriveState();
    }
    
    // Enable Operation
    if (state == DriveState::SwitchedOn) {
        m_controlword = 0x000F;  // Enable Operation
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::OperationEnabled, timeout_ms)) {
            return false;
        }
    }
    
    TETHER_LOGI(TAG, "Slave %u: Drive enabled", m_slave_index);
    return true;
}

bool CiA402Drive::disable() {
    m_controlword = 0x0000;  // Disable voltage
    return writeControlword(m_controlword);
}

bool CiA402Drive::quickStop() {
    m_controlword = 0x0002;  // Quick stop
    return writeControlword(m_controlword);
}

bool CiA402Drive::resetFault() {
    // Rising edge on fault reset bit
    m_controlword &= ~0x0080;
    writeControlword(m_controlword);
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    m_controlword |= 0x0080;
    writeControlword(m_controlword);
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    m_controlword &= ~0x0080;
    return writeControlword(m_controlword);
}

bool CiA402Drive::isEnabled() {
    return getDriveState() == DriveState::OperationEnabled;
}

bool CiA402Drive::isFaulted() {
    DriveState state = getDriveState();
    return state == DriveState::Fault || state == DriveState::FaultReactionActive;
}

bool CiA402Drive::isTargetReached() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::TargetReached) != 0;
}

// ============================================================================
// DynaDrive Custom FSM (rsl_drive_sdk / ANYdrive)
// ============================================================================

const char* CiA402Drive::getDynaDriveStateName(DynaDriveState state) {
    switch (state) {
        case DynaDriveState::NA:            return "NA";
        case DynaDriveState::ColdStart:     return "ColdStart";
        case DynaDriveState::WarmStart:     return "WarmStart";
        case DynaDriveState::Configure:     return "Configure";
        case DynaDriveState::Calibrate:     return "Calibrate";
        case DynaDriveState::Standby:       return "Standby";
        case DynaDriveState::MotorOp:       return "MotorOp";
        case DynaDriveState::ControlOp:     return "ControlOp";
        case DynaDriveState::Error:         return "Error";
        case DynaDriveState::Fatal:         return "Fatal";
        case DynaDriveState::MotorPreOp:    return "MotorPreOp";
        case DynaDriveState::DeviceMissing: return "DeviceMissing";
        default:                            return "Unknown";
    }
}

CiA402Drive::DynaDriveState CiA402Drive::decodeDynaDriveState(uint32_t statusword) {
    uint8_t state_id = static_cast<uint8_t>(statusword & 0x0F);
    switch (state_id) {
        case 0:  return DynaDriveState::NA;
        case 1:  return DynaDriveState::ColdStart;
        case 2:  return DynaDriveState::WarmStart;
        case 3:  return DynaDriveState::Configure;
        case 4:  return DynaDriveState::Calibrate;
        case 5:  return DynaDriveState::Standby;
        case 6:  return DynaDriveState::MotorOp;
        case 7:  return DynaDriveState::ControlOp;
        case 8:  return DynaDriveState::Error;
        case 9:  return DynaDriveState::Fatal;
        case 10: return DynaDriveState::MotorPreOp;
        case 11: return DynaDriveState::DeviceMissing;
        default: return DynaDriveState::Unknown;
    }
}

bool CiA402Drive::readDynaDriveStatusword(uint32_t& statusword) {
    auto result = m_master->sdoManager(m_slave_index).readU32(
        static_cast<uint16_t>(CiA402::Register::Statusword), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (!result.has_value()) return false;
    statusword = result.value();
    return true;
}

bool CiA402Drive::sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options controlword) {
    uint16_t cw = static_cast<uint16_t>(controlword);
    TETHER_LOGI(TAG, "Slave %u: DynaDrive sending controlword ID 0x%02X", m_slave_index, cw);
    auto result = m_master->sdoManager(m_slave_index).writeU16(
        static_cast<uint16_t>(CiA402::Register::Controlword), 0, cw,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

bool CiA402Drive::enableDynaDrive(uint32_t timeout_ms) {
    TETHER_LOGI(TAG, "Slave %u: Enabling DynaDrive (target ControlOp)...", m_slave_index);

    const uint32_t poll_interval = 100;
    uint32_t elapsed = 0;

    auto wait_for_state = [&](DynaDriveState target) -> bool {
        while (elapsed < timeout_ms) {
            uint32_t sw = 0;
            if (readDynaDriveStatusword(sw)) {
                DynaDriveState current = decodeDynaDriveState(sw);
                if (current == target) {
                    TETHER_LOGI(TAG, "Slave %u: Reached %s", m_slave_index, getDynaDriveStateName(target));
                    return true;
                }
                if (current == DynaDriveState::Fatal) {
                    TETHER_LOGE(TAG, "Slave %u: Fatal state reached!", m_slave_index);
                    return false;
                }
            }
            Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
            elapsed += poll_interval;
        }
        TETHER_LOGE(TAG, "Slave %u: Timeout waiting for %s", m_slave_index, getDynaDriveStateName(target));
        return false;
    };

    // Read current state
    uint32_t statusword = 0;
    if (!readDynaDriveStatusword(statusword)) {
        TETHER_LOGE(TAG, "Slave %u: Failed to read DynaDrive statusword", m_slave_index);
        return false;
    }
    DynaDriveState state = decodeDynaDriveState(statusword);
    TETHER_LOGI(TAG, "Slave %u: Current DynaDrive state = %s (0x%08X)", m_slave_index, getDynaDriveStateName(state), statusword);

    // If in Error, clear to Standby
    if (state == DynaDriveState::Error) {
        TETHER_LOGI(TAG, "Slave %u: Clearing Error -> Standby", m_slave_index);
        if (!sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::ClearErrorsToStandby)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::Standby)) return false;
        state = DynaDriveState::Standby;
    }

    // Standby -> MotorPreOp (auto -> MotorOp)
    if (state == DynaDriveState::Standby) {
        TETHER_LOGI(TAG, "Slave %u: Standby -> MotorPreOp", m_slave_index);
        if (!sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::StandbyToMotorPreOp)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::MotorOp)) return false;
        state = DynaDriveState::MotorOp;
    }

    // MotorOp -> ControlOp
    if (state == DynaDriveState::MotorOp) {
        TETHER_LOGI(TAG, "Slave %u: MotorOp -> ControlOp", m_slave_index);
        if (!sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::MotorOpToControlOp)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::ControlOp)) return false;
        state = DynaDriveState::ControlOp;
    }

    if (state == DynaDriveState::ControlOp) {
        TETHER_LOGI(TAG, "Slave %u: DynaDrive enabled (ControlOp)", m_slave_index);
        return true;
    }

    TETHER_LOGE(TAG, "Slave %u: Unexpected DynaDrive state %s during enable",
             m_slave_index, getDynaDriveStateName(state));
    return false;
}

bool CiA402Drive::disableDynaDrive() {
    TETHER_LOGI(TAG, "Slave %u: Disabling DynaDrive (ControlOp -> Standby)", m_slave_index);
    uint32_t statusword = 0;
    if (readDynaDriveStatusword(statusword)) {
        DynaDriveState state = decodeDynaDriveState(statusword);
        if (state == DynaDriveState::ControlOp) {
            return sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::ControlOpToStandby);
        }
    }
    return true;  // Already not in ControlOp
}

bool CiA402Drive::isDynaDriveControlOp() {
    uint32_t statusword = 0;
    if (readDynaDriveStatusword(statusword)) {
        return decodeDynaDriveState(statusword) == DynaDriveState::ControlOp;
    }
    return false;
}

bool CiA402Drive::waitForDriveState(DriveState target, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;
    
    while (elapsed < timeout_ms) {
        DriveState current = getDriveState();
        if (current == target) {
            return true;
        }
        if (current == DriveState::Fault) {
            TETHER_LOGE(TAG, "Slave %u: Fault during state transition", m_slave_index);
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
        elapsed += poll_interval;
    }
    
    TETHER_LOGE(TAG, "Slave %u: Timeout waiting for state %s", 
             m_slave_index, getDriveStateName(target));
    return false;
}

bool CiA402Drive::writeControlword(uint16_t controlword) {
    auto result = m_master->sdoManager(m_slave_index).writeU16(
        static_cast<uint16_t>(CiA402::Register::Controlword), 0, controlword,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

// Public wrapper to allow immediate SDO write from other modules
bool CiA402Drive::sendControlwordSDO(uint16_t controlword) {
    return writeControlword(controlword);
}

bool CiA402Drive::readStatusword(uint16_t& statusword) {
    auto result = m_master->sdoManager(m_slave_index).readU16(
        static_cast<uint16_t>(CiA402::Register::Statusword), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (!result.has_value()) return false;
    statusword = result.value();
    return true;
}

// ============================================================================
// Operating Mode
// ============================================================================

bool CiA402Drive::setOperatingMode(int8_t mode) {
    uint8_t umode = static_cast<uint8_t>(mode);
    auto write_res = m_master->sdoManager(m_slave_index).writeU8(
        static_cast<uint16_t>(CiA402::Register::ModesOfOperation), 0, umode,
        {.timeout_ms = m_sdo_timeout_ms});
    if (write_res.has_value()) {
        TETHER_LOGI(TAG, "Slave %u: Operating mode set to %d via SDO", m_slave_index, mode);

        // Read back mode display to verify
        uint8_t mode_display = 0;
        Tether::Platform::Clock::instance().delayMilliseconds(50); // Give drive time to process
        auto read_res = m_master->sdoManager(m_slave_index).readU8(
            static_cast<uint16_t>(CiA402::Register::ModesOfOperationDisplay), 0,
            {.timeout_ms = m_sdo_timeout_ms});
        if (read_res.has_value()) {
            mode_display = read_res.value();
            TETHER_LOGI(TAG, "Slave %u: Mode Display readback via SDO = %d (expected %d)",
                     m_slave_index, (int8_t)mode_display, mode);
        } else {
            TETHER_LOGW(TAG, "Slave %u: Failed to read Mode Display via SDO", m_slave_index);
        }
        return true;
    }

    // Diagnostic: SDO write failed. Try to read error registers (0x1001, 0x1003) to get more info
    TETHER_LOGE(TAG, "Slave %u: Failed to set operating mode via SDO (index 0x6060)", m_slave_index);

    auto err_reg_res = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(0x1001), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (err_reg_res.has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Error register (0x1001) = 0x%02X", m_slave_index, err_reg_res.value());
    } else {
        TETHER_LOGW(TAG, "Slave %u: Unable to read Error Register (0x1001)", m_slave_index);
    }

    // Manufacturer error log (0x1003) - subindex 0 = number of errors
    auto mfr_count_res = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(0x1003), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (mfr_count_res.has_value()) {
        uint8_t mfr_err_count = mfr_count_res.value();
        TETHER_LOGW(TAG, "Slave %u: Manufacturer Error count (0x1003) = %u", m_slave_index, (unsigned)mfr_err_count);
        for (uint8_t i = 1; i <= mfr_err_count && i < 16; ++i) {
            auto err_res = m_master->sdoManager(m_slave_index).readU32(
                static_cast<uint16_t>(0x1003), i,
                {.timeout_ms = m_sdo_timeout_ms});
            if (err_res.has_value()) {
                TETHER_LOGW(TAG, "Slave %u: Manufacturer Error[%u] = 0x%08" PRIX32, m_slave_index, (unsigned)i, err_res.value());
            }
        }
    } else {
        TETHER_LOGW(TAG, "Slave %u: Unable to read Manufacturer Error Log (0x1003)", m_slave_index);
    }

    // Additional diagnostic: submit an asynchronous SDO request for the same
    // ModesOfOperation object to capture full SDOResponse (abort code and any
    // returned/echoed data). This helps determine why the sync write failed.
    {
        SDO::SDORequest dreq = {};
        dreq.slave_index = m_slave_index;
        dreq.index = static_cast<uint16_t>(CiA402::Register::ModesOfOperation);
        dreq.subindex = 0;
        dreq.operation = SDO::SDOOperation::Download;
        dreq.data_size = 1;
        dreq.timeout_ms = m_sdo_timeout_ms;
        dreq.data[0] = umode;

#ifdef TETHER_DIAG_SDO_IO
        m_master->sdoManager(m_slave_index).setDiagEnabled(true);
#endif
        uint32_t req_id = m_master->sdoManager(m_slave_index).queueRequest(dreq);
        if (req_id == 0) {
            TETHER_LOGW(TAG, "Slave %u: Diagnostic SDO queue failed", m_slave_index);
#ifdef TETHER_DIAG_SDO_IO
            m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
        } else {
            SDO::SDOResponse resp = {};
            const uint32_t poll_ms = 100;
            uint32_t waited = 0;
            const uint32_t max_wait = (dreq.timeout_ms > 0) ? dreq.timeout_ms : SDO::kDefaultSDOTimeoutMs;
            while (waited < max_wait) {
                Tether::Platform::Clock::instance().delayMilliseconds(poll_ms);
                waited += poll_ms;
                if (m_master->sdoManager(m_slave_index).getResponse(req_id, resp)) {
                    TETHER_LOGI(TAG, "Slave %u: Diagnostic SDO response status=%u abort=0x%08" PRIX32 " size=%u",
                             m_slave_index, (unsigned)resp.status, (uint32_t)resp.abort_code, (unsigned)resp.data_size);
                    if (resp.data_size > 0) {
                        char hex[128] = {0};
                        const size_t n = std::min(resp.data_size, (size_t)40);
                        size_t hex_len = 0;
                        for (size_t i = 0; i < n && hex_len < sizeof(hex) - 1; ++i) {
                            int written = snprintf(hex + hex_len, sizeof(hex) - hex_len, "%02X", resp.data[i]);
                            if (written < 0 || written >= (int)(sizeof(hex) - hex_len)) break;
                            hex_len += written;
                            if (i + 1 < n && hex_len < sizeof(hex) - 1) {
                                written = snprintf(hex + hex_len, sizeof(hex) - hex_len, " ");
                                if (written < 0 || written >= (int)(sizeof(hex) - hex_len)) break;
                                hex_len += written;
                            }
                        }
                        TETHER_LOGI(TAG, "Slave %u: Diagnostic SDO data: %s", m_slave_index, hex);
                    }
#ifdef TETHER_DIAG_SDO_IO
                    m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
                    break;
                }
            }
            if (waited >= max_wait) {
                TETHER_LOGW(TAG, "Slave %u: Diagnostic SDO response timed out", m_slave_index);
#ifdef TETHER_DIAG_SDO_IO
                m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
            }
        }
    }

    return false;
}

int8_t CiA402Drive::getOperatingMode() {
    auto result = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(CiA402::Register::ModesOfOperationDisplay), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (result.has_value()) {
        return static_cast<int8_t>(result.value());
    }
    return 0;  // Unknown
}

// ============================================================================
// Homing
// ============================================================================

bool CiA402Drive::setHomingMethod(int8_t method) {
    uint8_t umethod = static_cast<uint8_t>(method);
    auto result = m_master->sdoManager(m_slave_index).writeU8(
        static_cast<uint16_t>(CiA402::Register::HomingMethod), 0, umethod,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

bool CiA402Drive::homeToCurrentPosition(int32_t home_offset) {
    TETHER_LOGI(TAG, "Slave %u: Homing to current position (offset=%ld)", 
             m_slave_index, (long)home_offset);
    
    // Set home offset
    if (home_offset != 0) {
        m_master->sdoManager(m_slave_index).writeI32(
            static_cast<uint16_t>(CiA402::Register::HomeOffset), 0, home_offset,
            {.timeout_ms = m_sdo_timeout_ms});
    }
    
    // Set homing method 35 (current position = home)
    if (!setHomingMethod(CiA402::HomingMethodValue::CurrentPosition)) {
        TETHER_LOGE(TAG, "Slave %u: Failed to set homing method", m_slave_index);
        return false;
    }
    
    // Switch to homing mode
    if (!setModeHM()) {
        TETHER_LOGE(TAG, "Slave %u: Failed to set homing mode", m_slave_index);
        return false;
    }
    
    // Execute homing
    return executeHoming(5000);  // 5 second timeout for current position homing
}

bool CiA402Drive::executeHoming(uint32_t timeout_ms) {
    // Start homing (set bit 4)
    m_controlword |= CiA402::ControlwordBits::HomingOperationStart;
    writeControlword(m_controlword);
    
    // Wait for homing complete
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 50;
    
    while (elapsed < timeout_ms) {
        if (isHomingComplete()) {
            TETHER_LOGI(TAG, "Slave %u: Homing complete", m_slave_index);
            m_controlword &= ~CiA402::ControlwordBits::HomingOperationStart;
            writeControlword(m_controlword);
            return true;
        }
        if (hasHomingError()) {
            TETHER_LOGE(TAG, "Slave %u: Homing error", m_slave_index);
            m_controlword &= ~CiA402::ControlwordBits::HomingOperationStart;
            writeControlword(m_controlword);
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
        elapsed += poll_interval;
    }
    
    TETHER_LOGE(TAG, "Slave %u: Homing timeout", m_slave_index);
    return false;
}

bool CiA402Drive::isHomingComplete() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::HomingAttained) != 0;
}

bool CiA402Drive::hasHomingError() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::HomingError) != 0;
}

} // namespace EtherCAT
